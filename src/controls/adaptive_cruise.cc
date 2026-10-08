#include "controls/adaptive_cruise.h"

#include "car/can_frame.h"
#include "common/model_output.h"
#include "common/utils_math.h"
#include "common/utils_json.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kAcceleratorReleaseDelayS = 0.5;
/* cruise_active 가 이만큼 꺼져 있어야 세션을 버린다. SCC12와 CLU11 추정이
 * 같은 필드를 쓰므로 한 틱 깜빡임으로 천장을 잃지 않게 유예를 둔다. */
constexpr double kCruiseInactiveTeardownS = 0.3;
/* 운전자가 버튼을 뗀 뒤 차량이 정착할 때까지. 이 구간에는 자동 명령을 쉬고,
 * 끝나면 실측 속도로 천장과 추정을 다시 잡는다. 길게 누르기(추정치는 엣지
 * 하나만 세므로 실제 감속량을 모른다)와 스텝 크기 불일치를 한꺼번에 흡수한다. */
constexpr double kDriverSettleS = 2.0;
/* 클러스터 속도가 추정과 이만큼 이 시간 이상 어긋나면 추정을 버리고 실측에
 * 다시 앵커한다. 펄스 유실이나 차량 스텝이 2 km/h가 아닌 경우를 잡는다. */
constexpr float kMismatchKph = 5.0f;
constexpr double kMismatchHoldS = 5.0;
/* 명령이 듣지 않는다고 본 뒤 물러나 있는 시간. */
constexpr double kIneffectiveBackoffS = 30.0;
/* 클러스터 속도가 이 폭 안에서 이 시간 유지되면 정착한 것으로 본다. */
constexpr float kClusterSteadyKph = 0.5f;
constexpr double kClusterSteadyHoldS = 0.5;
/* 클러스터/휠 비의 학습 시정수와 허용 범위. 클램프 끝에 붙으면 신호가 깨진
 * 것으로 보고 학습되지 않은 것으로 취급한다. */
constexpr double kDisplayScaleTauS = 5.0;
constexpr float kDisplayScaleMin = 0.97f;
constexpr float kDisplayScaleMax = 1.25f;
constexpr float kDisplayScaleLearnMinKph = 30.0f;

float minimum_speed_kph(bool speed_unit_mph) {
  return speed_unit_mph ? kMinimumCruiseSpeedMph * kMphToKph : kMinimumCruiseSpeedKph;
}

float display_step_kph(bool speed_unit_mph) {
  return kFixedCruiseStep * (speed_unit_mph ? kMphToKph : 1.0f);
}

bool valid_set_speed(float speed_kph) {
  return std::isfinite(speed_kph) && speed_kph > 0.0f && speed_kph < 300.0f;
}

bool valid_vision_lead(const AdaptiveCruiseInput &input) {
  return input.vision_lead_valid &&
         std::isfinite(input.vision_lead_probability) &&
         input.vision_lead_probability >= kLeadProbabilityThreshold &&
         std::isfinite(input.vision_lead_distance_m) &&
         std::isfinite(input.vision_lead_relative_speed_mps) &&
         input.vision_lead_distance_m >= 1.0f &&
         input.vision_lead_distance_m <= 150.0f &&
         std::fabs(input.vision_lead_relative_speed_mps) <= 40.0f;
}

}  // namespace

namespace {

constexpr JsonBoolField<AdaptiveCruiseConfig> kAdaptiveBools[] = {
    {"enabled", &AdaptiveCruiseConfig::enabled},
};
constexpr JsonFloatField<AdaptiveCruiseConfig> kAdaptiveFloats[] = {
    {"standstill_gap_m", 2.0f, 20.0f, &AdaptiveCruiseConfig::standstill_gap_m},
    {"following_time_s", 0.8f, 4.0f, &AdaptiveCruiseConfig::following_time_s},
    {"gap_correction_gain", 0.05f, 1.0f, &AdaptiveCruiseConfig::gap_correction_gain},
    {"max_slowdown_correction_mps", 0.5f, 10.0f, &AdaptiveCruiseConfig::max_slowdown_correction_mps},
    {"max_speedup_correction_mps", 0.0f, 5.0f, &AdaptiveCruiseConfig::max_speedup_correction_mps},
    {"deceleration_rate_kph_per_s", 0.5f, 5.0f, &AdaptiveCruiseConfig::deceleration_rate_kph_per_s},
    {"lead_hold_s", 0.1f, 2.0f, &AdaptiveCruiseConfig::lead_hold_s},
    {"lead_restore_delay_s", 1.0f, 10.0f, &AdaptiveCruiseConfig::lead_restore_delay_s},
    {"command_interval_s", 0.5f, 5.0f, &AdaptiveCruiseConfig::command_interval_s},
};

}  // namespace

bool load_adaptive_cruise_params_json(
    const std::string &path, AdaptiveCruiseConfig *config,
    std::string *error) {
  if (!config) return false;
  return load_json_param_file(path, [config](const std::string &text) {
    parse_json_fields(text, kAdaptiveBools, config);
    parse_json_fields(text, kAdaptiveFloats, config);
  }, error);
}

AdaptiveCruiseController::AdaptiveCruiseController(
    AdaptiveCruiseConfig config)
    : config_(config) {}

void AdaptiveCruiseController::update_config(
    const AdaptiveCruiseConfig &config) {
  config_ = config;
}

void AdaptiveCruiseController::Session::anchor(float speed_kph, double now_s) {
  maximum_speed_kph = speed_kph;
  commanded_speed_kph = speed_kph;
  last_command_s = now_s;
  reanchor_pending = false;
  mismatch_since_s = -1.0;
  ineffective_until_s = -1.0;
}

void AdaptiveCruiseController::DisplayScale::update(
    const AdaptiveCruiseInput &input, double dt_s) {
  if (dt_s <= 0.0) return;
  if (!std::isfinite(input.cluster_speed_kph) ||
      !std::isfinite(input.ego_speed_kph) ||
      input.cluster_speed_kph < kDisplayScaleLearnMinKph ||
      input.ego_speed_kph < kDisplayScaleLearnMinKph) {
    return;
  }
  const float raw = input.cluster_speed_kph / input.ego_speed_kph;
  /* 범위를 벗어나면 둘 중 하나가 깨진 것이다. 클램프해서 쓰면 "학습됐다"와
   * 구분이 안 되므로 학습을 무효로 돌리고 명령을 막는다. */
  if (raw < kDisplayScaleMin || raw > kDisplayScaleMax) {
    valid = false;
    return;
  }
  if (!valid) {
    scale = raw;
    valid = true;
    return;
  }
  const float alpha = static_cast<float>(dt_s / (kDisplayScaleTauS + dt_s));
  scale += alpha * (raw - scale);
}

void AdaptiveCruiseController::LeadFilter::update(
    const AdaptiveCruiseInput &input, const AdaptiveCruiseConfig &config) {
  if (!input.vision_lead_updated || !valid_vision_lead(input)) return;

  const bool reacquired = last_valid_s < 0.0 ||
                          input.now_s - last_valid_s > config.lead_hold_s;
  if (reacquired) {
    distance_m = input.vision_lead_distance_m;
    relative_speed_mps = input.vision_lead_relative_speed_mps;
  } else {
    const float distance_alpha =
        input.vision_lead_distance_m < distance_m ? 0.45f : 0.2f;
    distance_m += distance_alpha * (input.vision_lead_distance_m - distance_m);
    relative_speed_mps +=
        0.3f * (input.vision_lead_relative_speed_mps - relative_speed_mps);
  }
  last_valid_s = input.now_s;
}

/* 클러스터 속도가 kClusterSteadyKph 폭 안에서 kClusterSteadyHoldS 넘게 머물면 정착이다(고정형
 * 크루즈가 설정 속도에 닿았다). */
bool AdaptiveCruiseController::ClusterSteady::update(bool valid, float speed_kph,
                                                     double now_s) {
  if (valid) {
    if (ref_s < 0.0 || std::fabs(speed_kph - ref_kph) > kClusterSteadyKph) {
      ref_kph = speed_kph;
      ref_s = now_s;
    }
  } else {
    ref_s = -1.0;
  }
  return valid && ref_s >= 0.0 && now_s - ref_s >= kClusterSteadyHoldS;
}

struct AdaptiveCruiseController::Tick {
  bool accelerator_override = false;
  bool driver_main_pressed = false;
  float minimum_kph = 0.0f;
  float step_kph = 0.0f;
  bool speed_valid = false;
  bool cluster_valid = false;
  bool cluster_steady = false;
  bool driver_adjusting = false;
};

/* 세션 종료. cruise_active 는 SCC12와 CLU11 추정이 같은 필드를 쓰므로 한 틱
 * 깜빡임이 가능하다. 유예를 두되, 실제로 꺼지면 반드시 버린다 — 예전에는
 * 버리지 않아 다음 engage 때 이전 주행의 천장이 남았다. 선행차 필터도 잊어서
 * 다음 세션은 새로 본 거리에서 시작한다. */
void AdaptiveCruiseController::teardown_session(const AdaptiveCruiseInput &input,
                                                const Tick &tick) {
  if (input.cruise_active) {
    cruise_inactive_since_s_ = -1.0;
  } else if (cruise_inactive_since_s_ < 0.0) {
    cruise_inactive_since_s_ = input.now_s;
  }
  const bool cruise_off_settled =
      !input.cruise_active && cruise_inactive_since_s_ >= 0.0 &&
      input.now_s - cruise_inactive_since_s_ >= kCruiseInactiveTeardownS;
  if (!input.enabled || tick.driver_main_pressed || cruise_off_settled) {
    session_ = Session{};
    lead_.last_valid_s = -1.0;
  }
}

/* 운전자가 버튼을 만지는 동안과 뗀 뒤 정착까지는 자동 명령을 쉰다. 세션이 이미 있을 때만
 * 유예한다. 세션을 여는 첫 SET 자체를 막으면 안 된다. */
void AdaptiveCruiseController::track_driver_adjustment(const AdaptiveCruiseInput &input,
                                                       Tick *tick) {
  if (input.driver_button != 0 && session_.valid) {
    session_.driver_adjust_until_s = input.now_s + kDriverSettleS;
    session_.reanchor_pending = true;
  }
  tick->driver_adjusting = session_.driver_adjust_until_s >= 0.0 &&
                           input.now_s < session_.driver_adjust_until_s;
}

/* 세션 시작은 차량이 유효한 설정 속도를 보고한 뒤로 미룬다. */
void AdaptiveCruiseController::begin_session_if_ready(const AdaptiveCruiseInput &input,
                                                      const Tick &tick) {
  if (input.enabled && input.cruise_active && !tick.driver_main_pressed &&
      !session_.valid && valid_set_speed(input.driver_set_speed_kph)) {
    session_ = Session{};
    session_.valid = true;
    session_.anchor(std::max(tick.minimum_kph, input.driver_set_speed_kph), input.now_s);
  }
}

/* 운전자 조작이 끝나면 추측을 버리고 실측 속도에 다시 앵커한다. 고정형
 * 크루즈는 정착하면 클러스터 속도가 곧 설정 속도다. 길게 누르기(추정치는
 * 버튼 엣지 하나만 세므로 실제 감속량을 모른다), 2 km/h가 아닌 스텝,
 * 펄스 유실이 모두 여기서 흡수된다. */
void AdaptiveCruiseController::reanchor_after_driver(const AdaptiveCruiseInput &input,
                                                     const Tick &tick) {
  if (session_.valid && input.cruise_active && session_.reanchor_pending &&
      !tick.driver_adjusting && input.driver_button == 0 && tick.cluster_steady) {
    session_.anchor(std::max(tick.minimum_kph, input.cluster_speed_kph), input.now_s);
  }
}

/* 추측 적산이 실제와 오래 어긋나면(펄스 유실, 스텝 크기 불일치) 실측으로
 * 되돌린다. 천장은 건드리지 않는다 — 운전자 의도가 아니기 때문이다. */
void AdaptiveCruiseController::resolve_command_mismatch(const AdaptiveCruiseInput &input,
                                                        const Tick &tick) {
  Session &s = session_;
  if (s.valid && input.cruise_active && tick.cluster_valid &&
      !tick.driver_adjusting && s.command_frames_remaining == 0 &&
      s.last_command_s >= 0.0 &&
      input.now_s - s.last_command_s >= kMismatchHoldS) {
    if (std::fabs(input.cluster_speed_kph - s.commanded_speed_kph) > kMismatchKph) {
      if (s.mismatch_since_s < 0.0) s.mismatch_since_s = input.now_s;
      if (input.now_s - s.mismatch_since_s >= kMismatchHoldS) {
        /* 우리 추정이 실제보다 낮다는 건 SET-이 먹지 않았다는 뜻이다.
         * 실측으로 되돌리기만 하면 여유가 되살아나 같은 명령을 무한 반복한다. */
        if (input.cluster_speed_kph > s.commanded_speed_kph)
          s.ineffective_until_s = input.now_s + kIneffectiveBackoffS;
        s.commanded_speed_kph = clamp_float(input.cluster_speed_kph,
                                            tick.minimum_kph, s.maximum_speed_kph);
        s.mismatch_since_s = -1.0;
      }
    } else {
      s.mismatch_since_s = -1.0;
    }
  }
}

/* 선행차가 있으면 차간 보정을 더한 추종 속도, 없으면 (복원 지연 뒤) 천장. */
float AdaptiveCruiseController::target_speed(const AdaptiveCruiseInput &input,
                                             const Tick &tick, bool *lead_valid) const {
  const Session &s = session_;
  *lead_valid = s.valid && tick.speed_valid &&
                lead_.last_valid_s >= 0.0 &&
                input.now_s >= lead_.last_valid_s &&
                input.now_s - lead_.last_valid_s <= config_.lead_hold_s;
  float target_speed_kph = s.commanded_speed_kph;
  if (s.valid && *lead_valid && display_scale_.valid) {
    const float ego_speed_mps = std::max(0.0f, input.ego_speed_kph / 3.6f);
    const float lead_speed_mps =
        std::max(0.0f, ego_speed_mps + lead_.relative_speed_mps);
    const float desired_gap_m =
        config_.standstill_gap_m + config_.following_time_s * ego_speed_mps;
    const float slowdown_response_s =
        tick.step_kph / std::max(0.1f, config_.deceleration_rate_kph_per_s);
    const float prediction_horizon_s =
        std::min(2.0f, config_.command_interval_s + 0.5f * slowdown_response_s);
    const float predicted_lead_distance_m = std::max(
        1.0f, lead_.distance_m +
                  std::min(0.0f, lead_.relative_speed_mps) *
                      prediction_horizon_s);
    const float gap_correction_mps = clamp_float(
        (predicted_lead_distance_m - desired_gap_m) *
            config_.gap_correction_gain,
        -config_.max_slowdown_correction_mps,
        config_.max_speedup_correction_mps);
    /* 차간·상대속도는 휠 속도 기준, 설정 속도는 클러스터 표시 기준이다. K7
     * 실측으로 클러스터가 6.6% 높아, 환산 없이 비교하면 차간이 맞아도 SET-이
     * 계속 나가 설정 속도가 바닥까지 내려갔다. */
    target_speed_kph = clamp_float(
        (lead_speed_mps + gap_correction_mps) * 3.6f * display_scale_.scale,
        tick.minimum_kph, s.maximum_speed_kph);
  } else if (s.valid &&
             (lead_.last_valid_s < 0.0 ||
              input.now_s - lead_.last_valid_s >=
                  config_.lead_restore_delay_s)) {
    target_speed_kph = s.maximum_speed_kph;
  }
  return target_speed_kph;
}

/* 명령 게이트와 버튼 펄스. 척도를 모르면 두 척도를 비교할 수 없으므로 아예
 * 명령하지 않는다. 돌려주는 값이 이 틱에 보낼 버튼. */
int AdaptiveCruiseController::pace_buttons(const AdaptiveCruiseInput &input, const Tick &tick,
                                           float target_speed_kph, bool active) {
  Session &s = session_;
  const bool accelerator_released =
      !tick.accelerator_override &&
      (last_accelerator_override_s_ < 0.0 ||
       input.now_s - last_accelerator_override_s_ >=
           kAcceleratorReleaseDelayS);
  const bool command_allowed = active && tick.speed_valid && display_scale_.valid &&
                               input.controls_ready && !input.brake_pressed &&
                               accelerator_released && !tick.driver_adjusting &&
                               (s.ineffective_until_s < 0.0 ||
                                input.now_s >= s.ineffective_until_s) &&
                               input.driver_button == 0 && !tick.driver_main_pressed;
  if (!command_allowed) {
    s.command_button = 0;
    s.command_frames_remaining = 0;
  }

  int output_button = 0;
  if (command_allowed && s.command_frames_remaining > 0) {
    output_button = s.command_button;
    --s.command_frames_remaining;
  } else if (command_allowed) {
    /* 데드밴드는 한 스텝. 0.75스텝이면 한 번 누른 결과가 다시 밴드 밖으로
     * 나가 SET-/RES+가 번갈아 나온다(65 km/h 추종 180초: 87회 -> 37회). */
    const float command_deadband_kph = tick.step_kph;
    const bool wants_set =
        target_speed_kph <= s.commanded_speed_kph - command_deadband_kph &&
        s.commanded_speed_kph > tick.minimum_kph + 0.1f;
    const bool wants_resume =
        target_speed_kph >= s.commanded_speed_kph + command_deadband_kph &&
        s.commanded_speed_kph < s.maximum_speed_kph - 0.1f;
    /* SET-과 RES+에 같은 간격. 감속 응답을 기다리는 긴 간격은 연속 SET- 사이
     * 에만 쓴다. */
    const double slowdown_interval_s = std::max(
        static_cast<double>(config_.command_interval_s),
        static_cast<double>(tick.step_kph) /
            std::max(0.1, static_cast<double>(config_.deceleration_rate_kph_per_s)));
    const double button_interval_s = s.last_auto_command_was_set
        ? slowdown_interval_s
        : static_cast<double>(config_.command_interval_s);
    const bool interval_ready =
        s.last_command_s < 0.0 ||
        input.now_s - s.last_command_s >= button_interval_s;

    if (wants_set && interval_ready) {
      s.command_button = kCruiseButtonSet;
      s.last_auto_command_was_set = true;
      s.commanded_speed_kph =
          std::max(tick.minimum_kph, s.commanded_speed_kph - tick.step_kph);
    } else if (wants_resume && interval_ready) {
      s.command_button = kCruiseButtonResume;
      s.last_auto_command_was_set = false;
      s.commanded_speed_kph =
          std::min(s.maximum_speed_kph, s.commanded_speed_kph + tick.step_kph);
    } else {
      s.command_button = 0;
    }
    if (s.command_button != 0) {
      s.last_command_s = input.now_s;
      s.command_frames_remaining = config_.button_pulse_frames - 1;
      output_button = s.command_button;
      s.mismatch_since_s = -1.0;
    }
  }
  return output_button;
}

AdaptiveCruiseOutput AdaptiveCruiseController::update(
    const AdaptiveCruiseInput &input) {
  const double dt_s = last_update_s_ < 0.0 || input.now_s < last_update_s_
      ? 0.0 : std::min(0.1, input.now_s - last_update_s_);
  last_update_s_ = input.now_s;
  display_scale_.update(input, dt_s);

  Tick tick;
  tick.accelerator_override = input.gas_pressed || input.driver_accelerator_override;
  if (tick.accelerator_override) last_accelerator_override_s_ = input.now_s;
  tick.driver_main_pressed =
      input.driver_main_button != 0 && previous_driver_main_button_ == 0;
  tick.minimum_kph = minimum_speed_kph(input.speed_unit_mph);
  tick.step_kph = display_step_kph(input.speed_unit_mph);
  tick.speed_valid = std::isfinite(input.ego_speed_kph);
  tick.cluster_valid = std::isfinite(input.cluster_speed_kph) &&
                       input.cluster_speed_kph > 0.0f;
  tick.cluster_steady =
      cluster_steady_.update(tick.cluster_valid, input.cluster_speed_kph, input.now_s);

  teardown_session(input, tick);
  track_driver_adjustment(input, &tick);
  begin_session_if_ready(input, tick);
  reanchor_after_driver(input, tick);
  resolve_command_mismatch(input, tick);
  lead_.update(input, config_);
  bool lead_valid = false;
  const float target_speed_kph = target_speed(input, tick, &lead_valid);
  const bool active = input.enabled && session_.valid && input.cruise_active;
  const int output_button = pace_buttons(input, tick, target_speed_kph, active);

  previous_driver_main_button_ = input.driver_main_button;

  AdaptiveCruiseOutput output;
  output.session_valid = session_.valid;
  output.active = active;
  output.lead_valid = lead_valid;
  output.maximum_speed_kph = session_.maximum_speed_kph;
  output.commanded_speed_kph = session_.commanded_speed_kph;
  output.target_speed_kph = target_speed_kph;
  output.display_scale = display_scale_.scale;
  output.command_button = output_button;
  return output;
}
