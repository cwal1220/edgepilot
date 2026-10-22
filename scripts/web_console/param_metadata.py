"""파라미터 탭의 구성: 그룹(탭)마다 파일, 그 파일을 읽는 프로세스(reader: 떠 있는지만 페이지에 알린다, 다시 읽기는 그
프로세스가 스스로 한다), 섹션 순서와, 항목마다 섹션·단위·범위.
숫자 항목의 min/max는 런타임의 클램프 표와 같아야 한다(check_web_console.py가 대조). 화면 글자(그룹 이름과 안내, 섹션
이름, 항목의 이름·설명·효과)는 언어마다 페이지의 문구 묶음 static/i18n/<언어>.json에 있다: group.<그룹>.label·note,
section.<섹션>, param.<그룹>.<키>.label·description·increase·decrease·learned_role, 이름을 붙인 켜고 끄기의
toggle.<이름>, unit.<단위>, caution.<주의>, 그리고 다른 항목 때문에 쓰이지 않거나 쓰임이 바뀔 때의 안내
unused.<이름>·role.<이름>(아래 when). 단위 없는 수(이득, 비율, 가중치)는 단위를 두지 않는다.
check_web_console.py가 두 언어 모두 빠짐없는지 본다. 차량마다 다른 값(MDPS 위조 속도 등)은 설명에 적지 않는다.
페이지가 기본값(params.defaults)을 옆에 보여 준다."""
from __future__ import annotations

from typing import Any, Dict, Tuple


Condition = Dict[str, Any]


def _with(meta: Dict[str, Any], **extra: Any) -> Dict[str, Any]:
    """비어 있지 않은 선택 필드만 붙인다."""
    meta.update({key: value for key, value in extra.items() if value})
    return meta


# 조건이 볼 수 있는 런타임 사실(group="runtime"). 서버가 GET /api/params의 runtime에 담는다(LearnerMonitor.
# runtime_facts). controlsd 상태가 없거나 낡으면 null이고, 그때는 어느 조건도 참이 아니다.
RUNTIME_FACTS = ("lateral_nn_loaded",)  # controlsd가 학습한 토크 모델 파일을 읽었다(LearnerState 플래그)


def when(key: str, value: Any, text: str = "", *, group: str = "", also: Condition | None = None) -> Condition:
    """다른 항목의 값에 따른 조건: 그 항목(group을 주지 않으면 같은 그룹, "runtime"이면 RUNTIME_FACTS)이 value와
    같을 때 참이다(숫자 항목은 편집기가 저장하는 값 그대로, 예: 0). also가 있으면 그 조건도 참이어야 한다. 항목의
    unused에 두면 참인 동안 카드를 흐리게 하고 unused.<text>를, role에 두면 카드는 그대로 두고 role.<text>를 보인다.
    여럿이면 앞의 것 하나만 보인다. 값은 그대로 고칠 수 있다(전환 중이나 대체로 쓰이는 값이 있다)."""
    return _with({"key": key, "value": value, "text": text}, group=group, also=also)


def number(section: str, unit: str, step: float, minimum: float, maximum: float, *, control: str = "", tab: str = "",
           learned: bool = False, caution: str = "", unused: Tuple[Condition, ...] = (),
           role: Tuple[Condition, ...] = ()) -> Dict[str, Any]:
    """숫자 항목. control="slider"면 −/+ 대신 슬라이더로 고친다. tab="vehicle"이면 조향 탭 대신 차량 특성 탭(아래
    VEHICLE_TAB)에 나온다. learned면 자동 학습 중 이 수동값이 하는 일(learned_role 문구)이 있고, caution은 언제나
    보이는 주의(caution.<이름> 문구)다. unit은 기호 그대로 보이고, 언어마다 다르게 쓰는 것만 unit.<단위> 문구가 있다.
    unused·role은 다른 항목 때문에 쓰이지 않거나 쓰임이 바뀌는 조건(when)이다."""
    return _with({"section": section, "unit": unit, "step": step, "min": minimum, "max": maximum},
                 control=control, tab=tab, learned=learned, caution=caution, unused=list(unused), role=list(role))


def toggle(section: str, *, tab: str = "", learned: bool = False, card: str = "", labels: Tuple[str, str] = (),
           unused: Tuple[Condition, ...] = ()) -> Dict[str, Any]:
    """켜고 끄는 항목. increase/decrease 문구가 켜면·끄면의 설명이다. tab·learned·unused는 number와 같다. labels가
    있으면 스위치 대신 두 칸 버튼 묶음(켜면, 끄면 순, toggle.<이름> 문구)으로 고친다: 두 쪽이 다 이름이 있는 고르기
    (자동 학습·수동, locationd·ESP12). card는 그 탭의 섹션 대신 전용 카드가 바꾸는 항목의 카드 이름이다(예:
    tab="vehicle", card="calibration" = 차량 특성 탭의 카메라 보정 카드, POST /api/calibration/source). 파라미터
    API로는 바꿀 수 없고(ParamStore.update), section은 파일 그룹의 섹션이다."""
    return _with({"section": section}, tab=tab, learned=learned, card=card, labels=list(labels), unused=list(unused))


PARAM_GROUPS: Dict[str, Dict[str, Any]] = {
    "steering": {
        "file": "steering.json", "reader": "controlsd",
        "sections": ["basics", "torque_controller", "path_mode", "lane_position", "driver", "lkas_fault"],
    },
    "adaptive_cruise": {
        "file": "adaptive_cruise.json", "reader": "controlsd",
        "sections": ["cruise", "gap", "speed_response", "resume", "vision", "buttons"],
    },
    "recording": {"file": "recording.json", "reader": "", "sections": ["recording"]},
    "display": {"file": "display.json", "reader": "", "sections": ["backlight", "sound", "camera_mount"]},
}

# 차량 특성 탭: steering.json 항목 중 차의 성질인 것(tab="vehicle"), 섹션 순서대로. 학습기 섹션의 값들은 학습기
# 하나가 함께 내므로 스위치도 하나다. 켜면(자동 학습) 학습값을, 학습이 값을 내기 전과 끄면(수동) 수동값을 제어에
# 쓴다(controlsd LateralController::live_params, plan_delay_s). 고정 제원은 학습하지 않는 차량 값이다.
VEHICLE_TAB: Dict[str, Any] = {
    "sections": ["vehicle_model", "torque_model", "steer_delay", "learner_inputs"],
    "learners": {
        "vehicle_model": {"learner": "paramsd", "switch": "use_live_vehicle_params"},
        "torque_model": {"learner": "torqued", "switch": "use_live_torque_params"},
        "steer_delay": {"learner": "lagd", "switch": "use_live_delay"},
    },
}

# 다른 항목 때문에 쓰이지 않는 조건(unused)과 쓰임이 바뀌는 조건(role).
# - 조향을 끄면(enabled) 조향이 동작하지 않아(BlockReason::ControllerDisabled) 제어에만 쓰는 값은 쓰이지 않는다. 차량
#   모델 값과 학습기 사전값은 실제 곡률(광류 보정의 직선 판정, 차선 필터)과 학습에 계속 쓰이므로 두지 않는다.
# - 학습한 토크 모델(nn_feedforward, controlsd가 모델 파일을 읽었을 때)은 선형 피드포워드의 편향과 편경사를 대신한다.
#   그 값들은 모델로 넘어가는 1초 안쪽, 3 m/s 아래, 모델이 값을 못 낼 때만 쓰인다. 마찰 보상과 그 deadzone은 곡선
#   (|목표 횡가속| 0.2 m/s²부터)에서 모델에 더해지고, latAccelFactor는 PID의 P·I를 토크로 바꾸는 데만 쓰인다
#   (TorqueController::update).
# - 크루즈 값은 크루즈를 끄면 세션이 없어 쓰이지 않고(AdaptiveCruiseController::update_session), 조향을 끄면 조향이
#   동작하지 않아 버튼을 보내지 않는다(controls_ready, pace_buttons).
STEERING_OFF = when("enabled", False, "steering_off")
NN_LOADED = when("lateral_nn_loaded", True, group="runtime")
NN_ON = when("nn_feedforward", True, "nn_feedforward", also=NN_LOADED)
LANELESS = when("laneless_mode", True, "laneless")
LEARNING = ("auto", "manual")  # 학습기 스위치: 켜면 자동 학습(학습값), 끄면 수동(수동값)
CRUISE_OFF = (when("enabled", False, "cruise_off"), when("enabled", False, "steering_off_cruise", group="steering"))

PARAM_METADATA: Dict[str, Dict[str, Dict[str, Any]]] = {
    "steering": {
        "enabled": toggle("basics"),
        "steering_pressed_threshold": number("driver", "MDPS raw", 10, 0, 500, unused=(STEERING_OFF,)),
        "torque_lat_accel_factor": number("torque_model", "m/s²", 0.01, 0.5, 5.0, tab="vehicle", learned=True,
                                          caution="torqued_restart",
                                          role=(when("nn_feedforward", True, "nn_feedforward_pid", also=NN_LOADED),)),
        "torque_kp": number("torque_controller", "", 0.05, 0, 10, unused=(STEERING_OFF,)),
        "torque_ki": number("torque_controller", "", 0.01, 0, 2, unused=(STEERING_OFF,)),
        "steering_angle_deadzone_deg": number("torque_controller", "°", 0.1, 0, 3, unused=(STEERING_OFF,),
                                              role=(when("nn_feedforward", True, "nn_feedforward_curves",
                                                         also=NN_LOADED),)),
        "torque_straight_kp_scale": number("torque_controller", "x", 0.05, 0.3, 1, unused=(STEERING_OFF,)),
        "nn_feedforward": toggle("torque_controller", unused=(
            STEERING_OFF, when("lateral_nn_loaded", False, "nn_not_loaded", group="runtime"))),
        "torque_friction": number("torque_model", "torque", 0.005, 0, 0.3, tab="vehicle", learned=True,
                                  caution="torqued_restart",
                                  role=(when("nn_feedforward", True, "nn_feedforward_friction", also=NN_LOADED),)),
        "steer_ratio": number("vehicle_model", "", 0.1, 8, 25, tab="vehicle", learned=True),
        "tire_stiffness_factor": number("vehicle_model", "x", 0.05, 0.2, 2, tab="vehicle", learned=True),
        "steer_actuator_delay": number("steer_delay", "s", 0.01, 0.01, 1, tab="vehicle", learned=True),
        "avoid_lkas_fault_enabled": toggle("lkas_fault", unused=(STEERING_OFF,)),
        "avoid_lkas_fault_hold_angle_deg": number("lkas_fault", "°", 1, 0, 84, unused=(
            STEERING_OFF, when("avoid_lkas_fault_enabled", False, "lkas_fault_off"))),
        "live_bank_compensation": toggle("vehicle_model", tab="vehicle", learned=True, unused=(STEERING_OFF, NN_ON)),
        "use_live_vehicle_params": toggle("vehicle_model", tab="vehicle", labels=LEARNING),
        "use_live_torque_params": toggle("torque_model", tab="vehicle", labels=LEARNING),
        "use_live_delay": toggle("steer_delay", tab="vehicle", labels=LEARNING),
        "use_locationd_learner_inputs": toggle("learner_inputs", tab="vehicle", labels=("locationd", "esp12")),
        "torque_lat_accel_offset": number("torque_model", "m/s²", 0.01, -1.0, 1.0, tab="vehicle", learned=True,
                                          unused=(STEERING_OFF, NN_ON)),
        "angle_offset_deg": number("vehicle_model", "°", 0.1, -10, 10, tab="vehicle", learned=True),
        "lane_path_weight": number("lane_position", "", 0.1, 0.5, 10, unused=(STEERING_OFF, LANELESS)),
        "min_steer_speed_mps": number("basics", "m/s", 0.1, 0, 5, unused=(STEERING_OFF,)),
        "laneless_mode": toggle("path_mode"),
        "lane_change_min_speed_kph": number("driver", "km/h", 1, 0, 80, unused=(STEERING_OFF,)),
    },
    "adaptive_cruise": {
        "enabled": toggle("cruise", unused=CRUISE_OFF[1:]),
        "following_time_s": number("gap", "s", 0.1, 0.8, 4.0, unused=CRUISE_OFF),
        "standstill_gap_m": number("gap", "m", 0.5, 2.0, 20.0, unused=CRUISE_OFF),
        "gap_correction_gain": number("speed_response", "", 0.05, 0.05, 1.0, unused=CRUISE_OFF),
        "max_slowdown_correction_mps": number("speed_response", "m/s", 0.5, 0.5, 10.0, unused=CRUISE_OFF),
        "max_speedup_correction_mps": number("speed_response", "m/s", 0.5, 0.0, 5.0, unused=CRUISE_OFF),
        "deceleration_rate_kph_per_s": number("speed_response", "km/h/s", 0.1, 0.2, 5.0, unused=CRUISE_OFF),
        "lead_restore_delay_s": number("resume", "s", 0.5, 1.0, 10.0, unused=CRUISE_OFF),
        "command_interval_s": number("buttons", "s", 0.1, 0.5, 5.0, unused=CRUISE_OFF),
        "lead_hold_s": number("vision", "s", 0.1, 0.1, 2.0, unused=CRUISE_OFF),
    },
    "recording": {
        "enabled": toggle("recording"),
        "bitrate_bps": number("recording", "bps", 500000, 1000000, 10000000,
                              unused=(when("enabled", False, "recording_off"),)),
    },
    "display": {
        "enabled": toggle("backlight"),
        "brightness_percent": number("backlight", "%", 1, 1, 100, control="slider",
                                     unused=(when("enabled", False, "backlight_off"),)),
        "alert_volume_percent": number("sound", "%", 1, 0, 100, control="slider"),
        "camera_offset_m": number("camera_mount", "m", 0.01, -0.35, 0.35),
        "camera_height_m": number("camera_mount", "m", 0.01, 0.8, 2.0),
        "calibration_use_flow": toggle("camera_mount", tab="vehicle", card="calibration"),
    },
}
