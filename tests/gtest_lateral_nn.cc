// 학습한 토크 모델(docs/lateral-nn.md): 레포의 모델 파일이 학습 쪽 시험 벡터대로 읽히는지, 잘못된 파일을
// 거부하는지, 망이 무너지면 NaN을 내는지, 운용 입력(목표 궤적과 이력)이 창 정의대로 채워지는지, 차량 모델 역함수,
// 그리고 토크 제어기·횡제어기가 스위치·조향 상태·속도에 따라 모델 피드포워드를 쓰고 PID와 같은 곡률 한계를 지키는지.
#include "car/vehicle_can.h"
#include "common/json_value.h"
#include "common/lateral_nn_path.h"
#include "common/utils_file.h"
#include "controls/control_params.h"
#include "controls/lateral_controller.h"
#include "controls/lateral_nn.h"
#include "controls/lateral_torque.h"

#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

namespace {

// 토크 제어기 입력: 앞쪽 다섯과 편경사·학습값. 핸들 조작·안전 한계·요레이트는 기본값(없음)이다.
TorqueController::Input torque_in(bool active, float speed_mps, float curvature, float angle_deg, float delay_s,
                                  float bank = 0.0f, const LiveLateralParams &live = LiveLateralParams{}) {
    TorqueController::Input in;
    in.active = active;
    in.speed_mps = speed_mps;
    in.desired_curvature = curvature;
    in.steering_angle_deg = angle_deg;
    in.lat_delay_s = delay_s;
    in.road_bank_lat_accel = bank;
    in.live = live;
    return in;
}

// 학습한 토크 모델의 피드포워드와 섞는 법
TorqueController::NnFeedforward nn_ff(float torque, float weight = 1.0f, bool curve_friction = true,
                                      float curve_gain = 0.0f) {
    TorqueController::NnFeedforward nn;
    nn.torque = torque;
    nn.weight = weight;
    nn.curve_friction = curve_friction;
    nn.curve_gain = curve_gain;
    return nn;
}

constexpr const char *kModelPath = "models/lateral_nn/k7_hev_v3.json";  // 시험은 레포 루트에서 돈다

// 입력 넷(v, la, jerk, roll)에 선형 층 둘인 작은 모델. 첫 층이 v를 ±scale배로 두 갈래로 나누고 둘째 층이 더한다.
std::string tiny_model(float scale) {
    const std::string s = std::to_string(scale);
    return std::string(R"({"format": "edgepilot-lateral-nn", "version": 1, "name": "tiny",
  "features": ["v", "la", "jerk", "roll"],
  "spec": {"lag_s": 0.3, "la_past": [], "la_future": [], "roll_past": [], "roll_future": [], "extra": [],
           "extra_windows": []},
  "input_mean": [0, 0, 0, 0], "input_std": [1, 1, 1, 1], "output_clip": [-1.0, 1.0],
  "nets": [{"layers": [{"weight": [[)") +
           s + ", 0, 0, 0], [-" + s + R"(, 0, 0, 0]], "bias": [0, 0], "activation": "linear"},
                        {"weight": [[1, 1]], "bias": [0.25], "activation": "linear"}]}],
  "test_vectors": [{"input": [0, 0, 0, 0], "output": 0.25}]})";
}

std::shared_ptr<LateralNnModel> repository_model() {
    auto model = std::make_shared<LateralNnModel>();
    std::string error;
    EXPECT_TRUE(model->load(kModelPath, &error)) << error;
    return model;
}

// 레포 모델에서 friction_free만 거짓으로 바꾼 것: 보낸 토크를 그대로 배운 모델 자리(예전 k7_hev_v1, v2)
std::shared_ptr<LateralNnModel> model_with_friction_in_target() {
    std::string text = read_text_file(kModelPath);
    const std::string from = "\"friction_free\": true";
    const size_t at = text.find(from);
    EXPECT_NE(at, std::string::npos);
    if (at != std::string::npos) text.replace(at, from.size(), "\"friction_free\": false");
    auto model = std::make_shared<LateralNnModel>();
    std::string error;
    EXPECT_TRUE(model->load_text(text, &error)) << error;
    return model;
}

int feature_index(const LateralNnModel &model, const std::string &name) {
    const std::vector<std::string> names = model.spec().names();
    for (size_t i = 0; i < names.size(); ++i) {
        if (names[i] == name) return static_cast<int>(i);
    }
    ADD_FAILURE() << "no feature " << name;
    return 0;
}

VehicleCanState ready_vehicle(double t = 1.0, float speed_kph = 60.0f) {
    VehicleCanState vehicle;
    vehicle.has_lkas11_seed = vehicle.has_clu11_seed = vehicle.has_mdps12_seed = true;
    stamp_received_messages(&vehicle, t);
    vehicle.wheel_speed_fl_kph = vehicle.wheel_speed_fr_kph = speed_kph;
    vehicle.wheel_speed_rl_kph = vehicle.wheel_speed_rr_kph = speed_kph;
    vehicle.cluster_speed_raw = speed_kph * 1.05f;
    vehicle.gear = 5;
    return vehicle;
}

LateralPath usable_path() {
    LateralPath path;
    path.usable_for_steering = true;
    path.point_count = 30;
    path.reach_m = 60.0f;
    return path;
}

LateralTarget curve_target(float curvature, float speed_kph = 60.0f) {
    LateralTarget target;
    target.valid = target.mpc_solution_valid = true;
    for (int i = 0; i < kLateralControlN; ++i) {
        target.curvatures[i] = curvature;
        target.psis[i] = curvature * (speed_kph / 3.6f) * (10.0f * i * i / (32.0f * 32.0f));
    }
    return target;
}

// 강제 결합한 횡제어기에 같은 계획을 ticks번 넣고 마지막 결과를 본다
LateralControlOutput drive(const std::shared_ptr<LateralNnModel> &model, const LateralTarget &target, float speed_kph,
                           int ticks, bool switch_on = true) {
    LateralControllerConfig config;
    config.force_engaged = true;
    config.steering_params.nn_feedforward = switch_on;
    LateralController controller(config);
    if (model) controller.set_lateral_nn(model);
    LateralControlOutput result;
    for (int i = 0; i < ticks; ++i) {
        const double t = 1.0 + 0.01 * i;
        result = controller.update(usable_path(), target, ready_vehicle(t, speed_kph), t, i);
    }
    return result;
}

TEST(LateralNnModel, RepositoryModelLoadsWithItsTestVectors) {
    // 불러오기가 파일의 시험 벡터를 이 계산으로 다시 내 보고 받는다
    const auto model = repository_model();
    ASSERT_TRUE(model->loaded());
    EXPECT_EQ(model->input_count(), 33U);
}

// 라이브러리 모델: 런타임과 웹 콘솔이 고를 수 있는 이름(lateral_nn_path.h의 규칙)의 .json
bool library_model(const std::filesystem::path &path) {
    return path.extension() == ".json" && valid_lateral_nn_name(path.stem().string());
}

TEST(LateralNnModel, EveryLibraryModelLoads) {
    // 웹 콘솔에서 고를 수 있는 모델(models/lateral_nn/<차종>_v<버전>.json)은 모두 제 시험 벡터대로 읽힌다
    int count = 0;
    for (const auto &entry : std::filesystem::directory_iterator("models/lateral_nn")) {
        if (!library_model(entry.path())) continue;
        ++count;
        LateralNnModel model;
        std::string error;
        EXPECT_TRUE(model.load(entry.path().string(), &error)) << entry.path() << ": " << error;
    }
    EXPECT_GT(count, 0);
}

TEST(LateralNnModel, NamesMatchTheTrainingSide) {
    const auto model = repository_model();
    const std::vector<std::string> names = model->spec().names();
    ASSERT_EQ(names.size(), 33U);
    EXPECT_EQ(names[0], "v");
    EXPECT_EQ(names[4], "la-0.5");
    // 0.45 s는 파이썬과 같게 "+0.5"(float로 만들면 "+0.4"가 되어 불러오기가 실패한다)
    EXPECT_EQ(names[12], "la+0.5");
    EXPECT_EQ(names.back(), "angle+1.50");
}

TEST(LateralNnModel, RejectsBadFiles) {
    const std::string good = read_text_file(kModelPath);
    ASSERT_FALSE(good.empty());
    const auto broken = [&](const std::string &from, const std::string &to) {
        std::string text = good;
        const size_t at = text.find(from);
        EXPECT_NE(at, std::string::npos) << from;
        text.replace(at, from.size(), to);
        LateralNnModel model;
        std::string error;
        EXPECT_FALSE(model.load_text(text, &error)) << from << " -> " << to;
        EXPECT_FALSE(model.loaded());
        return error;
    };
    EXPECT_NE(broken("\"edgepilot-lateral-nn\"", "\"other\"").find("not an"), std::string::npos);
    EXPECT_NE(broken("\"la+0.5\"", "\"la+0.4\"").find("feature"), std::string::npos);
    EXPECT_NE(broken("\"tanh\"", "\"relu\"").find("activation"), std::string::npos);
    EXPECT_NE(broken("\"extra\": [", "\"extra\": [\"driver_tq\", ").find("extra"), std::string::npos);
    EXPECT_NE(broken("\"version\": 1", "\"version\": 1.9").find("not an"), std::string::npos);
    // float로 무한대가 되는 값, [-1, 1] 밖의 출력 범위
    EXPECT_NE(broken("\"input_mean\": [", "\"input_mean\": [1e39, ").find("input_mean"), std::string::npos);
    EXPECT_NE(broken("\"output_clip\": [\n  -1.0", "\"output_clip\": [\n  -2.0").find("output_clip"),
              std::string::npos);
    // 이력(1 s) 밖을 읽는 창
    EXPECT_NE(broken("\"la_past\": [\n   -0.5", "\"la_past\": [\n   -1.5").find("outside"), std::string::npos);
    // 파일의 시험 벡터와 다른 값을 내는 파일(첫 출력을 바꾼다)
    const size_t at = good.find("\"output\": ");
    ASSERT_NE(at, std::string::npos);
    const size_t end = good.find_first_of(",}", at);
    EXPECT_NE(broken(good.substr(at, end - at), "\"output\": 0.987").find("test vector"), std::string::npos);
    LateralNnModel model;
    std::string error;
    EXPECT_FALSE(model.load("models/no_such_model.json", &error));
    EXPECT_NE(error.find("cannot open"), std::string::npos);
}

TEST(LateralNnModel, NonFiniteInputGivesNan) {
    const auto model = repository_model();
    std::vector<float> inputs(model->input_count(), 0.0f);
    inputs[0] = 20.0f;
    EXPECT_TRUE(std::isfinite(model->evaluate(inputs.data())));
    inputs[5] = NAN;
    EXPECT_TRUE(std::isnan(model->evaluate(inputs.data())));
}

TEST(LateralNnModel, CollapsedNetGivesNanNotTheClipBound) {
    // 첫 층이 v=2를 +무한대와 -무한대로 보내고 둘째 층이 더하면 NaN이다. 출력 범위로 자르면 -1(최대 토크)이 된다.
    LateralNnModel model;
    std::string error;
    ASSERT_TRUE(model.load_text(tiny_model(3e38f), &error)) << error;
    const float zero[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    EXPECT_FLOAT_EQ(model.evaluate(zero), 0.25f);
    const float big[4] = {2.0f, 0.0f, 0.0f, 0.0f};
    EXPECT_TRUE(std::isnan(model.evaluate(big)));
}

TEST(LateralNnInputs, ConstantPlanFillsEveryWindow) {
    const auto model = repository_model();
    LateralNnInputs inputs;
    for (int i = 0; i < 100; ++i) inputs.push(1.2f, -5.0f);
    const float out =
        inputs.evaluate(*model, 20.0f, 0.01f, 0.4f, [](float) { return 1.2f; }, [](float) { return -5.0f; });
    EXPECT_TRUE(std::isfinite(out));
    const std::vector<float> &x = inputs.last_inputs();
    const std::vector<std::string> names = model->spec().names();
    ASSERT_EQ(x.size(), names.size());
    for (size_t i = 0; i < names.size(); ++i) {
        const std::string &n = names[i];
        if (n == "v") {
            EXPECT_FLOAT_EQ(x[i], 20.0f);
        } else if (n == "jerk") {
            EXPECT_NEAR(x[i], 0.0f, 1e-6f);
        } else if (n.rfind("la", 0) == 0) {
            EXPECT_FLOAT_EQ(x[i], 1.2f) << n;
        } else if (n.rfind("roll", 0) == 0) {
            EXPECT_FLOAT_EQ(x[i], 0.01f) << n;
        } else if (n.rfind("angle", 0) == 0) {
            EXPECT_FLOAT_EQ(x[i], -5.0f) << n;
        } else if (n == "long") {
            EXPECT_FLOAT_EQ(x[i], 0.4f);
        }
    }
    // 실제 모델로도 같은 입력의 결과다
    EXPECT_FLOAT_EQ(out, model->evaluate(x.data()));
}

TEST(LateralNnInputs, PastOffsetsReadTheHistory) {
    const auto model = repository_model();
    LateralNnInputs inputs;
    // 이력: 지금 값이 틱마다 0.01씩 오른다(마지막 = 0.99). 미래는 1.0 + δ.
    for (int i = 0; i < 100; ++i) inputs.push(0.01f * static_cast<float>(i), 0.0f);
    inputs.evaluate(*model, 20.0f, 0.0f, 0.0f, [](float d) { return 1.0f + d; }, [](float) { return 0.0f; });
    const std::vector<float> &x = inputs.last_inputs();
    // 응답 시각 0.3 s 기준: la-0.5 → 0.2 s 전(20틱 전), la-0.4 → 0.1 s 전, la-0.3 → 지금(계획), la+0.1 → 0.4 s 뒤
    EXPECT_NEAR(x[feature_index(*model, "la-0.5")], 0.79f, 1e-5f);
    EXPECT_NEAR(x[feature_index(*model, "la-0.4")], 0.89f, 1e-5f);
    EXPECT_NEAR(x[feature_index(*model, "la-0.3")], 1.0f, 1e-5f);
    EXPECT_NEAR(x[feature_index(*model, "la+0.1")], 1.4f, 1e-5f);
    EXPECT_NEAR(x[feature_index(*model, "la")], 1.3f, 1e-5f);
    EXPECT_NEAR(x[feature_index(*model, "jerk")], 1.0f, 1e-4f);
}

// 모델 파일 하나의 input_tests를 운용 입력으로 다시 만들어 본다(MatchTheTrainingCode)
void check_input_tests(const std::string &path) {
    SCOPED_TRACE(path);
    auto model = std::make_shared<LateralNnModel>();
    std::string error;
    ASSERT_TRUE(model->load(path, &error)) << error;
    JsonValue root;
    ASSERT_TRUE(JsonValue::parse(read_text_file(path), &root, &error)) << error;
    const JsonValue *tests = root.find("input_tests");
    ASSERT_TRUE(tests && tests->is_array() && !tests->array().empty());
    for (const JsonValue &test : tests->array()) {
        const auto numbers = [&](const char *key) {
            std::vector<float> out;
            for (const JsonValue &v : test.find(key)->array()) out.push_back(static_cast<float>(v.number()));
            return out;
        };
        const std::vector<float> la = numbers("la"), angle = numbers("angle"), expected = numbers("inputs");
        const float dt = static_cast<float>(test.find("dt")->number());
        const int now = static_cast<int>(test.find("now")->number());
        const auto at = [&](const std::vector<float> &values, float delta_s) {
            const float index = static_cast<float>(now) + delta_s / dt;
            const int i = std::clamp(static_cast<int>(std::floor(index)), 0, static_cast<int>(values.size()) - 2);
            const float f = index - static_cast<float>(i);
            return values[static_cast<size_t>(i)] * (1.0f - f) + values[static_cast<size_t>(i) + 1] * f;
        };
        LateralNnInputs inputs;
        for (int k = 99; k >= 0; --k)
            inputs.push(la[static_cast<size_t>(now - k)], angle[static_cast<size_t>(now - k)]);
        inputs.evaluate(
            *model, static_cast<float>(test.find("v")->number()), static_cast<float>(test.find("roll")->number()),
            static_cast<float>(test.find("long")->number()), [&](float d) { return at(la, d); },
            [&](float d) { return at(angle, d); });
        const std::vector<float> got = inputs.last_inputs();
        const std::vector<std::string> names = model->spec().names();
        ASSERT_EQ(got.size(), expected.size());
        for (size_t i = 0; i < got.size(); ++i) EXPECT_NEAR(got[i], expected[i], 1e-4f) << names[i];
    }
}

TEST(LateralNnInputs, MatchTheTrainingCode) {
    // 모델 파일의 input_tests: 학습 코드(nnlc dataset.rows)가 합성 궤적에서 만든 입력. 운용 입력(지난 값은 이력, 앞의
    // 값은 계획 궤적)으로 같은 입력이 나와야 한다. 이름은 그대로인데 뜻이 바뀐 학습 변경(저크 창, 시각 기준 등)을 잡는다.
    // 라이브러리의 모델마다 본다.
    for (const auto &entry : std::filesystem::directory_iterator("models/lateral_nn")) {
        if (library_model(entry.path())) check_input_tests(entry.path().string());
    }
}

TEST(TorqueController, AngleAtCurvatureInvertsCurvatureAtAngle) {
    TorqueController controller;
    const SteeringParams params;
    LiveLateralParams live;
    for (bool use_vehicle : {false, true}) {
        live.use_vehicle = use_vehicle;
        live.steer_ratio = 15.9f;
        live.stiffness_factor = 1.1f;
        live.angle_offset_deg = -1.4f;
        live.roll_rad = 0.02f;
        for (float speed : {5.0f, 15.0f, 30.0f}) {
            for (float curvature : {-0.02f, -0.002f, 0.0f, 0.002f, 0.02f}) {
                const float angle = controller.angle_map(speed, params, live)(curvature);
                EXPECT_NEAR(controller.curvature_at_angle(speed, angle, params, live), curvature, 1e-6f)
                    << "use_vehicle " << use_vehicle << " v " << speed << " c " << curvature;
            }
        }
    }
}

TEST(TorqueController, NnFeedforwardReplacesTheLinearFeedforward) {
    // 직선(목표 횡가속 0.16 m/s²)에 같은 입력을 50틱 넣고 마지막 상태를 본다. 마찰 계수만 다른 두 설정으로 모델 경로에
    // 마찰이 끼지 않는지 본다.
    const auto run = [](float nn, float friction) {
        SteeringParams params;
        params.torque_friction = friction;
        TorqueController controller;
        for (int i = 0; i < 50; ++i)
            controller.update(torque_in(true, 20.0f, 0.0004f, -3.0f, 0.3f, 0.1f), params, nn_ff(nn));
        return std::make_tuple(controller.feedforward(), controller.nn_feedforward_used(),
                               controller.normalized_output());
    };
    const SteeringParams defaults;
    const auto [ff_nn, used_nn, out_nn] = run(0.2f, 0.2f);
    EXPECT_TRUE(used_nn);
    EXPECT_FLOAT_EQ(ff_nn, 0.2f * defaults.torque_lat_accel_factor) << "편향·편경사·마찰 없이 모델 토크만";
    const auto [ff_nn0, used_nn0, out_nn0] = run(0.2f, 0.0f);
    EXPECT_NEAR(out_nn0, out_nn, 1e-6f) << "모델 경로는 마찰 계수와 무관하다";
    const auto [ff_lin, used_lin, out_lin] = run(NAN, 0.2f);
    EXPECT_FALSE(used_lin);
    EXPECT_NE(ff_lin, ff_nn);
    const auto [ff_lin0, used_lin0, out_lin0] = run(NAN, 0.0f);
    EXPECT_NE(out_lin0, out_lin) << "선형 경로는 마찰을 쓴다";
}

TEST(TorqueController, NnFeedforwardKeepsTheFrictionInCurves) {
    // 모델은 추종 오차를 모르므로 곡선에서는 선형과 같은 마찰 보상을 더한다: |목표 횡가속| 0.2 m/s²부터 0.6 m/s²까지
    // 늘려 그 뒤로는 전부. 같은 입력의 선형 경로에서 마찰 계수가 바꾸는 몫과 비교한다.
    const auto ff = [](float curvature, float friction, float nn) {
        SteeringParams params;
        params.torque_friction = friction;
        TorqueController controller;
        for (int i = 0; i < 50; ++i)
            controller.update(torque_in(true, 20.0f, curvature, -3.0f, 0.3f, 0.1f), params, nn_ff(nn));
        return controller.feedforward();
    };
    const float laf = SteeringParams{}.torque_lat_accel_factor;
    const auto friction_share = [&](float curvature, float nn) {
        return ff(curvature, 0.2f, nn) - ff(curvature, 0.0f, nn);
    };
    // 20 m/s에서 곡률 0.002 = 0.8 m/s²(전부), 0.001 = 0.4 m/s²(절반), 0.0004 = 0.16 m/s²(없음)
    EXPECT_FLOAT_EQ(ff(0.002f, 0.0f, 0.2f), 0.2f * laf) << "마찰 계수 0이면 모델만";
    EXPECT_NE(friction_share(0.002f, 0.2f), 0.0f);
    EXPECT_NEAR(friction_share(0.002f, 0.2f), friction_share(0.002f, NAN), 1e-6f);
    EXPECT_NEAR(friction_share(0.001f, 0.2f), 0.5f * friction_share(0.001f, NAN), 1e-6f);
    EXPECT_NEAR(friction_share(0.0004f, 0.2f), 0.0f, 1e-6f);
}

TEST(TorqueController, NnWeightBlendsTheTwoFeedforwards) {
    const auto ff = [](float nn, float weight) {
        TorqueController controller;
        controller.update(torque_in(true, 20.0f, 0.002f, -3.0f, 0.3f, 0.1f), SteeringParams{}, nn_ff(nn, weight));
        return std::make_pair(controller.feedforward(), controller.nn_feedforward_used());
    };
    const float linear = ff(NAN, 1.0f).first;
    const float model = ff(0.2f, 1.0f).first;
    EXPECT_NEAR(ff(0.2f, 0.5f).first, 0.5f * (linear + model), 1e-6f);
    EXPECT_FLOAT_EQ(ff(0.2f, 0.0f).first, linear);
    EXPECT_FALSE(ff(0.2f, 0.0f).second) << "무게 0은 선형만";
    EXPECT_TRUE(ff(0.2f, 0.01f).second);
}

TEST(LateralController, UsesTheModelOnlyWhenSwitchedOnAndSteering) {
    const auto model = repository_model();
    const LateralControlOutput on = drive(model, curve_target(0.003f), 60.0f, 50);
    ASSERT_TRUE(on.lat_active);
    EXPECT_TRUE(on.nn_feedforward_used);
    EXPECT_TRUE(std::isfinite(on.feedforward));
    EXPECT_NE(on.desired_torque, 0);
    EXPECT_FALSE(drive(model, curve_target(0.003f), 60.0f, 50, false).nn_feedforward_used) << "스위치를 끄면 선형";
    EXPECT_FALSE(drive(nullptr, curve_target(0.003f), 60.0f, 50).nn_feedforward_used) << "모델이 없으면 선형";
}

TEST(LateralController, ModelInputsKeepTheHoldAngleLimit) {
    // 손을 뗀 저속 회전: PID 요청은 80도가 내는 곡률에서 멈춘다. 모델 입력도 그 곡률에서 멈춰야 계획이 더 감아도
    // 피드포워드가 커지지 않는다(그대로 두면 80도 너머를 위한 토크로 85도 고장 각도 쪽으로 민다).
    const auto model = repository_model();
    const float speed_kph = 14.4f;  // 4 m/s: 횡가속 한계(0.19 1/m)보다 80도(약 0.03 1/m)가 좁다
    TorqueController vehicle_model;
    const float cap = std::fabs(vehicle_model.curvature_at_angle(speed_kph / 3.6f, 80.0f, SteeringParams{}));
    const LateralControlOutput at_cap = drive(model, curve_target(cap, speed_kph), speed_kph, 150);
    const LateralControlOutput beyond = drive(model, curve_target(3.0f * cap, speed_kph), speed_kph, 150);
    const LateralControlOutput inside = drive(model, curve_target(0.5f * cap, speed_kph), speed_kph, 150);
    ASSERT_TRUE(at_cap.nn_feedforward_used && beyond.nn_feedforward_used);
    EXPECT_FLOAT_EQ(beyond.feedforward, at_cap.feedforward);
    EXPECT_NE(inside.feedforward, at_cap.feedforward);
}

TEST(LateralController, SlowSpeedFallsBackToLinear) {
    // 학습 데이터가 없는 3 m/s 아래(7.2 km/h = 2 m/s)에서는 1초에 걸쳐 선형 피드포워드로 돌아간다
    const auto model = repository_model();
    const LateralControlOutput slow = drive(model, curve_target(0.003f, 7.2f), 7.2f, 150);
    ASSERT_TRUE(slow.lat_active);
    EXPECT_FALSE(slow.nn_feedforward_used);
}

TEST(LateralController, NanPlanUsesTheLinearFeedforward) {
    const auto model = repository_model();
    LateralTarget target = curve_target(0.003f);
    target.curvatures[3] = NAN;  // 모델이 읽는 0.3 s 근처
    const LateralControlOutput result = drive(model, target, 60.0f, 50);
    ASSERT_TRUE(result.lat_active);
    EXPECT_FALSE(result.nn_feedforward_used);
    EXPECT_TRUE(std::isfinite(result.feedforward));
}

TEST(LateralController, SwitchingBlendsOverOneSecond) {
    const auto model = repository_model();
    LateralControllerConfig config;
    config.force_engaged = true;
    config.steering_params.nn_feedforward = true;
    LateralController controller(config);
    controller.set_lateral_nn(model);
    int i = 0;
    const auto tick = [&] {
        const double t = 1.0 + 0.01 * i;
        const auto r = controller.update(usable_path(), curve_target(0.003f), ready_vehicle(t), t, i);
        ++i;
        return r;
    };
    for (int k = 0; k < 150; ++k) tick();  // 무게 1
    SteeringParams off = config.steering_params;
    off.nn_feedforward = false;
    controller.update_params(off);
    // 끈 뒤 1초 동안은 아직 섞여 있고(줄어드는 무게), 그 뒤 선형만
    for (int k = 0; k < 90; ++k) EXPECT_TRUE(tick().nn_feedforward_used) << k;
    for (int k = 0; k < 20; ++k) tick();
    EXPECT_FALSE(tick().nn_feedforward_used);
}

// 학습 데이터에서 뺀 차선 변경·회전(desire)과 깜빡이 동안에는 1초에 걸쳐 선형 피드포워드로 돌아가고, 끝나면 다시
// 1초에 걸쳐 모델로 간다(조향은 이어진다)
TEST(LateralController, LaneChangesAndBlinkersUseTheLinearFeedforward) {
    const auto model = repository_model();
    LateralControllerConfig config;
    config.force_engaged = true;
    config.steering_params.nn_feedforward = true;
    LateralController controller(config);
    controller.set_lateral_nn(model);
    int i = 0;
    const auto tick = [&](bool blinker, Desire desire) {
        const double t = 1.0 + 0.01 * i;
        VehicleCanState vehicle = ready_vehicle(t);
        vehicle.left_blinker = blinker;
        LateralTarget target = curve_target(0.003f);
        target.desire = desire;
        return controller.update(usable_path(), target, vehicle, t, i++);
    };
    for (int k = 0; k < 150; ++k) ASSERT_TRUE(tick(false, Desire::None).nn_feedforward_used || k < 2);
    LateralControlOutput r;
    for (int k = 0; k < 150; ++k) r = tick(true, Desire::None);
    EXPECT_TRUE(r.lat_active);
    EXPECT_FALSE(r.nn_feedforward_used) << "깜빡이";
    for (int k = 0; k < 150; ++k) r = tick(false, Desire::None);
    EXPECT_TRUE(r.nn_feedforward_used) << "끝나면 다시 모델";
    for (int k = 0; k < 150; ++k) r = tick(false, Desire::LaneChangeLeft);
    EXPECT_FALSE(r.nn_feedforward_used) << "차선 변경 desire";
}

// 모델의 roll 입력은 편경사 추정 / g다(학습 열 roll = extract_lateral_dataset의 bank / g). paramsd 롤은 부호가
// 반대라, 컨트롤러가 그것을 넣으면 학습과 어긋난다.
TEST(LateralController, ModelRollInputIsTheBankOverG) {
    const auto model = repository_model();
    LateralControllerConfig config;
    config.force_engaged = true;
    config.steering_params.nn_feedforward = true;
    LateralController controller(config);
    controller.set_lateral_nn(model);
    for (int i = 0; i < 300; ++i) {  // 곧은 길에서 횡가속 0.6 m/s²: 편경사 추정이 쌓인다
        const double t = 1.0 + 0.01 * i;
        VehicleCanState vehicle = ready_vehicle(t);
        vehicle.lat_accel_valid = vehicle.yaw_rate_valid = true;
        vehicle.lat_accel_mps2 = 0.6f;
        controller.update(usable_path(), curve_target(0.0f), vehicle, t, i);
    }
    const float bank = controller.road_bank_lat_accel();
    ASSERT_GT(std::fabs(bank), 0.1f);
    const std::vector<float> inputs = controller.lateral_nn_inputs();
    const std::vector<std::string> names = model->spec().names();
    ASSERT_EQ(inputs.size(), names.size());
    int rolls = 0;
    for (size_t i = 0; i < names.size(); ++i) {
        if (names[i].rfind("roll", 0) != 0) continue;
        EXPECT_FLOAT_EQ(inputs[i], bank / 9.81f) << names[i];
        ++rolls;
    }
    EXPECT_GT(rolls, 0);
}

TEST(LateralController, SwappingModelsKeepsTheBlend) {
    // 주행 중 고르기를 바꾸면 새 모델이 바로 이어받는다: 선형으로 떨어졌다가 1초에 걸쳐 오르지 않고, 처음부터 새
    // 모델로 달린 제어기와 같은 피드포워드를 낸다(이력은 모델과 상관없는 목표 궤적)
    auto other = std::make_shared<LateralNnModel>();
    std::string error;
    ASSERT_TRUE(other->load_text(tiny_model(2.0f), &error)) << error;
    LateralControllerConfig config;
    config.force_engaged = true;
    config.steering_params.nn_feedforward = true;
    LateralController swapped(config), reference(config);
    swapped.set_lateral_nn(repository_model());
    reference.set_lateral_nn(other);
    const auto tick = [](LateralController &controller, int i) {
        const double t = 1.0 + 0.01 * i;
        return controller.update(usable_path(), curve_target(0.003f), ready_vehicle(t), t, i);
    };
    int i = 0;
    for (; i < 150; ++i) {
        tick(swapped, i);
        tick(reference, i);
    }
    swapped.set_lateral_nn(other);
    const LateralControlOutput after = tick(swapped, i);
    const LateralControlOutput expected = tick(reference, i);
    ASSERT_TRUE(after.nn_feedforward_used);
    EXPECT_NEAR(after.feedforward, expected.feedforward, 1e-6f);
    swapped.set_lateral_nn(nullptr);
    EXPECT_FALSE(tick(swapped, ++i).nn_feedforward_used) << "모델을 빼면 바로 선형";
}

TEST(LateralNnModel, ReadsWhetherTheTargetIsFrictionFree) {
    // friction_free는 v3에서 더한 항목이라 없으면 거짓이다(그 전 모델은 보낸 토크를 그대로 배웠다)
    EXPECT_TRUE(repository_model()->spec().friction_free);
    EXPECT_FALSE(model_with_friction_in_target()->spec().friction_free);
    LateralNnModel tiny;
    std::string error;
    ASSERT_TRUE(tiny.load_text(tiny_model(1.0f), &error)) << error;
    EXPECT_FALSE(tiny.spec().friction_free) << "항목이 없으면 거짓";
    std::string text = read_text_file(kModelPath);
    const size_t at = text.find("\"friction_free\": true");
    ASSERT_NE(at, std::string::npos);
    text.replace(at, 21, "\"friction_free\": 1");
    LateralNnModel model;
    EXPECT_FALSE(model.load_text(text, &error));
    EXPECT_NE(error.find("friction_free"), std::string::npos) << error;
}

TEST(LateralNnModel, ReadsTheCurveGain) {
    // curve_gain은 2026-10-18에 더한 항목이라 없으면 0이다. 0..1 밖이나 숫자가 아니면 파일을 받지 않는다
    LateralNnModel tiny;
    std::string error;
    ASSERT_TRUE(tiny.load_text(tiny_model(1.0f), &error)) << error;
    EXPECT_FLOAT_EQ(tiny.curve_gain(), 0.0f) << "항목이 없으면 0";
    const auto with_gain = [](const std::string &value) {
        std::string text = tiny_model(1.0f);
        text.insert(1, "\"curve_gain\": " + value + ", ");
        return text;
    };
    LateralNnModel model;
    ASSERT_TRUE(model.load_text(with_gain("0.3"), &error)) << error;
    EXPECT_FLOAT_EQ(model.curve_gain(), 0.3f);
    for (const char *bad : {"-0.1", "1.5", "\"0.3\"", "true"}) {
        EXPECT_FALSE(model.load_text(with_gain(bad), &error)) << bad;
        EXPECT_NE(error.find("curve_gain"), std::string::npos) << error;
    }
    EXPECT_FLOAT_EQ(repository_model()->curve_gain(), 0.3f) << "k7_hev_v3의 폐루프 보정값";
}

TEST(TorqueController, CurveGainRaisesTheModelWhileTheCurveTightens) {
    // 마찰을 빼고 모델 피드포워드만 보면 이득은 (1 + gain · 커짐 · 곡선 램프 · 속도 램프)배다: 계획의 |목표 횡가속|이
    // 커지는 빠르기(curve_rise) 0→0.3 m/s³, |목표 횡가속| 0.2→0.6 m/s², 30→50 km/h.
    const auto ratio = [](float speed_mps, float curvature, float rise) {
        const auto ff = [&](float gain) {
            SteeringParams params;
            params.torque_friction = 0.0f;
            TorqueController controller;
            TorqueController::NnFeedforward nn = nn_ff(0.2f, 1.0f, true, gain);
            nn.curve_rise = rise;
            controller.update(torque_in(true, speed_mps, curvature, -3.0f, 0.3f), params, nn);
            return controller.feedforward();
        };
        return ff(0.3f) / ff(0.0f);
    };
    // 20 m/s에서 곡률 0.002는 0.8 m/s²
    EXPECT_NEAR(ratio(20.0f, 0.002f, 0.6f), 1.3f, 1e-3f) << "72 km/h, 커짐 0.6 m/s³: 전부";
    EXPECT_NEAR(ratio(20.0f, 0.002f, 0.15f), 1.15f, 1e-3f) << "커짐 0.15 m/s³: 절반";
    EXPECT_NEAR(ratio(20.0f, 0.002f, 0.0f), 1.0f, 1e-6f) << "곡선 유지: 이득 없음";
    EXPECT_NEAR(ratio(20.0f, 0.002f, -0.5f), 1.0f, 1e-6f) << "곡선 탈출: 이득 없음";
    EXPECT_NEAR(ratio(20.0f, -0.002f, 0.6f), 1.3f, 1e-3f) << "왼쪽 곡선도 |목표 횡가속|이 커지면 전부";
    EXPECT_NEAR(ratio(20.0f, 0.001f, 0.6f), 1.15f, 1e-3f) << "0.4 m/s²: 곡선 램프 절반";
    EXPECT_NEAR(ratio(40.0f / 3.6f, 0.006f, 0.6f), 1.15f, 1e-3f) << "40 km/h: 속도 램프 절반";
    EXPECT_NEAR(ratio(25.0f / 3.6f, 0.02f, 0.6f), 1.0f, 1e-6f) << "30 km/h 아래";
}

// 곡선 이득을 거는 때는 계획의 모양으로 정한다: 계획이 조여들면(진입) 켜고, 일정한 곡선은 계획이 바뀔 때마다 곡률이
// 조금씩 달라도(요청 잡음) 켜지 않는다. 같은 모델의 curve_gain만 0.3과 0으로 바꾼 두 컨트롤러의 피드포워드 비로 본다.
TEST(LateralController, CurveGainFollowsThePlanShape) {
    const auto model_with_gain = [](const char *gain) {
        std::string text = read_text_file(kModelPath);
        const std::string from = "\"curve_gain\": 0.3";
        const size_t at = text.find(from);
        EXPECT_NE(at, std::string::npos);
        if (at != std::string::npos) text.replace(at, from.size(), std::string("\"curve_gain\": ") + gain);
        auto model = std::make_shared<LateralNnModel>();
        std::string error;
        EXPECT_TRUE(model->load_text(text, &error)) << error;
        return model;
    };
    const auto gained = model_with_gain("0.3"), plain = model_with_gain("0");
    // 60 km/h. plan(tick)은 그 틱의 계획이고, 5틱(50 ms)마다 새 계획(촬영 시각)이 온다.
    const auto ratio = [&](const std::function<LateralTarget(int)> &plan) {
        const auto ff = [&](const std::shared_ptr<LateralNnModel> &model) {
            LateralControllerConfig config;
            config.force_engaged = true;
            config.steering_params.nn_feedforward = true;
            config.steering_params.torque_friction = 0.0f;
            config.steering_params.use_live_torque_params = false;
            LateralController controller(config);
            controller.set_lateral_nn(model);
            LateralControlOutput result;
            for (int i = 0; i < 200; ++i) {
                const double t = 1.0 + 0.01 * i;
                LateralTarget target = plan(i);
                target.capture_timestamp_ns = static_cast<uint64_t>((1.0 + 0.05 * (i / 5) - 0.05) * 1e9);
                result = controller.update(usable_path(), target, ready_vehicle(t), t, i);
            }
            EXPECT_TRUE(result.nn_feedforward_used);
            return result.feedforward;
        };
        return ff(gained) / ff(plain);
    };
    // 진입: 요청(psis, 0.0025 = 0.69 m/s²) 뒤의 계획 곡률이 초당 0.0016씩 조여든다(0.44 m/s³)
    const auto entering = [](int) {
        LateralTarget target = curve_target(0.0025f);
        for (int i = 0; i < kLateralControlN; ++i) target.curvatures[i] = 0.0025f + 0.0016f * model_t_idx(i);
        return target;
    };
    EXPECT_NEAR(ratio(entering), 1.3f, 1e-3f) << "조여드는 계획: 전부";
    // 유지: 일정한 곡률 0.003(0.83 m/s²)이 계획마다 ±4 % 흔들린다
    const auto holding = [](int tick) { return curve_target((tick / 5) % 2 ? 0.00312f : 0.00288f); };
    EXPECT_NEAR(ratio(holding), 1.0f, 1e-6f) << "흔들리는 일정한 곡선: 이득 없음";
}

TEST(TorqueController, CurveFrictionOnlyForFrictionFreeModels) {
    // 곡선(20 m/s, 곡률 0.002 = 0.8 m/s²)에서 마찰 계수가 바꾸는 피드포워드 몫: 마찰을 빼고 배운 모델에만 더한다
    const auto share = [](bool curve_friction) {
        const auto ff = [&](float friction) {
            SteeringParams params;
            params.torque_friction = friction;
            TorqueController controller;
            for (int i = 0; i < 50; ++i)
                controller.update(torque_in(true, 20.0f, 0.002f, -3.0f, 0.3f, 0.1f), params,
                                  nn_ff(0.2f, 1.0f, curve_friction));
            return controller.feedforward();
        };
        return ff(0.2f) - ff(0.0f);
    };
    EXPECT_NE(share(true), 0.0f);
    EXPECT_FLOAT_EQ(share(false), 0.0f);
}

TEST(LateralController, AddsCurveFrictionByTheModelsTarget) {
    // 마찰 몫을 빼고 배운 모델(v3)에는 곡선 마찰을 더하고, 보낸 토크를 그대로 배운 모델에는 더하지 않는다
    const auto ff = [](const std::shared_ptr<LateralNnModel> &model, float friction) {
        LateralControllerConfig config;
        config.force_engaged = true;
        config.steering_params.nn_feedforward = true;
        config.steering_params.torque_friction = friction;
        config.steering_params.use_live_torque_params = false;
        LateralController controller(config);
        controller.set_lateral_nn(model);
        LateralControlOutput result;
        for (int i = 0; i < 150; ++i) {
            const double t = 1.0 + 0.01 * i;
            result = controller.update(usable_path(), curve_target(0.003f), ready_vehicle(t), t, i);
        }
        EXPECT_TRUE(result.nn_feedforward_used);
        return result.feedforward;
    };
    const auto friction_free = repository_model();
    const auto sent_torque = model_with_friction_in_target();
    EXPECT_GT(std::fabs(ff(friction_free, 0.2f) - ff(friction_free, 0.0f)), 1e-3f);
    EXPECT_FLOAT_EQ(ff(sent_torque, 0.2f), ff(sent_torque, 0.0f));
}

}  // namespace
