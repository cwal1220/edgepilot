#pragma once

// openpilot master supercombo 원시 출력의 배치(model_output_layout, 블록 오프셋마다 static_assert)와 해석(plan,
// 차선선, 도로 경계, 앞차, pose, meta 가스 예측), 궤적 격자(T_IDXS, X_IDXS).

#include <array>
#include <vector>

// 모델 pose 출력(카메라 이동·회전과 표준편차). 온라인 보정과 locationd가 입력으로 쓴다.
struct PoseObservation {
    float trans[3];
    float rot[3];
    float trans_std[3];
    float rot_std[3];
    // road_transform 앞 3개(도로면 기준 카메라 위치, [2]가 높이)와 표준편차. upstream cameraOdometry.roadTransformTrans
    float road_trans[3];
    float road_trans_std[3];
};

constexpr int kTrajectorySize = 33;
constexpr int kLeadMhpSelection = 3;
constexpr int kLeadTrajLen = 6;
constexpr int kDesireLen = 8;
constexpr float kModelHeight = 1.22f;

// 런타임 안의 원시 출력 버퍼 배치. axmodel은 헤드를 이름 붙인 출력으로 따로 내고(split_outputs.py),
// SupercomboModel이 그것들을 이 배치로 모은 뒤 파서가 오프셋으로 읽는다. 배치는 예전 출시 모델(65a08adc)의
// output_slices 순서다. Smoked Cheddar(e170f08)의 출력 자체는 2580개이고 차선이 앞에 오는 다른 순서이며 action
// 4개가 붙지만, 헤드를 이름으로 받으므로 이 버퍼 순서와는 상관없다.
// 0.9.4와 달리 plan은 단일 가설(평균+표준편차), lead는 가설 없이 시간 오프셋
// (0/2/4 s) 3개가 각자 궤적을 갖고, 특징(hidden_state)은 512다.
constexpr int kModelOutputFloats = 2576;
constexpr int kModelFeatureLen = 512;

namespace model_output_layout {
constexpr int kMetaOffset = 0;
constexpr int kMetaSize = 55;
constexpr int kDesirePredOffset = 55;
constexpr int kDesirePredSize = 32;
constexpr int kPoseOffset = 87;  // 이동 3, 회전 3, 로그 표준편차 6
constexpr int kPoseSize = 12;
constexpr int kWideFromDeviceEulerOffset = 99;
constexpr int kWideFromDeviceEulerSize = 6;
constexpr int kRoadTransformOffset = 105;  // 이동 3, 회전 3, 로그 표준편차 6
constexpr int kRoadTransformSize = 12;
constexpr int kLaneOffset = 117;  // 평균 4x33x2, 이어서 log std
constexpr int kLaneLineSize = 4 * kTrajectorySize * 2;
constexpr int kLaneProbOffset = 645;  // 차선당 로짓 2개, 두 번째가 존재 확률
constexpr int kLaneProbSize = 8;
constexpr int kRoadEdgeOffset = 653;
constexpr int kRoadEdgeMeanSize = 2 * kTrajectorySize * 2;
constexpr int kLeadOffset = 917;  // 평균 3x6x4, 이어서 log std
constexpr int kLeadElementSize = 4;
constexpr int kLeadMeanSize = kLeadMhpSelection * kLeadTrajLen * kLeadElementSize;
constexpr int kLeadProbOffset = 1061;
constexpr int kLeadProbSize = kLeadMhpSelection;
constexpr int kFeatureOffset = 1064;
constexpr int kPlanOffset = 1576;  // 평균 33x15, 이어서 log std
constexpr int kPlanWidth = 15;
// Plan 행 안의 위치(openpilot modeld.constants.Plan): T_FROM_CURRENT_EULER 9:12, ORIENTATION_RATE 12:15.
constexpr int kPlanYawIndex = 11;
constexpr int kPlanYawRateIndex = 14;
constexpr int kDesireStateOffset = 2566;
// meta 안의 가속 페달 예측(openpilot constants.Meta GAS_PRESS = meta[31:55:4]): 운전자가 0, 2, 4, 6, 8, 10초 뒤
// 가속 페달을 밟고 있을 확률(로짓, sigmoid). 바로 뒤 칸(32부터)은 브레이크 페달인데 읽는 곳이 없어 풀지 않는다.
constexpr int kMetaGasPressIndex = 31;
constexpr int kMetaPressStride = 4;
// 출력 끝의 두 칸은 쓰지 않는다(output_slices 밖).
constexpr int kOutputPadding = 2;
static_assert(kDesirePredOffset == kMetaOffset + kMetaSize && kPoseOffset == kDesirePredOffset + kDesirePredSize &&
                  kWideFromDeviceEulerOffset == kPoseOffset + kPoseSize &&
                  kRoadTransformOffset == kWideFromDeviceEulerOffset + kWideFromDeviceEulerSize &&
                  kLaneOffset == kRoadTransformOffset + kRoadTransformSize &&
                  kLaneProbOffset == kLaneOffset + kLaneLineSize * 2 &&
                  kRoadEdgeOffset == kLaneProbOffset + kLaneProbSize &&
                  kLeadOffset == kRoadEdgeOffset + kRoadEdgeMeanSize * 2 &&
                  kLeadProbOffset == kLeadOffset + kLeadMeanSize * 2 &&
                  kFeatureOffset == kLeadProbOffset + kLeadProbSize &&
                  kPlanOffset == kFeatureOffset + kModelFeatureLen &&
                  kDesireStateOffset == kPlanOffset + kTrajectorySize * kPlanWidth * 2 &&
                  kModelOutputFloats == kDesireStateOffset + kDesireLen + kOutputPadding,
              "openpilot master supercombo output layout");
}  // namespace model_output_layout

// openpilot T_IDXS / X_IDXS 격자. 같은 식이 여러 파일에 재정의되지 않도록
// 여기 한 벌만 둔다.
inline double model_t_idx_double(int i) {
    const double t = static_cast<double>(i) / static_cast<double>(kTrajectorySize - 1);
    return 10.0 * t * t;
}

inline double model_x_idx_double(int i) {
    const double t = static_cast<double>(i) / static_cast<double>(kTrajectorySize - 1);
    return 192.0 * t * t;
}

inline float model_t_idx(int i) { return static_cast<float>(model_t_idx_double(i)); }
inline float model_x_idx(int i) { return static_cast<float>(model_x_idx_double(i)); }

// plan(점의 앞 거리 plan_x)이 차선 점 거리(X_IDXS)에 닿는 시각(openpilot plan→lane_t). 끝을 넘는 점은 NaN이고(플래너가
// 보간에서 뺀다), 정차 부근의 뒤로 뛰는 knot은 두 knot 시각 사이로 묶여 lane_t는 단조다.
void compute_lane_t(const float plan_x[kTrajectorySize], float lane_t[kTrajectorySize]);

struct ModelPoint {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct ParsedPlan {
    std::array<ModelPoint, kTrajectorySize> points{};
    // T_FROM_CURRENT_EULER z / ORIENTATION_RATE z (rad, rad/s). openpilot 메인의
    // get_curvature_from_plan이 쓰는 값이다(laneless 모드).
    std::array<float, kTrajectorySize> yaw{};
    std::array<float, kTrajectorySize> yaw_rate{};
};

struct ParsedLaneLine {
    float probability = 0.0f;
    float std = 0.0f;
    std::array<ModelPoint, kTrajectorySize> points{};
};

struct ParsedRoadEdge {
    float std = 0.0f;
    std::array<ModelPoint, kTrajectorySize> points{};
};

constexpr int kMetaPressHorizons = 6;  // 0, 2, 4, 6, 8, 10 s

struct ParsedMeta {
    std::array<float, kDesireLen> desire_state{};
    // 운전자가 그 시각에 가속 페달을 밟고 있을 확률(openpilot disengagePredictions.gasPressProbs)
    std::array<float, kMetaPressHorizons> gas_press{};
};

struct ParsedLeadPoint {
    float x = 0.0f;
    float y = 0.0f;
    float velocity = 0.0f;
    float acceleration = 0.0f;
};

struct ParsedLeadPrediction {
    std::array<ParsedLeadPoint, kLeadTrajLen> points{};
};

// 모델 앞차를 앞차로 보는 최소 존재 확률. openpilot radard처럼 한 값으로 고정해 비전 크루즈, 출발 알림,
// HUD와 웹 BEV(scripts/web_console/static/bev_data.js LEAD_PROBABILITY, check_web_console.py가 대조)가 같은 앞차를
// 본다.
constexpr float kLeadProbabilityThreshold = 0.5f;

// 모델 앞차 거리(카메라 기준)를 레이더 기준으로 바꾸는 오프셋(openpilot RADAR_TO_CAMERA). 비전 크루즈와
// 출발 알림(controls_tick.cc), HUD 거리 표시(hud_scene.cc), 웹 BEV(bev_data.js RADAR_TO_CAMERA,
// check_web_console.py가 대조)가 같은 값을 쓴다.
constexpr float kRadarToCameraDistanceM = 1.52f;

// 시간 오프셋(0/2/4 s)마다 궤적 하나와 존재 확률 하나. primary는 time_idx 번째의 첫 점을 존재 확률이
// min_probability 이상일 때만 준다.
struct ParsedLeads {
    std::array<ParsedLeadPrediction, kLeadMhpSelection> predictions{};
    std::array<float, kLeadMhpSelection> global_probabilities{};

    bool primary(int time_idx, float min_probability, ParsedLeadPoint *lead, float *probability = nullptr) const;
};

// valid가 거짓이면(모델 출력이 아직 없음) 나머지는 0이다.
struct ParsedModelOutput {
    bool valid = false;
    ParsedPlan plan;
    std::array<ParsedLaneLine, 4> lanes{};
    std::array<ParsedRoadEdge, 2> road_edges{};
    ParsedLeads leads;
    ParsedMeta meta;
    PoseObservation pose{};
};

class ModelOutputParser {
public:
    // raw는 모델 출력 하나(kModelOutputFloats개)
    static ParsedModelOutput parse(const float *raw);
};
