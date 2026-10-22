#pragma once

// 광류 보정기. 카메라가 실제로 움직이는 방향(소실점, FOE)을 영상에서 직접 잰다. 창 시작 프레임에서 고른
// 점들을 1프레임씩 kFlowWindowSteps번 따라가고(0.4초), 시작·끝 점 쌍으로 이동 방향과 작은 회전을 함께
// 푼다. 모델 pose를 거치지 않는 절대 측정이라 지금 보정 루프와 달리 피드백이 없다. 지금 루프는 모델이
// 워프 yaw를 18%만 따라가서 고정점이 치우친다(2026-10 분석, docs/flow-calibration.md).
// 이 파일은 하드웨어와 무관한 부분(풀이, 코너 고르기, 조건, 블록 집계)이다. 피라미드·코너 응답·LK는
// IVE가 맡는다(platform/maixcam2/maix_ive_flow, model/flow_worker). 추정은
// ModelState.flow_calibration에 기록되고, 웹 기기 설정이 광류면 워프의 pitch·yaw가 된다
// (model/calibration_source).

#include "common/ipc_messages.h"  // FlowCalibrationState

#include <cstdint>
#include <deque>
#include <vector>

constexpr int kFlowWindowSteps = 8;    // 창 길이(프레임). 20 fps에서 0.4초
constexpr int kFlowMaxPoints = 300;    // 창마다 고르는 점
constexpr int kFlowMinTracks = 50;     // 이보다 적게 남으면 창을 버린다
constexpr int kFlowBlockWindows = 20;  // 블록 하나에 넣는 창 수(중앙값을 낸다)
constexpr int kFlowMaxBlocks = 30;     // 기억하는 블록 수(곧은 길 약 4분 반)
constexpr int kFlowMinBlocks = 3;      // 추정을 내는 최소 블록 수
// 창 안 점 이동량 중앙값의 하한(px). 서 있는 장면(0.05 px 안팎)은 어떤 방향이든 에피폴라 식을 맞춰 무작위
// 방향이 통과했다(10-11 데스크 테스트). 실제 창(40 km/h 이상, 0.4초)은 야간 p1 0.85 px, 주간 p1 0.32 px이고
// 1 px 아래 창은 yaw가 0.1~0.2° 치우쳤다(먼 점만 남음). 1 px이면 야간 2.5%, 주간 8%를 버린다.
constexpr double kFlowMinParallaxPx = 1.0;
// 장착 변경: 최근 kFlowMountCheckBlocks개 블록이 모두 그 전 블록 평균에서 같은 쪽으로 kFlowMountShiftDeg보다
// 멀고 서로는 kFlowMountAgreeDeg 안이면 예전 블록을 버린다. 2026-10-22 보드를 다시 달자 카메라가 요 0.8°, 피치 0.7° 틀어졌는데
// 1.0° 문턱으로는 못 잡아, 저장된 30블록의 평균이 5분 동안 0.05°만 움직였고 차는 차로 중앙보다 30 cm 넘게 왼쪽으로
// 갔다(시내 주행이라 블록이 1~4분에 하나). 0.5°면 새 블록 셋째(306 s)에 잡는다. 10-12 이후 열세 경로의 블록 340개에서
// 0.4°까지 낮춰도 잘못 잡은 적이 없다(한 경로 안 블록 중앙값의 표준편차 중앙값 0.08°).
constexpr int kFlowMountCheckBlocks = 3;
constexpr double kFlowMountShiftDeg = 0.5;
constexpr double kFlowMountAgreeDeg = 0.5;

struct FlowIntrinsics {
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
};

struct FlowSolution {
    bool valid = false;
    double yaw_rad = 0.0;       // atan(m_x / m_z): calibration과 같은 규약
    double pitch_rad = 0.0;     // -atan(m_y / m_z)
    double rotation_rad = 0.0;  // 창 동안 카메라가 돈 각도
    double parallax_px = 0.0;   // 점 이동량(시작→끝)의 중앙값
    int inliers = 0;
    int points = 0;
};

// 창 시작·끝의 픽셀 좌표 n쌍에서 카메라 이동 방향을 푼다. 이동 방향 m = (a, b, 1)과 작은 회전 w를 두고
// 에피폴라 잔차 x2ᵀ R(w) [m]x x1을 Sampson으로 정규화한 뒤, Cauchy 가중 가우스-뉴턴으로 푼다. 시작값은
// 정면 이동과 회전 0이다. 곧은 길에서는 창 동안의 회전이 작아 여기서 출발해도 충분하다. 점이 50개보다
// 적거나, 이동량 중앙값이 kFlowMinParallaxPx보다 작거나(풀지 않는다), 맞는 점이 40개보다 적거나, 회전이
// 0.5°를 넘으면 valid가 false다(OpenCV 필수행렬과 0.03° 안, 구간별 흩어짐은 3배 작았다).
FlowSolution solve_flow_direction(const FlowIntrinsics &k, const float *start_xy, const float *end_xy, int n);

// 코너 응답 영상(5x5 Shi-Tomasi 최소 고유값, S16, 폭 width x 높이 height, stride = width)에서 점을 고른다.
// 8x8 칸마다 가장 강한 화소를 후보로 삼고, 최댓값의 quality배를 넘는 후보를 강한 순으로 고른다. 이미
// 고른 이웃 칸의 점과 5화소 안이면 건너뛴다. 응답은 축소 단 영상에서 냈다고 보고, 원본 좌표
// ((x + 0.5)·scale - 0.5, (y + row_offset + 0.5)·scale - 0.5)로 xy에 쓴다. 고른 점 수를 돌려준다.
int pick_flow_corners(const int16_t *response, int width, int height, int scale, int row_offset, double quality,
                      int max_points, float *xy);

// 잴 만한 순간인지. 곧고 빠르게 갈 때만 이동 방향이 차의 앞 방향과 같다.
struct FlowGateInput {
    float v_ego = 0.0f;      // m/s
    float curvature = 0.0f;  // 1/m (ControlState.actual_curvature)
    bool blinker = false;
    bool desire = false;  // 차선 변경 같은 desire가 있다
    bool vehicle_fresh = false;
};
bool flow_gate_open(const FlowGateInput &in);

// 끝난 블록의 중앙값들(오래된 것부터, rad). 재시작해도 추정을 바로 쓰도록 저장하는 내용이다
// (params/flow_calibration.json, model/calibration_service).
struct FlowBlocks {
    std::vector<float> yaw;
    std::vector<float> pitch;
    uint32_t windows = 0;
    uint32_t mount_changes = 0;
};

// 광류 측정원(보드: model/flow_worker의 FlowWorker, 시험: 가짜). 보정 서비스
// (model/calibration_service)가 모델 프레임마다 상태를 읽고, 끝난 블록을 받아 저장하고, 초기화를 요청한다.
// 측정원은 자기 스레드에서 돌아도 되고, 세 함수는 다른 스레드에서 불러도 안전해야 한다.
class FlowSource {
public:
    virtual ~FlowSource() = default;
    // 지금 상태와, 지금까지 처리한 초기화 수(request_reset 순서대로 늘어난다)
    virtual FlowCalibrationState state(uint32_t *resets_applied) const = 0;
    // 블록이 끝났거나 장착 변경으로 줄었으면 그 내용을 넘기고 true. 한 번 넘긴 내용은 다시 넘기지 않는다
    virtual bool take_blocks(FlowBlocks *out) = 0;
    // 블록과 진행 중인 창을 버리게 한다(다음 프레임 전에)
    virtual void request_reset() = 0;
};

// 측정 창 결과를 kFlowBlockWindows개씩 묶어 블록 중앙값을 내고, 최근 kFlowMaxBlocks개 블록 중앙값의
// 평균을 추정으로 낸다. 창 하나는 잡음이 크지만(IQR 약 ±0.5°), 블록 중앙값은 이상치에 강하고 블록
// 평균은 잡음을 줄인다. 장착이 바뀌면(kFlowMount*) 최근 블록만 남겨 바로 새 값으로 간다.
class FlowCalibrator {
public:
    FlowCalibrator() { reset(); }
    void reset();
    // 저장해 둔 블록으로 시작한다. 길이가 다르거나 유한하지 않은 값이 있으면 버리고 false. 최근
    // kFlowMaxBlocks개만 쓴다.
    bool restore(const FlowBlocks &blocks);
    FlowBlocks blocks() const;
    // 받아들인 창 하나(valid인 풀이). end_frame_id는 창의 끝 프레임.
    void add(const FlowSolution &window, int tracks, uint64_t end_frame_id);
    // 끝까지 갔지만 풀이가 맞지 않은 창. 수와 이동량을 남긴다(조건 조정용).
    void note_rejected(const FlowSolution &window) {
        ++state_.rejected_windows;
        state_.window_parallax = static_cast<float>(window.parallax_px);
    }
    FlowCalibrationState state() const { return state_; }

private:
    void detect_mount_change();
    void update_estimate();

    std::vector<float> block_yaw_, block_pitch_;  // 지금 모으는 블록
    std::deque<float> med_yaw_, med_pitch_;       // 끝난 블록의 중앙값(오래된 것부터, 최근 kFlowMaxBlocks개)
    FlowCalibrationState state_{};
};
