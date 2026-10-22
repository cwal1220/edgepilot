#pragma once

// 런타임 옵션(환경 변수)을 시작할 때 한 번 읽는다(AppConfig). MaixCAM2 카메라 내부 파라미터, 캡처 크기, overlayd와
// projection이 같이 쓰는 4:3 미리보기 자르기도 여기 있다.

#include <string>

constexpr unsigned kDefaultSensorWidth = 1920;
constexpr unsigned kDefaultSensorHeight = 1080;
// AI 캡처 기본 해상도. MaixCAM2(ov_os04d10, 가로 화각 80.6°)에서 720p는
// 13.2 px/°로 supercombo 학습 카메라(17.9 px/°)보다 거칠다. 1080p면 19.8 px/°
// 지만 파이프·워프 비용을 아직 재지 않았다.
constexpr unsigned kDefaultAiWidth = 1280;
constexpr unsigned kDefaultAiHeight = 720;
// MaixCAM2(ov_os04d10) 카메라 내부 파라미터, 1920x1080 기준. 런타임과 같은 카메라
// 경로(센서 전체 화각 축소, 보드 cam_flip/mirror)로 65" TV 11x6 체스보드를 찍어 구했다
// (2026-09-26, 42장, 재투영 0.52 px, 1σ fx/fy ±2, cx ±2.6, cy ±1.7; docs/model-pipeline.md).
// 모든 출력 해상도가 센서 전체 화각의 축소라 비례 환산된다.
// 왜곡(k1 -0.009, k2 0.013, k3 -0.021)은 평균 2.6 px라 워프는 핀홀로 둔다.
// 이전 값(순정 카메라 앱으로 측정, 1132.3/1131.5/932.8/556.1)과 초점거리는 같고
// cx가 7 px 달랐다.
constexpr float kCameraFx = 1131.24f;
constexpr float kCameraFy = 1130.85f;
constexpr float kCameraCx = 940.13f;
constexpr float kCameraCy = 552.60f;
// HUD 미리보기는 16:9 카메라 영상의 가운데를 이 비율(4:3, MaixCAM2 LCD)로 잘라
// 늘림 없이 보여 준다. overlayd의 IVPS 크롭과 차선 투영(projection.cc)이 함께 쓴다.
// 1920x1080 기준 크롭 폭 = 1080 * 4/3 = 1440.
constexpr float kPreviewAspect = 4.0f / 3.0f;
constexpr float kPreviewCropWidth1080 = static_cast<float>(kDefaultSensorHeight) * kPreviewAspect;

struct AppConfig {
    AppConfig() { set_warp_source(nv12_width, nv12_height); }

    std::string axmodel_path;

    unsigned nv12_width = kDefaultAiWidth;
    unsigned nv12_height = kDefaultAiHeight;

    // EDGEPILOT_CALIB_AUTO=0: 저장된 보정(calibration.json)을 고정해 쓰고 온라인 보정을 하지 않는다.
    bool calibration_auto = true;
    bool profile = false;
    // 광류 보정기(VPP·IVE 작업 스레드). 끄면 새로 재지 않는다. 저장해 둔 추정(params/flow_calibration.json)은 그래도
    // 기록되고 설정이 광류면 워프에 쓰이며, 없으면 ModelState.flow_calibration.status가 0이다. EDGEPILOT_FLOW_CALIB=0.
    bool flow_calibration = true;

    // 카메라 내부 파라미터(1920x1080 기준). EDGEPILOT_CAMERA_INTRINSICS="fx,fy,cx,cy"로 바꿀 수 있다(다른 카메라
    // 모듈). input_warp_*는 이 값을 캡처 해상도로 비례 환산한 것이다.
    float camera_fx = kCameraFx;
    float camera_fy = kCameraFy;
    float camera_cx = kCameraCx;
    float camera_cy = kCameraCy;
    float input_warp_fx, input_warp_fy, input_warp_cx, input_warp_cy;  // 생성자가 set_warp_source로 채운다

    static AppConfig from_env(int argc, char *argv[]);
    static AppConfig from_env_defaults();
    // camera_*를 width x height 소스로 환산해 input_warp_*에 넣는다.
    void set_warp_source(unsigned width, unsigned height);
    static std::string usage(const char *program_name);
};
