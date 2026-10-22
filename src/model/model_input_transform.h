#pragma once

// 모델 입력 워프 행렬(모델 픽셀 → 소스 픽셀). 보정 rpy와 카메라 장착으로 medmodel·sbigmodel 3x3을 만든다.

#include "common/app_config.h"
#include "common/model_output.h"

enum class ModelFrame {
    MedModel,
    SmallBigModel,
};

// 모델 입력 워프 행렬(모델 픽셀 -> 소스 픽셀). 보정 rpy와 카메라 장착으로 GDC(maix_gdc_warp.h)가 쓸 3x3을 만든다.
class ModelInputTransform {
public:
    explicit ModelInputTransform(const AppConfig &config, ModelFrame model_frame = ModelFrame::MedModel);

    void set_calibration(float roll_rad, float pitch_rad, float yaw_rad);
    // 카메라 좌우 장착 보정(sunnypilot camera offset과 같다). 모델이 offset_m만큼 오른쪽(+)에 있는
    // 가상 카메라에서 본 영상을 받도록, 높이 height_m 아래 도로면 기준 호모그래피를 워프에 더한다.
    // 모델은 카메라 위치를 차 중심으로 보므로 +면 차가 왼쪽으로 간다. 값은 기기 설정이 범위로 묶어 둔다.
    void set_camera_mount(float offset_m, float height_m);
    void projection_matrix(float *projection) const;

private:
    float fx_, fy_, cx_, cy_;
    float roll_rad_ = 0.0f;
    float pitch_rad_ = 0.0f;
    float yaw_rad_ = 0.0f;
    float camera_offset_m_ = 0.0f;
    float camera_height_m_ = kModelHeight;
    ModelFrame model_frame_ = ModelFrame::MedModel;
};
