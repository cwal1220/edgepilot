// 모델 입력 변환을 openpilot 참조식과 대조한다: 투영 행렬 ↔ model.py get_warp_matrix, 카메라 장착 이동, 호스트 도구
// (tools/model/model_warp.py)와 나누는 기준값, app_config의 카메라 내부 파라미터 환경 변수, GDC 워프 뒤의 YUV6 풀기.
#include "common/app_config.h"
#include "common/projection.h"
#include "common/utils_math.h"
#include "model/model_input_transform.h"
#include "model/yuv6_pack.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <stdexcept>
#include <vector>

namespace {

constexpr int kModelW = 512;
constexpr int kModelH = 256;

void matmul3d(const double *a, const double *b, double *out) {
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            double sum = 0.0;
            for (int k = 0; k < 3; ++k) sum += a[r * 3 + k] * b[k * 3 + c];
            out[r * 3 + c] = sum;
        }
    }
}

// rot = Rz(yaw)·Ry(pitch)·Rx(roll), openpilot rot_from_euler과 같은 순서
void rot_from_euler_ref(const double rpy[3], double *rot) {
    const double cr = std::cos(rpy[0]);
    const double sr = std::sin(rpy[0]);
    const double cp = std::cos(rpy[1]);
    const double sp = std::sin(rpy[1]);
    const double cy = std::cos(rpy[2]);
    const double sy = std::sin(rpy[2]);

    // clang-format off
    const double rx[9] = {
        1.0, 0.0, 0.0,
        0.0, cr, -sr,
        0.0, sr, cr,
    };
    const double ry[9] = {
        cp, 0.0, sp,
        0.0, 1.0, 0.0,
        -sp, 0.0, cp,
    };
    const double rz[9] = {
        cy, -sy, 0.0,
        sy, cy, 0.0,
        0.0, 0.0, 1.0,
    };
    // clang-format on

    double tmp[9];
    matmul3d(ry, rx, tmp);
    matmul3d(rz, tmp, rot);
}

TEST(ModelInputTransform, AppConfigEnvFeedback) {
    AppConfig fallback = AppConfig::from_env_defaults();
    EXPECT_EQ(fallback.nv12_width, kDefaultAiWidth) << "ISP 출력 폭 기본값은 오버스캔 폭";
    EXPECT_EQ(fallback.nv12_height, kDefaultAiHeight) << "ISP 출력 높이 기본값은 오버스캔 높이";
    EXPECT_NEAR(fallback.input_warp_fx, kCameraFx * 1280.0f / 1920.0f, 1e-3) << "ISP 출력 크기로 카메라 fx를 맞춘다";
    EXPECT_NEAR(fallback.input_warp_fy, kCameraFy * 720.0f / 1080.0f, 1e-3) << "ISP 출력 크기로 카메라 fy를 맞춘다";
    EXPECT_NEAR(fallback.input_warp_cx, kCameraCx * 1280.0f / 1920.0f, 1e-3) << "ISP 출력 크기로 카메라 cx를 맞춘다";
    EXPECT_NEAR(fallback.input_warp_cy, kCameraCy * 720.0f / 1080.0f, 1e-3) << "ISP 출력 크기로 카메라 cy를 맞춘다";
    const AppConfig plain;
    EXPECT_EQ(plain.input_warp_fx, fallback.input_warp_fx) << "기본 생성도 같은 환산";

    // 다른 카메라의 1080p 내부 파라미터로 바꾸면 비례 환산된다
    setenv("EDGEPILOT_CAMERA_INTRINSICS", "1583.3981,1583.7622,954.9441,545.1774", 1);
    AppConfig other = AppConfig::from_env_defaults();
    EXPECT_NEAR(other.input_warp_fx, 1583.3981f * 1280.0f / 1920.0f, 1e-3) << "fx 덮어쓰기";
    EXPECT_NEAR(other.input_warp_cy, 545.1774f * 720.0f / 1080.0f, 1e-3) << "cy 덮어쓰기";
    other.set_warp_source(1920, 1080);
    EXPECT_NEAR(other.input_warp_cx, 954.9441f, 1e-3) << "1080p 소스면 그대로";
    setenv("EDGEPILOT_CAMERA_INTRINSICS", "oops", 1);
    EXPECT_THROW(AppConfig::from_env_defaults(), std::runtime_error) << "잘못된 형식은 거부";
    unsetenv("EDGEPILOT_CAMERA_INTRINSICS");
}

// 모델 프레임 → 카메라 영상 투영 행렬의 참조식. openpilot common/transformations/model.py의
// get_warp_matrix를 double로 옮겼다: intrinsics @ view_frame_from_device_frame @ rot_from_euler(rpy)
// @ inv(model_intrinsics @ view_frame_from_device_frame).
void projection_reference(float roll, float pitch, float yaw, float fx, float fy, float cx, float cy,
                          double *projection, ModelFrame model_frame = ModelFrame::MedModel) {
    const bool big = model_frame == ModelFrame::SmallBigModel;
    const double model_f = big ? 455.0 : 910.0;
    // clang-format off
    const double model_k[9] = {
        model_f, 0.0, 256.0,
        0.0, model_f, big ? 0.5 * (256.0 + 47.6) : 47.6,
        0.0, 0.0, 1.0,
    };
    // clang-format on
    const double view[9] = {0, 1, 0, 0, 0, 1, 1, 0, 0};
    const double k[9] = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
    const double rpy[3] = {roll, pitch, yaw};
    double rot[9];
    rot_from_euler_ref(rpy, rot);
    double kv[9], kvr[9], mkv[9];
    matmul3d(k, view, kv);
    matmul3d(kv, rot, kvr);
    matmul3d(model_k, view, mkv);
    // mkv의 역행렬(여인수 전개)
    const double *m = mkv;
    const double det =
        m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6]) + m[2] * (m[3] * m[7] - m[4] * m[6]);
    const double inv[9] = {
        (m[4] * m[8] - m[5] * m[7]) / det, (m[2] * m[7] - m[1] * m[8]) / det, (m[1] * m[5] - m[2] * m[4]) / det,
        (m[5] * m[6] - m[3] * m[8]) / det, (m[0] * m[8] - m[2] * m[6]) / det, (m[2] * m[3] - m[0] * m[5]) / det,
        (m[3] * m[7] - m[4] * m[6]) / det, (m[1] * m[6] - m[0] * m[7]) / det, (m[0] * m[4] - m[1] * m[3]) / det,
    };
    matmul3d(kvr, inv, projection);
}

TEST(ModelInputTransform, ProjectionMatchesOpenpilot) {
    AppConfig camera_config;
    ModelInputTransform camera_transform(camera_config);
    float camera_projection[9];
    double camera_reference[9];
    camera_transform.projection_matrix(camera_projection);
    projection_reference(0.0, 0.0, 0.0, camera_config.input_warp_fx, camera_config.input_warp_fy,
                         camera_config.input_warp_cx, camera_config.input_warp_cy, camera_reference);
    for (int i = 0; i < 9; ++i)
        EXPECT_NEAR(camera_projection[i], camera_reference[i], 1e-4)
            << "기본 투영은 MaixCAM2 카메라 내부 파라미터를 쓴다";

    ModelInputTransform sbig_camera_transform(camera_config, ModelFrame::SmallBigModel);
    float sbig_camera_projection[9];
    double sbig_camera_reference[9];
    sbig_camera_transform.projection_matrix(sbig_camera_projection);
    projection_reference(0.0, 0.0, 0.0, camera_config.input_warp_fx, camera_config.input_warp_fy,
                         camera_config.input_warp_cx, camera_config.input_warp_cy, sbig_camera_reference,
                         ModelFrame::SmallBigModel);
    for (int i = 0; i < 9; ++i)
        EXPECT_NEAR(sbig_camera_projection[i], sbig_camera_reference[i], 1e-4)
            << "sbig 투영은 openpilot 가상 카메라를 쓴다";

    const std::array<std::array<float, 3>, 7> cases = {{
        {{0.0f, 0.0f, 0.0f}},
        {{0.0f, deg_to_rad(1.5f), 0.0f}},
        {{0.0f, deg_to_rad(-1.5f), 0.0f}},
        {{0.0f, 0.0f, deg_to_rad(1.0f)}},
        {{0.0f, deg_to_rad(1.1f), deg_to_rad(-0.8f)}},
        {{deg_to_rad(1.0f), 0.0f, 0.0f}},
        {{deg_to_rad(-2.0f), deg_to_rad(2.5f), deg_to_rad(1.5f)}},
    }};

    for (const auto &rpy : cases) {
        AppConfig config;
        ModelInputTransform transform(config);
        transform.set_calibration(rpy[0], rpy[1], rpy[2]);
        float actual[9];
        double expected[9];
        transform.projection_matrix(actual);
        projection_reference(rpy[0], rpy[1], rpy[2], config.input_warp_fx, config.input_warp_fy, config.input_warp_cx,
                             config.input_warp_cy, expected);
        for (int i = 0; i < 9; ++i) {
            const double tolerance = std::max(1e-4, std::fabs(expected[i]) * 1e-5);
            EXPECT_NEAR(actual[i], expected[i], tolerance) << "투영 행렬";
        }
    }
}

// GDC 워프 뒤의 YUV6 풀기(model/yuv6_pack.h): NEON 경로(이 보드와 Apple 실리콘)가 openpilot 순서의 바이트 단위
// 참조와 같다. 16열로 나눠떨어지지 않는 폭은 스칼라 꼬리가 맡는다.
TEST(Yuv6Pack, MatchesTheOpenpilotOrder) {
    for (const int width : {kModelW, 40}) {
        const int height = width == kModelW ? kModelH : 6;
        SCOPED_TRACE(width);
        std::vector<uint8_t> nv12(static_cast<size_t>(width) * height * 3 / 2);
        uint32_t seed = 12345;
        for (uint8_t &b : nv12) b = static_cast<uint8_t>((seed = seed * 1103515245u + 12345u) >> 24);
        std::vector<uint8_t> packed(static_cast<size_t>(width) * height * 3 / 2, 0);
        nv12_to_yuv6(nv12.data(), packed.data(), width, height);
        const int hw = width / 2, hh = height / 2, plane = hw * hh;
        for (int r = 0; r < hh; ++r) {
            for (int c = 0; c < hw; ++c) {
                const uint8_t *y0 = nv12.data() + 2 * r * width, *y1 = y0 + width;
                const uint8_t *uv = nv12.data() + width * height + r * width;
                const int at = r * hw + c;
                const uint8_t expected[6] = {y0[2 * c],     y1[2 * c], y0[2 * c + 1],
                                             y1[2 * c + 1], uv[2 * c], uv[2 * c + 1]};
                for (int p = 0; p < 6; ++p)
                    ASSERT_EQ(packed[p * plane + at], expected[p]) << "평면 " << p << " 행 " << r << " 열 " << c;
            }
        }
    }
}

// 카메라 장착 보정(sunnypilot camera offset): 가상 카메라가 offset만큼 오른쪽에 있을 때 가상 기준
// 호스트 도구의 워프(tools/model/model_warp.py projection_matrix)와 같은 행렬: 720p MaixCAM2 내부 파라미터에서
// rpy·모델 시점·카메라 장착별 모델 화소 -> 원본 화소 행렬. tests/check_recording_reader.py가 이 기준값을 읽어
// Python 쪽도 맞춘다. 상대 1e-5(단정밀도 곱셈 순서 차이) 안에서 같아야 한다.
TEST(ModelInputTransform, ProjectionGoldenSharedWithModelWarpPy) {
    // clang-format off
    // GOLDEN BEGIN: roll, pitch, yaw, sbig, camera_offset_m, camera_height_m -> 행렬 9개(행 우선)
    const struct {
        float roll, pitch, yaw;
        bool sbig;
        float offset, height;
        double p[9];
    } golden[] = {
        {0.0f, 0.0f, 0.0f, false, 0.0f, 1.22f,
         {0.828747272, 0, 414.594025, 0, 0.828461528, 328.96524, 0, 0, 1}},
        {0.02f, -0.03f, 0.01f, false, 0.0f, 1.22f,
         {0.821236074, -0.0373377427, 425.518921, 0.0122703603, 0.815863907, 348.853088,
          -1.1645795e-05, -3.27340858e-05, 1.00403953}},
        {0.02f, -0.03f, 0.01f, true, 0.0f, 1.22f,
         {1.64247215, -0.0746754855, 224.840942, 0.0245407205, 1.63172781, 136.850723,
          -2.329159e-05, -6.54681717e-05, 1.01540077}},
        {-0.015f, 0.04f, -0.02f, false, 0.2f, 1.35f,
         {0.841857553, 0.164225429, 387.723969, -0.00456404313, 0.843335032, 298.909393,
          2.13150761e-05, 4.7418016e-05, 0.991286516}},
        {-0.015f, 0.04f, -0.02f, true, -0.15f, 1.1f,
         {1.68371511, -0.150585905, 202.884506, -0.00912808627, 1.68926716, 83.7897797,
          4.26301522e-05, 8.27072508e-05, 0.975531995}},
    };
    // GOLDEN END
    // clang-format on
    AppConfig config;
    config.set_warp_source(1280, 720);
    for (const auto &g : golden) {
        ModelInputTransform transform(config, g.sbig ? ModelFrame::SmallBigModel : ModelFrame::MedModel);
        transform.set_calibration(g.roll, g.pitch, g.yaw);
        transform.set_camera_mount(g.offset, g.height);
        float p[9];
        transform.projection_matrix(p);
        for (int i = 0; i < 9; ++i) {
            // 원근 항(6, 7)은 1e-5 크기라 절대 오차로, 나머지는 상대 오차로 본다.
            const double tolerance = i == 6 || i == 7 ? 1e-9 : 1e-5 * std::max(1.0, std::fabs(g.p[i]));
            EXPECT_NEAR(p[i], g.p[i], tolerance) << "경우 rpy " << g.roll << " sbig " << g.sbig << " 번호 " << i;
        }
    }
}

// 도로면 점의 모델 픽셀은, 실제 카메라에서 그 도로 점(가상 위치 + offset)이 찍히는 픽셀로 간다.
// offset 0이면 기존 행렬과 같다.
TEST(ModelInputTransform, CameraMountShiftsGroundPlane) {
    AppConfig config;
    config.set_warp_source(1280, 720);
    const float roll = 0.004f, pitch = -0.012f, yaw = 0.009f;
    const float offset = 0.2f, height = 1.3f;
    for (const bool big : {false, true}) {
        ModelInputTransform plain(config, big ? ModelFrame::SmallBigModel : ModelFrame::MedModel);
        ModelInputTransform shifted(config, big ? ModelFrame::SmallBigModel : ModelFrame::MedModel);
        plain.set_calibration(roll, pitch, yaw);
        shifted.set_calibration(roll, pitch, yaw);
        shifted.set_camera_mount(offset, height);
        float p0[9], p1[9];
        plain.projection_matrix(p0);
        shifted.projection_matrix(p1);
        ModelInputTransform zero(config, big ? ModelFrame::SmallBigModel : ModelFrame::MedModel);
        zero.set_calibration(roll, pitch, yaw);
        zero.set_camera_mount(0.0f, height);
        float pz[9];
        zero.projection_matrix(pz);
        for (int i = 0; i < 9; ++i) EXPECT_FLOAT_EQ(pz[i], p0[i]) << "offset 0은 기존과 같다";

        const float mf = big ? 455.0f : 910.0f, mcx = 256.0f, mcy = big ? 0.5f * (256.0f + 47.6f) : 47.6f;
        float rot[9];
        rotation_from_rpy(roll, pitch, yaw, rot);
        for (const float y_right : {-1.5f, 0.0f, 1.8f}) {
            for (const float x_fwd : {8.0f, 20.0f, 45.0f}) {
                // 가상 카메라 기준 도로면 점(보정 좌표계: x 앞, y 오른쪽, z 아래 = 높이)
                const float pv[3] = {x_fwd, y_right, height};
                // 모델 픽셀: K_m · view · P (view: (y, z, x))
                const float u = mf * pv[1] / pv[0] + mcx, v = mf * pv[2] / pv[0] + mcy;
                const float w = p1[6] * u + p1[7] * v + p1[8];
                const float sx = (p1[0] * u + p1[1] * v + p1[2]) / w, sy = (p1[3] * u + p1[4] * v + p1[5]) / w;
                // 실제 카메라: 같은 도로 점은 P_v + (0, offset, 0)
                const float pr[3] = {pv[0], pv[1] + offset, pv[2]};
                float d[3];
                for (int r = 0; r < 3; ++r) d[r] = rot[r * 3] * pr[0] + rot[r * 3 + 1] * pr[1] + rot[r * 3 + 2] * pr[2];
                const float ex = config.input_warp_fx * d[1] / d[0] + config.input_warp_cx;
                const float ey = config.input_warp_fy * d[2] / d[0] + config.input_warp_cy;
                EXPECT_NEAR(sx, ex, 0.02f) << "big " << big << " x " << x_fwd << " y " << y_right;
                EXPECT_NEAR(sy, ey, 0.02f);
            }
        }
    }
}

}  // namespace
