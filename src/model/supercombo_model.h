#ifndef SUPERCOMBO_MODEL_H
#define SUPERCOMBO_MODEL_H

#include "common/app_config.h"
#include "model/ax_engine_session.h"
#include "model/model_input_transform.h"
#include "common/model_output.h"
#include "model/model_temporal.h"
#include "maixcam2/maix_gdc_warp.h"

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

/* openpilot master supercombo 코어(axmodel)를 AX630C NPU에서 돌린다. 공개 ONNX가
 * 그래프 안에 갖던 이미지·desire·특징 큐는 여기서 관리한다(model_temporal.h).
 * 입력: input_imgs/big_input_imgs uint8 [1,12,128,256], desire [1,25,8],
 * features_buffer [1,24,512], traffic_convention [1,2]. 출력 [1,2576]. */
class SupercomboModel
{
public:
    SupercomboModel(const char *model_file, const AppConfig &config);
    ~SupercomboModel();
    SupercomboModel(const SupercomboModel &) = delete;
    SupercomboModel &operator=(const SupercomboModel &) = delete;

    bool run_frame_nv12(const uint8_t *nv12, int src_w, int src_h,
                        std::vector<float> &raw_output);
    /* 하드웨어(GDC) 워프를 쓰면 소비자가 프레임을 이 CMM 버퍼에 직접 채우고
     * run_frame_preloaded로 실행한다. GDC를 못 쓰는 경우(NV21 소스,
     * EDGEPILOT_WARP_CPU=1, 크기 불일치) nullptr. */
    uint8_t *input_buffer(int src_w, int src_h);
    bool run_frame_preloaded(int src_w, int src_h, std::vector<float> &raw_output);
    /* 소스 NV12가 다른 CMM 블록(프레임 링 슬롯)에 있을 때 GDC가 직접 읽는다. 워프 뒤
     * source_still_valid()가 거짓이면(읽는 동안 덮어써짐) 모델 상태를 바꾸지 않고 false. */
    bool run_frame_phys(unsigned long long src_phys, int src_w, int src_h, std::vector<float> &raw_output,
                        const std::function<bool()> &source_still_valid);
    void set_input_calibration(const float rpy[3]);
    void set_camera_mount(float offset_m, float height_m);
    void set_desire(int desire);
    // 소스 프레임의 크로마 순서(NV21이면 true). 워프 두 탑에 같이 적용한다.
    void set_chroma_vu(bool vu);

private:
    /* CPU 워프 대체 경로에서 wide 탑 워프를 두 번째 코어에서 돌린다. 워프는 원본
     * NV12의 캐시 미스가 지배해서 두 탑을 나누면 시간이 크게 준다(메모리 대역을
     * 함께 쓰므로 절반까지는 안 된다). 출력은 한 스레드일 때와 비트 동일하다.
     * CPU 워프가 처음 필요할 때 띄운다(GDC만 쓰면 띄우지 않는다). */
    void wide_worker();
    bool infer(uint64_t t0, uint64_t t1, std::vector<float> &raw_output);

    enum Input { kRoad, kWide, kDesire, kFeatures, kTraffic, kInputCount };

    AxEngineSession session_;
    int index_[kInputCount] = {};
    /* 출력 헤드를 나눈 axmodel이면 model_output_assembly::kParts 순서대로 세션 출력 번호,
     * 예전 단일 출력(2576) axmodel이면 비어 있다. */
    std::vector<int> output_parts_;
    bool profile_ = false;  // EDGEPILOT_PROFILE: 30프레임마다 단계별 평균 ms
    ModelInputTransform input_transform_;
    ModelInputTransform big_input_transform_;
    std::unique_ptr<GdcWarp> gdc_;
    bool chroma_vu_ = false;
    ModelImageHistory road_history_;
    ModelImageHistory wide_history_;
    SupercomboTemporalState temporal_;
    double profile_ms_[4] = {};
    unsigned profile_frames_ = 0;

    std::thread wide_thread_;
    std::mutex wide_mutex_;
    std::condition_variable wide_cv_;
    const uint8_t *wide_src_ = nullptr;
    int wide_w_ = 0;
    int wide_h_ = 0;
    uint64_t wide_request_ = 0;
    uint64_t wide_done_ = 0;
    bool wide_ok_ = true;
    bool wide_stop_ = false;
};

#endif
