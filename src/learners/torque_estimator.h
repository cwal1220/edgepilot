#pragma once

// openpilot torqued(TorqueEstimator) 이식. 점은 locationd 요레이트·롤로 모은다(LateralLearners).

#include <algorithm>
#include <array>
#include <string>
#include <vector>

// CP.lateralTuning.torque. 사전값이자 허용 폭(배율 ±30%, 마찰 ±50%)과 캐시 키의 기준이다.
struct TorqueTuning {
    double lat_accel_factor = 0.0;  // 정규화 토크 1.0이 내는 횡가속도 [m/s²]
    double friction = 0.0;
};

// 한 제어 틱의 입력. 토크·요레이트는 상류 torqued처럼 우측 양수다(컨트롤러 곡률과 같은
// 관례라 학습 절편을 FF에 그대로 뺄 수 있다). 롤은 paramsd 관례.
struct TorqueEstimatorInput {
    double t_s = 0.0;
    bool inputs_fresh = false;  // 상류 sm.all_checks()
    bool lat_active = false;
    double steer_torque = 0.0;  // 실제 보낸 토크 / steer_max, 우측 양수
    double speed_mps = 0.0;
    bool steer_override = false;
    bool pose_valid = false;      // 요레이트·롤 출처가 유효
    double yaw_rate_rad_s = 0.0;  // 바이어스 제거 후, 우측 양수
    double roll_rad = 0.0;        // 양수 = 오른쪽이 낮다(paramsd·상류 NED 관례)
};

// lateralTorqueParameters. 제어는 필터값을 쓴다.
struct TorqueParams {
    bool inputs_ok = false;  // 봉투 valid
    bool valid = false;      // 필드 valid
    bool use_params = true;
    double lat_accel_factor_raw = 0.0;
    double lat_accel_offset_raw = 0.0;
    double friction_raw = 0.0;
    double lat_accel_factor = 0.0;
    double lat_accel_offset = 0.0;
    double friction = 0.0;
    int total_bucket_points = 0;
    int cal_perc = 0;
    double decay = 0.0;
    double max_resets = 0.0;
};

enum class TorqueRestore { None, Restored, KeyMismatch, Corrupt };
// 로그에 쓰는 이름(fresh, restored, key_mismatch, corrupt)
const char *torque_restore_name(TorqueRestore status);

class TorqueEstimator {
public:
    static constexpr int kBuckets = 8;
    static constexpr int kPointsPerBucket = 1500;
    static constexpr int kHistLen = 100;  // 5초 × 20 Hz

    // lag_s는 상류 lateralDelay 자리. cache가 비어 있지 않으면 상류처럼 먼저 복원한다.
    // Corrupt면 호출자가 캐시를 지운다(상류 params.remove). 키 불일치는 두기만 한다.
    TorqueEstimator(const TorqueTuning &offline, double lag_s, const std::string &cache = std::string());

    // 제어 틱마다 부른다. 4 Hz 출력 틱이면 true이고 params()가 갱신된다. 입력이 낡은 동안(차가 꺼짐)은 쉰다:
    // 상류 torqued는 시동이 켜졌을 때만 돈다.
    bool update(const TorqueEstimatorInput &in);
    // 생성 직후에는 복원한 점의 추정(필터는 그대로, inputs_ok 거짓)이고, 쉬는 동안은 지난 값에 inputs_ok만 거짓이다.
    const TorqueParams &params() const { return last_; }
    // 입력이 신선한 20 Hz 틱 240번(12초)마다, 첫 틱 포함. 상류처럼 get_msg를 한 번 더 돌린 결과라
    // 그 틱에는 필터가 두 번 갱신된다.
    bool persist_due() const { return persist_due_; }
    const std::string &persist_data() const { return persist_data_; }
    TorqueRestore restore_status() const { return restore_; }
    const TorqueTuning &tuning() const { return offline_; }
    // 상류 torqued는 lateralDelay 메시지(lagd)로 lag를 바꾼다. 이후 들어오는 점부터 적용한다. 값은 controlsd의 경로
    // 지연이다(lagd 0.15~0.65 s 또는 설정값).
    void set_lag(double lag_s) { lag_s_ = lag_s; }
    double lag() const { return lag_s_; }

    int bucket_size(int i) const { return buckets_[i].count; }
    int total_points() const;
    // 상류 get_points()[:, [0, 2]]: 버킷 순서, 버킷 안은 오래된 것부터.
    std::vector<std::array<double, 2>> points() const;

private:
    struct RawSample {
        double t = 0.0;  // 지연을 더한 시각
        double lat_active = 0.0;
        double steer = 0.0;
        double vego = 0.0;
        double steer_override = 0.0;
    };
    struct Bucket {  // NPQueue: 가득 차면 가장 오래된 점을 민다
        std::vector<std::array<double, 2>> points = std::vector<std::array<double, 2>>(kPointsPerBucket);
        int head = 0;
        int count = 0;
        // 오래된 점부터 f(점)을 부른다. 링의 두 구간을 차례로 돌아 점마다 나머지 연산을 하지 않는다(적합 4 Hz, 저장
        // 12초마다 최대 12,000점).
        template <class F>
        void for_each(F f) const {
            const int first = std::min(count, kPointsPerBucket - head);
            for (int i = 0; i < first; ++i) f(points[head + i]);
            for (int i = 0; i < count - first; ++i) f(points[i]);
        }
    };
    struct Filter {  // FirstOrderFilter(dt = DT_MDL)
        double x = 0.0;
        double alpha = 0.0;
    };

    void reset();
    void add_point(double x, double y);
    bool is_calculable() const;
    bool is_valid() const;
    int valid_percent() const;
    void estimate_params(double *slope, double *offset, double *friction) const;
    void update_params(double factor, double offset, double friction);
    void handle_device_motion(const TorqueEstimatorInput &in);
    double interp_raw(double t, double RawSample::*field) const;
    TorqueParams get_msg(bool inputs_ok);
    std::string serialize(const TorqueParams &p) const;
    TorqueRestore restore(const std::string &cache, double *factor, double *offset, double *friction);

    TorqueTuning offline_;
    double lag_s_ = 0.0;
    double min_factor_ = 0.0, max_factor_ = 0.0, min_friction_ = 0.0, max_friction_ = 0.0;
    double resets_ = 0.0;
    double decay_ = 0.0;
    std::array<RawSample, kHistLen> raw_{};
    int raw_head_ = 0;  // 가장 오래된 표본
    int raw_count_ = 0;
    std::array<Bucket, kBuckets> buckets_{};
    Filter factor_f_, offset_f_, friction_f_;
    long frame_ = -1;
    bool has_motion_t_ = false;
    double last_motion_t_ = 0.0;
    bool persist_due_ = false;
    long new_points_ = 0;  // 마지막 저장 뒤 더한 점
    TorqueRestore restore_ = TorqueRestore::None;
    std::string persist_data_;
    TorqueParams last_{};
};
