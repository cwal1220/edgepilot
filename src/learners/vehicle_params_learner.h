#pragma once

// openpilot paramsd(VehicleParamsLearner)·car_kf(CarKalman) 이식.
// 상류와 다른 것: 예측은 1 ms로 분할(상류의 0.05 s 오일러 한 걸음은 7.5 m/s 아래에서 발산),
// 조향각·속도는 매 틱 관측(상류는 20 Hz, 아래 VehicleParamsOptions), 입력이 낡으면 비활성과 같이
// 처리, locationd 롤이 무효인 틱은 ESP12 횡가속 롤로 대신한다.

#include <array>
#include <string>

struct VehicleModelConstants {
    double wheelbase_m = 0.0;
    double center_to_front_m = 0.0;
    double tire_stiffness_factor = 1.0;
    double steer_ratio = 0.0;  // 사전값. 유효 범위 [0.5, 2]배의 기준
};

// CarKalman(car_kf.py): 선형 단일트랙 동역학 위에서 SR·강성·영점·롤을 푸는 EKF.
class CarKalman {
public:
    static constexpr int kN = 9;
    enum State : int {
        kStiffness,
        kSteerRatio,
        kAngleOffset,
        kAngleOffsetFast,
        kSpeedX,
        kSpeedY,
        kYawRate,
        kSteerAngle,
        kRoadRoll
    };
    using Vec = std::array<double, kN>;
    using Mat = std::array<std::array<double, kN>, kN>;

    struct Globals {
        double inertia = 0.0, center_to_front = 0.0, center_to_rear = 0.0;  // 관성과 아래 강성은 질량당 값
        double stiffness_front = 0.0, stiffness_rear = 0.0;
    };

    static Vec initial_x();
    static Mat q();
    static Globals globals_from(const VehicleModelConstants &c);
    // 1차 오일러 한 걸음과 그 야코비안. 검사가 수치 미분과 대조한다.
    static void step(const Globals &g, const Vec &x, double h, Vec *x_next, Mat *f);

    explicit CarKalman(const Globals &g) : g_(g) {}
    void init(const Vec &x, const Mat &p, bool has_time, double t);
    void set_time(double t) {
        t_ = t;
        has_time_ = true;
    }
    double time() const { return t_; }
    // rednose predict_and_observe. 한 상태를 직접 보는 관측만 있다(H가 단위행).
    void predict_and_observe(double t, int state, double z, double r);

    const Vec &x() const { return x_; }
    const Mat &p() const { return p_; }
    Vec &mutable_x() { return x_; }  // 검사용

private:
    void predict(double dt);

    Globals g_;
    Vec x_{};
    Mat p_{};
    double t_ = 0.0;
    bool has_time_ = false;
};

// 한 제어 틱(100 Hz)의 입력. 요레이트·횡가속은 좌측 양수.
struct VehicleParamsInput {
    double t_s = 0.0;
    bool inputs_fresh = false;  // 상류 sm.all_checks()
    double steering_angle_deg = 0.0;
    double speed_mps = 0.0;
    int gear = 0;
    bool yaw_rate_valid = false;
    double yaw_rate_rad_s = 0.0;  // 바이어스 제거 후
    // 관측 표준편차. 기본은 ESP12 고주파 잡음(0.06~0.08°/s)에 여유를 둔 값, locationd면 그 std(상류)
    double yaw_rate_std_rad_s = 0.1 * 3.14159265358979323846 / 180.0;
    bool lat_accel_valid = false;
    double lat_accel_mps2 = 0.0;  // 비력(가속도계), 좌측 양수
    // 상류 paramsd처럼 locationd 자세 롤로 도로 롤을 관측한다(given이면 ESP 횡가속 대신).
    // 오른쪽이 낮으면 양수. valid는 locationd 센서 정상(상류 sensorsOK). 무효이거나 std·범위를
    // 벗어나면 그 틱은 ESP 횡가속 롤로 대신한다(아래 handle_device_motion).
    bool localizer_roll_given = false;
    bool localizer_roll_valid = false;
    double localizer_roll_rad = 0.0;
    double localizer_roll_std_rad = 0.0;
};

// vehicleParameters 메시지.
struct VehicleParams {
    bool inputs_ok = false;  // 봉투 valid
    bool valid = false;      // 필드 valid
    bool sensor_valid = true;
    double steer_ratio = 0.0;
    double stiffness_factor = 0.0;
    double roll_rad = 0.0;
    double angle_offset_average_deg = 0.0;
    double angle_offset_deg = 0.0;
    bool steer_ratio_valid = false;
    bool stiffness_factor_valid = false;
    bool angle_offset_average_valid = false;
    bool angle_offset_valid = false;
    double steer_ratio_std = 0.0;
    double stiffness_factor_std = 0.0;
    double angle_offset_average_std = 0.0;
    double angle_offset_fast_std = 0.0;
};

struct VehicleParamsOptions {
    const CarKalman::Mat *p_initial = nullptr;  // nullptr면 상류처럼 P0 = Q
    // 매 제어 틱 관측한다. false면 상류처럼 20 Hz마다 최신 조향각·속도만 보는데, 조향 입력이
    // 50 ms씩 멈춰 들어가 SR을 낮게 배우고(합성 참값 15.2 → 14.3~14.6) 빠른 영점이 커브마다
    // 그 몫을 메워 진입·탈출 곡률 오차가 6~9% 크다(2026-09-21·22 세 주행).
    bool car_state_every_tick = true;
};

class VehicleParamsLearner {
public:
    VehicleParamsLearner(const VehicleModelConstants &c, double steer_ratio, double stiffness_factor,
                         double angle_offset_rad, const VehicleParamsOptions &options = {});

    // 제어 틱마다 부른다. 20 Hz 틱이면 true이고 params()가 갱신된다.
    bool update(const VehicleParamsInput &in);
    const VehicleParams &params() const { return last_; }
    // 출력 1200번(1분)마다. 첫 출력에서도 참이다(상류 sm.frame % 1200 == 0).
    bool persist_due() const { return persist_due_; }
    CarKalman &kf() { return kf_; }

private:
    void reset(bool has_time, double t);
    void handle_car_state(const VehicleParamsInput &in);
    void handle_device_motion(const VehicleParamsInput &in);
    VehicleParams get_msg(bool inputs_ok);

    VehicleModelConstants c_;
    bool car_state_every_tick_ = true;
    double min_sr_ = 0.0;
    double max_sr_ = 0.0;
    CarKalman kf_;
    CarKalman::Vec x_initial_{};
    CarKalman::Mat p_initial_{};
    double observed_speed_ = 0.0;
    double observed_yaw_rate_ = 0.0;
    double observed_roll_ = 0.0;
    bool avg_offset_valid_ = true;
    bool total_offset_valid_ = true;
    bool roll_valid_ = true;
    double angle_offset_deg_ = 0.0;
    double avg_angle_offset_deg_ = 0.0;
    double roll_ = 0.0;
    bool active_ = false;
    bool has_motion_t_ = false;
    double last_motion_t_ = 0.0;
    long motion_frame_ = -1;
    bool persist_due_ = false;
    VehicleParams last_{};
};

// 상류 retrieve_initial_vehicle_params. 거부되면 restored=false로 사전값을 돌려주고,
// 호출자는 저장 파일을 지운다. 강성은 복원하지 않는다(젖은 노면 값이 넘어오지 않게).
struct VehicleParamsInit {
    bool restored = false;
    double steer_ratio = 0.0;
    double stiffness_factor = 1.0;
    double angle_offset_deg = 0.0;
    double yaw_bias_rad_s = 0.0;
};
VehicleParamsInit restore_vehicle_params(const std::string &json, const VehicleModelConstants &c);
std::string vehicle_params_persist_data(const VehicleParams &p, const VehicleModelConstants &c, double yaw_bias_rad_s);
