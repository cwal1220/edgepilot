#pragma once

/* 상류 opendbc CarStateBase.update_speed_kf: 휠 속도 평균(vEgoRaw)을 속도·가속도 칼만 필터(simple_kalman
 * KF1D, 정상 상태 이득)로 걸러 vEgo와 aEgo를 낸다. 상류처럼 100 Hz 제어 틱마다 한 번 부른다(휠 속도
 * 메시지가 50 Hz면 같은 값이 두 번 들어간다). */
class SpeedFilter {
public:
  SpeedFilter();

  /* vEgoRaw(m/s)를 넣고 vEgo(m/s)를 돌려준다. 2 m/s 넘게 벌어지면 그 값에서 다시 시작한다(상류: 서 있지
   * 않은 채 시작해도 가속도가 튀지 않게). 휠 속도가 낡아 NaN이면 NaN이고, 다음 값에서 다시 시작한다. */
  double update(double v_ego_raw);
  double a_ego() const { return a_ego_; }

private:
  // 정상 상태 칼만 이득(상류 get_kalman_gain)
  double gain_v_ = 0.0;
  double gain_a_ = 0.0;
  double v_ego_ = 0.0;
  double a_ego_ = 0.0;
  bool started_ = false;
};
