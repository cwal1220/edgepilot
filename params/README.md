# K7 YG HEV 파라미터 안내

이 디렉터리에는 KIA K7 YG HEV용 조향 제어, 주행 제한, 비전 크루즈, 기록, 디스플레이 및
카메라 캘리브레이션 값이 들어 있다. `controlsd`는 세 런타임 JSON
변경을 100ms 이내에 감지하고 다음 제어 주기에 즉시 반영한다. 제어 상태와
PID 상태는 유지되며 engage 여부에 따른 적용 지연은 없다.

기본 경로는 실행 디렉터리의 `params/`이다. 다른 디렉터리를 사용하려면
`EDGEPILOT_PARAMS_DIR=/path/to/params`를 지정한다.

## 튜닝 전 주의사항

- 조향 토크, 차량 고정값과 CAN 통신값은 반드시 정차 상태에서 수정한다.
- 비전 크루즈 주행 튜닝은 운전자가 직접 조작하지 말고 동승자가 한 항목씩 소폭 변경한다.
- Panda는 먼저 `nooutput` 또는 TX 비활성 상태에서 데이터와 부하를 검증한다.
- 한 번에 하나의 값만 조금씩 바꾸고, 변경 전후 로그를 남긴다.
- 허용 범위를 벗어난 숫자는 로더가 아래 표의 범위로 제한한다.
- JSON은 주석을 지원하지 않으므로 설명은 이 문서에서 관리한다.
- 차량 제원은 다른 차량 값으로 임의 변경하지 않는다.
- 웹 콘솔은 `EDGEPILOT_ENABLE_WEB_CONSOLE=1`일 때 기본 8080 포트에서 실행된다.
  `차량 특성` 탭은 학습되는 조향 값(조향비, 타이어 강성, 조향각 영점, 도로 기울기 보정, 토크 배율·마찰·편향, 조향
  지연)을 학습기(paramsd·torqued·lagd)별로 묶어, 값마다 지금 제어에 쓰는 값과 학습값·수동값을 한 카드에 보여준다.
  묶음마다 `자동 학습`/`수동`(아래 `use_live_*`)을 고르고, 수동값을 고치거나 학습값을 수동값으로 저장한다. 카메라
  캘리브레이션 초기화, 최근 10분 추이, 고정 제원(질량·축거·무게중심)도 이 탭에 있다.

## recording.json

| 파라미터 | 기본값 | 단위 | 설명 |
|---|---:|---|---|
| `enabled` | false | bool | 웹의 `주행 기록` 메뉴에서 변경한다. 켜면 모델이 실제 선택한 1280x720 20 FPS 프레임을 AX630C 하드웨어 H.264 인코더(VENC)로 저장하고 CAN RX/TX 및 모델·제어 상태를 함께 기록한다. |
| `bitrate_bps` | 8000000 | bps / 1M~20M | H.264 인코더 목표 비트레이트다. 인코더는 recordd 기동 시 한 번 열리므로 변경은 재시작부터 적용된다. |

기록은 `recordings/<시각>/` 아래에 생성된다. 영상은 60초 단위 H.264
세그먼트(`road.h264`)로 나뉜다. 프레임의 원본 캡처 시각은 각 세그먼트의 `frames.bin`,
CAN과 상태는 60초 청크 `events/NNN.bin`, 당시 파라미터는 `params/`에
저장된다. 남은 공간이 5 GiB
또는 전체 용량의 10% 미만이면 새 기록을 시작하지 않거나 진행 중인 기록을
종료한다.

## display.json

| 파라미터 | 기본값 | 단위 | 설명 |
|---|---:|---|---|
| `enabled` | true | bool | LCD 영상 파이프라인은 유지하고 백라이트만 켜거나 끈다(duty 0). |
| `brightness_percent` | 100 | % | 백라이트 PWM3(pwmchip0/pwm3, 10 kHz, 정극성) 점등률. 실제 duty는 밝기 x 보드 최대치(`/boot/board`의 `disp_max_backlight`, 기본 95%)다. |
| `alert_volume_percent` | 70 | % / 0~100 | 보드 스피커 알림음 크기. 0이면 소리를 내지 않는다. overlayd가 1초마다 파일을 보고 바뀌면 적용한 뒤 확인음을 한 번 낸다. 값이 없으면 `EDGEPILOT_ALERT_VOLUME`(기본 70)을 쓴다. |
| `camera_offset_m` | 0.0 | m / -0.35~0.35 | 보드를 차 중심에서 옆으로 달았을 때 모델이 차 중심에서 본 것처럼 입력 영상을 옮긴다(sunnypilot camera offset 방식). +면 차가 차선 안에서 왼쪽으로 간다(보드를 운전석 쪽에 달았으면 그 거리만큼 +). Lane·Laneless 모두에 적용되고 HUD 차선도 같이 맞춘다. ±0.35 m를 넘으면 앞차·신호등 인식이 나빠져 막아 둔다. modeld가 1초마다 파일을 보고, 바뀐 값은 초당 0.1 m씩 옮긴다. |
| `camera_height_m` | 1.22 | m / 0.8~2.0 | 도로면에서 카메라까지 높이다. `camera_offset_m`의 효과 크기를 정하는 데만 쓴다(오프셋이 0이면 무관). |
| `hud_debug` | false | bool | 주행 화면 왼쪽에 다른 카드에 없는 수치를 모은 진단 카드(FPS, 조향 토크, paramsd 강성·평균 영점, torqued 원시 추정·진행률, lagd 블록)를 띄운다. 아직 유효하지 않은 학습기 줄은 주황이다. 기어는 설정 속도 옆 카드에, TPMS와 카메라 보정은 아래 모서리 카드에, 보드 상태(CPU 온도·CPU·RAM·디스크)는 TPMS 위 카드에, 제어가 쓰는 학습값(SR·영점·토크 계수·지연)은 보정 위 카드에 늘 있고, 네트워크 정보는 오른쪽 위 상태 알약을 누르면 나온다. overlayd가 1초 안에 반영한다. 주행 화면 왼쪽 열을 눌러도 켜고 끌 수 있고, 그건 이 값이 바뀌거나 overlayd가 다시 시작할 때까지만 간다. |

설정은 웹의 `기기 설정` 메뉴에서 바꾼다. 백라이트는 웹 콘솔이 즉시 적용하고, 뜰 때마다
다시 적용한다. 알림음 크기는 웹 콘솔이 파일만 고치고 overlayd가 읽는다.

## driving.json

### 상태와 CAN

모델 경로와 차량 CAN 상태를 낡았다고 보는 시간(250 ms, 500 ms), 해제 뒤 순정 LKAS로 넘기기 전에
0 토크 프레임을 이어 보내는 시간(3초, openpilot_c2의 기본 인계 시간과 같다), 저속에서도 MDPS가 LKAS를 받도록 MDPS 버스에 쓰는 속도
(60 km/h)는 상류(openpilot·opendbc, 커뮤니티 포크)처럼 코드 상수다(`src/controls/control_params.h`의
`DrivingParams`). 파일에서 읽지 않는다.

### 차선 변경

| 파라미터 | 현재값 | 단위 / 허용 범위 | 설명 |
|---|---:|---|---|
| `lane_change_min_speed_kph` | 30.0 | km/h / 0~80 | 이 속도 미만에서는 차선 변경 desire를 내지 않는다. |

차선 변경은 openpilot desire_helper와 같다. 깜빡이를 켜고 그 방향으로 핸들을 밀면(운전자 토크
150 초과) 시작해 모델에 차선 변경 desire 펄스를 주고, 차선선 가중치를 0.5초에 걸쳐 뺀다. 모델이
차선 변경을 마쳤다고 할 때(`desire_state` 차선 변경 확률 0.02 미만), 10초가 지났을 때, 조향이
비활성일 때만 끝난다. 끝나면 차선선을 0.5초에 되살린다(openpilot은 1초).

운전자가 핸들을 잡아도 요청 토크를 줄이지 않는다(openpilot과 같음). 운전자와 반대 방향 토크만
panda와 같은 운전자 클램프(허용 50 + 운전자 토크 x 2)와 변화율 제한(프레임당 3 올림 / 7 내림)이
서서히 줄이고, 핸들을 잡은 동안에는 토크 컨트롤러의 적분기만 멈춘다. 예전의
`driver_torque_threshold`(운전자 토크 170을 1초 넘으면 방향과 무관하게 토크를 0까지 줄임)는
같은 방향으로 거드는 급커브에서도 어시스트를 없애 2026-09-27에 지웠다.

### 경로 모드

| 파라미터 | 현재값 | 단위 / 허용 범위 | 설명 |
|---|---:|---|---|
| `laneless_mode` | false | bool | `false`는 Lane 모드로, 차선 확률이 높으면 차선 중심 경로를 섞고 낮아지면 자동으로 모델 경로만 쓴다. `true`는 Laneless 모드로, openpilot 메인과 같은 방식(`get_curvature_from_plan`)이다. 차선·MPC·`path_offset_m` 없이 모델 plan의 yaw와 yaw rate로 목표 곡률 `2·ψ(t_d)/(v·t_d) − ψ̇(0)/v`를 만든다(t_d = 조향 지연 + plan 나이). Lane 모드에서 차선이 안 보이거나(교차로 등, 유효 확률 0.3 미만) 회전 desire를 줄 때도 같은 계산을 쓰고, 차선 MPC와의 인계는 목표 곡률에서 섞는다(차선을 버릴 때 0.5초, 되찾을 때 0.25초). 2026-10-06 전에는 이 구간에서 plan 위치를 차선 MPC로 따랐는데, 25 km/h 아래에서 laneless보다 26~30% 덜 꺾고 0.3~0.4초 늦었다(녹화 재생). 실주행 저속 회전의 운전자 토크 중앙값도 152로, laneless 80의 두 배였다. 웹의 `주행 제한` 메뉴에서 바꾸고, 현재 사용 중인 경로는 HUD에 `LANE`/`LANELESS`로 표시된다. 차로 변경은 두 모드 모두 모델의 desire 입력으로 동작한다. |
| `turn_desire` | false | bool | 실험 기능(openpilot에 없음). 결합 중 차선 변경 최소 속도(`lane_change_min_speed_kph`)보다 느릴 때 깜빡이를 켜면 모델에 좌·우회전 desire를 준다. desire는 펄스라 2.5초마다 다시 주고, 그동안은 Lane 모드라도 모델 경로를 따른다. 깜빡이를 끄면 바로 풀린다. 저속 차선 변경이나 갓길 정차도 회전으로 알리니 그때는 깜빡이를 끄거나 직접 조향한다. HUD에 `TURN LEFT`/`TURN RIGHT`로 표시된다. |

### 경로 제한

횡방향 경로 제한은 런타임 항목이 아니라 `src/controls/lateral_controller.h`의 고정
상수다: 최대 횡저크 `5.0 m/s^3`, 최대 횡가속 `3.0 m/s^2`(학습한 도로 롤만큼 이동), 최종 목표 곡률의 절대
상한은 openpilot과 같은 `0.2 1/m`(`kMaxCurvature`)로 적용되며 설정 항목으로 노출하지 않는다.

## adaptive_cruise.json

이 기능은 비전 모델의 선행차 거리와 상대속도를 이용해 순정 고정형 크루즈의
`SET-`와 `RES+` 버튼만 대신 누른다. 스로틀이나 브레이크를 직접 제어하지 않으므로
차간거리 유지와 감속을 보장하지 않으며 운전자가 항상 제동을 담당해야 한다.

브레이크 페달(AHB1 스트로크 3 mm 초과)을 밟으면 순정 크루즈와 함께 꺼지고, 운전자가 `SET`이나
`RES`로 순정 크루즈를 다시 켜야 버튼 조절이 다시 시작된다. 2026-10-06 전에는 이 차의 제동을 보지
못해, 브레이크 뒤에도 버튼을 눌러 순정 크루즈를 다시 켰다.

| 파라미터 | 현재값 | 단위 / 허용 범위 | 설명 |
|---|---:|---|---|
| `enabled` | true | bool | 비전 기반 순정 크루즈 버튼 조절을 켠다. `false`로 바꾸면 진행 중인 자동 버튼 펄스와 세션을 즉시 해제한다. |
| `standstill_gap_m` | 5.0 | m / 2~20 | 속도와 관계없이 목표 차간거리에 더하는 기본 거리다. |
| `following_time_s` | 1.8 | s / 0.8~4.0 | 현재 속도에 곱해 동적 차간거리를 만드는 시간 간격이다. |
| `gap_correction_gain` | 0.25 | gain / 0.05~1.0 | 실제 거리와 목표 거리의 오차를 목표 속도 보정으로 바꾸는 비율이다. |
| `max_slowdown_correction_mps` | 4.0 | m/s / 0.5~10 | 선행차가 가깝거나 느릴 때 목표 속도를 낮출 수 있는 최대 보정량이다. |
| `max_speedup_correction_mps` | 2.5 | m/s / 0~5 | 차간거리가 충분할 때 선행차 속도보다 높게 잡을 수 있는 최대 보정량이다. 최초 SET 상한은 넘지 않는다. |
| `deceleration_rate_kph_per_s` | 1.5 | km/h/s / 0.5~5 | `SET-` 이후 실측한 차량 감속 응답이다. 2 km/h 설정 단계를 이 값으로 나눈 시간을 연속 감속 명령의 최소 응답 시간으로 사용하고, 그동안 가까워질 선행차 거리도 미리 반영한다. |
| `lead_hold_s` | 0.6 | s / 0.1~2.0 | 비전 검출이 잠시 끊겨도 마지막 선행차를 유지하는 시간이다. |
| `lead_restore_delay_s` | 2.0 | s / 1~10 | 선행차가 사라진 뒤 최초 SET 상한으로 복귀하거나, 마지막 `SET-` 뒤 `RES+`로 명령 방향을 바꾸기 전 대기 시간이다. |
| `command_interval_s` | 1.0 | s / 0.5~5.0 | 연속 버튼 펄스를 시작할 수 있는 최소 간격이다. |

버튼 한 번을 이어 보내는 CLU11 프레임 수(5)는 상류와 커뮤니티 포크처럼 코드 상수다
(`AdaptiveCruiseConfig::button_pulse_frames`).

## steering.json

### 기본 토크 제한

`steer_max`(384), `steer_delta_up`(3), `steer_delta_down`(7), `steer_driver_allowance`(50),
`steer_driver_multiplier`(2), `steer_driver_factor`(1)은 panda의 hyundai safety가 같은 숫자를
강제하므로(`safety_hyundai.h`) 런타임 항목에서 뺐다. 올리면 panda가 프레임을 거부하고 내리면
순정보다 약해지기만 한다. 바꾸려면 panda 펌웨어와 함께 바꾸고 `src/controls/control_params.h`를 고친다.
조향각 게이트(`max_steering_angle_deg`)는 fault 회피가 켜져 있으면 실행되지 않는 죽은 경로라
파라미터와 코드를 함께 제거했다. 2026-09-18 실측에서 fault 회피는 끌 수 없는 것으로 확인됐다.

| 파라미터 | 현재값 | 단위 / 허용 범위 | 설명 |
|---|---:|---|---|
| `enabled` | true | bool | `false`면 조향 컨트롤러를 비활성화한다. |
| `steering_pressed_threshold` | 150 | MDPS raw torque / 0~500 | 토크 PID의 적분을 멈추는 운전자 조향 감지 기준이다. RK openpilot과 같이 5프레임 필터를 거치며, CAN 안전 제한용 `steer_driver_allowance`와는 별개다. |

### OpenPilot 토크 컨트롤러

값은 openpilot `latcontrol_torque`와 같은 표현이다(횡가속도 공간). 토크 = (FF + P + I) ÷
`torque_lat_accel_factor` + 마찰이므로 배율만 바꾸면 세 항이 같이 움직이고, KP·KI는 배율과
무관하게 상류 숫자와 바로 비교된다. 2026-09-24 이전 파일의 `torque_*_raw` 키는 로더가 거부한다.
옮길 때는 배율 = max_lat_raw ÷ kf_raw, KP = kp_raw ÷ 10, KI = ki_raw ÷ kf_raw, 마찰 = friction_raw ÷ 1000이다.

| 파라미터 | 현재값 | 단위 / 허용 범위 | 설명 |
|---|---:|---|---|
| `torque_lat_accel_factor` | 2.75 | m/s^2 / 0.5~5.0 | 정규화 토크 1.0이 내는 횡가속도다(openpilot `latAccelFactor`). 선행·비례·적분 토크가 모두 이 값으로 나뉜다. 2.75는 MaixCAM2 torqued 학습값(2026-09-27)이다. 이전 4.44는 SR 16.8의 과대 피드백과 짝을 이뤄 맞았고, SR 14.9로 고친 뒤 커브 언더스티어(좌커브 곡률비 0.86, 바깥 0.3 m)로 드러났다. 2026-09-23 실차에서 2.78로 바꿔 좌·우커브가 중앙(−0.07/−0.03 m)으로 돌아왔다. |
| `torque_kp` | 0.8 | gain / 0~10 | 횡가속도 오차의 비례 이득이다(openpilot `KP`). 속도별 이득 곡선 `KP_INTERP` [250, 120, 65, 30, 11.5, 5.5, 3.5, 2.0, KP]의 마지막 점(30 m/s 이상)이다. 나머지 점은 상류 고정값이라 저속 이득은 이 값으로 바뀌지 않는다. 이전 2.0은 v0.11 포팅 이후 튜닝된 적이 없는 상속값이었다. 2026-09-20 실차에서 확인했다. |
| `torque_ki` | 0.15 | gain / 0~2 | 횡가속도 오차의 적분 이득이다(openpilot `KI`, 상류 기본값). 이전 raw 표기에선 0.33(상류의 2.2배)이 드러나지 않았다. 2026-09-24 실차에서 0.15로 바꿔 좌커브 곡률비 0.90→0.96(n≈17), 직선 흔들림은 그대로였다. |
| `torque_friction` | 0.083 | 정규화 토크 / 0~0.3 | 조향계 마찰을 넘기 위해 오차 방향으로 더하는 토크다(openpilot `friction`, 상류도 토크 공간에 더한다). |

### 차량 모델과 지연

| 파라미터 | 현재값 | 단위 / 허용 범위 | 설명 |
|---|---:|---|---|
| `steer_ratio` | 14.72 | ratio / 8~25 | 핸들 조향각과 전륜 조향각의 비율이다. paramsd 학습값(2026-09-27)이고, RK의 KIA K7 HEV 차량값은 16.8이다. |
| `tire_stiffness_factor` | 0.83 | 배율 / 0.2~2.0 | 기준 타이어 횡강성에 적용하는 차량별 보정 계수다. paramsd 학습값(2026-09-27)이고, RK K7의 기본 배율은 1.0이다. |
| `steer_actuator_delay` | 0.42 | second / 0.01~1.0 | 조향 액추에이터 지연이다. **두 곳에 쓰인다.** `lag_adjusted_desired_curvature`는 이 값만큼 MPC 경로를 앞에서 읽어 커브 진입을 선행하고, 토크 컨트롤러의 요청 버퍼는 이 값만큼 **지난** 목표를 지금 측정과 비교한다. 그래서 키우면 선행은 늘지만 피드백이 낡아진다. 곡률이 빠르게 조여질 때 차가 이미 낡은 목표를 넘어서 있으면 오차 부호가 뒤집혀 커브 한복판에서 토크가 빠진다 — 0.46에서 조여지는 커브 69건 중 36%가 0.4초 안에 요청 토크가 30% 아래로 주저앉았고, 0.34에서는 14%였다(쌍 비교 McNemar p=0.0015). 상류 현대 기본값은 0.1(일부 0.2)이다. |
| `angle_offset_deg` | -1.57 | degree / -10~10 | 조향각 센서의 직진 오프셋이다. 실제 곡률 추정 전에 센서 각도에서 뺀다. paramsd 학습 평균(2026-09-21~23 주행, −1.5~−1.6°)을 넣었다. 이전 −0.7°는 0.9° 차이로 곡률 약 4e-4를 틀려, 좌커브는 덜 돌고 우커브는 더 도는 것처럼 보이게 했다. |
| `torque_lat_accel_offset` | -0.06 | m/s^2 / -1.0~1.0 | 장착 롤 오차 등이 만드는 상수 횡가속 편향을 feed-forward에서 뺀다(openpilot latAccelOffset). 값은 `tools/control/fit_lateral_params.py fit`으로 주행 로그에서 실측한다. 양수 = 차가 오른쪽으로 쏠릴 때 키우는 방향(+y=오른쪽 관례, openpilot 문서와 반대 어휘). fit 출력을 그대로 넣는다. |
| `live_bank_compensation` | true | bool | ESP12 실측으로 추정한 도로 편경사(2초 필터)를 feed-forward에서 실시간 보정한다. 센서로 보이는 편향(크라운·영점)은 bank가 흡수하므로, `torque_lat_accel_offset`은 센서에 안 보이는 토크 경로/기계 편향 전용 트림으로만 쓴다. 정적 roll 항목은 이것으로 대체되어 없앴다. |
| `use_live_vehicle_params` | true | bool | openpilot paramsd처럼 주행 중 학습한 조향비·타이어 강성·조향각 영점(합계)·도로 롤을 쓴다. 켜면 실제 곡률 계산이 `steer_ratio`·`angle_offset_deg` 대신 학습값을 쓰고, 롤은 실제 곡률·feed-forward(−roll·g)·곡률 횡가속 한계에 들어가며 `live_bank_compensation`을 대신한다. 학습값이 무효가 되면(영점·롤 한계 초과 등, 캘리브 완료 후) `paramsd_invalid`로 해제한다. 끄면 계산·기록만 한다. 학습 상태는 `live_parameters.json`(1분마다)에 남고 다음 시동에 이어진다. 웹 콘솔에서는 `차량 특성` 탭의 `자동 학습`/`수동`으로 고른다. |
| `use_live_torque_params` | true | bool | openpilot torqued처럼 토크→횡가속 배율(latAccelFactor)·편향·마찰을 학습해 토크 컨트롤러에 쓴다. 배율은 `torque_lat_accel_factor`(기본 2.75)의 ±30%, 마찰은 `torque_friction`의 ±50% 안이다. 켜면 `torque_lat_accel_offset`은 학습 편향으로 바뀐다. 버킷이 다 차기 전(`cal_perc` < 100)에는 사전값 그대로다. 점과 필터는 `live_torque_parameters.bin`(12초마다)에 남는다. 웹 콘솔에서는 `차량 특성` 탭의 `자동 학습`/`수동`으로 고른다. |
| `use_live_delay` | false | bool | openpilot lagd처럼 locationd가 추정한 조향 지연(목표 곡률 → 실제 요레이트)을 경로 지연(목표 곡률을 읽는 시점), 토크 컨트롤러의 요청 지연, torqued 지연에 쓴다. 추정이 확정(5블록)된 뒤에만 적용되고, 그 전에는 `steer_actuator_delay`를 쓴다. 웹 콘솔에서는 `차량 특성` 탭의 `자동 학습`/`수동`으로 고른다. |
| `use_locationd_learner_inputs` | true | bool | paramsd·torqued가 요레이트와 도로 롤을 locationd(IMU·카메라 융합)에서 받는다(openpilot과 같다). 끄면 ESP12 요레이트(자체 바이어스 추정)와 ESP12 횡가속으로 구한 롤을 쓴다. locationd가 없거나 낡으면 ESP12로 돌아간다. torqued는 시작할 때의 출처로만 점을 모으므로, 바꾸면 torqued에는 다음 시작부터 적용된다. 웹 콘솔에서는 `차량 특성` 탭의 학습 입력에서 켜고 끈다. |
| `mass_kg` | 1816.0 | kg / 1000~2600 | 차량 모델과 타이어 횡강성 계산에 사용하는 차량 질량이다. |
| `wheelbase_m` | 2.855 | m / 2.0~3.5 | 차량 축거다. |
| `center_to_front_ratio` | 0.4 | wheelbase ratio / 0.2~0.7 | 무게중심에서 전축까지 거리의 축거 대비 비율이다. |
| `path_offset_m` | 0.0 | m / -1~1 | 차선 중심 경로에 더하는 사용자 횡방향 보정이다. 양수는 목표 주행 위치를 우측, 음수는 좌측으로 이동한다. 차선이 안 보여 모델 경로를 따를 때(교차로 등)와 laneless 모드에는 적용하지 않는다: 차 기준인 모델 경로에 더하면 위치 고정점 없이 차가 그쪽으로 계속 밀린다. 실제 적용량은 차선 가중치를 따라 줄어든다. |
| `lane_path_weight` | 3.0 | weight / 0.5~10 | Lane 모드 MPC의 경로(횡위치) 가중치다. openpilot 0.9.4 `PATH_COST`는 1이다. 2026-10-05 폐루프 재생에서 3이면 고속도로 오른쪽 커브의 안쪽 치우침이 13.4 cm에서 10.6 cm로 줄고(급커브 18→14 cm) 횡저크는 12~19% 늘었다. Laneless 모드에는 쓰지 않는다. |
| `min_steer_speed_mps` | 1.0 | m/s / 0~5 | 이 속도 미만에서는 조향 토크를 내지 않는다(openpilot CP.minSteerSpeed). 1.1~3.6 km/h 크립에서 v0.9.4 plan이 포화 지시를 내는 대역을 덮는다. |

### 조향각 및 LKAS fault 보호

| 파라미터 | 현재값 | 단위 / 허용 범위 | 설명 |
|---|---:|---|---|
| `avoid_lkas_fault_enabled` | true | bool | 큰 조향각 fault 회피를 사용한다. K7 YG HEV 실측(2026-09-18): steer 요청이 켜진 채 85도 위에 0.98~1.12초 머물면 MDPS가 토크와 무관하게 ToiFlt/FailState를 세우고 각도가 85도 아래로 돌아올 때까지 어시스트를 끊는다. 85도 위에서도 토크는 그대로 내되 허용 프레임에 0에 닿을 만큼만 남기고(`steer_delta_down` × 남은 프레임), 그 프레임부터 85도 아래로 올 때까지 steer request를 ToiFlt 없이 끈다. 정차 대기 중에도 같고, 85도 위에서는 request를 새로 켜지 않는다. 그 체류 중 운전자가 핸들을 돌렸으면(조향 감지) 85도 아래가 아니라 15도 아래로 오고 손을 뗄 때까지 끈 채로 둔다(carrotpilot 해제 조건): 회전을 빠져나오며 핸들을 펴는 운전자를 밀지 않는다. 끄면 상류 hyundai `carcontroller.py`의 `common_fault_avoidance`를 쓴다: 85도 위에서 request를 89프레임 낸 뒤 2프레임 끊고(ToiFlt를 켜고 토크는 그대로) 다시 센다. K7에서는 끊었던 request를 85도 위에서 다시 켜면 3~14 ms 안에 fault가 났다(2026-10-03, 4번 중 4번). 끄면 안 된다. |
| `avoid_lkas_fault_hold_angle_deg` | 80.0 | degree / 0~180 | 운전자가 핸들을 잡지 않았을 때(steering pressed 아님) 컨트롤러가 스스로 가는 최대 핸들 각도다. 목표 곡률을 이 각도가 내는 곡률(학습 SR·강성·오프셋·롤을 쓰는 차량 모델) 안으로 묶어, 85도를 넘겨 0.89초 뒤 토크가 빠지고 핸들이 풀렸다 다시 잡는 반복 대신 이 각도에서 토크를 끊김 없이 유지한다. 운전자가 조향 중이면 묶지 않는다(더 감는 운전자를 밀지 않는다). 횡가속 한계(3.0 m/s²)가 더 좁은 약 35 km/h 위에서는 걸리지 않는다. 0이면 끈다. |

MDPS 고장 한계는 코드 상수다(`SteeringParams`): 85도 위에서 89프레임까지 요청을 유지하고, 이는 실측 고장
하한 98프레임보다 9프레임 이르다. 상류 hyundai `carcontroller.py`의 `MAX_ANGLE`·`MAX_ANGLE_FRAMES`와 같은
값이다. 회피를 끈 경우에 요청을 끊는 길이(2프레임)도 상류 `MAX_ANGLE_CONSECUTIVE_FRAMES`와 같은 코드 상수다. 고장이 더 일찍 나는 상황이 발견되면 이 상수를 낮춘다.

## calibration.json

이 파일은 일반 튜닝 파일이 아니라 온라인 카메라 캘리브레이션의 저장 상태다.
`EDGEPILOT_CALIB_AUTO=1`일 때 모델 pose와 실제 차량 속도로 갱신된다. 보드나
카메라 장착 각도가 달라지거나 카메라 내부 파라미터를 다시 측정하면 기존 값을
그대로 사용하지 말고 다시 캘리브레이션한다.

여기 있는 값은 새 설치의 초기 seed다. `scripts/upload_to_board.sh`는 보드에
이미 있는 런타임 파일을 덮어쓰지 않고 이 파일을 `params.defaults/`에 둔다.

| 파라미터 | 현재값 | 단위 | 설명 |
|---|---:|---|---|
| `version` | 1 | schema version | 저장 형식 버전이다. 사용자가 변경하지 않는다. |
| `rpy_rad` | `[-0.00134, -0.01746, -0.01960]` | radian `[roll, pitch, yaw]` | 카메라 자세 보정값이다. 약 `[-0.077, -1.001, -1.123]`도다. 유효 범위는 upstream과 같은 pitch -0.0907~0.17, yaw ±0.0691 rad이고, 범위를 벗어난 저장값도 불러와 invalid로 둔다. |
| `spread_rad` | `[0, 0, 0]` | radian `[roll, pitch, yaw]` | 유효 블록 사이 캘리브레이션 값의 분산 범위다. 작을수록 관측이 안정적이다. |
| `valid_blocks` | 0 | block | 저장된 유효 캘리브레이션 블록 수다. 5개 이상이면 calibrated 상태가 될 수 있다. |
| `height_m` | (없음 → 1.22) | m | 모델 road_transform z로 블록 평균한 도로면에서 카메라까지 높이다(upstream `extrinsicsCalibration.height`, 2026-10-05 추가). 없는 예전 파일은 1.22로 시작한다. 보정 뒤에는 높이 표준편차가 e^-3.5(0.030 m)보다 큰 표본을 버린다. 장착 변경으로 다시 모으면 1.22로 돌아간다. `display.camera_height_m`(카메라 오프셋 크기)에는 아직 쓰지 않는다. |

`valid_blocks`가 0인 것은 의도한 값이다. 2026-08-22에 카메라 내부 파라미터를
다시 측정하면서 `fx`가 2.6% 바뀌었고, 여기 있던 자세값은 이전 내부 파라미터
아래에서 학습된 것이라 그 오차를 일부 흡수하고 있었다(새 행렬 기준으로 다시
맞추면 pitch +4.9 mrad, yaw -5.8 mrad 차이가 난다). `rpy_rad`는 출발점으로
그대로 두고 `valid_blocks`만 지워 새 내부 파라미터 아래에서 다시 학습하게 했다.

`OnlineCalibrator::restore()`는 저장된 `valid_blocks`를 그대로 쓴다(openpilot
calibrationd와 같다). 5(`kInputsNeeded`) 미만이면 `rpy_rad`는 모델 입력 워프의
출발점일 뿐이고 상태는 `uncalibrated`라 결합이 막힌다. 새 블록 5개가 모여야
`calibrated`가 된다. 블록 하나는 채택 샘플 100개(`kBlockSize`)이고 채택은 약
24 km/h 이상 직진 주행에서만 일어나므로, 5블록은 조건을 만족하는 주행 25초
남짓에 해당한다.

파일을 아예 지우면 seed 없이 `rpy=[0,0,0]`에서 출발한다. 이 차량의 pitch가
-1.35도이므로 수렴 전까지 모델 입력 워프가 그만큼 틀어진다. 다시 학습시킬
때도 파일을 지우기보다 `rpy_rad`는 남기고 `valid_blocks`를 0으로 두는 편이
낫다.

카메라 마운트를 옮겼으면 웹 콘솔 `차량 특성` 탭의 **캘리브레이션 초기화**를
누른다. 해제(disengaged) 상태에서만 받으며(openpilot의 Reset Calibration과 같다),
`/dev/shm/edgepilot_calibration_reset`을 만들면 modeld가 1초 안에 이 파일을 지우고
보정기를 `rpy=[0,0,0]`, 0블록에서 다시 시작한다. 옛 장착의 값은 틀렸으므로 seed로
남기지 않는다. paramsd·torqued 학습값은 CAN 요레이트로 배우므로 그대로 둔다.
modeld가 안 떠 있으면 요청 파일은 남아 있다가 다음 시작 때 처리된다.
블록 사이 편차가 yaw 2°, pitch 4°를 넘으면 openpilot calibrationd와 같이 장착이
바뀐 것으로 보고 스스로 다시 보정한다: 마지막 블록 하나에서 다시 모으며 상태는
`recalibrating`(HUD `RECAL`)이고, 출력은 0.5초(10표본)에 걸쳐 새 값으로 옮겨 가며,
새 블록 5개가 차야 다시 보정 완료가 된다. 그 사이에는 저장하지 않는다. 그보다 작게
바뀐 장착은 옛 블록과 섞여 천천히 움직이므로 이 버튼으로 초기화한다.

내부 파라미터를 다시 측정해 옛 값을 seed로 남긴 채 다시 학습시키려면,
정차 상태에서 파이프라인을 멈추고 보드의 `params/calibration.json`을 백업한 뒤
`valid_blocks`를 0으로 바꿔 다시 쓰고 파이프라인을 시작한다. 실차에서는 충분한
속도로 직선에 가까운 도로를 주행하면서 카메라가 움직이지 않게 고정해야 한다.

## 권장 튜닝 순서

1. 카메라 장착을 고정하고 온라인 캘리브레이션을 완료한다.
2. `path_offset_m`로 차선 중심 위치를 먼저 맞춘다.
3. `angle_offset_deg`, `steer_ratio`, 차량 제원이 실제 차량과 맞는지 확인한다.
4. `torque_lat_accel_factor`, `torque_kp`, `torque_ki`, `torque_friction`을 한 항목씩 조정한다.
5. 토크가 정상적으로 추종된 뒤 `steer_actuator_delay`를 조정한다. 횡저크·횡가속
   상한은 런타임 항목이 아니므로 여기서 바꾸지 않는다(위 `경로 제한` 참고).
6. 마지막으로 운전자 토크 보호와 fault 회피 옵션을 검증한다.

각 단계에서 disengage 가능 여부, 운전자 개입 시 즉시 토크가 줄어드는지, CAN 오류와
MDPS fault가 없는지를 먼저 확인한다.
