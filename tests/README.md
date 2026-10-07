# 호스트 단위 테스트

보드 없이 호스트에서 도는 googletest 테스트다. 파일 하나가 실행 파일 하나이고
(`gtest_<이름>.cc` → `build-host/bin/gtest_<이름>`), 교차 빌드에는 들어가지 않는다.

## 실행

```sh
./scripts/run_host_tests.sh
```

`build-host`를 구성하고 `host_tests` 타깃을 빌드한 뒤 `ctest`로 전부 돌린다(C++ 196개와
Python 2개). googletest v1.18.0은 첫 구성 때 받아 온다(SHA256 고정).

하나만 돌릴 때:

```sh
ctest --test-dir build-host -R LateralController.InactiveDesiredTracksActual --output-on-failure
build-host/bin/gtest_lateral_learners --gtest_filter='LateralLearners.Torque*'
```

테스트는 `params/`를 상대 경로로 읽으므로 저장소 루트에서 돌아야 한다. `ctest`는 루트에서
돌리고, 실행 파일을 직접 부를 때도 루트에서 부른다.

## 목록

| 실행 파일 | 개수 | 검사 내용 |
| --- | ---: | --- |
| `gtest_adaptive_cruise` | 15 | 비전 크루즈 버튼 간격과 한계. 차량 모형과 폐루프로 돌려 설정 속도 동기화, 재설정, 반응 없는 차, 오르내림 반복, 고정 앞차 확률(0.5)을 본다 |
| `gtest_alert_tones` | 1 | 알림음 합성: 모든 소리가 무음에서 시작해 무음으로 끝나고, 봉우리가 같고, 0.3~2.5초이며, 서로 다르다 |
| `gtest_background_writer` | 1 | 루프 밖 파일 쓰기 스레드: 같은 경로는 최신 내용, 지우기, 실패 수, flush는 넘긴 쓰기가 끝날 때까지 기다림, 없앨 때 남은 쓰기 마치기 |
| `gtest_calibration` | 12 | 온라인 보정 상태 기계(calibrationd.py 참조, 카메라 높이·높이 표준편차 조건 포함), 장착 변경·초기화 재보정, 범위 밖 저장값 복원, 저장·복원·수동 보정, 환경 변수, 투영 행렬과 YUV6 워프(openpilot OpenCL 참조), NV21 색차 순서, 카메라 장착 위치 |
| `gtest_car` | 12 | K7 CAN 신호 해석(LCA11, WHL_SPD11, TPMS11, TCS13/15, SCC11, CLU11 크루즈 버튼과 고정형 크루즈 설정 속도, MDPS12 고장 필터)과 MDPS용 CLU11 속도 바꿔치기, CGW1 B-CAN 타임아웃을 안전한 값으로 읽기(깜빡이·비상등 꺼짐, 문 열림, 안전벨트 미착용), AHB1 페달 스트로크(3 mm 초과)와 TCS13 BrakeLight(AUTO HOLD)로 켜는 브레이크등, AHB1 페달이 끄는 고정형 크루즈 추정(TCS13 0이 덮어쓰지 않음, 밟은 채 RES 무시) |
| `gtest_control_holds` | 3 | 조향 경로 게이트(plan 도달 거리와 점 수), Panda 헬스 공백 홀드(100 ms), 잘못된 plan 홀드(150 ms) |
| `gtest_controls_tick` | 4 | controlsd 한 틱(`ControlsTick`)을 main처럼 10 ms마다: 신호 배치대로 채운 K7 CAN·20 Hz 모델·Panda 상태로 SET 결합과 LKAS11 송신(파워트레인·MDPS 버스), 문 열림 해제, Panda 거부 1초 유예 뒤 거절, AHB1 페달이 끄는 크루즈 추정(횡제어는 유지), 휠 속도가 끊기면 0.5초 뒤 비우는 크루즈·발행 속도(조향은 설정한 시간까지) |
| `gtest_departure_alert` | 14 | 정차 중 앞차 출발과 신호 대기 알림(짧은 plan 1.5초 무장, 무장 전 열림이면 다시 재기, 정차 2.7초 만의 녹색, 가속 확률, 서 있는 가까운 앞차 뒤 경로 깜빡임 거르기, 깜빡이 대기 중 가속 확률 끄기, 앞차 끊김 허용), controlsd 주기(100 Hz 틱, 모델 5틱마다)의 녹색 확인, 모델 프레임 사이에서 판단하는 가까운 앞차 거부, 서행은 정차를 끝내지 않고 가속 페달은 끝냄 |
| `gtest_desire_helper` | 5 | 차선 변경·회전 desire 상태 머신(`DesireHelper`): 깜빡이 쪽으로 핸들을 밀어야 시작, 차선선 0.5초 페이드 아웃·인, 사각지대 대기, 10초 시한, 도로 경계 쪽 차단과 경계가 사라질 때의 시작, 회전 desire 2.5초 펄스 |
| `gtest_device_settings` | 3 | 웹 기기 설정(`display.json`) 읽기: 범위 클램프, 잘못된 값은 이전 값 유지, 파일이 바뀔 때만 다시 읽기 |
| `gtest_hud_canvas` | 10 | HUD 캔버스: 스트레이트 알파 합성(부동소수 기준식 대조), 다각형 커버리지 합 = 기하 넓이, 먼 쪽 흐림, 둥근 사각형 곧은 행(이음매 없음), 그린 칸만 지우기, 글자 폭, 세로 패널 버퍼 = 가로 그림의 전치. 렌더러의 상태 테두리와 알림 카드, 다시 받은 버퍼 = 새 버퍼, 늘 있는 카드(TPMS·보정·보드 상태·학습값), 오토 홀드 배지, 세 자리 속도에서도 카드에 닿지 않는 노란 깜빡이, 설정 속도 카드 안의 SET, 상태 알약·왼쪽 열 터치 영역 |
| `gtest_hud_policy` | 12 | overlayd의 그리기 밖 판단: 제어 이벤트 알림(기준값, 카운터 리셋, 우선순위), 프레임마다 알림음 하나(해제 예고는 미루고 불가용 천이는 넘김, engage 거부 토스트 3초), 깜빡이 단계, 터치로 여닫는 카드, 차선 위치 평활 |
| `gtest_hud_state` | 5 | 제어·녹화·학습기·locationd 상태 → HUD 매핑(차선 변경·회전·조향 쉼 플래그, 운전자 토크 눈금, 제어가 쓰는 학습값), 차선 안 위치, 모든 `BlockReason`의 라벨, 알림 카드 우선순위 |
| `gtest_ipc_channels` | 2 | 공유 메모리 CAN 큐, 헤더가 맞지 않는 최신값 채널에는 붙지 않는 구독 |
| `gtest_lateral_controller` | 26 | 횡제어기: engage 게이트와 홀드, 토크 한계와 MDPS 고장 회피(85도 위 토크 상한과 steer 요청 끄기, 정차 대기·85도 위 결합, 손을 뗐을 때 80도 상한, 운전자가 넘겨받은 회전은 15도까지 해제), 회전 desire 중 깜빡이 방향으로 돌리는 운전자를 밀지 않음, 곡률 제한, Panda 게이트와 넘겨받기, LKAS HUD, 학습값 소비(끄면 비트 동일, `paramsd_invalid`, 롤 반영 곡률 한계, lagd 지연), CAN 픽스처 재생 |
| `gtest_lateral_learners` | 18 | paramsd EKF(야코비안, Joseph 양정치, 수렴, 게이트, 출력 한계, 저장, 자이로 바이어스), torqued(TLS와 닫힌 해 대조, 버킷, 게이트, 필터·decay, 4 Hz/12 s 스케줄, 캐시), controlsd 연결 |
| `gtest_lateral_mpc` | 1 | 횡 MPC 최적성. 동역학과 코스트를 따로 구현해 시나리오 8개의 수렴점에서 기울기를 본다([검증 기록](../docs/verification.md#lateral-mpc-solver)) |
| `gtest_lateral_planner` | 6 | 횡 플래너: laneless(openpilot `get_curvature_from_plan`), 차선 변경(상류 desire_helper), 실험용 회전 desire, `path_offset_m` 적용 범위, `lane_path_weight`, Lane 모드의 차선 없는 구간이 laneless와 같은지와 인계 블렌드 |
| `gtest_lateral_torque` | 8 | 토크 횡제어기: 조향 지연만큼의 요청 버퍼, 속도별 이득(KP_INTERP), 라이브 뱅크와 latAccelOffset feedforward, 학습값 소비(opendbc `calc_curvature`, 상류 latAccelFactor 구조) |
| `gtest_localization` | 11 | locationd(PoseKalman, LocationEstimator: 정지 수렴, 요레이트 추적, 자이로 교차검증, 되감기, IMU 묶음 무관, 정차 속도 가드, 부팅, IMU 장착 기울기)와 lagd(상류 상관식 대조, 주행 지연 추정, 개입·저속 무시) |
| `gtest_model_output` | 6 | supercombo raw 출력 레이아웃과 시간축 입력 규약(desire 펄스·풀링과 깜빡이로 끝나는 회전 desire, 특징 이력, 이미지 이력) |
| `gtest_model_state_fill` | 2 | modeld가 채우고 overlayd가 되돌리는 `ModelState` 왕복(남는 값, t=0 lead만, road_transform 없음, 보정 칸)과 `compute_lane_t`(일정 속도면 거리/속도, 짧은 plan 끝 뒤 NaN, 뒤로 뛰는 knot에도 단조) |
| `gtest_panda_can_codec` | 1 | panda USB CAN 패킹·언패킹 |
| `gtest_panda_firmware` | 8 | 판다 펌웨어 이미지 검사(앱 영역, 길이 필드, VERS 꼬리, 버전 문자열), 주차 중에만 플래싱하는 조건, F413 보드만, 상태 JSON 이스케이프. pandad의 health 패킷·패킷 버전·USB ID·safety 번호·앱 시작 주소가 `firmware/panda` 소스와 같은지(`board/health.h`를 그대로 가져와 대조) |
| `gtest_projection` | 4 | 화면 크기와 무관한 도로 점 투영, 설정된 카메라 내부 파라미터, 카메라 장착 오프셋 |
| `gtest_recording` | 6 | 디스크의 route 구조: EDGELOG1 청크, EDGEIDX1 인덱스(v8 이하 녹화의 예전 매직도 읽기), 매니페스트, params 스냅샷, 스테이징 비우기. 이벤트 로그 리더(끊긴 꼬리에서 멈춤, 큰 파일 머리), 옛 버전 ModelState 해석, 기록한 CAN 페이로드 왕복(CAN-FD, 끊긴 꼬리, 256 프레임 상한), 상태 채널 기록(생산자가 생긴 뒤 붙기, 새 스냅샷만, IMU는 채운 샘플까지), replayd route 리더(세그먼트 경계, 키프레임마다 코덱 설정, 청크를 넘는 CAN·판다 이벤트 시각 순, 머리가 깨진 프레임 인덱스는 그 세그먼트만 건너뜀) |

`tests/check_web_console.py`와 `tests/check_recording_reader.py`(Python unittest)도
`ctest`에 등록돼 함께 돈다. 앞의 것은 fastapi 없이 stdlib만으로 웹 콘솔을 본다: 파라미터 저장소와
기본값 동기화, `params/*.json`의 UI 메타데이터(섹션, 범위), UI min/max와 C++ `Json*Field` 클램프 표의
일치, 공유 메모리 상태(학습, locationd, 매니저 프로세스 표, 카메라 보정, BEV)의 배치와 `ipc_messages.h`의
일치, 판다 펌웨어 카드가 설치된 이미지를 `panda_firmware.cc`와 같은 규칙으로 보고 주차 중일 때만 플래싱
요청 파일을 쓰는지, 페이지 모듈의 import와 페이지가 부르는 API가 서버에 다 있는지, 화면 문구에 차종
이름이 없는지. 뒤의 것은 `tools/model/recording_reader.py`의 배치와 기록 타입 번호, 도구의 MaixCAM2
카메라 내부 파라미터를 C++ 헤더의 고정값과 대조한다(numpy가 필요하고, 없으면 건너뛴다).

## 인자를 주면 도는 모드

- `gtest_lateral_controller <fixture.can>`: `LateralController.CanFixture`가 녹화한 CAN을
  컨트롤러에 흘려 목표 곡률을 openpilot 참조식과 대조하고, LKAS/MDPS/CLU11 주기와 토크
  범위를 본다. 인자가 없으면 건너뛴다. 픽스처는 `tools/control/export_can_fixture.py`가
  녹화 `events/NNN.bin` 하나로 만든다. 60초 연속 주행 구간이어야 하며(활성 5900틱 초과,
  토크 > 0) 정차 구간은 실패한다.
- `gtest_model_output <SCODMP1 덤프>`: 테스트 대신 첫 프레임을 파싱해 출력한다.
  덤프는 `modeld`를 `EDGEPILOT_RAW_DUMP`로 돌려 만든다
  ([런타임 옵션](../docs/runtime-options.md)).

## 테스트 추가

1. `tests/gtest_<이름>.cc` 하나에 테스트와 도우미를 모두 둔다. 공유 헤더는 두지 않는다.
2. [`CMakeLists.txt`](CMakeLists.txt)에 한 줄을 더한다:
   `add_host_test(gtest_<이름> <라이브러리...>)`. 라이브러리는 `src/<폴더>/CMakeLists.txt`가 정의하는
   `common`, `utils_json`, `car`, `control_core`, `planning`, `learners`, `controls`, `localization`,
   `model`, `panda`, `recording`, `hud_state`, `hud`, `alert_tones` 중에서 고른다.
3. 작성 규칙:
   - 파일 첫머리에 `/* */` 한글 주석으로 무엇을, 무엇과 대조해 검사하는지 적는다.
   - TEST와 도우미는 익명 namespace 안에 둔다.
   - 주석과 실패 메시지는 한글로 쓴다. 단언 묶음의 뜻은 위에 `//` 한 줄로, 단언 하나의
     뜻은 `<< "..."`로 붙인다. TEST 위 주석은 이름만으로 모자랄 때만 단다.
   - 실패할 때 값이 보이는 단언을 쓴다. `ASSERT_TRUE(a < b)` 대신 `ASSERT_LT(a, b)`,
     `ASSERT_TRUE(std::fabs(a - b) < t)` 대신 `ASSERT_NEAR(a, b, t)`.
   - 값을 돌려주는 도우미에서는 `ASSERT_*`를 못 쓰니 `ADD_FAILURE() << ...; return {};`로
     실패를 남긴다.
   - 통과할 때는 아무것도 출력하지 않는다. 판정 없는 시간 측정은
     [`diagnostics/`](../diagnostics/README.md)의 벤치로 만든다.
