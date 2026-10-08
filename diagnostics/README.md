# 진단 도구

런타임 경로 밖에 두는 분석·재생·벤치 도구다. 기본 빌드에는 들어가지 않고
`EDGEPILOT_BUILD_DIAGNOSTICS=ON`일 때 [`CMakeLists.txt`](CMakeLists.txt)가 만든다.
단위 테스트와 Python 검사(`check_*.py`)는 [`../tests/`](../tests/README.md)에 있다.

## 빌드

호스트 도구는 AX 런타임, MaixCAM2 SDK, OpenCV 없이 빌드된다:

```sh
cmake -S . -B build-host \
  -DCMAKE_BUILD_TYPE=Release \
  -DEDGEPILOT_BUILD_RUNTIME=OFF \
  -DEDGEPILOT_BUILD_DIAGNOSTICS=ON
cmake --build build-host --target replay_closed_loop -j2
```

보드용으로는 `tools/docker_ax630/`의 컨테이너에서 `build-ax630/`을 구성할 때
`-DEDGEPILOT_BUILD_DIAGNOSTICS=ON`을 더하면 같은 도구가 `build-ax630/bin`에 생긴다
(`build.sh`는 옵션을 받지 않으므로 컨테이너 안에서 직접 구성한다). 보드 전용 도구는 없다.

## 도구

호스트:

| 도구 | 사용 | 하는 일 |
| --- | --- | --- |
| `replay_closed_loop` | `[옵션] <out.csv\|-> <events.bin...>` | 차 응답을 시뮬레이션해 횡제어 루프를 폐루프로 재생한다 |
| `replay_planner` | `[--laneless\|--lane] [--exact] [--vehicle] [--steering s.json] <out.csv> <events.bin...>` | 녹화한 `ModelState`/`ControlState`로 `LateralPlanner`를 다시 돌려 요구 곡률을 CSV로 쓴다. `--vehicle`은 녹화한 깜빡이·운전자 토크·active를 주고, `--exact`는 모든 출력을 비트 그대로(`%a`) 써서 리팩토링 전후 비교에 쓴다 |
| `replay_controls` | `[--steering s.json] [--cruise c.json] [--vehicle-json j] [--force-engaged] [--dump out.txt] <events.bin...>` | 녹화를 controlsd 한 틱의 로직(`ControlsTick`)에 10 ms 틱으로 다시 흘려, 보낼 CAN·`ControlState`·학습 출력의 다이제스트를 쓰고 녹화된 `ControlState`와 대조한다. 같은 코드면 출력이 비트 단위로 같아 controlsd 리팩토링 전후 비교에 쓴다 |
| `replay_lateral_learners` | `[옵션] <events.bin...>` | paramsd·torqued 학습기를 녹화에 돌려 학습값을 출력한다 |
| `replay_localization` | `<out.csv> <events.bin...>` | 녹화를 보드 `locationd`와 같은 코드(자세 칼만 필터 + lagd)에 기록 순서대로 흘려 IMU 묶음마다 CSV 한 행을 쓰고, 끝에 CAN 대비 요레이트 비교와 lagd 결과를 출력한다. IMU 기록이 있는 녹화가 필요하다 |
| `extract_lateral_dataset` | `<out.csv> <events.bin...>` | `ControlState`마다 CSV 한 행. CAN은 런타임과 같은 `vehicle_can`으로 푼다 |
| `hud_snapshot` | `[--model m.bin] [--control c.bin] [--iterations N] [--out PREFIX] [--portrait [--flip-x] [--flip-y]]` | HUD 시나리오 27개(또는 녹화한 모델·제어 상태)를 `overlayd`와 같은 640x480으로 `EDGEARGB` 프레임에 그리고 그리기 시간을 출력한다. `--portrait`는 보드처럼 세로 480x640 버퍼에 transpose로 그린다. 시간은 보드에서 잰다 |
| `alert_sound_preview` | `[--out PREFIX]` | `overlayd` 알림음을 같은 합성 코드로 `PREFIX_<이름>.wav`(48 kHz 모노, 100% 크기)에 쓴다 |

재생 도구의 입력은 `recordd` 녹화(`events/*.bin`)다.

### replay_closed_loop

녹화가 본 차선 기하를 시뮬레이션 차가 벌어진 만큼(`dy`, `dpsi`) 차체 좌표로 다시 돌리므로,
제어를 바꾸면 차가 실제로 다르게 움직인다. 운전자 토크가 있거나 비활성인 틱은 녹화 상태로
재동기화해, 자유 주행 구간마다 실제 자세에서 출발한다.

- 플랜트: `--wn`, `--zeta`, `--delay`, `--gain G`(속도 노드 전부) 또는 `--gain-pts a,b,c,d`
- 컨트롤러: `--sad`(steer_actuator_delay), `--kp`, `--ki`, `--laf`
- 운전자 개입 히스테리시스: `--driver-high`, `--driver-low`, `--driver-release`
- `--open-loop`: 자세 보정을 멈춰 플랜트가 녹화 주행을 얼마나 재현하는지만 본다
- `--steering route/params/steering.json`: 녹화 당시 튜닝(경로 모드 포함). 녹화의 LearnerState
  학습값(paramsd·torqued)은 controlsd처럼 컨트롤러에 넣고, 조향각 역모델도 같은 학습값을 쓴다.
- `--camera-shift D`: 카메라 장착 오프셋을 녹화보다 D m 바꾼 것처럼 모델 출력을 옮긴다.
- `--torque F,O,R`: torqued 값(배율·절편·마찰)을 바꿔 넣어 유효해진 뒤의 거동을 본다.
- laneless는 차선이 위치를 잡아 주지 않아 폐루프가 발산한다(2 m 클램프). Lane 모드에만 쓴다.

재현 점수(전체와 속도 구간별)를 표준 출력으로 낸다. CSV가 필요 없으면 출력 경로에 `-`를
준다. 플랜트 식별과 점수의 해석은 [폐루프 재생](../docs/closed-loop-replay.md)에 있다.

### replay_lateral_learners

controlsd와 같은 `LateralLearners`를 부른다. 활성과 보낸 토크는 `ControlState`에서,
운전자 개입은 기록된 운전자 토크를 컨트롤러와 같게 디바운스해서 얻는다.

- `--steering route/params/steering.json`: 녹화 당시 튜닝(사전값, 지연, 입력 출처)을 쓴다.
  없으면 코드 기본값이라 보드와 다를 수 있다.
- `--vehicle-json route/params/live_parameters.json`: paramsd 저장값으로 시작한다.
- `--source steering|locationd|esp`: paramsd·torqued의 요레이트·롤 출처. 기본(`steering`)은
  `use_locationd_learner_inputs`를 따르고, controlsd처럼 locationd가 0.5초 넘게 낡거나 필터가
  무효면 ESP12로 돌아간다.
- `--metric-from N`: 끝에 내는 곡률 대조를 0부터 센 N번 파일부터 센다. 앞 파일은 학습을
  수렴시키는 데만 쓴다.
- `--upstream-schedule`: 조향각·속도를 상류처럼 20 Hz로만 관측한다(기본은 매 틱).
- `--fit-all`: torqued 적합에 점을 전부 쓴다(기본은 상류처럼 무작위 2000점).
- `--torque-cache c.bin`: 있으면 복원하고 저장 틱마다 덮어써, 여러 주행을 런타임 캐시처럼
  잇는다.
- `--inputs`/`--torque-inputs`는 틱마다의 입력을, `--outputs`/`--torque-outputs`는 발행
  메시지를 남긴다. 참조 구현과 대조할 때 쓴다.

## 도구 작성 규칙

- 파일 첫머리의 `/* */` 한글 주석에 용도를 적고 마지막 줄을 `사용:`으로 끝낸다.
- 입력 파일은 위치 인자로, 옵션은 `--이름 값` 또는 값 없는 `--이름`으로 받는다. 모르는
  옵션이 오면 사용법을 출력한다.
- 사용법 오류는 종료 코드 2, 입력을 못 읽거나 결과가 맞지 않는 실행 오류는 1로 끝낸다.

## 자세한 절차

[진단 절차](../docs/diagnostics.md)에 HUD 스냅샷, NV12 재생, 모델 교체 검증, 차선 치우침
분석, 데이터셋 추출, 플래너 재생의 절차와 결과가 있다. 녹화 포맷은
[분할 런타임](../docs/runtime.md#recording-format)에 있다.
