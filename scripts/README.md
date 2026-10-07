# 스크립트

호스트에서 빌드·배포·검사에 쓰는 스크립트와, 보드에 설치돼 런타임과 함께 도는 Python이 있다.
보드용 파일은 `upload_to_board.sh`가 보드 설치 디렉터리(`/root/edgepilot`)의 최상위에 실행 파일과
나란히 둔다.

## 호스트

| 스크립트 | 사용 | 하는 일 |
| --- | --- | --- |
| `fetch_maixcam2_sdk.sh` | `[root@보드]` | MaixCAM2 교차 빌드 의존성을 `deps/ax630/`에 받는다. MSP SDK `v3.0.0_20250319114413`(SHA256 고정)과 MaixCDK 헤더(커밋 `30f4b8b`)는 네트워크에서, 보드의 `/opt/lib`, `libmaixcam_lib.so.1.2.5`, `libsamplerate`, OpenCV 4.11은 보드에서 SSH로 그대로 복사한다 |
| `upload_to_board.sh` | `[root@보드]` | `build-ax630/bin`의 런타임, 보드용 Python, 파라미터 기본값, 모델(`models/supercombo.axmodel`, 바뀌었을 때만), Panda 펌웨어 이미지(빌드돼 있으면 `firmware/panda.bin.signed`로)를 보드에 올린다. 보드의 `/usr/lib/libmaixcam_lib.so`가 빌드에 쓴 1.2.5와 다르면 1.2.5를 올리고 링크를 바꾼다(순정 파일은 `.stock`) |
| `install_autostart.sh` | `[--remove] [root@보드]` | `edgepilot.service`(부팅 때 매니저 실행, 순정 런처 대신), `camcal.service`, `wifi-dhcp-renew.service`를 설치하고 `/boot/configs`에 `maix_npu_ai_isp=1`을 넣는다 |
| `install_boot_tuning.sh` | `[--remove] [root@보드]` | 부팅을 빠르게 한다: AX 드라이버를 부팅 초반에 올리는 `edgepilot-drivers.service`를 설치하고, 쓰지 않는 순정 서비스를 끄고, journal 크기를 묶는다. 다음 부팅부터 적용된다([부팅 시간](../docs/boot-time.md)) |
| `run_host_tests.sh` | | 호스트 단위 테스트를 빌드하고 `ctest`로 전부 돌린다([tests/](../tests/README.md)). 보드도 `deps/`도 필요 없다 |

빌드 자체는 `tools/docker_ax630/build.sh`가 arm64 Ubuntu 22.04 컨테이너에서 한다(결과는
`build-ax630/bin`, Panda 펌웨어는 `build-ax630/panda/obj`). 보드 기본 주소는 `root@192.168.219.117`이고 SSH 키 인증을 쓴다.

환경 변수로 바꿀 수 있는 것:

- `upload_to_board.sh`: `models/supercombo.axmodel`은 보드 것과 체크섬이 다를 때만 보낸다. `EDGEPILOT_BIN_DIR`
  (기본 `build-ax630/bin`), `EDGEPILOT_BOARD_DIR`(기본 `/root/edgepilot`), `EDGEPILOT_PANDA_IMAGE`(기본
  `build-ax630/panda/obj/panda.bin.signed`). 실행 중인 바이너리는
  덮어쓸 수 없으므로 `.upload/`에 올린 뒤 `mv`로 바꾼다. 매니저는 다시 띄우지 않는다. 보드의
  `params/`는 덮어쓰지 않고, 기본값은 `params.defaults/`에 두어 없는 파일만 채운다.
- `run_host_tests.sh`: `EDGEPILOT_HOST_BUILD_DIR`(기본 `build-host`), `JOBS`.

자세한 빌드와 배포 절차는 [Build and deploy](../docs/build-and-deploy.md)에 있다.

## 보드

| 파일 | 사용 | 하는 일 |
| --- | --- | --- |
| `manager.py` | `python3 /root/edgepilot/manager.py [supercombo.axmodel]` | 런타임 감시자. 보드 UI 런처를 멈추고 프로세스를 순서대로 띄우며 죽으면 1초 뒤 다시 띄운다. 부팅 때는 `edgepilot.service`가 띄운다 |
| `web_console/` | `python3 -m web_console [--host 주소] [--port 포트]` | 웹 콘솔(FastAPI, 기본 `0.0.0.0:8080`). 매니저가 함께 띄운다. 파라미터 편집, 차량 특성(학습값과 수동값), BEV, 백라이트(`backlight.py`)를 맡고 페이지는 `static/`에 있다. 기기 설정 탭의 판다 펌웨어 카드에서 판다를 플래싱한다(주차 중에만, 쓰는 것은 `pandad`) |
| `web_console/static/` | (정적 파일) | 웹 콘솔 페이지(빌드 없는 ES 모듈): `index.html`, `console.css`, `console.js`(머리 줄·탭), `ui.js`(공용 도구), 파라미터 탭(`param_state.js`, `param_editor.js`, `param_view.js`), 차량 특성 탭(`vehicle_view.js`), 판다 카드(`panda_card.js`), BEV 탭(`bev_view.js`, `bev.js`, 자차 모델 `bev_ego.js`, 앞차 모델 `bev_car.js`, `bev_data.js`)과 three.js 0.186.1(`three/`) |
| `web_console/backlight.py` | (모듈) | LCD 백라이트 제어(PWM3 `pwmchip0/pwm3`, 10 kHz, duty = 밝기 x `/boot/board`의 `disp_max_backlight`). 웹 콘솔이 시작할 때와 `display.json`을 바꿀 때 적용한다 |
| `edgepilot.service` | `install_autostart.sh`가 설치 | 부팅 때 매니저를 띄우는 systemd 유닛(`Conflicts=launcher.service`, `Restart=always`). 자식 로그는 `/run/edgepilot/<이름>.log` |
| `edgepilot-drivers.service` | `install_boot_tuning.sh`가 설치 | AX 미디어 드라이버를 부팅 초반에 올린다. 순정 이미지는 Wi-Fi 연결을 기다리는 `rc.local`에서 올린다 |
| `camcal.service` | `systemctl start camcal` | 카메라 내부 파라미터 측정 캡처([camcal](../docs/camcal.md)). 런타임을 멈추고 돈다 |
| `wifi-dhcp-renew.service`, `wifi-dhcp-renew.sh` | `install_autostart.sh`가 설치 | Wi-Fi가 다른 AP에 다시 붙을 때마다 wlan0 DHCP 임대를 새로 받는다 |
| `requirements-web-console.txt` | `python3 -m pip install -r ...` | 웹 콘솔 의존성(fastapi, uvicorn) |

어떤 프로세스를 띄울지와 환경 변수는 [분할 런타임](../docs/runtime.md)과
[런타임 옵션](../docs/runtime-options.md)에 있다.

## 작성 규칙

- 첫머리 주석(Python은 모듈 docstring)에 용도를 한글로 적고 `사용:` 줄로 끝낸다.
- 셸 스크립트는 `set -euo pipefail`로 시작하고, 저장소 루트를 스스로 찾아 어디서 불러도
  돌게 한다. 보드에서도 도는 스크립트는 POSIX `sh`로 쓴다.
