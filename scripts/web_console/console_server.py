#!/usr/bin/env python3
"""파라미터 편집 웹 서버(FastAPI). params/의 JSON 설정을 보여 주고 고쳐 쓰며(바꾸면 controlsd에 SIGHUP을
보내 다시 읽게 한다), /dev/shm 채널을 읽어 학습값·추이·온라인 보정 초기화와 BEV 탭을 낸다. 매니저가
함께 띄운다.

사용: python3 -m web_console [--host 주소] [--port 포트]   (기본 0.0.0.0:8080,
EDGEPILOT_WEB_CONSOLE_HOST·EDGEPILOT_WEB_CONSOLE_PORT로도 바꿀 수 있다)
"""
from __future__ import annotations

import argparse
import asyncio
import collections
import gzip
import json
import math
import mmap
import os
import re
import signal
import stat
import struct
import tempfile
import threading
import time
from pathlib import Path
from typing import Any, Callable, Dict

# fastapi/uvicorn은 서버를 띄울 때만 import한다. tests/check_web_console.py가
# stdlib만으로 ParamStore와 PARAM_METADATA를 쓴다. 요청 모델만은 모듈 전역에
# 있어야 한다: `from __future__ import annotations` 때문에 FastAPI가 라우트의
# 문자열 애너테이션을 모듈 전역에서 해석하므로, 지역 클래스면 NameError로 죽는다.
try:
    from pydantic import BaseModel
except ImportError:  # 호스트 검사: 웹 스택 없이 import된다
    BaseModel = object  # type: ignore[assignment,misc]


class ParamPatch(BaseModel):  # type: ignore[misc,valid-type]
    values: Dict[str, Any]


class PandaFlashRequest(BaseModel):  # type: ignore[misc,valid-type]
    version: str


from .backlight import DisplayBacklight


CONTROLSD_NAME = "controlsd"
RECORDD_NAME = "recordd"
# 파라미터 파일 경로는 EDGEPILOT_PARAMS_DIR 하나로만 바꾼다. 파일별 override는
# 그 디렉터리 설정과 중복이라 없앴다.
GROUP_FILES = {
    "steering": "steering.json",
    "driving": "driving.json",
    "adaptive_cruise": "adaptive_cruise.json",
    "recording": "recording.json",
    "display": "display.json",
}


def param_meta(
    label: str,
    section: str,
    unit: str,
    step: float,
    minimum: float,
    maximum: float,
    description: str,
    increase: str,
    decrease: str,
    *,
    control: str | None = None,
) -> Dict[str, Any]:
    metadata = {
        "label": label,
        "section": section,
        "unit": unit,
        "step": step,
        "min": minimum,
        "max": maximum,
        "description": description,
        "increase": increase,
        "decrease": decrease,
    }
    if control is not None:
        metadata["control"] = control
    return metadata


PARAM_METADATA: Dict[str, Dict[str, Dict[str, Any]]] = {
    "steering": {
        "enabled": {
            "label": "자동 조향",
            "section": "기본 토크 제한",
            "description": "K7 조향 컨트롤러 전체를 켜거나 끕니다.",
            "increase": "켜면 모델 경로를 따라 조향 토크를 생성합니다.",
            "decrease": "끄면 조향 토크를 생성하지 않습니다.",
        },
        "steering_pressed_threshold": param_meta(
            "운전자 조향 감지값", "운전자 개입", "MDPS raw", 10, 0, 500,
            "PID 적분을 멈추는 운전자 조향 토크 기준입니다.",
            "더 강하게 핸들을 잡아야 운전자 개입으로 판단합니다.",
            "작은 핸들 입력도 더 빨리 운전자 개입으로 판단합니다.",
        ),
        "torque_lat_accel_factor": param_meta(
            "배율 latAccelFactor", "토크 컨트롤러", "m/s²", 0.01, 0.5, 5.0,
            "정규화 토크 1.0이 내는 횡가속도입니다(openpilot latAccelFactor). "
            "선행·비례·적분 토크가 모두 이 값으로 나뉩니다. torqued 학습값이나 "
            "fit_lateral_params.py fit 결과를 넣습니다.",
            "같은 목표에 토크가 작아져 추종이 약해지고 언더스티어가 늘어납니다.",
            "같은 목표에 토크가 커져 추종이 강해지고 포화 여유가 줄어듭니다.",
        ),
        "torque_kp": param_meta(
            "비례 이득 Kp", "토크 컨트롤러", "gain", 0.05, 0, 10,
            "횡가속도 오차에 반응하는 비례 이득입니다(openpilot KP). 저속에서는 "
            "openpilot의 속도별 곡선이 이 값을 대신하고, 30 m/s 위에서만 이 값이 "
            "그대로 쓰입니다(상류 기본 0.8).",
            "경로 오차를 더 빠르고 강하게 보정하지만 흔들림이 늘 수 있습니다.",
            "반응이 부드러워지지만 경로 오차 회복이 느려질 수 있습니다.",
        ),
        "torque_ki": param_meta(
            "적분 이득 Ki", "토크 컨트롤러", "gain", 0.01, 0, 2,
            "지속되는 횡가속도 오차를 누적해 없애는 적분 이득입니다"
            "(openpilot KI, 상류 기본 0.15).",
            "지속 오차를 빨리 없애지만 오버슈트가 늘 수 있습니다.",
            "누적 보정이 느려져 일정한 편향이 오래 남을 수 있습니다.",
        ),
        "torque_friction": param_meta(
            "조향 마찰 보상", "토크 컨트롤러", "정규화 토크", 0.005, 0, 0.3,
            "조향계 마찰을 넘기 위해 오차 방향으로 더하는 토크입니다(openpilot friction).",
            "작은 커브에도 핸들이 더 즉각 움직이지만 좌우 튐이 생길 수 있습니다.",
            "미세 조향이 부드러워지지만 dead zone이 커질 수 있습니다.",
        ),
        "torque_use_angle": {
            "label": "조향각 기반 곡률",
            "section": "토크 컨트롤러",
            "description": "실제 곡률 계산에 조향각 센서를 우선 사용합니다.",
            "increase": "켜면 조향각 기반 곡률을 사용합니다.",
            "decrease": "끄면 유효한 ESP yaw-rate와 속도별로 혼합합니다.",
        },
        "torque_output_sign": param_meta(
            "토크 출력 방향", "고정 차량 설정", "부호", 2, -1, 1,
            "K7 YG HEV의 조향 토크 방향입니다. 정상값은 -1입니다.",
            "1로 바뀌면 조향 방향이 반전됩니다.",
            "-1로 바뀌면 K7 기준 정상 조향 방향이 됩니다.",
        ),
        "steer_ratio": param_meta(
            "조향비", "차량 모델", "ratio", 0.1, 8, 25,
            "핸들 조향각과 전륜 조향각 사이의 차량 조향비입니다.",
            "같은 핸들 각도를 더 작은 전륜 조향으로 추정합니다.",
            "같은 핸들 각도를 더 큰 전륜 조향으로 추정합니다.",
        ),
        "tire_stiffness_factor": param_meta(
            "타이어 횡강성", "차량 모델", "배", 0.05, 0.2, 2,
            "차량 모델의 기준 타이어 횡강성에 곱하는 보정값입니다.",
            "타이어가 횡력에 더 단단하게 반응한다고 계산합니다.",
            "타이어가 더 유연하게 반응한다고 계산합니다.",
        ),
        "steer_actuator_delay": param_meta(
            "조향 반응 지연", "조향 반응", "초", 0.01, 0.01, 1,
            "현재 조향 명령이 차량에 반영되기까지의 예측 지연입니다.",
            "경로를 더 앞에서 읽어 커브 진입을 선행하지만 과하면 오버슈트할 수 있습니다.",
            "조향 선행량이 줄어 커브 반응이 늦어질 수 있습니다.",
        ),
        "avoid_lkas_fault_enabled": {
            "label": "LKAS fault 회피",
            "section": "LKAS fault 보호",
            "description": "큰 조향각에서 MDPS fault 전에 토크를 0으로 내리고, 85도 아래로 돌아올 때까지 "
            "steer request를 끕니다. 운전자가 넘겨받아 돌린 회전이면 핸들이 15도 아래로 오고 손을 뗄 때까지 "
            "끈 채로 둡니다.",
            "increase": "켜면 85도 위에서도 fault 전까지 토크를 유지하다 request를 끕니다.",
            "decrease": "끄면 큰 조향각에서도 request를 계속 유지해 약 1초 뒤 MDPS fault가 납니다.",
        },
        "avoid_lkas_fault_max_angle_deg": param_meta(
            "Fault 감시 조향각", "LKAS fault 보호", "°", 1, 1, 180,
            "LKAS fault 회피 카운터를 세는 절대 조향각입니다.",
            "더 큰 핸들 각도에서 회피 동작을 시작합니다.",
            "더 작은 핸들 각도부터 회피 동작을 준비합니다.",
        ),
        "avoid_lkas_fault_max_frames": param_meta(
            "Fault 허용 프레임", "LKAS fault 보호", "frame", 1, 0, 300,
            "큰 조향각에서 steer request를 유지하는 최대 100 Hz 프레임 수입니다. 이 프레임에 토크가 "
            "0에 닿도록 미리 내려오고, 그 뒤 85도 아래로 올 때까지 request를 끕니다.",
            "토크와 request를 더 오래 유지합니다(실측 fault 하한 98프레임).",
            "더 일찍 내려와 request를 끕니다.",
        ),
        "avoid_lkas_fault_hold_angle_deg": param_meta(
            "스스로 가는 최대 조향각", "LKAS fault 보호", "°", 1, 0, 180,
            "운전자가 핸들을 잡지 않았을 때 시스템이 스스로 돌리는 최대 핸들 각도입니다. 85도 fault "
            "각도 아래에서 멈춰 토크를 끊김 없이 유지합니다. 운전자가 조향 중이면 적용하지 않습니다. "
            "0이면 끕니다.",
            "시스템이 혼자 더 급하게 돌지만, 85도에 가까우면 넘어가 토크가 끊길 수 있습니다.",
            "시스템이 혼자 도는 반경이 넓어지고, 더 일찍 운전자 조향이 필요합니다.",
        ),
        "avoid_lkas_fault_cut_frames": param_meta(
            "Fault 컷 길이", "LKAS fault 보호", "frame", 1, 1, 100,
            "fault 회피를 끈 경우에만 쓰는 값으로, MDPS 오류가 이어질 때 steer request를 끊는 "
            "100 Hz 프레임 수입니다.",
            "request를 더 오래 끊습니다.",
            "request를 더 짧게 끊습니다.",
        ),
        "live_bank_compensation": {
            "label": "실시간 편경사 보정",
            "section": "차량 중심 보정",
            "description": "ESP12 실측 횡가속으로 추정한 도로 편경사를 FF에서 보정합니다.",
            "increase": "켜면 커브별 편경사까지 실시간 보정합니다.",
            "decrease": "끄면 상수 offset만 사용합니다.",
        },
        "use_live_vehicle_params": {
            "label": "paramsd 학습값 사용",
            "section": "실시간 학습",
            "description": "주행 중 학습한 조향비·타이어 강성·조향각 영점·도로 롤을 "
            "차량 모델과 feed-forward에 씁니다(openpilot paramsd). 끄면 계산·기록만 합니다.",
            "increase": "켜면 학습값을 쓰고 롤 보정이 실시간 편경사 보정을 대신합니다.",
            "decrease": "끄면 조향 탭의 수동 차량 값을 씁니다.",
        },
        "use_live_torque_params": {
            "label": "torqued 학습값 사용",
            "section": "실시간 학습",
            "description": "주행 중 학습한 토크→횡가속 배율·편향·마찰을 토크 컨트롤러에 "
            "씁니다(openpilot torqued). 사전값 대비 배율 ±30%, 마찰 ±50% 안에서 움직입니다.",
            "increase": "켜면 학습값을 씁니다.",
            "decrease": "끄면 조향 탭의 수동 토크 값을 씁니다.",
        },
        "use_live_delay": {
            "label": "lagd 조향 지연 사용",
            "section": "실시간 학습",
            "description": "주행 중 추정한 조향 지연(목표 곡률 → 실제 요레이트, openpilot lagd)을 "
            "목표 곡률을 읽는 경로 지연과 토크 컨트롤러·torqued의 지연에 씁니다(openpilot과 같음). "
            "추정이 확정(5블록)된 뒤에만 적용되고, 그 전에는 steer_actuator_delay를 씁니다.",
            "increase": "켜면 확정된 추정 지연을 씁니다(길수록 커브를 일찍 꺾습니다).",
            "decrease": "끄면 steer_actuator_delay를 씁니다.",
        },
        "use_locationd_learner_inputs": {
            "label": "학습 입력을 locationd로",
            "section": "실시간 학습",
            "description": "paramsd·torqued가 요레이트·도로 롤을 locationd(IMU·카메라 융합)에서 받습니다"
            "(openpilot과 같음). 끄면 ESP12 요레이트(자체 바이어스 추정)와 ESP12 횡가속으로 구한 롤을 씁니다. "
            "locationd가 없거나 끊기거나 그 틱의 자세가 무효면 ESP12로 대신합니다. torqued는 시작할 때의 "
            "출처로만 점을 모으고, 출처가 바뀐 캐시는 배율·마찰만 이어 쓰고 점·절편은 처음부터 다시 모읍니다"
            "(유효해지기까지 몇 시간).",
            "increase": "켜면 조향각 영점·롤은 locationd 기준으로 몇 분 안에, torqued는 다음 시작부터 다시 수렴합니다.",
            "decrease": "끄면 ESP12 기준으로 학습합니다(torqued는 다음 시작부터).",
        },
        "torque_lat_accel_offset": param_meta(
            "횡가속 편향 보정", "차량 중심 보정", "m/s²", 0.01, -1.0, 1.0,
            "장착 롤 오차 등이 만드는 상수 횡가속 편향을 feed-forward에서 "
            "뺍니다. fit_lateral_params.py fit의 latAccelOffset을 그대로 넣습니다.",
            "차가 오른쪽으로 쏠릴 때 키우는 방향입니다.",
            "차가 왼쪽으로 쏠릴 때 줄이는 방향입니다.",
        ),
        "angle_offset_deg": param_meta(
            "직진 조향각 오프셋", "차량 중심 보정", "°", 0.1, -10, 10,
            "직진 상태의 조향각 센서 편차를 실제 곡률 계산 전에 뺍니다.",
            "현재 센서 각도를 더 작게 보정합니다.",
            "현재 센서 각도를 더 크게 보정합니다.",
        ),
        "mass_kg": param_meta(
            "차량 질량", "차량 모델", "kg", 10, 1000, 2600,
            "차량 모델과 타이어 횡강성 계산에 사용하는 질량입니다.",
            "차량이 더 무겁다고 계산합니다.",
            "차량이 더 가볍다고 계산합니다.",
        ),
        "wheelbase_m": param_meta(
            "축거", "차량 모델", "m", 0.01, 2, 3.5,
            "전륜과 후륜 사이 거리입니다.",
            "같은 곡률에 더 큰 조향각이 필요하다고 계산합니다.",
            "같은 곡률에 더 작은 조향각이 필요하다고 계산합니다.",
        ),
        "center_to_front_ratio": param_meta(
            "전축 무게중심 비율", "차량 모델", "ratio", 0.01, 0.2, 0.7,
            "무게중심에서 전축까지 거리를 축거 비율로 나타냅니다.",
            "무게중심을 후방 쪽으로 계산합니다.",
            "무게중심을 전방 쪽으로 계산합니다.",
        ),
        "steer_ratio_rear": param_meta(
            "후륜 조향비", "고정 차량 설정", "ratio", 0.01, -0.5, 0.5,
            "후륜 조향 차량의 곡률 보정값입니다. K7 정상값은 0입니다.",
            "후륜 조향의 양의 보정량이 커집니다.",
            "후륜 조향의 음의 보정량이 커집니다.",
        ),
        "path_offset_m": param_meta(
            "주행 경로 좌우 보정", "차량 중심 보정", "m", 0.01, -1, 1,
            "차선 중심 경로를 좌우로 평행 이동합니다. 차선이 안 보여 모델 경로를 따를 때는 적용하지 않습니다.",
            "목표 주행 위치가 차량 기준 오른쪽으로 이동합니다.",
            "목표 주행 위치가 차량 기준 왼쪽으로 이동합니다.",
        ),
        "lane_path_weight": param_meta(
            "Lane 모드 차선 중앙 유지 강도", "차량 중심 보정", "weight", 0.1, 0.5, 10,
            "Lane 모드 경로 계획(MPC)이 차선 중앙에서 벗어난 위치를 얼마나 강하게 되돌릴지 정합니다. "
            "openpilot 원본은 1입니다. 3이면 커브에서 안쪽으로 붙는 정도가 줄고 좌우 움직임이 조금 늘어납니다. "
            "Laneless 모드에는 영향이 없습니다.",
            "차선 중앙으로 더 강하게 돌아가고, 조향이 조금 더 잦아집니다.",
            "더 부드럽지만 커브에서 차선 중앙에서 더 벗어날 수 있습니다.",
        ),
        "min_steer_speed_mps": param_meta(
            "최소 자동 조향 속도", "기본 토크 제한", "m/s", 0.1, 0, 5,
            "이 속도 미만에서 토크 컨트롤러 출력을 0으로 만듭니다.",
            "자동 조향이 시작되는 최소 속도가 높아집니다.",
            "더 낮은 속도에서도 자동 조향을 허용합니다.",
        ),
    },
    "driving": {
        "laneless_mode": {
            "label": "Laneless 모드",
            "section": "경로 모드",
            "description": "차선 융합을 끄고 모델이 낸 주행 경로만 따라갑니다. "
            "끄면 차선이 뚜렷할 때 차선 중심으로 붙는 Lane 모드입니다.",
            "increase": "켜면 차선이 보여도 모델 경로만 따라가고 HUD에 LANELESS로 표시됩니다.",
            "decrease": "끄면 차선 확률이 높을 때 차선 중심 경로를 섞는 Lane 모드로 돌아갑니다.",
        },
        "turn_desire": {
            "label": "교차로 회전 desire (실험)",
            "section": "경로 모드",
            "description": "결합 중 차선 변경 최소 속도보다 느릴 때 깜빡이를 켜면 모델에 좌·우회전 의도를 "
            "알려 회전 경로를 잡게 합니다(openpilot에 없는 기능). 2.5초마다 다시 알리고, 그동안은 "
            "Lane 모드라도 모델 경로를 따릅니다. 깜빡이를 끄면 모델 입력에서도 지워 바로 풀리고, 속도가 "
            "오르면 새로 알리지 않습니다(이미 알린 의도는 모델에 최대 5초 남습니다). 빠를 때 켠 깜빡이도 "
            "켜 둔 채 그 속도 아래로 감속하면 회전으로 알립니다(진행 중인 차선 변경이 먼저입니다). 그래서 "
            "저속 차선 변경·갓길 정차나, 차선 변경 뒤 깜빡이를 켠 채 정체로 감속할 때도 회전으로 알리니 "
            "그때는 깜빡이를 끄거나 직접 조향하세요. 회전 중 깜빡이 방향으로 핸들을 돌리면 시스템은 반대 "
            "방향 토크를 내지 않습니다.",
            "increase": "켜면 저속 깜빡이에서 모델이 회전 경로를 따릅니다. 처음엔 한적한 교차로에서 시험하세요.",
            "decrease": "끄면 교차로 회전은 운전자가 합니다(openpilot과 같음).",
        },
        "model_timeout_ms": param_meta(
            "모델 경로 유효 시간", "데이터 상태", "ms", 50, 50, 2000,
            "마지막 모델 경로를 유효하다고 인정하는 최대 시간입니다.",
            "모델 갱신이 늦어도 기존 경로를 더 오래 사용합니다.",
            "모델 정지 시 더 빨리 조향을 차단합니다.",
        ),
        "vehicle_state_timeout_ms": param_meta(
            "차량 상태 유효 시간", "데이터 상태", "ms", 50, 50, 2000,
            "CAN 차량 상태와 yaw-rate를 유효하다고 인정하는 최대 시간입니다.",
            "CAN 지연을 더 오래 허용하지만 오래된 상태를 쓸 수 있습니다.",
            "CAN 갱신이 멈추면 더 빨리 제어를 차단합니다.",
        ),
        "inactive_release_ms": param_meta(
            "Disengage 토크 해제 시간", "상태와 CAN", "ms", 100, 0, 5000,
            "Disengage 후 순정 LKAS에 넘기기 전 0 토크 프레임을 유지하는 시간입니다.",
            "0 토크 handoff를 더 오래 유지합니다.",
            "순정 LKAS로 더 빨리 제어권을 넘깁니다.",
        ),
        "mdps_speed_spoof_kph": param_meta(
            "MDPS 위조 속도", "고정 차량 설정", "km/h", 1, 30, 100,
            "K7 MDPS가 저속에서도 LKAS 조향을 허용하도록 보내는 속도입니다.",
            "MDPS에 더 높은 차량 속도로 보냅니다.",
            "MDPS에 더 낮은 차량 속도로 보냅니다. K7 기준은 60 km/h입니다.",
        ),
        "lane_change_min_speed_kph": param_meta(
            "차선 변경 최소 속도", "운전자 개입", "km/h", 1, 0, 80,
            "이 속도 미만에서는 방향지시등을 켜도 차선 변경을 시작하지 않습니다.",
            "더 빠른 속도에서만 차선 변경을 시작합니다.",
            "더 낮은 속도에서도 차선 변경을 시작합니다.",
        ),
    },
    "adaptive_cruise": {
        "enabled": {
            "label": "비전 크루즈",
            "section": "동작",
            "description": "비전 선행차를 기준으로 순정 크루즈의 SET-/RES+ 버튼을 자동 조절합니다.",
            "increase": "켜면 최초 SET 속도를 상한으로 비전 기반 속도 조절을 시작합니다.",
            "decrease": "끄면 자동 버튼 명령을 중지하고 현재 순정 크루즈 설정에 개입하지 않습니다.",
        },
        "following_time_s": param_meta(
            "주행 차간시간", "차간 거리", "초", 0.1, 0.8, 4.0,
            "현재 속도에 곱해 선행차와 유지할 동적 거리를 계산합니다.",
            "속도에 비례한 차간거리가 늘어 더 일찍 감속합니다.",
            "차간거리가 짧아지고 선행차에 더 가깝게 주행합니다.",
        ),
        "standstill_gap_m": param_meta(
            "기본 차간거리", "차간 거리", "m", 0.5, 2.0, 20.0,
            "속도와 무관하게 목표 차간거리에 더하는 기본 거리입니다.",
            "모든 속도에서 선행차와 더 멀리 떨어집니다.",
            "모든 속도에서 선행차와 더 가까워집니다.",
        ),
        "gap_correction_gain": param_meta(
            "거리 오차 반응", "속도 반응", "gain", 0.05, 0.05, 1.0,
            "실제 거리와 목표 거리의 차이를 목표 속도 보정으로 바꾸는 비율입니다.",
            "차간거리 변화에 더 빠르고 크게 반응하지만 속도 변동이 늘 수 있습니다.",
            "반응이 부드러워지지만 가까워지는 차량에 늦게 대응할 수 있습니다.",
        ),
        "max_slowdown_correction_mps": param_meta(
            "최대 감속 보정", "속도 반응", "m/s", 0.5, 0.5, 10.0,
            "선행차가 가깝거나 느릴 때 목표 속도를 낮추는 최대 보정량입니다.",
            "더 낮은 크루즈 설정을 요청해 감속 반응이 적극적이 됩니다.",
            "목표 속도 감소 폭이 작아져 감속 반응이 완만해집니다.",
        ),
        "max_speedup_correction_mps": param_meta(
            "최대 가속 보정", "속도 반응", "m/s", 0.5, 0.0, 5.0,
            "차간거리가 충분할 때 선행차 속도보다 높게 잡을 수 있는 최대 보정량입니다.",
            "저장된 최대 속도로 더 빠르게 복귀할 수 있습니다.",
            "속도 복귀가 보수적이고 느려집니다.",
        ),
        "deceleration_rate_kph_per_s": param_meta(
            "실측 감속 응답", "속도 반응", "km/h/s", 0.1, 0.5, 5.0,
            "SET- 명령 뒤 실제 차량 속도가 1초 동안 감소하는 실측값입니다. 다음 감속 명령 간격과 예상 차간거리 계산에 사용합니다.",
            "차량이 더 빨리 감속한다고 판단해 다음 SET-를 더 일찍 허용합니다.",
            "SET- 효과를 더 오래 기다리고 선행차와 가까워질 거리를 더 멀리 예측합니다.",
        ),
        "lead_restore_delay_s": param_meta(
            "선행차 소실 후 복귀", "복귀 동작", "초", 0.5, 1.0, 10.0,
            "선행차가 사라진 뒤 저장된 최대 속도로 복귀하기 전 기다리는 시간입니다.",
            "비전 검출이 끊겼을 때 현재 속도를 더 오래 유지합니다.",
            "선행차가 사라지면 최대 속도로 더 빨리 복귀합니다.",
        ),
        "command_interval_s": param_meta(
            "속도 변경 명령 간격", "버튼 송신", "초", 0.1, 0.5, 5.0,
            "연속 SET-/RES+ 버튼 펄스를 시작할 수 있는 최소 시간 간격입니다.",
            "크루즈 설정 속도가 더 천천히 변합니다.",
            "크루즈 설정 속도가 더 빠르게 변하지만 잦은 버튼 명령이 발생합니다.",
        ),
        "lead_hold_s": param_meta(
            "선행차 유지 시간", "비전 판정", "초", 0.1, 0.1, 2.0,
            "일시적으로 비전 lead가 끊겨도 마지막 선행차를 유효하게 유지하는 시간입니다.",
            "짧은 검출 누락에 덜 흔들리지만 오래된 lead를 더 오래 사용합니다.",
            "오래된 lead를 빨리 버리지만 검출 흔들림에 민감해집니다.",
        ),
        "button_pulse_frames": param_meta(
            "버튼 펄스 길이", "버튼 송신", "100 Hz frame", 1, 1, 10,
            "한 번의 SET-/RES+ 조작을 차량에 전달할 연속 CAN 프레임 수입니다.",
            "차량이 버튼을 인식하기 쉬워지지만 길게 누른 것으로 해석될 수 있습니다.",
            "펄스가 짧아지며 너무 작으면 차량이 명령을 놓칠 수 있습니다.",
        ),
    },
    "recording": {
        "enabled": {
            "label": "주행 데이터 기록",
            "section": "기록",
            "description": "모델이 실제 사용한 영상과 CAN 송수신, 모델·제어 상태를 함께 저장합니다.",
            "increase": "켜면 하드웨어 H.264 인코더로 기록을 시작합니다.",
            "decrease": "끄면 현재 기록을 안전하게 닫고 인코더를 유휴 상태로 둡니다.",
        },
        "bitrate_bps": param_meta(
            "기록 비트레이트", "기록", "bps", 500000, 1000000, 20000000,
            "H.264 인코더 목표 비트레이트입니다. 인코더는 기동할 때 한 번 열리므로 "
            "변경은 recordd 재시작부터 적용됩니다.",
            "화질이 올라가고 파일이 커집니다.",
            "파일이 작아지고 화질이 내려갑니다.",
        ),
    },
    "display": {
        "enabled": {
            "label": "디스플레이 전원",
            "section": "백라이트",
            "description": "LCD 영상 출력은 유지한 채 백라이트만 켜거나 끕니다.",
            "increase": "백라이트 PWM을 저장된 밝기로 다시 켭니다.",
            "decrease": "백라이트 PWM duty를 0으로 내려 화면을 완전히 끕니다.",
        },
        "brightness_percent": param_meta(
            "화면 밝기", "백라이트", "%", 1, 1, 100,
            "백라이트 PWM3(10 kHz) 점등률입니다. 실제 duty는 밝기 x 보드 최대치(/boot/board의 "
            "disp_max_backlight)입니다. 전원을 꺼도 이 값은 유지됩니다.",
            "화면이 밝아집니다.",
            "화면이 어두워집니다. 완전히 끄려면 전원 스위치를 사용합니다.",
            control="slider",
        ),
        "alert_volume_percent": param_meta(
            "알림음 크기", "소리", "%", 1, 0, 100,
            "보드 스피커로 내는 알림음(engage·해제·경고·출발 알림)의 크기입니다. overlayd가 "
            "1초 안에 반영하고, 바꿀 때마다 확인음을 한 번 냅니다.",
            "알림음이 커집니다.",
            "알림음이 작아집니다. 0%면 소리를 내지 않습니다.",
            control="slider",
        ),
        "camera_offset_m": param_meta(
            "카메라 좌우 위치 보정", "카메라 장착", "m", 0.01, -0.35, 0.35,
            "보드를 차 중심에서 옆으로 달았을 때 모델이 차 중심에서 본 것처럼 영상을 옮깁니다(sunnypilot "
            "camera offset과 같은 방식). Lane·Laneless 모두에 적용되고 HUD 차선도 같이 맞춥니다. "
            "±0.35 m를 넘으면 앞차·신호등 인식이 나빠져 막아 둡니다.",
            "차가 차선 안에서 왼쪽으로 갑니다(보드를 운전석 쪽에 달았으면 그 거리만큼 +).",
            "차가 오른쪽으로 갑니다.",
        ),
        "camera_height_m": param_meta(
            "카메라 높이", "카메라 장착", "m", 0.01, 0.8, 2.0,
            "도로면에서 카메라까지 높이입니다. 좌우 위치 보정의 크기를 정하는 데만 씁니다(보정이 0이면 무관).",
            "같은 보정값의 효과가 작아집니다.",
            "같은 보정값의 효과가 커집니다.",
        ),
        "hud_debug": {
            "label": "HUD 진단 정보",
            "section": "화면 표시",
            "description": "주행 화면 왼쪽에 다른 카드에 없는 진단 수치(FPS, 조향 토크, 학습기 상세)를 "
            "띄웁니다. 주행 화면 왼쪽 열을 눌러도 잠시 켜고 끌 수 있습니다. 기어, TPMS와 카메라 보정, 보드 "
            "상태, 제어가 쓰는 학습값은 늘 보이고, 네트워크 정보는 오른쪽 위를 누르면 나옵니다.",
            "increase": "켜면 진단 수치를 계속 보여 줍니다.",
            "decrease": "끄면 주행에 필요한 정보만 보여 줍니다.",
        },
    },
}

# 백라이트 하드웨어에 적용하는 display 항목. 나머지(알림음)는 overlayd가 파일에서 읽는다.
BACKLIGHT_KEYS = ("enabled", "brightness_percent")


def configured_paths() -> Dict[str, Path]:
    params_dir = Path(os.environ.get("EDGEPILOT_PARAMS_DIR", "params"))
    return {
        group: params_dir / filename
        for group, filename in GROUP_FILES.items()
    }


def configured_default_paths() -> Dict[str, Path]:
    params_dir = Path(os.environ.get("EDGEPILOT_PARAM_DEFAULTS_DIR", "params.defaults"))
    return {
        group: params_dir / filename
        for group, filename in GROUP_FILES.items()
    }


def find_process_pids(process_name: str) -> list[int]:
    pids = []
    proc = Path("/proc")
    if not proc.is_dir():
        return pids
    for entry in proc.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            argv0 = (entry / "cmdline").read_bytes().split(b"\0", 1)[0]
            if Path(os.fsdecode(argv0)).name == process_name:
                pids.append(int(entry.name))
        except (FileNotFoundError, PermissionError, ProcessLookupError, ValueError):
            continue
    return sorted(pids)


def find_controlsd_pids() -> list[int]:
    return find_process_pids(CONTROLSD_NAME)


def find_recordd_pids() -> list[int]:
    return find_process_pids(RECORDD_NAME)


def notify_controlsd() -> list[int]:
    notified = []
    for pid in find_controlsd_pids():
        try:
            os.kill(pid, signal.SIGHUP)
            notified.append(pid)
        except (PermissionError, ProcessLookupError):
            continue
    return notified


class ParamStore:
    def __init__(
        self,
        paths: Dict[str, Path] | None = None,
        notifier: Callable[[], list[int]] = notify_controlsd,
        default_paths: Dict[str, Path] | None = None,
        display_controller: DisplayBacklight | None = None,
    ):
        self.paths = paths or configured_paths()
        self.notifier = notifier
        self.lock = threading.Lock()
        self.default_paths = (
            default_paths
            if default_paths is not None
            else (configured_default_paths() if paths is None else {})
        )
        self.display_controller = display_controller
        self._sync_runtime_schema()
        if self.display_controller is not None and "display" in self.paths:
            try:
                self.display_controller.apply(self.read_group("display"))
            except (RuntimeError, ValueError):
                pass

    def _sync_runtime_schema(self) -> None:
        """런타임 JSON의 키 집합을 params.defaults의 스키마에 맞춘다.

        빠진 키는 기본값으로 채우고, 스키마에서 사라진 키는 지운다. 살아남은
        키의 값은 그대로 둔다. 지우지 않으면 편집기가 런타임 JSON 키를 그대로
        렌더링하므로(Object.entries), 코드가 더 이상 읽지 않는 파라미터가
        편집 가능한 채로 남아 튜닝이 적용된 것처럼 보인다.
        """
        for group, default_path in self.default_paths.items():
            runtime_path = self.paths.get(group)
            if runtime_path is None or not default_path.is_file():
                continue
            try:
                defaults = json.loads(default_path.read_text(encoding="utf-8"))
                runtime = (
                    json.loads(runtime_path.read_text(encoding="utf-8"))
                    if runtime_path.is_file()
                    else {}
                )
            except json.JSONDecodeError:
                continue
            if not isinstance(defaults, dict) or not isinstance(runtime, dict):
                continue
            merged = {key: runtime.get(key, value) for key, value in defaults.items()}
            if merged != runtime:
                self._atomic_write(runtime_path, merged)

    def read_group(self, group: str) -> Dict[str, Any]:
        path = self._path(group)
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except FileNotFoundError as exc:
            raise ValueError(f"parameter file not found: {path}") from exc
        except json.JSONDecodeError as exc:
            raise ValueError(f"invalid JSON in {path}: {exc}") from exc
        if not isinstance(data, dict):
            raise ValueError(f"parameter document must be an object: {path}")
        return data

    def snapshot(self) -> Dict[str, Any]:
        with self.lock:
            documents = {group: self.read_group(group) for group in self.paths}
        return {
            "params": documents,
            "metadata": PARAM_METADATA,
            "paths": {group: str(path) for group, path in self.paths.items()},
            "controlsd_pids": find_controlsd_pids(),
            "recordd_pids": find_recordd_pids(),
            "display_status": self._display_status(),
        }

    def update(self, group: str, values: Dict[str, Any]) -> Dict[str, Any]:
        if not values:
            raise ValueError("no parameter values supplied")
        with self.lock:
            document = self.read_group(group)
            unknown = sorted(set(values) - set(document))
            if unknown:
                raise KeyError(", ".join(unknown))
            document.update(values)
            if group == "display" and any(key in values for key in BACKLIGHT_KEYS):
                if self.display_controller is None:
                    raise ValueError("display backlight control is unavailable")
                self.display_controller.apply(document)
            self._atomic_write(self._path(group), document)
        notified = [] if group in ("recording", "display") else self.notifier()
        return {
            "group": group,
            "values": values,
            "params": document,
            "notified_pids": notified,
            "controlsd_pids": find_controlsd_pids(),
            "recordd_pids": find_recordd_pids(),
            "display_status": self._display_status(),
        }

    def _display_status(self) -> Dict[str, Any]:
        if self.display_controller is None:
            return {
                "available": False,
                "enabled": False,
                "brightness_percent": 0,
                "mode": "unavailable",
                "error": "display backlight control is unavailable",
            }
        return self.display_controller.status()

    def _path(self, group: str) -> Path:
        if group not in self.paths:
            raise KeyError(group)
        return self.paths[group]

    @staticmethod
    def _atomic_write(path: Path, document: Dict[str, Any]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        mode = stat.S_IMODE(path.stat().st_mode) if path.exists() else 0o664
        fd, temporary_name = tempfile.mkstemp(
            prefix=path.name + ".", suffix=".tmp", dir=path.parent
        )
        temporary = Path(temporary_name)
        try:
            os.fchmod(fd, mode)
            with os.fdopen(fd, "w", encoding="utf-8") as file:
                json.dump(document, file, ensure_ascii=False, indent=2)
                file.write("\n")
                file.flush()
                os.fsync(file.fileno())
            os.replace(temporary, path)
            directory_fd = os.open(path.parent, os.O_RDONLY)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        finally:
            if temporary.exists():
                temporary.unlink()


# ---------------------------------------------------------------- 공유 메모리 채널 읽기

IPC_MAGIC = 0x4B323349
IPC_VERSION = 1
IPC_HEADER = struct.Struct("<IIIIQQII")  # IpcHeader; seq가 홀수면 쓰는 중


def boottime_ns() -> int:
    clock = getattr(time, "CLOCK_BOOTTIME", time.CLOCK_MONOTONIC)  # monotonic_now_ns와 같은 시계
    return time.clock_gettime_ns(clock)


class IpcReader:
    """LatestChannel 하나를 읽기 전용으로 연다. 파일이 다시 만들어지면 새로 연다."""

    def __init__(self, path: str, payload_size: int):
        self.path = path
        self.payload_size = payload_size
        self._map: mmap.mmap | None = None
        self._inode = None

    def _reopen_if_needed(self) -> None:
        inode = os.stat(self.path).st_ino
        if self._map is not None and inode == self._inode:
            return
        if self._map is not None:
            self._map.close()
            self._map = None
        fd = os.open(self.path, os.O_RDONLY)
        try:
            self._map = mmap.mmap(fd, 0, access=mmap.ACCESS_READ)
        finally:
            os.close(fd)
        self._inode = inode

    def read_payload(self) -> tuple[int, int, bytes] | None:
        """(seq, 발행 시각 ns, 앞쪽 payload_size 바이트). 아직 없거나 쓰는 중이면 None."""
        try:
            self._reopen_if_needed()
            assert self._map is not None
            for _ in range(4):
                magic, version, _, _, seq, stamp, size, _ = IPC_HEADER.unpack_from(self._map, 0)
                if magic != IPC_MAGIC or version != IPC_VERSION or seq == 0 or seq & 1 or size < self.payload_size:
                    return None
                payload = self._map[IPC_HEADER.size:IPC_HEADER.size + self.payload_size]
                if IPC_HEADER.unpack_from(self._map, 0)[4] == seq:
                    return seq, stamp, payload
            return None
        except (OSError, ValueError, struct.error):
            self._map = None
            return None


def unpack_fields(layout: struct.Struct, fields: tuple, payload: bytes) -> Dict[str, Any]:
    """(이름, struct 형식) 순서의 배치로 payload를 dict로 푼다. 배열 형식("8h", "3f")은 list가 된다."""
    values = list(layout.unpack(payload[:layout.size]))
    state: Dict[str, Any] = {}
    for name, fmt in fields:
        count = int(fmt[:-1]) if len(fmt) > 1 else 1
        state[name] = values[:count] if count > 1 else values[0]
        del values[:count]
    return state


def flag_names(value: int, names: tuple) -> Dict[str, bool]:
    """비트 순서대로 이름 붙인 플래그."""
    return {name: bool(value >> bit & 1) for bit, name in enumerate(names)}


# ---------------------------------------------------------------- 학습 상태(paramsd·torqued)

LEARNER_STATE_PATH = os.environ.get("EDGEPILOT_LEARNER_STATE_PATH", "/dev/shm/edgepilot_learner_state")
# LearnerState(src/common/ipc_messages.h) 필드 순서. check_web_console.py가 C++ offsetof와 대조한다.
LEARNER_FIELDS = (
    ("timestamp_ns", "Q"), ("flags", "I"),
    ("steer_ratio", "f"), ("stiffness_factor", "f"), ("roll_rad", "f"),
    ("angle_offset_average_deg", "f"), ("angle_offset_deg", "f"),
    ("steer_ratio_std", "f"), ("stiffness_factor_std", "f"),
    ("angle_offset_average_std", "f"), ("angle_offset_fast_std", "f"), ("yaw_bias_rad_s", "f"),
    ("lat_accel_factor_raw", "f"), ("lat_accel_offset_raw", "f"), ("friction_raw", "f"),
    ("lat_accel_factor", "f"), ("lat_accel_offset", "f"), ("friction", "f"),
    ("decay", "f"), ("max_resets", "f"), ("total_bucket_points", "i"), ("cal_perc", "i"),
    ("road_bank_lat_accel", "f"),
    ("prior_steer_ratio", "f"), ("prior_lat_accel_factor", "f"), ("prior_friction", "f"),
    ("bucket_points", "8h"), ("plan_delay_s", "f"),
)
LEARNER_STATE = struct.Struct("<" + "".join(fmt for _, fmt in LEARNER_FIELDS))
LEARNER_FLAGS = (  # ipc_messages.h kLearner* 비트 순서
    "vehicle_inputs_ok", "vehicle_valid", "sensor_valid", "steer_ratio_valid",
    "stiffness_valid", "offset_average_valid", "offset_valid", "torque_inputs_ok",
    "torque_valid", "use_vehicle", "use_torque", "vehicle_restored", "torque_restored", "use_delay",
    "localizer_inputs",
)
LEARNER_HISTORY_S = 600
GRAVITY = 9.81


def decode_learner_state(payload: bytes) -> Dict[str, Any]:
    state = unpack_fields(LEARNER_STATE, LEARNER_FIELDS, payload)
    state["flags"] = flag_names(state["flags"], LEARNER_FLAGS)
    return state


class LearnerStateReader(IpcReader):
    """controlsd의 /edgepilot_learner_state."""

    def __init__(self, path: str = LEARNER_STATE_PATH):
        super().__init__(path, LEARNER_STATE.size)

    def read(self) -> tuple[int, int, Dict[str, Any]] | None:
        """(seq, 발행 시각 ns, 상태). 아직 없거나 쓰는 중이면 None."""
        latest = self.read_payload()
        if latest is None:
            return None
        seq, stamp, payload = latest
        return seq, stamp, decode_learner_state(payload)


def learner_trend_row(state: Dict[str, Any]) -> list[float]:
    """추이 그래프 한 점: 시각, 조향비, 영점 평균·합계, 롤, 편경사 롤 환산, 배율 원시·필터."""
    return [
        round(state["timestamp_ns"] * 1e-9, 2),
        round(state["steer_ratio"], 4),
        round(state["angle_offset_average_deg"], 4),
        round(state["angle_offset_deg"], 4),
        round(math.degrees(state["roll_rad"]), 4),
        round(math.degrees(-state["road_bank_lat_accel"] / GRAVITY), 4),
        round(state["lat_accel_factor_raw"], 4),
        round(state["lat_accel_factor"], 4),
    ]


def fixed_lateral_values(steering: Dict[str, Any]) -> Dict[str, Any]:
    """학습값과 나란히 보여줄 수동값."""
    return {
        "steer_ratio": steering.get("steer_ratio"),
        "tire_stiffness_factor": steering.get("tire_stiffness_factor"),
        "angle_offset_deg": steering.get("angle_offset_deg"),
        "torque_lat_accel_offset": steering.get("torque_lat_accel_offset"),
        "live_bank_compensation": steering.get("live_bank_compensation"),
        "lat_accel_factor": steering.get("torque_lat_accel_factor"),
        "friction": steering.get("torque_friction"),
    }


class LearnerMonitor:
    """1초마다 최신 상태를 읽어 10분 추이를 남긴다. 탭을 늦게 열어도 추이가 보인다."""

    def __init__(self, reader: LearnerStateReader | None = None):
        self.reader = reader or LearnerStateReader()
        self.lock = threading.Lock()
        self.history: collections.deque = collections.deque(maxlen=LEARNER_HISTORY_S)
        self._last_seq: int | None = None
        self._stop = threading.Event()

    def sample(self) -> tuple[int, int, Dict[str, Any]] | None:
        with self.lock:
            latest = self.reader.read()
            if latest is not None and latest[0] != self._last_seq:
                self._last_seq = latest[0]
                row = learner_trend_row(latest[2])
                if self.history and row[0] < self.history[-1][0]:
                    self.history.clear()  # 시각이 거꾸로 갔다(다른 부팅의 상태 파일)
                self.history.append(row)
            return latest

    def trend(self) -> list[list[float]]:
        with self.lock:
            return list(self.history)

    def start(self) -> None:
        def run() -> None:
            while not self._stop.wait(1.0):
                self.sample()

        threading.Thread(target=run, name="learner-monitor", daemon=True).start()

    def stop(self) -> None:
        self._stop.set()

    def snapshot(self, steering: Dict[str, Any]) -> Dict[str, Any]:
        latest = self.sample()
        result: Dict[str, Any] = {
            "available": latest is not None,
            "path": self.reader.path,
            "fixed": fixed_lateral_values(steering),
            "use_live_vehicle_params": steering.get("use_live_vehicle_params"),
            "use_live_torque_params": steering.get("use_live_torque_params"),
            "use_live_delay": steering.get("use_live_delay"),
        }
        if latest is not None:
            _, stamp, state = latest
            result["age_s"] = max(0.0, (boottime_ns() - stamp) * 1e-9)
            result["state"] = state
            result["trend_row"] = learner_trend_row(state)
        return result


# ---------------------------------------------------------------- locationd(자세·조향 지연)

LOCALIZATION_STATE_PATH = os.environ.get("EDGEPILOT_LOCALIZATION_STATE_PATH", "/dev/shm/edgepilot_localization")
# LocalizationState(src/common/ipc_messages.h) 필드 순서. check_web_console.py가 C++ 크기와 대조한다.
LOCALIZATION_FIELDS = (
    ("timestamp_ns", "Q"), ("flags", "I"), ("lag_status", "I"),
    ("orientation_calib", "3f"), ("orientation_std", "3f"),
    ("angular_velocity_calib", "3f"), ("angular_velocity_calib_std", "3f"),
    ("velocity_device", "3f"), ("velocity_device_std", "3f"), ("acceleration_calib", "3f"),
    ("lateral_delay_s", "f"), ("lag_estimate_s", "f"), ("lag_estimate_std_s", "f"),
    ("lag_valid_blocks", "i"), ("lag_cal_perc", "i"), ("lag_points", "I"), ("input_flags", "I"),
)
LOCALIZATION_STATE = struct.Struct("<" + "".join(fmt for _, fmt in LOCALIZATION_FIELDS))
LOCALIZATION_FLAGS = (  # ipc_messages.h kLocalization* 비트 순서
    "filter_valid", "inputs_ok", "sensors_ok", "posenet_ok", "calib_valid", "lag_restored",
)
LOCALIZATION_INPUT_FLAGS = ("accel_invalid", "gyro_invalid", "camera_invalid", "camera_guarded")


def decode_localization_state(payload: bytes) -> Dict[str, Any]:
    state = unpack_fields(LOCALIZATION_STATE, LOCALIZATION_FIELDS, payload)
    state["flags"] = flag_names(state["flags"], LOCALIZATION_FLAGS)
    state["input_flags"] = flag_names(state["input_flags"], LOCALIZATION_INPUT_FLAGS)
    return state


class LocalizationReader(IpcReader):
    """locationd의 /edgepilot_localization."""

    def __init__(self, path: str = LOCALIZATION_STATE_PATH):
        super().__init__(path, LOCALIZATION_STATE.size)

    def snapshot(self, steering: Dict[str, Any]) -> Dict[str, Any]:
        result: Dict[str, Any] = {"available": False, "initial_lag": steering.get("steer_actuator_delay")}
        latest = self.read_payload()
        if latest is not None:
            _, stamp, payload = latest
            result.update(available=True, age_s=max(0.0, (boottime_ns() - stamp) * 1e-9),
                          state=decode_localization_state(payload))
        return result


# ---------------------------------------------------------------- 카메라 캘리브레이션 초기화

MODEL_STATE_PATH = os.environ.get("EDGEPILOT_MODEL_STATE_PATH", "/dev/shm/edgepilot_model_state")
CONTROL_STATE_PATH = os.environ.get("EDGEPILOT_CONTROL_STATE_PATH", "/dev/shm/edgepilot_control_state")
# modeld의 CalibrationService가 1초마다 이 파일을 보고, 있으면 지우고 처음부터 다시 수렴한다.
CALIBRATION_RESET_PATH = os.environ.get("EDGEPILOT_CALIBRATION_RESET_PATH",
                                        "/dev/shm/edgepilot_calibration_reset")
# offsetof(ModelState, calibration). check_web_console.py가 ipc_messages.h와 대조한다.
MODEL_CALIBRATION_OFFSET = 3224
CALIBRATION_STATE = struct.Struct("<Ii3f3f")  # CalibrationState: status, valid_blocks, rpy, spread
CONTROL_STATE_HEAD = struct.Struct("<QII")  # ControlState: timestamp_ns, enabled, engaged
CALIBRATION_STATUS = ("uncalibrated", "calibrated", "invalid", "recalibrating")
STATE_STALE_S = 2.0


class CalibrationControl:
    """modeld가 발행한 보정 상태를 보여 주고, 해제 상태에서만 초기화를 요청한다.

    openpilot의 Reset Calibration과 같이 결합 중에는 받지 않는다. 학습값(paramsd·torqued)은
    CAN 요레이트로 배우므로 카메라 장착과 무관해 그대로 둔다."""

    def __init__(self, model_path: str = MODEL_STATE_PATH, control_path: str = CONTROL_STATE_PATH,
                 request_path: str = CALIBRATION_RESET_PATH):
        self.model = IpcReader(model_path, MODEL_CALIBRATION_OFFSET + CALIBRATION_STATE.size)
        self.control = IpcReader(control_path, CONTROL_STATE_HEAD.size)
        self.request_path = Path(request_path)

    def engaged(self) -> bool:
        """controlsd가 살아 있고 결합 중이면 True. controlsd가 없거나 멈췄으면 조향도 없다."""
        latest = self.control.read_payload()
        if latest is None:
            return False
        _, stamp, payload = latest
        if (boottime_ns() - stamp) * 1e-9 > STATE_STALE_S:
            return False
        _, _, engaged = CONTROL_STATE_HEAD.unpack(payload)
        return bool(engaged)

    def status(self) -> Dict[str, Any]:
        result: Dict[str, Any] = {
            "available": False,
            "engaged": self.engaged(),
            "reset_pending": self.request_path.exists(),
        }
        latest = self.model.read_payload()
        if latest is None:
            return result
        _, stamp, payload = latest
        status, valid_blocks, *values = CALIBRATION_STATE.unpack_from(payload, MODEL_CALIBRATION_OFFSET)
        result.update({
            "available": True,
            "age_s": max(0.0, (boottime_ns() - stamp) * 1e-9),
            "status": CALIBRATION_STATUS[status] if status < len(CALIBRATION_STATUS) else str(status),
            "valid_blocks": valid_blocks,
            "rpy_deg": [round(math.degrees(v), 3) for v in values[:3]],
            "spread_deg": [round(math.degrees(v), 3) for v in values[3:]],
        })
        return result

    def request_reset(self) -> Dict[str, Any]:
        if self.engaged():
            raise PermissionError("결합 중에는 초기화할 수 없습니다. 먼저 해제하세요.")
        self.request_path.parent.mkdir(parents=True, exist_ok=True)
        self.request_path.touch()
        return self.status()


# ---------------------------------------------------------------- 판다 펌웨어

# pandad가 1초마다 쓰는 연결·버전·플래싱 진행(src/panda/pandad.cc)과, 웹 콘솔이 쓰는 플래싱 요청.
# 요청 파일에는 쓸 이미지의 버전이 들어 있고, pandad는 그 버전이 설치된 이미지와 같고 차 상태가
# 맞을 때만 쓴다. 이미지는 pandad와 같이 EDGEPILOT_PANDA_FIRMWARE, 없으면 설치 디렉터리의
# firmware/panda.bin.signed다.
PANDA_STATUS_PATH = "/dev/shm/edgepilot_panda_status.json"
PANDA_FLASH_REQUEST_PATH = "/dev/shm/edgepilot_panda_flash"
PANDA_FIRMWARE_PATH = str(Path(__file__).resolve().parent /
                          (os.environ.get("EDGEPILOT_PANDA_FIRMWARE") or "firmware/panda.bin.signed"))
PANDA_STATUS_STALE_S = 3.0
# panda_protocol.h와 같은 값: 서명 128바이트, 앱 영역(섹터 1~3) 48 KB, F413 보드(black, uno, dos)
PANDA_SIGNATURE_BYTES = 128
PANDA_APP_MAX_BYTES = 3 * 16 * 1024
PANDA_FLASHABLE_HW = (3, 5, 6)
PANDA_VERSION_PATTERN = re.compile(rb"(?<![0-9A-Za-z])[A-Z]{3,8}-[0-9A-Za-z]{8}-(?:DEBUG|RELEASE)(?![0-9A-Za-z])")
# 플래싱 조건이 보는 ControlState 필드(ipc_messages.h). check_web_console.py가 대조한다.
PANDA_CONTROL_FIELDS = {"engaged": 12, "active": 16, "vehicle_fresh": 32, "gear": 52,
                        "cluster_speed_kph": 56, "ego_speed_kph": 232}
GEAR_PARK = 0  # car/can_frame.h kGearPark
PANDA_CONTROL_MAX_AGE_S = 1.0
PANDA_PARKED_MAX_SPEED_KPH = 1.0
# pandad·panda_flasher의 이유 코드
PANDA_REASONS = {
    "control_stale": "제어 상태가 없습니다 (controlsd 확인)",
    "vehicle_stale": "차 상태를 받지 못합니다 (시동·CAN 확인)",
    "engaged": "조향이 켜져 있습니다 (해제 후)",
    "not_park": "P단이 아닙니다",
    "moving": "차가 움직이고 있습니다",
    "image_invalid": "설치된 펌웨어 이미지가 없거나 잘못됐습니다",
    "image_changed": "설치된 이미지가 바뀌었습니다 (새로고침 후 다시)",
    "no_panda": "판다가 USB에 없습니다",
    "hw_unsupported": "지원하지 않는 판다입니다 (F413 보드만)",
    "busy": "다른 프로세스가 판다를 쓰고 있습니다",
    "usb_error": "USB 오류",
    "bootstub_timeout": "판다가 bootstub으로 넘어가지 않았습니다",
    "flasher_missing": "bootstub 플래셔가 응답하지 않습니다",
    "erase_failed": "플래시를 지우지 못했습니다",
    "write_failed": "쓰다가 실패했습니다",
    "verify_failed": "쓴 끝 주소가 맞지 않습니다",
    "app_timeout": "새 펌웨어가 시작되지 않았습니다 (bootstub에 머묾, 다시 쓰세요)",
    "version_mismatch": "판다가 이미지와 다른 버전을 알립니다",
}
PANDA_STEPS = {"bootstub": "bootstub 진입", "erase": "지우는 중", "write": "쓰는 중", "verify": "확인",
               "reboot": "재시작", "check": "버전 확인"}


def panda_image_info(path: str) -> Dict[str, Any]:
    """설치된 앱 이미지(firmware/panda의 panda.bin.signed)를 panda_firmware.cc와 같은 규칙으로 본다."""
    info: Dict[str, Any] = {"path": path, "present": False, "valid": False, "version": "", "size": 0, "error": ""}
    try:
        data = Path(path).read_bytes()
    except OSError:
        info["error"] = "없음"
        return info
    size = len(data)
    body = size - PANDA_SIGNATURE_BYTES
    if size <= PANDA_SIGNATURE_BYTES + 16:
        error = "너무 작음"
    elif size > PANDA_APP_MAX_BYTES:
        error = f"앱 영역({PANDA_APP_MAX_BYTES} B)보다 큼"
    elif size % 4:
        error = "4바이트 단위가 아님"
    elif struct.unpack_from("<I", data)[0] != body:
        error = "서명된 이미지가 아님 (길이 필드)"
    elif data[body - 8:body - 4] != b"VERS":
        error = "VERS 꼬리 없음"
    else:
        error = ""
    match = PANDA_VERSION_PATTERN.search(data)
    if not error and not match:
        error = "버전 문자열 없음"
    info.update(present=True, size=size, valid=not error, error=error,
                version=match.group().decode() if match else "")
    return info


def panda_flash_blocker(control: IpcReader) -> str:
    """panda_firmware.cc의 panda_flash_allowed와 같은 조건. 막는 이유 코드, 없으면 빈 문자열."""
    latest = control.read_payload()
    if latest is None:
        return "control_stale"
    _, stamp, payload = latest
    if (boottime_ns() - stamp) * 1e-9 > PANDA_CONTROL_MAX_AGE_S:
        return "control_stale"

    def value(name: str, fmt: str = "<I"):
        return struct.unpack_from(fmt, payload, PANDA_CONTROL_FIELDS[name])[0]

    if not value("vehicle_fresh"):
        return "vehicle_stale"
    if value("engaged") or value("active"):
        return "engaged"
    if value("gear", "<i") != GEAR_PARK:
        return "not_park"
    if max(value("ego_speed_kph", "<f"), value("cluster_speed_kph", "<f")) >= PANDA_PARKED_MAX_SPEED_KPH:
        return "moving"
    return ""


class PandaFirmware:
    """기기 설정 탭의 판다 펌웨어 카드. pandad의 상태 파일, 설치된 이미지, 차 상태를 모아 보여 주고,
    주차 중일 때만 플래싱을 요청한다. 쓰는 것은 pandad이고, 그쪽도 같은 조건을 다시 본다."""

    def __init__(self, status_path: str = PANDA_STATUS_PATH, request_path: str = PANDA_FLASH_REQUEST_PATH,
                 image_path: str = PANDA_FIRMWARE_PATH, control_path: str = CONTROL_STATE_PATH):
        self.status_path = Path(status_path)
        self.request_path = Path(request_path)
        self.image_path = image_path
        self.control = IpcReader(control_path, CONTROL_STATE_SIZE)

    def _pandad(self) -> tuple[Dict[str, Any] | None, float]:
        try:
            document = json.loads(self.status_path.read_text(encoding="utf-8"))
            age = max(0.0, (boottime_ns() - int(document["stamp_ns"])) * 1e-9)
        except (OSError, ValueError, KeyError, TypeError):
            return None, 0.0
        return (document if age <= PANDA_STATUS_STALE_S else None), age

    def status(self) -> Dict[str, Any]:
        pandad, age = self._pandad()
        link = pandad or {}
        flash = link.get("flash") or {}
        image = panda_image_info(self.image_path)
        blocker = panda_flash_blocker(self.control)
        version = link.get("firmware_version", "")
        return {
            "available": pandad is not None,
            "age_s": age,
            "mode": link.get("mode", "none"),
            "serial": link.get("serial", ""),
            "hw_type": link.get("hw_type", 0),
            "hw_name": link.get("hw_name", ""),
            "firmware_version": version,
            "flash": flash,
            "flash_error_text": PANDA_REASONS.get(flash.get("error", ""), flash.get("error", "")),
            "flash_step_text": PANDA_STEPS.get(flash.get("step", ""), ""),
            "image": image,
            "blocker": blocker,
            "blocker_text": PANDA_REASONS.get(blocker, blocker),
            "request_pending": self.request_path.exists(),
            "up_to_date": bool(image["version"]) and image["version"] == version,
        }

    def request_flash(self, version: str) -> Dict[str, Any]:
        status = self.status()
        image = status["image"]
        if not status["available"]:
            raise PermissionError("pandad가 실행 중이 아닙니다. 런타임을 멈췄다면 보드에서 panda_flash를 쓰세요.")
        if status["request_pending"] or status["flash"].get("state") == "flashing":
            raise PermissionError("이미 플래싱 중입니다.")
        if not image["valid"]:
            raise PermissionError(f"설치된 이미지를 쓸 수 없습니다: {image['error']}")
        if version != image["version"]:
            raise PermissionError(PANDA_REASONS["image_changed"])
        if status["mode"] == "none":
            raise PermissionError(PANDA_REASONS["no_panda"])
        if status["mode"] == "app" and status["hw_type"] not in PANDA_FLASHABLE_HW:
            raise PermissionError(PANDA_REASONS["hw_unsupported"])
        if status["blocker"]:
            raise PermissionError(f"지금은 쓸 수 없습니다: {status['blocker_text']}")
        temporary = self.request_path.with_name(self.request_path.name + ".tmp")
        temporary.write_text(version + "\n", encoding="utf-8")
        os.replace(temporary, self.request_path)
        return self.status()


# ---------------------------------------------------------------- BEV(위에서 본 장면)

# 웹 BEV 탭이 읽는 ModelState·ControlState 필드의 바이트 위치(src/common/ipc_messages.h). 보드는 두 페이로드를
# 발행된 그대로 흘려보내기만 하고(해석·JSON 없음), 페이지(web/bev_data.js)가 /api/bev에서 이 위치를
# 받아 직접 읽고 그린다. check_web_console.py가 EDGEPILOT_MODEL_STATE_AT·EDGEPILOT_CONTROL_STATE_AT과
# 대조한다.
MODEL_STATE_SIZE = 3576
CONTROL_STATE_SIZE = 240
BEV_MODEL_FIELDS = {
    "model_timestamp_ns": 16, "valid": 28, "plan": 304, "lanes": 700, "lane_probabilities": 2284,
    "road_edges": 2316, "road_edge_stds": 3108, "lead": 3148, "gas_press_probs": 3528,
}
BEV_CONTROL_FIELDS = {
    "timestamp_ns": 0, "enabled": 8, "engaged": 12, "active": 16, "left_blinker": 40, "right_blinker": 44, "gear": 52,
    "desired_curvature": 72,
    "actual_curvature": 76, "normalized_output": 80, "departure_alert_type": 144, "green_light_alert_armed": 152,
    "hud_flags": 184, "ego_speed_kph": 232,
}
BEV_HUD_FLAGS = {"Laneless": 0, "SteerPaused": 7, "BrakeLights": 11}  # ipc_messages.h kHudFlag<이름>의 비트
# 스트림 한 프레임: 머리(매직 "BEV1", ModelState·ControlState 바이트 수, 보드 CLOCK_BOOTTIME ns)와 두
# 페이로드. 새 모델 프레임마다(최대 BEV_MAX_HZ) 보내고, 모델이 멈추면 1초마다 모델 없이 보내 페이지가
# 끊김과 멈춤을 구분한다.
BEV_FRAME = struct.Struct("<IHHQ")
BEV_FRAME_MAGIC = 0x31564542
BEV_MAX_HZ = 20.0
BEV_IDLE_S = 1.0
# 스트림은 다음 모델 프레임이 나올 때(이번 발행 + 카메라 한 주기)까지 잔다. 늦으면 BEV_RETRY_S마다 다시
# 보고, BEV_STALL_S 넘게 안 나오면 모델이 멈춘 것으로 보고 BEV_STALLED_POLL_S마다만 본다.
BEV_MODEL_PERIOD_S = 0.05
BEV_RETRY_S = 0.005
BEV_STALL_S = 0.25
BEV_STALLED_POLL_S = 0.1


def bev_layout() -> Dict[str, Any]:
    return {
        "model": {"size": MODEL_STATE_SIZE, **BEV_MODEL_FIELDS},
        "control": {"size": CONTROL_STATE_SIZE, **BEV_CONTROL_FIELDS},
        "hud_flags": BEV_HUD_FLAGS,
    }


def bev_frame(model: bytes, control: bytes, now_ns: int) -> bytes:
    return BEV_FRAME.pack(BEV_FRAME_MAGIC, len(model), len(control), now_ns) + model + control


async def bev_frames(hz: float, model_path: str = MODEL_STATE_PATH, control_path: str = CONTROL_STATE_PATH,
                     idle_s: float = BEV_IDLE_S, stopping: Callable[[], bool] = lambda: False):
    """BEV 스트림. 새 모델 프레임이면 두 페이로드를 복사해 보낸다. 보드 CPU를 아끼려고 다음 프레임이 나올
    때까지 자므로 프레임마다 한 번 남짓만 깬다(20 ms 폴링은 이것만으로 한 코어의 3.7%였다).
    연결이 끊기면 Starlette가 이 제너레이터를 취소하고, 서버가 내려가기 시작하면(stopping) 스스로 끝난다.
    끝나지 않으면 uvicorn이 응답이 끝나기를 기다려 매니저의 SIGKILL(3초)까지 내려가지 않는다."""
    model = IpcReader(model_path, MODEL_STATE_SIZE)
    control = IpcReader(control_path, CONTROL_STATE_SIZE)
    period = 1.0 / min(max(hz, 1.0), BEV_MAX_HZ)
    sent_seq = None
    next_frame = sent_at = 0.0
    while not stopping():
        now = time.monotonic()
        latest = model.read_payload()
        pending = latest is not None and latest[0] != sent_seq
        if (pending and now >= next_frame) or now - sent_at >= idle_s:
            send = pending and now >= next_frame
            current = control.read_payload()
            yield bev_frame(latest[2] if send else b"", current[2] if current is not None else b"", boottime_ns())
            sent_at = now
            if send:
                sent_seq, next_frame, pending = latest[0], now + period - BEV_RETRY_S, False
        if pending:
            wait = next_frame - now  # 보낼 프레임이 있다: 보낼 차례까지
        elif latest is None:
            wait = BEV_STALLED_POLL_S
        else:
            due = (latest[1] - boottime_ns()) * 1e-9 + BEV_MODEL_PERIOD_S  # 다음 모델 프레임까지
            wait = due + 0.002 if due > 0 else BEV_RETRY_S if due > -BEV_STALL_S else BEV_STALLED_POLL_S
        await asyncio.sleep(min(max(wait, BEV_RETRY_S), sent_at + idle_s - now))


WEB_DIR = Path(__file__).resolve().parent / "static"


class WebAssets:
    """BEV 탭의 JS(이 파일 옆 web/: BEV 모듈과 three.js 0.186.1). 처음 요청 때 한 번 gzip해 메모리에
    두고(three.js 800 KB가 200 KB로), ETag가 같으면 304로 끝낸다. 파일이 바뀌면(mtime·크기) 다시 읽는다.
    web/ 밖과 .js가 아닌 파일은 내주지 않는다."""

    def __init__(self, root: Path = WEB_DIR):
        self.root = root.resolve()
        self.lock = threading.Lock()
        self.cache: Dict[str, tuple[str, bytes]] = {}

    def get(self, name: str) -> tuple[str, bytes] | None:
        """(ETag, gzip한 내용). 없으면 None."""
        path = (self.root / name).resolve()
        if path.suffix != ".js" or self.root not in path.parents:
            return None
        try:
            info = path.stat()
            etag = f'"{info.st_mtime_ns:x}-{info.st_size:x}"'
            with self.lock:
                cached = self.cache.get(name)
                if cached is None or cached[0] != etag:
                    cached = (etag, gzip.compress(path.read_bytes(), compresslevel=6, mtime=0))
                    self.cache[name] = cached
            return cached
        except OSError:
            return None


HTML = r"""<!doctype html>
<html lang="ko">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
  <title>K7 실시간 튜닝</title>
  <style>
    :root {
      color-scheme: dark;
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: #101214;
      color: #f4f6f7;
      --surface: #191c1f;
      --surface-raised: #202428;
      --line: #353b40;
      --muted: #a5adb4;
      --accent: #58a6e7;
      --good: #4bc78d;
      --warn: #efb85b;
      --bad: #ed7474;
          /* 버튼 공통: 아이콘·조정·토글·학습값 저장 버튼이 같이 쓴다 */
      --btn-bg: #262b2f;
      --btn-bg-hover: #31373c;
      --btn-border: #515960;
      --btn-radius: 6px;
    }
    * { box-sizing: border-box; }
    body { margin: 0; min-width: 320px; background: #101214; }
    button, input { font: inherit; }
    button { touch-action: manipulation; }
    .app-header {
      position: sticky; top: 0; z-index: 5;
      display: flex; align-items: center; gap: 14px;
      min-height: 64px; padding: 10px max(16px, env(safe-area-inset-right)) 10px max(16px, env(safe-area-inset-left));
      background: #171a1d; border-bottom: 1px solid var(--line);
    }
    .brand { min-width: 0; }
    h1 { margin: 0; font-size: 19px; line-height: 1.2; font-weight: 750; letter-spacing: 0; }
    .connection {
      display: flex; align-items: center; gap: 7px;
      color: var(--muted); font-size: 13px; white-space: nowrap;
    }
    .dot { width: 9px; height: 9px; flex: 0 0 auto; border-radius: 50%; background: var(--bad); }
    .dot.online { background: var(--good); }
    .icon-button {
      width: 44px; height: 44px; margin-left: auto; padding: 0;
      border: 1px solid var(--btn-border); border-radius: var(--btn-radius);
      background: var(--btn-bg); color: #fff; font-size: 24px; line-height: 1;
      cursor: pointer;
    }
    .icon-button:hover { background: var(--btn-bg-hover); }
    .group-tabs {
      position: sticky; top: 64px; z-index: 4;
      display: grid; grid-template-columns: repeat(7, minmax(0, 1fr));
      padding: 0 max(16px, env(safe-area-inset-right)) 0 max(16px, env(safe-area-inset-left));
      background: #171a1d; border-bottom: 1px solid var(--line);
    }
    .group-tab {
      min-height: 50px; border: 0; border-bottom: 3px solid transparent;
      border-radius: 0; background: transparent; color: var(--muted);
      padding: 0 4px; cursor: pointer; font-size: 13px; font-weight: 750;
    }
    .group-tab.active { color: #fff; border-bottom-color: var(--accent); }
    main {
      width: min(1100px, 100%); margin: 0 auto;
      padding: 16px max(16px, env(safe-area-inset-right)) max(40px, env(safe-area-inset-bottom)) max(16px, env(safe-area-inset-left));
    }
    .count { margin-bottom: 14px; color: var(--muted); font-size: 13px; }
    .group-note {
      display: none; margin-bottom: 14px; padding: 11px 12px;
      border: 1px solid #6a5630; border-radius: 6px;
      background: #241f16; color: #e6c985; font-size: 13px; line-height: 1.45;
    }
    .group-note.visible { display: block; }
    #message {
      display: none; margin-bottom: 14px; padding: 10px 12px;
      border: 1px solid #366249; border-radius: 6px;
      background: #17261f; color: #79daa9; font-size: 13px;
    }
    #message.visible { display: block; }
    #message.failed { border-color: #743f3f; background: #2a1919; color: #f08b8b; }
    .section { margin: 0 0 24px; }
    .section-title {
      display: flex; align-items: baseline; gap: 9px;
      margin: 0 0 9px; color: #e5e9ec; font-size: 15px; font-weight: 750;
    }
    .section-title span { color: #7f8991; font-size: 12px; font-weight: 600; }
    .param-list {
      display: grid; grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 10px;
    }
    .param-card {
      min-width: 0; padding: 14px;
      border: 1px solid var(--line); border-radius: 7px;
      background: var(--surface);
    }
    .param-card.busy { opacity: 0.66; }
    .param-card.saved { border-color: #397659; }
    .param-card.error { border-color: #8a4949; }
    .param-head { display: flex; align-items: flex-start; gap: 12px; justify-content: space-between; }
    .param-title { margin: 0; font-size: 16px; line-height: 1.3; font-weight: 750; letter-spacing: 0; }
    .param-unit { color: var(--warn); font-size: 12px; white-space: nowrap; }
    .param-key {
      margin-top: 3px; color: #737e86;
      font: 11px ui-monospace, SFMono-Regular, Menlo, monospace;
      overflow-wrap: anywhere;
    }
    .description { min-height: 40px; margin: 11px 0 9px; color: #c5cbd0; font-size: 13px; line-height: 1.5; }
    .effects {
      display: grid; grid-template-columns: 1fr 1fr; gap: 8px;
      margin-bottom: 13px;
    }
    .effect {
      padding: 8px 9px; border-left: 3px solid #576068;
      background: #15181a; color: #aeb6bc; font-size: 12px; line-height: 1.45;
    }
    .effect.up { border-left-color: var(--warn); }
    .effect.down { border-left-color: var(--accent); }
    .effect b { display: block; margin-bottom: 2px; color: #e5e9ec; font-size: 11px; }
    .number-control {
      display: grid; grid-template-columns: 54px minmax(100px, 1fr) 54px;
      gap: 8px; height: 50px;
    }
    .adjust-button {
      border: 1px solid var(--btn-border); border-radius: var(--btn-radius);
      background: var(--btn-bg); color: #fff; cursor: pointer;
      font-size: 28px; font-weight: 500; line-height: 1;
    }
    .adjust-button:hover { background: var(--btn-bg-hover); }
    .value-wrap { position: relative; min-width: 0; }
    .value-input {
      width: 100%; height: 50px; padding: 5px 9px 17px;
      border: 1px solid #535c63; border-radius: 6px;
      background: #0d0f11; color: #fff; text-align: center;
      font-size: 19px; font-weight: 750; font-variant-numeric: tabular-nums;
    }
    .value-input:focus { outline: 2px solid var(--accent); outline-offset: 0; }
    .range {
      position: absolute; bottom: 4px; left: 4px; right: 4px;
      color: #768089; text-align: center; font-size: 9px; pointer-events: none;
    }
    .toggle-control {
      display: flex; align-items: center; justify-content: space-between;
      width: 100%; height: 50px; padding: 0 14px;
      border: 1px solid var(--btn-border); border-radius: var(--btn-radius);
      background: var(--btn-bg); color: #d7dce0; cursor: pointer; font-weight: 750;
    }
    .toggle {
      position: relative; width: 52px; height: 28px;
      border-radius: 15px; background: #555e65; transition: background 120ms ease;
    }
    .toggle::after {
      content: ""; position: absolute; top: 4px; left: 4px;
      width: 20px; height: 20px; border-radius: 50%; background: #fff;
      transition: transform 120ms ease;
    }
    .toggle-control[aria-checked="true"] .toggle { background: var(--good); }
    .toggle-control[aria-checked="true"] .toggle::after { transform: translateX(24px); }
    .slider-control {
      display: grid; grid-template-columns: minmax(0, 1fr) 72px;
      align-items: center; gap: 12px; height: 50px;
    }
    .brightness-slider { width: 100%; accent-color: var(--good); cursor: pointer; }
    .slider-value {
      display: flex; align-items: center; justify-content: center;
      height: 42px; border: 1px solid #535c63; border-radius: 6px;
      background: #0d0f11; color: #fff; font-size: 18px;
      font-weight: 750; font-variant-numeric: tabular-nums;
    }
    .card-status { min-height: 18px; margin-top: 7px; color: #7f8991; font-size: 11px; text-align: right; }
    .card-status.ok { color: var(--good); }
    .card-status.fail { color: var(--bad); }
    .empty {
      padding: 28px; border: 1px solid var(--line); border-radius: 7px;
      color: var(--muted); text-align: center;
    }
    details { margin-top: 24px; color: #758088; font-size: 11px; }
    summary { cursor: pointer; }
    .file-path { margin-top: 8px; font-family: ui-monospace, monospace; overflow-wrap: anywhere; }
    button:disabled, input:disabled { cursor: default; opacity: 0.5; }
    .live-card { min-width: 0; padding: 14px; border: 1px solid var(--line); border-radius: 7px; background: var(--surface); }
    .live-head { display: flex; flex-wrap: wrap; align-items: center; gap: 7px; margin-bottom: 10px; }
    .live-title { margin: 0 6px 0 0; font-size: 16px; font-weight: 750; }
    .live-badges { display: flex; flex-wrap: wrap; gap: 7px; }
    .toggle-control.mini { width: auto; height: 36px; gap: 10px; margin-left: auto; padding: 0 10px; font-size: 13px; }
    .live-head .card-status { flex-basis: 100%; min-height: 0; margin: 0; }
    .adopt-row { display: flex; flex-wrap: wrap; gap: 8px; margin-top: 12px; }
    .adopt-button {
      display: flex; align-items: center; gap: 10px;
      flex: 1 1 160px; min-height: 50px; padding: 7px 12px;
      border: 1px solid var(--btn-border); border-radius: var(--btn-radius);
      background: var(--btn-bg); color: #e5e9ec; cursor: pointer;
      text-align: left; font-size: 13px; font-weight: 750;
    }
    .adopt-button:hover:not(:disabled) { background: var(--btn-bg-hover); }
    .adopt-button:focus-visible { outline: 2px solid var(--accent); outline-offset: 0; }
    .adopt-icon { flex: 0 0 auto; color: #d7dce0; font-size: 18px; line-height: 1; }
    .adopt-text { display: flex; flex-direction: column; gap: 2px; min-width: 0; }
    .adopt-button.danger { border-color: #6b3a3a; color: #f3b0b0; }
    .adopt-button.danger .adopt-icon { color: #f3b0b0; }
    .adopt-button.danger:hover:not(:disabled) { background: #331f1f; }
    .adopt-button small { display: block; color: #8f99a1; font-size: 11px; font-weight: 500; }
    .learner-note { margin: 9px 0 0; padding: 6px 9px; border-radius: 5px; font-size: 12px; line-height: 1.45; }
    .learner-note.ignored { background: #24292d; color: var(--muted); }
    .learner-note.prior { background: #15233a; color: #9cc8f0; }
    .learner-note.reset { background: #2a2417; color: var(--warn); }
    .param-card.ignored .description, .param-card.ignored .effects,
    .param-card.ignored .number-control, .param-card.ignored .toggle-control { opacity: 0.5; }
    .badge { padding: 2px 9px; border-radius: 10px; font-size: 12px; font-weight: 700; white-space: nowrap; }
    .badge.good { background: #17261f; color: var(--good); }
    .badge.warn { background: #2a2417; color: var(--warn); }
    .badge.bad { background: #2a1919; color: var(--bad); }
    .badge.accent { background: #15233a; color: var(--accent); }
    .badge.muted { background: #24292d; color: var(--muted); }
    .live-table { width: 100%; border-collapse: collapse; table-layout: fixed; font-size: 13px; font-variant-numeric: tabular-nums; }
    .live-table th { padding: 4px 0; color: #7f8991; font-size: 11px; font-weight: 600; text-align: right; }
    .live-table td { padding: 7px 0; border-top: 1px solid #2a2f33; color: #c5cbd0; text-align: right; overflow-wrap: anywhere; }
    .live-table th:first-child, .live-table td:first-child { width: 27%; text-align: left; }
    .live-table td.value { color: #fff; font-weight: 750; }
    .live-table td.note { color: #7f8991; font-size: 12px; }
    .live-table td.warn { color: var(--warn); }
    .live-table td.bad { color: var(--bad); }
    .live-foot { margin-top: 9px; color: #7f8991; font-size: 12px; line-height: 1.5; }
    .trend-head { display: flex; justify-content: space-between; gap: 8px; margin-bottom: 6px; font-size: 13px; }
    .trend-now { color: #fff; font-weight: 750; font-variant-numeric: tabular-nums; text-align: right; }
    .trend svg { display: block; width: 100%; height: 90px; border-radius: 5px; background: #15181a; }
    .trend-range { display: flex; justify-content: space-between; margin-top: 3px; color: #7f8991; font-size: 10px; }
    @media (max-width: 760px) {
      .app-header { gap: 9px; }
      .connection { margin-left: auto; }
      .icon-button { margin-left: 0; }
      .param-list { grid-template-columns: 1fr; }
      .description { min-height: 0; }
    }
    @media (max-width: 460px) {
      h1 { font-size: 17px; }
      .group-tab { font-size: 11px; }
      .connection span:last-child { max-width: 92px; overflow: hidden; text-overflow: ellipsis; }
      .effects { grid-template-columns: 1fr; }
      .number-control { grid-template-columns: 58px minmax(0, 1fr) 58px; }
      .adjust-button, .value-input, .toggle-control { height: 54px; }
      .number-control { height: 54px; }
    }
  </style>
  <!-- BEV 탭(/web/bev.js)이 처음 열릴 때 불러오는 three.js -->
  <script type="importmap">{"imports": {"three": "/web/three/three.module.min.js"}}</script>
</head>
<body>
  <header class="app-header">
    <div class="brand"><h1>K7 실시간 튜닝</h1></div>
    <div class="connection" id="connection">
      <span id="dot" class="dot"></span><span id="status">연결 확인 중</span>
    </div>
    <button id="reload" class="icon-button" type="button" title="새로고침" aria-label="새로고침">↻</button>
  </header>
  <nav class="group-tabs" aria-label="파라미터 그룹">
    <button class="group-tab active" data-group="steering" type="button">조향</button>
    <button class="group-tab" data-group="driving" type="button">주행 제한</button>
    <button class="group-tab" data-group="adaptive_cruise" type="button">비전 크루즈</button>
    <button class="group-tab" data-group="recording" type="button">주행 기록</button>
    <button class="group-tab" data-group="display" type="button">기기 설정</button>
    <button class="group-tab" data-group="learners" type="button">실시간 학습</button>
    <button class="group-tab" data-group="bev" type="button">BEV</button>
  </nav>
  <main>
    <div id="count" class="count"></div>
    <div id="group-note" class="group-note"></div>
    <div id="message" role="status"></div>
    <div id="sections"></div>
    <details>
      <summary>파라미터 파일</summary>
      <div id="path" class="file-path"></div>
    </details>
  </main>
  <script>
    let snapshot = null;
    let activeGroup = "steering";
    let messageTimer = null;

    const sections = document.getElementById("sections");
    const message = document.getElementById("message");
    const path = document.getElementById("path");
    const dot = document.getElementById("dot");
    const status = document.getElementById("status");
    const connection = document.getElementById("connection");
    const count = document.getElementById("count");
    const groupNote = document.getElementById("group-note");
    const sectionOrder = {
      steering: [
        "기본 토크 제한", "토크 컨트롤러", "조향 반응", "차량 중심 보정",
        "운전자 개입", "속도별 토크 제한", "Smooth steer", "차량 모델",
        "LKAS fault 보호", "고정 조향 한계", "고정 차량 설정", "기타",
      ],
      driving: [
        "경로 모드", "운전자 개입", "상태와 CAN", "데이터 상태",
        "고정 차량 설정", "기타",
      ],
      adaptive_cruise: [
        "동작", "차간 거리", "속도 반응", "복귀 동작", "비전 판정",
        "버튼 송신", "기타",
      ],
      recording: ["기록"],
      display: ["백라이트", "소리"],
    };
    const groupNotes = {
      adaptive_cruise: "변경값은 즉시 적용됩니다. 이 기능은 순정 크루즈 버튼만 조절하며 브레이크를 직접 제어하지 않습니다.",
      recording: "기록은 모델 입력과 같은 1280x720 프레임을 사용합니다. 영상·CAN·상태·파라미터가 한 경로에 함께 저장됩니다.",
      display: "화면을 꺼도 영상 파이프라인은 계속 동작하고, 밝기 값은 다음에 켤 때 그대로 복원됩니다. 알림음 크기는 1초 안에 반영되고 확인음이 한 번 납니다.",
      learners: "paramsd·torqued는 항상 계산하고 기록합니다. 제어에는 스위치를 켠 쪽만 씁니다. 1초마다 갱신하고 추이는 최근 10분입니다.",
      bev: "모델이 본 길과 앞차입니다. 경로가 주황·빨강인 곳은 모델이 속도를 줄이려는 곳, 흰 호는 목표 곡률, 하늘색 호는 실제 곡률입니다. 끌어서 돌리고 두 손가락·휠로 확대합니다.",
    };
    // 학습 스위치는 학습값을 보면서 켜도록 실시간 학습 탭에만 둔다
    const hiddenKeys = {steering: ["use_live_vehicle_params", "use_live_torque_params", "use_live_delay",
                                   "use_locationd_learner_inputs"]};

    function setConnection(pids, saved = false, recording = false) {
      const online = pids.length > 0;
      dot.classList.toggle("online", online);
      const process = recording ? "recordd" : "controlsd";
      connection.title = online ? `${process} PID ${pids.join(", ")}` : `${process}가 실행 중이 아닙니다`;
      status.textContent = online
        ? (saved ? (recording ? "기록 설정됨" : "실시간 적용됨") : (recording ? "기록기 연결됨" : "제어 연결됨"))
        : (saved ? "저장됨 · 프로세스 미연결" : "프로세스 미연결");
    }

    function refreshConnection(saved = false) {
      if (activeGroup === "bev") {
        showBevStatus();
        return;
      }
      if (activeGroup === "display") {
        const display = snapshot.display_status || {};
        const online = Boolean(display.available) && !display.error;
        dot.classList.toggle("online", online);
        connection.title = display.error || `백라이트 모드: ${display.mode}`;
        status.textContent = online
          ? (saved ? "기기 설정 적용됨" : "백라이트 연결됨")
          : "백라이트 제어 오류";
        return;
      }
      const recording = activeGroup === "recording";
      const pids = recording ? snapshot.recordd_pids : snapshot.controlsd_pids;
      setConnection(pids || [], saved, recording);
    }

    function setMessage(text, failed = false) {
      window.clearTimeout(messageTimer);
      message.textContent = text;
      message.className = text ? `visible${failed ? " failed" : ""}` : "";
      if (text && !failed) {
        messageTimer = window.setTimeout(() => {
          message.textContent = "";
          message.className = "";
        }, 1800);
      }
    }

    function genericMeta(key, value) {
      return {
        label: key,
        section: "기타",
        unit: "",
        step: Number.isInteger(value) ? 1 : 0.01,
        min: -1000000,
        max: 1000000,
        description: "추가 설명이 등록되지 않은 파라미터입니다.",
        increase: "값이 증가합니다.",
        decrease: "값이 감소합니다.",
      };
    }

    function decimalsForStep(step) {
      const text = String(step);
      if (text.includes("e-")) return Number(text.split("e-")[1]);
      return text.includes(".") ? text.split(".")[1].length : 0;
    }

    function clampAndRound(value, meta) {
      const clamped = Math.min(meta.max, Math.max(meta.min, value));
      const decimals = decimalsForStep(meta.step);
      return Number(clamped.toFixed(decimals));
    }

    async function loadParams(showMessage = false) {
      try {
        const response = await fetch("/api/params", {cache: "no-store"});
        if (!response.ok) throw new Error(await response.text());
        snapshot = await response.json();
        refreshConnection();
        if (showMessage) setMessage("최신 값을 불러왔습니다.");
        render();
      } catch (error) {
        setMessage(`읽기 실패: ${error.message}`, true);
      }
    }

    async function applyValue(key, value, card, input = null) {
      const group = card.dataset.group || activeGroup;
      card.classList.remove("saved", "error");
      card.classList.add("busy");
      card.querySelectorAll("button, input").forEach(control => control.disabled = true);
      const cardStatus = card.querySelector(".card-status");
      cardStatus.textContent = "적용 중";
      cardStatus.className = "card-status";
      try {
        const response = await fetch(`/api/params/${group}`, {
          method: "PATCH",
          headers: {"Content-Type": "application/json"},
          body: JSON.stringify({values: {[key]: value}}),
        });
        if (!response.ok) throw new Error(await response.text());
        const update = await response.json();
        snapshot.params[group] = update.params;
        snapshot.controlsd_pids = update.controlsd_pids || update.notified_pids;
        snapshot.recordd_pids = update.recordd_pids || [];
        snapshot.display_status = update.display_status || snapshot.display_status;
        const applied = update.params[key];
        if (input) input.value = String(applied);
        card.classList.add("saved");
        cardStatus.textContent = `적용됨 ${new Date().toLocaleTimeString("ko-KR", {hour12: false})}`;
        cardStatus.className = "card-status ok";
        refreshConnection(true);
        window.setTimeout(() => card.classList.remove("saved"), 900);
        return applied;
      } catch (error) {
        card.classList.add("error");
        cardStatus.textContent = "적용 실패";
        cardStatus.className = "card-status fail";
        setMessage(error.message, true);
        throw error;
      } finally {
        card.classList.remove("busy");
        card.querySelectorAll("button, input").forEach(control => control.disabled = false);
      }
    }

    function createEffects(meta, isBoolean) {
      const effects = document.createElement("div");
      effects.className = "effects";
      const down = document.createElement("div");
      down.className = "effect down";
      const downTitle = document.createElement("b");
      downTitle.textContent = isBoolean ? "끄면" : "값 감소";
      down.append(downTitle, document.createTextNode(meta.decrease));
      const up = document.createElement("div");
      up.className = "effect up";
      const upTitle = document.createElement("b");
      upTitle.textContent = isBoolean ? "켜면" : "값 증가";
      up.append(upTitle, document.createTextNode(meta.increase));
      effects.append(down, up);
      return effects;
    }

    function createNumberControl(key, value, meta, card) {
      const control = document.createElement("div");
      control.className = "number-control";
      const minus = document.createElement("button");
      minus.type = "button";
      minus.className = "adjust-button";
      minus.textContent = "−";
      minus.title = `${meta.step}${meta.unit ? ` ${meta.unit}` : ""} 감소`;
      minus.setAttribute("aria-label", minus.title);
      const valueWrap = document.createElement("div");
      valueWrap.className = "value-wrap";
      const input = document.createElement("input");
      input.className = "value-input";
      input.type = "number";
      input.inputMode = "decimal";
      input.value = String(value);
      input.step = String(meta.step);
      input.min = String(meta.min);
      input.max = String(meta.max);
      input.setAttribute("aria-label", `${meta.label} 현재값`);
      const range = document.createElement("span");
      range.className = "range";
      range.textContent = `${meta.min} ~ ${meta.max}${meta.unit ? ` ${meta.unit}` : ""}`;
      valueWrap.append(input, range);
      const plus = document.createElement("button");
      plus.type = "button";
      plus.className = "adjust-button";
      plus.textContent = "+";
      plus.title = `${meta.step}${meta.unit ? ` ${meta.unit}` : ""} 증가`;
      plus.setAttribute("aria-label", plus.title);

      const adjust = async direction => {
        const current = Number(input.value);
        if (!Number.isFinite(current)) {
          setMessage("숫자를 입력하세요.", true);
          return;
        }
        const next = clampAndRound(current + direction * meta.step, meta);
        input.value = String(next);
        try {
          await applyValue(key, next, card, input);
        } catch (_) {
          input.value = String(snapshot.params[activeGroup][key]);
        }
      };
      minus.addEventListener("click", () => adjust(-1));
      plus.addEventListener("click", () => adjust(1));
      input.addEventListener("keydown", event => {
        if (event.key === "Enter") input.blur();
      });
      input.addEventListener("change", async () => {
        const parsed = Number(input.value);
        if (!Number.isFinite(parsed)) {
          setMessage("숫자를 입력하세요.", true);
          input.value = String(snapshot.params[activeGroup][key]);
          return;
        }
        const next = clampAndRound(parsed, meta);
        input.value = String(next);
        try {
          await applyValue(key, next, card, input);
        } catch (_) {
          input.value = String(snapshot.params[activeGroup][key]);
        }
      });
      control.append(minus, valueWrap, plus);
      return control;
    }

    function createToggleControl(key, value, meta, card) {
      const button = document.createElement("button");
      button.type = "button";
      button.className = "toggle-control";
      button.setAttribute("role", "switch");
      button.setAttribute("aria-checked", String(value));
      const label = document.createElement("span");
      label.textContent = value ? "켜짐" : "꺼짐";
      const toggle = document.createElement("span");
      toggle.className = "toggle";
      button.append(label, toggle);
      button.addEventListener("click", async () => {
        const next = button.getAttribute("aria-checked") !== "true";
        try {
          const applied = await applyValue(key, next, card);
          button.setAttribute("aria-checked", String(applied));
          label.textContent = applied ? "켜짐" : "꺼짐";
        } catch (_) {}
      });
      return button;
    }

    function createSliderControl(key, value, meta, card) {
      const control = document.createElement("div");
      control.className = "slider-control";
      const input = document.createElement("input");
      input.className = "brightness-slider";
      input.type = "range";
      input.min = String(meta.min);
      input.max = String(meta.max);
      input.step = String(meta.step);
      input.value = String(value);
      input.setAttribute("aria-label", `${meta.label} 현재값`);
      const output = document.createElement("output");
      output.className = "slider-value";
      output.textContent = `${value}${meta.unit}`;
      input.addEventListener("input", () => {
        output.textContent = `${input.value}${meta.unit}`;
      });
      input.addEventListener("change", async () => {
        const previous = snapshot.params[activeGroup][key];
        const next = clampAndRound(Number(input.value), meta);
        try {
          const applied = await applyValue(key, next, card, input);
          output.textContent = `${applied}${meta.unit}`;
        } catch (_) {
          input.value = String(previous);
          output.textContent = `${previous}${meta.unit}`;
        }
      });
      control.append(input, output);
      return control;
    }

    function createCard(key, value, meta, group = activeGroup) {
      const card = document.createElement("article");
      card.className = "param-card";
      card.dataset.group = group;
      const head = document.createElement("div");
      head.className = "param-head";
      const identity = document.createElement("div");
      const title = document.createElement("h3");
      title.className = "param-title";
      title.textContent = meta.label;
      const technicalKey = document.createElement("div");
      technicalKey.className = "param-key";
      technicalKey.textContent = key;
      identity.append(title, technicalKey);
      const unit = document.createElement("div");
      unit.className = "param-unit";
      unit.textContent = meta.unit || (typeof value === "boolean" ? "ON / OFF" : "");
      head.append(identity, unit);
      const description = document.createElement("p");
      description.className = "description";
      description.textContent = meta.description;
      const cardStatus = document.createElement("div");
      cardStatus.className = "card-status";
      card.append(head);
      if (group === "steering") {
        const notes = learnerNotes(key, snapshot.params.steering);
        for (const [text, tone] of notes) card.appendChild(el("div", `learner-note ${tone}`, text));
        if (notes.some(([, tone]) => tone === "ignored")) card.classList.add("ignored");
      }
      card.append(description, createEffects(meta, typeof value === "boolean"));
      let editor;
      if (typeof value === "boolean") {
        editor = createToggleControl(key, value, meta, card);
      } else if (meta.control === "slider") {
        editor = createSliderControl(key, value, meta, card);
      } else {
        editor = createNumberControl(key, value, meta, card);
      }
      card.append(editor, cardStatus);
      return card;
    }

    /* 학습 스위치가 켜졌을 때 수동값의 역할. 무시되는 값은 흐리게, 사전값으로만 쓰이는 값은 표시한다.
     * torqued 사전값 셋은 스위치와 무관하게 캐시 키라 바꾸면 학습이 처음부터 다시 시작된다. */
    function learnerNotes(key, steering) {
      const vehicleOn = steering.use_live_vehicle_params === true;
      const torqueOn = steering.use_live_torque_params === true;
      const notes = [];
      if (vehicleOn && (key === "angle_offset_deg" || key === "live_bank_compensation"))
        notes.push(["paramsd 학습값 사용 중 · 이 값은 무시됩니다. 실시간 학습 탭에서 끄면 다시 쓰입니다.", "ignored"]);
      if (vehicleOn && key === "steer_ratio")
        notes.push(["paramsd 학습값 사용 중 · 학습 조향비의 출발점과 유효 범위(0.5~2배)로만 쓰입니다. 저장된 학습값이 있으면 그게 우선입니다.", "prior"]);
      if (vehicleOn && key === "tire_stiffness_factor")
        notes.push(["paramsd 학습값 사용 중 · 학습 강성 배율이 이 값에 곱해집니다.", "prior"]);
      if (torqueOn && key === "torque_lat_accel_offset")
        notes.push(["torqued 학습값 사용 중 · 이 값은 무시되고 학습 절편을 씁니다. 실시간 학습 탭에서 끄면 다시 쓰입니다.", "ignored"]);
      if (torqueOn && (key === "torque_lat_accel_factor" || key === "torque_friction"))
        notes.push(["torqued 학습값 사용 중 · 사전값과 허용 폭(배율 ±30%, 마찰 ±50%)으로만 쓰입니다.", "prior"]);
      if (key === "torque_lat_accel_factor" || key === "torque_friction")
        notes.push(["바꾸면 controlsd 다음 시작 때 torqued 학습이 처음부터 다시 시작됩니다.", "reset"]);
      if (steering.use_live_delay === true && key === "steer_actuator_delay")
        notes.push(["lagd 지연 사용 중 · 추정이 확정되면 경로 지연에는 추정값을 쓰고, 이 값은 확정 전 대체값과 토크 컨트롤러·torqued에 쓰입니다.", "prior"]);
      return notes;
    }

    /* 학습값을 수동값으로 옮긴다. target은 파라미터 단위, show는 학습값 표시. */
    const ADOPT = {
      vehicle: {
        switchKey: "use_live_vehicle_params",
        ready: s => s.flags.vehicle_valid,
        items: [
          {key: "steer_ratio", label: "조향비", show: s => num(s.steer_ratio, 2), target: s => s.steer_ratio},
          {key: "angle_offset_deg", label: "영점 평균", show: s => `${sgn(s.angle_offset_average_deg, 2)}°`,
           target: s => s.angle_offset_average_deg, ignoredWhenOn: true},
        ],
      },
      torque: {
        switchKey: "use_live_torque_params",
        ready: s => s.flags.torque_valid,
        items: [
          {key: "torque_lat_accel_factor", label: "배율", show: s => num(s.lat_accel_factor, 2),
           target: s => s.lat_accel_factor, resets: true},
          {key: "torque_friction", label: "마찰", show: s => num(s.friction, 3),
           target: s => s.friction, resets: true},
          {key: "torque_lat_accel_offset", label: "절편", show: s => sgn(s.lat_accel_offset, 3),
           target: s => s.lat_accel_offset, ignoredWhenOn: true},
        ],
      },
    };

    function adoptTarget(item, s) {
      const steering = snapshot.params.steering;
      const meta = snapshot.metadata.steering[item.key] || genericMeta(item.key, steering[item.key]);
      return {meta, current: steering[item.key], target: clampAndRound(item.target(s, steering), meta)};
    }

    async function adoptValue(shell, spec, item) {
      const s = shell.state;
      if (!s) return;
      const {meta, current, target} = adoptTarget(item, s);
      const on = snapshot.params.steering[spec.switchKey] === true;
      const lines = [`${meta.label} (${item.key})`, `${current} → ${target}`, ""];
      if (!on) lines.push("스위치가 꺼져 있어 지금 바로 제어에 반영됩니다.");
      else if (item.ignoredWhenOn) lines.push("스위치가 켜져 있어 지금은 무시되고, 스위치를 끄면 이 값을 씁니다.");
      else lines.push("스위치가 켜져 있어 사전값으로만 쓰입니다(controlsd 다음 시작부터).");
      if (item.resets) lines.push("controlsd 다음 시작 때 torqued 학습이 이 값을 새 사전값으로 처음부터 다시 시작합니다.");
      if (!window.confirm(lines.join("\n"))) return;
      try {
        await applyValue(item.key, target, shell.card);
      } catch (_) {}
      updateAdopt(shell, spec, s);  // applyValue가 카드의 버튼을 모두 다시 켠다
    }

    function updateAdopt(shell, spec, s) {
      shell.state = s;
      const ready = spec.ready(s);
      for (const item of spec.items) {
        const button = shell.adopt.get(item.key);
        const {current, target} = adoptTarget(item, s);
        const same = Number(current) === Number(target);
        const on = snapshot.params.steering[spec.switchKey] === true;
        const effect = !on ? "바로 제어에 반영"
          : item.ignoredWhenOn ? "스위치 켜짐: 끌 때 쓰임"
          : item.resets ? "스위치 켜짐: 사전값 · torqued 재학습"
          : "스위치 켜짐: 출발점·범위로만 쓰임";
        const detail = !ready ? "학습이 유효해지면 쓸 수 있습니다"
          : same ? `수동값과 같음 (${current})` : `${item.key} ${current} → ${target}`;
        const text = el("span", "adopt-text");
        text.append(document.createTextNode(`수동값에 저장 · ${item.label} ${item.show(s)}`),
                    el("small", "", detail), ...(ready && !same ? [el("small", "", effect)] : []));
        button.replaceChildren(el("span", "adopt-icon", "⤓"), text);
        button.disabled = !ready || same;
      }
    }

    // ------------------------------------------------------------ 실시간 학습
    const TREND_CHARTS = [
      {title: "조향비", series: [[1, "main"]], ref: f => [f.steer_ratio], span: 0.2, digits: 2},
      {title: "조향각 영점 (°) · 평균 / 합계", series: [[2, "main"], [3, "thin"]], ref: f => [f.angle_offset_deg], span: 0.2, digits: 2},
      {title: "도로 롤 (°) · 학습 / 편경사 환산", series: [[4, "main"], [5, "dash"]], ref: () => [], span: 0.5, digits: 2},
      {title: "토크 배율 · 필터 / 원시", series: [[7, "main"], [6, "raw"]], ref: (f, s) => [s.prior_lat_accel_factor, s.prior_lat_accel_factor * 0.7, s.prior_lat_accel_factor * 1.3], span: 0.2, digits: 2},
    ];
    let learnerTimer = null;
    let learnerTrend = [];
    let learnerWindow = 600;
    let learnerPanel = null;
    let learnerFailed = false;

    function num(value, digits) {
      return Number.isFinite(value) ? value.toFixed(digits) : "–";
    }
    function sgn(value, digits) {
      if (!Number.isFinite(value)) return "–";
      return `${value < 0 ? "−" : "+"}${Math.abs(value).toFixed(digits)}`;
    }
    function deg(rad) { return rad * 180 / Math.PI; }
    function el(tag, className, text) {
      const node = document.createElement(tag);
      if (className) node.className = className;
      if (text !== undefined) node.textContent = text;
      return node;
    }

    function liveTable(head, rows) {
      const table = el("table", "live-table");
      const headRow = el("tr");
      for (const text of head) headRow.appendChild(el("th", "", text));
      table.appendChild(headRow);
      for (const row of rows) {
        const tr = el("tr");
        row.forEach((cell, index) => {
          const [text, tone] = Array.isArray(cell) ? cell : [cell, ""];
          const kind = index === 1 ? "value" : index === 3 ? "note" : "";
          tr.appendChild(el("td", `${kind} ${tone}`.trim(), text));
        });
        table.appendChild(tr);
      }
      return table;
    }

    /* 제목·배지·스위치는 한 번 만들고 본문만 매초 바꾼다(누르는 중인 스위치를 갈아엎지 않게). */
    function liveShell(title, key, spec) {
      const card = el("article", "live-card");
      card.dataset.group = "steering";
      const head = el("div", "live-head");
      const badges = el("span", "live-badges");
      head.append(el("h3", "live-title", title), badges);
      const steering = snapshot.params.steering;
      if (key in steering) {
        const meta = snapshot.metadata.steering[key] || genericMeta(key, steering[key]);
        const toggle = createToggleControl(key, steering[key], meta, card);
        toggle.classList.add("mini");
        toggle.title = meta.label;
        head.append(toggle, el("div", "card-status"));
      }
      const body = el("div");
      const adopt = new Map();
      const row = el("div", "adopt-row");
      for (const item of spec.items) {
        const button = el("button", "adopt-button");
        button.type = "button";
        button.addEventListener("click", () => adoptValue(shell, spec, item));
        adopt.set(item.key, button);
        row.appendChild(button);
      }
      card.append(head, body, row);
      const shell = {card, badges, body, adopt, state: null};
      return shell;
    }

    function fillShell(shell, badges, children) {
      shell.badges.replaceChildren(...badges.map(([text, tone]) => el("span", `badge ${tone}`, text)));
      shell.body.replaceChildren(...children);
    }

    function vehicleCard(shell, s, f) {
      const flags = s.flags;
      const badges = ([
        flags.vehicle_valid ? ["유효", "good"] : ["무효", "bad"],
        ...(flags.vehicle_inputs_ok ? [] : [["입력 끊김", "warn"]]),
        ...(flags.sensor_valid ? [] : [["센서 불일치", "warn"]]),
        flags.use_vehicle ? ["제어에 사용 중", "accent"] : ["섀도", "muted"],
        flags.localizer_inputs ? ["입력 locationd", "accent"] : ["입력 ESP12", "muted"],
      ]);
      const children = [];
      const roll = deg(s.roll_rad);
      const bankRoll = deg(-s.road_bank_lat_accel / 9.81);
      children.push(liveTable(["항목", "학습값", "수동값", "±std · 범위"], [
        ["조향비", [num(s.steer_ratio, 2), flags.steer_ratio_valid ? "" : "bad"], num(f.steer_ratio, 2),
         `±${num(s.steer_ratio_std, 2)} · ${num(s.prior_steer_ratio * 0.5, 1)}~${num(s.prior_steer_ratio * 2, 1)}`],
        ["타이어 강성", [num(s.stiffness_factor, 3), flags.stiffness_valid ? "" : "bad"], `×${num(f.tire_stiffness_factor, 2)}`,
         `±${num(s.stiffness_factor_std, 3)} · 0.2~5`],
        ["영점 평균", [`${sgn(s.angle_offset_average_deg, 2)}°`, flags.offset_average_valid ? "" : "bad"], `${sgn(f.angle_offset_deg, 2)}°`,
         `±${num(deg(s.angle_offset_average_std), 2)}° · ±10°`],
        ["영점 합계(사용)", [`${sgn(s.angle_offset_deg, 2)}°`, flags.offset_valid ? "" : "bad"], "–",
         `빠른 성분 ±${num(deg(s.angle_offset_fast_std), 2)}°`],
        ["도로 롤", `${sgn(roll, 2)}°`, f.live_bank_compensation ? `편경사 ${sgn(bankRoll, 2)}°` : "보정 끔", "±10°"],
      ]));
      children.push(el("div", "live-foot",
        `자이로 바이어스 ${sgn(deg(s.yaw_bias_rad_s), 3)}°/s · 1분마다 저장 · ${flags.vehicle_restored ? "이번 시동에 복원" : "새로 시작"}`));
      fillShell(shell, badges, children);
    }

    function torqueCard(shell, s, f) {
      const flags = s.flags;
      const buckets = s.bucket_points;
      const calculable = buckets.every(n => n > 0) && s.lat_accel_factor_raw !== 0;
      const prior = s.prior_lat_accel_factor;
      const pf = s.prior_friction;
      const factorOut = calculable && (s.lat_accel_factor_raw < prior * 0.7 || s.lat_accel_factor_raw > prior * 1.3);
      const frictionOut = calculable && (s.friction_raw < pf * 0.5 || s.friction_raw > pf * 1.5);
      const badges = [
        flags.torque_valid ? ["유효", "good"] : [`학습 중 ${s.cal_perc}%`, "warn"],
        ...(flags.torque_inputs_ok ? [] : [["입력 끊김", "warn"]]),
        flags.use_torque ? ["제어에 사용 중", "accent"] : ["섀도", "muted"],
      ];
      const children = [];
      children.push(liveTable(["항목", "필터(사용값)", "원시", "사전값 · 허용"], [
        ["배율", num(s.lat_accel_factor, 2), [calculable ? num(s.lat_accel_factor_raw, 2) : "–", factorOut ? "warn" : ""],
         `${num(prior, 2)} · ${num(prior * 0.7, 2)}~${num(prior * 1.3, 2)}`],
        ["절편 (m/s²)", sgn(s.lat_accel_offset, 3), calculable ? sgn(s.lat_accel_offset_raw, 3) : "–",
         `수동 ${sgn(f.torque_lat_accel_offset, 3)}`],
        ["마찰", num(s.friction, 3), [calculable ? num(s.friction_raw, 3) : "–", frictionOut ? "warn" : ""],
         `${num(pf, 3)} · ${num(pf * 0.5, 3)}~${num(pf * 1.5, 3)}`],
      ]));
      children.push(el("div", "live-foot",
        `점 ${s.total_bucket_points.toLocaleString("ko-KR")} · decay ${num(s.decay, 1)} · 초기화 ${Math.max(0, Math.round(s.max_resets) - 1)}회 · 12초마다 저장 · ${flags.torque_restored ? "이번 시동에 복원" : "새로 시작"}`));
      fillShell(shell, badges, children);
    }

    function trendCard(chart, fixed, state) {
      const card = el("article", "live-card trend");
      const rows = learnerTrend;
      const tEnd = rows.length ? rows[rows.length - 1][0] : 0;
      const shown = rows.filter(r => r[0] >= tEnd - learnerWindow);
      const refs = chart.ref(fixed, state).filter(Number.isFinite);
      const values = [...refs];
      for (const [index, style] of chart.series)
        for (const r of shown) if (Number.isFinite(r[index]) && !(style === "raw" && r[index] === 0)) values.push(r[index]);
      const head = el("div", "trend-head");
      head.appendChild(el("span", "", chart.title));
      const last = shown.length ? shown[shown.length - 1] : null;
      head.appendChild(el("span", "trend-now",
        last ? chart.series.map(([index, style]) =>
          style === "raw" && last[index] === 0 ? "–" : num(last[index], chart.digits)).join(" / ") : "–"));
      card.appendChild(head);
      let lo = Math.min(...values), hi = Math.max(...values);
      if (!values.length) { lo = 0; hi = 1; }
      if (hi - lo < chart.span) { const mid = (hi + lo) / 2; lo = mid - chart.span / 2; hi = mid + chart.span / 2; }
      const pad = (hi - lo) * 0.08; lo -= pad; hi += pad;
      const W = 300, H = 90;
      const x = t => ((t - (tEnd - learnerWindow)) / learnerWindow) * W;
      const y = v => H - ((v - lo) / (hi - lo)) * H;
      const ns = "http://www.w3.org/2000/svg";
      const svg = document.createElementNS(ns, "svg");
      svg.setAttribute("viewBox", `0 0 ${W} ${H}`);
      svg.setAttribute("preserveAspectRatio", "none");
      svg.setAttribute("aria-label", chart.title);
      refs.forEach((value, i) => {
        const line = document.createElementNS(ns, "line");
        line.setAttribute("x1", "0"); line.setAttribute("x2", String(W));
        line.setAttribute("y1", String(y(value))); line.setAttribute("y2", String(y(value)));
        line.setAttribute("stroke", i === 0 ? "#8a949b" : "#5b646b");
        line.setAttribute("stroke-dasharray", i === 0 ? "5 4" : "2 4");
        line.setAttribute("vector-effect", "non-scaling-stroke");
        svg.appendChild(line);
      });
      const stroke = {main: "#58a6e7", thin: "#a5adb4", dash: "#a5adb4", raw: "#efb85b"};
      for (const [index, style] of chart.series) {
        const points = shown.filter(r => Number.isFinite(r[index]) && !(style === "raw" && r[index] === 0))
          .map(r => `${x(r[0]).toFixed(1)},${y(r[index]).toFixed(1)}`);
        if (points.length < 2) continue;
        const line = document.createElementNS(ns, "polyline");
        line.setAttribute("points", points.join(" "));
        line.setAttribute("fill", "none");
        line.setAttribute("stroke", stroke[style]);
        line.setAttribute("stroke-width", style === "main" ? "2" : "1.2");
        if (style === "dash") line.setAttribute("stroke-dasharray", "4 3");
        line.setAttribute("vector-effect", "non-scaling-stroke");
        svg.appendChild(line);
      }
      card.appendChild(svg);
      const range = el("div", "trend-range");
      range.append(el("span", "", `${num(lo, chart.digits)} ~ ${num(hi, chart.digits)}`),
                   el("span", "", refs.length ? `점선 ${num(refs[0], chart.digits)}` : "최근 10분"));
      card.appendChild(range);
      return card;
    }

    const CALIB_STATUS = {
      calibrated: ["보정 완료", "good"], uncalibrated: ["수렴 중", "warn"], invalid: ["범위 밖", "bad"],
      recalibrating: ["장착 변경 · 재보정 중", "warn"],
    };

    function calibShell() {
      const card = el("article", "live-card");
      const head = el("div", "live-head");
      const badges = el("span", "live-badges");
      head.append(el("h3", "live-title", "카메라 캘리브레이션"), badges);
      const body = el("div");
      const button = el("button", "adopt-button danger");
      button.type = "button";
      const row = el("div", "adopt-row");
      row.appendChild(button);
      card.append(head, body, row);
      const shell = {card, badges, body, button, data: null};
      button.addEventListener("click", () => resetCalibration(shell));
      return shell;
    }

    function calibCard(shell, c) {
      shell.data = c;
      if (!c || !c.available) {
        fillShell(shell, [["상태 없음", "muted"]], [el("div", "live-foot", "modeld가 보정 상태를 발행하지 않습니다.")]);
      } else {
        const [label, tone] = CALIB_STATUS[c.status] || [c.status, "muted"];
        const badges = [[label, tone], [`블록 ${c.valid_blocks}/50`, c.valid_blocks >= 5 ? "accent" : "muted"]];
        if (c.age_s > 2) badges.push([`${Math.round(c.age_s)}초 전`, "bad"]);
        const [r, p, y] = c.rpy_deg, [sr, sp, sy] = c.spread_deg;
        fillShell(shell, badges, [
          liveTable(["", "현재 (°)", "블록 편차 (°)", ""], [
            ["pitch", sgn(p, 2), num(sp, 2), "+ = 아래를 봄"],
            ["yaw", sgn(y, 2), num(sy, 2), "+ = 왼쪽을 봄"],
            ["roll", sgn(r, 2), num(sr, 2), ""],
          ]),
          el("div", "live-foot", "시속 24 km 이상 직진 100프레임(약 5초)이 한 블록, 5블록이 모이면 보정 완료. 마운트가 yaw 2°·pitch 4°보다 크게 바뀌면 스스로 다시 보정하고, 그보다 작게 옮겼으면 해제 상태에서 초기화하세요."),
        ]);
      }
      const blocked = c && c.engaged;
      const text = el("span", "adopt-text");
      text.append(document.createTextNode(c && c.reset_pending ? "초기화 요청됨 · modeld 대기 중" : "캘리브레이션 초기화"),
        el("small", "", blocked ? "결합 중에는 초기화할 수 없습니다" : "저장된 보정을 지우고 처음부터 다시 수렴합니다"));
      shell.button.replaceChildren(el("span", "adopt-icon", "↺"), text);
      shell.button.disabled = Boolean(blocked);
    }

    async function resetCalibration(shell) {
      const c = shell.data;
      const now = c && c.available ? `현재 pitch ${sgn(c.rpy_deg[1], 2)}° · yaw ${sgn(c.rpy_deg[2], 2)}° · 블록 ${c.valid_blocks}` : "현재 상태 없음";
      if (!window.confirm(["카메라 캘리브레이션을 초기화할까요?", now, "",
          "저장된 값(calibration.json)을 지우고 0°에서 다시 수렴합니다.",
          "수렴할 때까지(시속 24 km 이상 직진 약 30초) 조향이 부정확할 수 있습니다. 학습값은 그대로 둡니다."].join("\n"))) return;
      shell.button.disabled = true;
      try {
        const response = await fetch("/api/calibration/reset", {method: "POST"});
        const body = await response.json().catch(() => ({}));
        if (!response.ok) throw new Error(body.detail || response.statusText);
        setMessage("캘리브레이션 초기화를 요청했습니다");
        calibCard(shell, body);
      } catch (error) {
        setMessage(`초기화 실패: ${error.message}`, true);
        shell.button.disabled = false;
      }
    }

    /* paramsd·torqued 공통 입력 출처(요레이트·롤) 스위치와 지금 쓰는 출처. */
    function inputShell() {
      const card = el("article", "live-card");
      card.dataset.group = "steering";
      const head = el("div", "live-head");
      const badges = el("span", "live-badges");
      head.append(el("h3", "live-title", "학습 입력 · 요레이트·롤"), badges);
      const steering = snapshot.params.steering;
      const key = "use_locationd_learner_inputs";
      let meta = null;
      if (key in steering) {
        meta = snapshot.metadata.steering[key] || genericMeta(key, steering[key]);
        const toggle = createToggleControl(key, steering[key], meta, card);
        toggle.classList.add("mini");
        toggle.title = meta.label;
        head.append(toggle, el("div", "card-status"));
      }
      const body = el("div");
      card.append(head, body);
      return {card, badges, body, meta};
    }

    function inputCard(shell, learner) {
      const using = learner ? learner.flags.localizer_inputs : null;
      const badges = using === null ? [["상태 없음", "muted"]]
        : using ? [["locationd 사용 중", "accent"]] : [["ESP12 사용 중", "muted"]];
      fillShell(shell, badges, shell.meta ? [el("div", "live-foot", shell.meta.description)] : []);
    }

    function lagShell() {
      const card = el("article", "live-card");
      card.dataset.group = "steering";
      const head = el("div", "live-head");
      const badges = el("span", "live-badges");
      head.append(el("h3", "live-title", "lagd · 조향 지연"), badges);
      const steering = snapshot.params.steering;
      if ("use_live_delay" in steering) {
        const meta = snapshot.metadata.steering.use_live_delay || genericMeta("use_live_delay", steering.use_live_delay);
        const toggle = createToggleControl("use_live_delay", steering.use_live_delay, meta, card);
        toggle.classList.add("mini");
        toggle.title = meta.label;
        head.append(toggle, el("div", "card-status"));
      }
      const body = el("div");
      card.append(head, body);
      return {card, badges, body};
    }

    const LAG_STATUS = {0: ["추정 전", "warn"], 1: ["추정됨", "good"], 2: ["무효", "bad"]};

    function lagCard(shell, loc, learner) {
      if (!loc || !loc.available) {
        fillShell(shell, [["상태 없음", "muted"]], [el("div", "live-foot", "locationd가 발행하지 않습니다(IMU·locationd 확인).")]);
        return;
      }
      const s = loc.state, flags = s.flags;
      const poseOk = flags.filter_valid && flags.inputs_ok && flags.sensors_ok && flags.posenet_ok;
      const badges = [LAG_STATUS[s.lag_status] || [String(s.lag_status), "muted"],
        [`블록 ${s.lag_valid_blocks}/5`, s.lag_valid_blocks >= 5 ? "accent" : "muted"],
        poseOk ? ["자세 정상", "good"] : ["자세 무효", "bad"],
        learner && learner.flags.use_delay ? ["제어에 사용 중", "accent"] : ["섀도", "muted"]];
      const inputs = s.input_flags;
      for (const [key, label] of [["accel_invalid", "가속도 거부"], ["gyro_invalid", "자이로 거부"], ["camera_invalid", "카메라 거부"]])
        if (inputs[key]) badges.push([label, "warn"]);
      if (inputs.camera_guarded) badges.push(["카메라≠차속", "muted"]);
      if (loc.age_s > 2) badges.push([`${Math.round(loc.age_s)}초 전`, "bad"]);
      const ms = v => `${Math.round(v * 1000)} ms`;
      fillShell(shell, badges, [
        liveTable(["항목", "값", "비교", ""], [
          ["쓸 지연", ms(s.lateral_delay_s), `수동 ${ms(loc.initial_lag || 0)}`, "추정 전에는 수동값"],
          ["경로 지연(적용)", learner && learner.plan_delay_s > 0 ? ms(learner.plan_delay_s) : "–", "", "스위치를 켜고 확정되면 추정값"],
          ["진행 평균", `${ms(s.lag_estimate_s)} ±${ms(s.lag_estimate_std_s)}`, `창 안 점 ${s.lag_points}`, "블록 사이 0.1 s 넘으면 무효"],
          ["요레이트", `${sgn(deg(s.angular_velocity_calib[2]), 2)}°/s`, `±${num(deg(s.angular_velocity_calib_std[2]), 2)}`, "+ = 오른쪽"],
          ["도로 롤 · 피치", `${sgn(deg(s.orientation_calib[0]), 2)}° · ${sgn(deg(s.orientation_calib[1]), 2)}°`, "", "IMU 중력"],
        ]),
        el("div", "live-foot",
          `목표 곡률 → 실제 요레이트 지연(상류 lagd). 시속 40 km 이상, 조향 중·핸들 비조작·비포화 구간만 쓰고 100점마다 한 블록. 1분마다 저장 · ${flags.lag_restored ? "이번 시동에 복원" : "새로 시작"}`),
      ]);
    }

    function learnerSection(title, children, grid = true) {
      const section = el("section", "section");
      section.appendChild(el("h2", "section-title", title));
      const list = el("div", grid ? "param-list" : "");
      list.append(...children);
      section.appendChild(list);
      return section;
    }

    function updateLearners(data) {
      if (!learnerPanel) return;
      const panel = learnerPanel;
      calibCard(panel.calib, data.calibration);
      lagCard(panel.lag, data.localization, data.available ? data.state : null);
      inputCard(panel.input, data.available ? data.state : null);
      if (!data.available) {
        panel.notice.replaceChildren(el("div", "empty", "학습 상태가 없습니다. controlsd가 실행 중인지 확인하세요."));
        return;
      }
      const s = data.state, f = data.fixed;
      panel.notice.replaceChildren(...(data.age_s > 2
        ? [el("div", "group-note visible", `학습 상태가 ${Math.round(data.age_s)}초째 갱신되지 않습니다. controlsd를 확인하세요.`)]
        : []));
      vehicleCard(panel.vehicle, s, f);
      torqueCard(panel.torque, s, f);
      updateAdopt(panel.vehicle, ADOPT.vehicle, s);
      updateAdopt(panel.torque, ADOPT.torque, s);
      panel.trends.replaceChildren(...TREND_CHARTS.map(chart => trendCard(chart, f, s)));
    }

    async function pollLearners() {
      try {
        const response = await fetch("/api/learners", {cache: "no-store"});
        if (!response.ok) throw new Error(await response.text());
        const data = await response.json();
        const row = data.trend_row;
        if (row && learnerTrend.length && row[0] < learnerTrend[learnerTrend.length - 1][0]) learnerTrend = [];
        if (row && (!learnerTrend.length || row[0] > learnerTrend[learnerTrend.length - 1][0])) {
          learnerTrend.push(row);
          const cutoff = row[0] - learnerWindow;
          while (learnerTrend.length && learnerTrend[0][0] < cutoff) learnerTrend.shift();
        }
        path.textContent = data.path;
        if (learnerFailed) setMessage("");
        learnerFailed = false;
        updateLearners(data);
      } catch (error) {
        learnerFailed = true;
        setMessage(`학습 상태 읽기 실패: ${error.message}`, true);
      }
    }

    async function renderLearners() {
      sections.replaceChildren();
      count.textContent = "paramsd · torqued";
      groupNote.textContent = groupNotes.learners;
      groupNote.classList.add("visible");
      const vehicle = liveShell("paramsd · 차량 값", "use_live_vehicle_params", ADOPT.vehicle);
      const torque = liveShell("torqued · 토크 값", "use_live_torque_params", ADOPT.torque);
      const trendSection = learnerSection("최근 10분 추이", []);
      const calib = calibShell();
      const lag = lagShell();
      const input = inputShell();
      learnerPanel = {notice: el("div"), calib, lag, input, vehicle, torque, trends: trendSection.querySelector(".param-list")};
      sections.append(learnerPanel.notice, learnerSection("학습값 · 오른쪽 스위치로 제어에 사용", [vehicle.card, torque.card]),
                      learnerSection("학습 입력", [input.card]),
                      learnerSection("카메라 장착 · 조향 지연", [calib.card, lag.card]), trendSection);
      if (learnerTimer) return;
      learnerTimer = window.setInterval(pollLearners, 1000);
      try {
        const response = await fetch("/api/learners/trend", {cache: "no-store"});
        if (response.ok) {
          const trend = await response.json();
          learnerTrend = trend.rows;
          learnerWindow = trend.window_s;
        }
      } catch (_) {}
      pollLearners();
    }

    function stopLearners() {
      if (learnerTimer) window.clearInterval(learnerTimer);
      learnerTimer = null;
      learnerPanel = null;
    }

    // ------------------------------------------------------------ 판다 펌웨어
    /* 기기 설정 탭의 판다 펌웨어 카드. pandad가 쓰는 상태를 1초마다 읽고, 주차 중일 때만 설치된 이미지로
     * 플래싱을 요청한다(쓰는 것은 pandad이고, 그쪽도 같은 조건을 다시 본다). */
    let pandaTimer = null;
    let pandaShell = null;

    function pandaSection() {
      const card = el("article", "live-card");
      const head = el("div", "live-head");
      const badges = el("span", "live-badges");
      head.append(el("h3", "live-title", "Panda 펌웨어"), badges);
      const body = el("div");
      const button = el("button", "adopt-button danger");
      button.type = "button";
      const row = el("div", "adopt-row");
      row.appendChild(button);
      card.append(head, body, row);
      pandaShell = {card, badges, body, button, data: null};
      button.addEventListener("click", () => flashPanda(pandaShell));
      return learnerSection("판다", [card]);
    }

    function pandaCard(shell, p) {
      shell.data = p;
      const flash = p.flash || {};
      const flashing = flash.state === "flashing" || p.request_pending;
      const badges = [];
      if (!p.available) badges.push(["pandad 없음", "bad"]);
      else if (p.mode === "app") badges.push(["연결됨", "good"]);
      else if (p.mode === "bootstub") badges.push(["bootstub", "warn"]);
      else badges.push(["판다 없음", "bad"]);
      if (p.mode === "app" && p.image.valid) badges.push(p.up_to_date ? ["최신", "accent"] : ["다른 버전", "warn"]);
      if (flashing) badges.push([`쓰는 중 ${flash.percent || 0}%`, "accent"]);
      else if (flash.state === "done") badges.push(["플래싱 완료", "good"]);
      else if (flash.state === "failed") badges.push(["마지막 시도 실패", "bad"]);
      const current = p.mode === "app" ? (p.firmware_version || "–") : p.mode === "bootstub" ? "bootstub (앱 없음)" : "–";
      const children = [liveTable(["", "버전", "", ""], [
        ["판다", current, "", p.hw_name || ""],
        ["설치된 이미지", p.image.present ? (p.image.version || "–") : "없음", "",
         p.image.present ? `${p.image.size.toLocaleString()} B` : ""],
      ])];
      if (p.image.present && !p.image.valid) children.push(el("div", "live-foot", `이미지 문제: ${p.image.error}`));
      if (flashing) {
        children.push(el("div", "live-foot", `${p.flash_step_text || "요청 전달 중"} · 약 10초 걸립니다. 그동안 하네스가 순정 카메라 배선으로 동작합니다.`));
      } else if (flash.state === "failed") {
        children.push(el("div", "live-foot", `마지막 시도: ${p.flash_error_text}${flash.detail ? ` · ${flash.detail}` : ""}`));
      } else if (flash.state === "done") {
        children.push(el("div", "live-foot", `마지막 플래싱: ${flash.version}`));
      }
      children.push(el("div", "live-foot", "P단에 정지해 있고 조향이 꺼져 있을 때만 씁니다. 판다가 두 번 재부팅하는 약 10초 동안 조향 제어가 끊기고 하네스가 순정 카메라 배선으로 돌아갑니다."));
      fillShell(shell, badges, children);

      let reason = "";
      if (!p.available) reason = "pandad가 실행 중이 아닙니다";
      else if (flashing) reason = "쓰는 중입니다";
      else if (!p.image.valid) reason = p.image.present ? "설치된 이미지를 쓸 수 없습니다" : "설치된 이미지가 없습니다";
      else if (p.mode === "none") reason = "판다가 USB에 없습니다";
      else if (p.blocker) reason = p.blocker_text;
      const title = p.mode === "bootstub" ? "펌웨어 다시 쓰기" : p.up_to_date ? "같은 버전 다시 쓰기" : "펌웨어 플래싱";
      const text = el("span", "adopt-text");
      text.append(document.createTextNode(title), el("small", "", reason || `${p.image.version}을 판다에 씁니다`));
      shell.button.replaceChildren(el("span", "adopt-icon", "⤓"), text);
      shell.button.disabled = Boolean(reason);
    }

    async function flashPanda(shell) {
      const p = shell.data;
      if (!p || !p.image.valid) return;
      if (!window.confirm(["판다 펌웨어를 플래싱할까요?",
          `지금: ${p.mode === "app" ? p.firmware_version : p.mode}`,
          `쓸 이미지: ${p.image.version} (${p.image.size.toLocaleString()} B)`, "",
          "판다가 두 번 재부팅하는 약 10초 동안 조향 제어가 끊기고 하네스가 순정 카메라 배선으로 돌아갑니다.",
          "P단에 정지한 상태에서만 진행됩니다."].join("\n"))) return;
      shell.button.disabled = true;
      try {
        const response = await fetch("/api/panda/flash", {
          method: "POST", headers: {"Content-Type": "application/json"},
          body: JSON.stringify({version: p.image.version}),
        });
        const body = await response.json().catch(() => ({}));
        if (!response.ok) throw new Error(body.detail || response.statusText);
        setMessage("판다 펌웨어 플래싱을 요청했습니다");
        pandaCard(shell, body);
      } catch (error) {
        setMessage(`플래싱 요청 실패: ${error.message}`, true);
        shell.button.disabled = false;
      }
    }

    async function pollPanda() {
      const shell = pandaShell;
      if (!shell) return;
      try {
        const response = await fetch("/api/panda", {cache: "no-store"});
        if (!response.ok) throw new Error(response.statusText);
        const data = await response.json();
        if (shell === pandaShell) pandaCard(shell, data);
      } catch (error) {
        if (shell === pandaShell) fillShell(shell, [["읽기 실패", "bad"]], [el("div", "live-foot", error.message)]);
      }
    }

    function startPanda() {
      if (!pandaTimer) pandaTimer = window.setInterval(pollPanda, 1000);
      pollPanda();
    }

    function stopPanda() {
      if (pandaTimer) window.clearInterval(pandaTimer);
      pandaTimer = null;
      pandaShell = null;
    }

    // ------------------------------------------------------------ BEV
    /* 보드는 모델·제어 상태를 바이트 그대로 흘려보내기만 하고 해석과 그리기는 /web/bev.js가 이 브라우저에서
     * 한다. 패널과 WebGL은 한 번만 만들어 탭을 오가도 다시 쓰고, 스트림은 이 탭이 보일 때만 연다. */
    let bev = null;
    let bevLoading = null;
    let bevStatus = {live: false, text: "불러오는 중"};

    function showBevStatus() {
      if (activeGroup !== "bev") return;
      count.textContent = `BEV · ${bevStatus.text}`;
      dot.classList.toggle("online", bevStatus.live);
      status.textContent = bevStatus.live ? "모델 수신 중" : "모델 미수신";
      connection.title = bevStatus.text;
    }

    async function renderBev() {
      sections.replaceChildren();
      groupNote.textContent = groupNotes.bev;
      groupNote.classList.add("visible");
      path.textContent = "/dev/shm/edgepilot_model_state · /dev/shm/edgepilot_control_state";
      showBevStatus();
      try {
        bevLoading = bevLoading || import("/web/bev.js").then(module => module.createBev(update => {
          bevStatus = update;
          showBevStatus();
        }));
        bev = await bevLoading;
      } catch (error) {
        bevLoading = null;
        setMessage(`BEV를 불러오지 못했습니다: ${error.message}`, true);
        return;
      }
      if (activeGroup !== "bev") return;  // 불러오는 사이 다른 탭으로 갔다
      sections.replaceChildren(bev.element);
      if (!document.hidden) bev.start();
    }

    function stopBev() {
      if (bev) bev.stop();
    }

    document.addEventListener("visibilitychange", () => {
      if (!bev || activeGroup !== "bev") return;
      if (document.hidden) bev.stop();
      else bev.start();
    });

    function render() {
      if (activeGroup !== "bev") stopBev();
      if (activeGroup !== "display") stopPanda();
      if (activeGroup === "bev") {
        stopLearners();
        renderBev();
        return;
      }
      if (!snapshot) return;
      if (activeGroup === "learners") {
        renderLearners();
        return;
      }
      stopLearners();
      sections.replaceChildren();
      path.textContent = snapshot.paths[activeGroup];
      const note = groupNotes[activeGroup] || "";
      groupNote.textContent = note;
      groupNote.classList.toggle("visible", Boolean(note));
      const params = snapshot.params[activeGroup];
      const metadata = snapshot.metadata[activeGroup] || {};
      const hidden = hiddenKeys[activeGroup] || [];
      const visible = Object.entries(params).filter(([key]) => !hidden.includes(key));
      count.textContent = `${visible.length}개 항목`;
      const grouped = new Map();
      for (const [key, value] of visible) {
        const meta = metadata[key] || genericMeta(key, value);
        if (!grouped.has(meta.section)) grouped.set(meta.section, []);
        grouped.get(meta.section).push([key, value, meta]);
      }
      const orderedGroups = [...grouped.entries()];
      const order = sectionOrder[activeGroup] || [];
      orderedGroups.sort(([nameA], [nameB]) => {
        const rankA = order.includes(nameA) ? order.indexOf(nameA) : order.length;
        const rankB = order.includes(nameB) ? order.indexOf(nameB) : order.length;
        return rankA - rankB;
      });
      for (const [sectionName, entries] of orderedGroups) {
        const section = document.createElement("section");
        section.className = "section";
        const heading = document.createElement("h2");
        heading.className = "section-title";
        heading.append(document.createTextNode(sectionName));
        const sectionCount = document.createElement("span");
        sectionCount.textContent = `${entries.length}개`;
        heading.append(sectionCount);
        const list = document.createElement("div");
        list.className = "param-list";
        for (const [key, value, meta] of entries) {
          list.appendChild(createCard(key, value, meta));
        }
        section.append(heading, list);
        sections.appendChild(section);
      }
      if (!visible.length) {
        const empty = document.createElement("div");
        empty.className = "empty";
        empty.textContent = "표시할 항목이 없습니다.";
        sections.appendChild(empty);
      }
      if (activeGroup === "display") {
        sections.appendChild(pandaSection());
        startPanda();
      }
    }

    document.querySelectorAll(".group-tab").forEach(tab => {
      tab.addEventListener("click", () => {
        document.querySelectorAll(".group-tab").forEach(item => item.classList.remove("active"));
        tab.classList.add("active");
        activeGroup = tab.dataset.group;
        refreshConnection();
        render();
        window.scrollTo({top: 0, behavior: "smooth"});
      });
    });
    document.getElementById("reload").addEventListener("click", () => loadParams(true));
    loadParams();
  </script>
</body>
</html>
"""


def create_app(store: ParamStore | None = None,
               learner_monitor: LearnerMonitor | None = None,
               calibration: CalibrationControl | None = None,
               localization: LocalizationReader | None = None,
               panda: PandaFirmware | None = None,
               web: WebAssets | None = None,
               stopping: Callable[[], bool] = lambda: False) -> "FastAPI":
    from fastapi import FastAPI, Header, HTTPException, Query
    from fastapi.responses import HTMLResponse, Response, StreamingResponse

    param_store = store or ParamStore(display_controller=DisplayBacklight())
    monitor = learner_monitor or LearnerMonitor()
    calibration_control = calibration or CalibrationControl()
    localization_reader = localization or LocalizationReader()
    panda_firmware = panda or PandaFirmware()
    web_assets = web or WebAssets()
    application = FastAPI(title="K7 parameter server", docs_url="/docs")

    @application.on_event("startup")
    def start_monitor() -> None:
        monitor.start()

    @application.on_event("shutdown")
    def stop_monitor() -> None:
        monitor.stop()

    @application.get("/", response_class=HTMLResponse)
    def index() -> str:
        return HTML

    @application.get("/api/params")
    def get_params() -> Dict[str, Any]:
        try:
            return param_store.snapshot()
        except (KeyError, ValueError) as exc:
            raise HTTPException(status_code=500, detail=str(exc)) from exc

    @application.get("/api/learners")
    def get_learners() -> Dict[str, Any]:
        steering = param_store.read_group("steering")
        result = monitor.snapshot(steering)
        result["calibration"] = calibration_control.status()
        result["localization"] = localization_reader.snapshot(steering)
        return result

    @application.post("/api/calibration/reset")
    def reset_calibration() -> Dict[str, Any]:
        try:
            return calibration_control.request_reset()
        except PermissionError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        except OSError as exc:
            raise HTTPException(status_code=500, detail=str(exc)) from exc

    @application.get("/api/panda")
    def get_panda() -> Dict[str, Any]:
        return panda_firmware.status()

    @application.post("/api/panda/flash")
    def flash_panda(request: PandaFlashRequest) -> Dict[str, Any]:
        try:
            return panda_firmware.request_flash(request.version)
        except PermissionError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        except OSError as exc:
            raise HTTPException(status_code=500, detail=str(exc)) from exc

    @application.get("/api/learners/trend")
    def get_learner_trend() -> Dict[str, Any]:
        return {"rows": monitor.trend(), "window_s": LEARNER_HISTORY_S}

    @application.get("/api/bev")
    def get_bev_layout() -> Dict[str, Any]:
        return bev_layout()

    @application.get("/api/bev/stream")
    async def get_bev_stream(hz: float = Query(BEV_MAX_HZ, ge=1.0, le=BEV_MAX_HZ)):
        return StreamingResponse(bev_frames(hz, stopping=stopping), media_type="application/octet-stream",
                                 headers={"Cache-Control": "no-store"})

    @application.get("/web/{name:path}")
    def get_web_file(name: str, if_none_match: str | None = Header(None), accept_encoding: str = Header("")):
        asset = web_assets.get(name)
        if asset is None:
            raise HTTPException(status_code=404, detail="not found")
        etag, body = asset
        headers = {"ETag": etag, "Cache-Control": "no-cache", "Vary": "Accept-Encoding"}
        if if_none_match == etag:
            return Response(status_code=304, headers=headers)
        if "gzip" in accept_encoding:
            headers["Content-Encoding"] = "gzip"
        else:
            body = gzip.decompress(body)
        return Response(body, media_type="text/javascript", headers=headers)

    @application.patch("/api/params/{group}")
    def patch_params(group: str, patch: ParamPatch) -> Dict[str, Any]:
        try:
            return param_store.update(group, patch.values)
        except KeyError as exc:
            raise HTTPException(status_code=404, detail=f"unknown parameter: {exc}") from exc
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        except RuntimeError as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc

    return application


def main() -> None:
    import uvicorn

    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default=os.environ.get("EDGEPILOT_WEB_CONSOLE_HOST", "0.0.0.0"))
    parser.add_argument(
        "--port", type=int, default=int(os.environ.get("EDGEPILOT_WEB_CONSOLE_PORT", "8080"))
    )
    args = parser.parse_args()
    server: uvicorn.Server | None = None
    application = create_app(stopping=lambda: server is not None and server.should_exit)
    server = uvicorn.Server(uvicorn.Config(application, host=args.host, port=args.port, log_level="info"))
    server.run()


if __name__ == "__main__":
    main()
