"""차량 특성 탭의 학습 상태: controlsd가 내는 paramsd·torqued 학습값(LearnerState)과 그 10분 추이, locationd가 내는
자세·조향 지연(LocalizationState). 필드 배치는 src/common/ipc_messages.h와 같고 check_web_console.py가 대조한다.
수동값은 페이지가 파라미터에서 직접 읽는다."""
from __future__ import annotations

import collections
import math
import threading
import time
from typing import Any, Dict

from .shm_channel import IpcReader, flag_names, freshness, struct_layout, topic_path, unpack_fields
from .state_layout import (LEARNER_STATE_MAX_AGE_S, LEARNER_STATE_TOPIC, LOCALIZATION_STATE_MAX_AGE_S,
                           LOCALIZATION_STATE_TOPIC)

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
LEARNER_STATE = struct_layout(LEARNER_FIELDS)
LEARNER_FLAGS = (  # ipc_messages.h kLearner* 비트 순서
    "vehicle_inputs_ok", "vehicle_valid", "sensor_valid", "steer_ratio_valid",
    "stiffness_valid", "offset_average_valid", "offset_valid", "torque_inputs_ok",
    "torque_valid", "use_vehicle", "use_torque", "vehicle_restored", "torque_restored", "use_delay",
    "localizer_inputs", "lateral_nn_loaded",
)
LEARNER_HISTORY_S = 600
# 추이 행 사이의 최소 간격. 1초 스레드 말고 요청도 sample을 불러서, 새 발행마다 행을 쌓으면 10분 창이 3~5분만 남았다.
LEARNER_TREND_STEP_S = 0.9
GRAVITY = 9.81  # 편경사 롤 환산. 페이지(vehicle_view.js GRAVITY)도 같은 값과 식이다(check_web_console.py가 대조)
# 추이 행의 열 순서: 시각(s), 조향비, 영점 평균·합계(°), 롤과 편경사 롤 환산(°), 토크 배율 원시·필터. 페이지가 같은
# 이름(vehicle_view.js TREND_COLUMNS)으로 열을 찾는다(check_web_console.py가 대조).
LEARNER_TREND_COLUMNS = ("time_s", "steer_ratio", "angle_offset_average_deg", "angle_offset_deg", "roll_deg",
                         "bank_roll_deg", "lat_accel_factor_raw", "lat_accel_factor")

LOCALIZATION_FIELDS = (
    ("timestamp_ns", "Q"), ("flags", "I"), ("lag_status", "I"),
    ("orientation_calib", "3f"), ("orientation_std", "3f"),
    ("angular_velocity_calib", "3f"), ("angular_velocity_calib_std", "3f"),
    ("velocity_device", "3f"), ("velocity_device_std", "3f"), ("acceleration_calib", "3f"),
    ("lateral_delay_s", "f"), ("lag_estimate_s", "f"), ("lag_estimate_std_s", "f"),
    ("lag_valid_blocks", "i"), ("lag_cal_perc", "i"), ("lag_points", "I"), ("input_flags", "I"),
)
LOCALIZATION_STATE = struct_layout(LOCALIZATION_FIELDS)
LOCALIZATION_FLAGS = (  # ipc_messages.h kLocalization* 비트 순서
    "filter_valid", "inputs_ok", "sensors_ok", "posenet_ok", "calib_valid", "lag_restored",
)
LOCALIZATION_INPUT_FLAGS = ("accel_invalid", "gyro_invalid", "camera_invalid", "camera_guarded")


def decode_learner_state(payload: bytes) -> Dict[str, Any]:
    state = unpack_fields(LEARNER_STATE, LEARNER_FIELDS, payload)
    state["flags"] = flag_names(state["flags"], LEARNER_FLAGS)
    return state


def decode_localization_state(payload: bytes) -> Dict[str, Any]:
    state = unpack_fields(LOCALIZATION_STATE, LOCALIZATION_FIELDS, payload)
    state["flags"] = flag_names(state["flags"], LOCALIZATION_FLAGS)
    state["input_flags"] = flag_names(state["input_flags"], LOCALIZATION_INPUT_FLAGS)
    return state


def learner_trend_row(state: Dict[str, Any]) -> list[float]:
    """추이 그래프 한 점(LEARNER_TREND_COLUMNS 순서)."""
    values = {**state, "time_s": state["timestamp_ns"] * 1e-9, "roll_deg": math.degrees(state["roll_rad"]),
              "bank_roll_deg": math.degrees(-state["road_bank_lat_accel"] / GRAVITY)}
    return [round(values[name], 2 if name == "time_s" else 4) for name in LEARNER_TREND_COLUMNS]


class LearnerMonitor:
    """1초마다 최신 학습 상태를 읽어 10분 추이를 남긴다. 탭을 늦게 열어도 추이가 보인다."""

    def __init__(self, path: str = topic_path(LEARNER_STATE_TOPIC)):
        self.reader = IpcReader(path, LEARNER_STATE.size)
        self.lock = threading.Lock()
        self.history: collections.deque = collections.deque()  # 개수가 아니라 시각으로 자른다
        self._last_seq: int | None = None

    def sample(self) -> tuple[int, Dict[str, Any]] | None:
        """(발행 시각 ns, 학습 상태). 아직 없거나 쓰는 중이면 None. 새 발행이면 추이에 한 점을 더한다."""
        with self.lock:
            latest = self.reader.read_payload()
            if latest is None:
                return None
            seq, stamp, payload = latest
            state = decode_learner_state(payload)
            if seq != self._last_seq:
                self._last_seq = seq
                row = learner_trend_row(state)  # 시각은 부팅마다 새로 시작하고 /dev/shm·웹 콘솔도 같이 새로 뜬다
                if not self.history or row[0] - self.history[-1][0] >= LEARNER_TREND_STEP_S:
                    self.history.append(row)
                    while row[0] - self.history[0][0] > LEARNER_HISTORY_S:
                        self.history.popleft()
            return stamp, state

    def trend(self) -> Dict[str, Any]:
        with self.lock:
            return {"rows": list(self.history), "window_s": LEARNER_HISTORY_S}

    def start(self) -> None:
        """1초마다 읽는 스레드(프로세스와 함께 끝난다). 읽기 오류는 read_payload가 None으로 바꾼다."""
        def run() -> None:
            while True:
                time.sleep(1.0)
                self.sample()

        threading.Thread(target=run, name="learner-monitor", daemon=True).start()

    def status(self) -> Dict[str, Any]:
        """학습 상태와 추이 한 점, 파라미터 안내가 보는 런타임 사실(runtime, param_metadata.RUNTIME_FACTS).
        상태가 없거나 낡으면(controlsd가 없거나 멈춤) 런타임 사실은 None이다. 페이지는 폴링할 때마다 runtime으로
        안내를 갱신한다."""
        latest = self.sample()
        result: Dict[str, Any] = freshness(latest[0] if latest else None, LEARNER_STATE_MAX_AGE_S)
        if latest is not None:
            result.update(state=latest[1], trend_row=learner_trend_row(latest[1]))
        fresh = None if result["stale"] else result["state"]
        result["runtime"] = {"lateral_nn_loaded": fresh["flags"]["lateral_nn_loaded"] if fresh else None}
        return result

    def runtime_facts(self) -> Dict[str, Any]:
        return self.status()["runtime"]


class LocalizationReader(IpcReader):
    """locationd의 LocalizationState(자세 칼만 필터와 lagd 조향 지연)."""

    def __init__(self, path: str = topic_path(LOCALIZATION_STATE_TOPIC)):
        super().__init__(path, LOCALIZATION_STATE.size)

    def status(self) -> Dict[str, Any]:
        result, payload = self.read_status(LOCALIZATION_STATE_MAX_AGE_S)
        if payload is not None:
            result["state"] = decode_localization_state(payload)
        return result
