"""실시간 학습 탭의 상태: controlsd가 내는 paramsd·torqued 학습값(LearnerState)과 그 10분 추이, locationd가 내는
자세·조향 지연(LocalizationState). 필드 배치는 src/common/ipc_messages.h와 같고 check_web_console.py가 대조한다."""
from __future__ import annotations

import collections
import math
import threading
from typing import Any, Dict

from .shm_channel import IpcReader, age_s, flag_names, struct_layout, topic_path, unpack_fields

LEARNER_TOPIC = "/edgepilot_learner_state"
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
    "localizer_inputs",
)
LEARNER_HISTORY_S = 600
GRAVITY = 9.81

LOCALIZATION_TOPIC = "/edgepilot_localization"
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


class LearnerStateReader(IpcReader):
    """controlsd의 LearnerState."""

    def __init__(self, path: str = topic_path(LEARNER_TOPIC)):
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


def manual_lateral_values(steering: Dict[str, Any]) -> Dict[str, Any]:
    """학습값과 나란히 보여 줄 수동값(params/steering.json)."""
    return {
        "steer_ratio": steering.get("steer_ratio"),
        "tire_stiffness_factor": steering.get("tire_stiffness_factor"),
        "angle_offset_deg": steering.get("angle_offset_deg"),
        "torque_lat_accel_offset": steering.get("torque_lat_accel_offset"),
        "live_bank_compensation": steering.get("live_bank_compensation"),
    }


class LearnerMonitor:
    """1초마다 최신 학습 상태를 읽어 10분 추이를 남긴다. 탭을 늦게 열어도 추이가 보인다."""

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

    def trend(self) -> Dict[str, Any]:
        with self.lock:
            return {"rows": list(self.history), "window_s": LEARNER_HISTORY_S}

    def start(self) -> None:
        def run() -> None:
            while not self._stop.wait(1.0):
                self.sample()

        threading.Thread(target=run, name="learner-monitor", daemon=True).start()

    def stop(self) -> None:
        self._stop.set()

    def status(self, steering: Dict[str, Any]) -> Dict[str, Any]:
        latest = self.sample()
        result: Dict[str, Any] = {"available": latest is not None, "manual": manual_lateral_values(steering)}
        if latest is not None:
            _, stamp, state = latest
            result.update(age_s=age_s(stamp), state=state, trend_row=learner_trend_row(state))
        return result


class LocalizationReader(IpcReader):
    """locationd의 LocalizationState(자세 칼만 필터와 lagd 조향 지연)."""

    def __init__(self, path: str = topic_path(LOCALIZATION_TOPIC)):
        super().__init__(path, LOCALIZATION_STATE.size)

    def status(self, steering: Dict[str, Any]) -> Dict[str, Any]:
        result: Dict[str, Any] = {"available": False, "manual_delay_s": steering.get("steer_actuator_delay")}
        latest = self.read_payload()
        if latest is not None:
            _, stamp, payload = latest
            result.update(available=True, age_s=age_s(stamp), state=decode_localization_state(payload))
        return result
