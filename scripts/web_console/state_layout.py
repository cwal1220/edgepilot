"""웹 콘솔이 위치로 읽는 ModelState·ControlState 필드. src/common/ipc_messages.h의 EDGEPILOT_*_AT 고정값과
같아야 하고, check_web_console.py가 대조한다. BEV 페이지는 이 위치를 /api/bev로 받아 직접 읽는다."""
from __future__ import annotations

import struct
from typing import Any

MODEL_STATE_TOPIC = "/edgepilot_model_state"
CONTROL_STATE_TOPIC = "/edgepilot_control_state"
MODEL_STATE_SIZE = 3576
CONTROL_STATE_SIZE = 240

MODEL_STATE_AT = {
    "model_timestamp_ns": 16, "valid": 28, "plan": 304, "lanes": 700, "lane_probabilities": 2284,
    "road_edges": 2316, "road_edge_stds": 3108, "lead": 3148, "gas_press_probs": 3528,
}
MODEL_CALIBRATION_AT = 3224  # offsetof(ModelState, calibration)
CONTROL_STATE_AT = {
    "timestamp_ns": 0, "enabled": 8, "engaged": 12, "active": 16, "vehicle_fresh": 32,
    "left_blinker": 40, "right_blinker": 44, "gear": 52, "cluster_speed_kph": 56,
    "desired_curvature": 72, "actual_curvature": 76, "normalized_output": 80,
    "departure_alert_type": 144, "green_light_alert_armed": 152, "hud_flags": 184, "ego_speed_kph": 232,
}
HUD_FLAG_BITS = {"Laneless": 0, "SteerPaused": 7, "BrakeLights": 11}  # ipc_messages.h kHudFlag<이름>의 비트
GEAR_PARK = 0  # car/can_frame.h kGearPark


def control_value(payload: bytes, name: str, fmt: str = "<I") -> Any:
    """ControlState 페이로드에서 필드 하나."""
    return struct.unpack_from(fmt, payload, CONTROL_STATE_AT[name])[0]
