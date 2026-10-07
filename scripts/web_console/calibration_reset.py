"""카메라 캘리브레이션 카드: modeld가 ModelState에 싣는 온라인 보정 상태를 보여 주고, 해제 상태에서만 초기화를
요청한다. 요청은 파일 하나이고 modeld의 CalibrationService가 1초마다 보고 지운 뒤 처음부터 다시 수렴한다."""
from __future__ import annotations

import math
import os
import struct
from pathlib import Path
from typing import Any, Dict

from .shm_channel import IpcReader, age_s, topic_path
from .state_layout import (CONTROL_STATE_SIZE, CONTROL_STATE_TOPIC, MODEL_CALIBRATION_AT, MODEL_STATE_TOPIC,
                           control_value)

# modeld(calibration_service.cc)와 같은 변수로 바꿀 수 있다
CALIBRATION_RESET_PATH = os.environ.get("EDGEPILOT_CALIBRATION_RESET_PATH", "/dev/shm/edgepilot_calibration_reset")
CALIBRATION_STATE = struct.Struct("<Ii3f3f")  # CalibrationState: status, valid_blocks, rpy, spread
CALIBRATION_STATUS = ("uncalibrated", "calibrated", "invalid", "recalibrating")
CONTROL_STALE_S = 2.0


class CalibrationReset:
    """openpilot의 Reset Calibration과 같이 결합 중에는 받지 않는다. 학습값(paramsd·torqued)은 CAN 요레이트로
    배우므로 카메라 장착과 무관해 그대로 둔다."""

    def __init__(self, model_path: str = topic_path(MODEL_STATE_TOPIC),
                 control_path: str = topic_path(CONTROL_STATE_TOPIC), request_path: str = CALIBRATION_RESET_PATH):
        self.model = IpcReader(model_path, MODEL_CALIBRATION_AT + CALIBRATION_STATE.size)
        self.control = IpcReader(control_path, CONTROL_STATE_SIZE)
        self.request_path = Path(request_path)

    def engaged(self) -> bool:
        """controlsd가 살아 있고 결합 중이면 True. controlsd가 없거나 멈췄으면 조향도 없다."""
        latest = self.control.read_payload()
        if latest is None or age_s(latest[1]) > CONTROL_STALE_S:
            return False
        return bool(control_value(latest[2], "engaged"))

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
        status, valid_blocks, *values = CALIBRATION_STATE.unpack_from(payload, MODEL_CALIBRATION_AT)
        result.update({
            "available": True,
            "age_s": age_s(stamp),
            "status": CALIBRATION_STATUS[status] if status < len(CALIBRATION_STATUS) else str(status),
            "valid_blocks": valid_blocks,
            "rpy_deg": [round(math.degrees(v), 3) for v in values[:3]],
            "spread_deg": [round(math.degrees(v), 3) for v in values[3:]],
        })
        return result

    def request(self) -> Dict[str, Any]:
        if self.engaged():
            raise PermissionError("결합 중에는 초기화할 수 없습니다. 먼저 해제하세요.")
        self.request_path.parent.mkdir(parents=True, exist_ok=True)
        self.request_path.touch()
        return self.status()
