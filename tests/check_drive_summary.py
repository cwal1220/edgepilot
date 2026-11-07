#!/usr/bin/env python3
"""tools/model/drive_summary.py: 쓰는 hud_flags 비트가 C++(ipc_messages.h)와 같은지, 합성한 짧은 route에서 결합 비율,
해제 사유, 모델 경로 대체, 운전자 조작, 85° 정지, 빠진 모델 프레임, Panda 증가분을 맞게 세는지 본다.
ctest가 저장소 루트에서 돌린다. numpy가 없으면 77로 끝나고 ctest는 건너뛴 것으로 센다."""

import re
import struct
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools" / "model"))

try:
    import numpy as np
except ModuleNotFoundError:
    print("skipped: numpy is not installed")
    sys.exit(77)

from cpp_headers import IPC  # noqa: E402
import drive_summary as ds  # noqa: E402
import recording_reader as rr  # noqa: E402

SECOND = 1_000_000_000


def write_route(root: Path) -> Path:
    """10초 route: 50 km/h로 달리며 1~8초 결합(8초에 door_open으로 해제), 3~4초 모델 경로 대체, 5~5.5초 운전자가
    조향 명령과 반대로 핸들을 민다, 6~6.5초 85° 정지. 모델은 20 Hz인데 프레임 하나가 빠진다. Panda 차단 송신 2 → 5."""
    route = root / "2026-11-06--00-00-00-000"
    (route / "events").mkdir(parents=True)
    records = []

    def add(t_ns: int, rtype: int, payload: bytes) -> None:
        records.append(rr.EVENT_RECORD_HEADER.pack(t_ns, rtype, 0, len(payload)) + payload)

    def padded(row: np.ndarray, size: int) -> bytes:
        data = row.tobytes()
        return data + b"\0" * (size - len(data))

    t0 = 5 * SECOND
    for tick in range(1000):
        t = t0 + tick * 10_000_000
        s = tick / 100
        c = np.zeros(1, rr.CONTROL_STATE)
        c["timestamp_ns"] = t
        c["ego_speed_kph"] = 50.0
        c["engaged"] = c["active"] = 1 if 1.0 <= s < 8.0 else 0
        c["active_block"] = b"door_open" if s >= 8.0 else b""
        flags = 0
        if 3.0 <= s < 4.0:
            flags |= ds.HUD_LANELESS
        if 5.0 <= s < 5.5:
            flags |= ds.HUD_STEERING_PRESSED
            c["driver_torque"] = -200
        c["apply_torque"] = 100 if 1.0 <= s < 8.0 else 0
        if 6.0 <= s < 6.5:
            flags |= ds.HUD_STEER_PAUSED
        c["hud_flags"] = flags
        add(t, rr.RECORD_CONTROL_STATE, padded(c, rr.CONTROL_STATE_SIZE))
        if tick % 5 == 0 and tick != 500:  # 프레임 100이 빠진다
            m = np.zeros(1, rr.MODEL_STATE)
            m["frame_id"] = tick // 5
            m["capture_timestamp_ns"] = t
            m["model_execution_ms"] = 18.0
            add(t + 60_000_000, rr.RECORD_MODEL_STATE, m.tobytes())
        if tick % 50 == 0:
            p = np.zeros(1, rr.PANDA_STATE)
            p["timestamp_ns"] = t
            p["blocked_msg_cnt"] = 2 if s < 5.0 else 5
            p["voltage"] = 13_500
            add(t, rr.RECORD_PANDA_STATE, padded(p, rr.PANDA_STATE_SIZE))
    header = struct.pack("<8sIIQQ", rr.EVENT_LOG_MAGIC, rr.RECORDING_VERSION, struct.calcsize("<8sIIQQ"), 0, 0)
    (route / "events" / "000.bin").write_bytes(header + b"".join(records))
    return route


class DriveSummary(unittest.TestCase):
    def test_hud_bits_match_the_runtime(self):
        for name, value in (("Laneless", ds.HUD_LANELESS), ("SoftDisabling", ds.HUD_SOFT_DISABLING),
                            ("SteerSaturated", ds.HUD_STEER_SATURATED),
                            ("LaneChangePending", ds.HUD_LANE_CHANGE_PENDING),
                            ("LaneChanging", ds.HUD_LANE_CHANGING), ("SteerPaused", ds.HUD_STEER_PAUSED),
                            ("SteerPausedByDriver", ds.HUD_STEER_PAUSED_BY_DRIVER),
                            ("SteeringPressed", ds.HUD_STEERING_PRESSED)):
            bit = int(re.search(r"kHudFlag" + name + r" = 1U << (\d+);", IPC).group(1))
            self.assertEqual(value, 1 << bit, name)

    def test_synthetic_route(self):
        with tempfile.TemporaryDirectory() as tmp:
            s = ds.summarize(write_route(Path(tmp)))
        self.assertAlmostEqual(s["minutes"], 9.99 / 60, places=3)
        self.assertAlmostEqual(s["engaged_share_moving"], 0.7, places=2)
        self.assertEqual(s["disengagements"], [{"t_s": 8.0, "kph": 50.0, "reason": "door_open"}])
        self.assertAlmostEqual(s["laneless_share"], 1 / 7, places=2)
        self.assertEqual(s["presses"]["episodes"], 1)
        self.assertEqual(s["presses"]["opposing_without_blinker"], 1, "토크 +100에 운전자 -200")
        self.assertEqual(s["presses"]["opposing_in_laneless"], 0)
        self.assertEqual(s["pauses_85"], {"count": 1, "by_driver": 0})
        self.assertEqual(s["model"]["lost_frames"], 1)
        self.assertAlmostEqual(s["model"]["latency_ms_p50"], 60.0, places=3)
        self.assertEqual(s["panda"]["blocked_msgs"], 3)


if __name__ == "__main__":
    unittest.main()
