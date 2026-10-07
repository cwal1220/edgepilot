#!/usr/bin/env python3
"""웹 콘솔(scripts/web_console) 검사. stdlib만 쓴다(fastapi 없이): 파라미터 저장, 공유 메모리 상태 읽기와 그
배치가 src/common/ipc_messages.h와 같은지, Panda 플래싱 조건, BEV 스트림, 페이지 파일과 서버 API가 맞는지."""
import asyncio
import gzip
import json
import os
import re
import struct
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scripts.web_console.backlight import duty_cycle_ns
from scripts.web_console.bev_stream import BEV_FRAME, BEV_FRAME_MAGIC, bev_frames, bev_layout
from scripts.web_console.calibration_reset import CALIBRATION_RESET_PATH, CALIBRATION_STATE, CalibrationReset
from scripts.web_console.learner_monitor import (
    LEARNER_FIELDS,
    LEARNER_FLAGS,
    LEARNER_STATE,
    LEARNER_TOPIC,
    LOCALIZATION_FIELDS,
    LOCALIZATION_FLAGS,
    LOCALIZATION_INPUT_FLAGS,
    LOCALIZATION_STATE,
    LOCALIZATION_TOPIC,
    LearnerMonitor,
    LearnerStateReader,
    LocalizationReader,
    manual_lateral_values,
)
from scripts.web_console.panda_update import (
    PANDA_APP_MAX_BYTES,
    PANDA_FLASH_REQUEST_PATH,
    PANDA_SIGNATURE_BYTES,
    PANDA_STATUS_PATH,
    PandaUpdate,
    panda_image_info,
)
from scripts.web_console.param_metadata import PARAM_GROUPS, PARAM_METADATA
from scripts.web_console.param_store import ParamStore, group_paths
from scripts.web_console.process_status import (
    MANAGER_STATE_HEAD,
    MANAGER_STATE_TOPIC,
    MAX_PROCESSES,
    PROCESS_STATE,
    ProcessStatus,
)
from scripts.web_console.shm_channel import IPC_HEADER, IPC_MAGIC, IPC_VERSION, boottime_ns
from scripts.web_console.state_layout import (
    CONTROL_STATE_AT,
    CONTROL_STATE_SIZE,
    CONTROL_STATE_TOPIC,
    GEAR_PARK,
    HUD_FLAG_BITS,
    MODEL_CALIBRATION_AT,
    MODEL_STATE_AT,
    MODEL_STATE_SIZE,
    MODEL_STATE_TOPIC,
)
from scripts.web_console.static_assets import STATIC_DIR, StaticAssets

IPC_MESSAGES = ROOT / "src" / "common" / "ipc_messages.h"


def cpp(path: Path = IPC_MESSAGES) -> str:
    return path.read_text(encoding="utf-8")


def cpp_offsets(macro: str) -> dict:
    """EDGEPILOT_<macro>_AT(필드, 위치) 고정값들."""
    return {name: int(at) for name, at in re.findall(rf"EDGEPILOT_{macro}_AT\((\w+), (\d+)\);", cpp())}


def cpp_size(struct_name: str) -> int:
    return int(re.search(rf"sizeof\({struct_name}\) == (\d+)", cpp()).group(1))


def snake(name: str) -> str:
    return re.sub(r"(?<!^)(?=[A-Z])", "_", name).lower()


def publish(path: Path, payload: bytes, seq: int = 2, stamp: int | None = None) -> None:
    """LatestChannel 하나를 쓴다(seq가 홀수면 쓰는 중)."""
    stamp = boottime_ns() if stamp is None else stamp
    path.write_bytes(IPC_HEADER.pack(IPC_MAGIC, IPC_VERSION, len(payload), 0, seq, stamp, len(payload), 0) + payload)


def pack_fields(layout, fields, values) -> bytes:
    flat = []
    for name, fmt in fields:
        value = values.get(name, [0] * int(fmt[:-1]) if len(fmt) > 1 else 0)
        flat.extend(value if isinstance(value, list) else [value])
    return layout.pack(*flat)


def control_payload(**values) -> bytes:
    """ControlState: 이름=값(정수 필드), gear는 부호 있는 정수, *_kph는 float."""
    payload = bytearray(CONTROL_STATE_SIZE)
    for name, value in values.items():
        fmt = "<f" if name.endswith("_kph") else "<i" if name == "gear" else "<I"
        struct.pack_into(fmt, payload, CONTROL_STATE_AT[name], value)
    return bytes(payload)


class FakeBacklight:
    def __init__(self):
        self.applied = []

    def apply(self, document):
        self.applied.append(dict(document))

    def status(self):
        latest = self.applied[-1] if self.applied else {}
        return {"available": True, "enabled": latest.get("enabled", True),
                "brightness_percent": latest.get("brightness_percent", 100), "mode": "test", "error": ""}


# ------------------------------------------------------------ 파라미터
class ParamStoreTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.paths = group_paths(self.root / "params")
        documents = {
            "steering": {"gain": 10, "enabled": True},
            "driving": {"delay": 0.4},
            "adaptive_cruise": {"following_time_s": 1.8},
            "recording": {"enabled": False},
            "display": {"enabled": True, "brightness_percent": 100},
        }
        self.paths["steering"].parent.mkdir()
        for group, document in documents.items():
            self.paths[group].write_text(json.dumps(document), encoding="utf-8")
        self.notified = []

        def notify(name):
            self.notified.append(name)
            return [123]

        self.notify = notify
        self.store = ParamStore(self.paths, notify=notify)

    def tearDown(self):
        self.temporary.cleanup()

    def test_atomic_single_value_update_signals_the_reader(self):
        result = self.store.update("steering", {"gain": 12})
        self.assertEqual(result["notified"], [123])
        self.assertEqual(self.notified, ["controlsd"])
        self.assertEqual(self.store.read_group("steering"), {"gain": 12, "enabled": True})
        self.assertEqual(list(self.paths["steering"].parent.glob("*.tmp")), [])

    def test_unknown_parameter_is_rejected(self):
        with self.assertRaises(KeyError):
            self.store.update("driving", {"unknown": 1})
        with self.assertRaises(KeyError):
            self.store.update("nonexistent", {"delay": 1})
        with self.assertRaises(ValueError):
            self.store.update("driving", {})
        self.assertEqual(self.notified, [])
        self.assertEqual(self.store.read_group("driving"), {"delay": 0.4})

    def test_files_read_by_their_own_process_signal_nobody(self):
        """recordd·overlayd·modeld는 파일을 스스로 다시 읽는다."""
        self.assertEqual(self.store.update("recording", {"enabled": True})["notified"], [])
        self.assertEqual(self.notified, [])

    def test_display_update_applies_hardware_without_signaling(self):
        backlight = FakeBacklight()
        store = ParamStore(self.paths, notify=self.notify, backlight=backlight)
        self.assertEqual(backlight.applied, [{"enabled": True, "brightness_percent": 100}], "applied at start")
        backlight.applied.clear()
        result = store.update("display", {"brightness_percent": 35})
        self.assertEqual(result["notified"], [])
        self.assertEqual(backlight.applied, [{"enabled": True, "brightness_percent": 35}])
        self.assertEqual(store.read_group("display")["brightness_percent"], 35)

    def test_display_update_without_backlight_is_refused(self):
        with self.assertRaises(ValueError):
            self.store.update("display", {"brightness_percent": 35})
        self.assertEqual(self.store.read_group("display")["brightness_percent"], 100)

    def test_alert_volume_update_leaves_backlight_alone(self):
        self.paths["display"].write_text(
            json.dumps({"enabled": True, "brightness_percent": 80, "alert_volume_percent": 70}), encoding="utf-8")
        backlight = FakeBacklight()
        store = ParamStore(self.paths, notify=self.notify, backlight=backlight)
        backlight.applied.clear()
        result = store.update("display", {"alert_volume_percent": 30})
        self.assertEqual(result["params"]["alert_volume_percent"], 30)
        self.assertEqual(backlight.applied, [], "volume is read by overlayd, not the backlight")
        store.update("display", {"brightness_percent": 60})
        self.assertEqual(backlight.applied[-1]["brightness_percent"], 60)
        self.assertEqual(json.loads(self.paths["display"].read_text())["alert_volume_percent"], 30)

    def test_display_pwm_duty_scales_by_board_maximum(self):
        self.assertEqual(duty_cycle_ns(100), 95_000)
        self.assertEqual(duty_cycle_ns(51), 48_450)
        self.assertEqual(duty_cycle_ns(1), 950)
        self.assertEqual(duty_cycle_ns(0), 0)
        self.assertEqual(duty_cycle_ns(100, max_percent=100), 100_000)

    def test_runtime_keys_follow_the_defaults(self):
        """빠진 키는 기본값으로 채우고 기본값에서 사라진 키는 지운다. 남은 키의 튜닝은 그대로."""
        defaults = self.root / "adaptive.defaults.json"
        defaults.write_text(json.dumps({"following_time_s": 1.8, "deceleration_rate_kph_per_s": 1.5}),
                            encoding="utf-8")
        self.paths["adaptive_cruise"].write_text(json.dumps({"following_time_s": 2.2, "retired": 1}),
                                                 encoding="utf-8")
        store = ParamStore(self.paths, {"adaptive_cruise": defaults}, notify=self.notify)
        self.assertEqual(store.read_group("adaptive_cruise"),
                         {"following_time_s": 2.2, "deceleration_rate_kph_per_s": 1.5})
        self.assertEqual(store.defaults(), {"adaptive_cruise": {"following_time_s": 1.8,
                                                                "deceleration_rate_kph_per_s": 1.5}})

    def test_defaults_sit_next_to_the_params_directory(self):
        """EDGEPILOT_PARAMS_DIR(없으면 params/)와 그 옆의 <이름>.defaults/(업로드 스크립트가 채운다)."""
        defaults = group_paths(self.root / "params.defaults")
        defaults["driving"].parent.mkdir()
        defaults["driving"].write_text(json.dumps({"delay": 0.3, "added": True}), encoding="utf-8")
        with mock.patch.dict(os.environ, {"EDGEPILOT_PARAMS_DIR": str(self.root / "params")}):
            store = ParamStore(notify=self.notify)
        self.assertEqual(store.paths, self.paths)
        self.assertEqual(store.read_group("driving"), {"delay": 0.4, "added": True})
        self.assertEqual(store.snapshot()["defaults"], {"driving": {"delay": 0.3, "added": True}})

    def test_snapshot_has_what_the_page_reads(self):
        snapshot = self.store.snapshot()
        self.assertEqual(set(snapshot), {"groups", "params", "defaults", "metadata", "paths"})
        self.assertEqual(list(snapshot["groups"]), list(PARAM_GROUPS), "tab order")
        for group, spec in snapshot["groups"].items():
            self.assertEqual(set(spec), {"label", "note", "sections", "notify"}, group)
        self.assertEqual(snapshot["paths"]["steering"], str(self.paths["steering"]))


class ParamMetadataTest(unittest.TestCase):
    def test_repository_params_have_complete_ui_metadata(self):
        for group, spec in PARAM_GROUPS.items():
            params = json.loads((ROOT / "params" / spec["file"]).read_text(encoding="utf-8"))
            metadata = PARAM_METADATA[group]
            self.assertEqual(set(params), set(metadata), group)
            for key, value in params.items():
                meta = metadata[key]
                for field in ("label", "section", "description", "increase", "decrease"):
                    self.assertTrue(meta.get(field), f"{group}.{key}.{field}")
                self.assertIn(meta["section"], spec["sections"], f"{group}.{key}")
                if isinstance(value, (int, float)) and not isinstance(value, bool):
                    for field in ("step", "min", "max"):
                        self.assertIsInstance(meta.get(field), (int, float), f"{group}.{key}.{field}")
                    self.assertLessEqual(meta["min"], value, f"{group}.{key}")
                    self.assertLessEqual(value, meta["max"], f"{group}.{key}")
            used = {meta["section"] for meta in metadata.values()}
            self.assertEqual([name for name in spec["sections"] if name not in used], [], f"{group}: empty sections")

    def test_learner_switches_live_on_the_learner_tab(self):
        """학습 스위치는 학습값을 보면서 켜도록 실시간 학습 탭에만 둔다(조향 탭에서 숨김)."""
        hidden = {key for key, meta in PARAM_METADATA["steering"].items() if meta.get("hidden")}
        self.assertEqual(hidden, {"use_live_vehicle_params", "use_live_torque_params", "use_live_delay",
                                  "use_locationd_learner_inputs"})
        page = (STATIC_DIR / "learner_view.js").read_text(encoding="utf-8")
        for key in hidden:
            self.assertIn(f'"{key}"', page)

    def test_ui_ranges_match_runtime_clamps(self):
        """The loaders clamp to their Json*Field tables; the editor must show
        the same min/max or it accepts values the runtime silently changes."""
        row = re.compile(r'\{"(\w+)",\s*(-?[\d.]+)f?,\s*(-?[\d.]+)f?,\s*&\w+::\w+\}')
        tables = {
            "steering": ("src/controls/control_params.cc", ("kSteeringInts", "kSteeringFloats")),
            "driving": ("src/controls/control_params.cc", ("kDrivingInts", "kDrivingFloats")),
            "adaptive_cruise": ("src/controls/adaptive_cruise.cc", ("kAdaptiveInts", "kAdaptiveFloats")),
            # display는 백라이트(backlight.py) 항목도 있어 런타임이 읽는 키만 대조한다
            "display": ("src/common/device_settings.h", ("kDeviceSettingsFloats",)),
        }
        for group, (source, names) in tables.items():
            text = (ROOT / source).read_text(encoding="utf-8")
            runtime = {}
            for name in names:
                body = re.search(name + r"\[\] = \{(.*?)\n\};", text, re.S)
                self.assertIsNotNone(body, f"{source}: {name}")
                runtime.update({key: (float(low), float(high)) for key, low, high in row.findall(body.group(1))})
            self.assertTrue(runtime, f"{source}: no rows parsed")
            ui = {key: meta for key, meta in PARAM_METADATA[group].items() if "min" in meta}
            if group == "display":
                ui = {key: meta for key, meta in ui.items() if key in runtime}
            self.assertEqual(set(runtime), set(ui), group)
            for key, (low, high) in runtime.items():
                self.assertEqual((float(ui[key]["min"]), float(ui[key]["max"])), (low, high), f"{group}.{key}")

    def test_ui_text_names_no_particular_car(self):
        """다른 차도 붙일 수 있게 화면 문구는 차종을 말하지 않는다."""
        texts = [json.dumps(PARAM_METADATA, ensure_ascii=False), json.dumps(PARAM_GROUPS, ensure_ascii=False)]
        texts += [path.read_text(encoding="utf-8") for path in STATIC_DIR.glob("*.html")]
        texts += [path.read_text(encoding="utf-8") for path in STATIC_DIR.glob("*_view.js")]
        texts += [(STATIC_DIR / name).read_text(encoding="utf-8") for name in ("console.js", "panda_card.js")]
        for text in texts:
            self.assertNotRegex(text, r"(?i)\bk7\b|그랜저|grandeur")
        self.assertEqual([path.name for path in STATIC_DIR.rglob("*") if "k7" in path.name.lower()], [])


# ------------------------------------------------------------ 학습 상태
def learner_payload(**values):
    return pack_fields(LEARNER_STATE, LEARNER_FIELDS, values)


class LearnerStateTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.path = Path(self.temporary.name) / "edgepilot_learner_state"

    def tearDown(self):
        self.temporary.cleanup()

    def publish(self, seq, **values):
        publish(self.path, learner_payload(**values), seq=seq, stamp=123)

    def test_layout_matches_cpp(self):
        """ipc_messages.h의 크기·offsetof 고정값·플래그 비트와 Python 필드 배치가 같아야 한다."""
        self.assertEqual(LEARNER_STATE.size, cpp_size("LearnerState"))
        offsets, offset = {}, 0
        for name, fmt in LEARNER_FIELDS:
            offsets[name] = offset
            offset += struct.calcsize("<" + fmt)
        asserted = cpp_offsets("LEARNER_STATE")
        self.assertGreaterEqual(len(asserted), 8)
        for name, expected in asserted.items():
            self.assertEqual(offsets[name], expected, name)
        bits = re.findall(r"constexpr uint32_t kLearner(\w+) = 1U << (\d+);", cpp())
        self.assertEqual([snake(name) for name, _ in bits], list(LEARNER_FLAGS))
        self.assertEqual([int(bit) for _, bit in bits], list(range(len(LEARNER_FLAGS))))
        self.assertIn(f'kLearnerStateTopic[] = "{LEARNER_TOPIC}";', cpp())

    def test_reader_decodes_and_skips_torn_writes(self):
        self.publish(4, timestamp_ns=5_000_000_000, flags=0b1000000011, steer_ratio=14.88,
                     bucket_points=[12, 204, 464, 1440, 1500, 1036, 292, 49])
        reader = LearnerStateReader(str(self.path))
        seq, _, state = reader.read()
        self.assertEqual(seq, 4)
        self.assertAlmostEqual(state["steer_ratio"], 14.88, places=5)
        self.assertEqual(state["bucket_points"][4], 1500)
        self.assertTrue(state["flags"]["vehicle_inputs_ok"] and state["flags"]["vehicle_valid"])
        self.assertTrue(state["flags"]["use_vehicle"] and not state["flags"]["use_torque"])
        self.publish(5, steer_ratio=1.0)  # 쓰는 중
        self.assertIsNone(reader.read())
        self.path.unlink()
        self.assertIsNone(reader.read())

    def test_monitor_keeps_one_trend_row_per_publish(self):
        monitor = LearnerMonitor(LearnerStateReader(str(self.path)))
        self.assertFalse(monitor.status({})["available"])
        self.publish(2, timestamp_ns=1_000_000_000, lat_accel_factor=4.44)
        monitor.sample()
        monitor.sample()
        self.publish(4, timestamp_ns=2_000_000_000, lat_accel_factor=4.40)
        steering = json.loads((ROOT / "params" / "steering.json").read_text(encoding="utf-8"))
        status = monitor.status(steering)
        self.assertTrue(status["available"])
        self.assertEqual(status["manual"]["steer_ratio"], steering["steer_ratio"])
        self.assertEqual([row[0] for row in monitor.trend()["rows"]], [1.0, 2.0])
        self.assertAlmostEqual(status["trend_row"][7], 4.40, places=5)
        self.publish(6, timestamp_ns=500_000_000)  # 시각이 거꾸로: 다른 부팅
        monitor.sample()
        self.assertEqual([row[0] for row in monitor.trend()["rows"]], [0.5])

    def test_manual_values_are_what_the_page_compares(self):
        steering = {"steer_ratio": 14.9, "tire_stiffness_factor": 0.83, "angle_offset_deg": -1.5,
                    "torque_lat_accel_offset": -0.06, "live_bank_compensation": True, "torque_kp": 0.8}
        self.assertEqual(manual_lateral_values(steering), {key: value for key, value in steering.items()
                                                           if key != "torque_kp"})
        page = (STATIC_DIR / "learner_view.js").read_text(encoding="utf-8")
        self.assertEqual(set(re.findall(r"\bm\.(\w+)", page)), set(manual_lateral_values({})))


class LocalizationStateTest(unittest.TestCase):
    def test_layout_matches_cpp(self):
        self.assertEqual(LOCALIZATION_STATE.size, cpp_size("LocalizationState"))
        flags = re.findall(r"constexpr uint32_t kLocalization(\w+) = 1U << (\d+);", cpp())
        state_flags = [int(bit) for name, bit in flags if not name.startswith(("Invalid", "CameraGuarded"))]
        input_flags = [int(bit) for name, bit in flags if name.startswith(("Invalid", "CameraGuarded"))]
        self.assertEqual(state_flags, list(range(len(LOCALIZATION_FLAGS))))
        self.assertEqual(input_flags, list(range(len(LOCALIZATION_INPUT_FLAGS))))
        self.assertIn(f'kLocalizationStateTopic[] = "{LOCALIZATION_TOPIC}";', cpp())

    def test_reader_decodes(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "edgepilot_localization"
            reader = LocalizationReader(str(path))
            self.assertFalse(reader.status({})["available"])
            publish(path, pack_fields(LOCALIZATION_STATE, LOCALIZATION_FIELDS, {
                "flags": 0b100001, "lag_status": 1, "input_flags": 0b1010, "lateral_delay_s": 0.42,
                "lag_valid_blocks": 6, "angular_velocity_calib": [0.0, 0.0, 0.1]}))
            result = reader.status({"steer_actuator_delay": 0.34})
            self.assertTrue(result["available"])
            self.assertEqual(result["manual_delay_s"], 0.34)
            state = result["state"]
            self.assertAlmostEqual(state["lateral_delay_s"], 0.42, places=5)
            self.assertEqual(state["lag_valid_blocks"], 6)
            self.assertAlmostEqual(state["angular_velocity_calib"][2], 0.1, places=5)
            self.assertTrue(state["flags"]["filter_valid"] and state["flags"]["lag_restored"])
            self.assertFalse(state["flags"]["inputs_ok"])
            self.assertTrue(state["input_flags"]["gyro_invalid"] and state["input_flags"]["camera_guarded"])
            self.assertFalse(state["input_flags"]["camera_invalid"])


# ------------------------------------------------------------ 카메라 보정
class CalibrationResetTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        root = Path(self.temporary.name)
        self.model = root / "edgepilot_model_state"
        self.control = root / "edgepilot_control_state"
        self.request = root / "edgepilot_calibration_reset"
        self.calibration = CalibrationReset(str(self.model), str(self.control), str(self.request))

    def tearDown(self):
        self.temporary.cleanup()

    def publish_model(self, status, blocks, rpy_deg):
        payload = bytearray(MODEL_STATE_SIZE)
        rad = [x * 3.141592653589793 / 180 for x in rpy_deg]
        CALIBRATION_STATE.pack_into(payload, MODEL_CALIBRATION_AT, status, blocks, *rad, 0.1, 0.2, 0.3)
        publish(self.model, bytes(payload))

    def publish_control(self, engaged, stamp=None):
        publish(self.control, control_payload(engaged=engaged, vehicle_fresh=1), stamp=stamp)

    def test_layout_and_request_path_match_cpp(self):
        offset = re.search(r"offsetof\(ModelState, calibration\) == (\d+)", cpp())
        self.assertEqual(int(offset.group(1)), MODEL_CALIBRATION_AT)
        self.assertEqual(CALIBRATION_STATE.size, 32)
        service = cpp(ROOT / "src" / "model" / "calibration_service.cc")
        self.assertIn(f'"EDGEPILOT_CALIBRATION_RESET_PATH",\n                                     "{CALIBRATION_RESET_PATH}"',
                      service)

    def test_status_decodes_model_state(self):
        self.assertFalse(self.calibration.status()["available"])
        self.publish_model(1, 7, [0.0, 0.29, -0.18])
        status = self.calibration.status()
        self.assertTrue(status["available"])
        self.assertEqual((status["status"], status["valid_blocks"]), ("calibrated", 7))
        self.assertAlmostEqual(status["rpy_deg"][1], 0.29, places=3)
        self.assertAlmostEqual(status["rpy_deg"][2], -0.18, places=3)

    def test_reset_is_refused_while_engaged(self):
        self.publish_control(engaged=1)
        with self.assertRaises(PermissionError):
            self.calibration.request()
        self.assertFalse(self.request.exists())
        self.publish_control(engaged=0)
        self.assertTrue(self.calibration.request()["reset_pending"])
        self.assertTrue(self.request.exists())

    def test_stale_or_missing_controlsd_does_not_block(self):
        self.assertFalse(self.calibration.engaged())
        self.publish_control(engaged=1, stamp=boottime_ns() - 5_000_000_000)
        self.assertFalse(self.calibration.engaged())


# ------------------------------------------------------------ Panda 펌웨어
def panda_image(version="EDGE-f9907afb-DEBUG", body=1024):
    """sign.py의 꼴: [본문 길이][... 버전 ...][VERS][2] 뒤에 서명 128바이트."""
    data = bytearray(b"\x11" * body)
    struct.pack_into("<I", data, 0, body)
    data[63:64 + len(version) + 1] = b"\0" + version.encode() + b"\0"
    data[body - 8:body] = b"VERS" + struct.pack("<I", 2)
    return bytes(data) + b"\xab" * PANDA_SIGNATURE_BYTES


class PandaUpdateTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        root = Path(self.temporary.name)
        self.status = root / "edgepilot_panda_status.json"
        self.request = root / "edgepilot_panda_flash"
        self.image = root / "panda.bin.signed"
        self.control = root / "edgepilot_control_state"
        self.panda = PandaUpdate(str(self.status), str(self.request), str(self.image), str(self.control))

    def tearDown(self):
        self.temporary.cleanup()

    def publish_pandad(self, mode="app", version="DEV-23456789-DEBUG", hw_type=3, flash=None, stamp=None):
        self.status.write_text(json.dumps({
            "stamp_ns": boottime_ns() if stamp is None else stamp, "mode": mode, "serial": "abc",
            "hw_type": hw_type, "hw_name": "black panda", "firmware_version": version,
            "flash": flash or {"state": "idle", "step": "", "percent": 0, "error": "", "detail": "",
                               "version": "", "stamp_ns": 0}}))

    def publish_control(self, gear=GEAR_PARK, speed=0.0, vehicle_fresh=1, engaged=0, stamp=None):
        publish(self.control, control_payload(gear=gear, ego_speed_kph=speed, vehicle_fresh=vehicle_fresh,
                                              engaged=engaged), stamp=stamp)

    def test_constants_match_cpp(self):
        can_frame = cpp(ROOT / "src" / "car" / "can_frame.h")
        self.assertEqual(int(re.search(r"kGearPark = (\d+);", can_frame).group(1)), GEAR_PARK)
        protocol = cpp(ROOT / "src" / "panda" / "panda_protocol.h")
        self.assertIn(f"kPandaSignatureBytes = {PANDA_SIGNATURE_BYTES};", protocol)
        self.assertIn("kPandaAppMaxBytes = 3 * 16 * 1024;", protocol)
        self.assertEqual(PANDA_APP_MAX_BYTES, 3 * 16 * 1024)
        pandad = cpp(ROOT / "src" / "panda" / "pandad.cc")
        self.assertIn(f'kFirmwareStatusPath[] = "{PANDA_STATUS_PATH}";', pandad)
        self.assertIn(f'kFlashRequestPath[] = "{PANDA_FLASH_REQUEST_PATH}";', pandad)
        self.assertIn('kDefaultFirmwarePath[] = "firmware/panda.bin.signed";', pandad)

    def test_image_info_follows_panda_firmware_rules(self):
        self.assertFalse(panda_image_info(str(self.image))["present"])
        self.image.write_bytes(panda_image())
        info = panda_image_info(str(self.image))
        self.assertTrue(info["valid"], info["error"])
        self.assertEqual((info["version"], info["size"]), ("EDGE-f9907afb-DEBUG", 1024 + PANDA_SIGNATURE_BYTES))
        broken = bytearray(panda_image())
        broken[0] ^= 4
        self.image.write_bytes(bytes(broken))
        self.assertFalse(panda_image_info(str(self.image))["valid"])
        self.image.write_bytes(panda_image(body=PANDA_APP_MAX_BYTES))
        self.assertFalse(panda_image_info(str(self.image))["valid"])

    def test_flash_is_requested_only_when_parked(self):
        self.image.write_bytes(panda_image())
        self.publish_pandad()
        self.publish_control(gear=5)
        status = self.panda.status()
        self.assertEqual(status["blocker"], "not_park")
        self.assertFalse(status["up_to_date"])
        with self.assertRaises(PermissionError):
            self.panda.request("EDGE-f9907afb-DEBUG")
        self.assertFalse(self.request.exists())
        for blocker, kwargs in (("moving", {"speed": 3.0}), ("engaged", {"engaged": 1}),
                                ("vehicle_stale", {"vehicle_fresh": 0}),
                                ("control_stale", {"stamp": boottime_ns() - 5_000_000_000})):
            self.publish_control(**kwargs)
            self.assertEqual(self.panda.status()["blocker"], blocker)

        self.publish_control()
        with self.assertRaises(PermissionError):
            self.panda.request("EDGE-00000000-DEBUG")  # 페이지가 본 이미지가 아니다
        status = self.panda.request("EDGE-f9907afb-DEBUG")
        self.assertTrue(status["request_pending"])
        self.assertEqual(self.request.read_text(), "EDGE-f9907afb-DEBUG\n")
        with self.assertRaises(PermissionError):
            self.panda.request("EDGE-f9907afb-DEBUG")  # 이미 요청했다

    def test_flash_needs_a_running_pandad_and_a_panda(self):
        self.image.write_bytes(panda_image())
        self.publish_control()
        with self.assertRaises(PermissionError):
            self.panda.request("EDGE-f9907afb-DEBUG")
        self.publish_pandad(stamp=boottime_ns() - 10_000_000_000)
        self.assertFalse(self.panda.status()["available"])
        self.publish_pandad(mode="none", version="")
        with self.assertRaises(PermissionError):
            self.panda.request("EDGE-f9907afb-DEBUG")
        self.publish_pandad(hw_type=7)
        with self.assertRaises(PermissionError):
            self.panda.request("EDGE-f9907afb-DEBUG")
        self.publish_pandad(mode="bootstub", version="", hw_type=0)  # 앱이 서지 않아도 다시 쓸 수 있다
        self.assertTrue(self.panda.request("EDGE-f9907afb-DEBUG")["request_pending"])

    def test_status_reports_flash_progress(self):
        self.publish_pandad(version="EDGE-f9907afb-DEBUG",
                            flash={"state": "failed", "step": "write", "percent": 40, "error": "app_timeout",
                                   "detail": "x", "version": "EDGE-f9907afb-DEBUG", "stamp_ns": 1})
        self.image.write_bytes(panda_image())
        status = self.panda.status()
        self.assertTrue(status["up_to_date"])
        self.assertEqual(status["flash"]["error"], "app_timeout")
        self.assertIn("bootstub", status["flash_error_text"])
        self.assertEqual(status["flash_step_text"], "쓰는 중")


# ------------------------------------------------------------ 런타임 상태
class ProcessStatusTest(unittest.TestCase):
    def test_layout_matches_manager_and_cpp(self):
        """manager.py가 쓰는 ManagerState(overlayd도 읽는다)와 웹 콘솔이 읽는 배치가 ipc_messages.h와 같아야 한다."""
        from scripts import manager

        def constant(name):
            return int(re.search(rf"constexpr \w+ {name} = (0x[0-9a-fA-F]+|\d+);", cpp()).group(1), 0)

        self.assertEqual(manager.IPC_MAGIC, constant("kIpcMagic"))
        self.assertEqual(manager.IPC_VERSION, constant("kIpcVersion"))
        self.assertEqual(manager.MAX_PROCESSES, constant("kMaxProcesses"))
        self.assertEqual(manager.HEADER_SIZE, cpp_size("IpcHeader"))
        self.assertEqual(manager.PROCESS.size, cpp_size("ProcessState"))
        self.assertEqual(manager.MANAGER_STATE_SIZE, cpp_size("ManagerState"))
        self.assertEqual((IPC_MAGIC, IPC_VERSION, IPC_HEADER.size),
                         (constant("kIpcMagic"), constant("kIpcVersion"), cpp_size("IpcHeader")))
        self.assertEqual((PROCESS_STATE.format, MAX_PROCESSES), (manager.PROCESS.format, manager.MAX_PROCESSES))
        self.assertLessEqual(MANAGER_STATE_HEAD.size + PROCESS_STATE.size * MAX_PROCESSES, manager.MANAGER_STATE_SIZE)
        self.assertIn(f'kManagerStateTopic[] = "{MANAGER_STATE_TOPIC}";', cpp())
        self.assertIn(f'LatestPublisher("{MANAGER_STATE_TOPIC}"', cpp(ROOT / "scripts" / "manager.py"))

    def test_status_lists_processes_and_goes_stale(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "edgepilot_manager_state"
            processes = ProcessStatus(str(path))
            self.assertEqual(processes.status(), {"available": False, "age_s": 0.0, "processes": []})

            def write(stamp):
                payload = bytearray(MANAGER_STATE_HEAD.size + PROCESS_STATE.size * MAX_PROCESSES)
                MANAGER_STATE_HEAD.pack_into(payload, 0, stamp, 2, 0)
                PROCESS_STATE.pack_into(payload, MANAGER_STATE_HEAD.size, b"controlsd", 1)
                PROCESS_STATE.pack_into(payload, MANAGER_STATE_HEAD.size + PROCESS_STATE.size, b"web_console", 0)
                publish(path, bytes(payload), stamp=stamp)

            write(boottime_ns())
            status = processes.status()
            self.assertTrue(status["available"])
            self.assertEqual(status["processes"], [{"name": "controlsd", "running": True},
                                                   {"name": "web_console", "running": False}])
            write(boottime_ns() - 10_000_000_000)
            self.assertFalse(processes.status()["available"])

    def test_manager_starts_the_package(self):
        from scripts import manager

        source = cpp(ROOT / "scripts" / "manager.py")
        self.assertIn('ProcSpec("web_console", [sys.executable, "-m", "web_console"]', source)
        self.assertIn('installed="web_console/__main__.py"', source)
        self.assertTrue((ROOT / "scripts" / "web_console" / "__main__.py").is_file())
        self.assertTrue(hasattr(manager, "ProcSpec"))


# ------------------------------------------------------------ BEV
class BevTest(unittest.TestCase):
    def test_layout_matches_cpp_offsets(self):
        """페이지가 위치로 읽는 필드는 ipc_messages.h가 offsetof로 고정한 그 위치여야 한다."""
        self.assertEqual(cpp_size("ModelState"), MODEL_STATE_SIZE)
        self.assertEqual(cpp_size("ControlState"), CONTROL_STATE_SIZE)
        self.assertEqual(cpp_offsets("MODEL_STATE"), MODEL_STATE_AT)
        control = cpp_offsets("CONTROL_STATE")
        for name, at in CONTROL_STATE_AT.items():
            self.assertEqual(control.get(name), at, name)
        flags = {name: int(bit) for name, bit in re.findall(r"constexpr uint32_t kHudFlag(\w+) = 1U << (\d+);", cpp())}
        for name, bit in HUD_FLAG_BITS.items():
            self.assertEqual(flags.get(name), bit, name)
        self.assertIn(f'kModelStateTopic[] = "{MODEL_STATE_TOPIC}";', cpp())
        self.assertIn(f'kControlStateTopic[] = "{CONTROL_STATE_TOPIC}";', cpp())
        layout = bev_layout()
        self.assertEqual((layout["model"]["size"], layout["control"]["size"]), (MODEL_STATE_SIZE, CONTROL_STATE_SIZE))

    def test_lead_constants_match_cpp(self):
        """페이지의 앞차 확률 문턱과 레이더-카메라 거리는 비전 크루즈·출발 알림·HUD와 같은 model_output.h 값이어야 한다."""
        source = cpp(ROOT / "src" / "common" / "model_output.h")
        text = (STATIC_DIR / "bev_data.js").read_text(encoding="utf-8")
        for cpp_name, page_name in (("kLeadProbabilityThreshold", "LEAD_PROBABILITY"),
                                    ("kRadarToCameraDistanceM", "RADAR_TO_CAMERA")):
            value = float(re.search(rf"constexpr float {cpp_name} = ([\d.]+)f;", source).group(1))
            page = float(re.search(rf"const {page_name} = ([\d.]+);", text).group(1))
            self.assertEqual(page, value, page_name)

    def test_page_reads_what_the_server_says(self):
        """bev_data.js NEEDS의 필드가 /api/bev 레이아웃에 다 있어야 한다(빠지면 페이지가 그리지 않는다). ControlState
        위치표는 Panda·보정 카드와 같이 쓰므로 BEV가 읽지 않는 필드도 있다."""
        text = (STATIC_DIR / "bev_data.js").read_text(encoding="utf-8")
        needs = re.search(r"const NEEDS = \{(.*?)\n\};", text, re.S).group(1)
        parts = {part: set(re.findall(r'"(\w+)"', keys)) for part, keys in re.findall(r"(\w+): \[(.*?)\]", needs, re.S)}
        layout = bev_layout()
        self.assertEqual(set(parts), set(layout))
        for part, fields in parts.items():
            self.assertLessEqual(fields, set(layout[part]), part)
        self.assertEqual(parts["model"], set(layout["model"]), "ModelState fields are the BEV's alone")

    def test_stream_sends_new_model_frames_and_idles_without_them(self):
        def fill(path, seq, value, size):
            publish(path, bytes([value]) * size, seq=seq)

        def split(frame):
            magic, model, control, now = BEV_FRAME.unpack_from(frame)
            self.assertEqual(magic, BEV_FRAME_MAGIC)
            self.assertEqual(len(frame), BEV_FRAME.size + model + control)
            body = frame[BEV_FRAME.size:]
            return body[:model], body[model:], now

        with tempfile.TemporaryDirectory() as directory:
            model, control = Path(directory) / "model_state", Path(directory) / "control_state"
            fill(model, 2, 1, MODEL_STATE_SIZE)
            fill(control, 2, 7, CONTROL_STATE_SIZE)

            async def take():
                frames = bev_frames(20, str(model), str(control), idle_s=0.2)
                first = await anext(frames)
                fill(model, 4, 2, MODEL_STATE_SIZE)
                second = await anext(frames)
                control.unlink()
                third = await anext(frames)  # 새 모델 프레임 없음: idle_s 뒤 머리만
                await frames.aclose()
                return first, second, third

            first, second, third = (split(frame) for frame in asyncio.run(take()))
        self.assertEqual((first[0], first[1]), (bytes([1]) * MODEL_STATE_SIZE, bytes([7]) * CONTROL_STATE_SIZE))
        self.assertEqual(second[0], bytes([2]) * MODEL_STATE_SIZE)
        self.assertEqual((third[0], third[1]), (b"", b""))
        self.assertLess(first[2], third[2])

    def test_stream_ends_when_the_server_stops(self):
        """uvicorn은 응답이 끝나기를 기다리므로, 서버가 내려가기 시작하면 스트림이 스스로 끝나야 한다."""
        stopping = [False]

        async def take():
            frames = bev_frames(20, "/nonexistent/model", "/nonexistent/control", idle_s=0.05,
                                stopping=lambda: stopping[0])
            first = await anext(frames)
            stopping[0] = True
            with self.assertRaises(StopAsyncIteration):
                await asyncio.wait_for(anext(frames), 1.0)
            return first

        self.assertEqual(BEV_FRAME.unpack_from(asyncio.run(take()))[1:3], (0, 0))


# ------------------------------------------------------------ 페이지 파일
class StaticPageTest(unittest.TestCase):
    def test_assets_are_gzipped_once_and_stay_in_static(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "static"
            (root / "three").mkdir(parents=True)
            (root / "three" / "a.js").write_text("export const a = 1;\n", encoding="utf-8")
            (root / "notes.txt").write_text("x", encoding="utf-8")
            (root / ".hidden.js").write_text("x", encoding="utf-8")
            (Path(directory) / "outside.js").write_text("x", encoding="utf-8")
            assets = StaticAssets(root)
            first = assets.get("three/a.js")
            self.assertEqual(gzip.decompress(first.gzipped), b"export const a = 1;\n")
            self.assertEqual(first.media_type, "text/javascript; charset=utf-8")
            self.assertIs(assets.get("three/a.js").gzipped, first.gzipped)
            for name in ("../outside.js", "notes.txt", ".hidden.js", "missing.js", "/etc/hosts", "three", ""):
                self.assertIsNone(assets.get(name), name)
            (root / "three" / "a.js").write_text("export const a = 22;\n", encoding="utf-8")
            self.assertNotEqual(assets.get("three/a.js").etag, first.etag)

    def test_page_modules_and_three_resolve(self):
        page = (STATIC_DIR / "index.html").read_text(encoding="utf-8")
        imports = json.loads(re.search(r'<script type="importmap">(.*?)</script>', page, re.S).group(1))["imports"]
        for url in [*imports.values(), *re.findall(r'(?:src|href)="(/static/[^"]+)"', page)]:
            self.assertTrue((STATIC_DIR / Path(url).relative_to("/static")).is_file(), url)
        self.assertIn('src="/static/console.js"', page)
        for module in [*STATIC_DIR.glob("*.js"), *STATIC_DIR.glob("three/*.js")]:
            text = module.read_text(encoding="utf-8")
            for target in re.findall(r"""(?:from|import)\s*\(?\s*["'](\./[^"']+)["']""", text):
                self.assertTrue((module.parent / target).is_file(), f"{module.name} imports {target}")
            for bare in re.findall(r"""^import[^;]*?from\s*["']([^"'./][^"']*)["']""", text, re.M):
                self.assertIn(bare, imports, f"{module.name} imports {bare}")
        for element_id in ("main", "tabs", "runtime", "runtime-panel", "reload", "toasts"):
            self.assertIn(f'id="{element_id}"', page)

    def test_page_calls_only_routes_the_server_has(self):
        server = (ROOT / "scripts" / "web_console" / "console_server.py").read_text(encoding="utf-8")
        routes = []
        for method, path in re.findall(r'@application\.(get|post|patch)\("([^"]+)"\)', server):
            pattern = re.sub(r"\\\{\w+:path\\\}", ".+", re.sub(r"\\\{\w+\\\}", "[^/]+", re.escape(path)))
            routes.append((method.upper(), re.compile(pattern + r"\Z")))
        calls = set()
        for module in STATIC_DIR.glob("*.js"):
            text = module.read_text(encoding="utf-8")
            for method, url in re.findall(r"api\.(get|post|patch)\(\s*[`\"]([^`\"]+)[`\"]", text):
                calls.add((method.upper(), url))
            for url in re.findall(r"fetch\(\s*[`\"]([^`\"]+)[`\"]", text):
                calls.add(("GET", url))
        self.assertGreaterEqual(len(calls), 9)
        for method, url in sorted(calls):
            path = re.sub(r"\$\{[^}]+\}", "x", url).split("?", 1)[0]
            self.assertTrue(any(method == route_method and pattern.match(path) for route_method, pattern in routes),
                            f"{method} {url}")


if __name__ == "__main__":
    unittest.main()
