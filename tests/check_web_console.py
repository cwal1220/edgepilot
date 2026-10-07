#!/usr/bin/env python3
import asyncio
import gzip
import json
import re
import struct
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scripts.web_console.backlight import duty_cycle_ns
from scripts.web_console.console_server import (
    BEV_CONTROL_FIELDS,
    BEV_FRAME,
    BEV_FRAME_MAGIC,
    BEV_HUD_FLAGS,
    BEV_MODEL_FIELDS,
    CALIBRATION_STATE,
    CONTROL_STATE_HEAD,
    CONTROL_STATE_SIZE,
    HTML,
    MODEL_CALIBRATION_OFFSET,
    MODEL_STATE_SIZE,
    CalibrationControl,
    IPC_HEADER,
    IPC_MAGIC,
    LEARNER_FIELDS,
    LEARNER_STATE,
    LearnerMonitor,
    LearnerStateReader,
    LOCALIZATION_FIELDS,
    LOCALIZATION_STATE,
    LocalizationReader,
    LOCALIZATION_FLAGS,
    LOCALIZATION_INPUT_FLAGS,
    PANDA_APP_MAX_BYTES,
    PANDA_CONTROL_FIELDS,
    PANDA_FLASH_REQUEST_PATH,
    PANDA_SIGNATURE_BYTES,
    PANDA_STATUS_PATH,
    GEAR_PARK,
    PARAM_METADATA,
    PandaFirmware,
    ParamStore,
    WEB_DIR,
    WebAssets,
    bev_frames,
    bev_layout,
    boottime_ns,
    fixed_lateral_values,
    panda_image_info,
)


class FakeDisplayController:
    def __init__(self):
        self.applied = []

    def apply(self, document):
        self.applied.append(dict(document))

    def status(self):
        latest = self.applied[-1] if self.applied else {}
        return {
            "available": True,
            "enabled": latest.get("enabled", True),
            "brightness_percent": latest.get("brightness_percent", 100),
            "mode": "test",
            "error": "",
        }


class ParamStoreTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        root = Path(self.temporary.name)
        self.paths = {
            "steering": root / "steering.json",
            "driving": root / "driving.json",
            "adaptive_cruise": root / "adaptive_cruise.json",
            "recording": root / "recording.json",
            "display": root / "display.json",
        }
        self.paths["steering"].write_text(
            json.dumps({"gain": 10, "enabled": True}), encoding="utf-8"
        )
        self.paths["driving"].write_text(
            json.dumps({"delay": 0.4}), encoding="utf-8"
        )
        self.paths["adaptive_cruise"].write_text(
            json.dumps({"following_time_s": 1.8}), encoding="utf-8"
        )
        self.paths["recording"].write_text(
            json.dumps({"enabled": False}), encoding="utf-8"
        )
        self.paths["display"].write_text(
            json.dumps({"enabled": True, "brightness_percent": 100}), encoding="utf-8"
        )
        self.notifications = 0

        def notify():
            self.notifications += 1
            return [123]

        self.store = ParamStore(self.paths, notify)

    def tearDown(self):
        self.temporary.cleanup()

    def test_atomic_single_value_update(self):
        result = self.store.update("steering", {"gain": 12})
        self.assertEqual(result["notified_pids"], [123])
        self.assertEqual(self.notifications, 1)
        self.assertEqual(self.store.read_group("steering"), {"gain": 12, "enabled": True})
        self.assertEqual(list(self.paths["steering"].parent.glob("*.tmp")), [])

    def test_unknown_parameter_is_rejected(self):
        with self.assertRaises(KeyError):
            self.store.update("driving", {"unknown": 1})
        self.assertEqual(self.notifications, 0)
        self.assertEqual(self.store.read_group("driving"), {"delay": 0.4})

    def test_recording_toggle_does_not_signal_controlsd(self):
        result = self.store.update("recording", {"enabled": True})
        self.assertEqual(result["notified_pids"], [])
        self.assertEqual(self.notifications, 0)

    def test_display_update_applies_hardware_without_signaling_controlsd(self):
        controller = FakeDisplayController()
        store = ParamStore(
            self.paths,
            self.store.notifier,
            display_controller=controller,
        )
        controller.applied.clear()
        result = store.update("display", {"brightness_percent": 35})
        self.assertEqual(result["notified_pids"], [])
        self.assertEqual(controller.applied, [{"enabled": True, "brightness_percent": 35}])
        self.assertEqual(store.read_group("display")["brightness_percent"], 35)

    def test_alert_volume_update_leaves_backlight_alone(self):
        self.paths["display"].write_text(
            json.dumps({"enabled": True, "brightness_percent": 80, "alert_volume_percent": 70}),
            encoding="utf-8")
        controller = FakeDisplayController()
        store = ParamStore(self.paths, self.store.notifier, display_controller=controller)
        controller.applied.clear()
        result = store.update("display", {"alert_volume_percent": 30})
        self.assertEqual(result["params"]["alert_volume_percent"], 30)
        self.assertEqual(result["notified_pids"], [])
        self.assertEqual(controller.applied, [], "volume is read by overlayd, not the backlight")
        store.update("display", {"brightness_percent": 60})
        self.assertEqual(controller.applied[-1]["brightness_percent"], 60)
        self.assertEqual(json.loads(self.paths["display"].read_text())["alert_volume_percent"], 30)

    def test_display_pwm_duty_scales_by_board_maximum(self):
        self.assertEqual(duty_cycle_ns(100), 95_000)
        self.assertEqual(duty_cycle_ns(51), 48_450)
        self.assertEqual(duty_cycle_ns(1), 950)
        self.assertEqual(duty_cycle_ns(0), 0)
        self.assertEqual(duty_cycle_ns(100, max_percent=100), 100_000)

    def test_missing_defaults_are_added_without_overwriting_tuning(self):
        defaults = Path(self.temporary.name) / "adaptive.defaults.json"
        defaults.write_text(
            json.dumps({"following_time_s": 1.8, "deceleration_rate_kph_per_s": 1.5}),
            encoding="utf-8",
        )
        self.paths["adaptive_cruise"].write_text(
            json.dumps({"following_time_s": 2.2}), encoding="utf-8"
        )
        store = ParamStore(
            self.paths,
            lambda: [],
            {"adaptive_cruise": defaults},
        )
        self.assertEqual(
            store.read_group("adaptive_cruise"),
            {"following_time_s": 2.2, "deceleration_rate_kph_per_s": 1.5},
        )

    def test_repository_params_have_complete_ui_metadata(self):
        root = Path(__file__).resolve().parents[1]
        for group, filename in (
            ("steering", "steering.json"),
            ("driving", "driving.json"),
            ("adaptive_cruise", "adaptive_cruise.json"),
            ("recording", "recording.json"),
            ("display", "display.json"),
        ):
            params = json.loads((root / "params" / filename).read_text(encoding="utf-8"))
            self.assertEqual(set(params), set(PARAM_METADATA[group]))
            for key, value in params.items():
                metadata = PARAM_METADATA[group][key]
                for field in ("label", "section", "description", "increase", "decrease"):
                    self.assertTrue(metadata.get(field), f"{group}.{key}.{field}")
                if isinstance(value, (int, float)) and not isinstance(value, bool):
                    for field in ("step", "min", "max"):
                        self.assertIsInstance(
                            metadata.get(field), (int, float), f"{group}.{key}.{field}"
                        )

    def test_ui_ranges_match_runtime_clamps(self):
        """The loaders clamp to their Json*Field tables; the editor must show
        the same min/max or it accepts values the runtime silently changes."""
        root = Path(__file__).resolve().parents[1]
        row = re.compile(r'\{"(\w+)",\s*(-?[\d.]+)f?,\s*(-?[\d.]+)f?,\s*&\w+::\w+\}')
        tables = {
            "steering": ("src/controls/control_params.cc", ("kSteeringInts", "kSteeringFloats")),
            "driving": ("src/controls/control_params.cc", ("kDrivingInts", "kDrivingFloats")),
            "adaptive_cruise": ("src/controls/adaptive_cruise.cc", ("kAdaptiveInts", "kAdaptiveFloats")),
            # display는 백라이트(backlight.py) 항목도 있어 런타임이 읽는 키만 대조한다
            "display": ("src/common/device_settings.h", ("kDeviceSettingsFloats",)),
        }
        for group, (source, names) in tables.items():
            text = (root / source).read_text(encoding="utf-8")
            runtime = {}
            for name in names:
                body = re.search(name + r"\[\] = \{(.*?)\n\};", text, re.S)
                self.assertIsNotNone(body, f"{source}: {name}")
                runtime.update({key: (float(low), float(high))
                                for key, low, high in row.findall(body.group(1))})
            self.assertTrue(runtime, f"{source}: no rows parsed")
            ui = {key: meta for key, meta in PARAM_METADATA[group].items() if "min" in meta}
            if group == "display":
                ui = {key: meta for key, meta in ui.items() if key in runtime}
            self.assertEqual(set(runtime), set(ui), group)
            for key, (low, high) in runtime.items():
                self.assertEqual((float(ui[key]["min"]), float(ui[key]["max"])), (low, high),
                                 f"{group}.{key}")


def learner_payload(**values):
    fields = []
    for name, fmt in LEARNER_FIELDS:
        default = [0] * int(fmt[:-1]) if len(fmt) > 1 else 0
        value = values.get(name, default)
        fields.extend(value if isinstance(value, list) else [value])
    return LEARNER_STATE.pack(*fields)


class LearnerStateTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.path = Path(self.temporary.name) / "edgepilot_learner_state"

    def tearDown(self):
        self.temporary.cleanup()

    def publish(self, seq, **values):
        payload = learner_payload(**values)
        header = IPC_HEADER.pack(IPC_MAGIC, 1, LEARNER_STATE.size, 0, seq, 123, len(payload), 0)
        self.path.write_bytes(header + payload)

    def test_layout_matches_cpp_offsets(self):
        """ipc_messages.h의 offsetof 고정값과 Python 필드 배치가 같아야 한다."""
        source = (Path(__file__).resolve().parents[1] / "src" / "common" / "ipc_messages.h").read_text(encoding="utf-8")
        size = int(re.search(r"sizeof\(LearnerState\) == (\d+)", source).group(1))
        self.assertEqual(LEARNER_STATE.size, size)
        offsets, offset = {}, 0
        for name, fmt in LEARNER_FIELDS:
            offsets[name] = offset
            offset += struct.calcsize("<" + fmt)
        asserted = re.findall(r"EDGEPILOT_LEARNER_STATE_AT\((\w+), (\d+)\);", source)
        self.assertGreaterEqual(len(asserted), 8)
        for name, expected in asserted:
            self.assertEqual(offsets[name], int(expected), name)

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
        self.assertFalse(monitor.snapshot({})["available"])
        self.publish(2, timestamp_ns=1_000_000_000, lat_accel_factor=4.44)
        monitor.sample()
        monitor.sample()
        self.publish(4, timestamp_ns=2_000_000_000, lat_accel_factor=4.40)
        snapshot = monitor.snapshot(json.loads(
            (Path(__file__).resolve().parents[1] / "params" / "steering.json").read_text(encoding="utf-8")))
        self.assertTrue(snapshot["available"])
        self.assertEqual([row[0] for row in monitor.trend()], [1.0, 2.0])
        self.assertAlmostEqual(snapshot["trend_row"][7], 4.40, places=5)
        self.publish(6, timestamp_ns=500_000_000)  # 시각이 거꾸로: 다른 부팅
        monitor.sample()
        self.assertEqual([row[0] for row in monitor.trend()], [0.5])

    def test_fixed_values_are_the_manual_values(self):
        fixed = fixed_lateral_values({"torque_lat_accel_factor": 2.8, "torque_friction": 0.1,
                                      "steer_ratio": 14.9})
        self.assertEqual((fixed["lat_accel_factor"], fixed["friction"], fixed["steer_ratio"]),
                         (2.8, 0.1, 14.9))

    def test_page_has_learner_tab(self):
        self.assertIn('data-group="learners"', HTML)
        self.assertIn("/api/learners/trend", HTML)
        # HTML은 일반 문자열이라 JS의 \n을 두 번 이스케이프해야 한다(아니면 스크립트 전체가 죽는다)
        self.assertIn('lines.join("\\n")', HTML)
        # 학습 스위치는 실시간 학습 탭에만 있다(조향 탭에서 숨김)
        hidden = re.search(r"hiddenKeys = \{steering: \[(.*?)\]\}", HTML, re.S)
        self.assertIsNotNone(hidden)
        self.assertEqual(set(re.findall(r'"(\w+)"', hidden.group(1))),
                         {"use_live_vehicle_params", "use_live_torque_params", "use_live_delay",
                          "use_locationd_learner_inputs"})
        self.assertIn('inputShell()', HTML)



class CalibrationControlTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        root = Path(self.temporary.name)
        self.model = root / "edgepilot_model_state"
        self.control = root / "edgepilot_control_state"
        self.request = root / "edgepilot_calibration_reset"
        self.calibration = CalibrationControl(str(self.model), str(self.control), str(self.request))

    def tearDown(self):
        self.temporary.cleanup()

    @staticmethod
    def write(path, payload, stamp=None):
        stamp = boottime_ns() if stamp is None else stamp
        path.write_bytes(IPC_HEADER.pack(IPC_MAGIC, 1, len(payload), 0, 2, stamp, len(payload), 0) + payload)

    def publish_model(self, status, blocks, rpy_deg):
        payload = bytearray(3256)
        rad = [x * 3.141592653589793 / 180 for x in rpy_deg]
        CALIBRATION_STATE.pack_into(payload, MODEL_CALIBRATION_OFFSET, status, blocks, *rad, 0.1, 0.2, 0.3)
        self.write(self.model, bytes(payload))

    def publish_control(self, engaged, stamp=None):
        self.write(self.control, CONTROL_STATE_HEAD.pack(1, 1, engaged) + bytes(228), stamp)

    def test_offset_matches_cpp(self):
        source = (Path(__file__).resolve().parents[1] / "src" / "common" / "ipc_messages.h").read_text(encoding="utf-8")
        offset = re.search(r"offsetof\(ModelState, calibration\) == (\d+)", source)
        self.assertEqual(int(offset.group(1)), MODEL_CALIBRATION_OFFSET)
        self.assertEqual(CALIBRATION_STATE.size, 32)
        self.assertRegex(source, r"EDGEPILOT_CONTROL_STATE_AT\(engaged, 12\);")

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
            self.calibration.request_reset()
        self.assertFalse(self.request.exists())
        self.publish_control(engaged=0)
        self.assertTrue(self.calibration.request_reset()["reset_pending"])
        self.assertTrue(self.request.exists())

    def test_stale_or_missing_controlsd_does_not_block(self):
        self.assertFalse(self.calibration.engaged())
        self.publish_control(engaged=1, stamp=boottime_ns() - 5_000_000_000)
        self.assertFalse(self.calibration.engaged())

    def test_page_has_reset_button(self):
        self.assertIn("/api/calibration/reset", HTML)


def panda_image(version="EDGE-f9907afb-DEBUG", body=1024):
    """sign.py의 꼴: [본문 길이][... 버전 ...][VERS][2] 뒤에 서명 128바이트."""
    data = bytearray(b"\x11" * body)
    struct.pack_into("<I", data, 0, body)
    data[63:64 + len(version) + 1] = b"\0" + version.encode() + b"\0"
    data[body - 8:body] = b"VERS" + struct.pack("<I", 2)
    return bytes(data) + b"\xab" * PANDA_SIGNATURE_BYTES


class PandaFirmwareTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        root = Path(self.temporary.name)
        self.status = root / "edgepilot_panda_status.json"
        self.request = root / "edgepilot_panda_flash"
        self.image = root / "panda.bin.signed"
        self.control = root / "edgepilot_control_state"
        self.panda = PandaFirmware(str(self.status), str(self.request), str(self.image), str(self.control))

    def tearDown(self):
        self.temporary.cleanup()

    def publish_pandad(self, mode="app", version="DEV-23456789-DEBUG", hw_type=3, flash=None, stamp=None):
        self.status.write_text(json.dumps({
            "stamp_ns": boottime_ns() if stamp is None else stamp, "mode": mode, "serial": "abc",
            "hw_type": hw_type, "hw_name": "black panda", "firmware_version": version,
            "flash": flash or {"state": "idle", "step": "", "percent": 0, "error": "", "detail": "",
                               "version": "", "stamp_ns": 0}}))

    def publish_control(self, gear=GEAR_PARK, speed=0.0, vehicle_fresh=1, engaged=0, stamp=None):
        payload = bytearray(CONTROL_STATE_SIZE)
        for name, value in (("engaged", engaged), ("vehicle_fresh", vehicle_fresh)):
            struct.pack_into("<I", payload, PANDA_CONTROL_FIELDS[name], value)
        struct.pack_into("<i", payload, PANDA_CONTROL_FIELDS["gear"], gear)
        struct.pack_into("<f", payload, PANDA_CONTROL_FIELDS["ego_speed_kph"], speed)
        stamp = boottime_ns() if stamp is None else stamp
        self.control.write_bytes(IPC_HEADER.pack(IPC_MAGIC, 1, len(payload), 0, 2, stamp, len(payload), 0)
                                 + bytes(payload))

    def test_constants_match_cpp(self):
        root = Path(__file__).resolve().parents[1]
        messages = (root / "src" / "common" / "ipc_messages.h").read_text(encoding="utf-8")
        control = {name: int(at) for name, at in re.findall(r"EDGEPILOT_CONTROL_STATE_AT\((\w+), (\d+)\);", messages)}
        for name, at in PANDA_CONTROL_FIELDS.items():
            self.assertEqual(control.get(name), at, name)
        can_frame = (root / "src" / "car" / "can_frame.h").read_text(encoding="utf-8")
        self.assertEqual(int(re.search(r"kGearPark = (\d+);", can_frame).group(1)), GEAR_PARK)
        protocol = (root / "src" / "panda" / "panda_protocol.h").read_text(encoding="utf-8")
        self.assertIn(f"kPandaSignatureBytes = {PANDA_SIGNATURE_BYTES};", protocol)
        self.assertIn("kPandaAppMaxBytes = 3 * 16 * 1024;", protocol)
        self.assertEqual(PANDA_APP_MAX_BYTES, 3 * 16 * 1024)
        pandad = (root / "src" / "panda" / "pandad.cc").read_text(encoding="utf-8")
        self.assertIn(f'kFirmwareStatusPath[] = "{PANDA_STATUS_PATH}";', pandad)
        self.assertIn(f'kFlashRequestPath[] = "{PANDA_FLASH_REQUEST_PATH}";', pandad)

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
            self.panda.request_flash("EDGE-f9907afb-DEBUG")
        self.assertFalse(self.request.exists())
        for blocker, kwargs in (("moving", {"speed": 3.0}), ("engaged", {"engaged": 1}),
                                ("vehicle_stale", {"vehicle_fresh": 0}),
                                ("control_stale", {"stamp": boottime_ns() - 5_000_000_000})):
            self.publish_control(**kwargs)
            self.assertEqual(self.panda.status()["blocker"], blocker)

        self.publish_control()
        with self.assertRaises(PermissionError):
            self.panda.request_flash("EDGE-00000000-DEBUG")  # 페이지가 본 이미지가 아니다
        status = self.panda.request_flash("EDGE-f9907afb-DEBUG")
        self.assertTrue(status["request_pending"])
        self.assertEqual(self.request.read_text(), "EDGE-f9907afb-DEBUG\n")
        with self.assertRaises(PermissionError):
            self.panda.request_flash("EDGE-f9907afb-DEBUG")  # 이미 요청했다

    def test_flash_needs_a_running_pandad_and_a_panda(self):
        self.image.write_bytes(panda_image())
        self.publish_control()
        with self.assertRaises(PermissionError):
            self.panda.request_flash("EDGE-f9907afb-DEBUG")
        self.publish_pandad(stamp=boottime_ns() - 10_000_000_000)
        self.assertFalse(self.panda.status()["available"])
        self.publish_pandad(mode="none", version="")
        with self.assertRaises(PermissionError):
            self.panda.request_flash("EDGE-f9907afb-DEBUG")
        self.publish_pandad(hw_type=7)
        with self.assertRaises(PermissionError):
            self.panda.request_flash("EDGE-f9907afb-DEBUG")
        self.publish_pandad(mode="bootstub", version="", hw_type=0)  # 앱이 서지 않아도 다시 쓸 수 있다
        self.assertTrue(self.panda.request_flash("EDGE-f9907afb-DEBUG")["request_pending"])

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

    def test_page_has_panda_card(self):
        self.assertIn("/api/panda/flash", HTML)
        self.assertIn("pandaSection()", HTML)


class LocalizationStateTest(unittest.TestCase):
    def test_layout_matches_cpp_size(self):
        source = (Path(__file__).resolve().parents[1] / "src" / "common" / "ipc_messages.h").read_text(encoding="utf-8")
        size = int(re.search(r"sizeof\(LocalizationState\) == (\d+)", source).group(1))
        self.assertEqual(LOCALIZATION_STATE.size, size)
        flags = re.findall(r"constexpr uint32_t kLocalization(\w+) = 1U << (\d+);", source)
        state_flags = [int(bit) for name, bit in flags if not name.startswith(("Invalid", "CameraGuarded"))]
        input_flags = [int(bit) for name, bit in flags if name.startswith(("Invalid", "CameraGuarded"))]
        self.assertEqual(state_flags, list(range(len(LOCALIZATION_FLAGS))))
        self.assertEqual(input_flags, list(range(len(LOCALIZATION_INPUT_FLAGS))))

    def test_reader_decodes(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "edgepilot_localization"
            reader = LocalizationReader(str(path))
            self.assertFalse(reader.snapshot({})["available"])
            fields = []
            values = {"flags": 0b100001, "lag_status": 1, "input_flags": 0b1010, "lateral_delay_s": 0.42, "lag_valid_blocks": 6,
                      "angular_velocity_calib": [0.0, 0.0, 0.1]}
            for name, fmt in LOCALIZATION_FIELDS:
                default = [0.0] * int(fmt[:-1]) if len(fmt) > 1 else 0
                value = values.get(name, default)
                fields.extend(value if isinstance(value, list) else [value])
            payload = LOCALIZATION_STATE.pack(*fields)
            header = IPC_HEADER.pack(IPC_MAGIC, 1, LOCALIZATION_STATE.size, 0, 2, boottime_ns(), len(payload), 0)
            path.write_bytes(header + payload)
            result = reader.snapshot({"steer_actuator_delay": 0.34})
            self.assertTrue(result["available"])
            self.assertEqual(result["initial_lag"], 0.34)
            state = result["state"]
            self.assertAlmostEqual(state["lateral_delay_s"], 0.42, places=5)
            self.assertEqual(state["lag_valid_blocks"], 6)
            self.assertAlmostEqual(state["angular_velocity_calib"][2], 0.1, places=5)
            self.assertTrue(state["flags"]["filter_valid"] and state["flags"]["lag_restored"])
            self.assertFalse(state["flags"]["inputs_ok"])
            self.assertTrue(state["input_flags"]["gyro_invalid"] and state["input_flags"]["camera_guarded"])
            self.assertFalse(state["input_flags"]["camera_invalid"])

    def test_page_has_lag_card(self):
        self.assertIn("lagd · 조향 지연", HTML)


class BevTest(unittest.TestCase):
    def test_layout_matches_cpp_offsets(self):
        """페이지가 위치로 읽는 필드는 ipc_messages.h가 offsetof로 고정한 그 위치여야 한다."""
        source = (Path(__file__).resolve().parents[1] / "src" / "common" / "ipc_messages.h").read_text(encoding="utf-8")
        self.assertEqual(int(re.search(r"sizeof\(ModelState\) == (\d+)", source).group(1)), MODEL_STATE_SIZE)
        self.assertEqual(int(re.search(r"sizeof\(ControlState\) == (\d+)", source).group(1)), CONTROL_STATE_SIZE)
        model = {name: int(at) for name, at in re.findall(r"EDGEPILOT_MODEL_STATE_AT\((\w+), (\d+)\);", source)}
        self.assertEqual(model, BEV_MODEL_FIELDS)
        control = {name: int(at) for name, at in re.findall(r"EDGEPILOT_CONTROL_STATE_AT\((\w+), (\d+)\);", source)}
        for name, at in BEV_CONTROL_FIELDS.items():
            self.assertEqual(control.get(name), at, name)
        flags = {name: int(bit) for name, bit in re.findall(r"constexpr uint32_t kHudFlag(\w+) = 1U << (\d+);", source)}
        for name, bit in BEV_HUD_FLAGS.items():
            self.assertEqual(flags.get(name), bit, name)
        layout = bev_layout()
        self.assertEqual((layout["model"]["size"], layout["control"]["size"]), (MODEL_STATE_SIZE, CONTROL_STATE_SIZE))

    def test_lead_constants_match_cpp(self):
        """페이지의 앞차 확률 문턱과 레이더-카메라 거리는 비전 크루즈·출발 알림·HUD와 같은 model_output.h 값이어야 한다."""
        source = (Path(__file__).resolve().parents[1] / "src" / "common" / "model_output.h").read_text(encoding="utf-8")
        text = (WEB_DIR / "bev_data.js").read_text(encoding="utf-8")
        for cpp_name, page_name in (("kLeadProbabilityThreshold", "LEAD_PROBABILITY"),
                                    ("kRadarToCameraDistanceM", "RADAR_TO_CAMERA")):
            cpp = float(re.search(rf"constexpr float {cpp_name} = ([\d.]+)f;", source).group(1))
            page = float(re.search(rf"const {page_name} = ([\d.]+);", text).group(1))
            self.assertEqual(page, cpp, page_name)

    def test_page_reads_what_the_server_says(self):
        """bev_data.js NEEDS가 /api/bev 레이아웃과 같은 필드를 가져야 한다(어긋나면 페이지가 그리지 않는다)."""
        text = (WEB_DIR / "bev_data.js").read_text(encoding="utf-8")
        needs = re.search(r"const NEEDS = \{(.*?)\n\};", text, re.S).group(1)
        parts = {part: set(re.findall(r'"(\w+)"', keys)) for part, keys in re.findall(r"(\w+): \[(.*?)\]", needs, re.S)}
        self.assertEqual({part: set(fields) for part, fields in bev_layout().items()}, parts)

    def test_stream_sends_new_model_frames_and_idles_without_them(self):
        def publish(path, seq, fill, size):
            payload = bytes([fill]) * size
            path.write_bytes(IPC_HEADER.pack(IPC_MAGIC, 1, size, 0, seq, boottime_ns(), size, 0) + payload)

        def split(frame):
            magic, model, control, now = BEV_FRAME.unpack_from(frame)
            self.assertEqual(magic, BEV_FRAME_MAGIC)
            self.assertEqual(len(frame), BEV_FRAME.size + model + control)
            body = frame[BEV_FRAME.size:]
            return body[:model], body[model:], now

        with tempfile.TemporaryDirectory() as directory:
            model, control = Path(directory) / "model_state", Path(directory) / "control_state"
            publish(model, 2, 1, MODEL_STATE_SIZE)
            publish(control, 2, 7, CONTROL_STATE_SIZE)

            async def take():
                frames = bev_frames(20, str(model), str(control), idle_s=0.2)
                first = await anext(frames)
                publish(model, 4, 2, MODEL_STATE_SIZE)
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

    def test_web_assets_are_gzipped_once_and_stay_in_web(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "web"
            (root / "three").mkdir(parents=True)
            (root / "three" / "a.js").write_text("export const a = 1;\n", encoding="utf-8")
            (root / "notes.txt").write_text("x", encoding="utf-8")
            (Path(directory) / "outside.js").write_text("x", encoding="utf-8")
            assets = WebAssets(root)
            etag, body = assets.get("three/a.js")
            self.assertEqual(gzip.decompress(body), b"export const a = 1;\n")
            self.assertIs(assets.get("three/a.js")[1], body)
            for name in ("../outside.js", "notes.txt", "missing.js", "/etc/hosts", "three"):
                self.assertIsNone(assets.get(name), name)
            (root / "three" / "a.js").write_text("export const a = 22;\n", encoding="utf-8")
            self.assertNotEqual(assets.get("three/a.js")[0], etag)

    def test_page_has_bev_tab_and_its_modules_are_there(self):
        self.assertIn('data-group="bev"', HTML)
        self.assertIn('import("/web/bev.js")', HTML)
        imports = json.loads(re.search(r'<script type="importmap">(.*?)</script>', HTML).group(1))["imports"]
        self.assertTrue((WEB_DIR / Path(imports["three"]).relative_to("/web")).is_file())
        self.assertTrue((WEB_DIR / "bev.js").is_file())
        for module in [*WEB_DIR.glob("*.js"), *WEB_DIR.glob("three/*.js")]:
            text = module.read_text(encoding="utf-8")
            for target in re.findall(r"""(?:from|import)\s*["'](\./[^"']+)["']""", text):
                self.assertTrue((module.parent / target).is_file(), f"{module.name} imports {target}")
            for bare in re.findall(r"""^import[^;]*?from\s*["']([^"'./][^"']*)["']""", text, re.M):
                self.assertIn(bare, imports, f"{module.name} imports {bare}")


class ManagerIpcTest(unittest.TestCase):
    def test_manager_state_matches_cpp(self):
        """manager.py가 쓰는 managerState 배치(overlayd가 읽는다)와 웹 콘솔이 읽는 채널 머리가
        ipc_messages.h와 같아야 한다."""
        from scripts import manager
        from scripts.web_console import console_server

        source = (Path(__file__).resolve().parents[1] / "src" / "common" / "ipc_messages.h").read_text(encoding="utf-8")

        def constant(name):
            return int(re.search(rf"constexpr \w+ {name} = (0x[0-9a-fA-F]+|\d+);", source).group(1), 0)

        def size(struct_name):
            return int(re.search(rf"sizeof\({struct_name}\) == (\d+)", source).group(1))

        self.assertEqual(manager.IPC_MAGIC, constant("kIpcMagic"))
        self.assertEqual(manager.IPC_VERSION, constant("kIpcVersion"))
        self.assertEqual(manager.MAX_PROCESSES, constant("kMaxProcesses"))
        self.assertEqual(manager.HEADER_SIZE, size("IpcHeader"))
        self.assertEqual(manager.PROCESS.size, size("ProcessState"))
        self.assertEqual(manager.MANAGER_STATE_SIZE, size("ManagerState"))
        self.assertEqual(console_server.IPC_MAGIC, constant("kIpcMagic"))
        self.assertEqual(console_server.IPC_VERSION, constant("kIpcVersion"))
        self.assertEqual(console_server.IPC_HEADER.size, size("IpcHeader"))


if __name__ == "__main__":
    unittest.main()
