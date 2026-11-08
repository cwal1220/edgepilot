#!/usr/bin/env python3
"""웹 콘솔(scripts/web_console) 검사. stdlib만 쓴다(fastapi 없이): 파라미터 저장, 공유 메모리 상태 읽기와 그
배치가 src/common/ipc_messages.h와 같은지, Panda 플래싱 조건, BEV 스트림, 페이지 파일과 서버 API가 맞는지."""
import ast
import asyncio
import gzip
import hashlib
import json
import math
import os
import re
import shutil
import struct
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from cpp_headers import cpp_constant, cpp_flags, cpp_offsets, cpp_size, cpp_string  # noqa: E402
from scripts.web_console.bev_stream import BEV_FRAME, BEV_FRAME_MAGIC, BEV_IDLE_S, bev_frames, bev_layout
from scripts.web_console.calibration_control import (
    CALIBRATION_RESET_PATH,
    CALIBRATION_SOURCE_STATE,
    CALIBRATION_STATE,
    FLOW_CALIBRATION_STATE,
    CalibrationControl,
)
from scripts.web_console.learner_monitor import (
    GRAVITY,
    LEARNER_FIELDS,
    LEARNER_FLAGS,
    LEARNER_STATE,
    LEARNER_TREND_COLUMNS,
    LOCALIZATION_FIELDS,
    LOCALIZATION_FLAGS,
    LOCALIZATION_INPUT_FLAGS,
    LOCALIZATION_STATE,
    LearnerMonitor,
    LocalizationReader,
)
from scripts.web_console.driving_models import DEFAULT_MODEL as DEFAULT_DRIVING_MODEL
from scripts.web_console.driving_models import MODEL_DIR as DRIVING_MODEL_DIR
from scripts.web_console.driving_models import NAME_PATTERN as DRIVING_NAME_PATTERN
from scripts.web_console.driving_models import SELECTION_FILE as DRIVING_SELECTION_FILE
from scripts.web_console.driving_models import STATUS_PATH as DRIVING_STATUS_PATH
from scripts.web_console.driving_models import DrivingModels
from scripts.web_console.driving_models import read_choice as read_driving_choice
from scripts.web_console.lateral_nn_models import (
    MODEL_DIR,
    SELECTION_FILE,
    STATUS_PATH,
    LateralNnModels,
    read_choice,
)
from scripts.web_console.panda_update import (
    IMAGE_ERRORS,
    PANDA_APP_MAX_BYTES,
    PANDA_FLASHABLE_HW,
    PANDA_FLASH_REQUEST_PATH,
    PANDA_SIGNATURE_BYTES,
    PANDA_STATUS_PATH,
    PARKED_MAX_SPEED_KPH,
    REASONS,
    STEPS,
    PandaUpdate,
    panda_image_info,
)
from scripts.web_console.param_metadata import PARAM_GROUPS, PARAM_METADATA, RUNTIME_FACTS, VEHICLE_TAB
from scripts.web_console.param_store import ParamStore, group_paths, read_document
from scripts.web_console.recordings import (
    MIN_FREE_BYTES,
    MIN_FREE_PERCENT,
    RECORD_ROOT,
    RECORD_STATE,
    ROUTE_NAME,
    STAGING_ROOT,
    STORAGE_BLOCKED,
    Recordings,
)
from scripts.web_console.refusal import Refusal
from scripts.web_console.process_status import (
    MANAGER_STATE_HEAD,
    MAX_PROCESSES,
    PROCESS_STATE,
    VERSION_MARKER,
    ProcessStatus,
    process_running,
    read_binary_version,
)
from scripts.web_console.shm_channel import IPC_HEADER, IPC_MAGIC, IPC_VERSION, IpcReader, boottime_ns
from scripts.web_console.state_layout import (
    CONTROL_FORMATS,
    CONTROL_STATE_AT,
    CONTROL_STATE_MAX_AGE_S,
    CONTROL_STATE_SIZE,
    CONTROL_STATE_TOPIC,
    GEAR_PARK,
    HUD_FLAG_BITS,
    LEARNER_STATE_TOPIC,
    LOCALIZATION_STATE_TOPIC,
    MANAGER_STATE_TOPIC,
    MODEL_CALIBRATION_AT,
    MODEL_CALIBRATION_SOURCE_AT,
    MODEL_FLOW_CALIBRATION_AT,
    MODEL_STATE_AT,
    MODEL_STATE_SIZE,
    MODEL_STATE_TOPIC,
    RECORD_STATE_TOPIC,
)
from scripts.web_console.static_assets import STATIC_DIR, StaticAssets

IPC_MESSAGES = ROOT / "src" / "common" / "ipc_messages.h"
WEB_CONSOLE = ROOT / "scripts" / "web_console"
I18N_DIR = STATIC_DIR / "i18n"
LANGUAGES = ("ko", "en")
HANGUL = re.compile(r"[\uac00-\ud7a3]")
# 메타데이터 card(전용 카드가 바꾸는 항목)마다 그 카드가 부르는 API
CARD_ROUTES = {"calibration": "/api/calibration/source"}


def catalog(lang: str) -> dict:
    return json.loads((I18N_DIR / f"{lang}.json").read_text(encoding="utf-8"))


def js_code(text: str) -> list:
    """JS 소스의 (줄 번호, 주석을 뺀 줄). 문자열 밖의 // 뒤와 한 줄 /* */를 지운다(이 저장소의 JS는 // 주석만 쓴다)."""
    lines = []
    for number, line in enumerate(text.splitlines(), 1):
        line = re.sub(r"/\*.*?\*/", "", line)
        for match in re.finditer(r"(?:^|(?<=\s))//", line):
            before = line[:match.start()]
            if all(before.count(quote) % 2 == 0 for quote in "\"'`"):
                line = before
                break
        lines.append((number, line))
    return lines


def page_modules() -> list:
    return sorted(path for path in STATIC_DIR.glob("*.js"))


def cpp(path: Path = IPC_MESSAGES) -> str:
    return path.read_text(encoding="utf-8")


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
    """ControlState: 이름=값. 형식은 state_layout.CONTROL_FORMATS(없으면 uint32)."""
    payload = bytearray(CONTROL_STATE_SIZE)
    for name, value in values.items():
        struct.pack_into(CONTROL_FORMATS.get(name, "<I"), payload, CONTROL_STATE_AT[name], value)
    return bytes(payload)


# 로더가 값을 자르는 Json*Field 표: 그룹 -> (소스, 표 이름들)
RUNTIME_CLAMP_TABLES = {
    "steering": ("src/controls/control_params.cc", ("kSteeringInts", "kSteeringFloats")),
    "adaptive_cruise": ("src/controls/adaptive_cruise.cc", ("kAdaptiveFloats",)),
    "recording": ("src/recording/recordd.cc", ("kRecordingInts",)),
    "display": ("src/common/device_settings.h", ("kDeviceSettingsFloats",)),
}


def runtime_clamps(group: str) -> dict:
    """그룹의 로더 표에 있는 {키: (최저, 최고)}."""
    source, tables = RUNTIME_CLAMP_TABLES[group]
    text = (ROOT / source).read_text(encoding="utf-8")
    row = re.compile(r'\{"(\w+)",\s*(-?[\d.]+)f?,\s*(-?[\d.]+)f?,\s*&\w+::\w+\}')
    clamps = {}
    for table in tables:
        body = re.search(table + r"\[\] = \{(.*?)\n\};", text, re.S).group(1)
        clamps.update({key: (float(low), float(high)) for key, low, high in row.findall(body)})
    return clamps


def metadata_keys(group: str, key: str, meta: dict) -> set:
    """항목 하나의 메타데이터에서 나오는 문구 키(param_state.js describe가 만든다). unit.<단위>는 언어마다 다르게 쓰는
    단위에만 있어 따로 본다."""
    fields = ["label", "description", "increase", "decrease"] + (["learned_role"] if meta.get("learned") else [])
    keys = {f"param.{group}.{key}.{field}" for field in fields}
    keys.update(f"toggle.{name}" for name in meta.get("labels", []))
    keys.add(f"section.{meta['section']}")
    if meta.get("caution"):
        keys.add(f"caution.{meta['caution']}")
    for kind in ("unused", "role"):
        keys.update(f"{kind}.{condition['text']}" for condition in meta.get(kind, []))
    return keys


# ------------------------------------------------------------ 파라미터
class ParamStoreTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.paths = group_paths(self.root / "params")
        documents = {
            "steering": {"torque_kp": 1.0, "enabled": True},
            "adaptive_cruise": {"following_time_s": 1.8},
            "recording": {"enabled": False},
            "display": {"enabled": True, "brightness_percent": 100},
        }
        self.paths["steering"].parent.mkdir()
        for group, document in documents.items():
            self.paths[group].write_text(json.dumps(document), encoding="utf-8")
        self.checked = []  # 실행 여부를 물은 프로세스

        def running(name):
            self.checked.append(name)
            return True

        self.running = running
        self.store = ParamStore(self.paths, running=running)

    def tearDown(self):
        self.temporary.cleanup()

    def test_atomic_single_value_update_reports_the_reader(self):
        """파일만 바꾼다(controlsd가 1초 안에 스스로 다시 읽는다). 읽는 프로세스가 도는지만 알린다."""
        result = self.store.update("steering", {"torque_kp": 1.2})
        self.assertIs(result["reader_running"], True)
        self.assertEqual(self.checked, ["controlsd"])
        self.assertEqual(self.store.read_group("steering"), {"torque_kp": 1.2, "enabled": True})
        self.assertEqual(list(self.paths["steering"].parent.glob("*.tmp")), [])
        stopped = ParamStore(self.paths, running=lambda name: False).update("steering", {"torque_kp": 1.3})
        self.assertIs(stopped["reader_running"], False, "controlsd가 없으면 저장만 됐다고 알린다")
        self.assertEqual(self.store.read_group("steering")["torque_kp"], 1.3)
        self.assertNotIn("kill", (WEB_CONSOLE / "process_status.py").read_text(encoding="utf-8"), "신호를 보내지 않는다")

    def test_card_managed_setting_is_changed_only_through_its_card(self):
        """전용 카드가 바꾸는 항목(메타데이터 card)은 일반 경로로 받지 않는다. 카드의 API가 받을 조건을
        건너뛰지 않게."""
        self.paths["display"].write_text(
            json.dumps({"enabled": True, "brightness_percent": 100, "calibration_use_flow": False}), encoding="utf-8")
        with self.assertRaises(Refusal) as refused:
            self.store.update("display", {"calibration_use_flow": True})
        self.assertEqual((refused.exception.code, refused.exception.status), ("error.param_managed", 400))
        self.assertIs(self.store.read_group("display")["calibration_use_flow"], False)
        self.store.update("display", {"calibration_use_flow": True}, allow_managed=True)
        self.assertIs(self.store.read_group("display")["calibration_use_flow"], True)

    def test_unknown_parameter_is_rejected(self):
        for group, values, code, status in (("adaptive_cruise", {"unknown": 1}, "error.param_unknown", 404),
                                            ("nonexistent", {"following_time_s": 1}, "error.param_unknown", 404),
                                            ("adaptive_cruise", {}, "error.param_no_values", 400)):
            with self.assertRaises(Refusal) as refused:
                self.store.update(group, values)
            self.assertEqual((refused.exception.code, refused.exception.status), (code, status), (group, values))
        self.assertEqual(self.checked, [])
        self.assertEqual(self.store.read_group("adaptive_cruise"), {"following_time_s": 1.8})

    def test_values_must_be_what_the_runtime_loader_reads(self):
        """런타임 로더(utils_json.cc)는 켜고 끄는 값이 true/false가 아니거나 숫자가 유한하지 않으면 파일을
        거부하고, 시작할 때 거부하면 controlsd가 뜨지 못한다. 그런 값은 쓰지 않는다. 메타데이터의 범위 밖,
        step이 정수인 항목의 소수도."""
        self.paths["adaptive_cruise"].write_text(json.dumps({"following_time_s": 1.8, "standstill_gap_m": 5}),
                                                 encoding="utf-8")
        self.paths["display"].write_text(json.dumps({"enabled": True, "brightness_percent": 100}), encoding="utf-8")
        cases = (
            ("steering", {"enabled": 1}, "error.param_invalid.boolean"),
            ("steering", {"enabled": None}, "error.param_invalid.boolean"),
            ("steering", {"torque_kp": True}, "error.param_invalid.number"),
            ("steering", {"torque_kp": None}, "error.param_invalid.number"),
            ("steering", {"torque_kp": float("nan")}, "error.param_invalid.number"),
            ("steering", {"torque_kp": float("inf")}, "error.param_invalid.number"),
            ("steering", {"torque_kp": "12"}, "error.param_invalid.number"),
            # strtof의 범위 오류: float 밖의 큰 수, 정규화되지 않는 작은 수, float로 바꿀 수도 없는 정수
            ("steering", {"torque_kp": 1e39}, "error.param_invalid.number"),
            ("steering", {"torque_kp": 1e-40}, "error.param_invalid.number"),
            ("steering", {"torque_kp": 10 ** 400}, "error.param_invalid.number"),
            ("adaptive_cruise", {"following_time_s": 9.0}, "error.param_invalid.range"),
            ("display", {"brightness_percent": 50.5}, "error.param_invalid.integer"),
        )
        for group, values, code in cases:
            before = self.store.read_group(group)
            with self.assertRaises(Refusal) as refused:
                self.store.update(group, values)
            self.assertEqual((refused.exception.code, refused.exception.status), (code, 400), values)
            self.assertEqual(self.store.read_group(group), before, values)
        self.store.update("steering", {"torque_kp": 0})  # 0은 범위 오류가 아니다
        self.store.update("adaptive_cruise", {"standstill_gap_m": 5.5})  # step 0.5: 처음 값이 정수여도 소수를 받는다
        self.assertEqual(self.store.read_group("adaptive_cruise")["standstill_gap_m"], 5.5)
        self.store.update("adaptive_cruise", {"following_time_s": 2})
        self.assertEqual(self.store.read_group("adaptive_cruise")["following_time_s"], 2)
        self.assertEqual(self.checked, ["controlsd"] * 3)

    def test_broken_file_is_a_coded_refusal(self):
        self.paths["recording"].write_text("{", encoding="utf-8")
        with self.assertRaises(Refusal) as refused:
            self.store.update("recording", {"enabled": True})
        self.assertEqual((refused.exception.code, refused.exception.status), ("error.param_file", 500))

    def test_groups_without_a_reader_check_nothing(self):
        """recordd·overlayd·modeld도 파일을 스스로 다시 읽는다. 저장만 됐다는 안내는 controlsd 그룹에만 있다."""
        self.assertIs(self.store.update("recording", {"enabled": True})["reader_running"], True)
        self.assertEqual(self.checked, [])

    def test_display_update_only_writes_the_file(self):
        """백라이트는 overlayd만 쓴다: 웹 콘솔은 display.json만 바꾸고 아무에게도 알리지 않는다(overlayd가 1초 안에
        읽는다). 예전에는 웹 콘솔이 PWM에도 따로 써서 overlayd가 다시 뜨면 화면과 페이지가 어긋났다."""
        result = self.store.update("display", {"brightness_percent": 35, "enabled": False})
        self.assertIs(result["reader_running"], True)
        self.assertEqual(self.checked, [])
        self.assertEqual(json.loads(self.paths["display"].read_text())["brightness_percent"], 35)
        self.assertNotIn("backlight", (WEB_CONSOLE / "param_store.py").read_text(encoding="utf-8").lower()
                         .split('"""', 2)[2], "파라미터 저장은 백라이트를 다루지 않는다")

    def test_backlight_status_reads_the_pwm(self):
        """페이지의 백라이트 표시는 sysfs의 실제 값이다. 밝기는 보드 최대치로 나눠 설정 값과 같은 눈금으로 보인다."""
        from scripts.web_console.backlight import DisplayBacklight
        with tempfile.TemporaryDirectory() as directory:
            chip = Path(directory) / "pwmchip0"
            board = Path(directory) / "board"
            board.write_text("disp_max_backlight=95\n")
            missing = DisplayBacklight(pwm_chip=chip, board_file=board).status()
            self.assertEqual((missing["available"], missing["brightness_percent"]), (False, None))
            pwm = chip / "pwm3"
            pwm.mkdir(parents=True)
            for name, value in (("enable", "1"), ("period", "100000"), ("duty_cycle", "33250")):
                (pwm / name).write_text(value + "\n")
            status = DisplayBacklight(pwm_chip=chip, board_file=board).status()
            self.assertEqual((status["available"], status["enabled"], status["brightness_percent"]), (True, True, 35))
            (pwm / "duty_cycle").write_text("0\n")
            self.assertFalse(DisplayBacklight(pwm_chip=chip, board_file=board).status()["enabled"], "duty 0은 꺼짐")
        # 같은 식으로 쓴다: duty = 주기 x 밝기 x 보드 최대치(maix_display.cc write_backlight)
        source = cpp(ROOT / "platform" / "maixcam2" / "maix_display.cc")
        self.assertIn("period * std::clamp(percent, 0.0f, 100.0f) / 100.0f * max_percent / 100.0f", source)
        self.assertIn('board_config("disp_max_backlight", 95)', source)
        self.assertIn("DEFAULT_MAX_PERCENT = 95", (WEB_CONSOLE / "backlight.py").read_text(encoding="utf-8"))

    def test_runtime_keys_follow_the_defaults(self):
        """빠진 키는 기본값으로 채우고 기본값에서 사라진 키는 지운다. 남은 키의 튜닝은 그대로."""
        defaults = self.root / "adaptive.defaults.json"
        defaults.write_text(json.dumps({"following_time_s": 1.8, "deceleration_rate_kph_per_s": 1.5}),
                            encoding="utf-8")
        self.paths["adaptive_cruise"].write_text(json.dumps({"following_time_s": 2.2, "retired": 1}),
                                                 encoding="utf-8")
        store = ParamStore(self.paths, {"adaptive_cruise": defaults}, running=self.running)
        self.assertEqual(store.read_group("adaptive_cruise"),
                         {"following_time_s": 2.2, "deceleration_rate_kph_per_s": 1.5})
        self.assertEqual(store.defaults(), {"adaptive_cruise": {"following_time_s": 1.8,
                                                                "deceleration_rate_kph_per_s": 1.5}})

    def test_defaults_sit_next_to_the_params_directory(self):
        """EDGEPILOT_PARAMS_DIR(없으면 params/)와 그 옆의 <이름>.defaults/(업로드 스크립트가 채운다)."""
        defaults = group_paths(self.root / "params.defaults")
        defaults["adaptive_cruise"].parent.mkdir()
        for group, path in defaults.items():  # 업로드 스크립트는 그룹마다 채운다
            path.write_text(self.paths[group].read_text(encoding="utf-8"), encoding="utf-8")
        defaults["adaptive_cruise"].write_text(json.dumps({"following_time_s": 1.5, "added": True}), encoding="utf-8")
        with mock.patch.dict(os.environ, {"EDGEPILOT_PARAMS_DIR": str(self.root / "params")}):
            store = ParamStore(running=self.running)
        self.assertEqual(store.paths, self.paths)
        self.assertEqual(store.read_group("adaptive_cruise"), {"following_time_s": 1.8, "added": True})
        self.assertEqual(store.snapshot()["defaults"]["adaptive_cruise"], {"following_time_s": 1.5, "added": True})

    def test_snapshot_has_what_the_page_reads(self):
        snapshot = self.store.snapshot()
        self.assertEqual(set(snapshot), {"groups", "vehicle_tab", "params", "defaults", "metadata", "paths"})
        self.assertEqual(list(snapshot["groups"]), list(PARAM_GROUPS), "tab order")
        for group, spec in snapshot["groups"].items():
            self.assertEqual(set(spec), {"sections", "reader"}, group)
        self.assertEqual(snapshot["paths"]["steering"], str(self.paths["steering"]))


class ParamMetadataTest(unittest.TestCase):
    def test_repository_params_have_complete_ui_metadata(self):
        """항목마다 섹션이 있고 두 언어 모두 그 항목의 문구(metadata_keys)가 있으며, 섹션은 그 항목이 나오는 탭(파일
        그룹, tab="vehicle"이면 차량 특성 탭)의 섹션이고, 빈 섹션이 없다."""
        catalogs = {lang: catalog(lang) for lang in LANGUAGES}
        tabs = {}
        for group, spec in PARAM_GROUPS.items():
            params = json.loads((ROOT / "params" / spec["file"]).read_text(encoding="utf-8"))
            metadata = PARAM_METADATA[group]
            self.assertEqual(set(params), set(metadata), group)
            tabs.setdefault(group, (spec["sections"], set()))
            for key, value in params.items():
                meta = metadata[key]
                self.assertTrue(meta.get("section"), f"{group}.{key}.section")
                for lang, texts in catalogs.items():
                    for name in metadata_keys(group, key, meta):
                        self.assertTrue(texts.get(name), f"{lang}: {name}")
                    if not meta.get("learned"):
                        self.assertNotIn(f"param.{group}.{key}.learned_role", texts, f"{lang}: {group}.{key}")
                self.assertIn(meta.get("tab", group), {group, "vehicle"}, f"{group}.{key}")
                if meta.get("card"):
                    # 차량 특성 탭의 전용 카드가 자기 API로 바꾸는 항목: 섹션은 파일 그룹의 것이지만 그 탭에 카드가
                    # 없으니 섹션을 채우지 않는다
                    self.assertEqual(meta.get("tab"), "vehicle", f"{group}.{key}")
                    self.assertIn(meta["card"], CARD_ROUTES, f"{group}.{key}")
                    self.assertIn(meta["section"], spec["sections"], f"{group}.{key}")
                    page = (STATIC_DIR / "vehicle_view.js").read_text(encoding="utf-8")
                    self.assertIn(f'api.post("{CARD_ROUTES[meta["card"]]}"', page, f"{group}.{key}")
                else:
                    if meta.get("tab") == "vehicle":
                        self.assertEqual(group, "steering", f"{group}.{key}: the vehicle tab edits steering.json")
                    sections, used = tabs.setdefault(meta.get("tab", group), (VEHICLE_TAB["sections"], set()))
                    self.assertIn(meta["section"], sections, f"{group}.{key}")
                    used.add(meta["section"])
                if isinstance(value, (int, float)) and not isinstance(value, bool):
                    for field in ("step", "min", "max"):
                        self.assertIsInstance(meta.get(field), (int, float), f"{group}.{key}.{field}")
                    self.assertLessEqual(meta["min"], value, f"{group}.{key}")
                    self.assertLessEqual(value, meta["max"], f"{group}.{key}")
        for tab, (sections, used) in tabs.items():
            self.assertEqual([name for name in sections if name not in used], [], f"{tab}: empty sections")

    def test_conditions_name_real_options(self):
        """다른 항목에 따른 안내(unused·role, param_metadata.when): 조건의 항목이 있고 자기 자신이 아니며, 값은 그
        항목이 가질 수 있는 것(켜고 끄는 항목은 bool, 숫자 항목은 범위 안의 수, 런타임 사실은 bool)이다. also도 같고
        문구가 없다. 문구가 두 언어에 다 있는지는 I18nCatalogTest가 본다."""
        files = {group: read_document(path) for group, path in group_paths(ROOT / "params").items()}

        def check(condition, group, key, name):
            self.assertEqual(set(condition) - {"group", "also"}, {"key", "value", "text"}, name)
            other_group, other_key, value = condition.get("group", group), condition["key"], condition["value"]
            if other_group == "runtime":
                self.assertIn(other_key, RUNTIME_FACTS, name)
                self.assertIsInstance(value, bool, name)
            else:
                self.assertIn(other_key, files[other_group], name)
                self.assertNotEqual((other_group, other_key), (group, key), name)
                other = PARAM_METADATA[other_group][other_key]
                if isinstance(files[other_group][other_key], bool):
                    self.assertIsInstance(value, bool, name)
                else:
                    self.assertIsInstance(value, (int, float), name)
                    self.assertNotIsInstance(value, bool, name)
                    self.assertTrue(other["min"] <= value <= other["max"], name)
            if "also" in condition:
                self.assertEqual(condition["also"]["text"], "", name)
                check(condition["also"], group, key, f"{name} also")

        seen = 0
        for group, items in PARAM_METADATA.items():
            for key, meta in items.items():
                for kind in ("unused", "role"):
                    for condition in meta.get(kind, []):
                        seen += 1
                        self.assertTrue(condition["text"], f"{group}.{key} {kind}")
                        check(condition, group, key, f"{group}.{key} {kind} {condition}")
        self.assertGreater(seen, 0)

    def test_vehicle_tab_matches_the_page(self):
        """차량 특성 탭: 학습기 섹션마다 스위치가 하나 있고, 나머지 항목은 학습값을 읽는 법(vehicle_view.js LIVE)과
        자동 학습 중의 쓰임(learned, learned_role 문구)이 있다. 학습기 이름은 페이지의 LEARNERS와 같다."""
        steering = PARAM_METADATA["steering"]
        page = (STATIC_DIR / "vehicle_view.js").read_text(encoding="utf-8")
        live = set(re.findall(r"^  (\w+): \{$", re.search(r"const LIVE = \{\n(.*?)\n\};", page, re.S).group(1), re.M))
        learners_block = re.search(r"const LEARNERS = \{\n(.*?)\n\};", page, re.S).group(1)
        learners = set(re.findall(r"^  (\w+): \{$", learners_block, re.M))
        learned = set()
        for name, spec in VEHICLE_TAB["learners"].items():
            self.assertIn(name, VEHICLE_TAB["sections"])
            switch = steering[spec["switch"]]
            self.assertEqual((switch.get("tab"), switch["section"]), ("vehicle", name), spec["switch"])
            keys = {key for key, meta in steering.items() if meta.get("tab") == "vehicle" and meta["section"] == name}
            for key in keys - {spec["switch"]}:
                self.assertTrue(steering[key].get("learned"), key)
                learned.add(key)
        self.assertEqual(live, learned)
        self.assertEqual(learners, {spec["learner"] for spec in VEHICLE_TAB["learners"].values()})
        self.assertFalse([key for key, meta in steering.items() if meta.get("learned") and key not in learned])

    def test_ui_ranges_match_runtime_clamps(self):
        """로더는 Json*Field 표대로 값을 자른다. 편집기도 같은 min/max를 보여야 하고, 아니면 런타임이 말없이
        바꾸는 값을 받아들인다."""
        self.assertEqual(set(RUNTIME_CLAMP_TABLES), set(PARAM_GROUPS))
        for group in PARAM_GROUPS:
            runtime = runtime_clamps(group)
            self.assertTrue(runtime, f"{group}: no rows parsed")
            ui = {key: meta for key, meta in PARAM_METADATA[group].items() if "min" in meta}
            self.assertEqual(set(runtime), set(ui), group)
            for key, (low, high) in runtime.items():
                self.assertEqual((float(ui[key]["min"]), float(ui[key]["max"])), (low, high), f"{group}.{key}")

    def test_params_readme_ranges_match_runtime_clamps(self):
        """params/README.md의 Range 열이 로더 표(Json*Field)와 같아야 한다: 문서만 보고 넣은 값이 말없이 잘렸다
        (예전 hold angle 0~180, 로더는 0~84)."""
        readme = (ROOT / "params" / "README.md").read_text(encoding="utf-8")
        sections = dict(re.findall(r"^## (\S+\.json)\n(.*?)(?=^## |\Z)", readme, re.S | re.M))

        def number(text):
            scale = {"k": 1e3, "M": 1e6}.get(text[-1], 1.0)
            return float(text.rstrip("kM")) * scale

        for group, spec in PARAM_GROUPS.items():
            documented = {key: (number(low), number(high)) for key, low, high in
                          re.findall(r"^\| `(\w+)` \|[^|]*\|[^|]*\| (-?[\d.]+[kM]?) to (-?[\d.]+[kM]?) \|",
                                     sections[spec["file"]], re.M)}
            self.assertEqual(documented, runtime_clamps(group), spec["file"])

    def test_ui_text_names_no_particular_car(self):
        """다른 차도 붙일 수 있게 화면 문구는 차종을 말하지 않는다. 한글이 바로 붙는 "K7은"도 잡는다."""
        texts = [json.dumps(PARAM_METADATA, ensure_ascii=False), json.dumps(PARAM_GROUPS, ensure_ascii=False)]
        texts += [(I18N_DIR / f"{lang}.json").read_text(encoding="utf-8") for lang in LANGUAGES]
        texts += [path.read_text(encoding="utf-8") for path in STATIC_DIR.glob("*.html")]
        texts += [path.read_text(encoding="utf-8") for path in STATIC_DIR.glob("*_view.js")]
        texts += [(STATIC_DIR / name).read_text(encoding="utf-8")
                  for name in ("console.js", "panda_card.js", "lateral_nn_card.js")]
        for text in texts:
            self.assertNotRegex(text, r"(?i)(?<![0-9a-z])k7(?![0-9a-z])|그랜저|grandeur")
        self.assertEqual([path.name for path in STATIC_DIR.rglob("*") if "k7" in path.name.lower()], [])



class I18nCatalogTest(unittest.TestCase):
    """화면 글자는 언어마다 static/i18n/<언어>.json에만 있다: 두 언어의 키와 자리 값이 같고, 페이지와 서버가 쓰는 키가
    다 있으며 안 쓰는 키가 없고, 코드에는 화면에 나갈 한글이 남지 않는다."""

    def setUp(self):
        self.catalogs = {lang: catalog(lang) for lang in LANGUAGES}

    def test_languages_have_the_same_keys_and_placeholders(self):
        placeholders = lambda text: set(re.findall(r"\{(\w+)\}", text))
        ko, en = self.catalogs["ko"], self.catalogs["en"]
        self.assertEqual(sorted(set(ko) ^ set(en)), [])
        for key in ko:
            forms = {lang: texts[key] if isinstance(texts[key], dict) else {"other": texts[key]}
                     for lang, texts in self.catalogs.items()}
            for lang, by_count in forms.items():
                self.assertIn("other", by_count, f"{lang}: {key}")
                self.assertLessEqual(set(by_count), {"zero", "one", "two", "few", "many", "other"}, f"{lang}: {key}")
                for text in by_count.values():
                    self.assertIsInstance(text, str, f"{lang}: {key}")
                    self.assertTrue(text.strip(), f"{lang}: {key}")
                    if len(by_count) > 1:
                        self.assertIn("count", placeholders(text), f"{lang}: {key} plural without {{count}}")
            names = {lang: set().union(*(placeholders(text) for text in by_count.values()))
                     for lang, by_count in forms.items()}
            self.assertEqual(names["ko"], names["en"], key)
            self.assertFalse(HANGUL.search(json.dumps(en[key], ensure_ascii=False)), f"en: {key}")

    def test_page_and_server_keys_exist_and_none_is_unused(self):
        """페이지의 "이름공간.키" 글자와 index.html의 data-i18n*, 서버의 거부 코드가 다 있어야 한다. 반대로 문구마다
        그것을 쓰는 곳(글자 그대로, `접두사.${...}` 템플릿, 메타데이터에서 나오는 키, 서버 코드)이 있어야 한다."""
        keys = set(self.catalogs["en"])
        spaces = {key.split(".")[0] for key in keys}
        used, prefixes = set(), set()
        for module in page_modules():
            code = "\n".join(line for _, line in js_code(module.read_text(encoding="utf-8")))
            # el("section.section", ...)의 "태그.클래스"는 키가 아니다
            for key in re.findall(r"(?<!el\()[\"'`]([a-z_]+(?:\.[A-Za-z0-9_]+)+)[\"'`]", code):
                if key.split(".")[0] not in spaces:
                    continue
                if key not in keys and any(other.startswith(f"{key}.") for other in keys):
                    prefixes.add(f"{key}.")  # coded("panda.reason", 코드)처럼 접두사만 쓰는 글자
                    continue
                self.assertIn(key, keys, f"{module.name}: {key}")
                used.add(key)
            prefixes.update(re.findall(r"`([a-z_]+(?:\.[A-Za-z0-9_]+)*\.)\$\{", code))
        page = (STATIC_DIR / "index.html").read_text(encoding="utf-8")
        for key in re.findall(r'data-i18n(?:-[a-z-]+)?="([^"]+)"', page):
            self.assertIn(key, keys, f"index.html: {key}")
            used.add(key)
        # 메타데이터에서 나오는 키(param_state.js가 만든다)
        for group, items in PARAM_METADATA.items():
            for key, meta in items.items():
                used.update(metadata_keys(group, key, meta))
                if meta.get("unit") and f"unit.{meta['unit']}" in keys:
                    used.add(f"unit.{meta['unit']}")
        for group, spec in PARAM_GROUPS.items():
            used.add(f"group.{group}.label")
            used.update(f"section.{name}" for name in spec["sections"])
            if f"group.{group}.note" in keys:
                used.add(f"group.{group}.note")
        used.update(f"section.{name}" for name in VEHICLE_TAB["sections"])
        # 서버의 코드: Panda 사유·단계·이미지 문제와 거부(Refusal)
        used.update(f"panda.reason.{code}" for code in REASONS)
        used.update(f"panda.step.{code}" for code in STEPS)
        used.update(f"panda.image.{code}" for code in IMAGE_ERRORS if code)
        for module in WEB_CONSOLE.glob("*.py"):
            text = module.read_text(encoding="utf-8")
            for code in re.findall(r'Refusal\(\s*"([^"]+)"', text):
                self.assertIn(code, keys, f"{module.name}: {code}")
                used.add(code)
            for prefix in re.findall(r'Refusal\(\s*f"([a-z_.]+)\{', text):
                self.assertTrue(any(key.startswith(prefix) for key in keys), f"{module.name}: {prefix}")
        missing = sorted(used - keys)
        self.assertEqual(missing, [], "keys used but not in the catalogs")
        unused = sorted(key for key in keys - used if not any(key.startswith(prefix) for prefix in prefixes))
        self.assertEqual(unused, [], "catalog keys nothing uses")

    def test_nothing_shadows_the_translation_function(self):
        """t를 가져다 쓰는 모듈은 t라는 이름을 따로 묶지 않는다(지역 변수, 매개변수). 가리면 t("...")가 숫자를 부르다
        멈춘다: BEV의 frame()이 모델 시각을 t에 두어 상태 줄이 "t is not a function"으로 끊겼다(2026-10-14)."""
        # const t, t =>, catch (t), 그리고 t를 묶는 구조 분해(const {t, x} = …, const [t] = …; {t: 다른 이름}은 괜찮다)
        binding = re.compile(r"\b(?:let|const|var)\s+t\b|(?<![\w.$])t\s*=>|\bcatch\s*\(\s*t\s*\)"
                             r"|\b(?:let|const|var)\s*[{\[][^}\]=]*(?<![\w.$])t\b(?!\s*:)")
        params = re.compile(r"(?<![\w.$])(\w*)\s*\(([^()]*)\)\s*(?:=>|\{)")
        found = []
        for module in page_modules():
            text = module.read_text(encoding="utf-8")
            if not re.search(r'import\s*\{[^}]*\bt\b[^}]*\}\s*from\s*"\./i18n\.js"', text):
                continue
            for number, line in js_code(text):
                if binding.search(line):
                    found.append(f"{module.name}:{number}")
                    continue
                for name, args in params.findall(line):
                    if name in ("if", "while", "for", "switch", "return"):
                        continue
                    names = [re.split(r"[\s=]", arg.strip().lstrip("{[").rstrip("}]"))[0] for arg in args.split(",")]
                    if "t" in names:
                        found.append(f"{module.name}:{number}")
        self.assertEqual(found, [])

    def test_no_screen_text_left_in_code(self):
        """한글은 주석과 문구 묶음에만 있다: 페이지 모듈과 index.html의 코드, 서버 모듈의 문자열(독스트링 빼고).
        그 언어로 쓴 언어 이름처럼 일부러 둔 줄은 i18n-allow 주석을 단다."""
        found = []
        for module in page_modules():
            text = module.read_text(encoding="utf-8")
            raw = text.splitlines()
            found += [f"{module.name}:{number}" for number, line in js_code(text)
                      if HANGUL.search(line) and "i18n-allow" not in raw[number - 1]]
        page = re.sub(r"<!--.*?-->", "", (STATIC_DIR / "index.html").read_text(encoding="utf-8"), flags=re.S)
        found += ["index.html" for _ in HANGUL.findall(page)][:1]
        # CSS가 그리는 글자(content)도 화면 글자다: 문구 묶음에서 data-* 속성으로 받는다(attr())
        for sheet in STATIC_DIR.glob("*.css"):
            code = re.sub(r"/\*.*?\*/", "", sheet.read_text(encoding="utf-8"), flags=re.S)
            found += [f"{sheet.name}: {value}" for value in re.findall(r"content:\s*([^;}]*)", code)
                      if HANGUL.search(value)]
        for module in WEB_CONSOLE.glob("*.py"):
            tree = ast.parse(module.read_text(encoding="utf-8"))
            docstrings = {id(node.body[0].value) for node in ast.walk(tree)
                          if isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef))
                          and node.body and isinstance(node.body[0], ast.Expr)
                          and isinstance(node.body[0].value, ast.Constant)}
            found += [f"{module.name}:{node.lineno}" for node in ast.walk(tree)
                      if isinstance(node, ast.Constant) and isinstance(node.value, str) and id(node) not in docstrings
                      and HANGUL.search(node.value)]
        self.assertEqual(found, [])

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
        bits = cpp_flags("kLearner")
        self.assertEqual([snake(name) for name in bits], list(LEARNER_FLAGS))
        self.assertEqual(list(bits.values()), list(range(len(LEARNER_FLAGS))))

    def monitor(self) -> LearnerMonitor:
        return LearnerMonitor(str(self.path))

    def test_monitor_decodes_and_skips_torn_writes(self):
        self.publish(4, timestamp_ns=5_000_000_000, flags=0b1000000011, steer_ratio=14.88,
                     bucket_points=[12, 204, 464, 1440, 1500, 1036, 292, 49])
        monitor = self.monitor()
        stamp, state = monitor.sample()
        self.assertEqual(stamp, 123)
        self.assertAlmostEqual(state["steer_ratio"], 14.88, places=5)
        self.assertEqual(state["bucket_points"][4], 1500)
        self.assertTrue(state["flags"]["vehicle_inputs_ok"] and state["flags"]["vehicle_valid"])
        self.assertTrue(state["flags"]["use_vehicle"] and not state["flags"]["use_torque"])
        self.publish(5, steer_ratio=1.0)  # 쓰는 중
        self.assertIsNone(monitor.sample())
        self.path.unlink()
        self.assertIsNone(monitor.sample())

    def test_reader_follows_a_channel_that_grows_in_place(self):
        """생산자는 있던 공유 메모리 파일을 제자리에서 늘린다(ftruncate, 같은 inode). 새 배포의 더 큰 메시지를 옛 매핑의
        잘린 조각으로 읽으면 해석이 깨져 콘솔 전체가 멈췄다. 파일이 메시지보다 작으면 없는 것으로 본다."""
        size = LEARNER_STATE.size
        publish(self.path, bytes(size - 16))
        reader = IpcReader(str(self.path), size)
        self.assertIsNone(reader.read_payload())
        inode = self.path.stat().st_ino
        publish(self.path, bytes(size + 32), seq=4)
        self.assertEqual(self.path.stat().st_ino, inode)
        seq, _, payload = reader.read_payload()
        self.assertEqual((seq, len(payload)), (4, size))

    def test_runtime_facts_follow_a_fresh_state(self):
        """파라미터 안내가 보는 런타임 사실: controlsd가 학습한 토크 모델을 읽었는지(플래그). 상태가 없거나 낡으면
        None이다(그때는 어느 조건도 참이 아니다)."""
        monitor = self.monitor()
        self.assertEqual(monitor.runtime_facts(), {"lateral_nn_loaded": None})
        self.assertEqual(set(monitor.runtime_facts()), set(RUNTIME_FACTS))
        loaded = 1 << LEARNER_FLAGS.index("lateral_nn_loaded")
        publish(self.path, learner_payload(flags=loaded), seq=2)
        self.assertEqual(monitor.runtime_facts(), {"lateral_nn_loaded": True})
        publish(self.path, learner_payload(flags=0), seq=4)
        self.assertEqual(monitor.runtime_facts(), {"lateral_nn_loaded": False})
        publish(self.path, learner_payload(flags=loaded), seq=6, stamp=123)  # 오래전 상태
        self.assertEqual(monitor.runtime_facts(), {"lateral_nn_loaded": None})

    def test_polled_status_carries_the_runtime_facts(self):
        """페이지는 폴링 응답의 runtime으로 안내를 갱신한다(param_state.updateRuntime). 페이지를 연 뒤에 controlsd가
        모델을 읽어도 안내가 따라온다."""
        monitor = self.monitor()
        self.assertEqual(monitor.status()["runtime"], {"lateral_nn_loaded": None})
        publish(self.path, learner_payload(flags=1 << LEARNER_FLAGS.index("lateral_nn_loaded")), seq=2)
        self.assertEqual(monitor.status()["runtime"], {"lateral_nn_loaded": True})
        publish(self.path, learner_payload(flags=0), seq=4, stamp=123)  # 낡은 상태는 모른다(None)
        self.assertEqual(monitor.status()["runtime"], {"lateral_nn_loaded": None})
        server = (WEB_CONSOLE / "console_server.py").read_text(encoding="utf-8")
        self.assertEqual(server.count('"runtime": learners.runtime_facts()'), 1)  # 파라미터
        self.assertIn('"runtime": learner["runtime"]', server)  # 학습 토크 모델(GET·POST가 같이 쓴다)
        self.assertEqual(server.count("return lateral_nn_status()"), 2)
        for module in ("lateral_nn_card.js", "vehicle_view.js"):
            self.assertIn("updateRuntime(", (WEB_CONSOLE / "static" / module).read_text(encoding="utf-8"), module)

    def test_monitor_keeps_one_trend_row_per_publish(self):
        monitor = self.monitor()
        self.assertFalse(monitor.status()["available"])
        self.publish(2, timestamp_ns=1_000_000_000, lat_accel_factor=4.44)
        monitor.sample()
        monitor.sample()
        self.publish(4, timestamp_ns=2_000_000_000, lat_accel_factor=4.40)
        status = monitor.status()
        self.assertTrue(status["available"])
        self.assertEqual([row[0] for row in monitor.trend()["rows"]], [1.0, 2.0])
        self.assertAlmostEqual(status["trend_row"][LEARNER_TREND_COLUMNS.index("lat_accel_factor")], 4.40, places=5)

    def test_trend_columns_and_bank_match_the_page(self):
        """추이 행의 열과 편경사 롤 환산은 서버(learner_trend_row)와 페이지(vehicle_view.js TREND_COLUMNS, 편경사 보정
        카드의 GRAVITY 식)가 따로 안다: 열마다 그 이름의 값이 들어가고, 페이지의 열 이름·순서와 식이 같아야 한다."""
        self.publish(2, timestamp_ns=3_000_000_000, steer_ratio=15.5, angle_offset_average_deg=0.25,
                     angle_offset_deg=0.5, roll_rad=0.01, road_bank_lat_accel=-0.2, lat_accel_factor_raw=2.5,
                     lat_accel_factor=2.75)
        row = dict(zip(LEARNER_TREND_COLUMNS, self.monitor().status()["trend_row"]))
        expected = {"time_s": 3.0, "steer_ratio": 15.5, "angle_offset_average_deg": 0.25, "angle_offset_deg": 0.5,
                    "roll_deg": math.degrees(0.01), "bank_roll_deg": math.degrees(0.2 / GRAVITY),
                    "lat_accel_factor_raw": 2.5, "lat_accel_factor": 2.75}
        self.assertEqual(set(row), set(expected))
        for name, value in expected.items():
            self.assertAlmostEqual(row[name], value, places=3, msg=name)
        page = (STATIC_DIR / "vehicle_view.js").read_text(encoding="utf-8")
        columns = re.findall(r'"(\w+)"', re.search(r"const TREND_COLUMNS = \[(.*?)\];", page, re.S).group(1))
        self.assertEqual(columns, list(LEARNER_TREND_COLUMNS))
        charts = re.search(r"const TREND_CHARTS = \[\n(.*?)\n\];", page, re.S).group(1)
        series = re.findall(r'\["(\w+)", "\w+"\]', charts)
        self.assertTrue(series)
        self.assertLessEqual(set(series), set(columns) - {"time_s"})
        self.assertEqual(float(re.search(r"const GRAVITY = ([\d.]+);", page).group(1)), GRAVITY)
        self.assertIn("deg(-s.road_bank_lat_accel / GRAVITY)", page)

    def test_monitor_trend_rows_are_a_second_apart_and_cut_by_time(self):
        monitor = self.monitor()
        # 요청마다 sample이 불려도 행은 0.9초 이상 떨어져야 쌓인다
        for seq, stamp_s in enumerate((1.0, 1.3, 1.6, 2.0, 2.5, 3.1), start=1):
            self.publish(2 * seq, timestamp_ns=int(stamp_s * 1e9))
            monitor.sample()
        self.assertEqual([row[0] for row in monitor.trend()["rows"]], [1.0, 2.0, 3.1])
        # 10분 창은 행 개수가 아니라 마지막 행의 시각으로 자른다
        self.publish(20, timestamp_ns=int(602.0 * 1e9))
        monitor.sample()
        self.assertEqual([row[0] for row in monitor.trend()["rows"]], [2.0, 3.1, 602.0])


class LocalizationStateTest(unittest.TestCase):
    def test_layout_matches_cpp(self):
        """크기만 대조하면 필드 순서가 바뀌어도 모른다: 형식·이름·배열 길이를 C++ 구조체 차례대로 대조한다."""
        self.assertEqual(LOCALIZATION_STATE.size, cpp_size("LocalizationState"))
        kinds = {"Q": "uint64_t", "I": "uint32_t", "i": "int32_t", "f": "float"}
        fields = [(kinds[fmt[-1]], name, int(fmt[:-1] or 1)) for name, fmt in LOCALIZATION_FIELDS]
        body = re.search(r"struct LocalizationState \{(.*?)\n\};", cpp(), re.S).group(1)
        declared = [(kind, name, int(count or 1)) for kind, name, count in
                    re.findall(r"(uint64_t|uint32_t|int32_t|float) (\w+)(?:\[(\d+)\])?", body)]
        self.assertEqual(fields, declared)
        flags = cpp_flags("kLocalization")
        state_flags = [bit for name, bit in flags.items() if not name.startswith(("Invalid", "CameraGuarded"))]
        input_flags = [bit for name, bit in flags.items() if name.startswith(("Invalid", "CameraGuarded"))]
        self.assertEqual(state_flags, list(range(len(LOCALIZATION_FLAGS))))
        self.assertEqual(input_flags, list(range(len(LOCALIZATION_INPUT_FLAGS))))

    def test_reader_decodes(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "edgepilot_localization"
            reader = LocalizationReader(str(path))
            self.assertFalse(reader.status()["available"])
            publish(path, pack_fields(LOCALIZATION_STATE, LOCALIZATION_FIELDS, {
                "flags": 0b100001, "lag_status": 1, "input_flags": 0b1010, "lateral_delay_s": 0.42,
                "lag_valid_blocks": 6, "angular_velocity_calib": [0.0, 0.0, 0.1]}))
            result = reader.status()
            self.assertTrue(result["available"])
            state = result["state"]
            self.assertAlmostEqual(state["lateral_delay_s"], 0.42, places=5)
            self.assertEqual(state["lag_valid_blocks"], 6)
            self.assertAlmostEqual(state["angular_velocity_calib"][2], 0.1, places=5)
            self.assertTrue(state["flags"]["filter_valid"] and state["flags"]["lag_restored"])
            self.assertFalse(state["flags"]["inputs_ok"])
            self.assertTrue(state["input_flags"]["gyro_invalid"] and state["input_flags"]["camera_guarded"])
            self.assertFalse(state["input_flags"]["camera_invalid"])


# ------------------------------------------------------------ 카메라 보정
class CalibrationControlTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        root = Path(self.temporary.name)
        self.model = root / "edgepilot_model_state"
        self.control = root / "edgepilot_control_state"
        self.request = root / "edgepilot_calibration_reset"
        paths = group_paths(root / "params")
        paths["display"].parent.mkdir()
        paths["display"].write_text(json.dumps({"enabled": True, "calibration_use_flow": False}), encoding="utf-8")
        self.store = ParamStore(paths, running=lambda name: True)
        self.calibration = CalibrationControl(self.store, str(self.model), str(self.control), str(self.request))

    def tearDown(self):
        self.temporary.cleanup()

    def publish_model(self, status, blocks, rpy_deg, flow=None, source=None):
        payload = bytearray(MODEL_STATE_SIZE)
        rad = [x * 3.141592653589793 / 180 for x in rpy_deg]
        CALIBRATION_STATE.pack_into(payload, MODEL_CALIBRATION_AT, status, blocks, *rad, 0.1, 0.2, 0.3)
        if flow is not None:
            FLOW_CALIBRATION_STATE.pack_into(payload, MODEL_FLOW_CALIBRATION_AT, *flow)
        if source is not None:
            CALIBRATION_SOURCE_STATE.pack_into(payload, MODEL_CALIBRATION_SOURCE_AT, *source)
        publish(self.model, bytes(payload))

    def publish_control(self, engaged, stamp=None):
        publish(self.control, control_payload(engaged=engaged, vehicle_fresh=1), stamp=stamp)

    def test_layout_and_request_path_match_cpp(self):
        offset = re.search(r"offsetof\(ModelState, calibration\) == (\d+)", cpp())
        self.assertEqual(int(offset.group(1)), MODEL_CALIBRATION_AT)
        self.assertEqual(CALIBRATION_STATE.size, 32)
        for field, at in (("flow_calibration", MODEL_FLOW_CALIBRATION_AT),
                          ("calibration_source", MODEL_CALIBRATION_SOURCE_AT)):
            self.assertEqual(int(re.search(rf"offsetof\(ModelState, {field}\) == (\d+)", cpp()).group(1)), at, field)
        self.assertEqual(FLOW_CALIBRATION_STATE.size, cpp_size("FlowCalibrationState"))
        self.assertEqual(CALIBRATION_SOURCE_STATE.size, cpp_size("CalibrationSourceState"))
        self.assertEqual(cpp_flags("kCalibrationSource"), {"FlowUsable": 0, "Pending": 1, "Ramping": 2})
        service = cpp(ROOT / "src" / "model" / "calibration_service.cc")
        self.assertRegex(service, rf'"EDGEPILOT_CALIBRATION_RESET_PATH",\s*"{re.escape(CALIBRATION_RESET_PATH)}"')

    def test_status_decodes_model_state(self):
        missing = self.calibration.status()
        self.assertEqual((missing["available"], missing["stale"], missing["age_s"]), (False, True, None))
        self.publish_model(1, 7, [0.0, 0.29, -0.18])
        status = self.calibration.status()
        self.assertEqual((status["available"], status["stale"]), (True, False))
        self.assertEqual((status["status"], status["valid_blocks"]), ("calibrated", 7))
        self.assertAlmostEqual(status["rpy_deg"][1], 0.29, places=3)
        self.assertAlmostEqual(status["rpy_deg"][2], -0.18, places=3)

    def test_status_decodes_flow_and_source(self):
        rad = 3.141592653589793 / 180
        flow = (1234, 2, 240, 14, -0.15 * rad, -0.03 * rad, 0.04 * rad, 0.06 * rad, -0.2 * rad, 0.1 * rad, 0.002 * rad,
                297, 296, 1, 6.1, 12, 9)
        source = (1, 1, 1 | 4, 1, 50, 0.0, -0.06 * rad, -0.30 * rad)
        self.publish_model(1, 50, [0.0, -0.15, -0.03], flow=flow, source=source)
        status = self.calibration.status()
        self.assertEqual((status["source"], status["flow_usable"], status["ramping"]), ("flow", True, True))
        self.assertFalse(status["switch_pending"])
        f = status["flow"]
        self.assertEqual((f["status"], f["estimate_blocks"], f["windows"], f["mount_changes"], f["rejected_windows"]),
                         ("estimate", 12, 240, 1, 9))
        self.assertAlmostEqual(f["yaw_deg"], -0.03, places=3)
        self.assertAlmostEqual(f["sd_deg"][1], 0.06, places=3)
        self.assertAlmostEqual(f["window"]["parallax_px"], 6.1, places=2)
        self.assertEqual(f["window"]["tracks"], 297)
        self.assertEqual((status["pose"]["status"], status["pose"]["valid_blocks"]), ("calibrated", 50))
        self.assertAlmostEqual(status["pose"]["rpy_deg"][2], -0.30, places=3)

    def test_reset_is_refused_while_engaged(self):
        self.publish_control(engaged=1)
        with self.assertRaises(Refusal) as refused:
            self.calibration.request_reset()
        self.assertEqual((refused.exception.code, refused.exception.status), ("error.calibration_reset_engaged", 409))
        self.assertFalse(self.request.exists())
        self.publish_control(engaged=0)
        self.assertTrue(self.calibration.request_reset()["reset_pending"])
        self.assertTrue(self.request.exists())

    def test_source_is_changed_only_while_disengaged(self):
        """보정 출처는 이 카드만 바꾸고(파라미터 API는 받지 않는다) 결합 중에는 받지 않는다. 상태의 use_flow는 modeld가
        다시 읽기 전에도 고른 값을 보이게 설정 파일에서 읽는다."""
        self.publish_control(engaged=1)
        with self.assertRaises(Refusal):
            self.calibration.set_source(True)
        self.assertIs(self.store.read_group("display")["calibration_use_flow"], False)
        self.publish_control(engaged=0)
        self.assertTrue(self.calibration.set_source(True)["use_flow"])
        self.assertIs(self.store.read_group("display")["calibration_use_flow"], True)
        self.assertFalse(self.calibration.set_source(False)["use_flow"])

    def test_stale_or_missing_controlsd_does_not_block(self):
        self.assertFalse(self.calibration.engaged())
        self.publish_control(engaged=1, stamp=boottime_ns() - 5_000_000_000)
        self.assertFalse(self.calibration.engaged())

    def test_empty_reset_path_variable_means_the_default(self):
        """C++ env_string처럼 빈 변수는 없는 것이다(빈 경로가 "."이 되어 초기화 요청이 늘 걸려 있는 것처럼 보였다)."""
        with mock.patch.dict(os.environ, {"EDGEPILOT_CALIBRATION_RESET_PATH": ""}):
            calibration = CalibrationControl(self.store, str(self.model), str(self.control))
        self.assertEqual(str(calibration.request_path), CALIBRATION_RESET_PATH)


# ------------------------------------------------------------ Panda 펌웨어
def panda_image(version="EDGE-f9907afb-DEBUG", body=1024):
    """sign.py의 꼴: [본문 길이][... 버전 ...][VERS][2] 뒤에 서명 128바이트."""
    data = bytearray(b"\x11" * body)
    struct.pack_into("<I", data, 0, body)
    data[63:64 + len(version) + 1] = b"\0" + version.encode() + b"\0"
    data[body - 8:body] = b"VERS" + struct.pack("<I", 2)
    return bytes(data) + b"\xab" * PANDA_SIGNATURE_BYTES


class LateralNnModelsTest(unittest.TestCase):
    """학습 토크 모델 카드: 레포의 모델 파일 이름, 고르기 파일 읽기·쓰기, 거부, controlsd 상태, C++과 같은 경로·규칙."""

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        root = Path(self.temporary.name)
        self.models_dir = root / "models"
        shutil.copytree(ROOT / MODEL_DIR, self.models_dir)
        self.selection = root / "params" / SELECTION_FILE
        self.default = root / "params.defaults" / SELECTION_FILE
        self.status_file = root / "status.json"
        self.models = LateralNnModels(self.models_dir, self.selection, self.default, str(self.status_file))

    def tearDown(self):
        self.temporary.cleanup()

    def test_paths_and_name_rule_match_cpp(self):
        path_cc = cpp(ROOT / "src" / "common" / "lateral_nn_path.h")
        self.assertIn(f'kLateralNnModelDir = "{MODEL_DIR.as_posix()}";', path_cc)
        self.assertIn(f'kLateralNnSelectionFile = "{SELECTION_FILE}";', path_cc)
        select_cc = cpp(ROOT / "src" / "controls" / "lateral_nn_select.cc")
        self.assertIn(f'"EDGEPILOT_LATERAL_NN_STATUS", "{STATUS_PATH}"', select_cc)
        self.assertEqual(json.loads((ROOT / "params" / SELECTION_FILE).read_text())["model"], "k7_hev_v3")
        for name, ok in (("k7_hev_v3", True), ("Car-2.1", True), (".hidden", False), ("../x", False),
                         ("a" * 64, True), ("a" * 65, False)):
            self.selection.parent.mkdir(exist_ok=True)
            self.selection.write_text(json.dumps({"model": name}))
            self.assertEqual(read_choice(self.selection), (name, "") if ok else ("", "bad_name"), name)

    def test_lists_the_model_files_by_name(self):
        self.assertEqual(self.models.models(), ["k7_hev_v3", "k7_hev_v4"])
        for name in ("k7_hev_v10.json", "k7_hev_v9.json", ".hidden.json", "notes.txt"):
            (self.models_dir / name).write_text("{}")
        self.assertEqual(self.models.models(), ["k7_hev_v3", "k7_hev_v4", "k7_hev_v9", "k7_hev_v10"],
                         "숫자는 수로 정렬, 규칙 밖 이름은 뺀다")

    def test_select_writes_the_choice_and_refuses_unknown(self):
        self.assertEqual(self.models.status(controlsd_running=True)["selection_error"], "none")
        (self.models_dir / "k7_hev_v4.json").write_text("{}")  # 고르기는 파일 내용을 읽지 않는다
        self.models.select("k7_hev_v4")
        status = self.models.status(controlsd_running=True)
        self.assertEqual(json.loads(self.selection.read_text()), {"model": "k7_hev_v4"})
        self.assertEqual((status["selected"], status["selection_error"]), ("k7_hev_v4", ""))
        for name in ("k7_hev_v9", "../steering"):
            with self.assertRaises(Refusal) as refused:
                self.models.select(name)
            self.assertEqual(refused.exception.code, "error.lateral_nn_unknown")
        self.assertEqual(json.loads(self.selection.read_text()), {"model": "k7_hev_v4"}, "거부하면 그대로")

    def test_status_reports_default_and_controlsd(self):
        self.default.parent.mkdir()
        self.default.write_text(json.dumps({"model": "k7_hev_v3"}))
        self.assertIsNone(self.models.status(controlsd_running=True)["controlsd"])
        self.status_file.write_text(json.dumps({"selected": "k7_hev_v4", "loaded": "k7_hev_v3", "path": "",
                                                "error": "k7_hev_v4: bad", "inputs": 33, "unix_time": 1}))
        status = self.models.status(controlsd_running=True)
        self.assertEqual(status["default"], "k7_hev_v3")
        self.assertEqual(status["controlsd"]["loaded"], "k7_hev_v3")
        # controlsd는 읽을 때만 상태 파일을 쓴다: 멈춘 뒤에는 남은 파일을 쓰는 중으로 보이지 않는다
        self.assertIsNone(self.models.status(controlsd_running=False)["controlsd"])


class DrivingModelsTest(unittest.TestCase):
    """주행 모델 카드: models/*.axmodel 목록(기본 먼저, 규칙 밖 이름은 뺀다, 체크섬), 고르기 파일(이름과 체크섬), 결합·주행
    중 거부, 매니저 상태, manager.py와 같은 경로·규칙."""

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        root = Path(self.temporary.name)
        self.models_dir = root / "models"
        self.models_dir.mkdir()
        (self.models_dir / "supercombo.axmodel").write_bytes(b"shipped")
        (self.models_dir / "ea2a_gnb.axmodel").write_bytes(b"student")
        self.selection = root / "params" / DRIVING_SELECTION_FILE
        self.status_file = root / "status.json"
        self.control = root / "edgepilot_control_state"
        self.models = DrivingModels(self.models_dir, self.selection, str(self.status_file), str(self.control))

    def tearDown(self):
        self.temporary.cleanup()

    def test_paths_and_name_rule_match_the_manager(self):
        from scripts import manager
        self.assertEqual(DRIVING_MODEL_DIR.as_posix(), manager.MODEL_DIR)
        self.assertEqual(f"{manager.MODEL_DIR}/{DEFAULT_DRIVING_MODEL}.axmodel", manager.DEFAULT_MODEL_PATH)
        self.assertEqual(DRIVING_SELECTION_FILE, manager.MODEL_SELECTION_FILE)
        self.assertEqual(DRIVING_STATUS_PATH, manager.MODEL_STATUS_PATH)
        self.assertEqual(DRIVING_NAME_PATTERN.pattern, manager.MODEL_NAME.pattern)
        self.selection.parent.mkdir()
        self.assertEqual(read_driving_choice(self.selection), ("", "none"))
        self.assertEqual(manager.model_choice(str(self.selection)), ("", "none"))
        documents = ({"model": "ea2a_gnb"}, {"model": "models/ea2a_gnb.axmodel"}, {"model": "../x"},
                     {"model": "models/../x.axmodel"}, {"model": 3}, {}, [])
        for document in documents:
            self.selection.write_text(json.dumps(document))
            self.assertEqual(read_driving_choice(self.selection), manager.model_choice(str(self.selection)), document)
        self.selection.write_text(json.dumps({"model": "models/ea2a_gnb.axmodel"}))
        self.assertEqual(read_driving_choice(self.selection), ("ea2a_gnb", ""), "deploy_student.py의 경로 꼴")
        self.selection.write_text("{")
        self.assertEqual(read_driving_choice(self.selection), ("", "unreadable"))
        self.assertEqual(manager.model_choice(str(self.selection)), ("", "unreadable"))

    def test_lists_models_default_first_with_checksums(self):
        for name in ("aa.axmodel", ".hidden.axmodel", "notes.txt"):
            (self.models_dir / name).write_bytes(b"x")
        (self.models_dir / "dir.axmodel").mkdir()
        models = self.models.models()
        self.assertEqual([model["name"] for model in models], ["supercombo", "aa", "ea2a_gnb"])
        student = models[2]
        self.assertEqual((student["bytes"], student["sha256"]), (7, hashlib.sha256(b"student").hexdigest()))
        (self.models_dir / "ea2a_gnb.axmodel").write_bytes(b"student v2")  # 크기가 바뀌면 다시 계산한다
        self.assertEqual(self.models.models()[2]["sha256"], hashlib.sha256(b"student v2").hexdigest())

    def test_select_writes_name_and_checksum_and_refuses_unknown(self):
        status = self.models.status(manager_running=True, modeld_running=True)
        self.assertEqual((status["selected"], status["selection_error"], status["default"]), ("", "none", "supercombo"))
        self.models.select("ea2a_gnb")
        document = json.loads(self.selection.read_text())
        self.assertEqual((document["model"], document["axmodel_sha256"]),
                         ("ea2a_gnb", hashlib.sha256(b"student").hexdigest()))
        self.assertEqual(self.models.status(manager_running=True, modeld_running=True)["selected"], "ea2a_gnb")
        for name in ("ea2a_gq", "../steering", "supercombo.axmodel"):
            with self.assertRaises(Refusal) as refused:
                self.models.select(name)
            self.assertEqual((refused.exception.code, refused.exception.status), ("error.driving_model_unknown", 404))
        self.assertEqual(json.loads(self.selection.read_text())["model"], "ea2a_gnb", "거부하면 그대로")

    def test_refuses_while_engaged_or_moving(self):
        cases = ((control_payload(engaged=1), "engaged"), (control_payload(active=1), "engaged"),
                 (control_payload(vehicle_fresh=1, ego_speed_kph=30.0), "moving"),
                 (control_payload(vehicle_fresh=1, cluster_speed_kph=5.0), "moving"),
                 (control_payload(vehicle_fresh=1, ego_speed_kph=0.0), ""),
                 (control_payload(vehicle_fresh=0, ego_speed_kph=30.0), ""))  # CAN이 없으면(시동 꺼짐) 속도는 보지 않는다
        for payload, blocker in cases:
            publish(self.control, payload)
            self.assertEqual(self.models.blocker(), blocker)
            self.assertEqual(self.models.status(manager_running=True, modeld_running=True)["blocker"], blocker)
            if blocker:
                with self.assertRaises(Refusal) as refused:
                    self.models.select("ea2a_gnb")
                self.assertEqual(refused.exception.code, f"error.driving_model_{blocker}")
                self.assertFalse(self.selection.exists())
        publish(self.control, control_payload(engaged=1), stamp=boottime_ns() - int(5e9))
        self.assertEqual(self.models.blocker(), "", "controlsd가 멈췄으면 조향도 없다")
        self.models.select("ea2a_gnb")

    def test_status_reports_the_manager_only_while_it_runs(self):
        self.status_file.write_text(json.dumps({"model": "models/ea2a_gnb.axmodel", "source": "selection",
                                                "selected": "ea2a_gnb", "error": "", "unix_time": 1}))
        status = self.models.status(manager_running=True, modeld_running=False)
        self.assertEqual((status["manager"]["model"], status["modeld_running"]), ("models/ea2a_gnb.axmodel", False))
        self.assertIsNone(self.models.status(manager_running=False, modeld_running=False)["manager"])

    def test_server_and_page_use_the_card(self):
        server = (WEB_CONSOLE / "console_server.py").read_text(encoding="utf-8")
        self.assertEqual(server.count('"/api/driving_model"'), 2)
        self.assertEqual(server.count("return driving_model_status()"), 2)
        page = (STATIC_DIR / "console.js").read_text(encoding="utf-8")
        self.assertIn("steering: () => [createDrivingModelCard(), createLateralNnCard()]", page)
        card = (STATIC_DIR / "driving_model_card.js").read_text(encoding="utf-8")
        self.assertIn('api.post("/api/driving_model"', card)


class RecordingsTest(unittest.TestCase):
    """주행 기록 카드: SD의 route 목록(이름 규칙, 최근 것부터, 크기·길이, 쓰는 중), recordd 상태와 남은 공간, 지우기와
    거부(하나라도 거부하면 아무것도 지우지 않는다)."""

    OLD, CUT, BUSY, NEW = ("2026-10-12--22-37-07-989", "2026-10-16--23-07-05-554", "2026-10-16--23-13-33-641",
                           "2026-10-17--09-00-00-000")

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        base = Path(self.temporary.name)
        self.root, self.staging = base / "recordings", base / "staging"
        self.root.mkdir()
        self.staging.mkdir()
        self.state_path = base / "record_state"
        self.recordings = Recordings(self.root, self.staging, str(self.state_path))
        # 닫힌 것(60 s), 끝맺지 못한 것(전원이 끊김), SD와 스테이징에 다 있는 것(쓰는 중), 스테이징에만 있는 것(막 시작)
        self.route(self.root, self.OLD, complete=True, frames=1200, segments=2)
        self.route(self.root, self.CUT, complete=False, frames=0, segments=3)
        self.route(self.root, self.BUSY, complete=False, frames=0, segments=1)
        self.route(self.staging, self.BUSY, complete=False, frames=0, segments=2)
        self.route(self.staging, self.NEW, complete=False, frames=0, segments=0)
        # route가 아닌 것들: 규칙 밖 이름, 파일, 밖을 가리키는 링크
        (self.root / "notes").mkdir()
        (self.root / "2026-10-16").mkdir()
        (self.root / "2026-10-15--00-00-00-000.txt").write_text("")
        self.outside = base / "outside"
        self.route(base, "outside", complete=True, frames=20, segments=1)
        (self.root / "2026-10-14--00-00-00-000").symlink_to(self.outside, target_is_directory=True)

    def tearDown(self):
        self.temporary.cleanup()

    @staticmethod
    def route(where, name, *, complete, frames, segments, size=5000):
        path = where / name
        (path / "events").mkdir(parents=True)
        (path / "events" / "000.bin").write_bytes(b"x" * size)
        for index in range(segments):
            segment = path / "segments" / f"{index:03d}"
            segment.mkdir(parents=True)
            (segment / "road.h264").write_bytes(b"x" * size)
        (path / "manifest.json").write_text(json.dumps({"version": 13, "complete": complete, "fps": 20,
                                                        "video_frames": frames}))
        return path

    def test_constants_match_cpp(self):
        recordd = cpp(ROOT / "src" / "recording" / "recordd.cc")
        writer = cpp(ROOT / "src" / "recording" / "recording_writer.cc")
        self.assertIn(f'env_string("EDGEPILOT_RECORD_ROOT", "{RECORD_ROOT}")', recordd)
        self.assertIn(f'env_string("EDGEPILOT_RECORD_STAGING", "{STAGING_ROOT}")', writer)
        self.assertEqual(MIN_FREE_BYTES, 5 * 1024 ** 3)
        self.assertIn("kMinimumFreeBytes = 5ULL * 1024 * 1024 * 1024;", writer)
        self.assertIn(f"kMinimumFreePercent = {MIN_FREE_PERCENT};", writer)
        # route_name: put_time("%Y-%m-%d--%H-%M-%S") + '-' + 세 자리 밀리초
        self.assertIn('"%Y-%m-%d--%H-%M-%S") << \'-\' << std::setw(3)', writer)
        self.assertTrue(ROUTE_NAME.fullmatch(self.OLD))
        header = cpp()
        fields = re.search(r"struct RecordState \{(.*?)\};", header, re.S).group(1)
        self.assertEqual(re.findall(r"(uint\d+_t) (\w+)", fields),
                         [("uint64_t", "timestamp_ns"), ("uint32_t", "active"), ("uint32_t", "storage_blocked")])
        self.assertEqual(RECORD_STATE.size, 16)
        blocked = dict(re.findall(r"kStorageBlocked(\w+) = (\d+);", header))
        self.assertEqual({int(blocked["SdReserve"]): "sd_reserve", int(blocked["Staging"]): "staging"}, STORAGE_BLOCKED)
        # 카드는 녹화 탭의 recording 섹션 뒤에 붙는다
        self.assertIn("recording", PARAM_GROUPS["recording"]["sections"])
        self.assertIn('after: "recording"', (STATIC_DIR / "recordings_card.js").read_text(encoding="utf-8"))

    def test_lists_routes_newest_first(self):
        routes = self.recordings.routes()
        self.assertEqual([route["name"] for route in routes], [self.NEW, self.BUSY, self.CUT, self.OLD])
        by_name = {route["name"]: route for route in routes}
        self.assertEqual([route["busy"] for route in routes], [True, True, False, False])
        self.assertEqual(by_name[self.OLD]["duration_s"], 60.0)
        self.assertEqual((by_name[self.OLD]["complete"], by_name[self.CUT]["complete"]), (True, False))
        self.assertIsNone(by_name[self.CUT]["duration_s"], "끝맺지 못한 것은 길이를 모른다")
        self.assertIsNone(by_name[self.BUSY]["duration_s"])
        self.assertEqual([by_name[name]["segments"] for name in (self.OLD, self.CUT, self.BUSY, self.NEW)],
                         [2, 3, 2, 0])
        # 크기는 할당된 블록: 적어도 파일 크기만큼, 쓰는 중인 것은 SD와 스테이징을 더한다
        self.assertGreaterEqual(by_name[self.OLD]["bytes"], 3 * 5000)
        self.assertGreater(by_name[self.BUSY]["bytes"], by_name[self.NEW]["bytes"])
        status = self.recordings.status()
        self.assertEqual(status["total_bytes"], sum(route["bytes"] for route in status["routes"]))

    def test_reports_recordd_and_the_free_space(self):
        recording = self.recordings.recording()
        self.assertEqual((recording["available"], recording["active"], recording["storage_blocked"]),
                         (False, False, ""))
        publish(self.state_path, RECORD_STATE.pack(boottime_ns(), 1, 1))
        recording = self.recordings.recording()
        self.assertEqual(
            (recording["available"], recording["stale"], recording["active"], recording["storage_blocked"]),
            (True, False, True, "sd_reserve"))
        disk = self.recordings.status()["disk"]
        self.assertGreater(disk["total_bytes"], 0)
        self.assertLessEqual(disk["free_bytes"], disk["total_bytes"])
        self.assertEqual(disk["reserve_bytes"], max(MIN_FREE_BYTES, disk["total_bytes"] * MIN_FREE_PERCENT // 100))
        missing = Recordings(self.root / "missing", self.staging / "missing", str(self.state_path))
        self.assertEqual((missing.routes(), missing.disk()), ([], None))

    def test_delete_removes_the_chosen_routes(self):
        before = {route["name"]: route["bytes"] for route in self.recordings.routes()}
        result = self.recordings.delete([self.OLD, self.CUT, self.OLD])
        self.assertEqual(result["deleted"], [self.OLD, self.CUT], "같은 이름은 한 번")
        self.assertEqual(result["freed_bytes"], before[self.OLD] + before[self.CUT])
        self.assertFalse((self.root / self.OLD).exists() or (self.root / self.CUT).exists())
        self.assertEqual([route["name"] for route in result["routes"]], [self.NEW, self.BUSY])
        self.assertTrue((self.root / "notes").is_dir() and self.outside.is_dir(), "route가 아닌 것은 그대로")

    def test_delete_refuses_and_deletes_nothing(self):
        cases = (([], "error.recordings_none", 400), ([self.OLD, "../notes"], "error.recording_bad_name", 400),
                 ([self.OLD, "2026-10-16"], "error.recording_bad_name", 400),
                 ([self.OLD, self.BUSY], "error.recording_busy", 409),
                 ([self.OLD, self.NEW], "error.recording_busy", 409),
                 ([self.OLD, "2026-01-01--00-00-00-000"], "error.recording_unknown", 404),
                 ([self.OLD, "2026-10-14--00-00-00-000"], "error.recording_unknown", 404))
        for names, code, status in cases:
            with self.assertRaises(Refusal, msg=names) as refused:
                self.recordings.delete(names)
            self.assertEqual((refused.exception.code, refused.exception.status), (code, status), names)
        self.assertTrue((self.root / self.OLD).is_dir(), "하나라도 거부하면 아무것도 지우지 않는다")
        self.assertTrue((self.outside / "manifest.json").is_file(), "링크는 따라가지 않는다")


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
        # 플래싱 조건(panda_flash_allowed)과 지원 보드(panda_hw_type_flashable)도 같아야 한다
        firmware = cpp(ROOT / "src" / "panda" / "panda_firmware.cc")
        max_age_ns = int(re.search(r"kControlMaxAgeNs = ([\d']+)ULL;", firmware).group(1).replace("'", ""))
        self.assertEqual(max_age_ns, round(CONTROL_STATE_MAX_AGE_S * 1e9))
        self.assertEqual(float(re.search(r"kParkedMaxSpeedKph = ([\d.]+)f;", firmware).group(1)), PARKED_MAX_SPEED_KPH)
        flashable = re.search(r"return (hw_type == kPandaHw\w+(?: \|\| hw_type == kPandaHw\w+)*);", firmware).group(1)
        hw = {name: int(value) for name, value in re.findall(r"kPandaHw(\w+) = (\d+);", protocol)}
        self.assertEqual(sorted(hw[name] for name in re.findall(r"kPandaHw(\w+)", flashable)),
                         sorted(PANDA_FLASHABLE_HW))

    def test_image_version_prefers_debug_like_cpp(self):
        """panda_firmware.cc find_version처럼 -DEBUG를 먼저 찾는다(앞쪽에 -RELEASE 꼴이 있어도)."""
        image = bytearray(panda_image(version="EDGE-f9907afb-DEBUG"))
        image[20:20 + 23] = b"\0REL-12345678-RELEASE\0"  # 버전(64바이트째)보다 앞
        self.image.write_bytes(bytes(image))
        self.assertEqual(panda_image_info(str(self.image))["version"], "EDGE-f9907afb-DEBUG")

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
        with self.assertRaises(Refusal) as refused:
            self.panda.request_flash("EDGE-f9907afb-DEBUG")
        self.assertEqual(refused.exception.detail(), {"code": "panda.reason.not_park"})
        self.assertFalse(self.request.exists())
        for blocker, kwargs in (("moving", {"speed": 3.0}), ("engaged", {"engaged": 1}),
                                ("vehicle_stale", {"vehicle_fresh": 0}),
                                ("control_stale", {"stamp": boottime_ns() - 5_000_000_000})):
            self.publish_control(**kwargs)
            self.assertEqual(self.panda.status()["blocker"], blocker)

        self.publish_control()
        with self.assertRaises(Refusal):
            self.panda.request_flash("EDGE-00000000-DEBUG")  # 페이지가 본 이미지가 아니다
        status = self.panda.request_flash("EDGE-f9907afb-DEBUG")
        self.assertTrue(status["request_pending"])
        self.assertEqual(self.request.read_text(), "EDGE-f9907afb-DEBUG\n")
        with self.assertRaises(Refusal):
            self.panda.request_flash("EDGE-f9907afb-DEBUG")  # 이미 요청했다

    def test_flash_needs_a_running_pandad_and_a_panda(self):
        self.image.write_bytes(panda_image())
        self.publish_control()
        with self.assertRaises(Refusal) as refused:
            self.panda.request_flash("EDGE-f9907afb-DEBUG")
        self.assertEqual(refused.exception.code, "error.pandad_missing")
        self.publish_pandad(stamp=boottime_ns() - 10_000_000_000)
        stale = self.panda.status()
        self.assertEqual((stale["available"], stale["stale"], stale["mode"]), (True, True, "none"))
        self.publish_pandad(mode="none", version="")
        with self.assertRaises(Refusal):
            self.panda.request_flash("EDGE-f9907afb-DEBUG")
        self.publish_pandad(hw_type=7)
        self.assertFalse(self.panda.status()["hw_supported"])
        with self.assertRaises(Refusal) as refused:
            self.panda.request_flash("EDGE-f9907afb-DEBUG")
        self.assertEqual(refused.exception.code, "panda.reason.hw_unsupported")
        # 앱이 서지 않은 bootstub 판다는 CAN을 넘기지 않아 차 상태가 낡지만, 차 상태 없이 다시 쓸 수 있다
        self.publish_control(vehicle_fresh=0, gear=5, speed=40.0)
        self.publish_pandad(mode="bootstub", version="", hw_type=0)
        self.assertEqual(self.panda.status()["blocker"], "")
        self.publish_control(vehicle_fresh=0, engaged=1)  # 살아 있는 제어 상태가 결합 중이면 막는다
        self.assertEqual(self.panda.status()["blocker"], "engaged")
        self.publish_control(vehicle_fresh=0, stamp=boottime_ns() - 5_000_000_000)
        self.assertEqual(self.panda.status()["blocker"], "")
        self.assertTrue(self.panda.request_flash("EDGE-f9907afb-DEBUG")["request_pending"])

    def test_status_reports_flash_progress(self):
        self.publish_pandad(version="EDGE-f9907afb-DEBUG",
                            flash={"state": "failed", "step": "write", "percent": 40, "error": "app_timeout",
                                   "detail": "x", "version": "EDGE-f9907afb-DEBUG", "stamp_ns": 1})
        self.image.write_bytes(panda_image())
        status = self.panda.status()
        self.assertTrue(status["up_to_date"])
        self.assertEqual((status["flash"]["error"], status["flash"]["step"]), ("app_timeout", "write"))
        self.assertTrue(status["hw_supported"])
        # 서버는 코드만 낸다: 글자는 페이지의 panda.reason.<코드>, panda.step.<단계> 문구
        self.assertFalse([key for key in status if key.endswith("_text")])


# ------------------------------------------------------------ 런타임 상태
class ProcessStatusTest(unittest.TestCase):
    def test_layout_matches_cpp(self):
        """웹 콘솔이 읽는 ManagerState(manager.py가 쓰고 overlayd도 읽는다) 배치가 ipc_messages.h와 같아야 한다. manager.py
        쪽은 check_manager.py가 본다."""
        self.assertEqual((IPC_MAGIC, IPC_VERSION, IPC_HEADER.size),
                         (cpp_constant("kIpcMagic"), cpp_constant("kIpcVersion"), cpp_size("IpcHeader")))
        self.assertEqual((PROCESS_STATE.size, MAX_PROCESSES), (cpp_size("ProcessState"), cpp_constant("kMaxProcesses")))
        self.assertLessEqual(MANAGER_STATE_HEAD.size + PROCESS_STATE.size * MAX_PROCESSES, cpp_size("ManagerState"))

    @staticmethod
    def publish_processes(path, entries, stamp=None):
        stamp = boottime_ns() if stamp is None else stamp
        payload = bytearray(MANAGER_STATE_HEAD.size + PROCESS_STATE.size * MAX_PROCESSES)
        MANAGER_STATE_HEAD.pack_into(payload, 0, stamp, len(entries), 0)
        for index, (name, running) in enumerate(entries):
            at = MANAGER_STATE_HEAD.size + index * PROCESS_STATE.size
            PROCESS_STATE.pack_into(payload, at, name.encode(), running)
        publish(path, bytes(payload), stamp=stamp)

    def test_status_lists_processes_and_goes_stale(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "edgepilot_manager_state"
            processes = ProcessStatus(str(path), install_dir=Path(directory), proc=Path(directory) / "proc")
            self.assertEqual(processes.status(), {"available": False, "stale": True, "age_s": None, "processes": []})

            self.publish_processes(path, [("controlsd", 1), ("web_console", 0)])
            status = processes.status()
            self.assertEqual((status["available"], status["stale"]), (True, False))
            self.assertEqual(status["processes"], [
                {"name": "controlsd", "running": True, "version": None, "installed_version": None},
                {"name": "web_console", "running": False, "version": None, "installed_version": None}])
            self.publish_processes(path, [("controlsd", 1), ("web_console", 0)], stamp=boottime_ns() - 10_000_000_000)
            self.assertTrue(processes.status()["stale"])

    def test_versions_come_from_the_running_and_installed_binaries(self):
        """버전은 실행 중인 실행 파일(/proc/<pid>/exe)과 설치된 실행 파일에 새긴 표식에서 읽는다. 리허설에서는 replayd가
        camerad 칸에서 돈다."""
        def binary(path, version, pad=5000):
            path.write_bytes(b"\x7fELF" + b"\0" * pad + VERSION_MARKER + version.encode() + b"\0more")

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            proc, install = root / "proc", root / "install"
            install.mkdir()
            for pid, argv0, version in ((101, "./controlsd", "v0.0.4-6-gaaaaaaa"),
                                        (102, "./replayd", "v0.0.4-6-gaaaaaaa"),
                                        (103, "/usr/bin/python3", "")):
                (proc / str(pid)).mkdir(parents=True)
                (proc / str(pid) / "cmdline").write_bytes(argv0.encode() + b"\0-m\0")
                if version:
                    binary(proc / str(pid) / "exe", version)
            binary(install / "controlsd", "v0.0.4-7-gbbbbbbb-dirty")  # 배포했지만 아직 다시 뜨지 않았다
            binary(install / "replayd", "v0.0.4-6-gaaaaaaa")
            binary(install / "modeld", "v0.0.4-6-gaaaaaaa")
            path = root / "edgepilot_manager_state"
            processes = ProcessStatus(str(path), install_dir=install, proc=proc)
            self.publish_processes(path, [("camerad", 1), ("controlsd", 1), ("modeld", 0), ("web_console", 1)])
            # 버전은 머리 줄의 목록을 펼쳤을 때만 읽는다(/proc 전체를 훑는다)
            self.assertEqual({p["version"] for p in processes.status()["processes"]}, {None})
            by_name = {p["name"]: (p["version"], p["installed_version"])
                       for p in processes.status(versions=True)["processes"]}
            self.assertEqual(by_name, {"camerad": ("v0.0.4-6-gaaaaaaa", "v0.0.4-6-gaaaaaaa"),
                                       "controlsd": ("v0.0.4-6-gaaaaaaa", "v0.0.4-7-gbbbbbbb-dirty"),
                                       "modeld": (None, "v0.0.4-6-gaaaaaaa"),
                                       "web_console": (None, None)})
            # 파라미터를 저장할 때 읽는 프로세스가 도는지도 argv[0]의 파일 이름으로 본다
            self.assertEqual((process_running("controlsd", proc), process_running("modeld", proc)), (True, False))

    def test_binary_version_marker(self):
        header = cpp(ROOT / "src" / "common" / "build_version.h")
        self.assertIn(f'#define EDGEPILOT_VERSION_MARKER_PREFIX "{VERSION_MARKER.decode()}"', header)
        template = cpp(ROOT / "cmake" / "build_version.cc.in")
        self.assertIn('EDGEPILOT_VERSION_MARKER_PREFIX "@version@"', template)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "binary"
            for content, expected in ((VERSION_MARKER + b"unknown\0", "unknown"),
                                      (b"x" * 100 + VERSION_MARKER + b"\0", None),          # 빈 버전
                                      (VERSION_MARKER + b"v0.0.4", None),                    # 끝나지 않았다
                                      (VERSION_MARKER + b"v" * 65 + b"\0", None),           # 64자보다 길다
                                      (b"no marker here\0", None),
                                      # 같은 글자로 시작하는 다른 문자열(테스트의 식 문자열 같은)은 건너뛴다
                                      (VERSION_MARKER + b'" + version\0' + VERSION_MARKER + b"v1\0", "v1")):
                path.write_bytes(content)
                self.assertEqual(read_binary_version(path), expected, content[:40])
            self.assertIsNone(read_binary_version(Path(directory) / "missing"))

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
        flags = cpp_flags("kHudFlag")
        for name, bit in HUD_FLAG_BITS.items():
            self.assertEqual(flags.get(name), bit, name)
        topics = {"ModelState": MODEL_STATE_TOPIC, "ControlState": CONTROL_STATE_TOPIC,
                  "LearnerState": LEARNER_STATE_TOPIC, "LocalizationState": LOCALIZATION_STATE_TOPIC,
                  "RecordState": RECORD_STATE_TOPIC, "ManagerState": MANAGER_STATE_TOPIC}
        for name, topic in topics.items():
            self.assertEqual(cpp_string(f"k{name}Topic"), topic, name)
        layout = bev_layout()
        self.assertEqual((layout["model"]["size"], layout["control"]["size"]), (MODEL_STATE_SIZE, CONTROL_STATE_SIZE))

    def test_lead_constants_match_cpp(self):
        """페이지의 앞차 확률 문턱과 레이더-카메라 거리는 비전 크루즈·출발 알림·HUD와 같은 model_output.h
        값이어야 한다."""
        source = cpp(ROOT / "src" / "common" / "model_output.h")
        text = (STATIC_DIR / "bev_data.js").read_text(encoding="utf-8")
        for cpp_name, page_name in (("kLeadProbabilityThreshold", "LEAD_PROBABILITY"),
                                    ("kRadarToCameraDistanceM", "RADAR_TO_CAMERA")):
            value = float(re.search(rf"constexpr float {cpp_name} = ([\d.]+)f;", source).group(1))
            page = float(re.search(rf"const {page_name} = ([\d.]+);", text).group(1))
            self.assertEqual(page, value, page_name)

    def test_bev_constants_match_cpp(self):
        """BEV가 HUD·출발 알림처럼 그리고 판단하도록 옮겨 온 값들(hud_scene.cc, departure_alert.cc, can_frame.h)이
        C++과 같아야 한다."""
        bev = (STATIC_DIR / "bev.js").read_text(encoding="utf-8")
        data = (STATIC_DIR / "bev_data.js").read_text(encoding="utf-8")
        scene = cpp(ROOT / "src" / "hud" / "hud_scene.cc")
        alert = cpp(ROOT / "src" / "controls" / "departure_alert.cc")

        def cpp_value(text, name):
            return float(re.search(rf"constexpr (?:float|double|int) {name} = (-?[\d.]+)f?;", text).group(1))

        def js_object(text, name):
            body = re.search(rf"const {name} = \{{([^}}]*)\}};", text).group(1)
            return {key: float(value) for key, value in re.findall(r"(\w+): (-?[\d.]+)", body)}

        def js_value(text, name):
            return float(re.search(rf"\b{name} = (-?[\d.]+)", text).group(1))

        self.assertEqual(js_value(bev, "PATH_WIDTH"), 2 * cpp_value(scene, "kPathHalfWidth"))
        self.assertEqual(js_value(bev, "LANE_MIN_PROBABILITY"), cpp_value(scene, "kLaneMinProbability"))
        self.assertEqual(js_value(bev, "EDGE_MIN_CONFIDENCE"), cpp_value(scene, "kEdgeMinConfidence"))
        risk = js_object(bev, "RISK")
        self.assertEqual((risk["distance"], risk["closing"]),
                         (cpp_value(scene, "kLeadRiskDistanceM"), cpp_value(scene, "kLeadRiskClosingMps")))
        depart = js_object(bev, "DEPART")
        self.assertEqual(depart["stopped"], cpp_value(alert, "kStoppedSpeedMps"))
        self.assertEqual(depart["moving"], cpp_value(alert, "kMovingResetSpeedMps"))
        self.assertEqual(depart["gas"], cpp_value(alert, "kGasPressGoProbability"))
        self.assertEqual(depart["open"], cpp_value(alert, "kOpenPlanDistanceM"))
        self.assertEqual(depart["arm"], cpp_value(alert, "kStoppedPlanDistanceM"))
        self.assertEqual(js_value(data, "LANE_WIDTH_INDEX"), cpp_value(scene, "kLaneRulerIndex"))
        draw = js_object(data, "DRAW")
        self.assertEqual((draw["min"], draw["max"]),
                         (cpp_value(scene, "kMinDrawDistance"), cpp_value(scene, "kMaxDrawDistance")))
        self.assertEqual(js_value(data, "DRIVE_GEAR"),
                         cpp_value(cpp(ROOT / "src" / "car" / "can_frame.h"), "kGearDrive"))
        # 곡률 호를 그리는 최저 속도(ARC.speed m/s)는 BEV 안내 문구에 km/h로 적혀 있다(문구는 bev.js보다 먼저 뜬다)
        arc_kph = round(js_object(bev, "ARC")["speed"] * 3.6)
        for lang in LANGUAGES:
            self.assertEqual([int(n) for n in re.findall(r"(\d+) km", catalog(lang)["bev.note"])], [arc_kph], lang)

    def test_vehicle_tab_bands_and_bev_frame_match(self):
        """차량 특성 탭이 보이는 허용 폭(torqued 배율 ±30%, 마찰 ±50%, paramsd 조향비 0.5~2배)과 BEV 프레임 매직이
        학습기·서버와 같아야 한다."""
        page = (STATIC_DIR / "vehicle_view.js").read_text(encoding="utf-8")
        torque = cpp(ROOT / "src" / "learners" / "torque_estimator.cc")
        learner = cpp(ROOT / "src" / "learners" / "vehicle_params_learner.cc")
        factor = float(re.search(r"kFactorSanity = ([\d.]+);", torque).group(1))
        friction = float(re.search(r"kFrictionSanity = ([\d.]+);", torque).group(1))
        self.assertIn(f"s.lat_accel_factor_raw, s.prior_lat_accel_factor, {factor}, 2)", page)
        self.assertIn(f"s.friction_raw, s.prior_friction, {friction}, 3)", page)
        low = float(re.search(r"min_sr_\(([\d.]+) \* c\.steer_ratio\)", learner).group(1))
        high = float(re.search(r"max_sr_\(([\d.]+) \* c\.steer_ratio\)", learner).group(1))
        self.assertIn(f"s.prior_steer_ratio * {low:g}, 1)", page)
        self.assertIn(f"s.prior_steer_ratio * {high:g}, 1)", page)
        magic_text = re.search(r"const MAGIC = (0x[0-9a-fA-F]+);", (STATIC_DIR / "bev_data.js").read_text()).group(1)
        magic = int(magic_text, 16)
        self.assertEqual(magic, BEV_FRAME_MAGIC)

    def test_page_gives_up_on_a_silent_board(self):
        """보드가 말없이 사라지면 페이지가 기다리지 않는다: BEV 스트림은 서버의 빈 프레임 주기(BEV_IDLE_S)보다 넉넉히
        긴 침묵에서 다시 연결하고, GET에는 시한이 있다(POST·PATCH에는 없다: 늦은 성공을 실패로 알리지 않게)."""
        data = (STATIC_DIR / "bev_data.js").read_text(encoding="utf-8")
        silence_ms = int(re.search(r"const STREAM_SILENCE_MS = (\d+);", data).group(1))
        self.assertGreaterEqual(silence_ms, 2 * BEV_IDLE_S * 1000)
        self.assertIn("await silenceLimited(reader.read())", data)
        ui = (STATIC_DIR / "ui.js").read_text(encoding="utf-8")
        self.assertRegex(ui, r"const GET_TIMEOUT_MS = \d+;")
        self.assertIn('get: (url, {signal} = {}) => request("GET", url, undefined, signal, GET_TIMEOUT_MS)', ui)
        self.assertRegex(ui, r'post: \(url, body = \{\}\) => request\("POST", url, body\),')
        self.assertRegex(ui, r'patch: \(url, body\) => request\("PATCH", url, body\),')

    def test_bev_reads_lead_structs_where_cpp_pins_them(self):
        """페이지는 ModelState 안의 앞차 구조체를 서버가 알려 준 시작 위치 + 고정 오프셋으로 읽는다. 그 오프셋이
        ipc_messages.h가 offsetof로 못 박은 자리와 같아야 한다."""
        pinned = {(name, field): int(at) for name, field, at in
                  re.findall(r"offsetof\((Lead\w*State), (\w+)\) == (\d+)", cpp())}
        page = (STATIC_DIR / "bev_data.js").read_text(encoding="utf-8")
        reads = {
            ("LeadState", "probability"): r"probability: f32\(lead \+ (\d+)\)",
            ("LeadState", "x"): r"\bx: f32\(lead \+ (\d+)\)",
            ("LeadState", "y"): r"\by: f32\(lead \+ (\d+)\)",
            ("LeadState", "velocity"): r"speed: f32\(lead \+ (\d+)\)",
            ("LeadState", "acceleration"): r"accel: f32\(lead \+ (\d+)\)",
            ("LeadFlowState", "valid"): r"u32\(flow \+ (\d+)\)",
            ("LeadFlowState", "expansion_rate"): r"rate: f32\(flow \+ (\d+)\)",
            ("LeadFlowState", "lead_x"): r"\bx: f32\(flow \+ (\d+)\)",
            ("LeadTrackState", "distance_m"): r"\bx: f32\(track \+ (\d+)\)",
            ("LeadTrackState", "relative_speed_mps"): r"relative: f32\(track \+ (\d+)\)",
        }
        for key, pattern in reads.items():
            found = re.search(pattern, page)
            self.assertIsNotNone(found, key)
            self.assertEqual(int(found.group(1)), pinned[key], key)
        # 0번 자리의 valid는 더하기 없이 읽는다
        self.assertIn("lead: u32(lead)", page)
        self.assertIn("track: u32(track)", page)
        self.assertEqual((pinned[("LeadState", "valid")], pinned[("LeadTrackState", "valid")]), (0, 0))

    def test_lane_filter_constants_match_cpp(self):
        """BEV의 차선 필터 사본(lane_filter.js)의 가중·끊김·다시 시작 기준은 플래너의 lane_line_filter와 같아야 한다."""
        header = cpp(ROOT / "src" / "planning" / "lane_line_filter.h")
        source = cpp(ROOT / "src" / "planning" / "lane_line_filter.cc")
        page = (STATIC_DIR / "lane_filter.js").read_text(encoding="utf-8")
        for cpp_text, cpp_name, page_name in ((header, "kLaneLineFilterWeight", "LANE_FILTER_WEIGHT"),
                                              (source, "kMaxGapS", "MAX_GAP_S"),
                                              (source, "kRestartReachM", "RESTART_REACH_M"),
                                              (source, "kRestartM", "RESTART_M")):
            value = float(re.search(rf"constexpr double {cpp_name} = ([\d.]+);", cpp_text).group(1))
            ours = float(re.search(rf"const {page_name} = ([\d.]+);", page).group(1))
            self.assertEqual(ours, value, page_name)

    def test_lane_filter_golden_is_shared(self):
        """C++ 필터가 내는 기준 출력(gtest_lane_line_filter)과 JS 사본이 맞추는 값(check_bev_lane_filter.mjs)이
        같아야 한다."""
        def golden(path):
            text = path.read_text(encoding="utf-8")
            block = re.search(r"GOLDEN BEGIN(.*?)GOLDEN END", text, re.S).group(1)
            return [float(v) for v in re.findall(r"-?\d+\.\d+", block)]
        cpp_golden = golden(ROOT / "tests" / "gtest_lane_line_filter.cc")
        self.assertEqual(len(cpp_golden), 36)
        self.assertEqual(golden(ROOT / "tests" / "check_bev_lane_filter.mjs"), cpp_golden)

    def test_lane_filter_copy_matches_cpp(self):
        """JS 사본을 jsc(macOS)나 node로 돌려 C++ 기준 출력과 맞춘다. 둘 다 없으면 건너뛴다."""
        jsc = Path("/System/Library/Frameworks/JavaScriptCore.framework/Versions/Current/Helpers/jsc")
        script = ROOT / "tests" / "check_bev_lane_filter.mjs"
        if jsc.is_file():
            command = [str(jsc), "-m", str(script)]
        elif shutil.which("node"):
            command = ["node", str(script)]
        else:
            self.skipTest("no JavaScript engine (jsc or node)")
        run = subprocess.run(command, capture_output=True, text=True, timeout=60)
        self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
        self.assertIn("all passed", run.stdout)

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
        # 읽는 함수가 쓰는 필드와 HUD 비트는 모두 NEEDS에 있어야 한다: 빠진 필드는 서버가 몰라도 검사를 지나 위치
        # undefined(NaN)에서 읽었다
        for function, part in (("readModel", "model"), ("readControl", "control")):
            body = re.search(r"function " + function + r"\(at, [^)]*\) \{(.*?)\n\}", text, re.S).group(1)
            self.assertLessEqual(set(re.findall(r"\bat\.(\w+)", body)), parts[part], function)
        self.assertEqual(set(re.findall(r'\bflag\("(\w+)"\)', text)), parts["hud_flags"])

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
    def test_asset_cache_is_keyed_by_the_resolved_file(self):
        """같은 파일을 다른 글자로 요청해도 캐시는 하나다(요청마다 사본이 쌓이지 않는다)."""
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "static"
            (root / "three").mkdir(parents=True)
            (root / "three" / "a.js").write_text("export const a = 1;\n", encoding="utf-8")
            assets = StaticAssets(root)
            for name in ("three/a.js", "./three/a.js", "three//a.js", "three/./a.js"):
                self.assertIsNotNone(assets.get(name), name)
            self.assertIsNone(assets.get("three/../three/a.js"))  # ..은 숨김 경로처럼 거부한다
            self.assertIsNone(assets.get("a\x00.js"))  # 경로에 NUL: 500이 아니라 없는 파일
            self.assertEqual(list(assets.cache), ["three/a.js"])

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
        assets = StaticAssets()
        for url in [*imports.values(), *re.findall(r'(?:src|href)="(/static/[^"]+)"', page)]:
            self.assertTrue((STATIC_DIR / Path(url).relative_to("/static")).is_file(), url)
            self.assertIsNotNone(assets.get(str(Path(url).relative_to("/static"))), f"{url}: the server does not serve it")
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
