#!/usr/bin/env python3
"""scripts/manager.py 검사. stdlib만 쓴다: 매니저가 쓰는 ManagerState 배치가 src/common/ipc_messages.h와 같은지,
다시 띄우기 규칙(빨리 죽기를 거듭하면 늦추기, 스스로 멈춘 것은 세지 않기), 끝낼 때 recordd를 더 기다리기, 로그 자르기,
pandad에 주는 safety 기본값이 controlsd가 기다리는 값과 같은지. ctest가 저장소 루트에서 돌린다."""
import hashlib
import json
import os
import re
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from cpp_headers import cpp_constant, cpp_size  # noqa: E402
from scripts import manager  # noqa: E402

IPC_MESSAGES = ROOT / "src" / "common" / "ipc_messages.h"


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


class ManagerTest(unittest.TestCase):
    def test_layout_matches_cpp(self):
        """매니저가 쓰는 ManagerState(overlayd와 웹 콘솔이 읽는다)가 ipc_messages.h와 같아야 한다."""
        self.assertEqual(manager.IPC_MAGIC, cpp_constant("kIpcMagic"))
        self.assertEqual(manager.IPC_VERSION, cpp_constant("kIpcVersion"))
        self.assertEqual(manager.MAX_PROCESSES, cpp_constant("kMaxProcesses"))
        self.assertEqual(manager.HEADER_SIZE, cpp_size("IpcHeader"))
        self.assertEqual(manager.PROCESS.size, cpp_size("ProcessState"))
        self.assertEqual(manager.MANAGER_STATE_SIZE, cpp_size("ManagerState"))
        topic = re.search(r'kManagerStateTopic\[\] = "([^"]+)";', read(IPC_MESSAGES)).group(1)
        self.assertIn(f'LatestPublisher("{topic}"', read(ROOT / "scripts" / "manager.py"))

    def test_backs_off_a_crash_loop(self):
        """빨리 죽기를 거듭하는 프로세스는 세 번까지 1초, 그 뒤 2·4·8·16초에 다시 띄운다. camerad는 늘 1초."""
        self.assertEqual([manager.restart_delay("web_console", n) for n in range(10)],
                         [1.0, 1.0, 1.0, 1.0, 2.0, 4.0, 8.0, 16.0, 16.0, 16.0])
        self.assertEqual({manager.restart_delay("camerad", n) for n in range(10)}, {1.0})

    def test_does_not_count_its_own_stops(self):
        """매니저가 멈춘 프로세스(camerad를 따라 내린 overlayd 등)의 종료는 빠른 종료로 세지 않는다."""
        spec = manager.ProcSpec("overlayd", ["/bin/true"])
        state = manager.ProcState(spec, started_monotonic=100.0, quick_exits=3)
        state.stop_requested = True
        self.assertEqual(manager.note_exit(state, 102.0), 1.0)
        self.assertEqual(state.quick_exits, 3)
        self.assertFalse(state.stop_requested)
        self.assertEqual(manager.note_exit(state, 102.0), 2.0)  # 스스로 죽으면 센다
        self.assertEqual(state.quick_exits, 4)
        state.started_monotonic = 0.0
        self.assertEqual(manager.note_exit(state, 102.0), 1.0)  # 10초 넘게 살았으면 처음부터
        self.assertEqual(state.quick_exits, 0)

    def test_waits_longer_for_recordd_on_shutdown(self):
        """끝낼 때는 recordd만 더 기다린다: SIGTERM 뒤 스테이징에 남은 녹화를 SD로 옮기고 스스로 끝난다. 다른 프로세스는
        평소 시한에 SIGKILL한다."""
        m = object.__new__(manager.Manager)
        script = "trap 'sleep 0.6; exit 0' TERM; while :; do sleep 0.05; done"
        m.procs = {}
        for name in ("recordd", "overlayd"):
            state = manager.ProcState(manager.ProcSpec(name, ["/bin/sh", "-c", script]))
            # 셸이 SIGTERM으로 끝난 sleep을 "Terminated: 15"로 알리므로 출력은 버린다
            state.proc = subprocess.Popen(state.spec.cmd, start_new_session=True, stderr=subprocess.DEVNULL)
            m.procs[name] = state
        time.sleep(0.3)  # trap이 걸리기를 기다린다
        m.stop_all(timeout_s=0.3, recordd_timeout_s=3.0)
        self.assertEqual(m.procs["recordd"].proc.wait(timeout=5), 0, "recordd는 스스로 끝날 때까지 기다렸다")
        self.assertEqual(m.procs["overlayd"].proc.wait(timeout=5), -signal.SIGKILL, "다른 프로세스는 시한에 내렸다")
        self.assertLess(manager.RECORDD_STOP_TIMEOUT_S, 10.0, "edgepilot.service TimeoutStopSec 안")

    def test_sets_the_safety_mode_controlsd_expects(self):
        """매니저가 pandad에 주는 safety 모드·인자 기본값이 controlsd가 기다리는 값(control_holds.h)과 같다. 다르면
        Panda가 준비돼도 결합하지 못한다."""
        source = read(ROOT / "scripts" / "manager.py")
        mode = re.search(r'setdefault\("EDGEPILOT_PANDA_SAFETY", "(\w+)"\)', source).group(1)
        param = re.search(r'setdefault\("EDGEPILOT_PANDA_SAFETY_PARAM", "(\d+)"\)', source).group(1)
        pandad = read(ROOT / "src" / "panda" / "pandad.cc")
        constant = re.search(r'if \(mode == "' + mode + r'"\) return (\w+);', pandad).group(1)
        holds = read(ROOT / "src" / "controls" / "control_holds.h")
        self.assertIn(f"kExpectedPandaSafetyModel = {constant};", holds)
        self.assertIn(f"kExpectedPandaSafetyParam = {param};", holds)

    def test_trims_a_log_even_when_the_copy_fails(self):
        """.log.1로 남기지 못해도(공간 부족 등) 로그는 비운다. tmpfs가 차는 것이 더 나쁘다."""
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "camerad.log"
            log.write_bytes(b"x" * (manager.LOG_MAX_BYTES + 10))
            (Path(directory) / "camerad.log.1").mkdir()  # 사본 자리에 디렉터리가 있어 복사가 실패한다
            m = object.__new__(manager.Manager)
            m.procs = {"camerad": None}
            m.log_dir = directory
            m.trim_logs()
            self.assertEqual(log.stat().st_size, 0)

    def test_starts_the_package(self):
        source = read(ROOT / "scripts" / "manager.py")
        self.assertIn('ProcSpec("web_console", [sys.executable, "-m", "web_console"]', source)
        self.assertIn('installed="web_console/__main__.py"', source)
        self.assertTrue((ROOT / "scripts" / "web_console" / "__main__.py").is_file())
        self.assertTrue(hasattr(manager, "ProcSpec"))


class DrivingModelTest(unittest.TestCase):
    """주행 모델: 인자 > EDGEPILOT_MODEL > 웹 콘솔의 고르기(params/driving_model.json) > 기본. 고르기가 바뀌면 modeld만
    새 모델로 다시 띄우고, 고른 모델로 modeld가 거듭 바로 죽으면 기본 모델로 돌아간다(고르기를 다시 쓰면 다시 해 본다)."""

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        (self.root / "models").mkdir()
        (self.root / "params").mkdir()
        for name in ("supercombo", "ea2a_gnb"):
            (self.root / "models" / f"{name}.axmodel").write_bytes(b"x")
        self.selection = self.root / "params" / manager.MODEL_SELECTION_FILE
        self.cwd = os.getcwd()
        os.chdir(self.root)  # 매니저처럼 설치 디렉터리 기준
        self.env = mock.patch.dict(os.environ, {"EDGEPILOT_MODEL": ""})
        self.env.start()

    def tearDown(self):
        self.env.stop()
        os.chdir(self.cwd)
        self.temporary.cleanup()

    def choose(self, model: str):
        self.selection.write_text(json.dumps({"model": model}))
        stat = self.selection.stat()  # 같은 나노초에 두 번 쓰면 바뀐 것을 못 본다
        os.utime(self.selection, ns=(stat.st_atime_ns, stat.st_mtime_ns + 1_000_000))

    def resolve(self, argument=None, failed=""):
        choice = manager.resolve_model(argument, str(self.selection), failed)
        return choice.path, choice.source, choice.selected, choice.error

    def test_resolves_in_order(self):
        default = manager.DEFAULT_MODEL_PATH
        self.assertEqual(self.resolve(), (default, "default", "", ""))
        self.choose("ea2a_gnb")
        self.assertEqual(self.resolve(), ("models/ea2a_gnb.axmodel", "selection", "ea2a_gnb", ""))
        self.choose("models/ea2a_gnb.axmodel")  # deploy_student.py가 쓰는 꼴
        self.assertEqual(self.resolve(), ("models/ea2a_gnb.axmodel", "selection", "ea2a_gnb", ""))
        self.assertEqual(self.resolve(failed="models/ea2a_gnb.axmodel"), (default, "default", "ea2a_gnb", "failed"))
        self.assertEqual(self.resolve("/tmp/x.axmodel"), ("/tmp/x.axmodel", "argument", "ea2a_gnb", ""))
        os.environ["EDGEPILOT_MODEL"] = "/root/test.axmodel"
        self.assertEqual(self.resolve(), ("/root/test.axmodel", "env", "ea2a_gnb", ""))
        os.environ["EDGEPILOT_MODEL"] = ""
        self.choose("ea2a_gq")
        self.assertEqual(self.resolve(), (default, "default", "ea2a_gq", "missing"))
        self.choose("../../etc/x")
        self.assertEqual(self.resolve(), (default, "default", "", "bad_name"))
        self.selection.write_text("[]")
        self.assertEqual(self.resolve(), (default, "default", "", "unreadable"))

    def manager_with_modeld(self) -> manager.Manager:
        m = object.__new__(manager.Manager)
        m.argument_model = None
        m.selection_path = str(self.selection)
        m.model_status_path = str(self.root / "driving_model_status.json")
        m.model_record_path = str(self.root / "params" / manager.MODEL_RECORD_FILE)
        m.model_digests = {}
        m.selection_stamp = manager.file_stamp(m.selection_path)
        m.failed_model = ""
        m.model_choice = manager.resolve_model(None, m.selection_path)
        m.model = m.model_choice.path
        state = manager.ProcState(manager.ProcSpec("modeld", ["./modeld", m.model]))
        state.proc = subprocess.Popen(["/bin/sh", "-c", "sleep 30"], start_new_session=True)
        m.procs = {"modeld": state}
        return m

    def status(self, m) -> dict:
        return json.loads(Path(m.model_status_path).read_text())

    def test_restarts_only_modeld_on_a_new_choice_and_falls_back(self):
        m = self.manager_with_modeld()
        modeld = m.procs["modeld"]
        m.check_model_choice()
        self.assertTrue(modeld.running(), "고르기가 그대로면 아무것도 하지 않는다")
        self.choose("ea2a_gnb")
        m.check_model_choice()
        self.assertEqual(modeld.spec.cmd, ["./modeld", "models/ea2a_gnb.axmodel"])
        self.assertEqual(modeld.proc.wait(timeout=5), -signal.SIGTERM)
        self.assertTrue(modeld.stop_requested, "매니저가 멈춘 것이라 빠른 종료로 세지 않는다")
        self.assertEqual({k: self.status(m)[k] for k in ("model", "source", "selected", "error")},
                         {"model": "models/ea2a_gnb.axmodel", "source": "selection", "selected": "ea2a_gnb", "error": ""})
        # 새 모델로 modeld가 거듭 바로 죽는다: 기본 모델로 돌아간다
        modeld.proc = None
        modeld.quick_exits = manager.MODEL_QUICK_EXITS_FOR_FALLBACK
        m.check_model_choice()
        self.assertEqual(modeld.spec.cmd, ["./modeld", manager.DEFAULT_MODEL_PATH])
        self.assertEqual((modeld.quick_exits, self.status(m)["error"]), (0, "failed"))
        m.check_model_choice()
        self.assertEqual(modeld.spec.cmd, ["./modeld", manager.DEFAULT_MODEL_PATH], "실패한 고르기를 다시 쓰지 않는다")
        # 고르기를 다시 쓰면 다시 해 본다
        self.choose("ea2a_gnb")
        m.check_model_choice()
        self.assertEqual(modeld.spec.cmd, ["./modeld", "models/ea2a_gnb.axmodel"])
        self.assertEqual(self.status(m)["error"], "")

    def record(self, m) -> dict:
        return json.loads(Path(m.model_record_path).read_text())

    def test_records_the_running_model_and_opens_a_new_route(self):
        """녹화 중에 바꿔도 route마다 실제로 달린 모델 하나만 남는다: 모델 기록(params, recordd가 route를 열 때 복사)을
        먼저 쓰고 recordd를 다시 띄운다. 기본 모델로 돌아갈 때도 같고, 모델이 그대로면 recordd는 그대로 둔다."""
        (self.root / "models" / "ea2a_gnb.axmodel").write_bytes(b"student")
        m = self.manager_with_modeld()
        recordd = manager.ProcState(manager.ProcSpec("recordd", ["./recordd"]))
        recordd.proc = subprocess.Popen(["/bin/sh", "-c", "sleep 30"], start_new_session=True)
        m.procs["recordd"] = recordd
        m.write_model_status()  # run()이 자식을 띄우기 전에 한다
        self.assertEqual(self.record(m), {"model": manager.DEFAULT_MODEL_PATH,
                                          "axmodel_sha256": hashlib.sha256(b"x").hexdigest(), "source": "default",
                                          "selected": "", "error": ""})
        stamp = manager.file_stamp(m.model_record_path)
        m.write_model_status()
        self.assertEqual(manager.file_stamp(m.model_record_path), stamp, "같은 내용이면 SD에 다시 쓰지 않는다")

        self.choose("ea2a_gnb")
        m.check_model_choice()
        self.assertEqual(self.record(m), {"model": "models/ea2a_gnb.axmodel",
                                          "axmodel_sha256": hashlib.sha256(b"student").hexdigest(),
                                          "source": "selection", "selected": "ea2a_gnb", "error": ""})
        self.assertEqual(recordd.proc.wait(timeout=5), -signal.SIGTERM)
        self.assertTrue(recordd.stop_requested, "새 route를 열려고 멈춘 것이라 빠른 종료로 세지 않는다")
        m.procs["modeld"].proc.wait(timeout=5)

        # 기본 모델로 돌아가도 기록과 route가 따라간다
        recordd.proc = subprocess.Popen(["/bin/sh", "-c", "sleep 30"], start_new_session=True)
        recordd.stop_requested = False
        m.procs["modeld"].proc = None
        m.procs["modeld"].quick_exits = manager.MODEL_QUICK_EXITS_FOR_FALLBACK
        m.check_model_choice()
        self.assertEqual((self.record(m)["model"], self.record(m)["error"]), (manager.DEFAULT_MODEL_PATH, "failed"))
        self.assertEqual(recordd.proc.wait(timeout=5), -signal.SIGTERM)

        # 모델은 그대로이고 기록의 고르기만 바뀌면(EDGEPILOT_MODEL이 이긴다) recordd는 그대로다
        recordd.proc = subprocess.Popen(["/bin/sh", "-c", "sleep 30"], start_new_session=True)
        m.argument_model = manager.DEFAULT_MODEL_PATH
        self.choose("supercombo")
        m.check_model_choice()
        self.assertEqual(self.record(m)["source"], "argument")
        self.assertTrue(recordd.running())
        manager.Manager.terminate_proc(m, recordd, signal.SIGKILL)
        recordd.proc.wait(timeout=5)

    def test_override_ignores_the_choice(self):
        m = self.manager_with_modeld()
        m.argument_model = "/tmp/test.axmodel"
        m.model_choice = manager.resolve_model(m.argument_model, m.selection_path)
        m.model = m.model_choice.path
        m.procs["modeld"].spec.cmd = ["./modeld", m.model]
        self.choose("ea2a_gnb")
        m.check_model_choice()
        self.assertTrue(m.procs["modeld"].running())
        self.assertEqual(m.procs["modeld"].spec.cmd, ["./modeld", "/tmp/test.axmodel"])
        self.assertEqual((self.status(m)["source"], self.status(m)["selected"]), ("argument", "ea2a_gnb"))
        manager.Manager.terminate_proc(m, m.procs["modeld"], signal.SIGKILL)
        m.procs["modeld"].proc.wait(timeout=5)

if __name__ == "__main__":
    unittest.main()
