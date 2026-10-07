#!/usr/bin/env python3
"""MaixCAM2(AX630C) 보드 런타임 감시자. 프로세스를 시작 순서대로 띄우고(camerad가 VI와
AX 공용 풀을 연 뒤 overlayd) 죽으면 1초 뒤 다시 띄우며, 1초마다 managerState를 /dev/shm에
낸다. 어떤 프로세스를 띄울지는 EDGEPILOT_ENABLE_CONTROL·EDGEPILOT_ENABLE_PANDA·
EDGEPILOT_ENABLE_WEB_CONSOLE이 정한다. EDGEPILOT_REPLAY_ROUTE(+ _START, _DURATION 초)를 주면 카메라와
판다 대신 그 녹화 route를 재생하는 리허설 모드로 돈다(replayd가 camerad 자리를 맡는다).

시작 전에 보드 UI 런처와 그 앱을 멈춘다(CPU 약 30%를 쓰고 카메라·화면과 겹친다,
EDGEPILOT_STOP_LAUNCHER=0이면 그대로 둔다). USB-C 역할은 EDGEPILOT_USB_ROLE(host|device)이 있으면
그대로 고정하고, 없으면 pandad를 띄울 때만 호스트로 바꿨다가 끝낼 때 되돌린다. EDGEPILOT_LOG_DIR를 주면(부팅 서비스 edgepilot.service) 자식의 출력을
그 디렉터리의 <이름>.log에 남기고 LOG_MAX_BYTES를 넘으면 비운다(tmpfs에 두어 SD에
쓰지 않는다). 주지 않으면 터미널로 그대로 나온다.

사용: python3 manager.py [supercombo.axmodel]   (모델 기본값은 설치 디렉터리의
models/supercombo.axmodel, EDGEPILOT_MODEL로도 바꿀 수 있다)
"""
import mmap
import os
import signal
import struct
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional


IPC_MAGIC = 0x4B323349
IPC_VERSION = 1
HEADER = struct.Struct("<IIIIQQII")
HEADER_SIZE = HEADER.size
# ProcessState: overlayd가 읽는 것은 이름과 running뿐이다.
PROCESS = struct.Struct("<16sI")
MAX_PROCESSES = 12
# C++ ManagerState는 8바이트 정렬이라 배열 뒤에 꼬리 패딩이 붙는다.
_MANAGER_STATE_BODY = 8 + 4 + 4 + PROCESS.size * MAX_PROCESSES
MANAGER_STATE_SIZE = (_MANAGER_STATE_BODY + 7) // 8 * 8
DEFAULT_MODEL_PATH = "models/supercombo.axmodel"
USB_ROLE_FILE = "/sys/class/usb_role/8000000.dwc3-role-switch/role"
LOG_MAX_BYTES = 1 << 20
# camerad가 VI를 열면 AX 공용 풀이 다시 설정돼서 먼저 떠 있던 화면·인코더의 풀이 무효가
# 된다. camerad가 다시 뜰 때마다 이 프로세스들도 camerad가 자리 잡은 뒤에 다시 띄운다.
CAMERA = "camerad"
CAMERA_DEPENDENTS = ("overlayd", "recordd")
CAMERA_SETTLE_S = 1.5
# camerad가 크래시(신호)로 죽으면 VI/IVPS 그룹과 AX 풀을 풀지 못한 채 남아서, 다른 프로세스
# (modeld 등)가 AX를 쥐고 있는 한 새 camerad가 VI를 열지 못한다. 그때와 시작 직후 연달아
# 실패할 때는 전부 내렸다가 다시 띄운다.
CAMERA_QUICK_FAIL_S = 5.0
CAMERA_QUICK_FAILS_FOR_FULL_RESTART = 2


def now_ns() -> int:
    if hasattr(time, "CLOCK_BOOTTIME"):
        return time.clock_gettime_ns(time.CLOCK_BOOTTIME)
    return time.monotonic_ns()


def env_enabled(name: str, default: bool = False) -> bool:
    """utils_process.h의 env_flag와 같은 규약을 따른다."""
    value = os.environ.get(name)
    if value is None or value == "":
        return default
    return value.lower() not in ("0", "false", "no", "off", "n")


def default_model_path() -> str:
    return os.environ.get("EDGEPILOT_MODEL") or DEFAULT_MODEL_PATH


class LatestPublisher:
    def __init__(self, name: str, payload_size: int):
        path = "/dev/shm/" + name.lstrip("/")
        self.payload_size = payload_size
        self.fd = os.open(path, os.O_CREAT | os.O_RDWR, 0o664)
        os.ftruncate(self.fd, HEADER_SIZE + payload_size)
        self.map = mmap.mmap(self.fd, HEADER_SIZE + payload_size)
        self._init_header_if_needed()

    def _read_header(self):
        self.map.seek(0)
        return HEADER.unpack(self.map.read(HEADER_SIZE))

    def _write_header(self, seq: int, timestamp_ns: int, payload_size: int):
        self.map.seek(0)
        self.map.write(HEADER.pack(
            IPC_MAGIC,
            IPC_VERSION,
            self.payload_size,
            0,
            seq,
            timestamp_ns,
            payload_size,
            0,
        ))

    def _init_header_if_needed(self):
        try:
            magic, version, capacity, _, seq, ts, size, _ = self._read_header()
        except struct.error:
            magic = version = capacity = seq = ts = size = 0
        if magic != IPC_MAGIC or version != IPC_VERSION or capacity != self.payload_size:
            self._write_header(0, 0, 0)

    def publish(self, payload: bytes):
        if len(payload) > self.payload_size:
            raise ValueError("payload too large")
        magic, version, capacity, reserved0, seq, ts, size, reserved1 = self._read_header()
        if seq & 1:
            seq += 1
        self._write_header(seq + 1, ts, size)
        self.map.seek(HEADER_SIZE)
        self.map.write(payload)
        if len(payload) < self.payload_size:
            self.map.write(b"\x00" * (self.payload_size - len(payload)))
        self._write_header(seq + 2, now_ns(), len(payload))

    def close(self):
        self.map.close()
        os.close(self.fd)


@dataclass
class ProcSpec:
    name: str
    cmd: List[str]
    nice: int = 0
    installed: str = ""  # 설치 여부를 볼 경로(기본은 cmd[0])


def process_specs(model: str) -> List[ProcSpec]:
    """시작 순서대로. camerad가 VI를 열며 AX 공용 풀을 설정하므로 화면(overlayd)보다 먼저 뜬다."""
    enable_control = env_enabled("EDGEPILOT_ENABLE_CONTROL", True)
    # 리허설: 녹화 route를 카메라·판다 대신 재생한다(replayd). 나머지는 그대로다.
    replay = os.environ.get("EDGEPILOT_REPLAY_ROUTE")
    camera = (ProcSpec(CAMERA, ["./replayd", replay,
                                os.environ.get("EDGEPILOT_REPLAY_START", "0"),
                                os.environ.get("EDGEPILOT_REPLAY_DURATION", "0")], 0)
              if replay else ProcSpec(CAMERA, ["./camerad"], 0))
    specs = [
        camera,
        ProcSpec("overlayd", ["./overlayd"], 10),
        ProcSpec("recordd", ["./recordd"], 15),
        ProcSpec("modeld", ["./modeld", model], -15),
    ]
    # 보드 IMU와 locationd. 없거나 죽으면 학습기가 ESP12 값으로 돌아가고, 리허설에서는 책상 위라 뺀다.
    if not replay:
        specs.append(ProcSpec("imud", ["./imud"], 10))
        # 자세 추정(paramsd·torqued 입력)과 조향 지연 학습. IMU가 없으면 발행 없이 기다린다.
        specs.append(ProcSpec("locationd", ["./locationd"], 5))
    # Panda는 USB-C를 호스트로 바꿔 쓰므로 명시적으로 켤 때만 띄운다(edgepilot.service가 켠다).
    if env_enabled("EDGEPILOT_ENABLE_PANDA") and not replay:
        specs.append(ProcSpec("pandad", ["./pandad"], -10))
    if enable_control:
        specs.append(ProcSpec("controlsd", ["./controlsd"], -8))
    if env_enabled("EDGEPILOT_ENABLE_WEB_CONSOLE", enable_control):
        specs.append(ProcSpec("web_console", [sys.executable, "-m", "web_console"], 10,
                              installed="web_console/__main__.py"))
    missing = [spec.name for spec in specs if not os.path.exists(spec.installed or spec.cmd[0])]
    if missing:
        print(f"manager: not installed, skipping {', '.join(missing)}", flush=True)
    return [spec for spec in specs if spec.name not in missing]


def prepare_board(run_pandad: bool) -> Optional[str]:
    """UI 런처를 멈추고, pandad가 뜨면 USB-C를 호스트로 바꾼다. 바꾸기 전 역할을 돌려준다."""
    if env_enabled("EDGEPILOT_STOP_LAUNCHER", True):
        ret = subprocess.run(["systemctl", "stop", "launcher.service"],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode
        print(f"manager: launcher.service stop -> {ret}", flush=True)
        # 런처가 띄운 앱(예: 카메라 앱)이 남아 있으면 카메라·NPU를 쥐고 있다.
        subprocess.run(["pkill", "-TERM", "-f", "^/maixapp/apps/"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if not os.path.exists(USB_ROLE_FILE):
        return None
    # EDGEPILOT_USB_ROLE(host|device)를 주면 그 역할로 두고 끝낼 때 되돌리지 않는다(부팅 서비스는
    # host: 차에서 판다를 USB-C 호스트로 붙인다). 없으면 pandad가 뜰 때만 host로 바꾸고 되돌린다.
    fixed = os.environ.get("EDGEPILOT_USB_ROLE", "").strip().lower()
    if fixed and fixed not in ("host", "device"):
        print(f"manager: ignoring EDGEPILOT_USB_ROLE={fixed!r} (host or device)", flush=True)
        fixed = ""
    wanted = fixed or ("host" if run_pandad else "")
    if not wanted:
        return None
    try:
        with open(USB_ROLE_FILE) as f:
            previous = f.read().strip()
        if previous != wanted:
            with open(USB_ROLE_FILE, "w") as f:
                f.write(wanted)
        print(f"manager: USB-C role {previous} -> {wanted}", flush=True)
        return None if fixed else previous
    except OSError as exc:
        print(f"manager: cannot set USB role {wanted}: {exc}", flush=True)
        return None


def restore_usb_role(previous: Optional[str]):
    if not previous or previous == "host":
        return
    try:
        with open(USB_ROLE_FILE, "w") as f:
            f.write(previous)
        print(f"manager: USB-C role restored to {previous}", flush=True)
    except OSError as exc:
        print(f"manager: cannot restore USB role: {exc}", flush=True)


@dataclass
class ProcState:
    spec: ProcSpec
    proc: Optional[subprocess.Popen] = None
    next_restart_monotonic: float = 0.0
    started_monotonic: float = 0.0

    def running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None


def child_setup(nice_adjust: int):
    os.setsid()
    if nice_adjust != 0:
        try:
            os.nice(nice_adjust)
        except OSError:
            pass


class Manager:
    def __init__(self, argv: List[str]):
        if len(argv) > 2:
            raise ValueError(
                f"Usage: {argv[0] if argv else 'manager.py'} [supercombo.axmodel]"
            )
        self.model = argv[1] if len(argv) >= 2 else default_model_path()
        print(f"manager: model {self.model}", flush=True)
        os.environ.setdefault("EDGEPILOT_PANDA_TX", "1")
        os.environ.setdefault("EDGEPILOT_PANDA_ENGAGED", "1")
        os.environ.setdefault("EDGEPILOT_PANDA_SAFETY", "hyundaiCommunity")
        os.environ.setdefault("EDGEPILOT_PANDA_SAFETY_PARAM", "0")
        self.shutdown = False
        self.manager_state = LatestPublisher("/edgepilot_manager_state", MANAGER_STATE_SIZE)
        # 시작 순서 = 상태 테이블 순서. dict는 삽입 순서를 지킨다.
        self.procs: Dict[str, ProcState] = {
            spec.name: ProcState(spec=spec)
            for spec in process_specs(self.model)
        }
        self.previous_usb_role: Optional[str] = None
        self.camera_quick_fails = 0
        self.log_dir = os.environ.get("EDGEPILOT_LOG_DIR") or None
        if self.log_dir:
            os.makedirs(self.log_dir, exist_ok=True)

    def log_path(self, name: str) -> str:
        return os.path.join(self.log_dir, name + ".log")

    def trim_logs(self):
        """자식은 O_APPEND로 쓰므로 파일을 0으로 자르면 새 끝에서 이어 쓴다."""
        for name in self.procs:
            path = self.log_path(name)
            try:
                if os.path.getsize(path) > LOG_MAX_BYTES:
                    os.truncate(path, 0)
            except OSError:
                pass

    def start_proc(self, state: ProcState):
        if not os.path.exists(state.spec.cmd[0]):
            raise FileNotFoundError(f"{state.spec.cmd[0]} not found")
        log = open(self.log_path(state.spec.name), "ab") if self.log_dir else None
        try:
            state.proc = subprocess.Popen(
                state.spec.cmd,
                stdout=log,
                stderr=subprocess.STDOUT if log else None,
                preexec_fn=lambda nice=state.spec.nice: child_setup(nice),
            )
        finally:
            if log:
                log.close()
        state.started_monotonic = time.monotonic()
        print(f"manager: started {state.spec.name} pid={state.proc.pid} nice={state.spec.nice} "
              f"cmd={' '.join(state.spec.cmd)}",
              flush=True)

    def stop_all(self, timeout_s: float = 3.0):
        for state in self.procs.values():
            self.terminate_proc(state, signal.SIGTERM)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if all(not state.running() for state in self.procs.values()):
                break
            time.sleep(0.1)
        for state in self.procs.values():
            self.terminate_proc(state, signal.SIGKILL)

    def restart_all(self):
        """모두 내린 뒤 감시 루프가 시작 순서대로(camerad 먼저) 다시 띄우게 한다."""
        self.stop_all()
        restart_at = time.monotonic() + 1.0
        for state in self.procs.values():
            if state.proc is not None:
                state.proc.wait()
            state.proc = None
            state.next_restart_monotonic = restart_at

    def camera_settled(self) -> bool:
        camera = self.procs.get(CAMERA)
        if camera is None:
            return True
        return camera.running() and time.monotonic() - camera.started_monotonic >= CAMERA_SETTLE_S

    def terminate_proc(self, state: ProcState, sig=signal.SIGTERM):
        if not state.running():
            return
        try:
            os.killpg(os.getpgid(state.proc.pid), sig)
        except ProcessLookupError:
            pass

    def publish_state(self):
        timestamp = now_ns()
        payload = struct.pack("<QII", timestamp, min(len(self.procs), MAX_PROCESSES), 0)
        for state in list(self.procs.values())[:MAX_PROCESSES]:
            proc_name = state.spec.name.encode("ascii")[:15].ljust(16, b"\x00")
            payload += PROCESS.pack(proc_name, 1 if state.running() else 0)
        payload += b"\x00" * (MANAGER_STATE_SIZE - len(payload))
        self.manager_state.publish(payload)

    def handle_signal(self, signum, _frame):
        print(f"\nmanager: signal {signum}, stopping children", flush=True)
        self.shutdown = True
        for state in self.procs.values():
            self.terminate_proc(state, signal.SIGTERM)

    def run(self) -> int:
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, self.handle_signal)

        self.previous_usb_role = prepare_board("pandad" in self.procs)
        for state in self.procs.values():
            if state.spec.name in CAMERA_DEPENDENTS and not self.camera_settled():
                continue  # 감시 루프가 camerad가 자리 잡은 뒤에 띄운다
            self.start_proc(state)
            time.sleep(0.3)

        last_publish = 0.0
        exit_code = 0
        while not self.shutdown:
            now = time.monotonic()
            if now - last_publish >= 1.0:
                self.publish_state()
                if self.log_dir:
                    self.trim_logs()
                last_publish = now

            for state in self.procs.values():
                if state.proc is None:
                    continue
                ret = state.proc.poll()
                if ret is None:
                    continue
                print(f"\nmanager: {state.spec.name} exited code={ret}", flush=True)
                state.proc = None
                exit_code = ret if ret != 0 else exit_code
                state.next_restart_monotonic = now + 1.0
                if state.spec.name == CAMERA:
                    quick = now - state.started_monotonic < CAMERA_QUICK_FAIL_S
                    self.camera_quick_fails = self.camera_quick_fails + 1 if quick else 0
                    if ret < 0 or self.camera_quick_fails >= CAMERA_QUICK_FAILS_FOR_FULL_RESTART:
                        print(f"manager: camerad {'crashed' if ret < 0 else 'keeps failing'}, "
                              "restarting everything to release the AX resources", flush=True)
                        self.camera_quick_fails = 0
                        self.restart_all()
                        break
                    for name in CAMERA_DEPENDENTS:
                        if name in self.procs and self.procs[name].running():
                            print(f"manager: restarting {name} after camerad", flush=True)
                            self.terminate_proc(self.procs[name], signal.SIGTERM)

            for state in self.procs.values():
                if state.proc is not None or self.shutdown:
                    continue
                if state.spec.name in CAMERA_DEPENDENTS and not self.camera_settled():
                    continue
                if now >= state.next_restart_monotonic:
                    self.start_proc(state)

            time.sleep(0.1)

        self.stop_all()
        self.publish_state()
        self.manager_state.close()
        restore_usb_role(self.previous_usb_role)
        return exit_code


def main(argv: List[str]) -> int:
    # 실행 파일(./*d)과 models/는 설치 디렉터리 기준이다.
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    try:
        return Manager(argv).run()
    except Exception as exc:
        print(f"manager error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
