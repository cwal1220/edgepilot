#!/usr/bin/env python3
"""MaixCAM2(AX630C) 보드 런타임 감시자. 프로세스를 시작 순서대로 띄우고(camerad가 VI와
AX 공용 풀을 연 뒤 overlayd) 죽으면 다시 띄우며(10초 안에 거듭 죽으면 최대 16초까지 늦춘다), 1초마다
managerState를 /dev/shm에 낸다. 어떤 프로세스를 띄울지는 EDGEPILOT_ENABLE_CONTROL·EDGEPILOT_ENABLE_PANDA·
EDGEPILOT_ENABLE_WEB_CONSOLE이 정한다. EDGEPILOT_REPLAY_ROUTE(+ _START, _DURATION 초)를 주면 카메라와 판다 대신 그
녹화 route를 재생하는 리허설 모드로 돈다(replayd가 camerad 자리를 맡는다).

시작 전에 보드 UI 런처와 그 앱을 멈춘다(CPU 약 30%를 쓰고 카메라·화면과 겹친다,
EDGEPILOT_STOP_LAUNCHER=0이면 그대로 둔다). USB-C 역할은 EDGEPILOT_USB_ROLE(host|device)이 있으면
그대로 고정하고, 없으면 pandad를 띄울 때만 호스트로 바꿨다가 끝낼 때 되돌린다.
EDGEPILOT_LOG_DIR를 주면(부팅 서비스 edgepilot.service) 자식의 출력을 그 디렉터리의 <이름>.log에 남기고
LOG_MAX_BYTES를 넘으면 <이름>.log.1로 하나 남긴 뒤 비운다(tmpfs에 두어 SD에 쓰지 않는다). 주지 않으면
터미널로 그대로 나온다.

주행 모델은 인자, EDGEPILOT_MODEL, 웹 콘솔에서 고른 것(params/driving_model.json의 {"model": "<이름>"} =
models/<이름>.axmodel), 기본 models/supercombo.axmodel 순으로 정한다. 고르기는 1초마다 다시 읽고, 바뀌면 modeld만 새
모델로 다시 띄운다(인자나 EDGEPILOT_MODEL이 있으면 고르기는 쓰지 않는다). 고른 모델로 modeld가 거듭 바로 죽으면 기본
모델로 돌아간다(고르기를 다시 쓰면 다시 해 본다). 쓰는 모델과 그 출처는 /dev/shm/edgepilot_driving_model.json에 낸다.
주행 기록용으로 실제로 쓰는 모델(경로, sha256, 출처)을 params/driving_model_running.json에도 남기고, 쓰는 모델이 바뀌면
recordd도 다시 띄운다. recordd는 route를 열 때만 params를 복사하므로, 이렇게 해야 녹화 중에 바꿔도 새 route가 열리고 각
route가 실제로 달린 모델 하나만 가리킨다.

사용: python3 manager.py [supercombo.axmodel]
"""
import hashlib
import json
import mmap
import os
import re
import shutil
import signal
import struct
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple


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
MODEL_DIR = "models"
DEFAULT_MODEL_PATH = "models/supercombo.axmodel"
# 웹 콘솔의 주행 모델 카드(web_console/driving_models.py)와 같은 파일·규칙이다. 고르기 파일은 파라미터 디렉터리에 있어
# recordd가 route마다 녹화한다.
MODEL_SELECTION_FILE = "driving_model.json"
MODEL_STATUS_PATH = "/dev/shm/edgepilot_driving_model.json"  # EDGEPILOT_DRIVING_MODEL_STATUS로 바꿀 수 있다
# 실제로 쓰는 모델의 기록(파라미터 디렉터리, recordd가 route를 열 때 복사한다). 내용이 바뀔 때만 쓴다(SD).
MODEL_RECORD_FILE = "driving_model_running.json"
MODEL_NAME = re.compile(r"[A-Za-z0-9_-][A-Za-z0-9_.-]{0,63}")
# 고른 모델로 modeld가 연달아 이만큼 빨리(QUICK_EXIT_S 안에) 죽으면 기본 모델로 돌아간다(약 4초)
MODEL_QUICK_EXITS_FOR_FALLBACK = 3
USB_ROLE_FILE = "/sys/class/usb_role/8000000.dwc3-role-switch/role"
LOG_MAX_BYTES = 1 << 20
# camerad가 VI를 열면 AX 공용 풀이 다시 설정돼서 먼저 떠 있던 화면·인코더의 풀이 무효가
# 된다. camerad가 다시 뜰 때마다 이 프로세스들도 camerad가 자리 잡은 뒤에 다시 띄운다.
CAMERA = "camerad"
CAMERA_DEPENDENTS = ("overlayd", "recordd")
# 멈추라고(SIGTERM) 한 뒤 SIGKILL까지 기다리는 시간. 끝낼 때 recordd는 tmpfs 스테이징에 남은 녹화를 SD로 옮긴 뒤
# 끝나므로 더 기다린다. edgepilot.service의 TimeoutStopSec(10초)이 매니저 전체를 SIGKILL하기 전에 끝나야 한다.
STOP_TIMEOUT_S = 3.0
RECORDD_STOP_TIMEOUT_S = 8.0
CAMERA_SETTLE_S = 1.5
# camerad가 크래시(신호)로 죽으면 VI/IVPS 그룹과 AX 풀을 풀지 못한 채 남아서, 다른 프로세스
# (modeld 등)가 AX를 쥐고 있는 한 새 camerad가 VI를 열지 못한다. 그때와 시작 직후 연달아
# 실패할 때는 전부 내렸다가 다시 띄운다.
CAMERA_QUICK_FAIL_S = 5.0
CAMERA_QUICK_FAILS_FOR_FULL_RESTART = 2
# 다른 프로세스가 이보다 빨리 죽기를 거듭하면 다시 띄우는 간격을 늘린다(1초마다 다시 띄우면 웹 콘솔은 FastAPI를
# 매번 다시 읽고, journal에 두 줄씩 쌓인다). 세 번까지는 1초, 그 뒤 2·4·8·16초. 오래 돌면 처음으로 돌아간다.
QUICK_EXIT_S = 10.0
RESTART_DELAYS_S = (1.0, 1.0, 1.0, 1.0, 2.0, 4.0, 8.0, 16.0)  # 연달아 빨리 죽은 횟수별
# 감시 루프의 최대 대기. 프로세스가 죽은 것을 이 안에 알아챈다.
SUPERVISE_INTERVAL_S = 0.25


def now_ns() -> int:
    return time.clock_gettime_ns(time.CLOCK_BOOTTIME)


def env_enabled(name: str, default: bool = False) -> bool:
    """utils_process.h의 env_flag와 같은 규약을 따른다."""
    value = os.environ.get(name)
    if value is None or value == "":
        return default
    return value.lower() not in ("0", "false", "no", "off", "n")


def params_dir() -> str:
    """C++ 프로세스·웹 콘솔과 같은 파라미터 디렉터리(EDGEPILOT_PARAMS_DIR, 없으면 params/)."""
    return os.environ.get("EDGEPILOT_PARAMS_DIR") or "params"


def model_choice(path: str) -> Tuple[str, str]:
    """고르기 파일의 (이름, 오류). {"model": "<이름>"}이고, deploy_student.py가 쓰는 "models/<이름>.axmodel" 꼴도 받는다.
    파일이 없으면 ("", "none")."""
    try:
        with open(path, encoding="utf-8") as file:
            document = json.load(file)
    except FileNotFoundError:
        return "", "none"
    except (OSError, ValueError):
        return "", "unreadable"
    if not isinstance(document, dict):  # JSON 객체가 아니면 깨진 파일과 같다(웹 콘솔 read_json_object)
        return "", "unreadable"
    name = document.get("model")
    if not isinstance(name, str):
        return "", "no_model"
    prefix, suffix = MODEL_DIR + "/", ".axmodel"
    if name.startswith(prefix) and name.endswith(suffix):
        name = name[len(prefix):-len(suffix)]
    return (name, "") if MODEL_NAME.fullmatch(name) else ("", "bad_name")


@dataclass(frozen=True)
class ModelChoice:
    path: str  # modeld에 주는 모델
    source: str  # argument | env | selection | default
    selected: str = ""  # 고르기 파일의 이름(인자나 EDGEPILOT_MODEL이 있으면 쓰지 않는다)
    error: str = ""  # 고르기를 쓰지 못한 이유: unreadable, no_model, bad_name, missing(파일 없음), failed(modeld가 죽음)


def resolve_model(argument: Optional[str], selection_path: str, failed: str = "") -> ModelChoice:
    """인자 > EDGEPILOT_MODEL > 고르기 > 기본. failed는 이번 고르기로 modeld가 거듭 죽은 모델 경로다."""
    name, error = model_choice(selection_path)
    error = "" if error == "none" else error
    if argument:
        return ModelChoice(argument, "argument", name, error)
    if os.environ.get("EDGEPILOT_MODEL"):
        return ModelChoice(os.environ["EDGEPILOT_MODEL"], "env", name, error)
    if not name:
        return ModelChoice(DEFAULT_MODEL_PATH, "default", "", error)
    path = f"{MODEL_DIR}/{name}.axmodel"
    if not os.path.isfile(path):
        return ModelChoice(DEFAULT_MODEL_PATH, "default", name, "missing")
    if path == failed and path != DEFAULT_MODEL_PATH:
        return ModelChoice(DEFAULT_MODEL_PATH, "default", name, "failed")
    return ModelChoice(path, "selection", name)


def file_stamp(path: str) -> Optional[Tuple[int, int]]:
    try:
        st = os.stat(path)
    except OSError:
        return None
    return st.st_mtime_ns, st.st_size


def sha256_file(path: str) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as file:
        for block in iter(lambda: file.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def write_json_atomic(path: str, document: dict):
    """tmpfs의 상태 파일: 임시 파일에 쓰고 rename(읽는 쪽이 반쯤 쓴 파일을 보지 않게)."""
    directory = os.path.dirname(path) or "."
    fd, temporary = tempfile.mkstemp(prefix=os.path.basename(path) + ".", suffix=".tmp", dir=directory)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as file:
            json.dump(document, file)
        os.chmod(temporary, 0o664)
        os.replace(temporary, path)
    except OSError:
        try:
            os.remove(temporary)
        except OSError:
            pass
        raise


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
        magic, version, capacity, *_ = self._read_header()
        if magic != IPC_MAGIC or version != IPC_VERSION or capacity != self.payload_size:
            self._write_header(0, 0, 0)

    def publish(self, payload: bytes):
        """payload는 payload_size 그대로다."""
        _, _, _, _, seq, ts, size, _ = self._read_header()
        if seq & 1:  # 쓰다가 죽은 이전 매니저
            seq += 1
        self._write_header(seq + 1, ts, size)
        self.map.seek(HEADER_SIZE)
        self.map.write(payload)
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
    # EDGEPILOT_USB_ROLE(host|device)를 주면 그 역할로 두고 끝낼 때 되돌리지 않는다(부팅 서비스는
    # host: 차에서 판다를 USB-C 호스트로 붙인다). 없으면 pandad가 뜰 때만 host로 바꾸고 되돌린다.
    fixed = os.environ.get("EDGEPILOT_USB_ROLE", "").strip().lower()
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


def restart_delay(name: str, quick_exits: int) -> float:
    """죽은 프로세스를 다시 띄우기까지의 초. camerad는 자기 규칙(전체 재시작)이 따로 있어 늘 1초다."""
    if name == CAMERA:
        return 1.0
    return RESTART_DELAYS_S[min(quick_exits, len(RESTART_DELAYS_S) - 1)]


@dataclass
class ProcState:
    spec: ProcSpec
    proc: Optional[subprocess.Popen] = None
    next_restart_monotonic: float = 0.0
    started_monotonic: float = 0.0
    quick_exits: int = 0  # 연달아 QUICK_EXIT_S 안에 죽은 횟수
    stop_requested: bool = False  # 매니저가 멈추라고 보냈다(그 종료는 빠른 종료로 세지 않는다)

    def running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None


def note_exit(state: ProcState, now: float) -> float:
    """종료를 세고 다음 시작까지의 지연을 돌려준다. 매니저가 멈춘 것(camerad를 따라 내린 overlayd·recordd 등)은
    죽은 것이 아니므로 빠른 종료로 세지 않는다."""
    if not state.stop_requested:
        quick = now - state.started_monotonic < QUICK_EXIT_S
        state.quick_exits = state.quick_exits + 1 if quick else 0
    state.stop_requested = False
    return restart_delay(state.spec.name, state.quick_exits)


def child_setup(nice_adjust: int):
    os.setsid()
    if nice_adjust != 0:
        os.nice(nice_adjust)


class Manager:
    def __init__(self, argv: List[str]):
        if len(argv) > 2:
            raise ValueError(
                f"Usage: {argv[0] if argv else 'manager.py'} [supercombo.axmodel]"
            )
        self.argument_model = argv[1] if len(argv) >= 2 else None
        self.selection_path = os.path.join(params_dir(), MODEL_SELECTION_FILE)
        self.model_status_path = os.environ.get("EDGEPILOT_DRIVING_MODEL_STATUS") or MODEL_STATUS_PATH
        self.model_record_path = os.path.join(params_dir(), MODEL_RECORD_FILE)
        self.model_digests: Dict[str, Tuple[Optional[Tuple[int, int]], str]] = {}  # 경로 -> (파일 stamp, sha256)
        self.selection_stamp = file_stamp(self.selection_path)
        self.failed_model = ""  # 이번 고르기로 modeld가 거듭 죽은 모델(고르기 파일이 바뀌면 잊는다)
        self.model_choice = resolve_model(self.argument_model, self.selection_path)
        self.model = self.model_choice.path
        print(f"manager: model {self.model} ({self.model_choice.source})", flush=True)
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
        """자식은 O_APPEND로 쓰므로 파일을 0으로 자르면 새 끝에서 이어 쓴다. 자르기 전 내용은 <이름>.log.1로 하나
        남겨, 주행 중 문제의 로그가 자르는 순간 사라지지 않게 한다(로그는 tmpfs에 있다)."""
        for name in self.procs:
            path = self.log_path(name)
            try:
                if os.path.getsize(path) <= LOG_MAX_BYTES:
                    continue
            except OSError:
                continue
            try:
                shutil.copyfile(path, path + ".1")
            except OSError:
                # 공간이 모자라 남기지 못해도 자르기는 한다(tmpfs가 차면 더 나쁘다). 반쯤 쓴 사본은 지운다.
                try:
                    os.remove(path + ".1")
                except OSError:
                    pass
            os.truncate(path, 0)

    def start_proc(self, state: ProcState):
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
        state.stop_requested = False
        print(f"manager: started {state.spec.name} pid={state.proc.pid} nice={state.spec.nice} "
              f"cmd={' '.join(state.spec.cmd)}",
              flush=True)

    def stop_all(self, timeout_s: float = STOP_TIMEOUT_S, recordd_timeout_s: Optional[float] = None):
        """모두 SIGTERM을 보내고 timeout_s 뒤에도 남은 것은 SIGKILL한다. recordd_timeout_s를 주면 recordd만 그만큼
        기다린다: 끝낼 때 recordd는 tmpfs 스테이징에 남은 녹화를 SD로 옮긴 뒤 끝난다(전원을 끄면 tmpfs가 사라진다)."""
        for state in self.procs.values():
            self.terminate_proc(state, signal.SIGTERM)
        start = time.monotonic()

        def deadline(state: ProcState) -> float:
            wait = recordd_timeout_s if recordd_timeout_s is not None and state.spec.name == "recordd" else timeout_s
            return start + wait

        while True:
            now = time.monotonic()
            for state in self.procs.values():
                if state.running() and now >= deadline(state):
                    self.terminate_proc(state, signal.SIGKILL)
            if all(not state.running() or now >= deadline(state) for state in self.procs.values()):
                break
            time.sleep(0.1)

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
        state.stop_requested = True
        try:
            os.killpg(os.getpgid(state.proc.pid), sig)
        except ProcessLookupError:
            pass

    def write_model_status(self):
        choice = self.model_choice
        try:
            write_json_atomic(self.model_status_path, {"model": choice.path, "source": choice.source,
                                                       "selected": choice.selected, "error": choice.error,
                                                       "unix_time": time.time()})
        except OSError as exc:
            print(f"manager: cannot write {self.model_status_path}: {exc}", flush=True)
        self.write_model_record()

    def model_digest(self, path: str) -> str:
        """모델 파일의 sha256(보드에서 37 MB에 약 0.3초라 파일 stamp가 같으면 다시 계산하지 않는다), 못 읽으면 ""."""
        stamp = file_stamp(path)
        cached = self.model_digests.get(path)
        if cached is None or cached[0] != stamp:
            try:
                cached = (stamp, sha256_file(path) if stamp is not None else "")
            except OSError:
                cached = (stamp, "")
            self.model_digests[path] = cached
        return cached[1]

    def write_model_record(self):
        """params/driving_model_running.json: modeld에 준 모델과 그 sha256, 출처. recordd가 route를 열 때 복사하므로
        주행 기록마다 실제로 달린 모델이 남는다(고르기 파일은 바라는 것이고, 기본 모델로 돌아갔거나 EDGEPILOT_MODEL이
        있으면 다르다). 같은 내용이면 다시 쓰지 않는다."""
        choice = self.model_choice
        record = {"model": choice.path, "axmodel_sha256": self.model_digest(choice.path), "source": choice.source,
                  "selected": choice.selected, "error": choice.error}
        try:
            with open(self.model_record_path, encoding="utf-8") as file:
                if json.load(file) == record:
                    return
        except (OSError, ValueError):
            pass
        try:
            write_json_atomic(self.model_record_path, record)
        except OSError as exc:
            print(f"manager: cannot write {self.model_record_path}: {exc}", flush=True)

    def check_model_choice(self):
        """1초마다: 고르기 파일이 바뀌었거나 고른 모델로 modeld가 거듭 죽었으면 모델을 다시 정하고, 달라졌으면 modeld를
        새 모델로, recordd를 새 route로 다시 띄운다(매니저가 멈춘 것이라 빠른 종료로 세지 않는다). 모델 기록을 먼저 쓰고
        recordd는 1초 뒤에 뜨므로 새 route의 params 사본에 새 모델이 들어간다."""
        stamp = file_stamp(self.selection_path)
        if stamp != self.selection_stamp:
            self.selection_stamp = stamp
            self.failed_model = ""
        modeld = self.procs.get("modeld")
        if (modeld is not None and self.model_choice.source == "selection"
                and modeld.quick_exits >= MODEL_QUICK_EXITS_FOR_FALLBACK and self.failed_model != self.model):
            print(f"manager: modeld keeps exiting with {self.model}, back to {DEFAULT_MODEL_PATH}", flush=True)
            self.failed_model = self.model
        choice = resolve_model(self.argument_model, self.selection_path, self.failed_model)
        if choice == self.model_choice:
            return
        self.model_choice = choice
        self.write_model_status()
        if choice.path == self.model:
            return
        print(f"manager: driving model {self.model} -> {choice.path} ({choice.source})", flush=True)
        self.model = choice.path
        if modeld is not None:
            modeld.spec.cmd = ["./modeld", choice.path]
            modeld.quick_exits = 0
            if modeld.running():
                self.terminate_proc(modeld, signal.SIGTERM)
            else:
                modeld.next_restart_monotonic = time.monotonic()
        recordd = self.procs.get("recordd")
        if recordd is not None and recordd.running():
            print("manager: restarting recordd so the next route records the new model", flush=True)
            self.terminate_proc(recordd, signal.SIGTERM)

    def publish_state(self):
        timestamp = now_ns()
        payload = struct.pack("<QII", timestamp, len(self.procs), 0)
        for state in self.procs.values():  # 이름은 15자 안이고 16s가 NUL로 채운다
            payload += PROCESS.pack(state.spec.name.encode("ascii"), 1 if state.running() else 0)
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
        self.write_model_status()
        waiting = []  # camerad가 자리 잡기를 기다리는 화면·기록
        for state in self.procs.values():
            if state.spec.name in CAMERA_DEPENDENTS and not self.camera_settled():
                waiting.append(state)
                continue
            self.start_proc(state)
            time.sleep(0.3)
            # camerad가 자리 잡았으면 다음 차례보다 먼저 띄운다: 시작 순서가 끝날 때까지 기다리면 LCD가
            # 0.6초 늦게 켜진다
            while waiting and self.camera_settled():
                self.start_proc(waiting.pop(0))
                time.sleep(0.3)
        # 남은 것은 감시 루프가 camerad가 자리 잡은 뒤에 띄운다

        last_publish = 0.0
        exit_code = 0
        while not self.shutdown:
            now = time.monotonic()
            if now - last_publish >= 1.0:
                self.check_model_choice()
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
                delay = note_exit(state, now)
                if delay > 1.0:
                    print(f"manager: {state.spec.name} keeps exiting, next start in {delay:.0f} s", flush=True)
                state.next_restart_monotonic = now + delay
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

            time.sleep(SUPERVISE_INTERVAL_S)

        # 끝낼 때(systemctl stop, reboot)는 recordd가 스테이징에 남은 녹화를 SD로 옮길 시간을 준다
        self.stop_all(recordd_timeout_s=RECORDD_STOP_TIMEOUT_S)
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
