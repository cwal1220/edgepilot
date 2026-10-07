"""런타임 프로세스 상태: 매니저가 1초마다 내는 ManagerState(프로세스 이름과 실행 여부, src/common/ipc_messages.h)와,
파라미터를 바꿨을 때 다시 읽으라고 신호를 보낼 프로세스 찾기."""
from __future__ import annotations

import os
import signal
import struct
from pathlib import Path
from typing import Any, Dict, List

from .shm_channel import IpcReader, age_s, topic_path

MANAGER_STATE_TOPIC = "/edgepilot_manager_state"
MANAGER_STATE_HEAD = struct.Struct("<QII")  # timestamp_ns, process_count, reserved
PROCESS_STATE = struct.Struct("<16sI")      # ProcessState: name, running
MAX_PROCESSES = 12
MANAGER_STALE_S = 3.0


def find_process_pids(name: str) -> List[int]:
    """argv[0]의 파일 이름이 name인 프로세스들."""
    pids = []
    proc = Path("/proc")
    if not proc.is_dir():
        return pids
    for entry in proc.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            argv0 = (entry / "cmdline").read_bytes().split(b"\0", 1)[0]
            if Path(os.fsdecode(argv0)).name == name:
                pids.append(int(entry.name))
        except (FileNotFoundError, PermissionError, ProcessLookupError, ValueError):
            continue
    return sorted(pids)


def signal_process(name: str, signum: int = signal.SIGHUP) -> List[int]:
    """name 프로세스들에 신호를 보내고, 받은 pid를 돌려준다."""
    notified = []
    for pid in find_process_pids(name):
        try:
            os.kill(pid, signum)
            notified.append(pid)
        except (PermissionError, ProcessLookupError):
            continue
    return notified


class ProcessStatus:
    def __init__(self, path: str = topic_path(MANAGER_STATE_TOPIC)):
        self.reader = IpcReader(path, MANAGER_STATE_HEAD.size + PROCESS_STATE.size * MAX_PROCESSES)

    def status(self) -> Dict[str, Any]:
        """{"available", "age_s", "processes": [{"name", "running"}]}. 매니저가 없으면(또는 3초 넘게 멈췄으면)
        available이 False다."""
        latest = self.reader.read_payload()
        if latest is None:
            return {"available": False, "age_s": 0.0, "processes": []}
        _, stamp, payload = latest
        _, count, _ = MANAGER_STATE_HEAD.unpack_from(payload)
        processes = []
        for index in range(min(count, MAX_PROCESSES)):
            raw_name, running = PROCESS_STATE.unpack_from(payload, MANAGER_STATE_HEAD.size + index * PROCESS_STATE.size)
            processes.append({"name": raw_name.split(b"\0", 1)[0].decode("ascii", "replace"), "running": bool(running)})
        age = age_s(stamp)
        return {"available": age <= MANAGER_STALE_S, "age_s": age, "processes": processes}
