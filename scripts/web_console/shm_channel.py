"""/dev/shm의 LatestChannel(src/common/ipc_channels.h)을 읽기 전용으로 연다. 웹 콘솔의 상태 모듈이 함께 쓴다."""
from __future__ import annotations

import mmap
import os
import struct
import time
from typing import Any, Dict

IPC_MAGIC = 0x4B323349
IPC_VERSION = 1
IPC_HEADER = struct.Struct("<IIIIQQII")  # IpcHeader; seq가 홀수면 쓰는 중


def topic_path(topic: str) -> str:
    """ipc_messages.h의 토픽 이름("/edgepilot_...")을 공유 메모리 파일 경로로."""
    return "/dev/shm" + topic


def boottime_ns() -> int:
    clock = getattr(time, "CLOCK_BOOTTIME", time.CLOCK_MONOTONIC)  # monotonic_now_ns와 같은 시계
    return time.clock_gettime_ns(clock)


def age_s(stamp_ns: int) -> float:
    """CLOCK_BOOTTIME 시각 stamp_ns가 몇 초 전인가."""
    return max(0.0, (boottime_ns() - stamp_ns) * 1e-9)


class IpcReader:
    """LatestChannel 하나를 읽기 전용으로 연다. 파일이 다시 만들어지면 새로 연다."""

    def __init__(self, path: str, payload_size: int):
        self.path = path
        self.payload_size = payload_size
        self._map: mmap.mmap | None = None
        self._inode = None

    def _reopen_if_needed(self) -> None:
        inode = os.stat(self.path).st_ino
        if self._map is not None and inode == self._inode:
            return
        if self._map is not None:
            self._map.close()
            self._map = None
        fd = os.open(self.path, os.O_RDONLY)
        try:
            self._map = mmap.mmap(fd, 0, access=mmap.ACCESS_READ)
        finally:
            os.close(fd)
        self._inode = inode

    def read_payload(self) -> tuple[int, int, bytes] | None:
        """(seq, 발행 시각 ns, 앞쪽 payload_size 바이트). 아직 없거나 쓰는 중이면 None."""
        try:
            self._reopen_if_needed()
            assert self._map is not None
            for _ in range(4):
                magic, version, _, _, seq, stamp, size, _ = IPC_HEADER.unpack_from(self._map, 0)
                if magic != IPC_MAGIC or version != IPC_VERSION or seq == 0 or seq & 1 or size < self.payload_size:
                    return None
                payload = self._map[IPC_HEADER.size:IPC_HEADER.size + self.payload_size]
                if IPC_HEADER.unpack_from(self._map, 0)[4] == seq:
                    return seq, stamp, payload
            return None
        except (OSError, ValueError, struct.error):
            self._map = None
            return None


def struct_layout(fields: tuple) -> struct.Struct:
    """(이름, struct 형식) 순서의 필드 배치."""
    return struct.Struct("<" + "".join(fmt for _, fmt in fields))


def unpack_fields(layout: struct.Struct, fields: tuple, payload: bytes) -> Dict[str, Any]:
    """(이름, struct 형식) 순서의 배치로 payload를 dict로 푼다. 배열 형식("8h", "3f")은 list가 된다."""
    values = list(layout.unpack(payload[:layout.size]))
    state: Dict[str, Any] = {}
    for name, fmt in fields:
        count = int(fmt[:-1]) if len(fmt) > 1 else 1
        state[name] = values[:count] if count > 1 else values[0]
        del values[:count]
    return state


def flag_names(value: int, names: tuple) -> Dict[str, bool]:
    """비트 순서대로 이름 붙인 플래그."""
    return {name: bool(value >> bit & 1) for bit, name in enumerate(names)}
