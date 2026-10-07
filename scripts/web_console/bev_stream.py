"""BEV 탭(위에서 본 장면)의 스트림. 보드는 ModelState·ControlState 페이로드를 발행된 그대로 흘려보내기만 하고
(해석·JSON 없음), 페이지(static/bev_data.js)가 /api/bev에서 필드 위치를 받아 직접 읽고 그린다."""
from __future__ import annotations

import asyncio
import struct
import time
from typing import Any, Callable, Dict

from .shm_channel import IpcReader, boottime_ns, topic_path
from .state_layout import (CONTROL_STATE_AT, CONTROL_STATE_SIZE, CONTROL_STATE_TOPIC, HUD_FLAG_BITS,
                           MODEL_STATE_AT, MODEL_STATE_SIZE, MODEL_STATE_TOPIC)

# 스트림 한 프레임: 머리(매직 "BEV1", ModelState·ControlState 바이트 수, 보드 CLOCK_BOOTTIME ns)와 두 페이로드.
# 새 모델 프레임마다(최대 BEV_MAX_HZ) 보내고, 모델이 멈추면 1초마다 모델 없이 보내 페이지가 끊김과 멈춤을
# 구분한다.
BEV_FRAME = struct.Struct("<IHHQ")
BEV_FRAME_MAGIC = 0x31564542
BEV_MAX_HZ = 20.0
BEV_IDLE_S = 1.0
# 스트림은 다음 모델 프레임이 나올 때(이번 발행 + 카메라 한 주기)까지 잔다. 늦으면 BEV_RETRY_S마다 다시
# 보고, BEV_STALL_S 넘게 안 나오면 모델이 멈춘 것으로 보고 BEV_STALLED_POLL_S마다만 본다.
BEV_MODEL_PERIOD_S = 0.05
BEV_RETRY_S = 0.005
BEV_STALL_S = 0.25
BEV_STALLED_POLL_S = 0.1


def bev_layout() -> Dict[str, Any]:
    return {
        "model": {"size": MODEL_STATE_SIZE, **MODEL_STATE_AT},
        "control": {"size": CONTROL_STATE_SIZE, **CONTROL_STATE_AT},
        "hud_flags": HUD_FLAG_BITS,
    }


def bev_frame(model: bytes, control: bytes, now_ns: int) -> bytes:
    return BEV_FRAME.pack(BEV_FRAME_MAGIC, len(model), len(control), now_ns) + model + control


async def bev_frames(hz: float, model_path: str = topic_path(MODEL_STATE_TOPIC),
                     control_path: str = topic_path(CONTROL_STATE_TOPIC), idle_s: float = BEV_IDLE_S,
                     stopping: Callable[[], bool] = lambda: False):
    """BEV 스트림. 새 모델 프레임이면 두 페이로드를 복사해 보낸다. 보드 CPU를 아끼려고 다음 프레임이 나올
    때까지 자므로 프레임마다 한 번 남짓만 깬다(20 ms 폴링은 이것만으로 한 코어의 3.7%였다).
    연결이 끊기면 Starlette가 이 제너레이터를 취소하고, 서버가 내려가기 시작하면(stopping) 스스로 끝난다.
    끝나지 않으면 uvicorn이 응답이 끝나기를 기다려 매니저의 SIGKILL(3초)까지 내려가지 않는다."""
    model = IpcReader(model_path, MODEL_STATE_SIZE)
    control = IpcReader(control_path, CONTROL_STATE_SIZE)
    period = 1.0 / min(max(hz, 1.0), BEV_MAX_HZ)
    sent_seq = None
    next_frame = sent_at = 0.0
    while not stopping():
        now = time.monotonic()
        latest = model.read_payload()
        pending = latest is not None and latest[0] != sent_seq
        if (pending and now >= next_frame) or now - sent_at >= idle_s:
            send = pending and now >= next_frame
            current = control.read_payload()
            yield bev_frame(latest[2] if send else b"", current[2] if current is not None else b"", boottime_ns())
            sent_at = now
            if send:
                sent_seq, next_frame, pending = latest[0], now + period - BEV_RETRY_S, False
        if pending:
            wait = next_frame - now  # 보낼 프레임이 있다: 보낼 차례까지
        elif latest is None:
            wait = BEV_STALLED_POLL_S
        else:
            due = (latest[1] - boottime_ns()) * 1e-9 + BEV_MODEL_PERIOD_S  # 다음 모델 프레임까지
            wait = due + 0.002 if due > 0 else BEV_RETRY_S if due > -BEV_STALL_S else BEV_STALLED_POLL_S
        await asyncio.sleep(min(max(wait, BEV_RETRY_S), sent_at + idle_s - now))
