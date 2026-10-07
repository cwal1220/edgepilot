"""웹 콘솔 서버(FastAPI): 파라미터 편집, 런타임 상태, 실시간 학습·카메라 보정, Panda 펌웨어, BEV 스트림을 내고
페이지(static/)를 서빙한다. 매니저가 함께 띄운다.

사용: python3 -m web_console [--host 주소] [--port 포트]   (기본 0.0.0.0:8080,
EDGEPILOT_WEB_CONSOLE_HOST·EDGEPILOT_WEB_CONSOLE_PORT로도 바꿀 수 있다)

API
  GET   /api/status              매니저의 프로세스 상태와 백라이트
  GET   /api/params              파라미터 그룹·현재값·기본값·설명
  PATCH /api/params/{group}      값 바꾸기 {"values": {...}}
  GET   /api/learners            paramsd·torqued 학습값, 카메라 보정, locationd 조향 지연
  GET   /api/learners/trend      학습값 10분 추이
  POST  /api/calibration/reset   카메라 보정 초기화(해제 상태에서만)
  GET   /api/panda               Panda 펌웨어와 설치된 이미지
  POST  /api/panda/flash         Panda 플래싱 요청 {"version": ...}(주차 중에만)
  GET   /api/bev                 BEV가 읽는 필드 위치
  GET   /api/bev/stream          BEV 스트림(ModelState·ControlState 페이로드)
  GET   /static/{path}           페이지 파일
"""
from __future__ import annotations

import argparse
import contextlib
import gzip
import os
from typing import Any, Callable, Dict

# fastapi/uvicorn은 서버를 띄울 때만 import한다. tests/check_web_console.py가 stdlib만으로 나머지 모듈을 쓴다.
# 요청 모델만은 이 모듈 전역에 있어야 한다: `from __future__ import annotations` 때문에 FastAPI가 라우트의
# 문자열 애너테이션을 모듈 전역에서 해석하므로, 지역 클래스면 NameError로 죽는다.
try:
    from pydantic import BaseModel
except ImportError:  # 호스트 검사: 웹 스택 없이 import된다
    BaseModel = object  # type: ignore[assignment,misc]

from .backlight import DisplayBacklight
from .bev_stream import BEV_MAX_HZ, bev_frames, bev_layout
from .calibration_reset import CalibrationReset
from .learner_monitor import LearnerMonitor, LocalizationReader
from .panda_update import PandaUpdate
from .param_store import ParamStore
from .process_status import ProcessStatus
from .static_assets import StaticAssets


class ParamPatch(BaseModel):  # type: ignore[misc,valid-type]
    values: Dict[str, Any]


class PandaFlashRequest(BaseModel):  # type: ignore[misc,valid-type]
    version: str


def create_app(store: ParamStore | None = None, processes: ProcessStatus | None = None,
               learners: LearnerMonitor | None = None, localization: LocalizationReader | None = None,
               calibration: CalibrationReset | None = None, panda: PandaUpdate | None = None,
               assets: StaticAssets | None = None, backlight: DisplayBacklight | None = None,
               stopping: Callable[[], bool] = lambda: False) -> "FastAPI":
    from fastapi import FastAPI, Header, HTTPException, Query
    from fastapi.responses import Response, StreamingResponse

    backlight = backlight or DisplayBacklight()
    store = store or ParamStore(backlight=backlight)
    processes = processes or ProcessStatus()
    learners = learners or LearnerMonitor()
    localization = localization or LocalizationReader()
    calibration = calibration or CalibrationReset()
    panda = panda or PandaUpdate()
    assets = assets or StaticAssets()

    @contextlib.asynccontextmanager
    async def lifespan(_application):
        learners.start()
        yield
        learners.stop()

    application = FastAPI(title="edgepilot web console", docs_url="/docs", lifespan=lifespan)

    def refuse(call: Callable[[], Dict[str, Any]]) -> Dict[str, Any]:
        """요청을 받지 않을 상태면(PermissionError) 409, 파일 오류면 500."""
        try:
            return call()
        except PermissionError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        except OSError as exc:
            raise HTTPException(status_code=500, detail=str(exc)) from exc

    def static_response(name: str, if_none_match: str | None, accept_encoding: str) -> Response:
        asset = assets.get(name)
        if asset is None:
            raise HTTPException(status_code=404, detail="not found")
        headers = {"ETag": asset.etag, "Cache-Control": "no-cache", "Vary": "Accept-Encoding"}
        if if_none_match == asset.etag:
            return Response(status_code=304, headers=headers)
        body = asset.gzipped
        if "gzip" in accept_encoding:
            headers["Content-Encoding"] = "gzip"
        else:
            body = gzip.decompress(body)
        return Response(body, media_type=asset.media_type, headers=headers)

    @application.get("/")
    def index(if_none_match: str | None = Header(None), accept_encoding: str = Header("")):
        return static_response("index.html", if_none_match, accept_encoding)

    @application.get("/static/{name:path}")
    def static_file(name: str, if_none_match: str | None = Header(None), accept_encoding: str = Header("")):
        return static_response(name, if_none_match, accept_encoding)

    @application.get("/api/status")
    def get_status() -> Dict[str, Any]:
        return {**processes.status(), "backlight": backlight.status()}

    @application.get("/api/params")
    def get_params() -> Dict[str, Any]:
        try:
            return store.snapshot()
        except (KeyError, ValueError) as exc:
            raise HTTPException(status_code=500, detail=str(exc)) from exc

    @application.patch("/api/params/{group}")
    def patch_params(group: str, patch: ParamPatch) -> Dict[str, Any]:
        try:
            return store.update(group, patch.values)
        except KeyError as exc:
            raise HTTPException(status_code=404, detail=f"unknown parameter: {exc}") from exc
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        except RuntimeError as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc

    @application.get("/api/learners")
    def get_learners() -> Dict[str, Any]:
        steering = store.read_group("steering")
        return {**learners.status(steering), "calibration": calibration.status(),
                "localization": localization.status(steering)}

    @application.get("/api/learners/trend")
    def get_learner_trend() -> Dict[str, Any]:
        return learners.trend()

    @application.post("/api/calibration/reset")
    def reset_calibration() -> Dict[str, Any]:
        return refuse(calibration.request)

    @application.get("/api/panda")
    def get_panda() -> Dict[str, Any]:
        return panda.status()

    @application.post("/api/panda/flash")
    def flash_panda(request: PandaFlashRequest) -> Dict[str, Any]:
        return refuse(lambda: panda.request(request.version))

    @application.get("/api/bev")
    def get_bev_layout() -> Dict[str, Any]:
        return bev_layout()

    @application.get("/api/bev/stream")
    async def get_bev_stream(hz: float = Query(BEV_MAX_HZ, ge=1.0, le=BEV_MAX_HZ)):
        return StreamingResponse(bev_frames(hz, stopping=stopping), media_type="application/octet-stream",
                                 headers={"Cache-Control": "no-store"})

    return application


def main() -> None:
    import uvicorn

    parser = argparse.ArgumentParser(prog="python3 -m web_console")
    parser.add_argument("--host", default=os.environ.get("EDGEPILOT_WEB_CONSOLE_HOST", "0.0.0.0"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("EDGEPILOT_WEB_CONSOLE_PORT", "8080")))
    args = parser.parse_args()
    server: uvicorn.Server | None = None
    application = create_app(stopping=lambda: server is not None and server.should_exit)
    server = uvicorn.Server(uvicorn.Config(application, host=args.host, port=args.port, log_level="info"))
    server.run()
