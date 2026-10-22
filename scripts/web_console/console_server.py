"""웹 콘솔 서버(FastAPI): 파라미터 편집, 런타임 상태, 학습 상태·카메라 보정, Panda 펌웨어, 주행 기록, BEV 스트림을
내고 페이지(static/)를 서빙한다. 매니저가 함께 띄운다.

사용: python3 -m web_console [--host 주소] [--port 포트]   (기본 0.0.0.0:8080)

API(받지 않는 요청은 상태 코드와 {"detail": {"code": 문구 키, 자리 값...}}, refusal.py)
  GET   /api/status              매니저의 프로세스 상태와 백라이트(?versions=1이면 실행 파일의 빌드 버전도)
  GET   /api/params              파라미터 그룹·현재값·기본값·설명, 안내가 보는 런타임 사실(runtime)
  PATCH /api/params/{group}      값 바꾸기 {"values": {...}}
  GET   /api/learners            paramsd·torqued 학습값, 카메라 보정, locationd 조향 지연
  GET   /api/learners/trend      학습값 10분 추이
  POST  /api/calibration/reset   카메라 보정 초기화(해제 상태에서만)
  POST  /api/calibration/source  보정 출처 {"use_flow": ...}(해제 상태에서만)
  GET   /api/lateral_nn          학습 토크 모델 목록, 고른 것, controlsd가 쓰는 것
  POST  /api/lateral_nn          학습 토크 모델 고르기 {"model": ...}(controlsd가 재시작 없이 바꾼다)
  GET   /api/panda               Panda 펌웨어와 설치된 이미지
  POST  /api/panda/flash         Panda 플래싱 요청 {"version": ...}(주차 중에만)
  GET   /api/recordings          SD의 주행 기록(route)마다 크기·길이, 남은 공간, recordd의 녹화 상태
  POST  /api/recordings/delete   주행 기록 지우기 {"names": [...]}(쓰는 중이거나 옮기는 중인 것은 받지 않는다)
  GET   /api/bev                 BEV가 읽는 필드 위치
  GET   /api/bev/stream          BEV 스트림(ModelState·ControlState 페이로드)
  GET   /static/{path}           페이지 파일
"""
from __future__ import annotations

import argparse
import gzip
from typing import Any, Callable, Dict, List

# 이 모듈은 서버를 띄울 때(__main__)만 import된다. tests/check_web_console.py는 나머지 모듈만 stdlib으로 쓴다.
# 요청 모델은 이 모듈 전역에 있어야 한다: `from __future__ import annotations` 때문에 FastAPI가 라우트의 문자열
# 애너테이션을 모듈 전역에서 해석하므로, 지역 클래스면 NameError로 죽는다.
import uvicorn
from fastapi import FastAPI, Header, HTTPException, Query
from fastapi.responses import JSONResponse, Response, StreamingResponse
from pydantic import BaseModel

from .backlight import DisplayBacklight
from .bev_stream import BEV_MAX_HZ, bev_frames, bev_layout
from .calibration_control import CalibrationControl
from .lateral_nn_models import LateralNnModels
from .learner_monitor import LearnerMonitor, LocalizationReader
from .panda_update import PandaUpdate
from .param_store import ParamStore
from .process_status import ProcessStatus
from .recordings import Recordings
from .refusal import Refusal
from .static_assets import StaticAssets


class ParamPatch(BaseModel):
    values: Dict[str, Any]


class PandaFlashRequest(BaseModel):
    version: str


class CalibrationSourceRequest(BaseModel):
    use_flow: bool


class LateralNnRequest(BaseModel):
    model: str


class RecordingDeleteRequest(BaseModel):
    names: List[str]


# stopping: 서버가 끝나는 중인지(스트림이 그만 보낸다)
def create_app(stopping: Callable[[], bool]) -> FastAPI:
    backlight = DisplayBacklight()
    store = ParamStore()
    processes = ProcessStatus()
    learners = LearnerMonitor()
    learners.start()
    localization = LocalizationReader()
    calibration = CalibrationControl(store)
    panda = PandaUpdate()
    assets = StaticAssets()
    lateral_nn = LateralNnModels()
    recordings = Recordings()

    application = FastAPI(title="edgepilot web console", docs_url="/docs")

    # 모든 라우트가 같은 꼴로 거부한다: Refusal은 그 상태 코드와 문구 키, 파일 오류는 500과 error.io(이유 글자)
    @application.exception_handler(Refusal)
    async def refused(_request, exc: Refusal):
        return JSONResponse(status_code=exc.status, content={"detail": exc.detail()})

    @application.exception_handler(OSError)
    async def file_failed(_request, exc: OSError):
        refusal = Refusal("error.io", status=500, error=str(exc))
        return JSONResponse(status_code=500, content={"detail": refusal.detail()})

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
    def get_status(versions: bool = Query(False)) -> Dict[str, Any]:
        return {**processes.status(versions=versions), "backlight": backlight.status()}

    @application.get("/api/params")
    def get_params() -> Dict[str, Any]:
        return {**store.snapshot(), "runtime": learners.runtime_facts()}

    @application.patch("/api/params/{group}")
    def patch_params(group: str, patch: ParamPatch) -> Dict[str, Any]:
        return store.update(group, patch.values)

    @application.get("/api/learners")
    def get_learners() -> Dict[str, Any]:
        return {**learners.status(), "calibration": calibration.status(), "localization": localization.status()}

    @application.get("/api/learners/trend")
    def get_learner_trend() -> Dict[str, Any]:
        return learners.trend()

    @application.post("/api/calibration/reset")
    def reset_calibration() -> Dict[str, Any]:
        return calibration.request_reset()

    @application.post("/api/calibration/source")
    def set_calibration_source(request: CalibrationSourceRequest) -> Dict[str, Any]:
        return calibration.set_source(request.use_flow)

    # 학습 토크 모델 카드: controlsd가 도는지는 학습 상태의 신선도로 보고, 페이지는 runtime(lateral_nn_loaded)으로
    # 파라미터 안내를 갱신한다. 학습 상태는 응답마다 한 번 읽는다.
    def lateral_nn_status() -> Dict[str, Any]:
        learner = learners.status()
        return {**lateral_nn.status(controlsd_running=not learner["stale"]), "runtime": learner["runtime"]}

    @application.get("/api/lateral_nn")
    def get_lateral_nn() -> Dict[str, Any]:
        return lateral_nn_status()

    @application.post("/api/lateral_nn")
    def set_lateral_nn(request: LateralNnRequest) -> Dict[str, Any]:
        lateral_nn.select(request.model)
        return lateral_nn_status()

    @application.get("/api/panda")
    def get_panda() -> Dict[str, Any]:
        return panda.status()

    @application.post("/api/panda/flash")
    def flash_panda(request: PandaFlashRequest) -> Dict[str, Any]:
        return panda.request_flash(request.version)

    @application.get("/api/recordings")
    def get_recordings() -> Dict[str, Any]:
        return recordings.status()

    @application.post("/api/recordings/delete")
    def delete_recordings(request: RecordingDeleteRequest) -> Dict[str, Any]:
        return recordings.delete(request.names)

    @application.get("/api/bev")
    def get_bev_layout() -> Dict[str, Any]:
        return bev_layout()

    @application.get("/api/bev/stream")
    async def get_bev_stream(hz: float = Query(BEV_MAX_HZ, ge=1.0, le=BEV_MAX_HZ)):
        return StreamingResponse(bev_frames(hz, stopping=stopping), media_type="application/octet-stream",
                                 headers={"Cache-Control": "no-store"})

    return application


def main() -> None:
    parser = argparse.ArgumentParser(prog="python3 -m web_console")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()
    server: uvicorn.Server | None = None
    application = create_app(stopping=lambda: server is not None and server.should_exit)
    # 접근 로그는 끈다: 열린 페이지마다 1~2초 폴링 한 번에 한 줄씩 로그(SD의 journal)에 쌓인다.
    server = uvicorn.Server(uvicorn.Config(application, host=args.host, port=args.port, log_level="info",
                                           access_log=False))
    server.run()
