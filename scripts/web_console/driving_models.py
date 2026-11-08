"""조향 탭의 주행 모델 카드: models/의 axmodel 파일(크기, 체크섬), 지금 고른 것(params/driving_model.json), 매니저가
modeld에 준 것(상태 파일, scripts/manager.py)을 보여 주고 고르기를 바꾼다. 매니저는 1초 안에 새 고르기를 보고 modeld만
새 모델로 다시 띄운다. 그동안 몇 초 모델 출력이 없으므로 결합 중에는 받지 않는다(해제 상태면 주행 중에도 받는다). 고른 모델로
modeld가 거듭 바로 죽으면 매니저가 기본 모델로 돌아가고 상태에 failed를 남긴다. 인자나 EDGEPILOT_MODEL(시험용 drop-in)이
있으면 매니저는 고르기를 쓰지 않는다(상태의 source). 이름 규칙과 경로는 manager.py와 같다(check_web_console.py가 대조한다).

고르기 파일에는 이름과 그 파일의 sha256을 같이 쓴다. 실제로 달린 모델은 매니저가 params/driving_model_running.json에
따로 남기고(기본 모델로 돌아갔거나 EDGEPILOT_MODEL이 있으면 고르기와 다르다), 모델이 바뀌면 recordd를 다시 띄워 새 route를
연다. recordd가 route를 열 때 params/*.json을 복사하므로 route마다 실제로 달린 모델 하나가 남는다. 체크섬은 파일의 (수정 시각, 크기)가 같은 동안 다시 계산하지 않는다(보드에서 37 MB에 약 0.3초)."""
from __future__ import annotations

import hashlib
import re
import time
from pathlib import Path
from typing import Any, Dict, List, Tuple

from .files import env_path, params_dir, read_json_object, read_json_or_none, write_json_atomic
from .refusal import Refusal
from .shm_channel import IpcReader, topic_path
from .state_layout import CONTROL_STATE_MAX_AGE_S, CONTROL_STATE_SIZE, CONTROL_STATE_TOPIC, control_engaged

# manager.py와 같다(작업 디렉터리 기준)
MODEL_DIR = Path("models")
DEFAULT_MODEL = "supercombo"  # models/supercombo.axmodel(manager.py DEFAULT_MODEL_PATH)
SELECTION_FILE = "driving_model.json"
STATUS_PATH = "/dev/shm/edgepilot_driving_model.json"  # EDGEPILOT_DRIVING_MODEL_STATUS로 바꿀 수 있다
NAME_PATTERN = re.compile(r"[A-Za-z0-9_-][A-Za-z0-9_.-]{0,63}")
SUFFIX = ".axmodel"


def read_choice(path: Path) -> Tuple[str, str]:
    """고르기 파일의 (이름, 오류). manager.py model_choice와 같은 규칙: 파일이 없으면 ("", "none"), deploy_student.py가
    쓰는 "models/<이름>.axmodel" 꼴도 받는다."""
    try:
        document = read_json_object(path)
    except FileNotFoundError:
        return "", "none"
    except (OSError, ValueError):
        return "", "unreadable"
    name = document.get("model")
    if not isinstance(name, str):
        return "", "no_model"
    prefix = MODEL_DIR.as_posix() + "/"
    if name.startswith(prefix) and name.endswith(SUFFIX):
        name = name[len(prefix):-len(SUFFIX)]
    return (name, "") if NAME_PATTERN.fullmatch(name) else ("", "bad_name")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as file:
        for block in iter(lambda: file.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


class DrivingModels:
    def __init__(self, model_dir: Path = MODEL_DIR, selection_path: Path | None = None, status_path: str | None = None,
                 control_path: str = topic_path(CONTROL_STATE_TOPIC)):
        self.model_dir = model_dir
        self.selection_path = selection_path or params_dir() / SELECTION_FILE
        self.status_path = Path(status_path) if status_path else env_path("EDGEPILOT_DRIVING_MODEL_STATUS", STATUS_PATH)
        self.control = IpcReader(control_path, CONTROL_STATE_SIZE)
        self.digests: Dict[str, Tuple[Tuple[int, int], str]] = {}  # 이름 -> ((수정 시각 ns, 크기), sha256)

    def digest(self, name: str, path: Path) -> Tuple[Dict[str, Any], str]:
        st = path.stat()
        stamp = (st.st_mtime_ns, st.st_size)
        cached = self.digests.get(name)
        if cached is None or cached[0] != stamp:
            cached = (stamp, sha256_file(path))
            self.digests[name] = cached
        return {"bytes": st.st_size, "modified": int(st.st_mtime)}, cached[1]

    def models(self) -> List[Dict[str, Any]]:
        """고를 수 있는 모델: 기본 모델 먼저, 나머지는 이름 순. 각각 {name, bytes, modified, sha256}."""
        names = sorted((path.name[:-len(SUFFIX)] for path in self.model_dir.glob("*" + SUFFIX)
                        if path.is_file() and NAME_PATTERN.fullmatch(path.name[:-len(SUFFIX)])),
                       key=lambda name: (name != DEFAULT_MODEL, name))
        result = []
        for name in names:
            info, sha = self.digest(name, self.model_dir / f"{name}{SUFFIX}")
            result.append({"name": name, **info, "sha256": sha})
        return result

    def blocker(self) -> str:
        """지금 바꾸면 안 되는 이유(engaged), 없으면 빈 문자열. 결합(또는 조향) 중에만 막는다: 해제 상태에서는 modeld가
        몇 초 쉬어도 조향하지 않으므로 주행 중에도 바꿀 수 있다(2026-11-08 사용자 결정). controlsd가 없거나 멈췄으면
        조향도 없다."""
        latest = self.control.read_fresh(CONTROL_STATE_MAX_AGE_S)
        return "engaged" if latest is not None and control_engaged(latest[2]) else ""

    def status(self, manager_running: bool, modeld_running: bool) -> Dict[str, Any]:
        """manager는 매니저가 낸 {model, source, selected, error, unix_time}이고, 매니저가 돌지 않으면 None이다."""
        selected, selection_error = read_choice(self.selection_path)
        manager = read_json_or_none(self.status_path) if manager_running else None
        return {"models": self.models(), "default": DEFAULT_MODEL, "selected": selected,
                "selection_error": selection_error, "manager": manager, "modeld_running": modeld_running,
                "blocker": self.blocker()}

    def select(self, name: str) -> None:
        """고르기를 바꾼다. 목록에 있는 이름만, 해제 상태일 때만 받는다."""
        models = {model["name"]: model for model in self.models()}
        if name not in models:
            raise Refusal("error.driving_model_unknown", status=404, name=name)
        blocker = self.blocker()
        if blocker == "engaged":
            raise Refusal("error.driving_model_engaged")
        write_json_atomic(self.selection_path, {"model": name, "axmodel_sha256": models[name]["sha256"],
                                                "selected_at": time.strftime("%Y-%m-%dT%H:%M:%S%z")})
