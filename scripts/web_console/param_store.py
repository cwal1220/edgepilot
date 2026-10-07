"""파라미터 파일(params/*.json) 읽기·쓰기. 바꾸면 그 그룹을 읽는 프로세스에 SIGHUP으로 알리고(controlsd), 기기
설정의 백라이트 항목은 하드웨어에도 바로 적용한다. 기본값은 params/ 옆의 params.defaults/(업로드 스크립트가
저장소의 params/에서 채운다)이고, 런타임 파일의 키 집합을 기본값에 맞춰 둔다."""
from __future__ import annotations

import json
import os
import stat
import tempfile
import threading
from pathlib import Path
from typing import Any, Callable, Dict, List

from .backlight import DisplayBacklight
from .param_metadata import PARAM_GROUPS, PARAM_METADATA
from .process_status import signal_process

# 백라이트 하드웨어에 적용하는 기기 설정 항목. 나머지(알림음 등)는 overlayd·modeld가 파일에서 읽는다.
BACKLIGHT_KEYS = ("enabled", "brightness_percent")


def params_dir() -> Path:
    """런타임 파라미터 디렉터리. 프로세스들과 같이 EDGEPILOT_PARAMS_DIR, 없으면 작업 디렉터리의 params/."""
    return Path(os.environ.get("EDGEPILOT_PARAMS_DIR", "params"))


def group_paths(directory: Path) -> Dict[str, Path]:
    return {group: directory / spec["file"] for group, spec in PARAM_GROUPS.items()}


def read_document(path: Path) -> Dict[str, Any]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise ValueError(f"parameter file not found: {path}") from exc
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid JSON in {path}: {exc}") from exc
    if not isinstance(data, dict):
        raise ValueError(f"parameter document must be an object: {path}")
    return data


def write_document(path: Path, document: Dict[str, Any]) -> None:
    """임시 파일에 쓰고 fsync한 뒤 rename으로 바꿔 넣는다(읽는 쪽이 반쯤 쓴 파일을 보지 않게)."""
    path.parent.mkdir(parents=True, exist_ok=True)
    mode = stat.S_IMODE(path.stat().st_mode) if path.exists() else 0o664
    fd, temporary_name = tempfile.mkstemp(prefix=path.name + ".", suffix=".tmp", dir=path.parent)
    temporary = Path(temporary_name)
    try:
        os.fchmod(fd, mode)
        with os.fdopen(fd, "w", encoding="utf-8") as file:
            json.dump(document, file, ensure_ascii=False, indent=2)
            file.write("\n")
            file.flush()
            os.fsync(file.fileno())
        os.replace(temporary, path)
        directory_fd = os.open(path.parent, os.O_RDONLY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    finally:
        if temporary.exists():
            temporary.unlink()


class ParamStore:
    def __init__(self, paths: Dict[str, Path] | None = None, default_paths: Dict[str, Path] | None = None,
                 notify: Callable[[str], List[int]] = signal_process, backlight: DisplayBacklight | None = None):
        directory = params_dir()
        self.paths = paths if paths is not None else group_paths(directory)
        if default_paths is None:
            default_paths = group_paths(directory.with_name(directory.name + ".defaults")) if paths is None else {}
        self.default_paths = default_paths
        self.notify = notify
        self.backlight = backlight
        self.lock = threading.Lock()
        self._sync_with_defaults()
        if self.backlight is not None and "display" in self.paths:
            try:
                self.backlight.apply(self.read_group("display"))
            except (RuntimeError, ValueError):
                pass

    def _sync_with_defaults(self) -> None:
        """런타임 파일의 키 집합을 기본값에 맞춘다. 빠진 키는 기본값으로 채우고, 기본값에서 사라진 키는 지운다.
        살아남은 키의 값은 그대로 둔다. 지우지 않으면 코드가 더 이상 읽지 않는 파라미터가 편집기에 남아 튜닝이
        적용된 것처럼 보인다."""
        for group, defaults in self.defaults().items():
            runtime_path = self.paths.get(group)
            if runtime_path is None:
                continue
            try:
                runtime = read_document(runtime_path) if runtime_path.is_file() else {}
            except ValueError:
                continue
            merged = {key: runtime.get(key, value) for key, value in defaults.items()}
            if merged != runtime:
                write_document(runtime_path, merged)

    def defaults(self) -> Dict[str, Dict[str, Any]]:
        """그룹별 기본값. 기본값 파일이 없거나 깨진 그룹은 뺀다."""
        result = {}
        for group, path in self.default_paths.items():
            try:
                result[group] = read_document(path)
            except ValueError:
                continue
        return result

    def read_group(self, group: str) -> Dict[str, Any]:
        return read_document(self._path(group))

    def snapshot(self) -> Dict[str, Any]:
        """페이지가 파라미터 탭을 그리는 데 필요한 것 전부: 그룹 구성(이름, 안내, 섹션 순서, 알릴 프로세스),
        현재값, 기본값, 항목 설명, 파일 경로."""
        with self.lock:
            documents = {group: self.read_group(group) for group in self.paths}
        return {
            "groups": {group: {key: spec[key] for key in ("label", "note", "sections", "notify")}
                       for group, spec in PARAM_GROUPS.items() if group in self.paths},
            "params": documents,
            "defaults": self.defaults(),
            "metadata": PARAM_METADATA,
            "paths": {group: str(path) for group, path in self.paths.items()},
        }

    def update(self, group: str, values: Dict[str, Any]) -> Dict[str, Any]:
        if not values:
            raise ValueError("no parameter values supplied")
        with self.lock:
            document = self.read_group(group)
            unknown = sorted(set(values) - set(document))
            if unknown:
                raise KeyError(", ".join(unknown))
            document.update(values)
            if group == "display" and any(key in values for key in BACKLIGHT_KEYS):
                if self.backlight is None:
                    raise ValueError("display backlight control is unavailable")
                self.backlight.apply(document)
            write_document(self._path(group), document)
        process = PARAM_GROUPS.get(group, {}).get("notify", "")
        return {"group": group, "params": document, "notified": self.notify(process) if process else []}

    def _path(self, group: str) -> Path:
        if group not in self.paths:
            raise KeyError(group)
        return self.paths[group]
