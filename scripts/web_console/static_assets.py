"""페이지 파일(이 패키지의 static/: HTML, CSS, JS 모듈, three.js 0.186.1). 처음 요청 때 한 번 gzip해 메모리에
두고(three.js 800 KB가 200 KB로), ETag가 같으면 304로 끝낸다. 파일이 바뀌면(mtime·크기) 다시 읽는다.
static/ 밖과 아래 형식이 아닌 파일, 숨김 파일은 내주지 않는다."""
from __future__ import annotations

import gzip
import threading
from pathlib import Path
from typing import Dict, NamedTuple

STATIC_DIR = Path(__file__).resolve().parent / "static"
MEDIA_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".js": "text/javascript; charset=utf-8",
    ".svg": "image/svg+xml",
}


class StaticFile(NamedTuple):
    etag: str
    gzipped: bytes
    media_type: str


class StaticAssets:
    def __init__(self, root: Path = STATIC_DIR):
        self.root = root.resolve()
        self.lock = threading.Lock()
        self.cache: Dict[str, StaticFile] = {}

    def get(self, name: str) -> StaticFile | None:
        """파일 하나(ETag, gzip한 내용, 형식). 없거나 내줄 수 없으면 None."""
        path = (self.root / name).resolve()
        media_type = MEDIA_TYPES.get(path.suffix)
        if media_type is None or self.root not in path.parents or any(part.startswith(".") for part in Path(name).parts):
            return None
        try:
            info = path.stat()
            etag = f'"{info.st_mtime_ns:x}-{info.st_size:x}"'
            with self.lock:
                cached = self.cache.get(name)
                if cached is None or cached.etag != etag:
                    cached = StaticFile(etag, gzip.compress(path.read_bytes(), compresslevel=6, mtime=0), media_type)
                    self.cache[name] = cached
            return cached
        except OSError:
            return None
