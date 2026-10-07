#!/usr/bin/env python3
"""Panda 펌웨어 버전 문자열을 찍는다: EDGE-<소스 sha256 앞 8자리>-DEBUG.

펌웨어가 USB(0xd6)로 알려 주는 값이라, 보드에 올라간 펌웨어가 어느 소스로 빌드됐는지
pandad와 웹 콘솔이 이것으로 구분한다. git이 없는 보드에서도 같은 값이 나오도록 파일
내용으로만 정한다: board/, crypto/, certs/와 Makefile, 이 스크립트(빌드 결과에 닿는 것
전부). 숨김 파일과 obj/, __pycache__는 뺀다.

사용: panda_version.py
"""
import hashlib
import os

ROOT = os.path.dirname(os.path.abspath(__file__))


def source_files():
    paths = ["Makefile", os.path.basename(__file__)]
    for top in ("board", "crypto", "certs"):
        for dirpath, dirnames, filenames in os.walk(os.path.join(ROOT, top)):
            dirnames[:] = [d for d in dirnames if not d.startswith(".") and d != "__pycache__"]
            paths += [os.path.relpath(os.path.join(dirpath, f), ROOT) for f in filenames
                      if not f.startswith(".") and not f.endswith((".pyc", "~"))]
    return sorted(paths)


def version():
    digest = hashlib.sha256()
    for path in source_files():
        digest.update(path.encode() + b"\0")
        with open(os.path.join(ROOT, path), "rb") as f:
            digest.update(f.read() + b"\0")
    return f"EDGE-{digest.hexdigest()[:8]}-DEBUG"


if __name__ == "__main__":
    print(version())
