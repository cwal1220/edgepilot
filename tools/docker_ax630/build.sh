#!/usr/bin/env bash
# usage: tools/docker_ax630/build.sh   (저장소 루트에서)
# scripts/fetch_maixcam2_sdk.sh로 deps/ax630을 받아 둔 뒤 build-ax630/에 런타임을 빌드한다. Panda
# 펌웨어도 같이 빌드한다(build-ax630/panda/obj/panda.bin.signed, upload_to_board.sh가 올린다).
set -euo pipefail
cd "$(dirname "$0")/../.."
for dep in lib/libax_engine.so maix/libmaixcam_lib.so; do
  [ -e "deps/ax630/$dep" ] || { echo "deps/ax630/$dep missing (run scripts/fetch_maixcam2_sdk.sh)"; exit 1; }
done
docker build -q -t edgepilot-ax630-build tools/docker_ax630 >/dev/null
# 실행 파일에 새길 버전(cmake/build_version.cmake). 컨테이너 안의 git 대신 호스트에서 읽어 넘긴다.
# 바이너리를 비교하려고 고정하려면 EDGEPILOT_VERSION을 주고 부른다.
version="${EDGEPILOT_VERSION:-$(git describe --tags --always --dirty --match 'v[0-9]*' 2>/dev/null || echo unknown)}"
docker run --rm -e EDGEPILOT_VERSION="$version" -v "$PWD":/src -w /src edgepilot-ax630-build bash -c '
  cmake -S . -B build-ax630 -DCMAKE_BUILD_TYPE=Release -DAX_LIB_DIR=/src/deps/ax630/lib \
        -DAX_RPATH=/opt/lib >/dev/null &&
  cmake --build build-ax630 -j"$(nproc)" &&
  make -s -C firmware/panda OBJ=../../build-ax630/panda/obj'
