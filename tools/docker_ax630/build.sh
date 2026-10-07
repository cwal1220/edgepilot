#!/usr/bin/env bash
# usage: tools/docker_ax630/build.sh   (저장소 루트에서)
# scripts/fetch_maixcam2_sdk.sh로 deps/ax630을 받아 둔 뒤 build-ax630/에 런타임을 빌드한다. Panda
# 펌웨어도 같이 빌드한다(build-ax630/panda/obj/panda.bin.signed, upload_to_board.sh가 올린다).
set -euo pipefail
cd "$(dirname "$0")/../.."
for dep in lib/libax_engine.so maix/libmaixcam_lib.so opencv_lib/libopencv_core.so; do
  [ -e "deps/ax630/$dep" ] || { echo "deps/ax630/$dep missing (run scripts/fetch_maixcam2_sdk.sh)"; exit 1; }
done
docker build -q -t edgepilot-ax630-build tools/docker_ax630 >/dev/null
docker run --rm -v "$PWD":/src -w /src edgepilot-ax630-build bash -c '
  cmake -S . -B build-ax630 -DCMAKE_BUILD_TYPE=Release -DAX_LIB_DIR=/src/deps/ax630/lib \
        -DAX_RPATH=/opt/lib >/dev/null &&
  cmake --build build-ax630 -j"$(nproc)" &&
  make -s -C firmware/panda OBJ=../../build-ax630/panda/obj'
