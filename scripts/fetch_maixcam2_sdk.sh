#!/usr/bin/env bash
# MaixCAM2 교차 빌드 의존성을 deps/ax630에 받는다(tools/docker_ax630/build.sh가 쓴다).
#  - AX620E MSP SDK: 보드 런타임과 같은 v3.0.0_20250319114413 (MaixCDK 릴리스, SHA256 고정)
#  - MaixCDK 헤더: libmaixcam_lib의 C++ 인터페이스 ax_middleware.hpp와 그것이 쓰는
#    maix_basic/maix_image 헤더, OS04D10을 아는 MSP 샘플 공통 소스 (커밋 고정)
#  - 보드에서(ABI를 맞추려고 그대로): /opt/lib(AX 런타임: ax_engine·ax_sys·ax_ivps…),
#    /usr/lib/libmaixcam_lib.so.1.2.5, libsamplerate
# 사용: scripts/fetch_maixcam2_sdk.sh root@<보드 IP>   (또는 EDGEPILOT_BOARD=root@<보드 IP>)
set -euo pipefail
cd "$(dirname "$0")/.."
BOARD="${1:-${EDGEPILOT_BOARD:-}}"
if [ -z "$BOARD" ]; then
  echo "usage: $0 root@<board-ip>   (or set EDGEPILOT_BOARD=root@<board-ip>)" >&2
  exit 2
fi
DEPS=deps/ax630
MSP_VER=v3.0.0_20250319114413
MSP_SHA=20ab34bded456d8328b825bb2cc842937651596ea43f7075de95be7ced9a953d
MAIXCDK_COMMIT=30f4b8b
mkdir -p "$DEPS/maix" "$DEPS/lib"

if [ ! -d "$DEPS/maixcam2_msp" ]; then
  tarball="$DEPS/maixcam2_msp_${MSP_VER}.tar.xz"
  curl -fL -o "$tarball" "https://github.com/sipeed/MaixCDK/releases/download/v0.0.0/maixcam2_msp_arm64_glibc_${MSP_VER}.tar.xz"
  echo "${MSP_SHA}  ${tarball}" | shasum -a 256 -c -
  tar xf "$tarball" -C "$DEPS" && rm "$tarball"
fi

if [ ! -d "$DEPS/MaixCDK" ]; then
  git clone -q --filter=blob:none --sparse https://github.com/sipeed/MaixCDK.git "$DEPS/MaixCDK"
  git -C "$DEPS/MaixCDK" checkout -q "$MAIXCDK_COMMIT"
  git -C "$DEPS/MaixCDK" sparse-checkout set components/basic/include components/vision/include \
    components/maixcam_lib/include components/3rd_party/maixcam2_msp/msp/sample/common
fi

rsync -aL "$BOARD:/usr/lib/libmaixcam_lib.so.1.2.5" "$DEPS/maix/libmaixcam_lib.so"
rsync -aL "$BOARD:/usr/lib/aarch64-linux-gnu/libsamplerate.so.0" "$DEPS/maix/" 2>/dev/null ||
  rsync -aL "$BOARD:/usr/lib/libsamplerate.so.0" "$DEPS/maix/"
ln -sf libsamplerate.so.0 "$DEPS/maix/libsamplerate.so"
rsync -a "$BOARD:/opt/lib/" "$DEPS/lib/"
echo "maixcam2 media deps ready in $DEPS"
