#!/usr/bin/env bash
# 교차 빌드한 런타임을 MaixCAM2에 올린다: 실행 파일, 보드용 Python, 웹 콘솔(web_console/, 페이지 포함),
# 파라미터 기본값, 그리고 모델(models/supercombo.axmodel, 보드의 것과 체크섬이 다를 때만 보낸다).
# Panda 펌웨어 이미지(build.sh가 만든 build-ax630/panda/obj/panda.bin.signed)가 있으면
# firmware/panda.bin.signed로 올린다. 판다에 쓰는 것은 웹 콘솔이나 panda_flash다.
# 보드의 params/는 덮어쓰지 않고, 기본값은 params.defaults/에 두어 없는 파일만 채운다.
# 실행 중인 바이너리는 덮어쓸 수 없으므로 .upload/에 올린 뒤 mv로 바꾼다. 매니저는 다시
# 띄우지 않는다.
# 사용: scripts/upload_to_board.sh [root@보드]
#   (기본 root@192.168.219.117, 설치 디렉터리 EDGEPILOT_BOARD_DIR=/root/edgepilot, 빌드 build-ax630)
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

BOARD="${1:-root@192.168.219.117}"
DEST="${EDGEPILOT_BOARD_DIR:-/root/edgepilot}"
BIN_DIR="${EDGEPILOT_BIN_DIR:-build-ax630/bin}"
AXMODEL="models/supercombo.axmodel"
PANDA_IMAGE="${EDGEPILOT_PANDA_IMAGE:-build-ax630/panda/obj/panda.bin.signed}"
SSH=(ssh -o StrictHostKeyChecking=no)
SCP=(scp -q -o StrictHostKeyChecking=no)

runtime_files=(
  "${BIN_DIR}/camerad"
  "${BIN_DIR}/modeld"
  "${BIN_DIR}/overlayd"
  "${BIN_DIR}/controlsd"
  "${BIN_DIR}/recordd"
  "${BIN_DIR}/camcal"
  "${BIN_DIR}/imud"
  "${BIN_DIR}/locationd"
  scripts/manager.py
  scripts/requirements-web-console.txt
)
[ -x "${BIN_DIR}/pandad" ] && runtime_files+=("${BIN_DIR}/pandad")
[ -x "${BIN_DIR}/panda_flash" ] && runtime_files+=("${BIN_DIR}/panda_flash")
param_files=(calibration.json adaptive_cruise.json steering.json driving.json recording.json display.json)

for file in "${runtime_files[@]}" "$AXMODEL" scripts/web_console/__main__.py; do
  [ -f "$file" ] || { echo "Missing: $file" >&2; exit 1; }
done

"${SSH[@]}" "$BOARD" "rm -rf '$DEST/.upload' && mkdir -p '$DEST/.upload' '$DEST/models' '$DEST/params' '$DEST/params.defaults' '$DEST/firmware'"
"${SCP[@]}" "${runtime_files[@]}" "$BOARD:$DEST/.upload/"
# 웹 콘솔은 디렉터리째 보낸다(파이썬 캐시와 macOS 메타 파일은 빼고)
COPYFILE_DISABLE=1 tar -C scripts --exclude=__pycache__ --exclude=.DS_Store -cf - web_console |
  "${SSH[@]}" "$BOARD" "tar -C '$DEST/.upload' -xf -"
model_sha="$(shasum -a 256 "$AXMODEL" | cut -d' ' -f1)"
board_sha="$("${SSH[@]}" "$BOARD" "sha256sum '$DEST/models/supercombo.axmodel' 2>/dev/null | cut -d' ' -f1" || true)"
model_note="model unchanged"
if [ "$model_sha" != "$board_sha" ]; then
  "${SCP[@]}" "$AXMODEL" "$BOARD:$DEST/.upload/supercombo.axmodel"
  model_note="model updated"
fi
"${SCP[@]}" "${param_files[@]/#/params/}" "$BOARD:$DEST/params.defaults/"
panda_note=""
if [ -f "$PANDA_IMAGE" ]; then
  "${SCP[@]}" "$PANDA_IMAGE" "$BOARD:$DEST/.upload/panda.bin.signed"
  panda_note=", panda firmware $(cat "$(dirname "$PANDA_IMAGE")/version" 2>/dev/null || echo image)"
fi
"${SSH[@]}" "$BOARD" "set -e; cd '$DEST'
  if [ -f .upload/supercombo.axmodel ]; then mv .upload/supercombo.axmodel models/; fi
  if [ -f .upload/panda.bin.signed ]; then mv .upload/panda.bin.signed firmware/; fi
  rm -rf web_console && mv .upload/web_console .
  rm -f models/supercombo_npu1.axmodel  # 예전 AI-ISP용 별도 모델(이제 supercombo.axmodel 하나)
  for f in .upload/*; do [ -f \"\$f\" ] && mv \"\$f\" .; done
  for name in ${param_files[*]}; do [ -e params/\$name ] || cp params.defaults/\$name params/; done
  rm -rf .upload; sync"
# camerad·overlayd는 deps/ax630/maix/libmaixcam_lib.so(보드에서 받은 1.2.5)에 맞춰 빌드한다. 새로 구운
# v4.12.5 이미지의 /usr/lib/libmaixcam_lib.so는 크기만 같은 다른 빌드라 camerad가 SIGSEGV로 죽는다
# (2026-10-02 SD 교체). 다르면 1.2.5를 올리고 링크를 건다(순정 파일은 .stock으로 남긴다).
lib_note=""
lib_sha="$(shasum -a 256 deps/ax630/maix/libmaixcam_lib.so | cut -d' ' -f1)"
board_lib_sha="$("${SSH[@]}" "$BOARD" "sha256sum /usr/lib/libmaixcam_lib.so 2>/dev/null | cut -d' ' -f1" || true)"
if [ "$lib_sha" != "$board_lib_sha" ]; then
  "${SCP[@]}" deps/ax630/maix/libmaixcam_lib.so "$BOARD:/usr/lib/libmaixcam_lib.so.1.2.5"
  "${SSH[@]}" "$BOARD" "cd /usr/lib && { [ -L libmaixcam_lib.so ] || [ ! -e libmaixcam_lib.so ] ||
      mv libmaixcam_lib.so libmaixcam_lib.so.stock; } && ln -sfn libmaixcam_lib.so.1.2.5 libmaixcam_lib.so && sync"
  lib_note=", libmaixcam_lib 1.2.5 installed"
fi
echo "Uploaded runtime to $BOARD:$DEST ($model_note$lib_note$panda_note)"
