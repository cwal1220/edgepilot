#!/usr/bin/env bash
# 호스트에서 도는 테스트를 전부 빌드하고 ctest로 돌린다. 보드도 교차 툴체인도 필요 없다.
# C++ 테스트는 googletest(처음 configure 때 내려받는다), check_web_console.py는 stdlib만 쓴다.
# check_recording_reader.py는 numpy가 있어야 돌고, 없으면 ctest가 건너뛴 것으로 센다.
# 하나만 돌리려면 ctest --test-dir build-host -R <이름>, 또는 build-host/bin/<테스트> --gtest_filter=<패턴>
# 사용: scripts/run_host_tests.sh
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

build_dir="${EDGEPILOT_HOST_BUILD_DIR:-build-host}"
jobs="${JOBS:-$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 2)}"

cmake -S . -B "${build_dir}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DEDGEPILOT_BUILD_RUNTIME=OFF \
  -DEDGEPILOT_BUILD_DIAGNOSTICS=ON >/dev/null
cmake --build "${build_dir}" -j"${jobs}" --target host_tests >/dev/null
ctest --test-dir "${build_dir}" -j"${jobs}" --output-on-failure
