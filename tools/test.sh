#!/usr/bin/env bash
# Configure, build and run every test.
# Usage: tools/test.sh [extra cmake arguments...]
# Environment: PDE2D_BUILD_DIR (default: build), PDE2D_BUILD_JOBS (default 4,
# at most 4), PDE2D_TEST_JOBS (default 4).
set -euo pipefail
cd "$(dirname "$0")/.."
build_dir="${PDE2D_BUILD_DIR:-build}"
jobs="${PDE2D_BUILD_JOBS:-4}"
if (( jobs > 4 )); then
  jobs=4
fi
generator=()
if [[ ! -f "$build_dir/CMakeCache.txt" ]] && command -v ninja > /dev/null; then
  generator=(-G Ninja)
fi
cmake -S . -B "$build_dir" "${generator[@]}" "$@"
cmake --build "$build_dir" -j "$jobs"
ctest --test-dir "$build_dir" -j "${PDE2D_TEST_JOBS:-4}" --output-on-failure
