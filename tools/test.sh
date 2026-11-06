#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
build_dir="${PDE2D_BUILD_DIR:-build/geometry}"
cmake -S . -B "$build_dir" -DCMAKE_BUILD_TYPE=Release -DPDE2D_BUILD_FEM=OFF "$@" -DBUILD_TESTING=ON
cmake --build "$build_dir" -j "${PDE2D_BUILD_JOBS:-2}"
ctest --test-dir "$build_dir" --output-on-failure --no-tests=error -j "${PDE2D_TEST_JOBS:-2}"
