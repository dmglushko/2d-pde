#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
test_dir=$(mktemp -d "${TMPDIR:-/tmp}/2d-pde-tests.XXXXXXXX")
trap 'rm -rf -- "$test_dir"' EXIT

"${CXX:-g++}" -std=c++17 -O2 -Wall -Wextra -Werror -Icpp \
  tests/discretization.cpp -o "$test_dir/discretization_test"
"$test_dir/discretization_test" "$test_dir"
"${CXX:-g++}" -std=c++17 -O2 -Wall -Wextra -Werror -Icpp \
  examples/learning/first_mesh.cpp -o "$test_dir/first_mesh"
"$test_dir/first_mesh" "$test_dir/first_mesh.pde2d"
"${CXX:-g++}" -std=c++17 -O2 -Wall -Wextra -Werror \
  examples/mesh_viewer/produce_mesh.cpp -o "$test_dir/produce_mesh"
"$test_dir/produce_mesh" "$test_dir/viewer.pde2d"
node tools/mesh_viewer/test_mesh.cjs
