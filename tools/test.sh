#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
test_dir=$(mktemp -d "${TMPDIR:-/tmp}/2d-pde-tests.XXXXXXXX")
trap 'rm -rf -- "$test_dir"' EXIT

"${CXX:-g++}" -std=c++23 -O2 -Wall -Wextra -Werror -Icpp \
  tests/curve_operations.cpp -o "$test_dir/curve_operations_test"
"$test_dir/curve_operations_test"
"${CXX:-g++}" -std=c++23 -O2 -Wall -Wextra -Werror -Icpp \
  tests/discretization.cpp -o "$test_dir/discretization_test"
"$test_dir/discretization_test" "$test_dir"
"${CXX:-g++}" -std=c++23 -O2 -Wall -Wextra -Werror -Icpp \
  tests/curve_length.cpp -o "$test_dir/curve_length_test"
"$test_dir/curve_length_test"
"${CXX:-g++}" -std=c++23 -O2 -Wall -Wextra -Werror -Icpp \
  tests/region_editing.cpp -o "$test_dir/region_editing_test"
"$test_dir/region_editing_test" "$test_dir"
"${CXX:-g++}" -std=c++23 -O2 -Wall -Wextra -Werror -Icpp \
  examples/learning/first_mesh.cpp -o "$test_dir/first_mesh"
"$test_dir/first_mesh" "$test_dir/first_mesh.pde2d"
"${CXX:-g++}" -std=c++23 -O2 -Wall -Wextra -Werror \
  examples/mesh_viewer/produce_mesh.cpp -o "$test_dir/produce_mesh"
"$test_dir/produce_mesh" "$test_dir/viewer.pde2d"
node tools/mesh_viewer/test_mesh.cjs

"${CXX:-g++}" -std=c++23 -O2 -Wall -Wextra -Werror -Icpp \
  examples/meshing/showcase.cpp -o "$test_dir/showcase"
"$test_dir/showcase" "$test_dir/showcase-meshes"
node - "$test_dir/showcase-meshes" <<'JS'
const fs=require('fs'),path=require('path'),vm=require('vm'),assert=require('assert/strict');
const context={window:{}};vm.createContext(context);
vm.runInContext(fs.readFileSync('tools/mesh_viewer/mesh.js','utf8'),context);
const files=fs.readdirSync(process.argv[2]).filter(name=>name.endsWith('.pde2d'));
assert.equal(files.length,19);
for(const name of files) {
  const mesh=context.window.MeshTools.parseMesh(fs.readFileSync(path.join(process.argv[2],name),'utf8'));
  assert.ok(mesh.options);assert.ok(mesh.boundaryContext);
  assert.equal(context.window.MeshTools.analyse(mesh).issues.length,0,name);
}
console.log('19 C++ showcase results load in the viewer without structural issues');
JS

"${CXX:-g++}" -std=c++23 -O2 -Wall -Wextra -Werror -Icpp $(pkg-config --cflags libxml-2.0) tests/svg.cpp $(pkg-config --libs libxml-2.0) -o "$test_dir/svg_test"
"$test_dir/svg_test" "$test_dir"
