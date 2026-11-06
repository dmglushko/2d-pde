# 2D PDE project

This project implements geometry, triangle meshing, and a scalar linear 2D
finite element PDE engine. The C++23 geometry library is header-only; SVG I/O
additionally uses libxml2. The optional numerical module uses SymEngine for
symbolic coefficients and Eigen for sparse linear algebra. Native meshing uses
Boost.Multiprecision for exact predicate fallbacks.

Implemented:

- Editable circles, ellipses, rectangles, capped rectangles, regular polygons,
  and general simple polygons, with translation and rotation.
- Free-function meshing with ear clipping or constrained Delaunay triangulation,
  plus fixed, straight-chord, or analytical-curve boundary refinement.
- Three-node P1 connectivity, optional maximum-edge-length refinement, explicit
  completion reports, and v3 result export/import for resumed curve refinement.
- Named boundary regions, stable curve IDs, arc-length editing preserved on
  resize, and analytical SVG geometry import/export with region membership.
- An offline mesh viewer and reproducible [graphical mesh gallery](docs/showcase/README.md).
- Raw partial-derivative equation input, reusable boundary conditions, P1
  assembly, selectable quadrature, sparse solvers, and first/second-order time
  evolution, including dynamic Robin boundaries with `u_t`.
- [Computed static, heat, and wave demonstrations](docs/fem/showcase.html)
  with exact-solution comparisons and refinement checks.
- [100-second driven Dirichlet and source-free Neumann waves](docs/fem/long-waves.md)
  on a larger square, with execution profiling and energy/work checks.

## P1-only design

Every triangle has three vertex indices. The intended FEM model has one scalar
unknown per vertex with linear interpolation. There is no element-order setting.
Higher-order work must remain a separate optional module. All mesh files use
fixed `ORDER 1`; old v1 P1 files remain readable. Raw meshes export as v2;
`MeshResult` exports as v3 with settings, report, and optional boundary context.

The [numerical module guide](docs/fem/README.md) documents supported equations,
boundary classes, sign conventions, build dependencies, and limitations.
General boundary input can represent more conditions than the initial solver
supports; unsupported cases fail explicitly. The study guide provides broader
background beyond the implemented methods.

## Run and inspect

With a C++23 compiler, Boost development headers, Node.js, pkg-config, and
libxml2 development headers:

```sh
./tools/test.sh
```

The core geometry and meshing headers do not require libxml2. Set `CXX=clang++`
to choose another compiler. To produce a mesh separately:

```sh
g++ -std=c++23 -Wall -Wextra -Werror -Icpp examples/learning/first_mesh.cpp -o /tmp/first_mesh
/tmp/first_mesh /tmp/rectangle.pde2d
```

Open [tools/mesh_viewer/index.html](tools/mesh_viewer/index.html) in a browser
and load the output file. A bundled example appears on startup. For standalone
PNGs and saved source meshes, see the [showcase](docs/showcase/README.md).

To build the PDE module and its tests (Eigen3 and Boost headers required;
SymEngine is found locally or fetched from a pinned release):

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
ctest --test-dir build --output-on-failure
./build/fem_showcase build/fem-showcase.json
node tools/render_solutions.mjs build/fem-showcase.json docs/fem/showcase.html
```

## Project map

- [Geometry, free-function meshing, region editing, and I/O](cpp/discretization/README.md)
- [Equation input, boundary conditions, assembly, and solutions](docs/fem/README.md)
- [Learning walkthrough](docs/learning/README.md), [reference](docs/learning/reference.md), and [questions](docs/learning/questions.md)
- [Viewer instructions](tools/mesh_viewer/README.md)
- [Historical audit](docs/audit-2026-10-23.md), predating the current API
- Broader background: [FEM guide PDF](pde_aware_fem_guide.pdf) and [LaTeX source](pde_aware_fem_guide.tex)

The mesher accepts one simple outline without holes. CDT size refinement uses
longest-edge midpoint insertion and full legalization; it has no minimum-angle
control, smoothing, or local size field, and no classical circumcenter quality
refinement guarantee. Always check `MeshResult::report`: fixed-boundary conflicts
and exhausted work budgets return explicit unmet-size reports with a mesh.
Externally supplied meshes are checked for connected disk topology, duplicate
and unused vertices, and crossing or overlapping edges. Geometric validation
uses a quadratic edge-pair check; no minimum-angle guarantee is made.
