# 2D PDE project

This project implements geometry and triangle meshing for a future 2D finite
element PDE engine. The C++23 library is header-only; SVG I/O additionally
uses libxml2. Native meshing uses the Boost.Multiprecision headers for exact
predicate fallbacks. An offline browser viewer inspects exported meshes.

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

## P1-only design

Every triangle has three vertex indices. The intended FEM model has one scalar
unknown per vertex with linear interpolation. There is no element-order setting.
Higher-order work must remain a separate optional module. All mesh files use
fixed `ORDER 1`; old v1 P1 files remain readable. Raw meshes export as v2;
`MeshResult` exports as v3 with settings, report, and optional boundary context.

PDE definition, basis evaluation, assembly, boundary-condition application,
solvers, time integration, and solution error estimation remain future work.
The study guide discusses these methods without claiming an implemented solver.

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

## Project map

- [Geometry, free-function meshing, region editing, and I/O](cpp/discretization/README.md)
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
