# 2D PDE project

This project currently implements the geometry and triangle meshing foundation
for a future 2D finite element PDE engine. The C++17 library is header-only and
uses the standard library. An offline browser viewer inspects exported meshes.

Implemented:

- Built-in circles, ellipses, rectangles, semicircle-capped rectangles, regular
  polygons, and general simple polygons, with translation and rotation.
- Five triangle mesh generators: basic and quality ear clipping, uniform
  refinement, Delaunay edge flips, and local size refinement.
- P1 and P2 triangle connectivity, conforming edge splits, optional interior
  smoothing, and versioned `.pde2d` export/import with structural validation.
- A mesh viewer with pan/zoom, triangle selection, quality measurements, and
  topology diagnostics, plus working examples and a guided code walkthrough.

PDE definition, finite element basis evaluation and assembly, boundary-condition
application, linear/nonlinear solvers, time integration, and solution error
estimation remain future work. The study guide discusses those methods; it does
not describe an implemented solver.

## Run and inspect

From the project root, with a C++17 compiler and Node.js on `PATH`:

```sh
./tools/test.sh
```

This builds and checks the C++ library and both examples in a temporary
directory, then runs the viewer's Node tests. Set `CXX=clang++` to choose another
compiler. No packages need to be installed.

To produce a mesh separately:

```sh
g++ -std=c++17 -Wall -Wextra -Werror -Icpp examples/learning/first_mesh.cpp -o /tmp/first_mesh
/tmp/first_mesh /tmp/rectangle.pde2d
```

Open [tools/mesh_viewer/index.html](tools/mesh_viewer/index.html) in a browser,
then load that file. A bundled P2 example appears on startup.

## Project map

- [Geometry, meshing, and I/O API](cpp/discretization/README.md)
- [Learning walkthrough](docs/learning/README.md) and [parameter reference](docs/learning/reference.md)
- [Viewer instructions](tools/mesh_viewer/README.md)
- [Audit findings and validation](docs/audit-2026-10-23.md)
- [FEM study guide (PDF)](pde_aware_fem_guide.pdf) and [LaTeX source](pde_aware_fem_guide.tex)

The mesher accepts one simple outline without holes. Its numeric tolerances and
finite refinement budget limit nearly degenerate input. Local size fields are
sampled at triangle centroids. No global minimum angle or overlap-free guarantee
is established for externally supplied mesh files; see the parameter reference
for the validation limits.
