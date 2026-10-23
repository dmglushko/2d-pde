# 2D PDE project

This project currently implements the geometry and triangle meshing foundation
for a future 2D finite element PDE engine. The C++17 library is header-only and
uses the standard library. An offline browser viewer inspects exported meshes.

Implemented:

- Built-in circles, ellipses, rectangles, semicircle-capped rectangles, regular
  polygons, and general simple polygons, with translation and rotation.
- Five triangle mesh generators: basic and quality ear clipping, uniform
  refinement, Delaunay edge flips, and local size refinement.
- Three-node P1 triangle connectivity, conforming edge splits, optional interior
  smoothing, and versioned `.pde2d` export/import with structural validation.
- A mesh viewer with pan/zoom, triangle selection, quality measurements, and
  topology diagnostics, plus working examples and a guided code walkthrough.

## P1-only design

This project uses linear, three-vertex triangles (P1). The intended FEM model
has one scalar unknown at each mesh vertex and linear interpolation within each
triangle. The public API has no element-order setting; the code, examples,
viewer, and learning exercises all follow this one model.

P2 support has been removed to keep studying FEM and implementing the solver
manageable. Future higher-order work must be a separate, optional module with
its own types and implementation. It must not add branches, configuration, or
prerequisites to the P1 learning and implementation path.

Existing version-1 P1 mesh files remain compatible. The file's `ORDER 1` record
is fixed format metadata; other orders are rejected.

PDE definition, finite element basis evaluation and assembly, boundary-condition
application, linear/nonlinear solvers, time integration, and solution error
estimation remain future work. The study guide discusses those methods; it does
not describe an implemented solver.

## Run and inspect

From the project root, with a C++17 compiler and Node.js on `PATH`:

```sh
./tools/test.sh
```

This checks 40 P1 shape/algorithm combinations and builds both examples in a
temporary directory, then runs the viewer's Node tests. Set `CXX=clang++` to
choose another compiler. No packages need to be installed.

To produce a mesh separately:

```sh
g++ -std=c++17 -Wall -Wextra -Werror -Icpp examples/learning/first_mesh.cpp -o /tmp/first_mesh
/tmp/first_mesh /tmp/rectangle.pde2d
```

Open [tools/mesh_viewer/index.html](tools/mesh_viewer/index.html) in a browser,
then load that file. A bundled P1 example appears on startup.

## Project map

- [Geometry, meshing, and I/O API](cpp/discretization/README.md)
- [Learning walkthrough](docs/learning/README.md) and [parameter reference](docs/learning/reference.md)
- [Viewer instructions](tools/mesh_viewer/README.md)
- [Historical audit findings](docs/audit-2026-10-23.md) (before the P1-only change)
- Optional broader background: [FEM study guide (PDF)](pde_aware_fem_guide.pdf) and [LaTeX source](pde_aware_fem_guide.tex)

The mesher accepts one simple outline without holes. Its numeric tolerances and
finite refinement budget limit nearly degenerate input. Local size fields are
sampled at triangle centroids. No global minimum angle or overlap-free guarantee
is established for externally supplied mesh files; see the parameter reference
for the validation limits.
