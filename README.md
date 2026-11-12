# pde2d

A C++23 library, written to learn from, for two-dimensional geometry,
triangle meshing and their file formats, heading towards one linear scalar PDE solved with P1
finite elements.

This is a from-scratch rewrite. The previous header-only code is on the
branch `archive/header-only-v1`, kept for reference only.

What exists now:

- `pde2d_geometry`: exact curves (lines, Bézier curves, elliptical arcs),
  `CurveGeometry` (closed loops with region names; holes and islands are
  found from nesting), and ready-made shapes.
- `pde2d_mesh`: a mesher with a fixed stage order (boundary discretizer,
  initial triangulator, optional refiner, improvers), the v1 components
  (chord-length boundary discretization, ear clipping, CDT, Ruppert,
  longest-edge bisection, Delaunay flips, Laplacian smoothing), and a
  `Remesher` that refines an existing mesh. A mesh made by the mesher
  keeps its curve links, so refining it again puts new boundary points on
  the true curves.
- `pde2d_io`: SVG import and export of geometries, SVG pictures of
  meshes, the `.pde2d` mesh format of the previous code (read by the
  browser viewer in `tools/mesh_viewer/`), and Gmsh `.msh` import
  (versions 4.1 and 2.2, ASCII).

## Build and test

Linux (or WSL) with CMake 3.25+ and a C++23 compiler. `pde2d_geometry`
and `pde2d_mesh` have no external dependencies; `pde2d_io` needs the
libxml2 development files (`libxml2-dev`), used privately to parse SVG.
The tests need Boost (only the header-only Boost.Multiprecision, used as
an independent oracle for the exact predicates); the viewer tests run
when Node.js is installed.

```sh
tools/test.sh          # configure, build (at most -j4) and run every test
tools/format.sh        # clang-format every C++ file
```

## Example: a plate with a hole and an island

```cpp
#include "pde2d/geometry/curve_geometry.hpp"
#include "pde2d/mesh/cdt.hpp"
#include "pde2d/mesh/chord_length_discretizer.hpp"
#include "pde2d/mesh/mesher.hpp"
#include "pde2d/mesh/ruppert.hpp"

#include <memory>
#include <numbers>

using namespace pde2d;

geometry::Loop square(double half, const std::string &region) {
  geometry::Point a{-half, -half}, b{half, -half}, c{half, half},
      d{-half, half};
  return {{{geometry::Line{a, b}, region},
           {geometry::Line{b, c}, region},
           {geometry::Line{c, d}, region},
           {geometry::Line{d, a}, region}}};
}

int main() {
  // Loop roles come from nesting: the square is the outer boundary, the
  // circle a hole, the small square an island. Directions are fixed
  // automatically.
  geometry::Loop hole{{{geometry::EllipticalArc{{0, 0}, 1, 1, 0, 0,
                                                2 * std::numbers::pi},
                        "hole"}}};
  geometry::CurveGeometry plate(
      {square(2, "wall"), hole, square(0.25, "island")});

  mesh::Mesher mesher(
      std::make_shared<mesh::ChordLengthDiscretizer>(
          mesh::ChordLengthOptions{.max_curve_deviation = 0.01}),
      std::make_shared<mesh::Cdt>(),
      std::make_shared<mesh::Ruppert>(
          mesh::RuppertOptions{.min_angle_degrees = 25}));

  mesh::MeshResult result = mesher.run(plate, {.max_edge_length = 0.2});
  // result.mesh: nodes, triangles, boundary edges with region ids, and
  // region_names. result.target_met says whether every edge is at most
  // 0.2; result.reports starts with the "pipeline" report (the plan and
  // its warnings), then has one report per stage.
  return result.target_met ? 0 : 1;
}
```

Link against `pde2d_mesh` (it brings `pde2d_geometry`).

## Example: from SVG to the viewer

```cpp
#include "pde2d/io/pde2d_mesh.hpp"
#include "pde2d/io/svg.hpp"
// ... and the mesher headers as above

// Each <path> gives curves; its data-region (or id) names the region.
geometry::CurveGeometry shape = io::import_svg("shape.svg");
mesh::MeshResult result = mesher.run(shape, {.max_edge_length = 0.1});
io::export_pde2d(result.mesh, "shape.pde2d"); // open in tools/mesh_viewer
io::export_mesh_svg(result.mesh, "shape_mesh.svg");
```

Link against `pde2d_io`. `examples/mesh_viewer/produce_mesh.cpp` is a
complete program (built as `build/examples/produce_mesh`); see
[tools/mesh_viewer/README.md](tools/mesh_viewer/README.md).

## Documentation

- [AGENTS.md](AGENTS.md): guide for contributors and coding agents.
- [docs/architecture.md](docs/architecture.md): modules, types and the
  data flow from geometry to mesh.
- [docs/decisions.md](docs/decisions.md): the owner's design decisions.
- [docs/vision.md](docs/vision.md): where the project is heading.
- [docs/formats/pde2d-mesh.md](docs/formats/pde2d-mesh.md): the `.pde2d`
  mesh file format.
