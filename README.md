# pde2d

A C++23 library, written to learn from, for two-dimensional geometry and
triangle meshing, heading towards one linear scalar PDE solved with P1
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
  `Remesher` that refines an existing mesh.

## Build and test

Linux (or WSL) with CMake 3.25+ and a C++23 compiler. The libraries have
no external dependencies; the tests need Boost (only the header-only
Boost.Multiprecision, used as an independent oracle for the exact
predicates).

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
  // 0.2; result.reports has one report per stage.
  return result.target_met ? 0 : 1;
}
```

Link against `pde2d_mesh` (it brings `pde2d_geometry`).

## Documentation

- [AGENTS.md](AGENTS.md): guide for contributors and coding agents.
- [docs/architecture.md](docs/architecture.md): modules, types and the
  data flow from geometry to mesh.
- [docs/decisions.md](docs/decisions.md): the owner's design decisions.
- [docs/vision.md](docs/vision.md): where the project is heading.
