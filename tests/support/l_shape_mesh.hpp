#pragma once

// A mesh with many colours for the threaded tests: the L-shape [0,2]^2
// minus [1,2]^2, triangulated by CDT and refined by longest-edge
// bisection, so that a few nodes (the corners of the coarse
// triangulation) have dozens of triangles. Its sides are the regions
// "bottom" (y = 0), "right" (x = 2), "inner" (the two sides of the
// re-entrant corner), "top" (y = 2) and "left" (x = 0).

#include "support/check.hpp"

#include "pde2d/geometry/curve_geometry.hpp"
#include "pde2d/mesh/cdt.hpp"
#include "pde2d/mesh/chord_length_discretizer.hpp"
#include "pde2d/mesh/longest_edge_bisection.hpp"
#include "pde2d/mesh/mesh.hpp"
#include "pde2d/mesh/mesher.hpp"

#include <memory>
#include <utility>

namespace pde2d::test {

// Edges at most `h`.
inline std::shared_ptr<const mesh::Mesh> refined_l_shape(double h) {
  using geometry::Line;
  using geometry::Point;
  const Point p0{0, 0}, p1{2, 0}, p2{2, 1}, p3{1, 1}, p4{1, 2}, p5{0, 2};
  geometry::Loop loop;
  loop.curves = {{Line{p0, p1}, "bottom"}, {Line{p1, p2}, "right"},
                 {Line{p2, p3}, "inner"},  {Line{p3, p4}, "inner"},
                 {Line{p4, p5}, "top"},    {Line{p5, p0}, "left"}};
  const geometry::CurveGeometry geometry({loop});
  auto result = mesh::Mesher(std::make_shared<mesh::ChordLengthDiscretizer>(),
                             std::make_shared<mesh::Cdt>(),
                             std::make_shared<mesh::LongestEdgeBisection>())
                    .run(geometry, {.max_edge_length = {.absolute = h}});
  check(result.target_met, "the mesh meets its edge length");
  return std::make_shared<const mesh::Mesh>(std::move(result.mesh));
}

} // namespace pde2d::test
