#pragma once

#include "pde2d/geometry/curve.hpp"
#include "pde2d/geometry/point.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace pde2d::mesh {

// Where a straight boundary edge came from: curve `curve` (an index into
// the curve list of the BoundaryPolygon or WorkingMesh that holds the link)
// between parameters t0 (at the edge's first node) and t1 (at its second).
struct CurveLink {
  std::size_t curve = 0;
  double t0 = 0.0;
  double t1 = 0.0;
};

// Data of one polygon edge.
struct PolygonEdge {
  std::size_t region = 0; // index into BoundaryPolygon::region_names
  std::optional<CurveLink> link;
};

// One closed polygon loop: edge i runs from points[i] to
// points[(i + 1) % size] and is described by edges[i]. Depth and parent
// have the same meaning as in geometry::CurveGeometry; even depth loops run
// counterclockwise and odd depth loops clockwise.
struct PolygonLoop {
  std::vector<geometry::Point> points;
  std::vector<PolygonEdge> edges;
  std::size_t depth = 0;
  std::optional<std::size_t> parent;
};

// The output of a boundary discretizer and the input of an initial
// triangulator: the boundary as straight edges.
struct BoundaryPolygon {
  std::vector<PolygonLoop> loops;
  std::vector<std::string> region_names;
  // The exact curves that curve links point to.
  std::vector<geometry::Curve> curves;
};

// Checks the BoundaryPolygon contract: sizes, indices, orientation by
// depth, nesting, and no two edges crossing or touching. Throws
// std::invalid_argument naming the first problem found.
void validate(const BoundaryPolygon &polygon);

} // namespace pde2d::mesh
