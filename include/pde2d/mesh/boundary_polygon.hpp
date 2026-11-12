#pragma once

#include "pde2d/geometry/point.hpp"
#include "pde2d/mesh/curve_chain.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace pde2d::mesh {

// Where a straight boundary edge came from: chain `chain` (an index into
// the chain list of the BoundaryPolygon or WorkingMesh that holds the
// link) between arc lengths s0 (at the edge's first node) and s1 (at its
// second), with 0 <= s0 < s1 <= chain length. Chains follow the boundary
// direction (domain on the left), like the edges.
struct CurveLink {
  std::size_t chain = 0;
  double s0 = 0.0;
  double s1 = 0.0;
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
  // The curve chains that curve links point to.
  std::vector<CurveChain> chains;
};

// True if the link's chain exists and 0 <= s0 < s1 <= its length.
bool is_valid_link(const CurveLink &link,
                   const std::vector<CurveChain> &chains);

// Checks the BoundaryPolygon contract: sizes, indices, orientation by
// depth, nesting, and no two edges crossing or touching. Throws
// std::invalid_argument naming the first problem found.
void validate(const BoundaryPolygon &polygon);

} // namespace pde2d::mesh
