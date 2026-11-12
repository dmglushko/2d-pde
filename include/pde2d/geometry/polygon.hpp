#pragma once

#include "pde2d/geometry/point.hpp"

#include <cstddef>
#include <optional>
#include <span>
#include <vector>

// Helpers for closed polygons given as point lists without a repeated last
// point: segment i runs from points[i] to points[(i + 1) % size].
namespace pde2d::geometry {

// Shoelace formula, relative to the first point (so a small polygon far
// from the origin does not cancel): positive for a counterclockwise
// polygon, 0 for fewer than three points.
double signed_area(std::span<const Point> polygon);

// Exact point location: +1 strictly inside, 0 on the boundary, -1 outside.
int locate_in_polygon(std::span<const Point> polygon, Point p);

// Segment `segment` of loop `loop`.
struct SegmentRef {
  std::size_t loop = 0;
  std::size_t segment = 0;
};

struct Crossing {
  SegmentRef first, second;
};

// Finds two segments that share a point, among all segments of all loops.
// Two consecutive segments of one loop may share their common vertex, and
// nothing else. A zero-length segment is reported as a crossing with
// itself. Every loop needs at least three points (std::invalid_argument
// otherwise). Uses exact predicates; the result is deterministic.
std::optional<Crossing>
find_crossing(std::span<const std::vector<Point>> loops);

} // namespace pde2d::geometry
