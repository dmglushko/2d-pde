#pragma once

#include "pde2d/geometry/point.hpp"

// Exact geometric predicates. Each one first tries ordinary floating point
// with an error bound and falls back to exact integer arithmetic when the
// sign is uncertain, so the answer is always the sign of the exact
// determinant of the given double coordinates. Coordinates must be finite.
namespace pde2d::geometry {

// +1 if a, b, c turn counterclockwise, -1 if clockwise, 0 if collinear.
int orient(Point a, Point b, Point c);

// +1 if d lies strictly inside the circle through the counterclockwise
// triangle a, b, c; -1 if strictly outside; 0 if on the circle.
int incircle(Point a, Point b, Point c, Point d);

// True if p lies on the closed segment from a to b.
bool on_segment(Point a, Point b, Point p);

// True if the closed segments ab and cd share at least one point.
bool segments_intersect(Point a, Point b, Point c, Point d);

} // namespace pde2d::geometry
