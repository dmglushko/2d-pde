#pragma once

#include "pde2d/geometry/curve.hpp"
#include "pde2d/geometry/curve_geometry.hpp"
#include "pde2d/geometry/point.hpp"

#include <span>

namespace pde2d::geometry {

// An axis-aligned box, with min.x <= max.x and min.y <= max.y.
struct BoundingBox {
  Point min, max;

  friend bool operator==(const BoundingBox &, const BoundingBox &) = default;
};

// The smallest box containing both boxes.
BoundingBox merged(const BoundingBox &a, const BoundingBox &b);
// The smallest box containing the box and the point.
BoundingBox merged(const BoundingBox &box, Point p);
// The box grown by `margin` on every side.
BoundingBox padded(const BoundingBox &box, double margin);
// True if p lies in the closed box.
bool contains(const BoundingBox &box, Point p);
double width(const BoundingBox &box);
double height(const BoundingBox &box);
// The length of the box's diagonal.
double diagonal(const BoundingBox &box);

// The smallest box containing the points. Throws std::invalid_argument for
// an empty list.
BoundingBox bounding_box(std::span<const Point> points);

// The smallest box containing the curve: its end points and the points
// where x' or y' vanishes (roots of the Bezier derivatives, the axis
// extremes of an arc), each evaluated with point_at.
BoundingBox bounding_box(const Curve &curve);
// The smallest box containing every curve. Throws std::invalid_argument
// for an empty list.
BoundingBox bounding_box(std::span<const Curve> curves);
// The smallest box containing every curve of every loop. Throws
// std::invalid_argument when there is no curve.
BoundingBox bounding_box(std::span<const Loop> loops);
BoundingBox bounding_box(const CurveGeometry &geometry);

} // namespace pde2d::geometry
