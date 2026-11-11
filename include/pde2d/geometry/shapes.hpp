#pragma once

#include "pde2d/geometry/curve_geometry.hpp"

#include <cstddef>
#include <vector>

// Ready-made domains. Each function returns a plain CurveGeometry centered
// at the origin; use CurveGeometry::transformed to move, rotate or scale
// it. Sizes must be finite and positive (std::invalid_argument otherwise).
namespace pde2d::geometry {

// One full circle; region "boundary".
CurveGeometry circle(double radius);

// Semi-axes a (along x) and b (along y); region "boundary".
CurveGeometry ellipse(double a, double b);

// Regions "bottom", "right", "top", "left".
CurveGeometry rectangle(double width, double height);

// Two concentric circles: regions "outer" and "inner" (the hole).
CurveGeometry annulus(double outer_radius, double inner_radius);

enum class SemicircleEnd { positive_x, negative_x };

// A width x height rectangle with the short end at `end` replaced by a
// semicircle of diameter `height`, so the total length is
// width + height / 2. Regions "bottom", "top", "cap", and "left" or
// "right" for the straight short end.
CurveGeometry rectangle_with_semicircle(
    double width, double height, SemicircleEnd end = SemicircleEnd::positive_x);

// A simple polygon through the vertices, in either direction; region
// "boundary".
CurveGeometry polygon(std::vector<Point> vertices);

// A regular polygon with its circumcenter at the origin and its first
// vertex on the positive x axis; region "boundary".
CurveGeometry regular_polygon(std::size_t sides, double side_length);

} // namespace pde2d::geometry
