#include "pde2d/geometry/bounding_box.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <vector>

namespace pde2d::geometry {
namespace {

BoundingBox box_of(Point p) { return {p, p}; }

// Adds the curve point at t if t lies strictly inside (0, 1).
void add_interior(BoundingBox &box, const Curve &curve, double t) {
  if (t > 0 && t < 1)
    box = merged(box, point_at(curve, t));
}

// Roots in (0, 1) of a t^2 + b t + c (a linear or constant equation when a
// vanishes).
std::vector<double> quadratic_roots(double a, double b, double c) {
  std::vector<double> roots;
  if (a == 0) {
    if (b != 0)
      roots.push_back(-c / b);
    return roots;
  }
  const double disc = b * b - 4 * a * c;
  if (disc < 0)
    return roots;
  // The numerically stable form: one root from q, the other from c / q.
  const double q = -0.5 * (b + std::copysign(std::sqrt(disc), b));
  if (q != 0) {
    roots.push_back(q / a);
    roots.push_back(c / q);
  } else {
    roots.push_back(0.0); // b = 0 and disc = 0, so c = 0: a double root at 0
  }
  return roots;
}

// A line has no interior extremes.
void add_extremes(BoundingBox &, const Curve &, const Line &) {}

void add_extremes(BoundingBox &box, const Curve &curve,
                  const QuadraticBezier &c) {
  // x'(t) is linear: 2 [(p1 - p0) + t (p0 - 2 p1 + p2)].
  auto axis = [&](double p0, double p1, double p2) {
    const double d = p0 - 2 * p1 + p2;
    if (d != 0)
      add_interior(box, curve, (p0 - p1) / d);
  };
  axis(c.start.x, c.control.x, c.end.x);
  axis(c.start.y, c.control.y, c.end.y);
}

void add_extremes(BoundingBox &box, const Curve &curve, const CubicBezier &c) {
  // x'(t) / 3 = a t^2 + b t + c with the coefficients below.
  auto axis = [&](double p0, double p1, double p2, double p3) {
    const double a = -p0 + 3 * p1 - 3 * p2 + p3;
    const double b = 2 * (p0 - 2 * p1 + p2);
    const double k = p1 - p0;
    for (double t : quadratic_roots(a, b, k))
      add_interior(box, curve, t);
  };
  axis(c.start.x, c.control1.x, c.control2.x, c.end.x);
  axis(c.start.y, c.control1.y, c.control2.y, c.end.y);
}

void add_extremes(BoundingBox &box, const Curve &curve,
                  const EllipticalArc &c) {
  // x(a) = cx + rx cos(a) cos(phi) - ry sin(a) sin(phi) is extreme where
  // tan(a) = -ry sin(phi) / (rx cos(phi)); y likewise. Each gives two
  // angles, half a turn apart.
  const double cp = std::cos(c.rotation), sp = std::sin(c.rotation);
  const double angles[] = {std::atan2(-c.radius_y * sp, c.radius_x * cp),
                           std::atan2(c.radius_y * cp, c.radius_x * sp)};
  const double turn = 2 * std::numbers::pi;
  for (double base : angles)
    for (double a : {base, base + std::numbers::pi}) {
      // The parameter t with start + t * sweep = a (mod a full turn).
      double offset = std::fmod(a - c.start_angle, turn);
      if (c.sweep_angle < 0)
        offset = -offset;
      if (offset < 0)
        offset += turn;
      add_interior(box, curve, offset / std::abs(c.sweep_angle));
    }
}

} // namespace

BoundingBox merged(const BoundingBox &a, const BoundingBox &b) {
  return {{std::min(a.min.x, b.min.x), std::min(a.min.y, b.min.y)},
          {std::max(a.max.x, b.max.x), std::max(a.max.y, b.max.y)}};
}

BoundingBox merged(const BoundingBox &box, Point p) {
  return merged(box, box_of(p));
}

double diagonal(const BoundingBox &box) { return distance(box.min, box.max); }

BoundingBox bounding_box(const Curve &curve) {
  BoundingBox box = merged(box_of(start_point(curve)), end_point(curve));
  std::visit([&](const auto &c) { add_extremes(box, curve, c); }, curve);
  return box;
}

BoundingBox bounding_box(const CurveGeometry &geometry) {
  // A CurveGeometry has at least one loop, and every loop a curve.
  BoundingBox box = bounding_box(geometry.loops().front().curves.front().curve);
  for (const auto &loop : geometry.loops())
    for (const auto &c : loop.curves)
      box = merged(box, bounding_box(c.curve));
  return box;
}

} // namespace pde2d::geometry
