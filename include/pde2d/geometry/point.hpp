#pragma once

#include <cmath>

namespace pde2d::geometry {

// A point (or a vector) in the plane.
struct Point {
  double x = 0.0;
  double y = 0.0;

  friend bool operator==(const Point &, const Point &) = default;
};

inline Point operator+(Point a, Point b) { return {a.x + b.x, a.y + b.y}; }
inline Point operator-(Point a, Point b) { return {a.x - b.x, a.y - b.y}; }
inline Point operator*(double s, Point p) { return {s * p.x, s * p.y}; }
inline double dot(Point a, Point b) { return a.x * b.x + a.y * b.y; }
// z-component of the 3D cross product: positive when b is counterclockwise
// from a.
inline double cross(Point a, Point b) { return a.x * b.y - a.y * b.x; }
inline double norm(Point p) { return std::hypot(p.x, p.y); }
inline double distance(Point a, Point b) { return norm(a - b); }
inline Point midpoint(Point a, Point b) {
  return {0.5 * (a.x + b.x), 0.5 * (a.y + b.y)};
}
inline bool is_finite(Point p) {
  return std::isfinite(p.x) && std::isfinite(p.y);
}

} // namespace pde2d::geometry
