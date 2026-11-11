#pragma once

// Test geometries built from loops.

#include "pde2d/geometry/curve_geometry.hpp"
#include "pde2d/geometry/shapes.hpp"

#include <numbers>
#include <string>
#include <vector>

namespace pde2d::test {

using namespace pde2d::geometry;

inline Loop polygon_loop(const std::vector<Point> &p,
                         const std::string &region) {
  Loop loop;
  for (std::size_t i = 0; i < p.size(); ++i)
    loop.curves.push_back({Line{p[i], p[(i + 1) % p.size()]}, region});
  return loop;
}

inline Loop square_loop(Point corner, double size, const std::string &region) {
  return polygon_loop({corner,
                       {corner.x + size, corner.y},
                       {corner.x + size, corner.y + size},
                       {corner.x, corner.y + size}},
                      region);
}

inline Loop circle_loop(Point center, double r, const std::string &region) {
  return Loop{
      {{EllipticalArc{center, r, r, 0, 0, 2 * std::numbers::pi}, region}}};
}

// The L-shape [0,2]^2 minus [1,2]^2, area 3.
inline CurveGeometry l_shape() {
  return CurveGeometry({polygon_loop(
      {{0, 0}, {2, 0}, {2, 1}, {1, 1}, {1, 2}, {0, 2}}, "boundary")});
}

// A five-pointed star (concave, ten vertices).
inline CurveGeometry star() {
  std::vector<Point> p;
  for (int k = 0; k < 10; ++k) {
    double r = k % 2 == 0 ? 1.0 : 0.45;
    double a = std::numbers::pi / 2 + k * std::numbers::pi / 5;
    p.push_back({r * std::cos(a), r * std::sin(a)});
  }
  return CurveGeometry({polygon_loop(p, "boundary")});
}

// [0,6]^2 with a circular hole of radius 2 around (3,3), a square island
// [2.5,3.5]^2 inside the hole, and a square hole [0.5,1.5]^2.
inline CurveGeometry holes_and_island() {
  return CurveGeometry({square_loop({0, 0}, 6, "outer"),
                        circle_loop({3, 3}, 2, "hole"),
                        square_loop({2.5, 2.5}, 1, "island"),
                        square_loop({0.5, 0.5}, 1, "small_hole")});
}

inline double holes_and_island_area() {
  return 36 - 4 * std::numbers::pi + 1 - 1;
}

} // namespace pde2d::test
