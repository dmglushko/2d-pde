#include "support/check.hpp"

#include "pde2d/geometry/bounding_box.hpp"
#include "pde2d/geometry/curve.hpp"
#include "pde2d/geometry/curve_geometry.hpp"

#include <cmath>
#include <numbers>
#include <span>
#include <stdexcept>
#include <vector>

using namespace pde2d::geometry;
using namespace pde2d::test;

namespace {
constexpr double pi = std::numbers::pi;

void same_box(const BoundingBox &actual, const BoundingBox &expected,
              double tol, std::string_view what) {
  near(actual.min.x, expected.min.x, tol, what);
  near(actual.min.y, expected.min.y, tol, what);
  near(actual.max.x, expected.max.x, tol, what);
  near(actual.max.y, expected.max.y, tol, what);
}

// Independent oracle: the box of many points along the curve. It lies
// inside the exact box and approaches it as n grows.
BoundingBox sampled_box(const Curve &c, int n) {
  BoundingBox box{c.point_at(0), c.point_at(0)};
  for (int i = 1; i <= n; ++i)
    box = merged(box, c.point_at(double(i) / n));
  return box;
}

void check_against_samples(const Curve &c, std::string_view what) {
  const BoundingBox exact = c.bounding_box();
  const BoundingBox sampled = sampled_box(c, 200000);
  const double tiny = 1e-12;
  check(exact.min.x <= sampled.min.x + tiny &&
            exact.min.y <= sampled.min.y + tiny &&
            exact.max.x >= sampled.max.x - tiny &&
            exact.max.y >= sampled.max.y - tiny,
        what);
  // At a smooth extreme the samples miss it by O(1 / n^2).
  same_box(exact, sampled, 1e-8, what);
}

} // namespace

int main() {
  return run({
      {"box_helpers",
       [] {
         BoundingBox a{{0, 0}, {1, 1}}, b{{2, -1}, {3, 0}};
         check(merged(a, b) == BoundingBox{{0, -1}, {3, 1}}, "merged boxes");
         check(merged(a, Point{-1, 4}) == BoundingBox{{-1, 0}, {1, 4}},
               "merged point");
         near(diagonal(BoundingBox{{1, 1}, {4, 5}}), 5, 0, "3-4-5 diagonal");
         near(width(BoundingBox{{1, 1}, {4, 5}}), 3, 0, "width");
         near(height(BoundingBox{{1, 1}, {4, 5}}), 4, 0, "height");
         check(padded(a, 0.5) == BoundingBox{{-0.5, -0.5}, {1.5, 1.5}},
               "padded");
         check(contains(a, {1, 0}) && contains(a, {0.5, 0.5}), "closed box");
         check(!contains(a, {1.5, 0.5}) && !contains(a, {0.5, -0.1}),
               "outside");
         const std::vector<Point> points{{2, 1}, {-1, 3}, {0, -2}};
         check(bounding_box(points) == BoundingBox{{-1, -2}, {2, 3}},
               "box of points");
         expect_throws<std::invalid_argument>(
             [] { (void)bounding_box(std::span<const Point>{}); }, "no points");
         const std::vector<Curve> curves{Line{{0, 0}, {1, 1}},
                                         Line{{3, -1}, {2, 0}}};
         check(bounding_box(curves) == BoundingBox{{0, -1}, {3, 1}},
               "box of curves");
         expect_throws<std::invalid_argument>(
             [] { (void)bounding_box(std::span<const Curve>{}); }, "no curves");
       }},
      {"line",
       [] {
         check(Line{{3, -1}, {1, 2}}.bounding_box() ==
                   BoundingBox{{1, -1}, {3, 2}},
               "end points");
       }},
      {"quadratic_bezier",
       [] {
         // y(t) = 4 t (1 - t): peak 1 at t = 1/2; x is monotone.
         check(QuadraticBezier{{0, 0}, {1, 2}, {2, 0}}.bounding_box() ==
                   BoundingBox{{0, 0}, {2, 1}},
               "peak of the parabola");
         check_against_samples(QuadraticBezier{{0, 0}, {3, 1}, {1, 2}},
                               "quadratic with an x extreme");
       }},
      {"cubic_bezier",
       [] {
         check_against_samples(CubicBezier{{0, 0}, {3, 2}, {-2, 2}, {1, 0}},
                               "cubic with two x extremes");
         check_against_samples(CubicBezier{{0, 0}, {1, 1}, {2, -1}, {3, 0}},
                               "cubic with two y extremes");
         // Equally spaced x coordinates: x(t) = 3 t, so x' never vanishes
         // (both quadratic coefficients are 0).
         check_against_samples(CubicBezier{{0, 0}, {1, 3}, {2, 3}, {3, 0}},
                               "cubic with a linear x derivative");
       }},
      {"circular_arcs",
       [] {
         same_box(EllipticalArc{{1, 2}, 3, 3, 0, 0.3, 2 * pi}.bounding_box(),
                  {{-2, -1}, {4, 5}}, 1e-14, "full circle");
         same_box(EllipticalArc{{0, 0}, 1, 1, 0, 0, pi / 2}.bounding_box(),
                  {{0, 0}, {1, 1}}, 1e-15, "quarter circle");
         // Crossing 180 degrees: x reaches -1 in the middle.
         same_box(
             EllipticalArc{{0, 0}, 1, 1, 0, 3 * pi / 4, pi / 2}.bounding_box(),
             {{-1, -std::sqrt(0.5)}, {-std::sqrt(0.5), std::sqrt(0.5)}}, 1e-15,
             "arc across 180 degrees");
         // The same arc traversed backwards.
         same_box(
             EllipticalArc{{0, 0}, 1, 1, 0, 5 * pi / 4, -pi / 2}.bounding_box(),
             {{-1, -std::sqrt(0.5)}, {-std::sqrt(0.5), std::sqrt(0.5)}}, 1e-15,
             "negative sweep");
         // Start angles outside [0, 2 pi).
         same_box(EllipticalArc{{0, 0}, 1, 1, 0, -7 * pi, pi}.bounding_box(),
                  {{-1, -1}, {1, 0}}, 1e-14, "start angle -7 pi");
       }},
      {"rotated_ellipse",
       [] {
         // Half widths of a rotated ellipse: sqrt(rx^2 cos^2 + ry^2 sin^2)
         // and sqrt(rx^2 sin^2 + ry^2 cos^2).
         const double phi = 0.4, rx = 2, ry = 0.5;
         const double c = std::cos(phi), s = std::sin(phi);
         const double wx = std::sqrt(rx * rx * c * c + ry * ry * s * s);
         const double wy = std::sqrt(rx * rx * s * s + ry * ry * c * c);
         same_box(EllipticalArc{{1, 1}, rx, ry, phi, 0, 2 * pi}.bounding_box(),
                  {{1 - wx, 1 - wy}, {1 + wx, 1 + wy}}, 1e-14,
                  "full rotated ellipse");
         check_against_samples(EllipticalArc{{1, 1}, rx, ry, phi, 1.0, -2.5},
                               "rotated elliptical arc");
       }},
      {"geometry",
       [] {
         // A square with a circular hole, and a second piece (a disc) beside
         // it: the box of every curve of every loop.
         Loop square{{{Line{{0, 0}, {2, 0}}, "a"},
                      {Line{{2, 0}, {2, 2}}, "a"},
                      {Line{{2, 2}, {0, 2}}, "a"},
                      {Line{{0, 2}, {0, 0}}, "a"}}};
         Loop hole{{{EllipticalArc{{1, 1}, 0.5, 0.5, 0, 0, 2 * pi}, "h"}}};
         Loop disc{{{EllipticalArc{{4, 1}, 1, 1, 0, 0, 2 * pi}, "i"}}};
         same_box(CurveGeometry({square, hole, disc}).bounding_box(),
                  {{0, 0}, {5, 2}}, 1e-15, "all loops");
       }},
  });
}
