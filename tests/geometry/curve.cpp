#include "support/check.hpp"

#include "pde2d/geometry/curve.hpp"

#include <numbers>

using namespace pde2d::geometry;
using namespace pde2d::test;

namespace {
constexpr double pi = std::numbers::pi;

void same_point(Point a, Point b, double tol, std::string_view what) {
  near(a.x, b.x, tol, what);
  near(a.y, b.y, tol, what);
}

// Independent length: a fine polyline through the curve.
double polyline_length(const Curve &c, int n) {
  double total = 0;
  for (int i = 0; i < n; ++i)
    total += distance(c.point_at(double(i) / n), c.point_at(double(i + 1) / n));
  return total;
}
} // namespace

int main() {
  return run({
      {"line",
       [] {
         Curve c = Line{{0, 0}, {3, 4}};
         same_point(c.point_at(0.5), {1.5, 2}, 0, "midpoint");
         near(curve_length(c), 5, 0, "length");
         near(curve_length(c, 0.2), 1, 1e-15, "partial length");
         check(is_line(c), "is_line");
       }},
      {"circle_arc_length_exact",
       [] {
         Curve c = EllipticalArc{{1, 2}, 2, 2, 0, 0, pi};
         near(curve_length(c), 2 * pi, 1e-14, "half circle");
         same_point(c.point_at(0.5), {1, 4}, 1e-15, "top");
         near(parameter_at_length_fraction(c, 0.25), 0.25, 0, "fraction");
       }},
      {"ellipse_length",
       [] {
         // Ramanujan's second approximation is accurate to ~1e-10 here.
         double a = 2, b = 1, h = (a - b) * (a - b) / ((a + b) * (a + b));
         double ramanujan =
             pi * (a + b) * (1 + 3 * h / (10 + std::sqrt(4 - 3 * h)));
         Curve c = EllipticalArc{{0, 0}, a, b, 0, 0, 2 * pi};
         near(curve_length(c), ramanujan, 1e-8, "ellipse perimeter");
       }},
      {"bezier_length_matches_polyline",
       [] {
         Curve q = QuadraticBezier{{0, 0}, {1, 2}, {3, 0}};
         near(curve_length(q), polyline_length(q, 200000), 1e-8, "quadratic");
         Curve c = CubicBezier{{0, 0}, {1, 3}, {2, -3}, {3, 0}};
         near(curve_length(c), polyline_length(c, 200000), 1e-8, "cubic");
       }},
      {"length_fraction_inverts_length",
       [] {
         Curve c = CubicBezier{{0, 0}, {1, 3}, {2, -3}, {3, 0}};
         for (double f : {0.1, 0.5, 0.9}) {
           double t = parameter_at_length_fraction(c, f);
           near(curve_length(c, t) / curve_length(c), f, 1e-10, "fraction");
         }
       }},
      {"split_is_continuous_and_exact",
       [] {
         std::vector<Curve> curves = {
             Line{{0, 0}, {1, 1}}, QuadraticBezier{{0, 0}, {1, 2}, {3, 0}},
             CubicBezier{{0, 0}, {1, 3}, {2, -3}, {3, 0}},
             EllipticalArc{{0, 0}, 2, 1, 0.3, 0.2, 2.5}};
         for (const auto &c : curves) {
           auto [a, b] = c.split(0.3);
           same_point(end_point(a), c.point_at(0.3), 1e-14, "split point");
           same_point(start_point(b), c.point_at(0.3), 1e-14, "split point");
           // The first piece at s is the original at 0.3 s.
           same_point(a.point_at(0.5), c.point_at(0.15), 1e-14, "first");
           same_point(b.point_at(0.5), c.point_at(0.65), 1e-14, "second");
           Curve s = c.subcurve(0.2, 0.7);
           same_point(s.point_at(0.5), c.point_at(0.45), 1e-14, "subcurve");
           near(curve_length(a) + curve_length(b), curve_length(c), 1e-10,
                "lengths add");
         }
       }},
      {"reverse_and_transform",
       [] {
         Curve c = CubicBezier{{0, 0}, {1, 3}, {2, -3}, {3, 0}};
         same_point(c.reversed().point_at(0.25), c.point_at(0.75), 1e-15,
                    "reversed");
         Curve arc = EllipticalArc{{0, 0}, 1, 1, 0, 0, pi / 2};
         same_point(arc.reversed().point_at(0), {0, 1}, 1e-15, "reversed arc");
         CurveTransform f{
             .scale = 2, .rotation = pi / 2, .translation = {1, 0}};
         same_point(arc.transformed(f).point_at(0), {1, 2}, 1e-15, "moved");
         near(curve_length(arc.transformed(f)), pi, 1e-14, "scaled length");
       }},
      {"derivative_bounds_hold",
       [] {
         Curve c = CubicBezier{{0, 0}, {1, 3}, {2, -3}, {3, 0}};
         auto b = c.derivative_bounds();
         for (int i = 0; i <= 100; ++i) {
           double t = i / 100.0;
           check(norm(c.derivative_at(t)) <= b.first + 1e-12, "first");
           check(norm(c.second_derivative_at(t)) <= b.second + 1e-12, "second");
         }
       }},
      {"invalid_curves",
       [] {
         expect_throws<std::invalid_argument>(
             [] { EllipticalArc{{0, 0}, 0, 1, 0, 0, 1}.validate(); },
             "Invalid elliptical arc");
         expect_throws<std::invalid_argument>(
             [] { EllipticalArc{{0, 0}, 1, 1, 0, 0, 7}.validate(); },
             "Invalid elliptical arc");
         expect_throws<std::invalid_argument>(
             [] { (void)Line{{0, 0}, {1, 0}}.split(1.0); }, "between 0 and 1");
         expect_throws<std::invalid_argument>(
             [] {
               (void)parameter_at_length_fraction(Line{{0, 0}, {0, 0}}, .5);
             },
             "zero-length");
       }},
  });
}
