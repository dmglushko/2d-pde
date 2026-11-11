#include "support/check.hpp"

#include "pde2d/geometry/shapes.hpp"

#include <numbers>

using namespace pde2d::geometry;
using namespace pde2d::test;

namespace {
constexpr double pi = std::numbers::pi;
using Names = std::vector<std::string>;

double perimeter(const CurveGeometry &g, std::size_t loop) {
  double total = 0;
  for (const auto &c : g.loops()[loop].curves)
    total += curve_length(c.curve);
  return total;
}
} // namespace

int main() {
  return run({
      {"circle",
       [] {
         auto g = circle(2);
         check(g.region_names() == Names{"boundary"}, "regions");
         near(perimeter(g, 0), 4 * pi, 1e-13, "perimeter");
       }},
      {"ellipse",
       [] {
         auto g = ellipse(1, 1);
         near(perimeter(g, 0), 2 * pi, 1e-9, "a circle as an ellipse");
       }},
      {"rectangle",
       [] {
         auto g = rectangle(3, 1);
         check(g.region_names() == Names{"bottom", "right", "top", "left"},
               "regions");
         near(g.region_length("bottom"), 3, 0, "bottom");
         near(g.region_length("left"), 1, 0, "left");
       }},
      {"annulus",
       [] {
         auto g = annulus(2, 1);
         check(g.region_names() == Names{"outer", "inner"}, "regions");
         check(g.depth(0) == 0 && g.depth(1) == 1, "inner is a hole");
         near(g.region_length("inner"), 2 * pi, 1e-13, "inner length");
         expect_throws<std::invalid_argument>([] { (void)annulus(1, 2); },
                                              "smaller");
       }},
      {"rectangle_with_semicircle",
       [] {
         auto g = rectangle_with_semicircle(2, 1);
         check(g.region_names() == Names{"bottom", "cap", "top", "left"},
               "positive x");
         near(g.region_length("cap"), pi / 2, 1e-14, "cap");
         auto h = rectangle_with_semicircle(2, 1, SemicircleEnd::negative_x);
         check(h.region_names() == Names{"bottom", "right", "top", "cap"},
               "negative x");
       }},
      {"polygons",
       [] {
         // Given clockwise; stored counterclockwise.
         auto g = polygon({{0, 0}, {0, 1}, {1, 1}, {1, 0}});
         double twice = 0;
         for (const auto &c : g.loops()[0].curves) {
           Point a = start_point(c.curve), b = end_point(c.curve);
           twice += a.x * b.y - a.y * b.x;
         }
         near(twice / 2, 1, 0, "stored counterclockwise");
         near(perimeter(g, 0), 4, 0, "square");
         auto hex = regular_polygon(6, 0.5);
         near(perimeter(hex, 0), 3, 1e-14, "hexagon");
         expect_throws<std::invalid_argument>(
             [] { (void)regular_polygon(2, 1); }, "at least 3");
         expect_throws<std::invalid_argument>(
             [] { (void)polygon({{0, 0}, {1, 0}, {2, 0}}); },
             "intersects or touches itself");
       }},
      {"sizes_validated",
       [] {
         expect_throws<std::invalid_argument>([] { (void)circle(0); },
                                              "finite and positive");
         expect_throws<std::invalid_argument>([] { (void)rectangle(1, -1); },
                                              "finite and positive");
       }},
  });
}
