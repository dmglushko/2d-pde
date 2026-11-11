#include "support/check.hpp"

#include "pde2d/geometry/predicates.hpp"

#include <cmath>
#include <limits>

using namespace pde2d::geometry;
using namespace pde2d::test;

int main() {
  return run({
      {"orient_signs",
       [] {
         check(orient({0, 0}, {1, 0}, {0, 1}) == 1, "counterclockwise");
         check(orient({0, 0}, {0, 1}, {1, 0}) == -1, "clockwise");
         check(orient({0, 0}, {1, 1}, {2, 2}) == 0, "collinear");
       }},
      {"orient_exact_near_degenerate",
       [] {
         // Points on the line y = x, offset by one unit in the last place:
         // floating-point cross products cannot decide these.
         double x = 0.1, next = std::nextafter(0.1, 1.0);
         check(orient({0, 0}, {1e10, 1e10}, {x, x}) == 0, "on the line");
         check(orient({0, 0}, {1e10, 1e10}, {x, next}) == 1, "just above");
         check(orient({0, 0}, {1e10, 1e10}, {next, x}) == -1, "just below");
         // Exactly collinear points with large and small coordinates.
         check(orient({1e-300, 1e-300}, {1e300, 1e300}, {3, 3}) == 0,
               "wide range collinear");
       }},
      {"incircle_signs",
       [] {
         Point a{1, 0}, b{0, 1}, c{-1, 0};
         check(incircle(a, b, c, {0, 0}) == 1, "center inside");
         check(incircle(a, b, c, {0, -1}) == 0, "cocircular");
         check(incircle(a, b, c, {0, -1.0000001}) == -1, "outside");
         check(incircle(a, b, c, {0, std::nextafter(-1.0, 0.0)}) == 1,
               "one ulp inside");
       }},
      {"segments",
       [] {
         check(segments_intersect({0, 0}, {2, 2}, {0, 2}, {2, 0}), "cross");
         check(segments_intersect({0, 0}, {1, 0}, {1, 0}, {2, 5}),
               "shared end point");
         check(segments_intersect({0, 0}, {2, 0}, {1, 0}, {3, 0}),
               "collinear overlap");
         check(!segments_intersect({0, 0}, {1, 0}, {2, 0}, {3, 0}),
               "collinear apart");
         check(!segments_intersect({0, 0}, {1, 1}, {1, 0}, {2, 1}), "parallel");
         check(on_segment({0, 0}, {2, 2}, {1, 1}), "on segment");
         check(!on_segment({0, 0}, {2, 2}, {3, 3}), "beyond the end");
       }},
      {"non_finite_rejected",
       [] {
         double inf = std::numeric_limits<double>::infinity();
         expect_throws<std::invalid_argument>(
             [&] { (void)orient({0, 0}, {inf, 0}, {0, 0}); }, "finite");
       }},
  });
}
