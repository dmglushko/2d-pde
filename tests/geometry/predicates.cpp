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
         check(orient({1e-50, 1e-50}, {1e50, 1e50}, {3, 3}) == 0,
               "wide range collinear");
         check(orient({1e-50, 1e-50}, {1e50, 1e50},
                      {3, std::nextafter(3.0, 4)}) == 1,
               "wide range, just above");
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
      {"extreme_coordinates",
       [] {
         // Outside [2^-200, 2^200] the adaptive arithmetic could overflow
         // or underflow; undecided cases there use exact integers.
         check(orient({1e-300, 1e-300}, {1e300, 1e300}, {3, 3}) == 0,
               "very wide range, collinear");
         double tiny = std::numeric_limits<double>::denorm_min();
         check(orient({0, 0}, {1, 1}, {tiny, tiny}) == 0, "subnormal on line");
         check(orient({0, 0}, {1, 1}, {tiny, 2 * tiny}) == 1,
               "subnormal above");
         check(orient({0, 0}, {1e300, 1e300}, {1, std::nextafter(1.0, 0.0)}) ==
                   -1,
               "large and below");
         check(incircle({1e70, 0}, {0, 1e70}, {-1e70, 0}, {0, -1e70}) == 0,
               "large cocircular");
         check(incircle({1e-300, 0}, {0, 1e-300}, {-1e-300, 0},
                        {0, std::nextafter(-1e-300, 0.0)}) == 1,
               "tiny, just inside");
       }},
      {"mixed_scales_underflow",
       [] {
         // Coordinates of very different sizes: a product of two small
         // differences underflows, and multiplied by a huge lifted
         // distance its lost accuracy outweighs the filter's error bound.
         // Counterclockwise a, b, c; d lies outside their circle.
         Point a{0, 0x1p400}, b{0, 0x1p-600}, c{0x1p-900, 0x1p-150},
             d{0x1p-1000, 0};
         check(orient(a, b, c) == 1, "counterclockwise");
         check(incircle(a, b, c, d) == -1, "d outside the circle");
         check(incircle(b, c, a, d) == -1, "rotated, d outside");
         // orient: a product that underflows to a subnormal or to zero.
         check(orient({0, 0}, {0x1p-600, 0x1p-600}, {0x1p-500, 0x1p-500}) == 0,
               "tiny collinear");
         check(orient({0, 0}, {0x1p-600, 0x1p-560},
                      {0x1p-500, std::nextafter(0x1p-460, 0.0)}) == -1,
               "tiny, just below");
       }},
      {"non_finite_rejected",
       [] {
         double inf = std::numeric_limits<double>::infinity();
         expect_throws<std::invalid_argument>(
             [&] { (void)orient({0, 0}, {inf, 0}, {0, 0}); }, "finite");
       }},
  });
}
