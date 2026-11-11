#include "support/check.hpp"

#include "pde2d/geometry/shapes.hpp"

#include <numbers>

using namespace pde2d::geometry;
using namespace pde2d::test;

namespace {
constexpr double pi = std::numbers::pi;
using Names = std::vector<std::string>;
} // namespace

int main() {
  return run({
      {"rename",
       [] {
         auto g = rectangle(2, 1);
         g.rename_region("top", "lid");
         check(g.region_names() == Names{"bottom", "right", "lid", "left"},
               "renamed in place");
         expect_throws<std::invalid_argument>(
             [&] { g.rename_region("lid", "left"); }, "already exists");
         expect_throws<std::invalid_argument>(
             [&] { g.rename_region("top", "x"); }, "Unknown region 'top'");
         expect_throws<std::invalid_argument>(
             [&] { g.rename_region("lid", "a b"); }, "invalid");
       }},
      {"merge",
       [] {
         auto g = rectangle(2, 1);
         g.merge_region_into("left", "right");
         g.merge_region_into("top", "bottom");
         check(g.region_names() == Names{"bottom", "right"}, "two left");
         near(g.region_length("right"), 2, 1e-15, "sides");
         near(g.region_length("bottom"), 4, 1e-15, "top and bottom");
         expect_throws<std::invalid_argument>(
             [&] { g.merge_region_into("right", "right"); }, "itself");
         expect_throws<std::invalid_argument>(
             [&] { g.merge_region_into("right", "nope"); }, "Unknown region");
       }},
      {"split_full_loop_region",
       [] {
         auto g = circle(1);
         g.split_region("boundary", 0.25, "first", "rest");
         near(g.region_length("first"), pi / 2, 1e-12, "quarter");
         near(g.region_length("rest"), 3 * pi / 2, 1e-12, "rest");
         check(g.loops()[0].curves.size() == 2, "the circle became two arcs");
         // The first piece starts where the loop started (angle 0).
         Point s = start_point(g.loops()[0].curves[0].curve);
         near(s.x, 1, 1e-15, "start x");
         near(s.y, 0, 1e-15, "start y");
       }},
      {"split_across_curves",
       [] {
         auto g = rectangle(2, 1);
         g.merge_region_into("right", "bottom"); // bottom (2) + right (1)
         g.split_region("bottom", 0.5, "bottom", "corner");
         near(g.region_length("bottom"), 1.5, 1e-15, "first part");
         near(g.region_length("corner"), 1.5, 1e-15, "second part");
         check(g.loops()[0].curves.size() == 5, "bottom line was split");
       }},
      {"split_at_junction_splits_no_curve",
       [] {
         auto g = rectangle(2, 2);
         g.merge_region_into("right", "bottom");
         g.split_region("bottom", 0.5, "b", "r");
         check(g.loops()[0].curves.size() == 4, "no new curve");
         near(g.region_length("b"), 2, 0, "b");
       }},
      {"split_region_wrapping_the_loop_start",
       [] {
         // "left" (last curve) and "bottom" (first curve) form one piece
         // that crosses the loop's first curve.
         auto g = rectangle(2, 1);
         g.merge_region_into("bottom", "left");
         g.split_region("left", 0.5, "west", "south");
         near(g.region_length("west"), 1.5, 1e-15, "left + half bottom");
         near(g.region_length("south"), 1.5, 1e-15, "rest of bottom");
         check(g.loops()[0].curves.front().region == "west",
               "the loop's first curve is the first half of bottom");
       }},
      {"split_errors",
       [] {
         auto g = annulus(2, 1);
         g.merge_region_into("inner", "outer");
         expect_throws<std::invalid_argument>(
             [&] { g.split_region("outer", 0.5, "a", "b"); }, "several loops");
         auto r = rectangle(2, 1);
         r.merge_region_into("top", "bottom");
         expect_throws<std::invalid_argument>(
             [&] { r.split_region("bottom", 0.5, "a", "b"); },
             "several separate pieces");
         expect_throws<std::invalid_argument>(
             [&] { r.split_region("left", 0.5, "a", "right"); },
             "already exists");
         expect_throws<std::invalid_argument>(
             [&] { r.split_region("left", 0.5, "a", "a"); }, "must differ");
         expect_throws<std::invalid_argument>(
             [&] { r.split_region("left", 1.0, "a", "b"); },
             "strictly between 0 and 1");
       }},
  });
}
