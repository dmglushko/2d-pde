#include "support/check.hpp"

#include "pde2d/geometry/curve_geometry.hpp"

#include <numbers>

using namespace pde2d::geometry;
using namespace pde2d::test;

namespace {
constexpr double pi = std::numbers::pi;

Loop square_loop(double x0, double y0, double size, bool ccw,
                 const std::string &region = "wall") {
  std::vector<Point> p = {
      {x0, y0}, {x0 + size, y0}, {x0 + size, y0 + size}, {x0, y0 + size}};
  if (!ccw)
    p = {p[0], p[3], p[2], p[1]};
  Loop loop;
  for (std::size_t i = 0; i < 4; ++i)
    loop.curves.push_back({Line{p[i], p[(i + 1) % 4]}, region});
  return loop;
}

Loop circle_loop(Point c, double r, bool ccw, const std::string &region) {
  return Loop{{{EllipticalArc{c, r, r, 0, 0, ccw ? 2 * pi : -2 * pi}, region}}};
}

// Independent orientation check: signed area of densely sampled points.
double sampled_area(const Loop &loop) {
  std::vector<Point> p;
  for (const auto &c : loop.curves)
    for (int i = 0; i < 400; ++i)
      p.push_back(point_at(c.curve, i / 400.0));
  double twice = 0;
  for (std::size_t i = 0; i < p.size(); ++i) {
    Point a = p[i], b = p[(i + 1) % p.size()];
    twice += a.x * b.y - a.y * b.x;
  }
  return twice / 2;
}

void check_closed_and_oriented(const CurveGeometry &g) {
  for (std::size_t l = 0; l < g.loops().size(); ++l) {
    const auto &curves = g.loops()[l].curves;
    for (std::size_t c = 0; c < curves.size(); ++c) {
      Point e = end_point(curves[c].curve);
      Point s = start_point(curves[(c + 1) % curves.size()].curve);
      check(distance(e, s) < 1e-12, "stored loops stay closed");
    }
    double area = sampled_area(g.loops()[l]);
    check(g.depth(l) % 2 == 0 ? area > 0 : area < 0,
          "even depth counterclockwise, odd depth clockwise");
  }
}
} // namespace

int main() {
  return run({
      {"depth_and_parent_with_island",
       [] {
         // Loop order deliberately not outer-first. Every loop is given in
         // the "wrong" direction for its role.
         CurveGeometry g({
             square_loop(3, 3, 1, false, "island"), // inside the hole
             circle_loop({3.5, 3.5}, 2, true, "hole"),
             square_loop(0, 0, 7, false, "outer"),
             square_loop(10, 0, 1, false, "second_piece"),
         });
         check(g.depth(0) == 2 && g.depth(1) == 1 && g.depth(2) == 0 &&
                   g.depth(3) == 0,
               "depths");
         check(g.parent(0) == 1u && g.parent(1) == 2u, "parents");
         check(!g.parent(2) && !g.parent(3), "top level has no parent");
         check_closed_and_oriented(g);
       }},
      {"already_normalized_loops_are_kept",
       [] {
         Loop outer = square_loop(0, 0, 4, true);
         CurveGeometry g({outer, circle_loop({2, 2}, 1, false, "hole")});
         const auto *first = std::get_if<Line>(&g.loops()[0].curves[0].curve);
         check(first && first->start == Point{0, 0} &&
                   first->end == Point{4, 0},
               "outer loop unchanged");
         check_closed_and_oriented(g);
       }},
      {"far_from_origin_loop_is_not_reversed",
       [] {
         // A counterclockwise unit square at (1e9, 1e9). Its area must not
         // cancel to zero (which would read as clockwise and reverse it).
         for (bool ccw : {true, false}) {
           CurveGeometry g({square_loop(1e9, 1e9, 1, ccw)});
           const auto &line = std::get<Line>(g.loops()[0].curves[0].curve);
           check(line.start == Point{1e9, 1e9} &&
                     line.end == Point{1e9 + 1, 1e9},
                 "normalized counterclockwise");
         }
       }},
      {"reversal_keeps_regions_with_their_curves",
       [] {
         Loop loop = square_loop(0, 0, 1, false);
         loop.curves[0].region = "a"; // (0,0) -> (0,1) when clockwise
         CurveGeometry g({loop});
         for (const auto &c : g.loops()[0].curves) {
           const auto &line = std::get<Line>(c.curve);
           bool left_side = line.start.x == 0 && line.end.x == 0;
           check(left_side == (c.region == "a"), "region follows its curve");
         }
       }},
      {"open_loop",
       [] {
         Loop loop = square_loop(0, 0, 1, true);
         std::get<Line>(loop.curves[2].curve).end = {0, 0.9};
         expect_throws<std::invalid_argument>(
             [&] { CurveGeometry g({loop}); },
             "Loop 0 is not closed: curve 2 ends at");
       }},
      {"self_intersection",
       [] {
         Loop bowtie{{{Line{{0, 0}, {1, 1}}, "a"},
                      {Line{{1, 1}, {1, 0}}, "a"},
                      {Line{{1, 0}, {0, 1}}, "a"},
                      {Line{{0, 1}, {0, 0}}, "a"}}};
         expect_throws<std::invalid_argument>(
             [&] { CurveGeometry g({bowtie}); },
             "Loop 0 intersects or touches itself");
       }},
      {"loops_intersect",
       [] {
         expect_throws<std::invalid_argument>(
             [] {
               CurveGeometry g({square_loop(0, 0, 2, true),
                                circle_loop({2, 1}, 0.5, true, "c")});
             },
             "Loops 0 and 1 intersect or touch");
       }},
      {"loops_touch",
       [] {
         // A circle touching the square's side from inside.
         expect_throws<std::invalid_argument>(
             [] {
               CurveGeometry g({square_loop(0, 0, 2, true),
                                circle_loop({1, 1}, 1, true, "c")});
             },
             "intersect or touch");
         // Two squares sharing a corner.
         expect_throws<std::invalid_argument>(
             [] {
               CurveGeometry g(
                   {square_loop(0, 0, 1, true), square_loop(1, 1, 1, true)});
             },
             "intersect or touch");
       }},
      {"invalid_input",
       [] {
         expect_throws<std::invalid_argument>([] { CurveGeometry g({}); },
                                              "at least one loop");
         expect_throws<std::invalid_argument>([] { CurveGeometry g({Loop{}}); },
                                              "Loop 0 has no curves");
         Loop bad_name = square_loop(0, 0, 1, true, "has space");
         expect_throws<std::invalid_argument>(
             [&] { CurveGeometry g({bad_name}); },
             "Loop 0, curve 0: region name 'has space' is invalid");
         Loop zero = square_loop(0, 0, 1, true);
         zero.curves.insert(zero.curves.begin() + 1,
                            {Line{{1, 0}, {1, 0}}, "wall"});
         expect_throws<std::invalid_argument>(
             [&] { CurveGeometry g({zero}); },
             "Loop 0, curve 1 has zero length");
         Loop arc{{{EllipticalArc{{0, 0}, -1, 1, 0, 0, 2 * pi}, "a"}}};
         expect_throws<std::invalid_argument>(
             [&] { CurveGeometry g({arc}); },
             "Loop 0, curve 0: Invalid elliptical arc");
       }},
      {"mixed_curve_kinds",
       [] {
         Loop loop{{{Line{{0, 0}, {2, 0}}, "a"},
                    {QuadraticBezier{{2, 0}, {3, 1}, {2, 2}}, "b"},
                    {CubicBezier{{2, 2}, {1.5, 3}, {0.5, 1}, {0, 2}}, "c"},
                    {Line{{0, 2}, {0, 0}}, "a"}}};
         CurveGeometry g({loop});
         check(g.depth(0) == 0, "single loop");
         check(g.region_names() == std::vector<std::string>{"a", "b", "c"},
               "region order");
       }},
      {"transformed",
       [] {
         CurveGeometry g({square_loop(0, 0, 1, true)});
         auto moved = g.transformed({.scale = 2, .translation = {5, 0}});
         const auto &line = std::get<Line>(moved.loops()[0].curves[0].curve);
         check(line.start == Point{5, 0} && line.end == Point{7, 0}, "moved");
       }},
  });
}
