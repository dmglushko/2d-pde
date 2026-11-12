#include "support/check.hpp"
#include "support/shapes.hpp"

#include "pde2d/mesh/chord_length_discretizer.hpp"

#include <cmath>
#include <numbers>
#include <vector>

using namespace pde2d::mesh;
using namespace pde2d::test;

namespace {

constexpr double pi = std::numbers::pi;

BoundaryPolygon run_discretizer(const CurveGeometry &g, double h,
                                ChordLengthOptions o = {}) {
  StageReport report;
  return ChordLengthDiscretizer(o).run(g, {.max_edge_length = {.absolute = h}},
                                       report);
}

void check_chords(const BoundaryPolygon &p, double h) {
  for (const auto &loop : p.loops)
    for (std::size_t i = 0; i < loop.points.size(); ++i) {
      Point a = loop.points[i], b = loop.points[(i + 1) % loop.points.size()];
      check(distance(a, b) <= h, "chord at most max_edge_length");
      const auto &link = loop.edges[i].link;
      check(link.has_value(), "linked");
      const auto &chain = p.chains[link->chain];
      check(link->s0 < link->s1, "increasing arc length");
      near(distance(chain.point_at(link->s0), a), 0, 1e-15, "s0 at a");
      if (link->s1 < chain.length())
        near(distance(chain.point_at(link->s1), b), 0, 1e-15, "s1 at b");
    }
}

bool has_point(const BoundaryPolygon &p, Point q, double tolerance) {
  for (const auto &loop : p.loops)
    for (auto x : loop.points)
      if (distance(x, q) <= tolerance)
        return true;
  return false;
}

std::size_t edges_in_region(const BoundaryPolygon &p, std::size_t region) {
  std::size_t n = 0;
  for (const auto &loop : p.loops)
    for (const auto &e : loop.edges)
      n += e.region == region;
  return n;
}

// The unit circle as k arcs of one region, starting at angle 0.
CurveGeometry circle_of_arcs(int k) {
  Loop loop;
  for (int i = 0; i < k; ++i)
    loop.curves.push_back(
        {EllipticalArc{{0, 0}, 1, 1, 0, i * 2 * pi / k, 2 * pi / k},
         "boundary"});
  return CurveGeometry({loop});
}

} // namespace

int main() {
  return run({
      {"rectangle",
       [] {
         auto p = run_discretizer(rectangle(2, 1), 0.3);
         validate(p);
         // ceil(2 / 0.3) = 7 pieces on the long sides, ceil(1 / 0.3) = 4 on
         // the short ones.
         check(p.loops.size() == 1 && p.loops[0].points.size() == 22,
               "22 points");
         check_chords(p, 0.3);
       }},
      {"equal_pieces_and_counts",
       [] {
         auto p = run_discretizer(rectangle(2, 1), 0.5);
         // Exact divisions: 2 / 0.5 = 4 pieces on the long sides, 2 on the
         // short ones.
         check(p.loops[0].points.size() == 12, "12 points");
         auto c = run_discretizer(circle(1), 0.1);
         std::size_t n = c.loops[0].points.size();
         check(n == std::size_t(std::ceil(2 * pi / 0.1)), "ceil(L / h)");
         double first = distance(c.loops[0].points[0], c.loops[0].points[1]);
         for (std::size_t i = 0; i < n; ++i)
           near(distance(c.loops[0].points[i], c.loops[0].points[(i + 1) % n]),
                first, 1e-12, "equal chords");
         for (auto pt : c.loops[0].points)
           near(norm(pt), 1, 1e-15, "points on the circle");
         check_chords(c, 0.1);
       }},
      {"regions_and_depths",
       [] {
         auto p = run_discretizer(annulus(1, 0.5), 0.2);
         check(p.region_names == std::vector<std::string>{"outer", "inner"},
               "names");
         check(p.loops[1].depth == 1 && p.loops[1].parent == 0u, "hole");
         for (const auto &e : p.loops[1].edges)
           check(e.region == 1, "inner edges in region inner");
         validate(p);
       }},
      {"curve_deviation_option",
       [] {
         // Large h: the deviation bound decides the count. The bound used is
         // r w^2 / 8 for a piece with sweep w, so w <= sqrt(8 d / r).
         auto p = run_discretizer(circle(1), 10,
                                  {.max_curve_deviation = {.absolute = 0.001}});
         double w = 2 * pi / double(p.loops[0].points.size());
         check(w * w / 8 <= 0.001, "deviation bound met");
         // The real sagitta is smaller than the bound.
         check(1 - std::cos(w / 2) <= 0.001, "sagitta");
         check_chords(p, 10);
         expect_throws<std::invalid_argument>(
             [] {
               ChordLengthDiscretizer({.max_curve_deviation = {.absolute = 0}});
             },
             "ChordLengthOptions::max_curve_deviation.absolute must be finite "
             "and positive");
         expect_throws<std::invalid_argument>(
             [] {
               ChordLengthDiscretizer(
                   {.max_curve_deviation = {.relative = INFINITY}});
             },
             "ChordLengthOptions::max_curve_deviation.relative must be finite "
             "and positive");
       }},
      {"curve_deviation_is_off_by_default",
       [] {
         const auto d = ChordLengthOptions{}.max_curve_deviation;
         check(!d.absolute && !d.relative, "neither value set");
         // A huge h and no deviation bound: the minimum of three points.
         StageReport r;
         auto p = ChordLengthDiscretizer().run(
             circle(1), {.max_edge_length = {.absolute = 100}}, r);
         check(p.loops[0].points.size() == 3, "a triangle");
         check(!r.metrics.contains("max_curve_deviation"), "no metric");
         validate(p);
       }},
      {"relative_values_scale_with_the_domain",
       [] {
         // The domain size of a circle of radius r is its bounding-box
         // diagonal, 2 r sqrt(2). Relative values give the same polygon,
         // scaled, for every r.
         for (double rel_h : {0.2, 10.0}) {
           std::vector<std::size_t> counts;
           for (double r : {1e-3, 1.0, 1e3}) {
             StageReport report;
             auto p =
                 ChordLengthDiscretizer(
                     {.max_curve_deviation = {.relative = 1e-3}})
                     .run(circle(r), {.max_edge_length = {.relative = rel_h}},
                          report);
             const double size = 2 * r * std::sqrt(2.0);
             near(report.metrics.at("domain_size"), size, 1e-12 * size,
                  "domain size");
             near(report.metrics.at("max_edge_length"), rel_h * size,
                  1e-12 * size, "resolved edge length");
             near(report.metrics.at("max_curve_deviation"), 1e-3 * size,
                  1e-12 * size, "resolved deviation");
             check_chords(p, rel_h * size);
             counts.push_back(p.loops[0].points.size());
           }
           check(counts[0] == counts[1] && counts[1] == counts[2],
                 "same count at every scale");
         }
       }},
      {"absolute_and_relative_take_the_smaller",
       [] {
         // Unit square: domain size sqrt(2), perimeter 4.
         auto count = [](LengthBound h) {
           StageReport r;
           return ChordLengthDiscretizer()
               .run(rectangle(1, 1), {.max_edge_length = h}, r)
               .loops[0]
               .points.size();
         };
         const double s = std::sqrt(2.0);
         check(count({.absolute = 0.5}) == 8, "absolute only");
         check(count({.relative = 0.3 / s}) == 16, "relative only");
         check(count({.absolute = 0.5, .relative = 0.3 / s}) == 16,
               "relative smaller");
         check(count({.absolute = 0.3, .relative = 0.5 / s}) == 16,
               "absolute smaller");
       }},
      {"bezier_curves",
       [] {
         Loop loop{{{Line{{0, 0}, {2, 0}}, "a"},
                    {QuadraticBezier{{2, 0}, {3, 1}, {2, 2}}, "b"},
                    {CubicBezier{{2, 2}, {1.5, 3}, {0.5, 1}, {0, 2}}, "c"},
                    {Line{{0, 2}, {0, 0}}, "a"}}};
         auto p = run_discretizer(CurveGeometry({loop}), 0.1);
         validate(p);
         check_chords(p, 0.1);
       }},
      {"chains_one_arc_or_four",
       [] {
         for (double h : {0.1, 0.37, 0.9}) {
           auto one = run_discretizer(circle(1), h);
           auto four = run_discretizer(circle_of_arcs(4), h);
           check(one.chains.size() == 1 && four.chains.size() == 1,
                 "a smooth closed loop is one chain");
           near(four.chains[0].length(), 2 * pi, 1e-14, "chain length");
           const auto &p1 = one.loops[0].points, &p4 = four.loops[0].points;
           check(p1.size() == p4.size(), "same node count");
           for (std::size_t i = 0; i < p1.size(); ++i)
             near(distance(p1[i], p4[i]), 0, 1e-14, "same nodes");
           check_chords(four, h);
         }
         // With h = 0.37, 2 pi / 0.37 = 16.98: 17 pieces, so the quarter
         // junctions at angles pi/2, pi, 3 pi/2 are not nodes.
         auto four = run_discretizer(circle_of_arcs(4), 0.37);
         check(!has_point(four, {0, 1}, 1e-9) &&
                   !has_point(four, {-1, 0}, 1e-9),
               "junctions inside the chain are not nodes");
         check(has_point(four, {1, 0}, 0), "the seam (start of curve 0) is");
       }},
      {"chains_polyline_sampled_by_h",
       [] {
         // The bottom side is 200 tiny, nearly collinear segments.
         Loop loop;
         std::vector<Point> bottom;
         for (int k = 0; k <= 200; ++k)
           bottom.push_back(
               {0.01 * k, k == 0 || k == 200 ? 0.0 : 1e-4 * std::sin(k)});
         for (int k = 0; k < 200; ++k)
           loop.curves.push_back({Line{bottom[k], bottom[k + 1]}, "bottom"});
         loop.curves.push_back({Line{{2, 0}, {2, 1}}, "right"});
         loop.curves.push_back({Line{{2, 1}, {0, 1}}, "top"});
         loop.curves.push_back({Line{{0, 1}, {0, 0}}, "left"});
         auto p = run_discretizer(CurveGeometry({loop}), 0.45);
         validate(p);
         check(p.chains.size() == 4, "one chain per side");
         // ceil(2.00... / 0.45) = 5 pieces, not 200.
         check(edges_in_region(p, 0) == 5, "sampled by h, not by vertices");
         check_chords(p, 0.45);
       }},
      {"chains_break_at_regions_and_corners",
       [] {
         auto r = run_discretizer(rectangle(2, 1), 0.3);
         check(r.chains.size() == 4, "four sides");
         for (Point corner :
              {Point{-1, -0.5}, Point{1, -0.5}, Point{1, 0.5}, Point{-1, 0.5}})
           check(has_point(r, corner, 0), "corner is a node");
         // Two semicircles of different regions: smooth, but the region
         // changes, so both junctions are nodes.
         Loop halves{{{EllipticalArc{{0, 0}, 1, 1, 0, 0, pi}, "a"},
                      {EllipticalArc{{0, 0}, 1, 1, 0, pi, pi}, "b"}}};
         auto s = run_discretizer(CurveGeometry({halves}), 0.37);
         check(s.chains.size() == 2, "two chains");
         check(has_point(s, {1, 0}, 1e-15) && has_point(s, {-1, 0}, 1e-15),
               "region borders are nodes");
         // A 20 degree turn inside one region: joined by default (30), a
         // node when the threshold is 10 degrees.
         double rise = std::tan(20 * pi / 180);
         CurveGeometry bend({polygon_loop(
             {{0, 0}, {1, 0}, {2, rise}, {2, 2}, {0, 2}}, "wall")});
         auto joined = run_discretizer(bend, 0.3);
         check(!has_point(joined, {1, 0}, 1e-9), "20 degrees: no node");
         auto split = run_discretizer(bend, 0.3, {.corner_angle_degrees = 10});
         check(has_point(split, {1, 0}, 0), "threshold 10: a node");
         check(split.chains.size() == joined.chains.size() + 1, "one more");
         check_chords(joined, 0.3);
         expect_throws<std::invalid_argument>(
             [] { ChordLengthDiscretizer({.corner_angle_degrees = 181}); },
             "[0, 180]");
         check(ChordLengthOptions{}.corner_angle_degrees == 30 &&
                   default_corner_angle_degrees == 30,
               "default 30 degrees");
       }},
      {"chain_deviation_across_a_junction",
       [] {
         // The corner inside a chain is cut by the chords, but by no more
         // than the deviation bound.
         double rise = std::tan(25 * pi / 180);
         CurveGeometry bend({polygon_loop(
             {{0, 0}, {1, 0}, {2, rise}, {2, 2}, {0, 2}}, "wall")});
         for (double d : {0.05, 0.01, 0.002}) {
           auto p = run_discretizer(bend, 10,
                                    {.max_curve_deviation = {.absolute = d}});
           double worst = 0;
           const auto &pts = p.loops[0].points;
           for (std::size_t i = 0; i < pts.size(); ++i) {
             Point a = pts[i], b = pts[(i + 1) % pts.size()];
             if (a.x < 1 && b.x > 1) {
               // Distance of the corner (1, 0) from the chord ab.
               worst = std::abs(cross(b - a, Point{1, 0} - a)) / distance(a, b);
             }
           }
           check(worst <= d, "corner within the deviation of its chord");
         }
       }},
      {"validate_polygon",
       [] {
         auto p = run_discretizer(rectangle(1, 1), 0.5);
         auto wrong_direction = p;
         std::reverse(wrong_direction.loops[0].points.begin(),
                      wrong_direction.loops[0].points.end());
         expect_throws<std::invalid_argument>(
             [&] { validate(wrong_direction); }, "wrong direction");
         auto bad_region = p;
         bad_region.loops[0].edges[0].region = 9;
         expect_throws<std::invalid_argument>([&] { validate(bad_region); },
                                              "region id out of range");
         auto bad_depth = p;
         bad_depth.loops[0].depth = 2;
         expect_throws<std::invalid_argument>([&] { validate(bad_depth); },
                                              "no parent");
         expect_throws<std::invalid_argument>(
             [&] { (void)run_discretizer(rectangle(1, 1), -1); },
             "max_edge_length");
       }},
  });
}
