#include "support/check.hpp"
#include "support/shapes.hpp"

#include "pde2d/mesh/chord_length_discretizer.hpp"

#include <numbers>

using namespace pde2d::mesh;
using namespace pde2d::test;

namespace {

constexpr double pi = std::numbers::pi;

BoundaryPolygon run_discretizer(const CurveGeometry &g, double h,
                                ChordLengthOptions o = {}) {
  StageReport report;
  return ChordLengthDiscretizer(o).run(g, {h}, report);
}

void check_chords(const BoundaryPolygon &p, double h) {
  for (const auto &loop : p.loops)
    for (std::size_t i = 0; i < loop.points.size(); ++i) {
      Point a = loop.points[i], b = loop.points[(i + 1) % loop.points.size()];
      check(distance(a, b) <= h, "chord at most max_edge_length");
      const auto &link = loop.edges[i].link;
      check(link.has_value(), "linked");
      const auto &curve = p.curves[link->curve];
      near(distance(point_at(curve, link->t0), a), 0, 1e-15, "t0 at a");
      if (link->t1 < 1)
        near(distance(point_at(curve, link->t1), b), 0, 1e-15, "t1 at b");
    }
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
         auto p =
             run_discretizer(circle(1), 10, {.max_curve_deviation = 0.001});
         double w = 2 * pi / double(p.loops[0].points.size());
         check(w * w / 8 <= 0.001, "deviation bound met");
         // The real sagitta is smaller than the bound.
         check(1 - std::cos(w / 2) <= 0.001, "sagitta");
         check_chords(p, 10);
         expect_throws<std::invalid_argument>(
             [] { ChordLengthDiscretizer({.max_curve_deviation = 0}); },
             "must be positive");
       }},
      {"default_curve_deviation",
       [] {
         check(ChordLengthOptions{}.max_curve_deviation == 0.02, "0.02");
         // A huge h: only the default deviation bound sets the count, so
         // the sweep w of each piece has w^2 / 8 <= 0.02.
         auto p = run_discretizer(circle(1), 100);
         double w = 2 * pi / double(p.loops[0].points.size());
         check(w * w / 8 <= 0.02, "default bound met");
         check(p.loops[0].points.size() > 3, "more than a triangle");
       }},
      {"at_least_three_points",
       [] {
         auto p =
             run_discretizer(circle(1), 100, {.max_curve_deviation = INFINITY});
         check(p.loops[0].points.size() == 3, "a triangle");
         validate(p);
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
