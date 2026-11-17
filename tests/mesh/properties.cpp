// Mesh properties (measure) against an independent computation, and the
// warning angle of the Mesher and Remesher on both sides of the threshold.

#include "support/check.hpp"
#include "support/mesh_checks.hpp"
#include "support/shapes.hpp"

#include "pde2d/mesh/cdt.hpp"
#include "pde2d/mesh/chord_length_discretizer.hpp"
#include "pde2d/mesh/longest_edge_bisection.hpp"
#include "pde2d/mesh/mesh_properties.hpp"
#include "pde2d/mesh/mesher.hpp"
#include "pde2d/mesh/remesher.hpp"

#include <cmath>
#include <limits>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <vector>

using namespace pde2d::mesh;
using namespace pde2d::test;

namespace {

// Angles by the law of cosines, in long double: a different formula from
// the library's (atan2 of cross and dot).
struct Independent {
  double smallest_angle = 180, largest_angle = 0;
  double shortest_edge = 1e300, longest_edge = 0;
  std::size_t below = 0;
};

Independent independent(const Mesh &m, double warning) {
  Independent r;
  for (const auto &t : m.triangles) {
    long double len[3];
    for (int i = 0; i < 3; ++i) {
      Point a = m.nodes[t.nodes[(i + 1) % 3]],
            b = m.nodes[t.nodes[(i + 2) % 3]];
      len[i] = std::hypot(static_cast<long double>(a.x) - b.x,
                          static_cast<long double>(a.y) - b.y);
      r.shortest_edge = std::min(r.shortest_edge, static_cast<double>(len[i]));
      r.longest_edge = std::max(r.longest_edge, static_cast<double>(len[i]));
    }
    bool small = false;
    for (int i = 0; i < 3; ++i) {
      // The angle opposite edge i.
      long double b = len[(i + 1) % 3], c = len[(i + 2) % 3];
      long double cosine = (b * b + c * c - len[i] * len[i]) / (2 * b * c);
      cosine = std::clamp(cosine, -1.0L, 1.0L);
      double angle = static_cast<double>(std::acos(cosine) * 180 /
                                         std::numbers::pi_v<long double>);
      r.smallest_angle = std::min(r.smallest_angle, angle);
      r.largest_angle = std::max(r.largest_angle, angle);
      small = small || angle < warning;
    }
    r.below += small ? 1 : 0;
  }
  return r;
}

void agrees(const MeshProperties &p, const Mesh &m, double warning) {
  auto r = independent(m, warning);
  check(p.nodes == m.nodes.size() && p.triangles == m.triangles.size(),
        "counts");
  near(p.smallest_angle_degrees, r.smallest_angle, 1e-9, "smallest angle");
  near(p.largest_angle_degrees, r.largest_angle, 1e-9, "largest angle");
  near(p.shortest_edge, r.shortest_edge, 1e-14 * r.shortest_edge,
       "shortest edge");
  near(p.longest_edge, r.longest_edge, 1e-14 * r.longest_edge, "longest edge");
  near(p.warning_angle_degrees, warning, 0, "the warning angle is recorded");
  check(p.triangles_below_warning_angle == r.below, "count below warning");
}

// Two hand-made triangles: a right isosceles one (45, 45, 90) and a thin
// one with a 5.19 degree angle (atan(1 / 11)).
Mesh two_triangles() {
  Mesh m;
  m.nodes = {{0, 0}, {1, 0}, {0, 1}, {11, 0}};
  m.triangles = {{{0, 1, 2}}, {{1, 3, 2}}};
  m.region_names = {"r"};
  m.boundary_edges = {BoundaryEdge{{0, 1}, 0, std::nullopt},
                      BoundaryEdge{{1, 3}, 0, std::nullopt},
                      BoundaryEdge{{3, 2}, 0, std::nullopt},
                      BoundaryEdge{{2, 0}, 0, std::nullopt}};
  return m;
}

void hand_made() {
  const Mesh m = two_triangles();
  for (double warning : {0.0, 5.0, 10.0, 44.9, 45.0, 46.0, 60.0})
    agrees(measure(m, warning), m, warning);
  auto p = measure(m, 10);
  near(p.shortest_edge, 1, 0, "shortest edge 1");
  near(p.longest_edge, std::hypot(11.0, 1.0), 1e-15 * 11, "longest edge");
  near(p.smallest_angle_degrees, std::atan(1.0 / 11) * 180 / std::numbers::pi,
       1e-12, "the thin angle at node 3");
  check(p.triangles_below_warning_angle == 1, "one thin triangle");
  // The threshold on both sides: 45 is not below 45.
  check(measure(m, 45).triangles_below_warning_angle == 1,
        "the right triangle is not below 45");
  check(measure(m, 45.000001).triangles_below_warning_angle == 2,
        "both below just above 45");
  check(measure(m, 0).triangles_below_warning_angle == 0, "0 counts none");
}

void bad_input() {
  const Mesh m = two_triangles();
  for (double angle : {-1.0, 60.5, std::numeric_limits<double>::quiet_NaN()})
    expect_throws<std::invalid_argument>([&] { (void)measure(m, angle); },
                                         "must be in [0, 60] degrees");
  Mesh empty;
  expect_throws<std::invalid_argument>([&] { (void)measure(empty, 10); },
                                       "measure: the mesh has no triangles");
  Mesh bad = m;
  bad.triangles[1].nodes[1] = 9;
  expect_throws<std::invalid_argument>(
      [&] { (void)measure(bad, 10); },
      "measure: triangle 1 uses a node index out of range");
  expect_throws<std::invalid_argument>(
      [] {
        Mesher(std::make_shared<ChordLengthDiscretizer>(),
               std::make_shared<Cdt>(), nullptr, {},
               MesherOptions{.warning_angle_degrees = 70});
      },
      "MesherOptions::warning_angle_degrees must be in [0, 60] degrees");
  expect_throws<std::invalid_argument>(
      [] {
        Remesher(nullptr, {}, RemesherOptions{.warning_angle_degrees = -2});
      },
      "RemesherOptions::warning_angle_degrees must be in [0, 60] degrees");
}

bool warns(const StageReport &pipeline) {
  for (const auto &m : pipeline.messages)
    if (m.find("triangles have an angle below the warning angle") !=
        std::string::npos)
      return true;
  return false;
}

// The Mesher measures its final mesh once: the properties agree with an
// independent computation, target_met agrees with the longest edge, and
// the pipeline report warns exactly when triangles fall below the
// warning angle, on both sides of the mesh's smallest angle.
void mesher_warning_threshold() {
  auto make = [](double warning) {
    return Mesher(std::make_shared<ChordLengthDiscretizer>(),
                  std::make_shared<Cdt>(),
                  std::make_shared<LongestEdgeBisection>(), {},
                  MesherOptions{.warning_angle_degrees = warning});
  };
  const MeshSpec spec{.max_edge_length = {.absolute = 0.3}};
  auto r = make(default_warning_angle_degrees).run(l_shape(), spec);
  agrees(r.properties, r.mesh, 10);
  check(r.target_met == (longest_edge(r.mesh) <= r.max_edge_length),
        "target_met from the longest edge");
  check(warns(r.reports[0]) == (r.properties.triangles_below_warning_angle > 0),
        "a warning exactly when some triangle is below 10 degrees");
  const double smallest = r.properties.smallest_angle_degrees;
  // Just above the smallest angle: the triangle with it is counted.
  auto above = make(smallest + 1e-6).run(l_shape(), spec);
  check(above.properties.triangles_below_warning_angle >= 1, "counted");
  check(warns(above.reports[0]), "warned just above the smallest angle");
  agrees(above.properties, above.mesh, smallest + 1e-6);
  // Just below it: nothing.
  auto below = make(smallest - 1e-6).run(l_shape(), spec);
  check(below.properties.triangles_below_warning_angle == 0, "none counted");
  check(!warns(below.reports[0]), "no warning just below the smallest angle");
  check(same_mesh(above.mesh, below.mesh), "nothing is refused or changed");
}

// The Remesher's run and refine measure the same way.
void remesher_measures() {
  Mesh m = Mesher(std::make_shared<ChordLengthDiscretizer>(),
                  std::make_shared<Cdt>())
               .run(rectangle(2, 1), {.max_edge_length = {.absolute = 0.5}})
               .mesh;
  Remesher remesher(std::make_shared<LongestEdgeBisection>(), {},
                    RemesherOptions{.warning_angle_degrees = 30});
  auto run = remesher.run(m, MeshSpec{.max_edge_length = {.absolute = 0.2}});
  agrees(run.properties, run.mesh, 30);
  check(warns(run.reports[0]) ==
            (run.properties.triangles_below_warning_angle > 0),
        "run: warning consistent");
  std::vector<std::size_t> marked{0, 1, 2};
  auto refined = remesher.refine(m, marked);
  agrees(refined.properties, refined.mesh, 30);
  check(warns(refined.reports[0]) ==
            (refined.properties.triangles_below_warning_angle > 0),
        "refine: warning consistent");
}

} // namespace

int main() {
  return run({{"hand_made", hand_made},
              {"bad_input", bad_input},
              {"mesher_warning_threshold", mesher_warning_threshold},
              {"remesher_measures", remesher_measures}});
}
