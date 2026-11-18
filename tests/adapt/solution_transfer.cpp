// Solution transfer by interpolation: a P1 function is reproduced exactly
// on a refinement, with parents (bisection) and without (flips and
// smoothing, by point location); a node on a curved boundary edge gets
// the average of its parent edge's ends (the old u_h at the chord
// midpoint), also without parents (its nearest old boundary point);
// velocities go the same way and Dirichlet nodes get the new problem's
// data; in a thin annulus a new curve point takes its own wall's value,
// not the nearer other wall's (through links, or by its own loop); bad
// input throws. The oracle evaluates the old u_h by brute force
// (every triangle, barycentric coordinates), not with the code under test.

#include "solve/solve_support.hpp"
#include "support/check.hpp"
#include "support/shapes.hpp"

#include "pde2d/adapt/solution_transfer.hpp"
#include "pde2d/fem/meshed_problem.hpp"
#include "pde2d/geometry/curve_geometry.hpp"
#include "pde2d/mesh/cdt.hpp"
#include "pde2d/mesh/chord_length_discretizer.hpp"
#include "pde2d/mesh/delaunay_flips.hpp"
#include "pde2d/mesh/laplacian_smoothing.hpp"
#include "pde2d/mesh/mesher.hpp"
#include "pde2d/mesh/newest_vertex_bisection.hpp"
#include "pde2d/mesh/red_green_blue_refinement.hpp"
#include "pde2d/mesh/remesher.hpp"
#include "pde2d/mesh/ruppert.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <vector>

using namespace pde2d;
using adapt::transfer_solution;
using fem::Expression;
using geometry::Point;
using pde2d::test::check;
using pde2d::test::expect_throws;
using pde2d::test::near;

namespace {

// The value at p of the P1 function `values` on `m`, by trying every
// triangle; nothing when p is in none (with a relative slack of 1e-12 in
// the barycentric coordinates).
std::optional<double> evaluate(const mesh::Mesh &m,
                               const std::vector<double> &values, Point p) {
  for (const auto &t : m.triangles) {
    const Point a = m.nodes[t.nodes[0]], b = m.nodes[t.nodes[1]],
                c = m.nodes[t.nodes[2]];
    const double area = cross(b - a, c - a);
    const double l0 = cross(b - p, c - p) / area;
    const double l1 = cross(c - p, a - p) / area;
    const double l2 = cross(a - p, b - p) / area;
    if (l0 >= -1e-12 && l1 >= -1e-12 && l2 >= -1e-12)
      return l0 * values[t.nodes[0]] + l1 * values[t.nodes[1]] +
             l2 * values[t.nodes[2]];
  }
  return std::nullopt;
}

std::shared_ptr<const mesh::Mesh> meshed(const geometry::CurveGeometry &g,
                                         double h) {
  const mesh::Mesher mesher(std::make_shared<mesh::ChordLengthDiscretizer>(),
                            std::make_shared<mesh::Cdt>(),
                            std::make_shared<mesh::Ruppert>());
  return std::make_shared<const mesh::Mesh>(
      mesher.run(g, {.max_edge_length = {.absolute = h}}).mesh);
}

std::shared_ptr<const mesh::Mesh> square() {
  return meshed(
      geometry::CurveGeometry({test::square_loop({0, 0}, 1, "boundary")}),
      0.25);
}

std::shared_ptr<const mesh::Mesh> disk() {
  return meshed(geometry::CurveGeometry({test::circle_loop({0, 0}, 1, "rim")}),
                0.4);
}

// A problem with no Dirichlet nodes (u_n = 0 everywhere), so every
// transferred value is the interpolant's.
std::shared_ptr<const fem::MeshedProblem>
natural_problem(std::shared_ptr<const mesh::Mesh> m, const char *region) {
  fem::Problem problem{
      .equation = {.a_xx = Expression("-1"),
                   .a_yy = Expression("-1"),
                   .m1 = Expression("1")},
      .boundary_conditions = {{region, {.beta_n = Expression("1")}}},
      .initial = {.u = Expression("0")}};
  return std::make_shared<const fem::MeshedProblem>(std::move(m), problem);
}

// A function that is not linear, and one that is.
double wavy(Point p) { return std::sin(3 * p.x + 1) * std::cos(2 * p.y) + p.x; }
double linear(Point p) { return 1.5 - 2 * p.x + 0.75 * p.y; }

solve::Solution sampled(const std::shared_ptr<const mesh::Mesh> &m,
                        double (*f)(Point), double time = 0.5) {
  solve::Solution s{.mesh = m, .time = time};
  for (const Point &p : m->nodes)
    s.values.push_back(f(p));
  return s;
}

std::vector<std::size_t> all_triangles(const mesh::Mesh &m) {
  std::vector<std::size_t> marked(m.triangles.size());
  for (std::size_t t = 0; t < marked.size(); ++t)
    marked[t] = t;
  return marked;
}

// The marked triangles: every third one, so refinement is local.
std::vector<std::size_t> some_triangles(const mesh::Mesh &m) {
  std::vector<std::size_t> marked;
  for (std::size_t t = 0; t < m.triangles.size(); t += 3)
    marked.push_back(t);
  return marked;
}

mesh::RefinementResult
refine(const mesh::Mesh &m, std::shared_ptr<const mesh::Refiner> refiner,
       const std::vector<std::size_t> &marked,
       std::vector<std::shared_ptr<const mesh::Improver>> improvers = {}) {
  return mesh::Remesher(std::move(refiner), std::move(improvers))
      .refine(m, marked);
}

// Inside the old mesh, the new u_h equals the old one: checked at three
// points of every new triangle that lie in the old mesh.
void check_same_function(const solve::Solution &old,
                         const solve::Solution &moved) {
  const mesh::Mesh &m = *moved.mesh;
  std::size_t compared = 0;
  for (const auto &t : m.triangles) {
    const Point a = m.nodes[t.nodes[0]], b = m.nodes[t.nodes[1]],
                c = m.nodes[t.nodes[2]];
    for (const auto &l :
         {std::array{1.0 / 3, 1.0 / 3, 1.0 / 3}, std::array{0.6, 0.3, 0.1},
          std::array{0.1, 0.15, 0.75}}) {
      const Point p = l[0] * a + l[1] * b + l[2] * c;
      const auto before = evaluate(*old.mesh, old.values, p);
      if (!before)
        continue; // a child sticking out of its curved parent
      const double after = l[0] * moved.values[t.nodes[0]] +
                           l[1] * moved.values[t.nodes[1]] +
                           l[2] * moved.values[t.nodes[2]];
      near(after, *before, 1e-12, "the new u_h is the old one");
      ++compared;
    }
  }
  check(compared > 3 * m.triangles.size() / 2, "most points compared");
}

void reproduces_with_parents() {
  for (const auto &refiner : std::vector<std::shared_ptr<const mesh::Refiner>>{
           std::make_shared<mesh::NewestVertexBisection>(),
           std::make_shared<mesh::RedGreenBlueRefinement>()}) {
    const auto old_mesh = square();
    mesh::RefinementResult r =
        refine(*old_mesh, refiner, some_triangles(*old_mesh));
    check(r.parents.has_value(), "bisection keeps the parents");
    const auto new_mesh = std::make_shared<const mesh::Mesh>(r.mesh);
    const auto problem = natural_problem(new_mesh, "boundary");
    const solve::Solution old = sampled(old_mesh, wavy);
    const adapt::SolutionTransfer moved =
        transfer_solution(old, *problem, &*r.parents);
    check(moved.solution.mesh == new_mesh, "on the new problem's mesh");
    check(moved.solution.time == 0.5, "at the old time");
    check(moved.solution.velocities.empty(), "no velocities without old ones");
    for (std::size_t v = 0; v < old_mesh->nodes.size(); ++v)
      check(moved.solution.values[v] == old.values[v],
            "old nodes keep their values exactly");
    check_same_function(old, moved.solution);
    check(moved.report.stage == "solution_transfer", "report stage");
    check(moved.report.metrics.at("by_parents") == 1.0, "by parents");
    check(moved.report.metrics.at("kept_nodes") ==
              static_cast<double>(old_mesh->nodes.size()),
          "kept nodes");
    check(moved.report.metrics.at("edge_nodes") ==
              static_cast<double>(new_mesh->nodes.size() -
                                  old_mesh->nodes.size()),
          "edge nodes");
  }
}

void reproduces_by_point_location() {
  const auto old_mesh = square();
  // Flips and smoothing make the parents untrue: nodes move.
  mesh::RefinementResult r =
      refine(*old_mesh, std::make_shared<mesh::NewestVertexBisection>(),
             all_triangles(*old_mesh),
             {std::make_shared<mesh::DelaunayFlips>(),
              std::make_shared<mesh::LaplacianSmoothing>()});
  check(!r.parents.has_value(), "the improvers dropped the parents");
  const auto new_mesh = std::make_shared<const mesh::Mesh>(r.mesh);
  const auto problem = natural_problem(new_mesh, "boundary");
  const solve::Solution old = sampled(old_mesh, wavy);
  const adapt::SolutionTransfer moved = transfer_solution(old, *problem);
  std::size_t moved_nodes = 0;
  for (std::size_t v = 0; v < new_mesh->nodes.size(); ++v) {
    const Point p = new_mesh->nodes[v];
    const auto expected = evaluate(*old_mesh, old.values, p);
    check(expected.has_value(), "a straight boundary: every node inside");
    near(moved.solution.values[v], *expected, 1e-12,
         "the old u_h at the new node");
    if (v < old_mesh->nodes.size() && !(p == old_mesh->nodes[v]))
      ++moved_nodes;
  }
  double moves = 0;
  for (const auto &report : r.reports)
    if (report.stage == "laplacian_smoothing")
      moves = report.metrics.at("moves");
  std::cout << "INFO smoothing moves " << moves << ", old nodes moved "
            << moved_nodes << '\n';
  check(moves > 0, "smoothing moved nodes");
  // A node at an old node's position keeps its value exactly.
  for (std::size_t v = 0; v < old_mesh->nodes.size(); ++v)
    if (new_mesh->nodes[v] == old_mesh->nodes[v])
      check(moved.solution.values[v] == old.values[v], "exact at old nodes");
  // A linear function is reproduced at every node.
  const adapt::SolutionTransfer lin =
      transfer_solution(sampled(old_mesh, linear), *problem);
  for (std::size_t v = 0; v < new_mesh->nodes.size(); ++v)
    near(lin.solution.values[v], linear(new_mesh->nodes[v]), 1e-13,
         "linear reproduced");
  check(moved.report.metrics.at("by_parents") == 0.0, "by point location");
  check(moved.report.metrics.at("located") +
                moved.report.metrics.at("by_link") +
                moved.report.metrics.at("nearest_boundary") ==
            static_cast<double>(new_mesh->nodes.size()),
        "every node accounted for");
}

// On the disk, a refined boundary edge's new node lies on the circle, off
// the old chord and outside the old mesh: it gets (u[a] + u[b]) / 2, the
// old u_h at the chord midpoint; without parents its nearest old boundary
// point is that midpoint too (exactly so on a circle, up to rounding).
void curved_boundary() {
  const auto old_mesh = disk();
  const auto marked = all_triangles(*old_mesh);
  // Red-green-blue splits every edge of a marked triangle, so every
  // boundary edge.
  const auto rgb = std::make_shared<mesh::RedGreenBlueRefinement>();
  mesh::RefinementResult with = refine(*old_mesh, rgb, marked);
  mesh::RefinementResult without = refine(
      *old_mesh, rgb, marked, {std::make_shared<mesh::LaplacianSmoothing>()});
  check(with.parents && !without.parents, "parents only without smoothing");
  const auto with_mesh = std::make_shared<const mesh::Mesh>(with.mesh);
  const auto without_mesh = std::make_shared<const mesh::Mesh>(without.mesh);
  check(with_mesh->nodes.size() == without_mesh->nodes.size(),
        "the same refinement");
  const solve::Solution old = sampled(old_mesh, linear);
  const auto a = transfer_solution(old, *natural_problem(with_mesh, "rim"),
                                   &*with.parents);
  const auto b = transfer_solution(old, *natural_problem(without_mesh, "rim"));
  std::vector<bool> boundary(with_mesh->nodes.size(), false);
  for (const auto &edge : with_mesh->boundary_edges)
    boundary[edge.nodes[0]] = boundary[edge.nodes[1]] = true;
  std::size_t curve_nodes = 0;
  for (std::size_t v = old_mesh->nodes.size(); v < with_mesh->nodes.size();
       ++v) {
    const auto [p, q] = with.parents->nodes[v];
    const double average = (old.values[p] + old.values[q]) / 2;
    check(a.solution.values[v] == average, "the parent edge's average");
    if (!boundary[v])
      continue;
    ++curve_nodes;
    const Point x = with_mesh->nodes[v];
    check(!evaluate(*old_mesh, old.values, x).has_value(),
          "a curve point outside the old mesh");
    const Point chord_middle = midpoint(old_mesh->nodes[p], old_mesh->nodes[q]);
    check(distance(x, chord_middle) > 1e-3, "off the old chord");
    near(average, linear(chord_middle), 1e-14,
         "the value at the chord midpoint, not at the curve point");
    near(without_mesh->nodes[v].x, x.x, 0.0, "boundary nodes do not move");
    near(b.solution.values[v], average, 1e-12,
         "without parents: the nearest old boundary point");
  }
  check(curve_nodes >= 8, "curve points were made");
  // Interior nodes without parents: the old u_h at their (moved) position.
  for (std::size_t v = 0; v < without_mesh->nodes.size(); ++v) {
    if (boundary[v])
      continue;
    const auto expected =
        evaluate(*old_mesh, old.values, without_mesh->nodes[v]);
    if (expected)
      near(b.solution.values[v], *expected, 1e-12, "interior: located");
  }
}

// Velocities go the same way; Dirichlet nodes get g and g_t of the new
// problem at the solution's time.
void velocities_and_dirichlet() {
  const auto old_mesh = square();
  mesh::RefinementResult r =
      refine(*old_mesh, std::make_shared<mesh::NewestVertexBisection>(),
             some_triangles(*old_mesh));
  const auto new_mesh = std::make_shared<const mesh::Mesh>(r.mesh);
  fem::Problem wave{.equation = {.a_xx = Expression("-1"),
                                 .a_yy = Expression("-1"),
                                 .m2 = Expression("1")},
                    .boundary_conditions = {{"boundary",
                                             {.alpha = Expression("1"),
                                              .g = Expression("x + t^2")}}},
                    .initial = {.u = Expression("0"), .u_t = Expression("0")}};
  const fem::MeshedProblem problem(new_mesh, wave);
  solve::Solution old = sampled(old_mesh, wavy, 0.75);
  for (const Point &p : old_mesh->nodes)
    old.velocities.push_back(linear(p) * 3);
  const auto moved = transfer_solution(old, problem, &*r.parents);
  solve::Solution only_velocities = sampled(old_mesh, wavy, 0.75);
  only_velocities.values = old.velocities;
  const auto reference =
      transfer_solution(only_velocities, problem, &*r.parents);
  check(moved.solution.velocities.size() == new_mesh->nodes.size(),
        "velocities on the new mesh");
  std::vector<bool> dirichlet(new_mesh->nodes.size(), false);
  for (const auto &d : problem.dirichlet_nodes())
    dirichlet[d.node] = true;
  for (std::size_t v = 0; v < new_mesh->nodes.size(); ++v) {
    if (dirichlet[v]) {
      near(moved.solution.values[v], new_mesh->nodes[v].x + 0.75 * 0.75, 1e-14,
           "Dirichlet value g(t)");
      near(moved.solution.velocities[v], 1.5, 1e-14, "Dirichlet rate g_t(t)");
    } else {
      check(moved.solution.velocities[v] == reference.solution.values[v],
            "velocities interpolated like values");
      near(moved.solution.velocities[v], 3 * linear(new_mesh->nodes[v]), 1e-13,
           "a linear velocity reproduced");
    }
  }
  check(moved.report.metrics.at("dirichlet_nodes") ==
            static_cast<double>(problem.dirichlet_nodes().size()),
        "Dirichlet nodes counted");
  check(!problem.dirichlet_nodes().empty(), "there are Dirichlet nodes");
}

// A thin annulus, radii 1 and 1.02, both walls in the one region "wall":
// the inner circle (a hole, so clockwise) in 12 chords of 30 degrees, the
// outer in 24 of 15 (their chords stay outside radius 1.02 cos 7.5 deg =
// 1.011), with curve links. Each inner chord's triangle has the outer node
// at its middle angle as vertex 0, so the chord is its refinement edge.
mesh::Mesh thin_annulus() {
  const geometry::CurveGeometry g({test::circle_loop({0, 0}, 1.02, "wall"),
                                   test::circle_loop({0, 0}, 1.0, "wall")});
  mesh::Mesh m;
  m.region_names = {"wall"};
  for (const auto &loop : g.loops())
    m.chains.emplace_back(std::vector<geometry::Curve>{loop.curves[0].curve});
  const auto add_loop = [&](std::size_t chain, std::size_t pieces) {
    const std::size_t first = m.nodes.size();
    const double length = m.chains[chain].length();
    const auto s = [&](std::size_t k) {
      return k == pieces ? length
                         : length * static_cast<double>(k) /
                               static_cast<double>(pieces);
    };
    for (std::size_t k = 0; k < pieces; ++k)
      m.nodes.push_back(m.chains[chain].point_at(s(k)));
    for (std::size_t k = 0; k < pieces; ++k)
      m.boundary_edges.push_back(
          {.nodes = {first + k, first + (k + 1) % pieces},
           .region = 0,
           .link =
               mesh::CurveLink{.chain = chain, .s0 = s(k), .s1 = s(k + 1)}});
  };
  add_loop(0, 24); // outer: nodes 0..23, counterclockwise
  add_loop(1, 12); // inner: nodes 24..35
  // The outer node in the direction of p.
  const auto outer_towards = [&](Point p) {
    std::size_t best = 0;
    for (std::size_t j = 1; j < 24; ++j)
      if (dot(m.nodes[j], p) > dot(m.nodes[best], p))
        best = j;
    return best;
  };
  const auto add = [&](std::size_t a, std::size_t b, std::size_t c) {
    // Counterclockwise, a first.
    if (cross(m.nodes[b] - m.nodes[a], m.nodes[c] - m.nodes[a]) < 0)
      std::swap(b, c);
    m.triangles.push_back({{a, b, c}});
  };
  for (const auto &edge : m.boundary_edges) {
    const auto [p, q] = edge.nodes;
    if (p < 24) {
      // An outer edge: with the inner node at the angle of its end on a
      // multiple of 30 degrees (outer node e is at 15 e degrees, inner node
      // 24 + k at -30 k).
      const std::size_t e = p % 2 == 0 ? p : q;
      add(p, q, 24 + (12 - e / 2) % 12);
    } else {
      // An inner chord: with the outer node at its middle.
      add(outer_towards(m.nodes[p] + m.nodes[q]), p, q);
    }
  }
  return m;
}

void thin_annulus_walls() {
  mesh::Mesh annulus = thin_annulus();
  mesh::validate(annulus);
  // The inner chord from (1, 0) clockwise, and its triangle.
  const std::size_t a = 24, b = 25;
  std::size_t marked = annulus.triangles.size();
  for (std::size_t t = 0; t < annulus.triangles.size(); ++t) {
    const auto &v = annulus.triangles[t].nodes;
    if (std::ranges::count(v, a) + std::ranges::count(v, b) == 2)
      marked = t;
  }
  check(marked < annulus.triangles.size(), "the inner chord's triangle");
  const auto old_mesh = std::make_shared<const mesh::Mesh>(annulus);
  mesh::RefinementResult r = refine(
      *old_mesh, std::make_shared<mesh::NewestVertexBisection>(), {marked});
  check(r.mesh.nodes.size() == old_mesh->nodes.size() + 1, "one new node");
  const std::size_t n = old_mesh->nodes.size();
  const Point x = r.mesh.nodes[n];
  near(std::hypot(x.x, x.y), 1.0, 1e-14, "the new node is on the inner circle");
  // The premise: it is nearer the outer polygon (its node at the same
  // angle, 0.02 away) than its own chord (1 - cos 15 deg = 0.034).
  const Point chord_middle = midpoint(old_mesh->nodes[a], old_mesh->nodes[b]);
  double to_outer = std::numeric_limits<double>::infinity();
  for (std::size_t j = 0; j < 24; ++j)
    to_outer = std::min(to_outer, distance(x, old_mesh->nodes[j]));
  check(to_outer < 0.021 && distance(x, chord_middle) > 0.034,
        "nearer the other wall than its own chord");

  // u = 0 on the inner wall, 1 on the outer.
  const auto values = [](const mesh::Mesh &m) {
    solve::Solution s;
    for (const Point &p : m.nodes)
      s.values.push_back(std::hypot(p.x, p.y) > 1.01 ? 1.0 : 0.0);
    return s;
  };
  const auto check_transfer = [&](std::shared_ptr<const mesh::Mesh> before,
                                  std::shared_ptr<const mesh::Mesh> after,
                                  bool linked) {
    solve::Solution old = values(*before);
    old.mesh = before;
    const auto moved = transfer_solution(old, *natural_problem(after, "wall"));
    check(moved.solution.values[n] == 0.0, "the inner wall's value");
    for (std::size_t v = 0; v < n; ++v)
      check(moved.solution.values[v] == old.values[v], "old nodes keep theirs");
    const auto &m = moved.report.metrics;
    check(m.at("by_parents") == 0.0, "no parents");
    check(m.at("by_link") == (linked ? 37.0 : 0.0), "boundary nodes by link");
    check(m.at("other_loop_nearer") == (linked ? 0.0 : 1.0),
          "the nearest point overall was on the other wall");
  };
  const auto new_mesh = std::make_shared<const mesh::Mesh>(r.mesh);
  check(!new_mesh->chains.empty() && new_mesh->boundary_edges[0].link,
        "the refinement keeps the links");
  check_transfer(old_mesh, new_mesh, true);
  // Without links: by the inner loop.
  const auto strip = [](mesh::Mesh m) {
    m.chains.clear();
    for (auto &edge : m.boundary_edges)
      edge.link.reset();
    return std::make_shared<const mesh::Mesh>(std::move(m));
  };
  check_transfer(strip(*old_mesh), strip(*new_mesh), false);
}

void rejects_bad_input() {
  const auto old_mesh = square();
  mesh::RefinementResult r =
      refine(*old_mesh, std::make_shared<mesh::NewestVertexBisection>(),
             some_triangles(*old_mesh));
  const auto problem =
      natural_problem(std::make_shared<const mesh::Mesh>(r.mesh), "boundary");
  const solve::Solution good = sampled(old_mesh, wavy);

  solve::Solution no_mesh = good;
  no_mesh.mesh = nullptr;
  expect_throws<std::invalid_argument>(
      [&] { (void)transfer_solution(no_mesh, *problem); }, "has no mesh");
  solve::Solution short_values = good;
  short_values.values.pop_back();
  expect_throws<std::invalid_argument>(
      [&] { (void)transfer_solution(short_values, *problem); }, "values for");
  solve::Solution nan_value = good;
  nan_value.values[2] = std::numeric_limits<double>::quiet_NaN();
  expect_throws<std::invalid_argument>(
      [&] { (void)transfer_solution(nan_value, *problem); },
      "values that are not finite");
  solve::Solution short_velocities = good;
  short_velocities.velocities.assign(3, 0.0);
  expect_throws<std::invalid_argument>(
      [&] { (void)transfer_solution(short_velocities, *problem); },
      "velocities for");

  mesh::Parents wrong_size = *r.parents;
  wrong_size.nodes.pop_back();
  expect_throws<std::invalid_argument>(
      [&] { (void)transfer_solution(good, *problem, &wrong_size); },
      "the parents are for");
  mesh::Parents reversed = *r.parents;
  const std::size_t last = reversed.nodes.size() - 1;
  reversed.nodes[last] = {reversed.nodes[last][1], reversed.nodes[last][0]};
  expect_throws<std::invalid_argument>(
      [&] { (void)transfer_solution(good, *problem, &reversed); },
      "not two old node indices");
  mesh::Parents out_of_range = *r.parents;
  out_of_range.nodes[0] = {0, old_mesh->nodes.size()};
  expect_throws<std::invalid_argument>(
      [&] { (void)transfer_solution(good, *problem, &out_of_range); },
      "not two old node indices");
}

} // namespace

int main() {
  return test::run({
      {"a P1 function is reproduced through the parents",
       reproduces_with_parents},
      {"a P1 function is reproduced by point location",
       reproduces_by_point_location},
      {"a curved boundary node gets its parent edge's average",
       curved_boundary},
      {"velocities like values; Dirichlet nodes from the new problem",
       velocities_and_dirichlet},
      {"a thin annulus: a curve point keeps to its own wall",
       thin_annulus_walls},
      {"bad input throws", rejects_bad_input},
  });
}
