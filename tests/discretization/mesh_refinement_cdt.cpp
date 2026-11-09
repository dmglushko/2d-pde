// Constrained Delaunay refinement through the public meshing API. White-box
// checks of the private topology and broad phase are in
// internal/mesh_topology.cpp.
#include "discretization/mesh_io.hpp"
#include "support/cdt.hpp"
#include "support/check.hpp"
#include <cmath>
#include <iostream>
#include <map>
#include <numeric>
#include <optional>
#include <utility>
using namespace pde2d::discretization;
using pde2d_test::check;
using pde2d_test::expect_throws;
using pde2d_test::independent_cdt;
TriangleMesh witness() {
  return {{{-.5, 0}, {.5, 0}, {0, .25}, {0, -.8125}},
          {{0, 1, 2}, {1, 0, 3}},
          {{0, 3}, {3, 1}, {1, 2}, {2, 0}},
          {"a", "b", "c", "d"}};
}
void invalid_inputs() {
  // Collinear overlap and self-intersection are rejected before ear clipping.
  expect_throws([] {
    (void)generate_mesh(Boundary{{{0, 0}, {2, 0}, {1, 0}, {1, 1}}, {}});
  });
  expect_throws([] {
    (void)generate_mesh(Boundary{{{0, 0}, {2, 2}, {0, 2}, {2, 0}}, {}});
  });
  auto m = witness();
  m.elements.push_back({0, 1, 2});
  expect_throws([&] { validate_mesh_geometry(m); });
  // A third owner of an edge must be detected rather than silently discarded.
  auto nonmanifold = witness();
  nonmanifold.nodes.push_back({0, .5});
  nonmanifold.elements.push_back({0, 1, 4});
  expect_throws([&] { validate_mesh_geometry(nonmanifold); });
  MeshOptions o;
  o.refinement.max_edge_length = .1;
  expect_throws([&] { (void)refine_mesh(nonmanifold, o); });
}
// Longest edge of a mesh by full scan; equal lengths go to the first edge in
// lexicographic (sorted endpoint) order.
std::optional<std::pair<std::size_t, std::size_t>>
longest_edge(const TriangleMesh &m, double target) {
  std::map<std::pair<std::size_t, std::size_t>, bool> edges;
  for (auto t : m.elements)
    for (int e = 0; e < 3; ++e)
      edges[std::minmax(t[e], t[(e + 1) % 3])] = true;
  std::optional<std::pair<std::size_t, std::size_t>> best;
  double length = target * (1 + 1e-12);
  for (const auto &[e, present] : edges) {
    (void)present;
    const double d = std::hypot(m.nodes[e.first].x - m.nodes[e.second].x,
                                m.nodes[e.first].y - m.nodes[e.second].y);
    if (d > length) {
      best = e;
      length = d;
    }
  }
  return best;
}
// Each refinement step inserts the midpoint of the longest remaining edge,
// so a run with budget k+1 adds exactly that midpoint to the mesh of a run
// with budget k (runs are deterministic), and every result stays CDT.
void longest_edge_order() {
  MeshOptions o;
  o.boundary_treatment = BoundaryTreatment::SplitPolygonEdges;
  o.refinement.max_edge_length = .1;
  o.refinement.max_steps = 0;
  auto previous = refine_mesh(witness(), o);
  independent_cdt(previous.mesh);
  for (int budget = 1; budget <= 60; ++budget) {
    o.refinement.max_steps = budget;
    const auto next = refine_mesh(witness(), o);
    const auto expected = longest_edge(previous.mesh, .1);
    if (!expected) {
      check(next.report.size_satisfied &&
                next.mesh.nodes.size() == previous.mesh.nodes.size(),
            "Refinement continued after the size target was met");
      break;
    }
    check(next.report.refinement_steps == std::size_t(budget),
          "Unexpected refinement step count");
    check(next.mesh.nodes.size() == previous.mesh.nodes.size() + 1,
          "One step must insert one vertex");
    const auto a = previous.mesh.nodes[expected->first],
               b = previous.mesh.nodes[expected->second];
    const auto inserted = next.mesh.nodes.back();
    check(inserted.x == std::midpoint(a.x, b.x) &&
              inserted.y == std::midpoint(a.y, b.y),
          "Refinement disagrees with the ordered longest-edge scan");
    if (budget % 10 == 0)
      independent_cdt(next.mesh);
    previous = next;
  }
  // Equal-length tie: after the unit square's diagonal is split, the four
  // unit sides tie and the lexicographically first edge {0, 1} goes next.
  TriangleMesh square{{{0, 0}, {1, 0}, {1, 1}, {0, 1}},
                      {{0, 1, 2}, {0, 2, 3}},
                      {{0, 1}, {1, 2}, {2, 3}, {3, 0}},
                      {}};
  o.refinement.max_steps = 2;
  const auto tie = refine_mesh(square, o);
  check(tie.mesh.nodes.size() == 6 && tie.mesh.nodes[4].x == .5 &&
            tie.mesh.nodes[4].y == .5 && tie.mesh.nodes[5].x == .5 &&
            tie.mesh.nodes[5].y == 0,
        "Equal length tie order changed");
}
void witness_and_budgets() {
  MeshOptions o;
  o.boundary_treatment = BoundaryTreatment::SplitPolygonEdges;
  o.refinement.max_edge_length = 1;
  check(!is_constrained_delaunay(witness()), "Illegal witness accepted");
  for (int budget : {0, 1, 10}) {
    o.refinement.max_steps = budget;
    auto r = refine_mesh(witness(), o);
    independent_cdt(r.mesh);
    check(r.report.delaunay_verified, "CDT report lost");
    check(r.report.size_satisfied == (budget > 0),
          "Flip-created oversized edge missed");
    check(r.report.refinement_steps == std::size_t(budget > 0),
          "Unexpected witness steps");
    if (!budget)
      check(r.report.message == "Refinement step budget exhausted",
            "Budget message changed");
  }
  o.refinement.max_edge_length = .2;
  o.refinement.max_steps = 1;
  auto limited = refine_mesh(witness(), o);
  check(!limited.report.size_satisfied && limited.report.refinement_steps == 1,
        "Exhausted budget hidden");
  independent_cdt(limited.mesh);
  o.refinement.max_edge_length = 0;
  o.refinement.max_steps = 0;
  independent_cdt(refine_mesh(witness(), o).mesh);
  Boundary b{{{0, 0}, {.5, 0}, {1, 0}, {1, 1}, {0, 1}},
             {"a", "a", "b", "c", "d"}};
  o.refinement = {.125, 10000};
  auto a = generate_mesh(b, o), c = generate_mesh(b, o);
  check(a.mesh.elements == c.mesh.elements, "Nondeterministic refinement");
  independent_cdt(a.mesh);
  o.boundary_treatment = BoundaryTreatment::Fixed;
  o.refinement.max_edge_length = .5;
  check(!generate_mesh(b, o).report.size_satisfied,
        "Fixed boundary report lost");
}
int main() {
  return pde2d_test::run({{"invalid_inputs", invalid_inputs},
                          {"longest_edge_order", longest_edge_order},
                          {"witness_and_budgets", witness_and_budgets}});
}
