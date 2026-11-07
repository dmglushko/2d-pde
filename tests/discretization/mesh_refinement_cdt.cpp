#include "discretization/mesh_io.hpp"
#include "support/check.hpp"
#include <iostream>
#include <random>
using namespace pde2d::discretization;
using pde2d_test::check;
using pde2d_test::expect_throws;
TriangleMesh witness() {
  return {{{-.5, 0}, {.5, 0}, {0, .25}, {0, -.8125}},
          {{0, 1, 2}, {1, 0, 3}},
          {{0, 3}, {3, 1}, {1, 2}, {2, 0}},
          {"a", "b", "c", "d"}};
}
// Independent verifier: enumerate triangle pairs, derive each shared diagonal,
// and test the opposing vertex directly; no Topology or size heap is consulted.
void independent_cdt(const TriangleMesh &m) {
  for (std::size_t i = 0; i < m.elements.size(); ++i)
    for (std::size_t j = i + 1; j < m.elements.size(); ++j) {
      auto t = m.elements[i], u = m.elements[j];
      std::vector<std::size_t> shared;
      std::size_t c = 0, d = 0;
      for (auto a : t) {
        if (std::find(u.begin(), u.end(), a) != u.end())
          shared.push_back(a);
        else
          c = a;
      }
      if (shared.size() != 2)
        continue;
      for (auto a : u)
        if (a != shared[0] && a != shared[1])
          d = a;
      auto a = shared[0], b = shared[1];
      if (predicates::orient(m.nodes[c], m.nodes[d], m.nodes[a]) *
              predicates::orient(m.nodes[c], m.nodes[d], m.nodes[b]) >=
          0)
        continue;
      if (predicates::orient(m.nodes[a], m.nodes[b], m.nodes[c]) < 0)
        std::swap(a, b);
      check(predicates::incircle(m.nodes[a], m.nodes[b], m.nodes[c],
                                 m.nodes[d]) <= 0,
            "Independent CDT failure");
    }
}
void broad_phase() {
  using namespace mesh_detail;
  std::mt19937 rng(931);
  std::vector<Box> boxes;
  for (int i = 0; i < 1000; ++i) {
    double x = int(rng() % 100), y = int(rng() % 100);
    boxes.push_back({x, x + int(rng() % 5), y, y + int(rng() % 5)});
  }
  boxes.push_back({0, 0, 0, 0});
  boxes.push_back({0, 1, 0, 1}); // touching and degenerate
  BoxIndex index(boxes);
  for (std::size_t i = 0; i < boxes.size(); ++i) {
    std::vector<std::size_t> expected;
    for (std::size_t j = 0; j < boxes.size(); ++j)
      if (boxes[i].overlaps(boxes[j]))
        expected.push_back(j);
    check(index.candidates(boxes[i]) == expected,
          "Broad phase omitted contact");
  }
  expect_throws(
      [] { validate_polygon(Boundary{{{0, 0}, {2, 0}, {1, 0}, {1, 1}}, {}}); });
  expect_throws(
      [] { validate_polygon(Boundary{{{0, 0}, {2, 2}, {0, 2}, {2, 0}}, {}}); });
  auto m = witness();
  m.elements.push_back({0, 1, 2});
  expect_throws([&] { validate_mesh_geometry(m); });
}
void queue_keys() {
  using namespace mesh_detail;
  auto m = witness();
  Topology t(m);
  t.initialize_size(.1);
  auto old = t.generations.at({0, 1});
  t.remove(0);
  t.remove(1);
  t.add(0);
  t.add(1);
  check(t.generations.at({0, 1}) > old, "Recreated generation reused");
  auto heap = t.sizes;
  std::size_t stale = 0;
  while (!heap.empty()) {
    auto x = heap.top();
    heap.pop();
    if (t.generations.at(x.edge) != x.generation)
      ++stale;
  }
  check(stale > 0, "Stale/recreated heap entries were not exercised");
  t.legalize();
  independent_cdt(m);
  for (int step = 0; step < 100; ++step) {
    std::optional<Edge> expected;
    double length = .1 * (1 + 1e-12);
    for (const auto &[e, own] : t.owners) {
      (void)own;
      double d = distance(m.nodes[e.first], m.nodes[e.second]);
      if (d > length) {
        expected = e;
        length = d;
      }
    }
    auto actual = t.longest();
    check(actual == expected, "Heap disagrees with ordered full scan");
    if (!actual)
      break;
    auto e = *actual;
    t.split(e, {std::midpoint(m.nodes[e.first].x, m.nodes[e.second].x),
                std::midpoint(m.nodes[e.first].y, m.nodes[e.second].y)});
    t.legalize(false);
    check(t.pending.empty(), "Later legalization not drained");
    if (step % 10 == 0)
      independent_cdt(m);
  }
  TriangleMesh square{{{0, 0}, {1, 0}, {1, 1}, {0, 1}},
                      {{0, 1, 2}, {0, 2, 3}},
                      {{0, 1}, {1, 2}, {2, 3}, {3, 0}},
                      {}};
  Topology equal(square);
  equal.initialize_size(.1);
  equal.split({0, 2}, {.5, .5});
  equal.legalize(false);
  check(equal.longest() == std::optional<Edge>{{0, 1}},
        "Equal length tie order changed");
  // A third owner must be detected rather than silently discarded.
  auto nonmanifold = witness();
  nonmanifold.nodes.push_back({0, .5});
  nonmanifold.elements.push_back({0, 1, 4});
  expect_throws([&] { Topology invalid(nonmanifold); });
  expect_throws([&] { validate_mesh_geometry(nonmanifold); });
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
  return pde2d_test::run({{"broad_phase", broad_phase},
                          {"queue_keys", queue_keys},
                          {"witness_and_budgets", witness_and_budgets}});
}
