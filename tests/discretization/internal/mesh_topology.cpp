// White-box checks of private meshing data structures (mesh_detail). This
// directory is the only place tests may use *_detail names; see AGENTS.md.
// The same behavior is checked through the public API, where it is
// observable, in ../mesh_refinement_cdt.cpp.
#include "discretization/triangulate.hpp"
#include "support/cdt.hpp"
#include "support/check.hpp"
#include <random>
using namespace pde2d::discretization;
using pde2d_test::check;
using pde2d_test::expect_throws;
using pde2d_test::independent_cdt;
namespace {
TriangleMesh witness() {
  return {{{-.5, 0}, {.5, 0}, {0, .25}, {0, -.8125}},
          {{0, 1, 2}, {1, 0, 3}},
          {{0, 3}, {3, 1}, {1, 2}, {2, 0}},
          {"a", "b", "c", "d"}};
}
void box_index() {
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
}
} // namespace
int main() {
  return pde2d_test::run(
      {{"box_index", box_index}, {"queue_keys", queue_keys}});
}
