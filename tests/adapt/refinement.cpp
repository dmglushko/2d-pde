#include "adapt/refinement_checks.hpp"
#include "support/check.hpp"
#include "support/mesh_checks.hpp"
#include "support/shapes.hpp"

#include "pde2d/adapt/newest_vertex_bisection.hpp"
#include "pde2d/adapt/red_green_blue_refinement.hpp"

#include <map>
#include <memory>
#include <stdexcept>
#include <vector>

using namespace pde2d::adapt;
using namespace pde2d::test;
using pde2d::mesh::BoundaryEdge;

namespace {

using Marked = std::vector<std::size_t>;

std::vector<std::shared_ptr<const RefinementMethod>> methods() {
  return {std::make_shared<NewestVertexBisection>(),
          std::make_shared<RedGreenBlueRefinement>()};
}

// The unit square as two triangles split by the diagonal 0-2, without
// links, refinement edges on the diagonal.
Mesh two_triangle_square() {
  Mesh m;
  m.nodes = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
  m.triangles = {{{1, 2, 0}}, {{3, 0, 2}}};
  m.region_names = {"bottom", "side"};
  m.boundary_edges = {BoundaryEdge{{0, 1}, 0, std::nullopt},
                      BoundaryEdge{{1, 2}, 1, std::nullopt},
                      BoundaryEdge{{2, 3}, 1, std::nullopt},
                      BoundaryEdge{{3, 0}, 1, std::nullopt}};
  return m;
}

void longest_edges() {
  Mesh m;
  // Edges: 0-1 has squared length 4, 1-2 and 2-0 both 10 (a tie between
  // the edges opposite nodes 0 and 1: node 0 wins).
  m.nodes = {{0, 0}, {2, 0}, {1, 3}, {5, 0}};
  m.triangles = {{{1, 2, 0}}, {{2, 0, 1}}, {{1, 3, 2}}};
  set_longest_edges_as_refinement_edges(m);
  check(m.triangles[0].nodes == std::array<std::size_t, 3>{0, 1, 2},
        "tie: the vertex with the smaller index first");
  check(m.triangles[1].nodes == std::array<std::size_t, 3>{0, 1, 2},
        "tie from another rotation");
  // (1, 3, 2): edge 3-2 is the longest (4^2 + 3^2 = 25), opposite node 1.
  check(m.triangles[2].nodes == std::array<std::size_t, 3>{1, 3, 2},
        "already first");
  Mesh bad;
  bad.nodes = {{0, 0}};
  bad.triangles = {{{0, 1, 2}}};
  expect_throws<std::invalid_argument>(
      [&] { set_longest_edges_as_refinement_edges(bad); },
      "triangle 0 uses a node index out of range");
}

// Marking one triangle of the two-triangle square: it is split into four,
// and its neighbour, whose refinement edge is the shared diagonal, is
// bisected once (green). Exact counts.
void closure_counts() {
  for (const auto &method : methods()) {
    const Mesh in = two_triangle_square();
    const Marked marked{0};
    auto r = method->refine(in, marked);
    check_refinement(in, r, marked);
    check(r.mesh.triangles.size() == 6, method->name() + ": 4 + 2 triangles");
    check(r.mesh.nodes.size() == 7, method->name() + ": three new nodes");
    check(r.mesh.boundary_edges.size() == 6, "two boundary edges split");
    near(r.report.metrics.at("refined_by_closure"), 1, 0, "one by closure");
    check(r.report.stage == method->name(), "report stage");
    check(r.report.messages.empty(), "no message");
    // The new nodes, in edge order: 0-1, 0-2, 1-2.
    check(r.node_parents[4] == std::array<std::size_t, 2>{0, 1} &&
              r.node_parents[5] == std::array<std::size_t, 2>{0, 2} &&
              r.node_parents[6] == std::array<std::size_t, 2>{1, 2},
          "new nodes in edge order");
    // The neighbour (3, 0, 2) bisected at the diagonal's midpoint 5:
    // (5, 3, 0) and (5, 2, 3), newest vertex first.
    check(r.mesh.triangles[4].nodes == std::array<std::size_t, 3>{5, 3, 0} &&
              r.mesh.triangles[5].nodes == std::array<std::size_t, 3>{5, 2, 3},
          "green children with the newest vertex first");
  }
  // The two methods differ only in the four children of triangle 0.
  const Mesh in = two_triangle_square();
  auto nvb = NewestVertexBisection().refine(in, Marked{0});
  auto rgb = RedGreenBlueRefinement().refine(in, Marked{0});
  // Triangle (1, 2, 0): m0 = 5 (on 2-0), m1 = 4 (on 0-1), m2 = 6 (on 1-2).
  using T = std::array<std::size_t, 3>;
  std::vector<T> red, bisected;
  for (std::size_t t = 0; t < 4; ++t) {
    red.push_back(rgb.mesh.triangles[t].nodes);
    bisected.push_back(nvb.mesh.triangles[t].nodes);
  }
  check(red == std::vector<T>{{1, 6, 4}, {6, 2, 5}, {4, 5, 0}, {5, 4, 6}},
        "red children, each listing the images of v0, v1, v2");
  check(bisected == std::vector<T>{{6, 5, 1}, {6, 2, 5}, {4, 5, 0}, {4, 1, 5}},
        "NVB: bisect, then bisect both children");
}

void nothing_marked() {
  for (const auto &method : methods()) {
    const Mesh in = initial_mesh(l_shape(), 0.6);
    auto r = method->refine(in, Marked{});
    check(same_mesh(r.mesh, in), "the same mesh");
    check_refinement(in, r, Marked{});
    check(r.report.messages.size() == 1 &&
              r.report.messages[0].find("nothing was marked") !=
                  std::string::npos,
          "a message says so");
  }
}

void bad_input() {
  for (const auto &method : methods()) {
    const Mesh in = two_triangle_square();
    expect_throws<std::invalid_argument>(
        [&] { (void)method->refine(in, Marked{2}); },
        "marked triangle 2 is out of range (the mesh has 2 triangles)");
    Mesh clockwise = in;
    clockwise.triangles[0].nodes = {0, 2, 1};
    expect_throws<std::invalid_argument>(
        [&] { (void)method->refine(clockwise, Marked{0}); },
        method->name() + ": the input mesh is not valid");
  }
}

// Many markings on the square and the L-shape (from the Mesher, with
// links on straight lines, and without links): every result passes the
// oracles, repetitions in the marked list do not matter, and the same
// input gives the same output.
void straight_domains() {
  std::map<std::size_t, std::size_t> pattern_counts; // children -> count
  for (const auto &method : methods())
    for (auto geometry :
         {CurveGeometry({square_loop({0, 0}, 2, "square")}), l_shape()})
      for (bool links : {true, false}) {
        Mesh in = initial_mesh(geometry, 0.5);
        if (!links) {
          in.chains.clear();
          for (auto &e : in.boundary_edges)
            e.link.reset();
        }
        for (std::size_t step : {1, 3, 7}) {
          Marked marked;
          for (std::size_t t = 0; t < in.triangles.size(); t += step)
            marked.push_back(t);
          auto r = method->refine(in, marked);
          check_refinement(in, r, marked);
          Marked doubled = marked;
          doubled.insert(doubled.end(), marked.rbegin(), marked.rend());
          check(same_result(r, method->refine(in, doubled)),
                "repetitions and order of marks do not matter");
          check(same_result(r, method->refine(in, marked)), "deterministic");
          std::map<std::size_t, std::size_t> children;
          for (auto p : r.triangle_parents)
            ++children[p];
          for (auto [p, c] : children)
            ++pattern_counts[c];
        }
      }
  // Every pattern occurred: unrefined, one, two and three bisections (or
  // red).
  for (std::size_t c : {1, 2, 3, 4})
    check(pattern_counts[c] > 0,
          "a triangle with " + std::to_string(c) + " children occurred");
}

// The region tags of a polygon with several regions survive refinement.
void regions_kept() {
  CurveGeometry g({Loop{{{Line{{0, 0}, {3, 0}}, "bottom"},
                         {Line{{3, 0}, {3, 2}}, "right"},
                         {Line{{3, 2}, {0, 2}}, "top"},
                         {Line{{0, 2}, {0, 0}}, "left"}}}});
  for (const auto &method : methods()) {
    Mesh m = initial_mesh(g, 0.7);
    for (int cycle = 0; cycle < 3; ++cycle) {
      Marked all;
      for (std::size_t t = 0; t < m.triangles.size(); ++t)
        all.push_back(t);
      auto r = method->refine(m, all);
      check_refinement(m, r, all);
      m = r.mesh;
    }
    // Every boundary node of a region lies on its side.
    for (const auto &e : m.boundary_edges)
      for (auto v : e.nodes) {
        Point p = m.nodes[v];
        const std::string &region = m.region_names[e.region];
        bool on = (region == "bottom" && p.y == 0) ||
                  (region == "right" && p.x == 3) ||
                  (region == "top" && p.y == 2) ||
                  (region == "left" && p.x == 0);
        check(on, "boundary node on the side of its region " + region);
      }
  }
}

} // namespace

int main() {
  return run({{"longest_edges", longest_edges},
              {"closure_counts", closure_counts},
              {"nothing_marked", nothing_marked},
              {"bad_input", bad_input},
              {"straight_domains", straight_domains},
              {"regions_kept", regions_kept}});
}
