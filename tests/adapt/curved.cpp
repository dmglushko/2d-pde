// Refinement near curved boundaries: linked boundary edges are split at
// their curve point, so the mesh approaches the curve.

#include "adapt/refinement_checks.hpp"
#include "support/check.hpp"
#include "support/mesh_checks.hpp"
#include "support/shapes.hpp"

#include "pde2d/adapt/dorfler_marking.hpp"
#include "pde2d/adapt/newest_vertex_bisection.hpp"
#include "pde2d/adapt/red_green_blue_refinement.hpp"
#include "pde2d/mesh/curve_chain.hpp"
#include "pde2d/mesh/ruppert.hpp"

#include <cmath>
#include <map>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <vector>

using namespace pde2d::adapt;
using namespace pde2d::test;
using pde2d::mesh::BoundaryEdge;
using pde2d::mesh::CurveChain;
using pde2d::mesh::CurveLink;

namespace {

constexpr double pi = std::numbers::pi;
using Marked = std::vector<std::size_t>;

std::vector<std::shared_ptr<const RefinementMethod>> methods() {
  return {std::make_shared<NewestVertexBisection>(),
          std::make_shared<RedGreenBlueRefinement>()};
}

CurveChain arc_chain(pde2d::geometry::EllipticalArc arc) {
  return CurveChain(std::vector<pde2d::geometry::Curve>{arc});
}

Marked all_triangles(const Mesh &m) {
  Marked all;
  for (std::size_t t = 0; t < m.triangles.size(); ++t)
    all.push_back(t);
  return all;
}

struct Circle {
  std::string region;
  Point center;
  double radius;
};

// New nodes: on the circle of their region if they split a boundary edge
// of a circle region, else the exact midpoint of their edge.
void check_new_nodes(const Mesh &in, const RefinementResult &r,
                     const std::vector<Circle> &circles) {
  std::map<std::pair<std::size_t, std::size_t>, std::size_t> boundary_region;
  for (const auto &e : in.boundary_edges)
    boundary_region[undirected(e.nodes[0], e.nodes[1])] = e.region;
  for (std::size_t v = in.nodes.size(); v < r.mesh.nodes.size(); ++v) {
    auto [a, b] = r.node_parents[v];
    Point p = r.mesh.nodes[v];
    auto it = boundary_region.find({a, b});
    const Circle *circle = nullptr;
    if (it != boundary_region.end())
      for (const auto &c : circles)
        if (in.region_names[it->second] == c.region)
          circle = &c;
    if (circle) {
      near(distance(p, circle->center), circle->radius, 1e-12,
           "a new boundary node lies on the circle " + circle->region);
    } else {
      Point m{0.5 * (in.nodes[a].x + in.nodes[b].x),
              0.5 * (in.nodes[a].y + in.nodes[b].y)};
      check(p == m, "a new node off the circles is the midpoint");
    }
  }
}

// Refines `cycles` times (all triangles, or a Dörfler marking of an
// indicator peaked at `focus`) and checks that the area moves towards the
// exact one at every cycle.
void refine_towards(const CurveGeometry &g, double h,
                    const std::vector<Circle> &circles, double exact_area,
                    int cycles, bool uniform, Point focus = {0, 0}) {
  for (const auto &method : methods()) {
    Mesh m = initial_mesh(g, h);
    double error = std::abs(mesh_area(m) - exact_area);
    DorflerMarking dorfler({.theta = 0.5});
    for (int cycle = 0; cycle < cycles; ++cycle) {
      Marked marked;
      if (uniform) {
        marked = all_triangles(m);
      } else {
        std::vector<double> eta;
        for (std::size_t t = 0; t < m.triangles.size(); ++t) {
          auto c = corners(m, t);
          Point centroid{(c[0].x + c[1].x + c[2].x) / 3,
                         (c[0].y + c[1].y + c[2].y) / 3};
          eta.push_back(std::sqrt(triangle_area(c[0], c[1], c[2])) /
                        (distance(centroid, focus) + 0.05));
        }
        marked = dorfler.mark(m, eta);
      }
      auto r = method->refine(m, marked);
      check_refinement(m, r, marked, {.straight = false});
      check_new_nodes(m, r, circles);
      const double next = std::abs(mesh_area(r.mesh) - exact_area);
      check(next <= error, method->name() + ": area moves towards the exact "
                                            "area");
      if (uniform)
        check(next < error, method->name() + ": strictly, when every "
                                             "boundary edge is split");
      std::cout << "INFO " << method->name()
                << (uniform ? " uniform" : " local") << " cycle " << cycle + 1
                << ": " << r.mesh.triangles.size() << " triangles, area error "
                << next << ", min angle " << min_angle_degrees(r.mesh) << '\n';
      error = next;
      m = r.mesh;
    }
  }
}

void disc() {
  CurveGeometry g({circle_loop({0, 0}, 1, "circle")});
  refine_towards(g, 0.4, {{"circle", {0, 0}, 1}}, pi, 4, true);
  refine_towards(g, 0.4, {{"circle", {0, 0}, 1}}, pi, 6, false, {0.9, 0});
}

void square_with_hole() {
  CurveGeometry g(
      {square_loop({0, 0}, 4, "outer"), circle_loop({2, 2}, 1, "hole")});
  refine_towards(g, 0.6, {{"hole", {2, 2}, 1}}, 16 - pi, 3, true);
  refine_towards(g, 0.6, {{"hole", {2, 2}, 1}}, 16 - pi, 6, false, {3, 2});
}

// Minimum angles over uniform cycles on the disc, from a Ruppert mesh
// (no triangle with two boundary edges on the smooth circle). From the
// plain CDT mesh, which has such triangles ("ears"), red refinement
// halves their smallest angle every cycle: the red child at the ear's
// middle vertex is three consecutive curve points (see
// red_green_blue_refinement.hpp).
void disc_quality_mesh() {
  for (const auto &method : methods()) {
    Mesh m = initial_mesh(CurveGeometry({circle_loop({0, 0}, 1, "circle")}),
                          0.4, std::make_shared<pde2d::mesh::Ruppert>());
    const double initial = min_angle_degrees(m);
    for (int cycle = 0; cycle < 4; ++cycle) {
      auto marked = all_triangles(m);
      auto r = method->refine(m, marked);
      check_refinement(m, r, marked, {.straight = false});
      m = r.mesh;
      std::cout << "INFO " << method->name() << " Ruppert disc cycle "
                << cycle + 1 << ": min angle " << min_angle_degrees(m)
                << " (initial " << initial << ")\n";
      check(min_angle_degrees(m) >= 0.5 * initial,
            method->name() + ": the minimum angle stays bounded below");
    }
  }
}

// The same disc without links (as if imported): chord midpoints, area
// preserved exactly.
void unlinked_disc() {
  for (const auto &method : methods()) {
    Mesh m =
        initial_mesh(CurveGeometry({circle_loop({0, 0}, 1, "circle")}), 0.4);
    m.chains.clear();
    for (auto &e : m.boundary_edges)
      e.link.reset();
    for (int cycle = 0; cycle < 2; ++cycle) {
      auto marked = all_triangles(m);
      auto r = method->refine(m, marked);
      check_refinement(m, r, marked); // straight: midpoints, area kept
      m = r.mesh;
    }
  }
}

// One triangle whose bottom edge 0 -> 1 is linked to an arc bulging into
// the domain, far past the opposite vertex: the curve point would need
// the policy's re-triangulation, so refinement throws.
void inward_bulge_throws() {
  Mesh m;
  m.nodes = {{0, 0}, {2, 0}, {1, 0.2}};
  m.triangles = {{{0, 1, 2}}};
  m.region_names = {"r"};
  // (0,0) over (1,1) to (2,0).
  m.chains = {arc_chain({{1, 0}, 1, 1, 0, pi, -pi})};
  m.boundary_edges = {
      BoundaryEdge{{0, 1}, 0, CurveLink{0, 0, m.chains[0].length()}},
      BoundaryEdge{{1, 2}, 0, std::nullopt},
      BoundaryEdge{{2, 0}, 0, std::nullopt}};
  set_longest_edges_as_refinement_edges(m);
  for (const auto &method : methods())
    expect_throws<std::runtime_error>(
        [&] { (void)method->refine(m, Marked{0}); },
        method->name() + ": splitting boundary edge 0 (nodes 0 -> 1) of "
                         "region 'r' at its curve point would invert a child "
                         "of triangle 0: the boundary split policy would "
                         "have to re-triangulate");
}

// Two pieces: the bottom edge of the upper triangle is linked to an arc
// bulging out of it, into the lower triangle. Every child is
// counterclockwise, but the refined mesh would overlap itself.
void outward_bulge_into_another_piece_throws() {
  Mesh m;
  m.nodes = {{0, 0}, {2, 0}, {1, 1}, {0, -3}, {2, -3}, {1, -0.5}};
  m.triangles = {{{0, 1, 2}}, {{3, 4, 5}}};
  m.region_names = {"r"};
  // (0,0) under (1,-1) to (2,0).
  m.chains = {arc_chain({{1, 0}, 1, 1, 0, pi, pi})};
  m.boundary_edges = {
      BoundaryEdge{{0, 1}, 0, CurveLink{0, 0, m.chains[0].length()}},
      BoundaryEdge{{1, 2}, 0, std::nullopt},
      BoundaryEdge{{2, 0}, 0, std::nullopt},
      BoundaryEdge{{3, 4}, 0, std::nullopt},
      BoundaryEdge{{4, 5}, 0, std::nullopt},
      BoundaryEdge{{5, 3}, 0, std::nullopt}};
  set_longest_edges_as_refinement_edges(m);
  pde2d::mesh::validate(m); // the input is valid
  for (const auto &method : methods()) {
    expect_throws<std::runtime_error>(
        [&] { (void)method->refine(m, Marked{0}); },
        method->name() + ": a curve point of a split boundary edge cannot be "
                         "placed by a plain split: the refined mesh would "
                         "not be valid");
    // Refining only the lower piece is fine.
    auto r = method->refine(m, Marked{1});
    check_refinement(m, r, Marked{1}, {.straight = false});
  }
}

void deterministic() {
  for (const auto &method : methods()) {
    Mesh m = initial_mesh(CurveGeometry({square_loop({0, 0}, 4, "outer"),
                                         circle_loop({2, 2}, 1, "hole")}),
                          0.6);
    Marked marked;
    for (std::size_t t = 0; t < m.triangles.size(); t += 2)
      marked.push_back(t);
    check(same_result(method->refine(m, marked), method->refine(m, marked)),
          "same input, same output");
  }
}

} // namespace

int main() {
  return run({{"disc", disc},
              {"square_with_hole", square_with_hole},
              {"disc_quality_mesh", disc_quality_mesh},
              {"unlinked_disc", unlinked_disc},
              {"inward_bulge_throws", inward_bulge_throws},
              {"outward_bulge_into_another_piece_throws",
               outward_bulge_into_another_piece_throws},
              {"deterministic", deterministic}});
}
