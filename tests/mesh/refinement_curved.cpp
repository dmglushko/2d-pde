// Refinement of marked triangles near curved boundaries: linked boundary
// edges are split at their curve point (by the boundary split policy), so
// the mesh approaches the curve.

#include "mesh/refinement_checks.hpp"
#include "support/check.hpp"
#include "support/mesh_checks.hpp"
#include "support/shapes.hpp"

#include "pde2d/mesh/curve_chain.hpp"
#include "pde2d/mesh/ruppert.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <vector>

using namespace pde2d::mesh;
using namespace pde2d::test;

namespace {

constexpr double pi = std::numbers::pi;

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
    auto [a, b] = r.parents->nodes[v];
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
  for (const auto &refiner : bisection_refiners()) {
    Mesh m = initial_mesh(g, h);
    double error = std::abs(mesh_area(m) - exact_area);
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
        // The triangles with at least half the largest indicator.
        const double largest = *std::max_element(eta.begin(), eta.end());
        for (std::size_t t = 0; t < eta.size(); ++t)
          if (eta[t] >= 0.5 * largest)
            marked.push_back(t);
      }
      auto r = refine_with(refiner, m, marked);
      check_refinement(m, r, marked,
                       {.straight = false,
                        .marked_children = marked_children(*refiner)});
      check_new_nodes(m, r, circles);
      const double next = std::abs(mesh_area(r.mesh) - exact_area);
      // Up to rounding: a step that splits no boundary edge keeps the area,
      // summed over other triangles.
      check(next <= error + 1e-12,
            refiner->name() + ": area moves towards the exact area");
      // Red-green-blue with every triangle marked splits every boundary
      // edge (the bisection refiners split only one edge per triangle).
      if (uniform && refiner->name() == "red_green_blue")
        check(next < error, refiner->name() + ": strictly, when every "
                                             "boundary edge is split");
      std::cout << "INFO " << refiner->name()
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
  for (const auto &refiner : bisection_refiners()) {
    Mesh m = initial_mesh(CurveGeometry({circle_loop({0, 0}, 1, "circle")}),
                          0.4, std::make_shared<pde2d::mesh::Ruppert>());
    const double initial = min_angle_degrees(m);
    for (int cycle = 0; cycle < 4; ++cycle) {
      auto marked = all_triangles(m);
      auto r = refine_with(refiner, m, marked);
      check_refinement(m, r, marked,
                       {.straight = false,
                        .marked_children = marked_children(*refiner)});
      m = r.mesh;
      std::cout << "INFO " << refiner->name() << " Ruppert disc cycle "
                << cycle + 1 << ": min angle " << min_angle_degrees(m)
                << " (initial " << initial << ")\n";
      check(min_angle_degrees(m) >= 0.5 * initial,
            refiner->name() + ": the minimum angle stays bounded below");
    }
  }
}

// The same disc without links (as if imported): chord midpoints, area
// preserved exactly.
void unlinked_disc() {
  for (const auto &refiner : bisection_refiners()) {
    Mesh m =
        initial_mesh(CurveGeometry({circle_loop({0, 0}, 1, "circle")}), 0.4);
    m.chains.clear();
    for (auto &e : m.boundary_edges)
      e.link.reset();
    for (int cycle = 0; cycle < 2; ++cycle) {
      auto marked = all_triangles(m);
      auto r = refine_with(refiner, m, marked);
      // Straight: midpoints, area kept.
      check_refinement(m, r, marked,
                       {.marked_children = marked_children(*refiner)});
      m = r.mesh;
    }
  }
}

// One triangle whose bottom edge 0 -> 1 is linked to an arc bulging into
// the domain, far past the opposite vertex (a boundary node), so the
// boundary split policy cannot split the edge. It stays,
// the report says so, and nothing is refined (the edge is the bisection
// edge of the only triangle).
void inward_bulge_past_the_mesh() {
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
  for (const auto &refiner : bisection_refiners()) {
    auto r = refine_with(refiner, m, Marked{0});
    check(same_mesh(r.mesh, m), refiner->name() + ": nothing changed");
    check(r.parents.has_value(), "parents kept");
    const auto &report = refiner_report(r);
    near(report.metrics.at("unsplittable_edges"), 1, 0, "one unsplittable");
    check(!report.messages.empty() &&
              report.messages[0].find(
                  "boundary edge (0, 1) of region 'r' cannot be split: the "
                  "curve bulges in past another boundary node") != std::string::npos,
          refiner->name() + ": the report says why");
  }
}

// The bottom edge a -> b of triangle (c, a, b) is linked to an arc bulging
// into the domain, to (1, 0.8): that point lies in the neighbouring
// triangle, outside the angle at c, so the boundary split policy
// re-triangulates (inserts the point, forces a-p and p-b, cuts off what
// lies below them). The parents are dropped, and both reports say so.
void retriangulating_split_drops_parents() {
  Mesh m;
  m.nodes = {{0, 0}, {2, 0}, {2.2, 0.4}, {1, 2}};
  // (c, a, b): its refinement edge is a-b.
  m.triangles = {{{2, 0, 1}}, {{0, 2, 3}}};
  m.region_names = {"r"};
  // The circle through (0, 0), (1, 0.8), (2, 0): centre (1, -0.225).
  const double k = 0.225, radius = std::hypot(1.0, k);
  m.chains = {arc_chain(
      {{1, -k}, radius, radius, 0, pi - std::atan(k), -(pi - 2 * std::atan(k))})};
  m.boundary_edges = {
      BoundaryEdge{{0, 1}, 0, CurveLink{0, 0, m.chains[0].length()}},
      BoundaryEdge{{1, 2}, 0, std::nullopt},
      BoundaryEdge{{2, 3}, 0, std::nullopt},
      BoundaryEdge{{3, 0}, 0, std::nullopt}};
  validate(m);
  for (const auto &refiner :
       {std::shared_ptr<const Refiner>(std::make_shared<NewestVertexBisection>()),
        std::shared_ptr<const Refiner>(
            std::make_shared<RedGreenBlueRefinement>())}) {
    auto r = refine_with(refiner, m, Marked{0});
    check_valid_mesh(r.mesh);
    check(!r.parents, refiner->name() + ": parents dropped");
    const auto &report = refiner_report(r);
    check(report.metrics.at("retriangulated_splits") >= 1, "re-triangulated");
    bool said = false;
    for (const auto &text : report.messages)
      said = said || text.find("parents dropped: the boundary split policy "
                               "re-triangulated around boundary edge (0, 1) "
                               "of region 'r'") != std::string::npos;
    check(said, "the refiner's report says why");
    said = false;
    for (const auto &text : r.reports[0].messages)
      said = said || text.find("parents dropped: " + refiner->name() +
                               ": it could not keep them in this run") !=
                         std::string::npos;
    check(said, "the pipeline report names the refiner");
    // The new boundary point is on the arc.
    bool on_arc = false;
    for (const auto &e : r.mesh.boundary_edges)
      for (auto v : e.nodes)
        if (v >= m.nodes.size())
          on_arc = on_arc ||
                   std::abs(distance(r.mesh.nodes[v], {1, -k}) - radius) < 1e-12;
    check(on_arc, "the new boundary node lies on the arc");
  }
}

// Two pieces: the bottom edge of the upper triangle is linked to an arc
// bulging out of it, into the lower triangle. The policy splits it
// directly (every triangle stays counterclockwise), but the refined mesh
// would overlap itself, which the Remesher refuses.
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
  validate(m); // the input is valid
  for (const auto &refiner : bisection_refiners()) {
    expect_throws<std::runtime_error>(
        [&] { (void)refine_with(refiner, m, Marked{0}); },
        "Remesher::refine: the refined mesh is not valid");
    // Refining only the lower piece is fine.
    auto r = refine_with(refiner, m, Marked{1});
    check_refinement(m, r, Marked{1},
                     {.straight = false,
                      .marked_children = marked_children(*refiner)});
  }
}

// Red-green-blue at the ears of a plain CDT mesh of the disc (triangles
// with two boundary edges on the circle): the red child at the ear's
// middle vertex is three consecutive curve points, and its small angles
// halve with every cycle (see red_green_blue_refinement.hpp). The final
// mesh's properties catch it: the pipeline report warns once triangles
// fall below the warning angle, and the count agrees with an independent
// count.
void red_ears_warn() {
  Mesh m = initial_mesh(CurveGeometry({circle_loop({0, 0}, 1, "circle")}), 0.4);
  Remesher remesher(std::make_shared<RedGreenBlueRefinement>());
  bool warned = false;
  for (int cycle = 0; cycle < 5; ++cycle) {
    auto r = remesher.refine(m, all_triangles(m));
    std::size_t below = 0;
    for (std::size_t t = 0; t < r.mesh.triangles.size(); ++t) {
      auto c = corners(r.mesh, t);
      bool small = false;
      for (int i = 0; i < 3; ++i) {
        Point a = c[i], b = c[(i + 1) % 3], d = c[(i + 2) % 3];
        double ux = b.x - a.x, uy = b.y - a.y, vx = d.x - a.x, vy = d.y - a.y;
        small = small || std::atan2(std::abs(ux * vy - uy * vx),
                                    ux * vx + uy * vy) *
                                 180 / pi <
                             10;
      }
      below += small ? 1 : 0;
    }
    check(r.properties.triangles_below_warning_angle == below,
          "the count of triangles below 10 degrees");
    bool message = false;
    for (const auto &text : r.reports[0].messages)
      message = message ||
                text.find("triangles have an angle below the warning angle of "
                          "10 degrees") != std::string::npos;
    check(message == (below > 0), "a warning exactly when there are some");
    warned = warned || message;
    std::cout << "INFO red_green_blue CDT disc cycle " << cycle + 1
              << ": min angle " << r.properties.smallest_angle_degrees
              << ", below 10 degrees: " << below << '\n';
    m = r.mesh;
  }
  check(warned, "red refinement of the ears was warned about");
}

void deterministic() {
  for (const auto &refiner : bisection_refiners()) {
    Mesh m = initial_mesh(CurveGeometry({square_loop({0, 0}, 4, "outer"),
                                         circle_loop({2, 2}, 1, "hole")}),
                          0.6);
    Marked marked;
    for (std::size_t t = 0; t < m.triangles.size(); t += 2)
      marked.push_back(t);
    check(same_result(refine_with(refiner, m, marked), refine_with(refiner, m, marked)),
          "same input, same output");
  }
}

} // namespace

int main() {
  return run({{"disc", disc},
              {"square_with_hole", square_with_hole},
              {"disc_quality_mesh", disc_quality_mesh},
              {"unlinked_disc", unlinked_disc},
              {"inward_bulge_past_the_mesh", inward_bulge_past_the_mesh},
              {"retriangulating_split_drops_parents",
               retriangulating_split_drops_parents},
              {"outward_bulge_into_another_piece_throws",
               outward_bulge_into_another_piece_throws},
              {"red_ears_warn", red_ears_warn},
              {"deterministic", deterministic}});
}
