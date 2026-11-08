// refine_mesh(result, marked, options): only the marked part of the mesh
// gets finer, the result is a valid conforming CDT, region tags survive and
// curved boundary vertices stay on their curves. Oracles: geometric
// invariants (exact coordinates of untouched triangles, |p| = 1 on the
// circle) and counts.
#include "discretization/geometry.hpp"
#include "discretization/triangulate.hpp"
#include "support/check.hpp"
#include <array>
#include <cmath>
#include <iostream>
#include <set>
#include <string>
#include <utility>
#include <vector>
using namespace pde2d::discretization;
using pde2d_test::check;
using pde2d_test::expect_throws;

namespace {
using Key = std::array<std::pair<double, double>, 3>;
Point2 centroid(const TriangleMesh &m, std::size_t i) {
  Point2 c;
  for (auto v : m.elements[i]) {
    c.x += m.nodes[v].x / 3;
    c.y += m.nodes[v].y / 3;
  }
  return c;
}
Key key(const TriangleMesh &m, std::size_t i) {
  Key k;
  for (int j = 0; j < 3; ++j)
    k[j] = {m.nodes[m.elements[i][j]].x, m.nodes[m.elements[i][j]].y};
  std::sort(k.begin(), k.end());
  return k;
}
template <class Predicate>
std::set<Key> triangles(const TriangleMesh &m, Predicate keep) {
  std::set<Key> out;
  for (std::size_t i = 0; i < m.elements.size(); ++i)
    if (keep(centroid(m, i)))
      out.insert(key(m, i));
  return out;
}
template <class Predicate>
std::vector<std::size_t> select(const TriangleMesh &m, Predicate keep) {
  std::vector<std::size_t> out;
  for (std::size_t i = 0; i < m.elements.size(); ++i)
    if (keep(centroid(m, i)))
      out.push_back(i);
  return out;
}
void check_tags(const TriangleMesh &m) {
  // RectangleGeometry(2, 1): bottom y=-0.5, right x=1, top y=0.5, left x=-1.
  check(m.boundary_tags.size() == m.boundary_edges.size(), "tags lost");
  for (std::size_t i = 0; i < m.boundary_edges.size(); ++i) {
    auto a = m.nodes[m.boundary_edges[i].first],
         b = m.nodes[m.boundary_edges[i].second];
    const auto &tag = m.boundary_tags[i];
    bool ok = (tag == "bottom" && a.y == -.5 && b.y == -.5) ||
              (tag == "top" && a.y == .5 && b.y == .5) ||
              (tag == "left" && a.x == -1 && b.x == -1) ||
              (tag == "right" && a.x == 1 && b.x == 1);
    check(ok, "boundary edge has the wrong region tag: " + tag);
  }
}

void only_marked_region_gets_finer() {
  MeshOptions o;
  o.boundary_treatment = BoundaryTreatment::SplitPolygonEdges;
  o.boundary.max_edge_length = .2;
  o.refinement.max_edge_length = .2;
  auto r = generate_mesh(RectangleGeometry(2, 1), o);
  check(r.report.size_satisfied);
  auto left = [](Point2 c) { return c.x < -.5; };
  auto right = [](Point2 c) { return c.x > 0; };
  const auto right_before = triangles(r.mesh, right);
  const auto left_before = select(r.mesh, left).size();
  auto current = r;
  for (int pass = 0; pass < 3; ++pass) {
    auto marked = select(current.mesh, left);
    auto next = refine_mesh(current, marked, current.options);
    check(next.report.size_satisfied, next.report.message);
    check(next.report.delaunay_verified && is_constrained_delaunay(next.mesh));
    validate_mesh_geometry(next.mesh);
    check_tags(next.mesh);
    check(next.report.marked.has_value());
    const auto &m = *next.report.marked;
    check(m.marked_elements == marked.size());
    check(m.marked_edges > 0 && m.marked_edges <= marked.size());
    check(m.bisected_edges + m.absorbed_edges == m.marked_edges,
          "every selected edge is bisected or absorbed");
    check(m.bisected_edges > m.marked_edges / 2);
    check(m.unrefinable_elements == 0);
    check(next.report.refinement_steps >= m.bisected_edges);
    current = std::move(next);
  }
  const auto left_after = select(current.mesh, left).size();
  std::cout << "left region: " << left_before << " -> " << left_after
            << " elements\n";
  check(left_after > 4 * left_before, "marked region did not get finer");
  check(triangles(current.mesh, right) == right_before,
        "unmarked region changed");
}

void curved_boundary_stays_on_circle() {
  MeshOptions o;
  o.boundary_treatment = BoundaryTreatment::RefineOnCurves;
  o.boundary.max_edge_length = .4;
  o.refinement.max_edge_length = .4;
  auto r = generate_mesh(CircleGeometry(1.0), o);
  check(r.report.size_satisfied);
  auto sector = [](Point2 c) { return c.x > .6; };
  auto boundary_nodes = [](const TriangleMesh &m, bool in_sector) {
    std::set<std::pair<double, double>> out;
    for (auto [a, b] : m.boundary_edges) {
      (void)b;
      if ((m.nodes[a].x > .2) == in_sector)
        out.insert({m.nodes[a].x, m.nodes[a].y});
    }
    return out;
  };
  const auto far_before = boundary_nodes(r.mesh, false);
  const auto near_before = boundary_nodes(r.mesh, true).size();
  auto current = r;
  for (int pass = 0; pass < 2; ++pass) {
    auto next = refine_mesh(current, select(current.mesh, sector), o);
    check(next.report.size_satisfied, next.report.message);
    validate_mesh_geometry(next.mesh);
    check(is_constrained_delaunay(next.mesh));
    check(next.boundary_context.has_value());
    check(next.mesh.boundary_tags.size() == next.mesh.boundary_edges.size());
    for (const auto &tag : next.mesh.boundary_tags)
      check(tag == "boundary");
    for (auto [a, b] : next.mesh.boundary_edges) {
      (void)b;
      auto p = next.mesh.nodes[a];
      check(std::abs(std::hypot(p.x, p.y) - 1) <= 1e-12,
            "boundary vertex left the circle");
    }
    check(next.report.marked->bisected_edges > 0);
    current = std::move(next);
  }
  check(boundary_nodes(current.mesh, true).size() > near_before,
        "marked boundary triangles did not split boundary edges");
  check(boundary_nodes(current.mesh, false) == far_before,
        "unmarked boundary changed");
}

void fixed_boundary_uses_interior_edges() {
  MeshOptions o;
  o.boundary.max_edge_length = .25;
  o.refinement.max_edge_length = .3;
  auto r = generate_mesh(RectangleGeometry(2, 1), o);
  check(r.report.size_satisfied);
  // Mark every triangle that touches the boundary.
  std::set<std::size_t> on_boundary;
  for (auto [a, b] : r.mesh.boundary_edges)
    on_boundary.insert({a, b});
  std::vector<std::size_t> marked;
  for (std::size_t i = 0; i < r.mesh.elements.size(); ++i)
    for (auto v : r.mesh.elements[i])
      if (on_boundary.contains(v)) {
        marked.push_back(i);
        break;
      }
  auto next = refine_mesh(r, marked, o);
  validate_mesh_geometry(next.mesh);
  check(next.mesh.boundary_edges.size() == r.mesh.boundary_edges.size(),
        "Fixed treatment split a boundary edge");
  check_tags(next.mesh);
  check(next.report.marked->bisected_edges > 0);
  check(next.mesh.elements.size() > r.mesh.elements.size());
  // A single triangle has only boundary edges: nothing can be split.
  auto single = generate_mesh(Boundary{{{0, 0}, {1, 0}, {0, 1}}, {}});
  auto none = refine_mesh(single, {0}, MeshOptions{});
  check(!none.report.size_satisfied);
  check(none.report.marked->unrefinable_elements == 1);
  check(none.report.message.find("no edge the boundary treatment allows") !=
            std::string::npos,
        none.report.message);
}

void empty_and_invalid_marks() {
  MeshOptions o;
  o.boundary_treatment = BoundaryTreatment::SplitPolygonEdges;
  o.refinement.max_edge_length = .3;
  auto r = generate_mesh(RectangleGeometry(2, 1), o);
  auto same = refine_mesh(r, {}, o);
  check(same.mesh.elements == r.mesh.elements);
  check(same.report.size_satisfied && same.report.refinement_steps == 0);
  check(same.report.marked->marked_elements == 0);
  // Duplicates count once.
  auto twice = refine_mesh(r, {3, 3, 3}, o);
  check(twice.report.marked->marked_elements == 1);
  check(twice.report.marked->bisected_edges == 1);
  expect_throws<std::invalid_argument>(
      [&] { refine_mesh(r, {r.mesh.elements.size()}, o); },
      "Marked element index is out of range");
  auto ear = o;
  ear.algorithm = InteriorAlgorithm::EarClipping;
  expect_throws<std::invalid_argument>([&] { refine_mesh(r, {0}, ear); },
                                       "constrained Delaunay");
  // The step budget bounds marked refinement as well.
  auto budget = o;
  budget.refinement.max_steps = 1;
  auto partial =
      refine_mesh(r, select(r.mesh, [](Point2) { return true; }), budget);
  check(!partial.report.size_satisfied);
  check(partial.report.message == "Refinement step budget exhausted");
}
} // namespace

int main() {
  return pde2d_test::run(
      {{"only_marked_region_gets_finer", only_marked_region_gets_finer},
       {"curved_boundary_stays_on_circle", curved_boundary_stays_on_circle},
       {"fixed_boundary_uses_interior_edges",
        fixed_boundary_uses_interior_edges},
       {"empty_and_invalid_marks", empty_and_invalid_marks}});
}
