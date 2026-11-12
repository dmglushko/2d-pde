#include "support/check.hpp"
#include "support/mesh_checks.hpp"
#include "support/shapes.hpp"

#include "pde2d/mesh/cdt.hpp"
#include "pde2d/mesh/chord_length_discretizer.hpp"
#include "pde2d/mesh/ear_clipping.hpp"

#include <memory>

using namespace pde2d::mesh;
using namespace pde2d::test;

namespace {

BoundaryPolygon discretize(const CurveGeometry &g, double h) {
  StageReport report;
  auto polygon = ChordLengthDiscretizer().run(
      g, {.max_edge_length = {.absolute = h}}, report);
  validate(polygon);
  return polygon;
}

// A triangulation of a polygon with n points, H holes and P pieces, without
// added points, has n + 2H - 2P triangles (Euler's formula).
std::size_t expected_triangles(const BoundaryPolygon &p) {
  std::size_t n = 0, holes = 0, pieces = 0;
  for (const auto &loop : p.loops) {
    n += loop.points.size();
    (loop.depth % 2 ? holes : pieces) += 1;
  }
  return n + 2 * holes - 2 * pieces;
}

void check_triangulation(const InitialTriangulator &triangulator,
                         const CurveGeometry &g, double h, double expected_area,
                         bool delaunay) {
  auto polygon = discretize(g, h);
  StageReport report;
  Mesh m = triangulator.run(polygon, report).to_mesh();
  check_valid_mesh(m);
  check(
      m.nodes.size() ==
          [&] {
            std::size_t n = 0;
            for (const auto &l : polygon.loops)
              n += l.points.size();
            return n;
          }(),
      "no points added");
  check(m.triangles.size() == expected_triangles(polygon),
        "triangle count from Euler's formula");
  check(m.boundary_edges.size() == m.nodes.size(),
        "every polygon edge is a boundary edge");
  near(mesh_area(m), boundary_area(m), 1e-12 * boundary_area(m),
       "area inside the boundary");
  if (expected_area > 0)
    near(mesh_area(m), expected_area, 1e-12 * expected_area, "exact area");
  if (delaunay)
    check(is_delaunay(m), "constrained Delaunay");
  check(report.metrics.at("triangles") == double(m.triangles.size()), "metric");
}

void for_each_shape(const InitialTriangulator &t, bool delaunay) {
  check_triangulation(t, rectangle(2, 1), 0.25, 2, delaunay);
  check_triangulation(t, l_shape(), 0.3, 3, delaunay);
  check_triangulation(t, star(), 0.2, 0, delaunay);
  check_triangulation(t, circle(1), 0.1, 0, delaunay);
  check_triangulation(t, annulus(1, 0.4), 0.1, 0, delaunay);
  check_triangulation(t, holes_and_island(), 0.3, 0, delaunay);
  check_triangulation(t, rectangle_with_semicircle(3, 1), 0.2, 0, delaunay);
  check_triangulation(
      t,
      CurveGeometry({square_loop({0, 0}, 1, "a"), square_loop({2, 0}, 1, "b")}),
      0.2, 2, delaunay);
  // Many holes in a row, so bridges must avoid the other holes.
  std::vector<Loop> loops{
      polygon_loop({{0, 0}, {10, 0}, {10, 2}, {0, 2}}, "o")};
  for (int k = 0; k < 8; ++k)
    loops.push_back(square_loop({0.5 + 1.2 * k, 0.5}, 0.8, "h"));
  check_triangulation(t, CurveGeometry(loops), 0.4, 20 - 8 * 0.64, delaunay);
}

} // namespace

int main() {
  return run({
      {"ear_clipping_shapes", [] { for_each_shape(EarClipping(), false); }},
      {"cdt_shapes", [] { for_each_shape(Cdt(), true); }},
      {"ear_clipping_metrics",
       [] {
         StageReport report;
         auto m =
             EarClipping().run(discretize(holes_and_island(), 0.5), report);
         near(report.metrics.at("pieces"), 2, 0, "outer piece and island");
         near(report.metrics.at("bridges"), 2, 0, "two holes bridged");
       }},
      {"cdt_keeps_regions_and_links",
       [] {
         StageReport report;
         auto polygon = discretize(annulus(1, 0.5), 0.2);
         WorkingMesh w = Cdt().run(polygon, report);
         for (const auto &e : w.boundary_edges()) {
           check(e.link.has_value(), "every edge keeps its curve link");
           Point a = w.nodes()[e.nodes[0]];
           double r = norm(a);
           bool inner = w.region_names()[e.region] == "inner";
           near(r, inner ? 0.5 : 1.0, 1e-12, "region matches the circle");
         }
       }},
      {"deterministic",
       [] {
         auto polygon = discretize(holes_and_island(), 0.2);
         StageReport r1, r2;
         check(same_mesh(Cdt().run(polygon, r1).to_mesh(),
                         Cdt().run(polygon, r2).to_mesh()),
               "cdt");
         check(same_mesh(EarClipping().run(polygon, r1).to_mesh(),
                         EarClipping().run(polygon, r2).to_mesh()),
               "ear clipping");
       }},
  });
}
