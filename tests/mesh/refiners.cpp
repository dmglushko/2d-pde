#include "support/check.hpp"
#include "support/mesh_checks.hpp"
#include "support/shapes.hpp"

#include "pde2d/mesh/cdt.hpp"
#include "pde2d/mesh/chord_length_discretizer.hpp"
#include "pde2d/mesh/delaunay_flips.hpp"
#include "pde2d/mesh/ear_clipping.hpp"
#include "pde2d/mesh/longest_edge_bisection.hpp"
#include "pde2d/mesh/mesher.hpp"
#include "pde2d/mesh/newest_vertex_bisection.hpp"
#include "pde2d/mesh/red_green_blue_refinement.hpp"
#include "pde2d/mesh/ruppert.hpp"

#include <memory>
#include <numbers>

using namespace pde2d::mesh;
using namespace pde2d::test;

namespace {

constexpr double pi = std::numbers::pi;

MeshResult mesh_with(std::shared_ptr<const InitialTriangulator> triangulator,
                     std::shared_ptr<const Refiner> refiner,
                     const CurveGeometry &g, double h) {
  Mesher mesher(std::make_shared<ChordLengthDiscretizer>(), triangulator,
                refiner);
  return mesher.run(g, {.max_edge_length = {.absolute = h}});
}

// Boundary nodes of each region lie on the true curve: a circle of the
// given radius around `center`.
void check_on_circle(const Mesh &m, const std::string &region, Point center,
                     double radius) {
  std::size_t id = region_id(m, region);
  for (const auto &e : m.boundary_edges)
    if (e.region == id)
      for (auto v : e.nodes)
        near(distance(m.nodes[v], center), radius, 1e-12 * radius,
             "boundary node on the circle of region " + region);
}

void check_ruppert(const CurveGeometry &g, double h, double angle,
                   double exact_area, double area_tolerance) {
  for (auto triangulator :
       {std::shared_ptr<const InitialTriangulator>(
            std::make_shared<EarClipping>()),
        std::shared_ptr<const InitialTriangulator>(std::make_shared<Cdt>())}) {
    auto result = mesh_with(
        triangulator,
        std::make_shared<Ruppert>(RuppertOptions{.min_angle_degrees = angle}),
        g, h);
    const Mesh &m = result.mesh;
    check_valid_mesh(m);
    check_area(m, exact_area, area_tolerance);
    check(is_delaunay(m), "Ruppert output is constrained Delaunay");
    check(min_angle_degrees(m) >= angle - 1e-9, "min angle bound");
    check(result.target_met, "target met");
    check(longest_edge(m) <= h, "every edge at most max_edge_length");
    near(result.properties.longest_edge, longest_edge(m), 0,
         "reported longest edge");
  }
}

// Chord-length discretization at a fixed coarse edge length and without a
// deviation bound (the default), whatever the spec says, so that the refiner
// has to split most boundary edges.
class CoarseBoundary final : public BoundaryDiscretizer {
public:
  explicit CoarseBoundary(double h) : h_(h) {}
  std::string name() const override { return "coarse"; }
  BoundaryPolygon run(const CurveGeometry &g, const MeshSpec &,
                      StageReport &r) const override {
    return ChordLengthDiscretizer().run(
        g, {.max_edge_length = {.absolute = h_}}, r);
  }

private:
  double h_;
};

// The unit circle as four quarter arcs of one region: one chain.
CurveGeometry four_quarter_circle() {
  Loop loop;
  for (int i = 0; i < 4; ++i)
    loop.curves.push_back(
        {EllipticalArc{{0, 0}, 1, 1, 0, i * pi / 2, pi / 2}, "boundary"});
  return CurveGeometry({loop});
}

} // namespace

int main() {
  return run({
      {"ruppert_rectangle",
       [] { check_ruppert(rectangle(2, 1), 0.2, 30, 2, 1e-12); }},
      {"ruppert_l_shape", [] { check_ruppert(l_shape(), 0.3, 25, 3, 1e-12); }},
      {"ruppert_star",
       [] {
         // Shoelace area of the star's ten vertices.
         double area = 0;
         for (int k = 0; k < 10; ++k) {
           double r1 = k % 2 == 0 ? 1.0 : 0.45, r2 = k % 2 == 0 ? 0.45 : 1.0;
           area += 0.5 * r1 * r2 * std::sin(pi / 5);
         }
         check_ruppert(star(), 0.15, 20, area, 1e-12);
       }},
      {"ruppert_circle",
       [] {
         check_ruppert(circle(1), 0.15, 30, pi, 0.01);
         auto r = mesh_with(std::make_shared<Cdt>(),
                            std::make_shared<Ruppert>(), circle(1), 0.15);
         check_on_circle(r.mesh, "boundary", {0, 0}, 1);
       }},
      {"ruppert_annulus_curve_points",
       [] {
         check_ruppert(annulus(1, 0.4), 0.1, 28, pi * (1 - 0.16), 0.01);
         // A coarse boundary: most boundary points are inserted by the
         // refiner, and all of them lie on the circles.
         auto r = mesh_with(std::make_shared<EarClipping>(),
                            std::make_shared<Ruppert>(), annulus(1, 0.4), 0.5);
         check(r.mesh.boundary_edges.size() > 20, "refiner split boundary");
         check_on_circle(r.mesh, "outer", {0, 0}, 1);
         check_on_circle(r.mesh, "inner", {0, 0}, 0.4);
       }},
      {"bisection_run_starts_from_longest_edges",
       [] {
         // CDT stores its triangles' vertices in no order chosen for
         // bisection; run() first makes the longest edges the refinement
         // edges, so refinement keeps the smallest angle of the starting
         // mesh (from the CDT order it fell to about half of it here).
         const double start = min_angle_degrees(
             mesh_with(std::make_shared<Cdt>(), nullptr, l_shape(), 0.12).mesh);
         for (auto refiner :
              {std::shared_ptr<const Refiner>(
                   std::make_shared<NewestVertexBisection>()),
               std::shared_ptr<const Refiner>(
                   std::make_shared<RedGreenBlueRefinement>())}) {
           auto r =
               mesh_with(std::make_shared<Cdt>(), refiner, l_shape(), 0.12);
           check_valid_mesh(r.mesh);
           check(r.target_met, "target met");
           check(r.reports[3].metrics.at("refinement_edges_set") > 0,
                 "some triangles were rotated");
           check(min_angle_degrees(r.mesh) >= start - 1e-9,
                 "no angle below the starting mesh's smallest");
         }
       }},
      {"ruppert_holes_and_island",
       [] {
         check_ruppert(holes_and_island(), 0.4, 25, holes_and_island_area(),
                       0.05);
       }},
      {"ruppert_semicircle",
       [] {
         check_ruppert(rectangle_with_semicircle(3, 1), 0.2, 25, 3 + pi / 8,
                       0.02);
       }},
      {"ruppert_reports",
       [] {
         auto r = mesh_with(std::make_shared<EarClipping>(),
                            std::make_shared<Ruppert>(), rectangle(2, 1), 0.3);
         check(r.reports.size() == 4, "pipeline and three stages");
         check(r.reports[3].stage == "ruppert", "stage name");
         check(r.reports[3].metrics.at("initial_flips") > 0,
               "ruppert made the ear-clipped input Delaunay");
       }},
      {"ruppert_options",
       [] {
         expect_throws<std::invalid_argument>(
             [] { Ruppert(RuppertOptions{.min_angle_degrees = 40}); },
             "(0, 34]");
         // A very sharp corner cannot be fixed; the budget stops it.
         CurveGeometry sharp(
             {polygon_loop({{0, 0}, {4, 0}, {4, 0.5}}, "boundary")});
         expect_throws<std::runtime_error>(
             [&] {
               (void)mesh_with(std::make_shared<Cdt>(),
                               std::make_shared<Ruppert>(
                                   RuppertOptions{.max_points = 2000}),
                               sharp, 1);
             },
             "max_points = 2000");
       }},
      {"bisection_target",
       [] {
         for (const auto &g : {rectangle(2, 1), l_shape(), annulus(1, 0.4),
                               holes_and_island()}) {
           auto r = mesh_with(std::make_shared<EarClipping>(),
                              std::make_shared<LongestEdgeBisection>(), g, 0.2);
           check_valid_mesh(r.mesh);
           near(mesh_area(r.mesh), boundary_area(r.mesh),
                1e-12 * boundary_area(r.mesh), "area");
           check(r.target_met, "target met");
           check(longest_edge(r.mesh) <= 0.2, "every edge at most h");
         }
       }},
      {"bisection_with_local_flips_stays_delaunay",
       [] {
         // Local only: the mesh is Delaunay without a final pass.
         auto flips = std::make_shared<DelaunayFlips>(
             DelaunayFlipsOptions{.use = ImproverUse::local_only});
         Mesher mesher(std::make_shared<ChordLengthDiscretizer>(),
                       std::make_shared<Cdt>(),
                       std::make_shared<LongestEdgeBisection>(), {flips});
         auto r =
             mesher.run(l_shape(), {.max_edge_length = {.absolute = 0.15}});
         check(r.reports.size() == 4, "no final pass");
         check_valid_mesh(r.mesh);
         check(r.target_met, "target met");
         check(is_delaunay(r.mesh), "local flips after every split");
         const auto &m = r.reports[3].metrics;
         check(m.at("local_improver_changes") > 0,
               "the local improver did flip");
         check(m.at("local_improver_changes.delaunay_flips") ==
                   m.at("local_improver_changes"),
               "per-improver count");
       }},
      {"chain_splits_on_the_true_curve",
       [] {
         // Seven coarse chords over four quarter arcs: most chords span a
         // junction, and every split must still land on the circle.
         for (auto refiner :
              {std::shared_ptr<const Refiner>(std::make_shared<Ruppert>()),
               std::shared_ptr<const Refiner>(
                   std::make_shared<LongestEdgeBisection>())}) {
           Mesher mesher(std::make_shared<CoarseBoundary>(1.0),
                         std::make_shared<Cdt>(), refiner);
           auto r = mesher.run(four_quarter_circle(),
                               {.max_edge_length = {.absolute = 0.1}});
           check_valid_mesh(r.mesh);
           check(r.target_met, "target met");
           check(r.mesh.boundary_edges.size() > 60, "boundary refined");
           check_on_circle(r.mesh, "boundary", {0, 0}, 1);
           check_area(r.mesh, pi, 0.01);
         }
       }},
      {"coarse_hole_refined_without_inversions",
       [] {
         // The hole of the annulus starts as a triangle (sagitta 0.2), so
         // its curve points bulge far into the domain and many simple
         // splits would invert a triangle.
         double retriangulated = 0, deleted = 0;
         for (double inner : {0.4, 0.6})
           for (auto triangulator : {std::shared_ptr<const InitialTriangulator>(
                                         std::make_shared<EarClipping>()),
                                     std::shared_ptr<const InitialTriangulator>(
                                         std::make_shared<Cdt>())})
             for (auto refiner :
                  {std::shared_ptr<const Refiner>(std::make_shared<Ruppert>()),
                   std::shared_ptr<const Refiner>(
                       std::make_shared<LongestEdgeBisection>())})
               for (double h : {0.3, 0.1, 0.04}) {
                 Mesher mesher(std::make_shared<CoarseBoundary>(1.5),
                               triangulator, refiner);
                 auto r = mesher.run(annulus(1, inner),
                                     {.max_edge_length = {.absolute = h}});
                 check_valid_mesh(r.mesh);
                 check(r.target_met, "target met");
                 check_on_circle(r.mesh, "outer", {0, 0}, 1);
                 check_on_circle(r.mesh, "inner", {0, 0}, inner);
                 // The straight-edged mesh lies between the true area and
                 // the polygon through its boundary nodes.
                 check_area(r.mesh, pi * (1 - inner * inner), 4 * pi * h * h);
                 const auto &m = r.reports[3].metrics;
                 near(m.at("unsplittable_edges"), 0, 0, "every split worked");
                 retriangulated += m.at("retriangulated_splits");
                 deleted += m.at("deleted_nodes");
               }
         std::cout << "retriangulated " << retriangulated << ", deleted nodes "
                   << deleted << '\n';
         check(retriangulated > 0, "local re-triangulation was needed");
       }},
      {"regions_survive_refinement",
       [] {
         auto r = mesh_with(std::make_shared<Cdt>(),
                            std::make_shared<Ruppert>(), rectangle(2, 1), 0.1);
         const Mesh &m = r.mesh;
         double bottom = 0;
         for (const auto &e : m.boundary_edges) {
           Point a = m.nodes[e.nodes[0]], b = m.nodes[e.nodes[1]];
           const auto &name = m.region_names[e.region];
           if (name == "bottom") {
             near(a.y, -0.5, 0, "bottom edge on y = -0.5");
             near(b.y, -0.5, 0, "bottom edge on y = -0.5");
             bottom += distance(a, b);
           }
           if (name == "left")
             near(a.x, -1, 0, "left edge on x = -1");
         }
         near(bottom, 2, 1e-12, "bottom length");
       }},
  });
}
