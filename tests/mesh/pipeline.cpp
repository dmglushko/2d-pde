#include "support/check.hpp"
#include "support/mesh_checks.hpp"
#include "support/shapes.hpp"

#include "pde2d/geometry/polygon.hpp"
#include "pde2d/mesh/cdt.hpp"
#include "pde2d/mesh/chord_length_discretizer.hpp"
#include "pde2d/mesh/delaunay_flips.hpp"
#include "pde2d/mesh/ear_clipping.hpp"
#include "pde2d/mesh/laplacian_smoothing.hpp"
#include "pde2d/mesh/longest_edge_bisection.hpp"
#include "pde2d/mesh/mesher.hpp"
#include "pde2d/mesh/remesher.hpp"
#include "pde2d/mesh/ruppert.hpp"

#include <memory>
#include <numbers>

using namespace pde2d::mesh;
using namespace pde2d::test;

namespace {

constexpr double pi = std::numbers::pi;

auto chords() { return std::make_shared<ChordLengthDiscretizer>(); }
auto ears() { return std::make_shared<EarClipping>(); }
auto cdt() { return std::make_shared<Cdt>(); }
auto ruppert() { return std::make_shared<Ruppert>(); }
auto bisection() { return std::make_shared<LongestEdgeBisection>(); }
auto flips() { return std::make_shared<DelaunayFlips>(); }
auto smoothing() { return std::make_shared<LaplacianSmoothing>(); }

// A triangulator that declares no hole support; it delegates to ear
// clipping.
class NoHoles final : public InitialTriangulator {
public:
  std::string name() const override { return "no_holes"; }
  TriangulatorCapabilities capabilities() const override {
    return {.supports_holes = false, .supports_several_pieces = true};
  }
  WorkingMesh run(const BoundaryPolygon &p, StageReport &r) const override {
    return EarClipping().run(p, r);
  }
};

// A refiner that tolerates nothing, with a local improver.
class Strict final : public Refiner {
public:
  explicit Strict(std::shared_ptr<const LocalImprover> local)
      : local_(std::move(local)) {}
  std::string name() const override { return "strict"; }
  RefinerCapabilities capabilities() const override { return {}; }
  const LocalImprover *local_improver() const override { return local_.get(); }
  void run(WorkingMesh &, const MeshSpec &, StageReport &) const override {}

private:
  std::shared_ptr<const LocalImprover> local_;
};

// A local improver that moves nodes (does nothing here).
class Mover final : public LocalImprover {
public:
  std::string name() const override { return "mover"; }
  ImproverEffects effects() const override { return {.moves_nodes = true}; }
  std::size_t improve_around(WorkingMesh &,
                             std::span<const std::size_t>) const override {
    return 0;
  }
};

// True if p is a point of the closed segment ab, up to rounding.
bool on_chord(Point a, Point b, Point p) {
  double l = distance(a, b);
  double off = std::abs(cross(b - a, p - a)) / l;
  double along = dot(p - a, b - a) / (l * l);
  return off <= 1e-12 * l && along >= -1e-12 && along <= 1 + 1e-12;
}

} // namespace

int main() {
  return run({
      {"compatibility_checked_at_construction",
       [] {
         expect_throws<std::invalid_argument>(
             [] { Mesher m(chords(), ears(), ruppert(), {flips()}); },
             "ruppert already maintains Delaunay; delaunay_flips is "
             "redundant");
         expect_throws<std::invalid_argument>(
             [] { Remesher r(nullptr, {flips(), flips()}); },
             "delaunay_flips already maintains Delaunay");
         expect_throws<std::invalid_argument>(
             [] { Remesher r(std::make_shared<Strict>(flips()), {}); },
             "strict does not tolerate edge flips");
         expect_throws<std::invalid_argument>(
             [] {
               Remesher r(std::make_shared<Strict>(std::make_shared<Mover>()),
                          {});
             },
             "strict does not tolerate node moves");
         expect_throws<std::invalid_argument>([] { Mesher m(nullptr, ears()); },
                                              "boundary discretizer");
         expect_throws<std::invalid_argument>(
             [] { Remesher r(nullptr, {nullptr}); }, "null");
         // Allowed: a flip or move in between makes Delaunay flips useful
         // again, and bisection tolerates both kinds of local improver.
         Mesher ok1(chords(), ears(), ruppert(), {smoothing(), flips()});
         Mesher ok2(chords(), ears(), nullptr, {flips(), smoothing(), flips()});
         Mesher ok3(chords(), ears(),
                    std::make_shared<LongestEdgeBisection>(
                        LongestEdgeBisectionOptions{.local_improver = flips()}),
                    {});
       }},
      {"input_checked_when_run_starts",
       [] {
         Mesher mesher(chords(), std::make_shared<NoHoles>());
         // Constructing worked; the mismatch shows only with a hole.
         auto r = mesher.run(rectangle(1, 1), {0.5});
         check_valid_mesh(r.mesh);
         expect_throws<std::invalid_argument>(
             [&] { (void)mesher.run(annulus(1, 0.5), {0.5}); },
             "no_holes does not support holes; the geometry has 1");
         expect_throws<std::invalid_argument>(
             [&] { (void)mesher.run(rectangle(1, 1), {0}); },
             "max_edge_length must be finite and positive");
         expect_throws<std::invalid_argument>(
             [&] { (void)mesher.run(rectangle(1, 1), {INFINITY}); },
             "max_edge_length");
       }},
      {"target_met_is_checked_on_the_final_mesh",
       [] {
         // Without a refiner only the boundary is fine; interior edges of
         // the ear-clipped circle are long.
         auto r = Mesher(chords(), ears()).run(circle(1), {0.2});
         check(!r.target_met, "target missed");
         near(r.longest_edge, longest_edge(r.mesh), 0, "longest edge");
         check(r.longest_edge > 0.2, "longest edge above h");
         auto ok = Mesher(chords(), ears(), bisection()).run(circle(1), {0.2});
         check(ok.target_met, "with a refiner");
       }},
      {"holes_and_islands",
       [] {
         auto g = holes_and_island();
         for (auto r : {Mesher(chords(), cdt(), ruppert()).run(g, {0.3}),
                        Mesher(chords(), ears(), bisection(), {smoothing()})
                            .run(g, {0.3})}) {
           const Mesh &m = r.mesh;
           check_valid_mesh(m);
           check_area(m, holes_and_island_area(), 0.05);
           check(m.region_names == std::vector<std::string>{"outer", "hole",
                                                            "island",
                                                            "small_hole"},
                 "regions");
           // No triangle inside a hole (outside the island).
           for (const auto &t : m.triangles) {
             Point c = (1.0 / 3) * (m.nodes[t.nodes[0]] + m.nodes[t.nodes[1]] +
                                    m.nodes[t.nodes[2]]);
             bool in_circle = distance(c, {3, 3}) < 2 - 0.05;
             bool in_island = c.x > 2.5 && c.x < 3.5 && c.y > 2.5 && c.y < 3.5;
             bool in_small = c.x > 0.5 && c.x < 1.5 && c.y > 0.5 && c.y < 1.5;
             check(!(in_circle && !in_island) && !in_small,
                   "triangle centroid lies in the domain");
           }
         }
       }},
      {"several_pieces",
       [] {
         CurveGeometry g(
             {square_loop({0, 0}, 1, "a"), circle_loop({3, 0.5}, 0.5, "b")});
         auto r = Mesher(chords(), cdt(), ruppert()).run(g, {0.2});
         check_valid_mesh(r.mesh);
         check_area(r.mesh, 1 + pi / 4, 0.03);
       }},
      {"orientation_normalized_for_meshing",
       [] {
         // Outer loop clockwise, hole counterclockwise: both get fixed.
         Loop outer = square_loop({0, 0}, 2, "outer");
         Loop flipped;
         for (auto it = outer.curves.rbegin(); it != outer.curves.rend(); ++it)
           flipped.curves.push_back({reversed(it->curve), it->region});
         CurveGeometry g({flipped, square_loop({0.5, 0.5}, 1, "hole")});
         auto r = Mesher(chords(), cdt()).run(g, {0.5});
         check_valid_mesh(r.mesh);
         near(mesh_area(r.mesh), 3, 1e-12, "area 4 - 1");
       }},
      {"invalid_boundary_polygon_is_reported",
       [] {
         // The hole comes within 0.05 of the outer circle; at this edge
         // length the chords of the two circles cross.
         CurveGeometry g({circle_loop({0, 0}, 1, "outer"),
                          circle_loop({0, 0.05}, 0.9, "hole")});
         expect_throws<std::runtime_error>(
             [&] { (void)Mesher(chords(), cdt()).run(g, {1.0}); },
             "chord_length produced an invalid boundary polygon: Boundary "
             "polygon edges cross or touch");
         // A curve deviation bound fixes it.
         auto fine = std::make_shared<ChordLengthDiscretizer>(
             ChordLengthOptions{.max_curve_deviation = 0.01});
         auto r = Mesher(fine, cdt()).run(g, {1.0});
         check_valid_mesh(r.mesh);
       }},
      {"reports",
       [] {
         auto r = Mesher(chords(), ears(), bisection(), {flips(), smoothing()})
                      .run(rectangle(2, 1), {0.3});
         std::vector<std::string> stages;
         for (const auto &s : r.reports) {
           stages.push_back(s.stage);
           check(s.time.count() >= 0, "time recorded");
         }
         check(stages ==
                   std::vector<std::string>{
                       "chord_length", "ear_clipping", "longest_edge_bisection",
                       "delaunay_flips", "laplacian_smoothing"},
               "stage order");
         check(r.reports[0].metrics.at("points") ==
                   double(boundary_nodes(r.mesh).size()),
               "boundary points");
       }},
      {"deterministic",
       [] {
         auto g = holes_and_island();
         Mesher mesher(chords(), ears(), ruppert(), {smoothing(), flips()});
         check(
             same_mesh(mesher.run(g, {0.25}).mesh, mesher.run(g, {0.25}).mesh),
             "same input, same mesh");
       }},
      {"remesh_without_geometry_keeps_points_on_chords",
       [] {
         auto coarse = Mesher(chords(), cdt(), ruppert()).run(circle(1), {0.5});
         const Mesh &c = coarse.mesh;
         for (auto refiner : {std::shared_ptr<const Refiner>(ruppert()),
                              std::shared_ptr<const Refiner>(bisection())}) {
           auto fine = Remesher(refiner).run(c, {0.1});
           check_valid_mesh(fine.mesh);
           check(fine.target_met, "target met");
           check(fine.mesh.boundary_edges.size() > c.boundary_edges.size(),
                 "boundary was refined");
           near(mesh_area(fine.mesh), mesh_area(c), 1e-12,
                "same polygon, same area");
           for (auto v : boundary_nodes(fine.mesh)) {
             Point p = fine.mesh.nodes[v];
             bool found = false;
             for (const auto &e : c.boundary_edges)
               found = found ||
                       on_chord(c.nodes[e.nodes[0]], c.nodes[e.nodes[1]], p);
             check(found, "new boundary point on an original chord");
           }
         }
       }},
      {"remesh_with_geometry_puts_points_on_curves",
       [] {
         auto g = annulus(1, 0.5);
         auto coarse = Mesher(chords(), cdt(), ruppert()).run(g, {0.5});
         for (auto refiner : {std::shared_ptr<const Refiner>(ruppert()),
                              std::shared_ptr<const Refiner>(bisection())}) {
           auto fine = Remesher(refiner).run(coarse.mesh, g, {0.1});
           check_valid_mesh(fine.mesh);
           check(fine.target_met, "target met");
           check(fine.reports.front().stage == "curve_links", "first report");
           near(fine.reports.front().metrics.at("unlinked_edges"), 0, 0,
                "every edge linked");
           check(fine.mesh.boundary_edges.size() >
                     coarse.mesh.boundary_edges.size(),
                 "boundary refined");
           std::size_t inner = region_id(fine.mesh, "inner");
           for (const auto &e : fine.mesh.boundary_edges)
             for (auto v : e.nodes)
               near(norm(fine.mesh.nodes[v]), e.region == inner ? 0.5 : 1.0,
                    1e-12, "boundary node on its circle");
         }
       }},
      {"remesh_with_geometry_edge_across_curve_start",
       [] {
         // The geometry's circle starts at angle 0.1, inside a mesh edge:
         // that edge cannot be linked to one parameter interval.
         auto m = Mesher(chords(), cdt()).run(circle(1), {0.5}).mesh;
         auto rotated = circle(1).transformed({.rotation = 0.1});
         auto fine = Remesher(bisection()).run(m, rotated, {0.05});
         check_valid_mesh(fine.mesh);
         const auto &links = fine.reports.front();
         near(links.metrics.at("unlinked_edges"), 1, 0, "one edge");
         check(links.messages.size() == 1, "message about it");
         std::size_t off = 0;
         for (auto v : boundary_nodes(fine.mesh))
           if (std::abs(norm(fine.mesh.nodes[v]) - 1) > 1e-12)
             ++off;
         // Only the points inserted into that one chord are off the circle.
         check(off > 0, "points on the unlinked chord");
         check(off < fine.mesh.boundary_edges.size() / 10, "only those");
       }},
      {"remesh_rejects_mismatched_geometry",
       [] {
         auto m = Mesher(chords(), cdt()).run(circle(1), {0.5}).mesh;
         expect_throws<std::invalid_argument>(
             [&] { (void)Remesher(bisection()).run(m, circle(2), {0.1}); },
             "does not lie on a curve of that region");
         auto renamed = circle(1);
         renamed.rename_region("boundary", "wall");
         expect_throws<std::invalid_argument>(
             [&] { (void)Remesher(bisection()).run(m, renamed, {0.1}); },
             "mesh region 'boundary' does not exist in the geometry");
       }},
      {"remesh_rejects_invalid_mesh",
       [] {
         auto m = Mesher(chords(), cdt()).run(rectangle(1, 1), {0.5}).mesh;
         auto broken = m;
         broken.boundary_edges.pop_back();
         expect_throws<std::invalid_argument>(
             [&] { (void)Remesher(bisection()).run(broken, {0.1}); },
             "has no neighbour but is not a boundary edge");
         auto inverted = m;
         std::swap(inverted.triangles[0].nodes[0],
                   inverted.triangles[0].nodes[1]);
         expect_throws<std::invalid_argument>(
             [&] { (void)Remesher(bisection()).run(inverted, {0.1}); },
             "not counterclockwise");
       }},
  });
}
