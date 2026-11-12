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

#include <algorithm>
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

// A refiner that tolerates nothing and does nothing; it records the names
// of the local improvers it was given.
class Strict final : public Refiner {
public:
  std::string name() const override { return "strict"; }
  RefinerCapabilities capabilities() const override { return {}; }
  void run(WorkingMesh &, const MeshSpec &,
           std::span<const LocalImprover *const> local,
           StageReport &report) const override {
    for (const auto *l : local)
      report.messages.push_back("local " + l->name());
  }
};

// An improver with a local version and declared effects; it does nothing
// but count its calls.
class Probe final : public Improver, public LocalImprover {
public:
  Probe(std::string name, ImproverEffects effects, ImproverUse use)
      : name_(std::move(name)), effects_(effects), use_(use) {}
  std::string name() const override { return name_; }
  ImproverEffects effects() const override { return effects_; }
  ImproverUse use() const override { return use_; }
  const LocalImprover *as_local_improver() const override { return this; }
  void run(WorkingMesh &, const MeshSpec &, StageReport &) const override {}
  std::size_t improve_around(WorkingMesh &,
                             std::span<const std::size_t>) const override {
    return 1;
  }

private:
  std::string name_;
  ImproverEffects effects_;
  ImproverUse use_;
};

auto probe(std::string name, ImproverEffects effects, ImproverUse use) {
  return std::make_shared<Probe>(std::move(name), effects, use);
}

auto flips(ImproverUse use) {
  return std::make_shared<DelaunayFlips>(DelaunayFlipsOptions{.use = use});
}

bool has_message(const std::vector<std::string> &messages,
                 const std::string &text) {
  return std::find(messages.begin(), messages.end(), text) != messages.end();
}

std::vector<std::string> stage_names(const MeshResult &r) {
  std::vector<std::string> stages;
  for (const auto &s : r.reports)
    stages.push_back(s.stage);
  return stages;
}

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
      {"improver_modes_resolved_at_construction",
       [] {
         using U = ImproverUse;
         // Ruppert maintains Delaunay: flips cannot be local there.
         Mesher m1(chords(), ears(), ruppert(), {flips()});
         check(m1.plan().local_improvers.empty(), "no local improver");
         check(m1.plan().final_passes == std::vector<std::size_t>{0},
               "flips as the final pass");
         check(has_message(m1.plan().warnings,
                           "delaunay_flips: ruppert already maintains "
                           "Delaunay, so it runs only as a final pass"),
               "local use refused");
         check(has_message(m1.plan().warnings,
                           "delaunay_flips: the mesh is already Delaunay "
                           "after ruppert, so this final pass changes "
                           "nothing"),
               "redundant final pass");
         // local_only with nothing to run it: it does not run at all.
         Remesher r1(nullptr, {flips(U::local_only)});
         check(r1.plan().final_passes.empty(), "not a final pass");
         check(has_message(r1.plan().warnings,
                           "delaunay_flips: there is no refiner to run it "
                           "locally, so it does not run at all"),
               "no refiner");
         // Smoothing has no local version.
         Remesher r2(
             bisection(),
             {std::make_shared<LaplacianSmoothing>(
                 LaplacianSmoothingOptions{.use = U::local_and_final})});
         check(has_message(r2.plan().warnings,
                           "laplacian_smoothing: it has no local version, "
                           "so it runs only as a final pass"),
               "no local version");
         // A refiner that tolerates nothing refuses flips and moves.
         auto strict = std::make_shared<Strict>();
         Remesher r3(strict,
                     {probe("flipper", {.flips_edges = true}, U::local_only),
                      probe("mover", {.moves_nodes = true}, U::local_only),
                      probe("counter", {}, U::local_and_final)});
         check(has_message(r3.plan().warnings,
                           "flipper: strict does not tolerate edge flips, so "
                           "it does not run at all"),
               "flips refused");
         check(has_message(r3.plan().warnings,
                           "mover: strict does not tolerate node moves, so it "
                           "does not run at all"),
               "moves refused");
         check(r3.plan().local_improvers == std::vector<std::size_t>{2},
               "an improver without effects is accepted");
         check(r3.plan().final_passes == std::vector<std::size_t>{2},
               "and runs as a final pass too");
         // Defaults: smoothing is final only, flips local and final.
         check(LaplacianSmoothingOptions{}.use == U::final_only, "smoothing");
         check(DelaunayFlipsOptions{}.use == U::local_and_final, "flips");
         Remesher r4(bisection(), {flips(), smoothing(), flips()});
         check(r4.plan().local_improvers == std::vector<std::size_t>{0, 2},
               "local improvers in list order");
         check(r4.plan().final_passes == std::vector<std::size_t>{0, 1, 2},
               "final passes in list order");
         check(r4.plan().warnings.empty(), "nothing to warn about");
         // Only configurations that cannot work at all still throw.
         expect_throws<std::invalid_argument>([] { Mesher m(nullptr, ears()); },
                                              "boundary discretizer");
         expect_throws<std::invalid_argument>(
             [] { Remesher r(nullptr, {nullptr}); }, "null");
       }},
      {"pipeline_report",
       [] {
         auto strict = std::make_shared<Strict>();
         Mesher mesher(chords(), ears(), strict,
                       {probe("counter", {}, ImproverUse::local_and_final),
                        probe("mover", {.moves_nodes = true},
                              ImproverUse::local_and_final),
                        smoothing()});
         auto r = mesher.run(rectangle(1, 1), {0.5});
         check(stage_names(r) ==
                   std::vector<std::string>{"pipeline", "chord_length",
                                            "ear_clipping", "strict", "counter",
                                            "mover", "laplacian_smoothing"},
               "pipeline report once, first; final passes in list order");
         const auto &p = r.reports[0];
         check(p.messages == mesher.plan().warnings, "warnings as messages");
         check(p.messages.size() == 1, "one warning (mover)");
         near(p.metrics.at("local_improvers"), 1, 0, "one local improver");
         near(p.metrics.at("final_passes"), 3, 0, "three final passes");
         near(p.metrics.at("refiner.strict"), 1, 0, "the refiner");
         near(p.metrics.at("local.0.counter"), 0, 0, "local: improver 0");
         near(p.metrics.at("final.0.counter"), 0, 0, "final pass 0");
         near(p.metrics.at("final.1.mover"), 1, 0, "final pass 1");
         near(p.metrics.at("final.2.laplacian_smoothing"), 2, 0,
              "final pass 2");
         // The refiner received exactly the resolved local improvers.
         check(r.reports[3].messages ==
                   std::vector<std::string>{"local counter"},
               "local improvers passed to the refiner");
         // Every run copies the same report; so does a Remesher run.
         auto again = mesher.run(rectangle(1, 1), {0.5});
         check(again.reports[0].messages == p.messages, "copied per run");
         Remesher remesher(nullptr, {flips(), flips()});
         auto rr = remesher.run(r.mesh, {0.5});
         check(stage_names(rr) == std::vector<std::string>{"pipeline",
                                                           "delaunay_flips",
                                                           "delaunay_flips"},
               "remesher reports");
         check(rr.reports[0].messages.size() == 3,
               "two refused local uses and one redundant pass");
         auto rg = remesher.run(r.mesh, rectangle(1, 1), {0.5});
         check(rg.reports[0].stage == "pipeline" &&
                   rg.reports[1].stage == "curve_links",
               "pipeline, then curve links");
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
         // length and without a deviation bound the chords of the two
         // circles cross.
         CurveGeometry g({circle_loop({0, 0}, 1, "outer"),
                          circle_loop({0, 0.05}, 0.9, "hole")});
         auto coarse = std::make_shared<ChordLengthDiscretizer>(
             ChordLengthOptions{.max_curve_deviation = INFINITY});
         expect_throws<std::runtime_error>(
             [&] { (void)Mesher(coarse, cdt()).run(g, {1.0}); },
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
         for (const auto &s : r.reports)
           check(s.time.count() >= 0, "time recorded");
         check(stage_names(r) ==
                   std::vector<std::string>{
                       "pipeline", "chord_length", "ear_clipping",
                       "longest_edge_"
                       "bisection",
                       "delaunay_flips", "laplacian_smoothing"},
               "stage order");
         check(r.reports[3].metrics.at("local_improver_changes") > 0,
               "flips ran locally inside bisection");
         check(r.reports[1].metrics.at("points") ==
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
           check(fine.reports[1].stage == "curve_links", "after pipeline");
           near(fine.reports[1].metrics.at("unlinked_edges"), 0, 0,
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
         const auto &links = fine.reports[1];
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
