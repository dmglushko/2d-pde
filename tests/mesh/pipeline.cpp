#include "support/check.hpp"
#include "support/mesh_checks.hpp"
#include "support/shapes.hpp"

#include "pde2d/geometry/polygon.hpp"
#include "pde2d/mesh/cdt.hpp"
#include "pde2d/mesh/chord_length_discretizer.hpp"
#include "pde2d/mesh/delaunay_flips.hpp"
#include "pde2d/mesh/domain_size.hpp"
#include "pde2d/mesh/ear_clipping.hpp"
#include "pde2d/mesh/laplacian_smoothing.hpp"
#include "pde2d/mesh/longest_edge_bisection.hpp"
#include "pde2d/mesh/mesher.hpp"
#include "pde2d/mesh/remesher.hpp"
#include "pde2d/mesh/ruppert.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
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
  std::optional<Parents> refine(WorkingMesh &, std::span<const std::size_t>,
                                std::span<const LocalImprover *const>,
                                StageReport &) const override {
    return std::nullopt;
  }
};

// An improver with declared effects and roles (both by default); it does
// nothing but return one change per local call.
class Probe final : public Improver,
                    public LocalImprover,
                    public FinalImprover {
public:
  Probe(std::string name, ImproverEffects effects, ImproverUse use,
        bool local = true, bool final = true)
      : name_(std::move(name)), effects_(effects), use_(use), local_(local),
        final_(final) {}
  std::string name() const override { return name_; }
  ImproverEffects effects() const override { return effects_; }
  ImproverUse use() const override { return use_; }
  const LocalImprover *as_local() const override {
    return local_ ? this : nullptr;
  }
  const FinalImprover *as_final() const override {
    return final_ ? this : nullptr;
  }
  void run(WorkingMesh &, const MeshSpec &, StageReport &) const override {}
  std::size_t improve_around(WorkingMesh &,
                             std::span<const std::size_t>) const override {
    return 1;
  }

private:
  std::string name_;
  ImproverEffects effects_;
  ImproverUse use_;
  bool local_, final_;
};

auto probe(std::string name, ImproverEffects effects, ImproverUse use,
           bool local = true, bool final = true) {
  return std::make_shared<Probe>(std::move(name), effects, use, local, final);
}

// Distance from p to the boundary of the stadium: [0, 2] x [0, 2] with a
// half disc of radius 1 on each short end.
double stadium_distance(Point p) {
  if (p.x > 2)
    return std::abs(distance(p, {2, 1}) - 1);
  if (p.x < 0)
    return std::abs(distance(p, {0, 1}) - 1);
  return std::min(std::abs(p.y), std::abs(p.y - 2));
}

// The stadium as one region: its four junctions are smooth.
CurveGeometry stadium() {
  return CurveGeometry(
      {Loop{{{Line{{0, 0}, {2, 0}}, "wall"},
             {EllipticalArc{{2, 1}, 1, 1, 0, -pi / 2, pi}, "wall"},
             {Line{{2, 2}, {0, 2}}, "wall"},
             {EllipticalArc{{0, 1}, 1, 1, 0, pi / 2, pi}, "wall"}}}});
}

// The mesh without its chains and links, as if it had been imported.
Mesh without_links(Mesh m) {
  m.chains.clear();
  for (auto &e : m.boundary_edges)
    e.link.reset();
  return m;
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
         // Roles are symmetric: a role the improver does not have is a
         // warning.
         Remesher r5(
             bisection(),
             {probe("local_only_role", {}, U::local_and_final, true, false),
              probe("no_role", {}, U::final_only, true, false),
              probe("neither", {}, U::local_and_final, false, false)});
         check(r5.plan().local_improvers == std::vector<std::size_t>{0},
               "the local role runs");
         check(r5.plan().final_passes.empty(), "no final role");
         check(has_message(r5.plan().warnings,
                           "local_only_role: it has no final version, so it "
                           "runs only locally"),
               "no final version");
         check(has_message(r5.plan().warnings,
                           "no_role: it has no final version, so it does not "
                           "run at all"),
               "final only without a final version");
         check(has_message(r5.plan().warnings,
                           "neither: it has no local version, so it does not "
                           "run at all") &&
                   has_message(r5.plan().warnings,
                               "neither: it has no final version, so it does "
                               "not run at all"),
               "one warning per role");
         check(r5.plan().warnings.size() == 4, "four warnings");
         check(LaplacianSmoothing().as_local() == nullptr &&
                   LaplacianSmoothing().as_final() != nullptr,
               "smoothing declares only the final role");
         DelaunayFlips both;
         check(both.as_local() == &both && both.as_final() == &both,
               "flips declare both roles");
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
         auto r = mesher.run(rectangle(1, 1),
                             {.max_edge_length = {.absolute = 0.5}});
         check(stage_names(r) ==
                   std::vector<std::string>{"pipeline", "chord_length",
                                            "ear_clipping", "strict", "counter",
                                            "mover", "laplacian_smoothing"},
               "pipeline report once, first; final passes in list order");
         const auto &p = r.reports[0];
         check(p.messages.size() == 2, "the plan and one warning (mover)");
         check(p.messages[0] == "plan: refiner strict; local: counter; final: "
                                "counter, mover, laplacian_smoothing",
               "the plan in words");
         check(std::vector<std::string>(p.messages.begin() + 1,
                                        p.messages.end()) ==
                   mesher.plan().warnings,
               "then the warnings");
         check(p.metrics.size() == 2, "two metrics");
         near(p.metrics.at("local_improvers"), 1, 0, "one local improver");
         near(p.metrics.at("final_passes"), 3, 0, "three final passes");
         check(p.time.count() >= 0, "time of the planning");
         // The refiner received exactly the resolved local improvers.
         check(r.reports[3].messages ==
                   std::vector<std::string>{"local counter"},
               "local improvers passed to the refiner");
         // Every run copies the same report; so does a Remesher run.
         auto again = mesher.run(rectangle(1, 1),
                                 {.max_edge_length = {.absolute = 0.5}});
         check(again.reports[0].messages == p.messages, "copied per run");
         Remesher remesher(nullptr, {flips(), flips()});
         auto rr = remesher.run(r.mesh, {.max_edge_length = {.absolute = 0.5}});
         check(stage_names(rr) == std::vector<std::string>{"pipeline",
                                                           "delaunay_flips",
                                                           "delaunay_flips"},
               "remesher reports");
         check(rr.reports[0].messages.size() == 4,
               "the plan, two refused local uses and one redundant pass");
         check(rr.reports[0].messages[0] ==
                   "plan: no refiner; local: none; final: delaunay_flips, "
                   "delaunay_flips",
               "the plan without a refiner");
         auto rg = remesher.run(r.mesh, rectangle(1, 1),
                                {.max_edge_length = {.absolute = 0.5}});
         check(rg.reports[0].stage == "pipeline" &&
                   rg.reports[1].stage == "curve_links",
               "pipeline, then curve links");
       }},
      {"input_checked_when_run_starts",
       [] {
         Mesher mesher(chords(), std::make_shared<NoHoles>());
         // Constructing worked; the mismatch shows only with a hole.
         auto r = mesher.run(rectangle(1, 1),
                             {.max_edge_length = {.absolute = 0.5}});
         check_valid_mesh(r.mesh);
         expect_throws<std::invalid_argument>(
             [&] {
               (void)mesher.run(annulus(1, 0.5),
                                {.max_edge_length = {.absolute = 0.5}});
             },
             "no_holes does not support holes; the geometry has 1");
         expect_throws<std::invalid_argument>(
             [&] { (void)mesher.run(rectangle(1, 1), {}); },
             "MeshSpec::max_edge_length needs an absolute or a relative value");
         expect_throws<std::invalid_argument>(
             [&] {
               (void)mesher.run(rectangle(1, 1),
                                {.max_edge_length = {.absolute = 0}});
             },
             "MeshSpec::max_edge_length.absolute must be finite and positive");
         expect_throws<std::invalid_argument>(
             [&] {
               (void)mesher.run(rectangle(1, 1),
                                {.max_edge_length = {.relative = INFINITY}});
             },
             "MeshSpec::max_edge_length.relative must be finite and positive");
       }},
      {"length_bound_resolves_to_the_smaller_value",
       [] {
         near(resolve({.absolute = 2}, 5), 2, 0, "absolute");
         near(resolve({.relative = 0.1}, 5), 0.5, 0, "relative");
         near(resolve({.absolute = 2, .relative = 0.1}, 5), 0.5, 0,
              "relative smaller");
         near(resolve({.absolute = 0.2, .relative = 0.1}, 5), 0.2, 0,
              "absolute smaller");
         check(resolve({}, 5) == std::numeric_limits<double>::infinity(),
               "no value: no bound");
         validate(LengthBound{}, "x"); // no value is valid for a bound
         expect_throws<std::invalid_argument>(
             [] { validate(LengthBound{.relative = -1}, "x"); },
             "x.relative must be finite and positive");
       }},
      {"relative_edge_length_is_resolved_against_the_domain_size",
       [] {
         // A 2 x 1 rectangle: domain size sqrt(5).
         auto r =
             Mesher(chords(), cdt(), ruppert())
                 .run(rectangle(2, 1), {.max_edge_length = {.relative = 0.1}});
         const double size = std::sqrt(5.0);
         near(r.domain_size, size, 1e-15, "domain size");
         near(r.max_edge_length, 0.1 * size, 1e-15, "resolved edge length");
         check(r.target_met && r.properties.longest_edge <= 0.1 * size,
               "target met");
         for (const auto &s : r.reports)
           if (s.stage == "chord_length" || s.stage == "ruppert") {
             near(s.metrics.at("domain_size"), size, 1e-15,
                  "reported domain size");
             near(s.metrics.at("max_edge_length"), 0.1 * size, 1e-15,
                  "reported edge length");
           }
         // Without a refiner the result still says what h was.
         auto plain =
             Mesher(chords(), cdt())
                 .run(rectangle(2, 1), {.max_edge_length = {.absolute = 0.3}});
         near(plain.max_edge_length, 0.3, 0, "absolute edge length");
       }},
      {"relative_values_give_the_same_mesh_at_every_scale",
       [] {
         // Scaling by a power of two is exact in floating point, so with
         // relative values every decision is the same and the mesh is the
         // same mesh, scaled.
         auto discretizer = std::make_shared<ChordLengthDiscretizer>(
             ChordLengthOptions{.max_curve_deviation = {.relative = 0.002}});
         Mesher mesher(discretizer, cdt(), ruppert());
         const auto g = annulus(1, 0.4);
         const MeshSpec spec{.max_edge_length = {.relative = 0.08}};
         auto small = mesher.run(g, spec);
         for (double scale : {1.0 / 1024, 1024.0}) {
           auto big = mesher.run(g.transformed({.scale = scale}), spec);
           check(big.mesh.triangles.size() == small.mesh.triangles.size(),
                 "same triangle count");
           check(big.mesh.nodes.size() == small.mesh.nodes.size(),
                 "same node count");
           for (std::size_t v = 0; v < small.mesh.nodes.size(); ++v) {
             near(big.mesh.nodes[v].x, scale * small.mesh.nodes[v].x,
                  1e-12 * scale, "scaled node");
             near(big.mesh.nodes[v].y, scale * small.mesh.nodes[v].y,
                  1e-12 * scale, "scaled node");
           }
         }
       }},
      {"domain_size_from_chains_or_boundary_nodes",
       [] {
         const auto g = circle(1);
         auto r = Mesher(chords(), cdt())
                      .run(g, {.max_edge_length = {.absolute = 0.5}});
         // The output mesh carries the geometry's curves in its chains:
         // exactly the same number.
         check(domain_size(r.mesh) == domain_size(g), "chains: same size");
         check(r.domain_size == domain_size(g), "result: same size");
         near(domain_size(g), 2 * std::sqrt(2.0), 1e-15, "circle box");
         // Without chains the boundary nodes are the boundary: a polygon
         // inside the circle, so a smaller box.
         const Mesh m = without_links(r.mesh);
         check(domain_size(m) < domain_size(g), "nodes: smaller");
         // Splitting boundary edges at their midpoints does not change it,
         // so refinement keeps the same domain size.
         auto fine = Remesher(bisection())
                         .run(m, {.max_edge_length = {.relative = 0.02}});
         check(fine.mesh.boundary_edges.size() > m.boundary_edges.size(),
               "boundary refined");
         check(domain_size(fine.mesh) == domain_size(m), "same after refining");
         check(fine.domain_size == domain_size(m), "result size");
         near(fine.max_edge_length, 0.02 * domain_size(m), 0,
              "resolved against the nodes");
         check(fine.target_met, "target met");
         expect_throws<std::invalid_argument>([] { (void)domain_size(Mesh{}); },
                                              "no boundary edges");
       }},
      {"target_met_is_checked_on_the_final_mesh",
       [] {
         // Without a refiner only the boundary is fine; interior edges of
         // the ear-clipped circle are long.
         auto r = Mesher(chords(), ears())
                      .run(circle(1), {.max_edge_length = {.absolute = 0.2}});
         check(!r.target_met, "target missed");
         near(r.properties.longest_edge, longest_edge(r.mesh), 0,
              "longest edge");
         check(r.properties.longest_edge > 0.2, "longest edge above h");
         auto ok = Mesher(chords(), ears(), bisection())
                       .run(circle(1), {.max_edge_length = {.absolute = 0.2}});
         check(ok.target_met, "with a refiner");
       }},
      {"holes_and_islands",
       [] {
         auto g = holes_and_island();
         for (auto r : {Mesher(chords(), cdt(), ruppert())
                            .run(g, {.max_edge_length = {.absolute = 0.3}}),
                        Mesher(chords(), ears(), bisection(), {smoothing()})
                            .run(g, {.max_edge_length = {.absolute = 0.3}})}) {
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
         auto r = Mesher(chords(), cdt(), ruppert())
                      .run(g, {.max_edge_length = {.absolute = 0.2}});
         check_valid_mesh(r.mesh);
         check_area(r.mesh, 1 + pi / 4, 0.03);
       }},
      {"orientation_normalized_for_meshing",
       [] {
         // Outer loop clockwise, hole counterclockwise: both get fixed.
         Loop outer = square_loop({0, 0}, 2, "outer");
         Loop flipped;
         for (auto it = outer.curves.rbegin(); it != outer.curves.rend(); ++it)
           flipped.curves.push_back({(it->curve).reversed(), it->region});
         CurveGeometry g({flipped, square_loop({0.5, 0.5}, 1, "hole")});
         auto r = Mesher(chords(), cdt())
                      .run(g, {.max_edge_length = {.absolute = 0.5}});
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
         auto coarse =
             std::make_shared<ChordLengthDiscretizer>(ChordLengthOptions{});
         expect_throws<std::runtime_error>(
             [&] {
               (void)Mesher(coarse, cdt())
                   .run(g, {.max_edge_length = {.absolute = 1.0}});
             },
             "chord_length produced an invalid boundary polygon: Boundary "
             "polygon edges cross or touch");
         // A curve deviation bound fixes it.
         auto fine = std::make_shared<ChordLengthDiscretizer>(
             ChordLengthOptions{.max_curve_deviation = {.absolute = 0.01}});
         auto r =
             Mesher(fine, cdt()).run(g, {.max_edge_length = {.absolute = 1.0}});
         check_valid_mesh(r.mesh);
       }},
      {"reports",
       [] {
         auto r =
             Mesher(chords(), ears(), bisection(), {flips(), smoothing()})
                 .run(rectangle(2, 1), {.max_edge_length = {.absolute = 0.3}});
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
         check(same_mesh(
                   mesher.run(g, {.max_edge_length = {.absolute = 0.25}}).mesh,
                   mesher.run(g, {.max_edge_length = {.absolute = 0.25}}).mesh),
               "same input, same mesh");
       }},
      {"remesh_without_links_keeps_points_on_chords",
       [] {
         auto coarse =
             Mesher(chords(), cdt(), ruppert())
                 .run(circle(1), {.max_edge_length = {.absolute = 0.5}});
         // An imported mesh has no links.
         const Mesh c = without_links(coarse.mesh);
         for (auto refiner : {std::shared_ptr<const Refiner>(ruppert()),
                              std::shared_ptr<const Refiner>(bisection())}) {
           auto fine =
               Remesher(refiner).run(c, {.max_edge_length = {.absolute = 0.1}});
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
      {"output_mesh_carries_its_links",
       [] {
         auto g = annulus(1, 0.5);
         auto coarse = Mesher(chords(), cdt(), ruppert())
                           .run(g, {.max_edge_length = {.absolute = 0.5}});
         const Mesh &m = coarse.mesh;
         check(m.chains.size() == 2, "one chain per circle");
         for (const auto &e : m.boundary_edges) {
           check(e.link.has_value(), "every boundary edge is linked");
           check(is_valid_link(*e.link, m.chains), "valid link");
           // The link's ends are the edge's nodes.
           const auto &chain = m.chains[e.link->chain];
           near(distance(chain.point_at(e.link->s0), m.nodes[e.nodes[0]]), 0,
                1e-12, "s0 at the first node");
           near(distance(chain.point_at(e.link->s1), m.nodes[e.nodes[1]]), 0,
                1e-12, "s1 at the second node");
         }
         // Refined again from the output Mesh alone: new boundary nodes
         // lie on the true circles.
         for (auto refiner : {std::shared_ptr<const Refiner>(ruppert()),
                              std::shared_ptr<const Refiner>(bisection())}) {
           auto fine =
               Remesher(refiner).run(m, {.max_edge_length = {.absolute = 0.1}});
           check_valid_mesh(fine.mesh);
           check(fine.target_met, "target met");
           check(fine.reports[1].stage == refiner->name(), "no curve_links");
           check(fine.mesh.boundary_edges.size() > m.boundary_edges.size(),
                 "boundary refined");
           std::size_t inner = region_id(fine.mesh, "inner");
           for (const auto &e : fine.mesh.boundary_edges)
             for (auto v : e.nodes)
               near(norm(fine.mesh.nodes[v]), e.region == inner ? 0.5 : 1.0,
                    1e-12, "boundary node on its circle");
           // And again: the refined mesh carries its links too.
           auto finer = Remesher(refiner).run(
               fine.mesh, {.max_edge_length = {.absolute = 0.05}});
           for (const auto &e : finer.mesh.boundary_edges)
             near(norm(finer.mesh.nodes[e.nodes[0]]),
                  e.region == inner ? 0.5 : 1.0, 1e-12, "still on the circle");
         }
       }},
      {"remesh_with_geometry_links_across_smooth_junctions",
       [] {
         // The stadium's junctions are smooth, so the discretizer samples
         // it as one chain and its chords span the junctions. Linking to
         // the geometry builds the same chain, so every edge is linked.
         auto g = stadium();
         auto deviation = std::make_shared<ChordLengthDiscretizer>(
             ChordLengthOptions{.max_curve_deviation = {.absolute = 0.02}});
         auto coarse = Mesher(deviation, cdt())
                           .run(g, {.max_edge_length = {.absolute = 0.5}});
         const Mesh m = without_links(coarse.mesh);
         auto fine = Remesher(bisection())
                         .run(m, g, {.max_edge_length = {.absolute = 0.1}});
         check_valid_mesh(fine.mesh);
         const auto &links = fine.reports[1];
         near(links.metrics.at("chains"), 1, 0, "one chain");
         near(links.metrics.at("unlinked_edges"), 0, 0, "every edge linked");
         check(links.messages.empty(), "no message");
         for (auto v : boundary_nodes(fine.mesh))
           near(stadium_distance(fine.mesh.nodes[v]), 0, 1e-12,
                "boundary node on the stadium");
         // With a corner angle of 0 every junction is a break, and the
         // edges across junctions stay unlinked. The coarse mesh has 26
         // equal pieces, so the junctions at the seam (s = 0) and at half
         // the length (s = 2 + pi) are nodes; the other two are crossed.
         check(m.boundary_edges.size() == 26, "26 boundary edges");
         auto broken =
             Remesher(bisection())
                 .run(m, g, {.max_edge_length = {.absolute = 0.1}}, 0.0);
         near(broken.reports[1].metrics.at("chains"), 4, 0, "four chains");
         near(broken.reports[1].metrics.at("unlinked_edges"), 2, 0,
              "two edges across junctions");
         expect_throws<std::invalid_argument>(
             [&] {
               (void)Remesher(bisection())
                   .run(m, g, {.max_edge_length = {.absolute = 0.1}}, 200);
             },
             "Remesher: corner_angle_degrees must be in [0, 180]");
       }},
      {"remesh_with_geometry_puts_points_on_curves",
       [] {
         auto g = annulus(1, 0.5);
         auto coarse = Mesher(chords(), cdt(), ruppert())
                           .run(g, {.max_edge_length = {.absolute = 0.5}});
         for (auto refiner : {std::shared_ptr<const Refiner>(ruppert()),
                              std::shared_ptr<const Refiner>(bisection())}) {
           auto fine =
               Remesher(refiner).run(without_links(coarse.mesh), g,
                                     {.max_edge_length = {.absolute = 0.1}});
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
         auto m = Mesher(chords(), cdt())
                      .run(circle(1), {.max_edge_length = {.absolute = 0.5}})
                      .mesh;
         auto rotated = circle(1).transformed({.rotation = 0.1});
         auto fine =
             Remesher(bisection())
                 .run(m, rotated, {.max_edge_length = {.absolute = 0.05}});
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
         auto m = Mesher(chords(), cdt())
                      .run(circle(1), {.max_edge_length = {.absolute = 0.5}})
                      .mesh;
         expect_throws<std::invalid_argument>(
             [&] {
               (void)Remesher(bisection())
                   .run(m, circle(2), {.max_edge_length = {.absolute = 0.1}});
             },
             "does not lie on a curve of that region");
         auto renamed = circle(1);
         renamed.rename_region("boundary", "wall");
         expect_throws<std::invalid_argument>(
             [&] {
               (void)Remesher(bisection())
                   .run(m, renamed, {.max_edge_length = {.absolute = 0.1}});
             },
             "mesh region 'boundary' does not exist in the geometry");
       }},
      {"remesh_rejects_invalid_mesh",
       [] {
         auto m =
             Mesher(chords(), cdt())
                 .run(rectangle(1, 1), {.max_edge_length = {.absolute = 0.5}})
                 .mesh;
         auto broken = m;
         broken.boundary_edges.pop_back();
         expect_throws<std::invalid_argument>(
             [&] {
               (void)Remesher(bisection())
                   .run(broken, {.max_edge_length = {.absolute = 0.1}});
             },
             "has no neighbour but is not a boundary edge");
         auto inverted = m;
         std::swap(inverted.triangles[0].nodes[0],
                   inverted.triangles[0].nodes[1]);
         expect_throws<std::invalid_argument>(
             [&] {
               (void)Remesher(bisection())
                   .run(inverted, {.max_edge_length = {.absolute = 0.1}});
             },
             "not counterclockwise");
       }},
  });
}
