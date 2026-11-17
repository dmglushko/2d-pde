// The library delivers the refinement-edge order (mesh.hpp): the Mesher
// and Remesher::run give every triangle its longest edge as refinement
// edge; Remesher::refine does it only for the triangles whose order an
// edit chose (flips, Ruppert's insertions), while the triangles a
// bisection made and the untouched ones keep their order.
//
// Oracle of "longest edge first": the public
// set_longest_edges_as_refinement_edges (tested on its own in
// refinement.cpp) leaves such a triangle as it is.

#include "mesh/refinement_checks.hpp"
#include "support/check.hpp"
#include "support/mesh_checks.hpp"
#include "support/shapes.hpp"

#include "pde2d/mesh/delaunay_flips.hpp"
#include "pde2d/mesh/ear_clipping.hpp"
#include "pde2d/mesh/laplacian_smoothing.hpp"
#include "pde2d/mesh/ruppert.hpp"

#include <algorithm>
#include <array>
#include <map>
#include <memory>
#include <string>
#include <vector>

using namespace pde2d::test;
using namespace pde2d::mesh;

namespace {

using Nodes = std::array<std::size_t, 3>;

// Per triangle: is its longest edge (in the library's tie order) its
// refinement edge?
std::vector<bool> longest_first(const Mesh &m) {
  Mesh ordered = m;
  set_longest_edges_as_refinement_edges(ordered);
  std::vector<bool> result;
  for (std::size_t t = 0; t < m.triangles.size(); ++t)
    result.push_back(ordered.triangles[t].nodes == m.triangles[t].nodes);
  return result;
}

bool all_longest_first(const Mesh &m) {
  const auto first = longest_first(m);
  return std::all_of(first.begin(), first.end(), [](bool b) { return b; });
}

// The triangles by their vertex set, with their vertex order.
std::map<Nodes, Nodes> by_vertex_set(const Mesh &m) {
  std::map<Nodes, Nodes> result;
  for (const auto &t : m.triangles) {
    Nodes key = t.nodes;
    std::sort(key.begin(), key.end());
    result[key] = t.nodes;
  }
  return result;
}

// The order of t's vertex set in `known`, if it is there.
const Nodes *known_order(const std::map<Nodes, Nodes> &known, const Nodes &t) {
  Nodes key = t;
  std::sort(key.begin(), key.end());
  auto it = known.find(key);
  return it == known.end() ? nullptr : &it->second;
}

// Every triangle rotated by one: no triangle has its longest edge first.
Mesh rotated(Mesh m) {
  for (auto &t : m.triangles)
    t.nodes = {t.nodes[1], t.nodes[2], t.nodes[0]};
  return m;
}

double pipeline_metric(const std::vector<StageReport> &reports) {
  return reports.front().metrics.at("refinement_edges_set");
}

// The Mesher's output, whatever its stages, and Remesher::run's.
void run_delivers_longest_edges() {
  const MeshSpec spec{.max_edge_length = {.absolute = 0.3}};
  struct Case {
    std::string name;
    Mesher mesher;
  };
  auto flips = [](ImproverUse use) {
    return std::make_shared<DelaunayFlips>(DelaunayFlipsOptions{.use = use});
  };
  std::vector<Case> cases;
  cases.push_back({"cdt", Mesher(std::make_shared<ChordLengthDiscretizer>(),
                                 std::make_shared<Cdt>())});
  cases.push_back(
      {"ear clipping", Mesher(std::make_shared<ChordLengthDiscretizer>(),
                              std::make_shared<EarClipping>())});
  cases.push_back({"ruppert", Mesher(std::make_shared<ChordLengthDiscretizer>(),
                                     std::make_shared<Cdt>(),
                                     std::make_shared<Ruppert>())});
  cases.push_back(
      {"longest-edge bisection, local flips",
       Mesher(std::make_shared<ChordLengthDiscretizer>(),
              std::make_shared<Cdt>(), std::make_shared<LongestEdgeBisection>(),
              {flips(ImproverUse::local_only)})});
  cases.push_back({"newest vertex bisection, final flips and smoothing",
                   Mesher(std::make_shared<ChordLengthDiscretizer>(),
                          std::make_shared<Cdt>(),
                          std::make_shared<NewestVertexBisection>(),
                          {flips(ImproverUse::final_only),
                           std::make_shared<LaplacianSmoothing>()})});
  for (const auto &geometry : {l_shape(), star(), holes_and_island()})
    for (const auto &c : cases) {
      const MeshResult r = c.mesher.run(geometry, spec);
      check_valid_mesh(r.mesh);
      check(all_longest_first(r.mesh), c.name + ": longest edges first");
    }
  // The triangulators' own order is not the longest edges: the metric
  // counts the triangles that were rotated.
  const MeshResult cdt = cases[0].mesher.run(l_shape(), spec);
  check(pipeline_metric(cdt.reports) > 0, "cdt: triangles were rotated");

  // Remesher::run rotates every triangle, also without a refiner.
  const Mesh turned = rotated(cdt.mesh);
  check(!all_longest_first(turned), "the rotated mesh is not ordered");
  for (const auto &refiner : std::vector<std::shared_ptr<const Refiner>>{
           nullptr, std::make_shared<Ruppert>()}) {
    const MeshResult r = Remesher(refiner).run(turned, spec);
    check_valid_mesh(r.mesh);
    check(all_longest_first(r.mesh), "Remesher::run: longest edges first");
    if (!refiner) {
      // Rotations only: the same triangles, every one rotated back.
      check(same_mesh(r.mesh, cdt.mesh), "no refiner: the mesher's mesh");
      near(pipeline_metric(r.reports),
           static_cast<double>(turned.triangles.size()), 0,
           "every triangle rotated");
    }
  }
}

// Bisection keeps orders: the triangles nothing touched keep theirs (here
// none is the longest edge), and with no improver nothing is reordered.
void refine_keeps_untouched_and_bisection_orders() {
  const Mesh in = rotated(initial_mesh(l_shape(), 0.3));
  const Marked marked{0, 5, 17};
  const auto input = by_vertex_set(in);
  for (const auto &refiner : bisection_refiners()) {
    const RefinementResult r = refine_with(refiner, in, marked);
    check_refinement(in, r, marked,
                     {.marked_children = marked_children(*refiner)});
    std::size_t untouched = 0;
    bool same_order = true, newest_first = true;
    for (const auto &t : r.mesh.triangles) {
      if (const Nodes *order = known_order(input, t.nodes)) {
        ++untouched;
        same_order = same_order && *order == t.nodes;
      } else if (refiner->name() == "newest_vertex_bisection") {
        // Every triangle bisection made has its new node first.
        newest_first = newest_first && t.nodes[0] >= in.nodes.size();
      }
    }
    check(untouched > 0 && untouched < in.triangles.size(),
          refiner->name() + ": some triangles untouched");
    check(same_order, refiner->name() + ": untouched triangles keep their "
                                        "(not longest-edge) order");
    check(newest_first, refiner->name() + ": bisection's order kept");
    near(pipeline_metric(r.reports), 0, 0,
         refiner->name() + ": nothing reordered");
  }
}

// A final pass of Delaunay flips after newest vertex bisection: the
// triangles it made get their longest edge first, the others keep the
// order the bisection gave them (the same refinement without the pass).
void refine_reorders_what_flips_changed() {
  const Mesh in = initial_mesh(star(), 0.25);
  Marked marked;
  for (std::size_t t = 0; t < in.triangles.size(); t += 2)
    marked.push_back(t);
  const auto nvb = std::make_shared<NewestVertexBisection>();
  for (const ImproverUse use :
       {ImproverUse::final_only, ImproverUse::local_only}) {
    const bool final_pass = use == ImproverUse::final_only;
    const std::string name = final_pass ? "final flips" : "local flips";
    const RefinementResult plain = Remesher(nvb).refine(in, marked);
    const RefinementResult flipped =
        Remesher(nvb, {std::make_shared<DelaunayFlips>(
                          DelaunayFlipsOptions{.use = use})})
            .refine(in, marked);
    check_valid_mesh(flipped.mesh);
    const auto bisected = by_vertex_set(plain.mesh);
    const auto input = by_vertex_set(in);
    const auto first = longest_first(flipped.mesh);
    std::size_t made_by_flips = 0, kept_not_longest = 0;
    bool every_order_known = true;
    for (std::size_t t = 0; t < flipped.mesh.triangles.size(); ++t) {
      const Nodes &v = flipped.mesh.triangles[t].nodes;
      // With local flips the bisection runs on other triangles, so only
      // its pattern is known: input triangles keep their order, new
      // ones have the new node first.
      const Nodes *order =
          final_pass ? known_order(bisected, v) : known_order(input, v);
      const bool bisection_order =
          order ? *order == v : !final_pass && v[0] >= in.nodes.size();
      if (!order && final_pass)
        ++made_by_flips;
      if (bisection_order && !first[t])
        ++kept_not_longest;
      every_order_known = every_order_known && (bisection_order || first[t]);
    }
    check(every_order_known,
          name + ": every triangle has a bisection's or the longest-edge "
                 "order");
    if (final_pass) {
      check(flipped.mesh.nodes == plain.mesh.nodes, "the same nodes");
      check(made_by_flips > 0, name + ": the pass flipped edges");
      check(pipeline_metric(flipped.reports) > 0, name + ": reordered");
    }
    check(kept_not_longest > 0,
          name + ": bisection's orders are kept, not reset");
  }
}

// Ruppert's refine flips and inserts everywhere it works: the triangles
// it made or changed get their longest edge first, the others keep their
// order.
void ruppert_refine_reorders_what_it_changed() {
  const Mesh in =
      rotated(initial_mesh(l_shape(), 0.3, std::make_shared<Ruppert>()));
  const Marked marked{3};
  const RefinementResult r =
      Remesher(std::make_shared<Ruppert>()).refine(in, marked);
  check_valid_mesh(r.mesh);
  check(r.mesh.triangles.size() > in.triangles.size(), "refined");
  const auto input = by_vertex_set(in);
  const auto first = longest_first(r.mesh);
  std::size_t untouched = 0, changed = 0;
  bool ok = true;
  for (std::size_t t = 0; t < r.mesh.triangles.size(); ++t) {
    const Nodes &v = r.mesh.triangles[t].nodes;
    const Nodes *order = known_order(input, v);
    if (order && *order == v) {
      ++untouched;
      ok = ok && !first[t]; // rotated: never the longest edge
    } else {
      ++changed;
      ok = ok && first[t];
    }
  }
  check(ok, "changed triangles longest-edge first, the others as they were");
  check(untouched > 0 && changed > 0, "both kinds occur");
  check(pipeline_metric(r.reports) > 0 &&
            pipeline_metric(r.reports) <= static_cast<double>(changed),
        "the metric counts rotated triangles among the changed ones");
}

// A mesh from the Mesher is what a call by hand would make, so adaptive
// newest vertex bisection starts from the same mesh: no call is needed
// (tests/adapt/cycles.cpp checks the angles over many cycles).
void mesher_mesh_needs_no_call() {
  for (double h : {0.5, 0.2}) {
    const Mesh m = initial_mesh(l_shape(), h);
    Mesh by_hand = m;
    set_longest_edges_as_refinement_edges(by_hand);
    check(same_mesh(m, by_hand), "the call changes nothing");
  }
}

} // namespace

int main() {
  return run({
      {"run delivers the longest edges", run_delivers_longest_edges},
      {"refine keeps untouched and bisection orders",
       refine_keeps_untouched_and_bisection_orders},
      {"refine reorders what flips changed",
       refine_reorders_what_flips_changed},
      {"Ruppert's refine reorders what it changed",
       ruppert_refine_reorders_what_it_changed},
      {"a Mesher mesh needs no call", mesher_mesh_needs_no_call},
  });
}
