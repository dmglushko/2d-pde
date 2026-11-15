// Refinement of marked triangles (Remesher::refine) on straight domains,
// with the three bisection refiners, and the "until every edge is at most
// h" loop around it (run).

#include "mesh/refinement_checks.hpp"
#include "support/check.hpp"
#include "support/mesh_checks.hpp"
#include "support/shapes.hpp"

#include "pde2d/mesh/delaunay_flips.hpp"
#include "pde2d/mesh/laplacian_smoothing.hpp"
#include "pde2d/mesh/ruppert.hpp"

#include <map>
#include <memory>
#include <stdexcept>
#include <vector>

using namespace pde2d::mesh;
using namespace pde2d::test;

namespace {

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

// The index of the node at p (exactly), or the node count if none.
std::size_t node_at(const Mesh &m, Point p) {
  for (std::size_t v = 0; v < m.nodes.size(); ++v)
    if (m.nodes[v] == p)
      return v;
  return m.nodes.size();
}

// The triangle with these nodes in this order (a rotation does not count),
// or the triangle count if none.
std::size_t triangle_with(const Mesh &m, std::array<std::size_t, 3> nodes) {
  for (std::size_t t = 0; t < m.triangles.size(); ++t)
    if (m.triangles[t].nodes == nodes)
      return t;
  return m.triangles.size();
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

// Marking one triangle of the two-triangle square, exact counts. Newest
// vertex and longest-edge bisection bisect it at the diagonal (its
// refinement edge and its longest edge), which also bisects the
// neighbour. Red-green-blue halves its three edges (red) and bisects the
// neighbour at the diagonal (green).
void closure_counts() {
  const Mesh in = two_triangle_square();
  const Marked marked{0};
  for (const auto &refiner : bisection_refiners()) {
    auto r = refine_with(refiner, in, marked);
    check_refinement(in, r, marked,
                     {.marked_children = marked_children(*refiner)});
    const auto &report = refiner_report(r);
    check(report.stage == refiner->name(), "report stage");
    check(report.messages.empty(), "no message");
    const std::size_t centre = node_at(r.mesh, {0.5, 0.5});
    check(centre == 4, refiner->name() + ": the diagonal is split first");
    check(r.parents->nodes[centre] == std::array<std::size_t, 2>{0, 2},
          "the centre lies on the diagonal");
    if (refiner->name() != "red_green_blue") {
      check(r.mesh.triangles.size() == 4, refiner->name() + ": 2 + 2");
      check(r.mesh.nodes.size() == 5, refiner->name() + ": one new node");
      check(r.mesh.boundary_edges.size() == 4, "no boundary edge split");
      near(report.metrics.at("refined_by_closure"), 1, 0, "one by closure");
      // (1, 2, 0) -> (4, 1, 2), (4, 0, 1); (3, 0, 2) -> (4, 3, 0),
      // (4, 2, 3): the newest vertex first.
      for (auto t : {std::array<std::size_t, 3>{4, 1, 2},
                     {4, 0, 1},
                     {4, 3, 0},
                     {4, 2, 3}})
        check(triangle_with(r.mesh, t) < r.mesh.triangles.size(),
              refiner->name() + ": a bisection child, newest vertex first");
      continue;
    }
    check(r.mesh.triangles.size() == 6, "red-green-blue: 4 + 2 triangles");
    check(r.mesh.nodes.size() == 7, "red-green-blue: three new nodes");
    check(r.mesh.boundary_edges.size() == 6, "two boundary edges split");
    near(report.metrics.at("refined_by_closure"), 1, 0, "one by closure");
    near(report.metrics.at("red_triangles"), 1, 0, "one red triangle");
    // Triangle (v0, v1, v2) = (1, 2, 0): m0 on 2-0, m1 on 0-1, m2 on 1-2.
    const std::size_t m0 = centre, m1 = node_at(r.mesh, {0.5, 0}),
                      m2 = node_at(r.mesh, {1, 0.5});
    for (auto t : {std::array<std::size_t, 3>{1, m2, m1},
                   {m2, 2, m0},
                   {m1, m0, 0},
                   {m0, m1, m2}})
      check(triangle_with(r.mesh, t) < r.mesh.triangles.size(),
            "a red child, listing the images of v0, v1, v2");
    for (auto t : {std::array<std::size_t, 3>{m0, 3, 0}, {m0, 2, 3}})
      check(triangle_with(r.mesh, t) < r.mesh.triangles.size(),
            "a green child with the newest vertex first");
  }
}

// Newest vertex bisection of a triangle whose three edges are all split
// (two marked neighbours force it): bisected, then both children bisected.
void four_bisection_children() {
  Mesh m;
  // A triangle with a neighbour on each edge.
  m.nodes = {{0, 0}, {2, 0}, {1, 2}, {1, -1}, {2.2, 1.4}, {-0.2, 1.4}};
  m.triangles = {{{2, 0, 1}}, {{3, 1, 0}}, {{4, 2, 1}}, {{5, 0, 2}}};
  m.region_names = {"r"};
  m.boundary_edges = {BoundaryEdge{{0, 3}, 0, std::nullopt},
                      BoundaryEdge{{3, 1}, 0, std::nullopt},
                      BoundaryEdge{{1, 4}, 0, std::nullopt},
                      BoundaryEdge{{4, 2}, 0, std::nullopt},
                      BoundaryEdge{{2, 5}, 0, std::nullopt},
                      BoundaryEdge{{5, 0}, 0, std::nullopt}};
  // The outer triangles' refinement edges are their edges with the inner
  // one; the inner one's is 0-1.
  const Marked marked{1, 2, 3};
  auto r = refine_with(std::make_shared<NewestVertexBisection>(), m, marked);
  check_refinement(m, r, marked);
  std::size_t children = 0;
  for (auto p : r.parents->triangles)
    children += p == 0 ? 1 : 0;
  check(children == 4, "the inner triangle has four children");
}

void nothing_marked() {
  for (const auto &refiner : bisection_refiners()) {
    const Mesh in = initial_mesh(l_shape(), 0.6);
    auto r = refine_with(refiner, in, Marked{});
    check(same_mesh(r.mesh, in), "the same mesh");
    check_refinement(in, r, Marked{});
    const auto &messages = refiner_report(r).messages;
    check(messages.size() == 1 &&
              messages[0].find("nothing was marked") != std::string::npos,
          "a message says so");
  }
}

void bad_input() {
  for (const auto &refiner : bisection_refiners()) {
    const Mesh in = two_triangle_square();
    expect_throws<std::invalid_argument>(
        [&] { (void)refine_with(refiner, in, Marked{2}); },
        "Remesher::refine: marked triangle 2 is out of range (the mesh has 2 "
        "triangles)");
    Mesh clockwise = in;
    clockwise.triangles[0].nodes = {0, 2, 1};
    expect_throws<std::invalid_argument>(
        [&] { (void)refine_with(refiner, clockwise, Marked{0}); },
        "Remesher::refine: the input mesh is not valid");
    // The refiner checks too, when called on its own.
    WorkingMesh w = WorkingMesh::from_mesh(in);
    StageReport report;
    expect_throws<std::invalid_argument>(
        [&] { (void)refiner->refine(w, Marked{5}, {}, report); },
        refiner->name() + ": marked triangle 5 is out of range");
  }
  expect_throws<std::invalid_argument>(
      [] { (void)Remesher(nullptr).refine(two_triangle_square(), Marked{0}); },
      "Remesher::refine needs a refiner");
  expect_throws<std::invalid_argument>(
      [] { NewestVertexBisection({.max_points = 0}); },
      "NewestVertexBisectionOptions::max_points must be positive");
  expect_throws<std::invalid_argument>(
      [] { RedGreenBlueRefinement({.max_points = 0}); },
      "RedGreenBlueRefinementOptions::max_points must be positive");
  // The point budget stops refinement.
  expect_throws<std::runtime_error>(
      [] {
        Remesher remesher(std::make_shared<NewestVertexBisection>(
            NewestVertexBisectionOptions{.max_points = 3}));
        Mesh m = initial_mesh(l_shape(), 0.6);
        Marked all;
        for (std::size_t t = 0; t < m.triangles.size(); ++t)
          all.push_back(t);
        (void)remesher.refine(m, all);
      },
      "newest_vertex_bisection: inserted max_points = 3 points");
}

// Many markings on the square and the L-shape (from the Mesher, with
// links on straight lines, and without links): every result passes the
// oracles, repetitions in the marked list do not matter, and the same
// input gives the same output.
void straight_domains() {
  std::map<std::size_t, std::size_t> pattern_counts; // children -> count
  for (const auto &refiner : bisection_refiners())
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
          auto r = refine_with(refiner, in, marked);
          check_refinement(in, r, marked,
                           {.marked_children = marked_children(*refiner)});
          near(refiner_report(r).metrics.at("incompatible_splits"), 0, 0,
               "longest refinement edges never wait in a cycle");
          Marked doubled = marked;
          doubled.insert(doubled.end(), marked.rbegin(), marked.rend());
          check(same_result(r, refine_with(refiner, in, doubled)),
                "repetitions and order of marks do not matter");
          check(same_result(r, refine_with(refiner, in, marked)),
                "deterministic");
          std::map<std::size_t, std::size_t> children;
          for (auto p : r.parents->triangles)
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
  for (const auto &refiner : bisection_refiners()) {
    Mesh m = initial_mesh(g, 0.7);
    for (int cycle = 0; cycle < 3; ++cycle) {
      Marked all;
      for (std::size_t t = 0; t < m.triangles.size(); ++t)
        all.push_back(t);
      auto r = refine_with(refiner, m, all);
      check_refinement(m, r, all,
                       {.marked_children = marked_children(*refiner)});
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

// With improvers: the pipeline is planned as for run(), the improvers run,
// and parents survive only if every component keeps them.
void improvers_and_parents() {
  const Mesh in = initial_mesh(l_shape(), 0.5);
  Marked marked;
  for (std::size_t t = 0; t < in.triangles.size(); t += 2)
    marked.push_back(t);
  auto has_message = [](const StageReport &report, const std::string &text) {
    for (const auto &m : report.messages)
      if (m.find(text) != std::string::npos)
        return true;
    return false;
  };
  for (const auto &refiner : bisection_refiners()) {
    // Without improvers: parents, and no message about them.
    auto plain = refine_with(refiner, in, marked);
    check(plain.parents.has_value(), "parents without improvers");
    check(plain.reports.size() == 2, "pipeline and refiner reports");
    check(!has_message(plain.reports[0], "parents dropped"),
          "no message about parents");

    // Local and final Delaunay flips: the mesh is Delaunay, no parents.
    Remesher flipping(refiner, {std::make_shared<DelaunayFlips>()});
    auto flipped = flipping.refine(in, marked);
    check_valid_mesh(flipped.mesh);
    check(is_delaunay(flipped.mesh), "Delaunay after the flips");
    check(!flipped.parents, "flips drop the parents");
    check(flipped.reports.size() == 3, "pipeline, refiner, final pass");
    check(flipped.reports[2].stage == "delaunay_flips", "final pass report");
    check(has_message(flipped.reports[0],
                      "parents dropped: delaunay_flips (local): it flips "
                      "edges; delaunay_flips (final pass): it flips edges"),
          "the pipeline report names the improver");
    check(refiner_report(flipped).metrics.at("local_improver_changes") >= 0,
          "local improver counted");

    // Smoothing as a final pass.
    Remesher smoothing(refiner, {std::make_shared<LaplacianSmoothing>()});
    auto smoothed = smoothing.refine(in, marked);
    check(!smoothed.parents, "smoothing drops the parents");
    check(has_message(smoothed.reports[0],
                      "parents dropped: laplacian_smoothing (final pass): "
                      "it moves nodes"),
          "the pipeline report names smoothing");
    check(smoothed.mesh.nodes.size() == plain.mesh.nodes.size(),
          "the same refinement, then smoothing");
  }
  // Ruppert refines marked triangles by circumcenters; it flips.
  auto r = refine_with(std::make_shared<Ruppert>(), in, marked);
  check_valid_mesh(r.mesh);
  check(is_delaunay(r.mesh), "Ruppert keeps the mesh Delaunay");
  check(!r.parents, "Ruppert drops the parents");
  check(has_message(r.reports[0], "parents dropped: ruppert: it does not "
                                  "keep them"),
        "the pipeline report names Ruppert");
  check(r.mesh.nodes.size() > in.nodes.size(), "Ruppert added nodes");
  near(refiner_report(r).metrics.at("marked"),
       static_cast<double>(marked.size()), 0, "Ruppert marked count");
  near(mesh_area(r.mesh), mesh_area(in), 1e-12 * mesh_area(in),
       "Ruppert keeps the area of a straight domain");
}

// Ruppert's refine on a square split by its diagonal: both triangles
// share their circumcenter (the centre, on the diametral circles of the
// sides but not inside them), which is inserted once.
void ruppert_shared_circumcenter() {
  Mesh m;
  m.nodes = {{0, 0}, {2, 0}, {2, 2}, {0, 2}};
  m.triangles = {{{0, 1, 2}}, {{0, 2, 3}}};
  m.region_names = {"r"};
  m.boundary_edges = {BoundaryEdge{{0, 1}, 0, std::nullopt},
                      BoundaryEdge{{1, 2}, 0, std::nullopt},
                      BoundaryEdge{{2, 3}, 0, std::nullopt},
                      BoundaryEdge{{3, 0}, 0, std::nullopt}};
  auto r = refine_with(std::make_shared<Ruppert>(), m, Marked{0, 1});
  check_valid_mesh(r.mesh);
  check(node_at(r.mesh, {1, 1}) < r.mesh.nodes.size(), "the centre");
  const auto &metrics = refiner_report(r).metrics;
  near(metrics.at("circumcenters"), 1, 0, "one circumcenter inserted");
  near(metrics.at("skipped_circumcenters"), 1, 0, "the second one skipped");
}

// run() is "refine every triangle with a long edge" repeated: all three
// bisection refiners meet the spec, in the Mesher and in the Remesher.
void run_meets_the_spec() {
  for (const auto &refiner : bisection_refiners())
    for (const auto &g :
         {rectangle(2, 1), l_shape(), annulus(1, 0.4), holes_and_island()})
      for (double h : {0.3, 0.12}) {
        Mesher mesher(std::make_shared<ChordLengthDiscretizer>(),
                      std::make_shared<Cdt>(), refiner);
        auto r = mesher.run(g, {.max_edge_length = {.absolute = h}});
        check_valid_mesh(r.mesh);
        check(r.target_met, refiner->name() + ": target met");
        check(longest_edge(r.mesh) <= h, "every edge at most h");
        near(r.properties.longest_edge, longest_edge(r.mesh), 0,
             "the measured longest edge");
        near(mesh_area(r.mesh), boundary_area(r.mesh),
             1e-12 * boundary_area(r.mesh), "area");
        const auto &metrics = r.reports[3].metrics;
        check(metrics.at("steps") > 0, "steps reported");
        std::cout << "INFO " << refiner->name() << " run h = " << h << ": "
                  << r.mesh.triangles.size() << " triangles, min angle "
                  << r.properties.smallest_angle_degrees << '\n';
        // The same through the Remesher, from a coarser mesh.
        Mesh coarse = initial_mesh(g, 2 * h);
        auto again = Remesher(refiner).run(
            coarse, MeshSpec{.max_edge_length = {.absolute = h}});
        check(again.target_met, "Remesher::run meets the target");
        std::cout << "INFO " << refiner->name()
                  << " Remesher::run from longest refinement edges, h = " << h
                  << ": " << again.mesh.triangles.size()
                  << " triangles, min angle "
                  << again.properties.smallest_angle_degrees << '\n';
      }
}

// The two ways compose: refining marked triangles of a mesh made by run()
// keeps the oracles.
void refine_after_run() {
  for (const auto &refiner : bisection_refiners()) {
    Mesher mesher(std::make_shared<ChordLengthDiscretizer>(),
                  std::make_shared<Cdt>(), refiner);
    Mesh m = mesher.run(l_shape(), {.max_edge_length = {.absolute = 0.3}}).mesh;
    Marked marked;
    for (std::size_t t = 0; t < m.triangles.size(); t += 5)
      marked.push_back(t);
    auto r = refine_with(refiner, m, marked);
    check_refinement(m, r, marked,
                     {.marked_children = marked_children(*refiner)});
  }
}

} // namespace

int main() {
  return run({{"longest_edges", longest_edges},
              {"closure_counts", closure_counts},
              {"four_bisection_children", four_bisection_children},
              {"nothing_marked", nothing_marked},
              {"bad_input", bad_input},
              {"straight_domains", straight_domains},
              {"regions_kept", regions_kept},
              {"improvers_and_parents", improvers_and_parents},
              {"ruppert_shared_circumcenter", ruppert_shared_circumcenter},
              {"run_meets_the_spec", run_meets_the_spec},
              {"refine_after_run", refine_after_run}});
}
