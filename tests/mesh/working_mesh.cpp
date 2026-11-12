#include "support/check.hpp"
#include "support/mesh_checks.hpp"

#include "pde2d/mesh/delaunay.hpp"
#include "pde2d/mesh/working_mesh.hpp"

#include <numbers>

using namespace pde2d::mesh;
using namespace pde2d::test;
using pde2d::geometry::EllipticalArc;

namespace {

using std::numbers::pi;

// The circular arc from (0, 0) to (1, 0) through (0.5, top): it bulges
// into the domain above the chord when top > 0, out of it when top < 0.
// The arc-length middle is (0.5, top).
CurveChain bulge(double top) {
  // Center (0.5, k) at the same distance from (0, 0) and (0.5, top).
  double k = (top * top - 0.25) / (2 * top);
  double r = std::abs(top - k);
  double start = std::atan2(-k, -0.5), end = std::atan2(-k, 0.5);
  double sweep = end - start; // through the top of the circle
  if (top > 0 && sweep > 0)
    sweep -= 2 * pi;
  if (top < 0 && sweep < 0)
    sweep += 2 * pi;
  return CurveChain({EllipticalArc{{0.5, k}, r, r, 0, start, sweep}});
}

// Unit square (0,0)-(1,1) with one more node `extra` inside, the edge
// (0,0) -> (1,0) linked to `chain`. Triangles around `extra`.
WorkingMesh square_with(Point extra, const CurveChain &chain) {
  return WorkingMesh({{0, 0}, {1, 0}, extra, {1, 1}, {0, 1}},
                     {{0, 1, 2}, {1, 3, 2}, {2, 3, 4}, {0, 2, 4}},
                     {{{0, 1}, 0, CurveLink{0, 0, chain.length()}},
                      {{1, 3}, 0, {}},
                      {{3, 4}, 0, {}},
                      {{4, 0}, 0, {}}},
                     {"r"}, {chain});
}

// The unit square split along the diagonal 0-2.
//   3 --- 2
//   |   / |
//   | /   |
//   0 --- 1
WorkingMesh unit_square() {
  return WorkingMesh({{0, 0}, {1, 0}, {1, 1}, {0, 1}}, {{0, 1, 2}, {0, 2, 3}},
                     {{{0, 1}, 0, std::nullopt},
                      {{1, 2}, 1, std::nullopt},
                      {{2, 3}, 0, std::nullopt},
                      {{3, 0}, 1, std::nullopt}},
                     {"a", "b"});
}

} // namespace

int main() {
  return run({
      {"neighbours_and_boundary",
       [] {
         auto w = unit_square();
         // Edge 1 of triangle 0 (opposite node 1) is the diagonal 2 -> 0.
         check(w.neighbour(0, 1) == 1, "diagonal neighbour");
         check(w.neighbour(0, 0) == no_index, "edge 1-2 is boundary");
         check(w.boundary_edge_index(0, 0) == 1, "boundary edge 1 -> 2");
         check(w.boundary_edge_index(0, 1) == no_index, "interior");
         auto [t, e] = w.boundary_edge_location(3);
         check(t == 1 && w.boundary_edge_index(t, e) == 3, "location");
         check(w.find_edge(2, 0).first == 0 && w.find_edge(0, 2).first == 1,
               "directed edges");
         check(w.find_edge(1, 3).first == no_index, "no such edge");
         check(w.triangles_around(0).size() == 2, "node 0 in two triangles");
       }},
      {"validation",
       [] {
         using P = std::vector<Point>;
         expect_throws<std::invalid_argument>(
             [] {
               WorkingMesh w(
                   P{{0, 0}, {1, 0}, {0, 1}}, {{0, 2, 1}},
                   {{{0, 2}, 0, {}}, {{2, 1}, 0, {}}, {{1, 0}, 0, {}}}, {"a"});
             },
             "not counterclockwise");
         expect_throws<std::invalid_argument>(
             [] {
               WorkingMesh w(P{{0, 0}, {1, 0}, {0, 1}}, {{0, 1, 2}},
                             {{{0, 1}, 0, {}}, {{1, 2}, 0, {}}}, {"a"});
             },
             "Triangle edge (2, 0) has no neighbour but is not a boundary "
             "edge");
         expect_throws<std::invalid_argument>(
             [] {
               WorkingMesh w(
                   P{{0, 0}, {1, 0}, {0, 1}}, {{0, 1, 2}},
                   {{{0, 1}, 0, {}}, {{2, 1}, 0, {}}, {{2, 0}, 0, {}}}, {"a"});
             },
             "is not a triangle edge with the domain on its left");
         expect_throws<std::invalid_argument>(
             [] {
               WorkingMesh w(
                   P{{0, 0}, {1, 0}, {0, 1}, {5, 5}}, {{0, 1, 2}},
                   {{{0, 1}, 0, {}}, {{1, 2}, 0, {}}, {{2, 0}, 0, {}}}, {"a"});
             },
             "Node 3 is not used");
         // Two triangles touching at one node.
         expect_throws<std::invalid_argument>(
             [] {
               WorkingMesh w(P{{0, 0}, {1, 0}, {0, 1}, {-1, 0}, {0, -1}},
                             {{0, 1, 2}, {0, 3, 4}},
                             {{{0, 1}, 0, {}},
                              {{1, 2}, 0, {}},
                              {{2, 0}, 0, {}},
                              {{0, 3}, 0, {}},
                              {{3, 4}, 0, {}},
                              {{4, 0}, 0, {}}},
                             {"a"});
             },
             "Boundary loops touch at node 0");
         expect_throws<std::invalid_argument>(
             [] {
               WorkingMesh w(
                   P{{0, 0}, {1, 0}, {0, 1}}, {{0, 1, 2}},
                   {{{0, 1}, 3, {}}, {{1, 2}, 0, {}}, {{2, 0}, 0, {}}}, {"a"});
             },
             "region id out of range");
       }},
      {"flip",
       [] {
         auto w = unit_square();
         check(w.is_flippable(0, 1), "convex quad");
         check(!w.is_flippable(0, 0), "boundary edge");
         w.flip(0, 1);
         check(w.find_edge(1, 3).first != no_index ||
                   w.find_edge(3, 1).first != no_index,
               "new diagonal 1-3");
         check(w.find_edge(0, 2).first == no_index &&
                   w.find_edge(2, 0).first == no_index,
               "old diagonal gone");
         check_valid_mesh(w.to_mesh());
         check(w.boundary_edge_index(w.boundary_edge_location(1).first,
                                     w.boundary_edge_location(1).second) == 1,
               "boundary bookkeeping survives the flip");
       }},
      {"insert_and_split",
       [] {
         auto w = unit_square();
         auto t = w.insert_in_triangle(0, {0.7, 0.3});
         check(t.size() == 3 && w.triangles().size() == 4, "three from one");
         check_valid_mesh(w.to_mesh());
         auto [tri, edge] = w.find_edge(0, 2);
         w.split_interior_edge(tri, edge, {0.5, 0.5});
         check_valid_mesh(w.to_mesh());
         auto direct = w.split_boundary_edge(0);
         check(direct.split && !direct.retriangulated, "the direct step");
         Mesh m = w.to_mesh();
         check_valid_mesh(m);
         check(m.nodes.back() == Point{0.5, 0}, "chord midpoint");
         check(m.boundary_edges.size() == 5, "five boundary edges");
         near(mesh_area(m), 1, 1e-15, "area");
         expect_throws<std::invalid_argument>(
             [&] { w.insert_in_triangle(0, {5, 5}); }, "strictly inside");
       }},
      {"to_mesh_walks_loops",
       [] {
         auto w = unit_square();
         w.split_boundary_edge(0); // appends edge (4, 1)
         Mesh m = w.to_mesh();
         for (std::size_t k = 0; k < m.boundary_edges.size(); ++k)
           check(
               m.boundary_edges[k].nodes[1] ==
                   m.boundary_edges[(k + 1) % m.boundary_edges.size()].nodes[0],
               "each edge starts where the previous ends");
         check(m.boundary_edges[0].nodes == std::array<std::size_t, 2>{0, 4},
               "loop starts with the first edge");
         check(m.boundary_edges[1].region == 0, "the new half keeps region a");
       }},
      {"locate",
       [] {
         auto w = unit_square();
         auto in = w.locate({0.8, 0.2}, 1);
         check(in.kind == Location::Kind::inside && in.triangle == 0, "inside");
         auto on = w.locate({0.5, 0.5}, 0);
         check(on.kind == Location::Kind::on_edge, "on the diagonal");
         auto node = w.locate({1, 1}, 0);
         check(node.kind == Location::Kind::on_node, "a node");
         auto out = w.locate({2, 0.5}, 1);
         check(out.kind == Location::Kind::blocked &&
                   w.boundary_edge_index(out.triangle, out.index) == 1,
               "blocked by the right edge");
       }},
      {"curve_link_split_point",
       [] {
         // A quarter disc whose arc edge is linked to the true arc.
         using std::numbers::pi;
         EllipticalArc arc{{0, 0}, 1, 1, 0, 0, pi / 2};
         WorkingMesh w({{0, 0}, {1, 0}, {0, 1}}, {{0, 1, 2}},
                       {{{0, 1}, 0, {}},
                        {{1, 2}, 0, CurveLink{0, 0, pi / 2}},
                        {{2, 0}, 0, {}}},
                       {"r"}, {CurveChain({arc})});
         Point p = w.boundary_split_point(1);
         near(p.x, std::cos(pi / 4), 1e-15, "x on the arc");
         near(p.y, std::sin(pi / 4), 1e-15, "y on the arc");
         w.split_boundary_edge(1);
         const auto &e = w.boundary_edges();
         check(e[1].link && e[1].link->s1 == pi / 4, "first half");
         check(e[3].link && e[3].link->s0 == pi / 4 && e[3].link->s1 == pi / 2,
               "second half");
         Point q = w.boundary_split_point(3);
         near(q.x, std::cos(3 * pi / 8), 1e-15, "next split at 3 pi / 8");
         near(q.y, std::sin(3 * pi / 8), 1e-15, "next split at 3 pi / 8");
         // The output mesh keeps the chains and links, and a working mesh
         // made from it splits on the arc again.
         Mesh m = w.to_mesh();
         check(m.chains.size() == 1, "chains copied");
         std::size_t linked = 0;
         for (const auto &be : m.boundary_edges)
           linked += be.link.has_value();
         check(linked == 2, "both arc halves linked");
         auto again = WorkingMesh::from_mesh(m);
         check(again.chains().size() == 1, "chains back");
         bool on_arc = false;
         for (std::size_t k = 0; k < again.boundary_edges().size(); ++k)
           if (again.boundary_edges()[k].link) {
             Point r = again.boundary_split_point(k);
             on_arc = on_arc || std::abs(norm(r) - 1) < 1e-15;
           }
         check(on_arc, "split points on the arc");
       }},
      {"curve_point_that_would_invert",
       [] {
         // Edge 0 -> 1 is linked to an arc through (0.5, 0.3) that bulges
         // into the domain, past the opposite node (0.5, 0.1).
         double k = -0.16 / 0.6; // circle center (0.5, k) through the 3 points
         double r = 0.3 - k;
         double start = std::atan2(-k, -0.5), end = std::atan2(-k, 0.5);
         EllipticalArc arc{{0.5, k}, r, r, 0, start, end - start};
         near(pde2d::geometry::point_at(arc, 0.5).y, 0.3, 1e-15, "arc top");
         CurveChain chain({arc});
         WorkingMesh w({{0, 0}, {1, 0}, {0.5, 0.1}}, {{0, 1, 2}},
                       {{{0, 1}, 0, CurveLink{0, 0, chain.length()}},
                        {{1, 2}, 0, {}},
                        {{2, 0}, 0, {}}},
                       {"r"}, {chain});
         // Joining node 2 to the curve point would invert a triangle, and
         // the policy cannot re-triangulate either: the region it would
         // cut off contains node 2, a boundary node. Step (c): nothing
         // changes and the edge is marked.
         auto split = w.split_boundary_edge(0);
         check(!split.split && !split.retriangulated, "not split");
         check(split.failure ==
                   "the curve bulges in past another boundary node",
               "reason");
         check(w.nodes().size() == 3 && w.triangles().size() == 1,
               "mesh unchanged");
         check(w.is_unsplittable(0) && !w.is_unsplittable(1), "edge marked");
         check(!w.split_boundary_edge(0).split, "not retried");
       }},
      {"split_point_at_arc_length_middle",
       [] {
         // A chain of a line and a quarter circle that continues it
         // smoothly: (0,0) -> (1,0), then around (1,1) to (2,1).
         using std::numbers::pi;
         CurveChain chain({pde2d::geometry::Line{{0, 0}, {1, 0}},
                           EllipticalArc{{1, 1}, 1, 1, 0, -pi / 2, pi / 2}});
         near(chain.length(), 1 + pi / 2, 1e-15, "length");
         // An edge from s = 0.5 (on the line) to s = 2 (on the arc): the
         // middle s = 1.25 is 0.25 along the arc.
         Point a = chain.point_at(0.5), b = chain.point_at(2.0);
         near(a.x, 0.5, 0, "start on the line");
         WorkingMesh w({a, b, {0.5, 2}}, {{0, 1, 2}},
                       {{{0, 1}, 0, CurveLink{0, 0.5, 2.0}},
                        {{1, 2}, 0, {}},
                        {{2, 0}, 0, {}}},
                       {"r"}, {chain});
         Point p = w.boundary_split_point(0);
         near(distance(p, {1, 1}), 1, 1e-15, "on the circle");
         near(std::atan2(p.y - 1, p.x - 1), -pi / 2 + 0.25, 1e-14,
              "at arc length 0.25 along the arc");
         // An edge on the line only splits at its chord midpoint.
         WorkingMesh v({{0.25, 0}, {0.75, 0}, {0.5, 1}}, {{0, 1, 2}},
                       {{{0, 1}, 0, CurveLink{0, 0.25, 0.75}},
                        {{1, 2}, 0, {}},
                        {{2, 0}, 0, {}}},
                       {"r"}, {chain});
         check(v.boundary_split_point(0) == Point{0.5, 0}, "chord midpoint");
         // Positions at a junction.
         auto at = chain.position(1.0);
         check(at.curve == 1 && at.t == 0, "start of the later curve");
         auto before = chain.position_from_before(1.0);
         check(before.curve == 0 && before.t == 1, "end of the earlier one");
         expect_throws<std::invalid_argument>([] { CurveChain c({}); },
                                              "at least one curve");
       }},
      {"split_retriangulates_inward_bulge",
       [] {
         // The opposite node (0.95, 0.05) is close to the end of the
         // chord: joining it to the curve point (0.5, 0.2) would invert a
         // triangle, but it lies outside the region to cut off.
         auto chain = bulge(0.2);
         auto w = square_with({0.95, 0.05}, chain);
         auto r = w.split_boundary_edge(0);
         check(r.split && r.retriangulated, "split by re-triangulation");
         check(r.deleted_nodes == 0, "no node in the cut-off region");
         check(!w.has_deleted() || r.deleted_triangles > 0, "bookkeeping");
         Mesh m = w.to_mesh();
         check_valid_mesh(m);
         // The square minus the straight triangle (0,0), (1,0), (0.5, 0.2).
         near(mesh_area(m), 1 - 0.1, 1e-15, "area");
         near(m.nodes.back().x, 0.5, 1e-15, "new node at the curve point");
         near(m.nodes.back().y, 0.2, 1e-15, "new node at the curve point");
         check(m.boundary_edges.size() == 5, "five boundary edges");
         for (auto t : r.touched)
           check(!w.is_triangle_deleted(t), "touched triangles are live");
         // Links: the halves of the arc.
         const auto &e = w.boundary_edges();
         near(e[0].link->s1, chain.length() / 2, 0, "first half");
         near(e[4].link->s0, chain.length() / 2, 0, "second half");
       }},
      {"split_deletes_nodes_in_the_cut_off_region",
       [] {
         // The interior node (0.5, 0.08) lies between the chord and the
         // curve point (0.5, 0.2): outside the true domain, so it goes.
         auto w = square_with({0.5, 0.08}, bulge(0.2));
         auto r = w.split_boundary_edge(0);
         check(r.split && r.retriangulated, "split by re-triangulation");
         check(r.deleted_nodes == 1, "one node deleted");
         check(w.is_node_deleted(2), "node 2 is a tombstone");
         Mesh m = w.to_mesh();
         check_valid_mesh(m);
         check(m.nodes.size() == 5, "four corners and the curve point");
         for (auto p : m.nodes)
           check(!(p == Point{0.5, 0.08}), "the node is gone");
         near(mesh_area(m), 0.9, 1e-15, "area");
         // compact() gives the same numbering as to_mesh().
         w.compact();
         check(!w.has_deleted() && w.nodes().size() == 5, "compacted");
         check(same_mesh(w.to_mesh(), m), "same mesh");
         WorkingMesh copy(w.nodes(), w.triangles(), w.boundary_edges(),
                          w.region_names(), w.chains());
         check(copy.triangles().size() == w.triangles().size(),
               "a compacted mesh passes full validation");
       }},
      {"split_adds_a_cap_for_an_outward_bulge",
       [] {
         // The opposite node (-0.5, 0.1) lies beyond the chord's end, so
         // joining it to the curve point (0.5, -0.3) would invert a
         // triangle. The point lies outside the mesh: a triangle is added.
         auto chain = bulge(-0.3);
         WorkingMesh w({{0, 0}, {1, 0}, {-0.5, 0.1}}, {{0, 1, 2}},
                       {{{0, 1}, 0, CurveLink{0, 0, chain.length()}},
                        {{1, 2}, 0, {}},
                        {{2, 0}, 0, {}}},
                       {"r"}, {chain});
         auto r = w.split_boundary_edge(0);
         check(r.split && r.retriangulated,
               "the direct split would invert: split with a cap");
         Mesh m = w.to_mesh();
         check_valid_mesh(m);
         // Old triangle 0.05 plus the cap (0,0), (0.5,-0.3), (1,0): 0.15.
         near(mesh_area(m), 0.2, 1e-15, "area");
         check(m.triangles.size() == 2, "one triangle added");
       }},
      {"delaunay_routines",
       [] {
         // A long thin quad: the diagonal 0-2 is not Delaunay.
         WorkingMesh w({{0, 0}, {2, -0.2}, {4, 0}, {2, 0.2}},
                       {{0, 1, 2}, {0, 2, 3}},
                       {{{0, 1}, 0, {}},
                        {{1, 2}, 0, {}},
                        {{2, 3}, 0, {}},
                        {{3, 0}, 0, {}}},
                       {"a"});
         check(!is_locally_delaunay(w, 0, 1), "not Delaunay");
         check(flip_to_delaunay(w) == 1, "one flip");
         check(is_delaunay(w.to_mesh()), "Delaunay now");
         check(flip_to_delaunay(w) == 0, "nothing left to flip");
       }},
  });
}
