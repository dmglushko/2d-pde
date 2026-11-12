#include "boundary_split_policy.hpp"

#include "edge_recovery.hpp"

#include "pde2d/geometry/predicates.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

namespace pde2d::mesh::detail {

using geometry::orient;
using geometry::Point;
using geometry::segments_intersect;

namespace {

// Where p lies relative to the closed counterclockwise triangle (a, b, c).
enum class Side { outside, boundary, inside };

Side side_of(Point a, Point b, Point c, Point p) {
  int s0 = orient(a, b, p), s1 = orient(b, c, p), s2 = orient(c, a, p);
  if (s0 < 0 || s1 < 0 || s2 < 0)
    return Side::outside;
  if (s0 == 0 || s1 == 0 || s2 == 0)
    return Side::boundary;
  return Side::inside;
}

bool segment_meets_triangle(Point u, Point v, Point a, Point b, Point c) {
  return side_of(a, b, c, u) != Side::outside ||
         side_of(a, b, c, v) != Side::outside ||
         segments_intersect(u, v, a, b) || segments_intersect(u, v, b, c) ||
         segments_intersect(u, v, c, a);
}

// The v1 policy (decided 2026-11-12): (a) split directly; (b) re-triangulate
// locally; (c) give up and leave the mesh unchanged. No chord fallback.
class ArcLengthMidpointPolicy final : public BoundarySplitPolicy {
public:
  BoundarySplitResult split(WorkingMesh &mesh, std::size_t k) const override {
    BoundarySplitResult result;
    const auto &P = mesh.nodes();
    auto [t, edge] = mesh.boundary_edge_location(k);
    if (t == no_index)
      throw std::logic_error("Boundary edge is not in the triangulation");
    auto [a, b] = mesh.boundary_edges()[k].nodes;
    std::size_t c = mesh.triangles()[t][static_cast<std::size_t>(edge)];
    const Point p = mesh.boundary_split_point(k);

    // (a) The point is inside the angle at c: two triangles replace one.
    if (orient(P[c], P[a], p) > 0 && orient(P[c], p, P[b]) > 0) {
      auto touched = mesh.split_boundary_edge_directly(k);
      result.split = true;
      result.touched.assign(touched.begin(), touched.end());
      return result;
    }
    result.retriangulated = true;
    int side = orient(P[a], P[b], p);
    if (side < 0)
      return outward(mesh, k, p, result);
    if (side == 0) {
      result.retriangulated = false;
      result.failure = "the curve point lies on the line of the chord but "
                       "outside it";
      return result;
    }
    return inward(mesh, k, t, p, result);
  }

private:
  // (b) for a curve that bulges out of the mesh: the triangle (a, p, b)
  // is glued onto the edge. Its neighbouring boundary edges must stay
  // clear of it.
  static BoundarySplitResult outward(WorkingMesh &mesh, std::size_t k, Point p,
                                     BoundarySplitResult &result) {
    const auto &P = mesh.nodes();
    auto [a, b] = mesh.boundary_edges()[k].nodes;
    // The boundary edges arriving at a and leaving b.
    std::size_t next = no_index, before = no_index;
    for (auto s : mesh.triangles_around(b))
      for (int i = 0; i < 3; ++i) {
        std::size_t j = mesh.boundary_edge_index(s, i);
        if (j != no_index && mesh.boundary_edges()[j].nodes[0] == b)
          next = j;
      }
    for (auto s : mesh.triangles_around(a))
      for (int i = 0; i < 3; ++i) {
        std::size_t j = mesh.boundary_edge_index(s, i);
        if (j != no_index && mesh.boundary_edges()[j].nodes[1] == a)
          before = j;
      }
    // The cap (a, p, b) is counterclockwise; a neighbouring edge may only
    // touch it at its shared end point.
    auto clear = [&](std::size_t j, std::size_t shared) {
      if (j == no_index)
        return true;
      auto [u, v] = mesh.boundary_edges()[j].nodes;
      std::size_t other = u == shared ? v : u;
      if (other == a || other == b) // a loop of two edges cannot occur
        return false;
      Point o = P[other];
      if (side_of(P[a], p, P[b], o) != Side::outside)
        return false;
      // From the shared corner the edge must leave outside the cap's
      // angle there.
      if (shared == a)
        return !(orient(P[a], p, o) >= 0 && orient(P[a], P[b], o) <= 0);
      return !(orient(P[b], P[a], o) >= 0 && orient(P[b], p, o) <= 0);
    };
    if (!clear(before, a) || !clear(next, b)) {
      result.retriangulated = false;
      result.failure = "the curve bulges out across a neighbouring boundary "
                       "edge";
      return result;
    }
    auto touched = mesh.add_boundary_cap(k);
    result.split = true;
    result.touched.assign(touched.begin(), touched.end());
    return result;
  }

  // (b) for a curve that bulges into the mesh: p is inserted, a-p and p-b
  // are forced by flips, and the region between them and the old edge
  // (the straight triangle a, b, p) is cut off with its nodes. Everything
  // that could make this fail is checked first, so a failure changes
  // nothing.
  static BoundarySplitResult inward(WorkingMesh &mesh, std::size_t k,
                                    std::size_t t, Point p,
                                    BoundarySplitResult &result) {
    const auto &P = mesh.nodes();
    const auto &T = mesh.triangles();
    auto [a, b] = mesh.boundary_edges()[k].nodes;
    const Point A = P[a], B = P[b];
    auto fail = [&](std::string why) {
      result.retriangulated = false;
      result.failure = std::move(why);
      return result;
    };

    // 1. Every triangle meeting the closed triangle (a, b, p), found by
    //    spreading from t across edges that meet it.
    std::vector<std::size_t> seen{t}, stack{t};
    std::vector<std::size_t> boundary_edges;
    while (!stack.empty()) {
      std::size_t s = stack.back();
      stack.pop_back();
      for (int i = 0; i < 3; ++i) {
        std::size_t u = T[s][static_cast<std::size_t>((i + 1) % 3)],
                    v = T[s][static_cast<std::size_t>((i + 2) % 3)];
        if (!segment_meets_triangle(P[u], P[v], A, B, p))
          continue;
        std::size_t n = mesh.neighbour(s, i);
        if (n == no_index) {
          std::size_t j = mesh.boundary_edge_index(s, i);
          if (j != k && std::find(boundary_edges.begin(), boundary_edges.end(),
                                  j) == boundary_edges.end())
            boundary_edges.push_back(j);
        } else if (std::find(seen.begin(), seen.end(), n) == seen.end()) {
          seen.push_back(n);
          stack.push_back(n);
        }
      }
    }
    // 2. Nodes in it: those strictly inside are cut off with the region and
    //    must be interior nodes; none may lie on a-p or p-b.
    for (auto s : seen)
      for (auto v : T[s]) {
        if (v == a || v == b)
          continue;
        Side where = side_of(A, B, p, P[v]);
        if (where == Side::outside)
          continue;
        if (where == Side::boundary || P[v] == p)
          return fail("a node lies on the new boundary edges");
        if (mesh.is_boundary_node(v))
          return fail("the curve bulges in past another boundary node");
      }
    // 3. Other boundary edges may only touch the region at a or b, from
    //    outside its angle there.
    for (auto j : boundary_edges) {
      auto [u, v] = mesh.boundary_edges()[j].nodes;
      bool ok;
      if (u == a || v == a || u == b || v == b) {
        std::size_t shared = (u == a || v == a) ? a : b;
        std::size_t other = u == shared ? v : u;
        Point o = P[other];
        ok = side_of(A, B, p, o) == Side::outside &&
             (shared == a ? !(orient(A, B, o) >= 0 && orient(A, p, o) <= 0)
                          : !(orient(B, p, o) >= 0 && orient(B, A, o) <= 0));
      } else {
        ok = !segment_meets_triangle(P[u], P[v], A, B, p);
      }
      if (!ok)
        return fail("the curve bulges in across another boundary edge");
    }
    // 4. Insert p into the triangle containing it.
    Location loc = mesh.locate(p, t);
    std::vector<std::size_t> touched;
    if (loc.kind == Location::Kind::inside) {
      auto c = mesh.insert_in_triangle(loc.triangle, p);
      touched.assign(c.begin(), c.end());
    } else if (loc.kind == Location::Kind::on_edge &&
               mesh.neighbour(loc.triangle, loc.index) != no_index) {
      auto c = mesh.split_interior_edge(loc.triangle, loc.index, p);
      touched.assign(c.begin(), c.end());
    } else {
      return fail("the curve point is not inside the mesh");
    }
    // Nothing above changed the mesh; from here on every step is known to
    // succeed, so an exception is a bug.
    std::size_t q = mesh.nodes().size() - 1;
    try {
      recover_edge(mesh, a, q, &touched);
      recover_edge(mesh, q, b, &touched);
      auto kept = mesh.cut_off_boundary_edge(k, q, &result.deleted_nodes,
                                             &result.deleted_triangles);
      touched.insert(touched.end(), kept.begin(), kept.end());
    } catch (const std::exception &e) {
      throw std::logic_error(
          std::string("Boundary split: local re-triangulation failed after "
                      "its checks passed: ") +
          e.what());
    }
    std::sort(touched.begin(), touched.end());
    touched.erase(std::unique(touched.begin(), touched.end()), touched.end());
    for (auto s : touched)
      if (!mesh.is_triangle_deleted(s))
        result.touched.push_back(s);
    result.split = true;
    return result;
  }
};

} // namespace

const BoundarySplitPolicy &boundary_split_policy() {
  static const ArcLengthMidpointPolicy policy;
  return policy;
}

} // namespace pde2d::mesh::detail
