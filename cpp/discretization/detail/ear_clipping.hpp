#pragma once
// Private (mesh_detail) part of triangulate.hpp: ear clipping of the sampled
// boundary polygon, the seed triangulation of every meshing algorithm.
#include "../geometry_base.hpp"
#include "../mesh_validation.hpp"
#include "../predicates.hpp"
#include "../spatial_index.hpp"
#include "cdt_topology.hpp"
#include <algorithm>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <tuple>
#include <vector>

namespace pde2d::discretization::mesh_detail {
// Turn at the lexicographically least vertex of a closed ring of point
// indices: +1 counterclockwise, -1 clockwise. Collinear samples next to that
// vertex are searched through.
inline int ring_winding(const std::vector<Point2> &pts,
                        const std::vector<std::size_t> &ring) {
  auto least = std::min_element(ring.begin(), ring.end(), [&](auto a, auto b) {
    return std::tie(pts[a].x, pts[a].y) < std::tie(pts[b].x, pts[b].y);
  });
  auto k = static_cast<std::size_t>(least - ring.begin());
  int winding = 0;
  for (std::size_t offset = 1; offset < ring.size() && !winding; ++offset)
    winding =
        predicates::orient(pts[ring[(k + ring.size() - offset) % ring.size()]],
                           pts[*least], pts[ring[(k + offset) % ring.size()]]);
  if (!winding)
    throw std::invalid_argument("Zero-area polygon");
  return winding;
}
// Ear clipping of a counterclockwise ring of point indices. Indices may
// repeat (the two ends of a hole bridge occur twice); a point keeps blocking
// ears while any of its occurrences remains in the ring.
inline std::vector<Triangle> clip_ring(const std::vector<Point2> &pts,
                                       std::vector<std::size_t> ring) {
  std::vector<Box> point_boxes;
  for (auto p : pts)
    point_boxes.push_back(Box::segment(p, p));
  BoxIndex point_index(std::move(point_boxes));
  std::vector<std::size_t> active(pts.size(), 0);
  for (auto v : ring)
    ++active[v];
  std::vector<Triangle> triangles;
  while (ring.size() > 3) {
    bool clipped = false;
    for (std::size_t i = 0; i < ring.size(); ++i) {
      auto a = ring[(i + ring.size() - 1) % ring.size()], b = ring[i],
           c = ring[(i + 1) % ring.size()];
      if (predicates::orient(pts[a], pts[b], pts[c]) <= 0)
        continue;
      bool contains = false;
      auto box = Box::segment(pts[a], pts[b]);
      box = {std::min(box.xmin, pts[c].x), std::max(box.xmax, pts[c].x),
             std::min(box.ymin, pts[c].y), std::max(box.ymax, pts[c].y)};
      for (auto v : point_index.candidates(box))
        if (active[v] && v != a && v != b && v != c &&
            in_triangle(pts[v], pts[a], pts[b], pts[c])) {
          contains = true;
          break;
        }
      if (contains)
        continue;
      triangles.push_back({a, b, c});
      --active[b];
      ring.erase(ring.begin() + i);
      clipped = true;
      break;
    }
    if (!clipped)
      throw std::runtime_error("Could not find a nondegenerate ear");
  }
  triangles.push_back(ccw({ring[0], ring[1], ring[2]}, pts));
  return triangles;
}
// Whether m lies strictly inside the interior angle at v of a
// counterclockwise ring ... p, v, n ... (the domain is on the left).
inline bool in_wedge(Point2 p, Point2 v, Point2 n, Point2 m) {
  const int turn = predicates::orient(p, v, n),
            incoming = predicates::orient(p, v, m),
            outgoing = predicates::orient(v, n, m);
  if (turn > 0)
    return incoming > 0 && outgoing > 0;
  if (turn < 0)
    return incoming > 0 || outgoing > 0;
  return incoming > 0;
}
// One counterclockwise ring through the outer loop and every hole: each hole
// is joined by a bridge (a segment traversed in both directions) from its
// rightmost vertex M to a visible vertex V of the ring built so far. Holes go
// in decreasing order of their rightmost vertex, so a visible ring vertex
// always exists (the ray from M towards +x meets only the ring); the nearest
// visible occurrence is taken. Visibility uses exact predicates: the bridge
// may touch no loop edge or earlier bridge except at its own endpoints, and
// must leave V inside V's interior angle (which picks the right occurrence
// of a vertex that is already a bridge end).
inline std::vector<std::size_t>
bridged_ring(const std::vector<Point2> &pts,
             const std::vector<std::vector<std::size_t>> &loops) {
  auto ring = loops[0];
  if (ring_winding(pts, ring) < 0)
    std::reverse(ring.begin(), ring.end());
  struct Segment {
    std::size_t a, b;
  };
  std::vector<Segment> edges, bridges;
  std::vector<Box> boxes;
  for (const auto &loop : loops)
    for (std::size_t i = 0; i < loop.size(); ++i) {
      edges.push_back({loop[i], loop[(i + 1) % loop.size()]});
      boxes.push_back(Box::segment(pts[edges.back().a], pts[edges.back().b]));
    }
  BoxIndex edge_index(std::move(boxes));
  auto rightmost = [&](const std::vector<std::size_t> &loop) {
    return *std::max_element(loop.begin(), loop.end(), [&](auto a, auto b) {
      return std::tie(pts[a].x, pts[a].y) < std::tie(pts[b].x, pts[b].y);
    });
  };
  // A segment sharing endpoint `shared` with the bridge (shared, far) is
  // harmless unless the two overlap.
  auto overlaps = [&](std::size_t shared, std::size_t far, std::size_t other) {
    return predicates::on_segment(pts[shared], pts[far], pts[other]) ||
           predicates::on_segment(pts[shared], pts[other], pts[far]);
  };
  auto blocks = [&](Segment s, std::size_t m, std::size_t v) {
    if (s.a == m || s.a == v)
      return overlaps(s.a, s.a == m ? v : m, s.b);
    if (s.b == m || s.b == v)
      return overlaps(s.b, s.b == m ? v : m, s.a);
    return predicates::intersects(pts[m], pts[v], pts[s.a], pts[s.b]);
  };
  auto visible = [&](std::size_t m, std::size_t v) {
    for (auto j : edge_index.candidates(Box::segment(pts[m], pts[v])))
      if (blocks(edges[j], m, v))
        return false;
    for (auto s : bridges)
      if (blocks(s, m, v))
        return false;
    return true;
  };
  std::vector<std::size_t> order(loops.size() - 1);
  std::iota(order.begin(), order.end(), 1);
  std::stable_sort(order.begin(), order.end(), [&](auto g, auto h) {
    auto a = rightmost(loops[g]), b = rightmost(loops[h]);
    return std::tie(pts[b].x, pts[b].y) < std::tie(pts[a].x, pts[a].y);
  });
  for (auto h : order) {
    auto hole = loops[h];
    // Inside the merged ring a hole is traversed clockwise.
    if (ring_winding(pts, hole) > 0)
      std::reverse(hole.begin(), hole.end());
    const auto m = rightmost(hole);
    const auto start = static_cast<std::size_t>(
        std::find(hole.begin(), hole.end(), m) - hole.begin());
    std::vector<std::size_t> positions(ring.size());
    std::iota(positions.begin(), positions.end(), 0);
    auto squared = [&](std::size_t v) {
      double dx = pts[v].x - pts[m].x, dy = pts[v].y - pts[m].y;
      return dx * dx + dy * dy;
    };
    std::stable_sort(positions.begin(), positions.end(), [&](auto i, auto j) {
      return squared(ring[i]) < squared(ring[j]);
    });
    std::optional<std::size_t> bridge;
    for (auto i : positions) {
      const auto v = ring[i];
      if (in_wedge(pts[ring[(i + ring.size() - 1) % ring.size()]], pts[v],
                   pts[ring[(i + 1) % ring.size()]], pts[m]) &&
          visible(m, v)) {
        bridge = i;
        break;
      }
    }
    if (!bridge)
      throw std::runtime_error("Could not bridge a hole to the outer boundary");
    std::vector<std::size_t> splice;
    splice.reserve(hole.size() + 2);
    for (std::size_t k = 0; k <= hole.size(); ++k)
      splice.push_back(hole[(start + k) % hole.size()]);
    splice.push_back(ring[*bridge]);
    ring.insert(ring.begin() + static_cast<std::ptrdiff_t>(*bridge) + 1,
                splice.begin(), splice.end());
    bridges.push_back({m, ring[*bridge]});
  }
  return ring;
}
inline std::vector<Triangle> ear_clip(const Boundary &boundary) {
  validate_polygon(boundary);
  if (!boundary.holes.empty()) {
    const auto pts = boundary_points(boundary);
    std::vector<std::vector<std::size_t>> loops;
    std::size_t offset = 0;
    for (std::size_t k = 0; k < loop_count(boundary); ++k) {
      auto &loop = loops.emplace_back(loop_points(boundary, k).size());
      std::iota(loop.begin(), loop.end(), offset);
      offset += loop.size();
    }
    return clip_ring(pts, bridged_ring(pts, loops));
  }
  const auto &pts = boundary.points;
  std::vector<std::size_t> ring(pts.size());
  std::iota(ring.begin(), ring.end(), 0);
  // At the lexicographically least vertex the turn determines loop orientation,
  // including outlines with collinear samples (search through those samples).
  if (ring_winding(pts, ring) < 0)
    std::reverse(ring.begin(), ring.end());
  return clip_ring(pts, std::move(ring));
}
// Nodes are the loop vertices (outer loop first, then each hole); boundary
// edges follow each loop in its given direction, loop after loop.
inline TriangleMesh seed_mesh(const Boundary &boundary) {
  TriangleMesh m;
  m.nodes = boundary_points(boundary);
  m.elements = ear_clip(boundary);
  m.boundary_tags = boundary.edge_tags;
  bool tagged = !boundary.edge_tags.empty();
  for (const auto &hole : boundary.holes)
    tagged = tagged || !hole.edge_tags.empty();
  std::size_t offset = 0;
  for (std::size_t k = 0; k < loop_count(boundary); ++k) {
    const auto n = loop_points(boundary, k).size();
    for (std::size_t i = 0; i < n; ++i)
      m.boundary_edges.emplace_back(offset + i, offset + (i + 1) % n);
    if (k > 0 && tagged) {
      const auto &tags = boundary.holes[k - 1].edge_tags;
      if (m.boundary_tags.empty())
        m.boundary_tags.resize(offset);
      if (tags.empty())
        m.boundary_tags.resize(offset + n);
      else
        m.boundary_tags.insert(m.boundary_tags.end(), tags.begin(), tags.end());
    }
    offset += n;
  }
  return m;
}
} // namespace pde2d::discretization::mesh_detail
