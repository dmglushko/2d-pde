#include "pde2d/mesh/ear_clipping.hpp"

#include "polygon_nodes.hpp"

#include "pde2d/geometry/predicates.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <cmath>
#include <stdexcept>
#include <tuple>

namespace pde2d::mesh {

using geometry::orient;
using geometry::Point;

namespace {

// True if direction (to - at) lies strictly inside the domain's angle at
// vertex `at` of a ring whose neighbours are `before` and `after` (the
// domain is on the left of before -> at -> after).
bool in_sector(Point before, Point at, Point after, Point to) {
  // The domain sector runs counterclockwise from (after - at) to
  // (before - at).
  int turn = orient(at, after, before);
  int s1 = orient(at, after, to), s2 = orient(at, to, before);
  if (turn > 0)
    return s1 > 0 && s2 > 0;
  // A reflex or straight angle: inside unless in the closed complement.
  return !(orient(at, before, to) >= 0 && orient(at, to, after) >= 0);
}

struct Ring {
  std::vector<std::size_t> ids; // node ids, possibly repeated by bridges
};

// Joins `hole` (node ids in stored order, domain on the left) into
// `ring` with a bridge from the hole's rightmost vertex to the nearest
// ring vertex that sees it.
void bridge(Ring &ring, const std::vector<std::size_t> &hole,
            const std::vector<std::vector<std::size_t>> &others,
            const std::vector<Point> &P) {
  const std::size_t h = hole.size();
  std::size_t m = 0;
  for (std::size_t i = 1; i < h; ++i)
    if (std::tie(P[hole[i]].x, P[hole[m]].y) >
        std::tie(P[hole[m]].x, P[hole[i]].y))
      m = i;
  const Point M = P[hole[m]];
  const Point m_before = P[hole[(m + h - 1) % h]],
              m_after = P[hole[(m + 1) % h]];

  const std::size_t n = ring.ids.size();
  std::vector<std::size_t> order(n);
  for (std::size_t j = 0; j < n; ++j)
    order[j] = j;
  auto d2 = [&](std::size_t j) {
    Point d = P[ring.ids[j]] - M;
    return d.x * d.x + d.y * d.y;
  };
  std::stable_sort(order.begin(), order.end(),
                   [&](std::size_t a, std::size_t b) { return d2(a) < d2(b); });

  auto blocked_by = [&](const std::vector<std::size_t> &loop, Point V,
                        std::size_t v_id, std::size_t skip_id) {
    for (std::size_t k = 0; k < loop.size(); ++k) {
      std::size_t p = loop[k], q = loop[(k + 1) % loop.size()];
      if (p == v_id || q == v_id || p == skip_id || q == skip_id)
        continue;
      if (geometry::segments_intersect(M, V, P[p], P[q]))
        return true;
    }
    return false;
  };

  for (std::size_t j : order) {
    std::size_t v_id = ring.ids[j];
    Point V = P[v_id];
    if (!in_sector(m_before, M, m_after, V))
      continue;
    Point v_before = P[ring.ids[(j + n - 1) % n]],
          v_after = P[ring.ids[(j + 1) % n]];
    if (!in_sector(v_before, V, v_after, M))
      continue;
    if (blocked_by(ring.ids, V, v_id, hole[m]) ||
        blocked_by(hole, V, v_id, hole[m]))
      continue;
    bool free = true;
    for (const auto &other : others)
      if (blocked_by(other, V, v_id, hole[m])) {
        free = false;
        break;
      }
    if (!free)
      continue;
    std::vector<std::size_t> merged(ring.ids.begin(),
                                    ring.ids.begin() +
                                        static_cast<std::ptrdiff_t>(j + 1));
    for (std::size_t k = 0; k <= h; ++k)
      merged.push_back(hole[(m + k) % h]);
    merged.push_back(v_id);
    merged.insert(merged.end(),
                  ring.ids.begin() + static_cast<std::ptrdiff_t>(j + 1),
                  ring.ids.end());
    ring.ids = std::move(merged);
    return;
  }
  throw std::runtime_error("Ear clipping found no bridge to a hole");
}

// Cuts ears best-shaped first. The shape quality of a triangle is
// 4 sqrt(3) area / (sum of squared sides): 1 for an equilateral triangle,
// near 0 for a sliver. Cutting the best ear first keeps nearly collinear
// boundary points (straight boundaries are collinear only up to rounding)
// from being cut off as slivers.
void clip(const Ring &ring, const std::vector<Point> &P,
          std::vector<WorkingMesh::TriangleNodes> &out) {
  const std::size_t n = ring.ids.size();
  std::vector<std::size_t> prev(n), next(n), version(n, 0);
  std::vector<bool> removed(n, false);
  for (std::size_t i = 0; i < n; ++i) {
    prev[i] = (i + n - 1) % n;
    next[i] = (i + 1) % n;
  }
  auto is_ear = [&](std::size_t i) {
    std::size_t p = prev[i], q = next[i];
    std::size_t a = ring.ids[p], b = ring.ids[i], c = ring.ids[q];
    if (orient(P[a], P[b], P[c]) <= 0)
      return false;
    for (std::size_t r = next[q]; r != p; r = next[r]) {
      std::size_t x = ring.ids[r];
      if (x == a || x == b || x == c)
        continue;
      if (orient(P[a], P[b], P[x]) >= 0 && orient(P[b], P[c], P[x]) >= 0 &&
          orient(P[c], P[a], P[x]) >= 0)
        return false;
    }
    return true;
  };
  auto quality = [&](std::size_t i) {
    Point a = P[ring.ids[prev[i]]], b = P[ring.ids[i]],
          c = P[ring.ids[next[i]]];
    double area = 0.5 * geometry::cross(b - a, c - a);
    double sides = geometry::dot(b - a, b - a) + geometry::dot(c - b, c - b) +
                   geometry::dot(a - c, a - c);
    return 4 * std::sqrt(3.0) * area / sides;
  };
  // Max-heap of (quality, -position, version); stale entries are skipped.
  struct Entry {
    double quality;
    std::size_t position, version;
    bool operator<(const Entry &o) const {
      if (quality != o.quality)
        return quality < o.quality;
      return position > o.position;
    }
  };
  std::priority_queue<Entry> heap;
  auto offer = [&](std::size_t i) {
    ++version[i];
    if (is_ear(i))
      heap.push({quality(i), i, version[i]});
  };
  std::size_t remaining = n;
  auto rescan = [&] {
    for (std::size_t i = 0; i < n; ++i)
      if (!removed[i])
        offer(i);
  };
  rescan();
  while (remaining > 3) {
    if (heap.empty()) {
      rescan();
      if (heap.empty())
        throw std::runtime_error(
            "Ear clipping found no ear; the polygon is not simple");
    }
    Entry e = heap.top();
    heap.pop();
    std::size_t i = e.position;
    if (removed[i] || e.version != version[i])
      continue;
    std::size_t p = prev[i], q = next[i];
    out.push_back({ring.ids[p], ring.ids[i], ring.ids[q]});
    removed[i] = true;
    next[p] = q;
    prev[q] = p;
    --remaining;
    offer(p);
    offer(q);
  }
  std::size_t i = 0;
  while (removed[i])
    ++i;
  std::size_t p = prev[i], q = next[i];
  if (orient(P[ring.ids[p]], P[ring.ids[i]], P[ring.ids[q]]) <= 0)
    throw std::runtime_error("Ear clipping ended with a degenerate triangle");
  out.push_back({ring.ids[p], ring.ids[i], ring.ids[q]});
}

} // namespace

WorkingMesh EarClipping::run(const BoundaryPolygon &polygon,
                             StageReport &report) const {
  auto pn = detail::polygon_nodes(polygon);
  const auto &P = pn.nodes;
  auto loop_ids = [&](std::size_t l) {
    std::vector<std::size_t> ids(polygon.loops[l].points.size());
    for (std::size_t i = 0; i < ids.size(); ++i)
      ids[i] = pn.offset[l] + i;
    return ids;
  };
  std::vector<WorkingMesh::TriangleNodes> triangles;
  std::size_t pieces = 0, bridges = 0;
  for (std::size_t o = 0; o < polygon.loops.size(); ++o) {
    if (polygon.loops[o].depth % 2 != 0)
      continue;
    ++pieces;
    std::vector<std::size_t> holes;
    for (std::size_t l = 0; l < polygon.loops.size(); ++l)
      if (polygon.loops[l].parent == o)
        holes.push_back(l);
    // Rightmost holes first: then a visible ring vertex always exists.
    auto max_x = [&](std::size_t l) {
      double x = -INFINITY;
      for (auto p : polygon.loops[l].points)
        x = std::max(x, p.x);
      return x;
    };
    std::stable_sort(
        holes.begin(), holes.end(),
        [&](std::size_t a, std::size_t b) { return max_x(a) > max_x(b); });
    Ring ring{loop_ids(o)};
    for (std::size_t k = 0; k < holes.size(); ++k) {
      std::vector<std::vector<std::size_t>> others;
      for (std::size_t r = k + 1; r < holes.size(); ++r)
        others.push_back(loop_ids(holes[r]));
      bridge(ring, loop_ids(holes[k]), others, P);
      ++bridges;
    }
    clip(ring, P, triangles);
  }
  report.metrics["triangles"] = static_cast<double>(triangles.size());
  report.metrics["pieces"] = static_cast<double>(pieces);
  report.metrics["bridges"] = static_cast<double>(bridges);
  return WorkingMesh(std::move(pn.nodes), std::move(triangles),
                     std::move(pn.edges), polygon.region_names, polygon.chains);
}

} // namespace pde2d::mesh
