#include "pde2d/geometry/polygon.hpp"

#include "pde2d/geometry/bounding_box.hpp"

#include "pde2d/geometry/predicates.hpp"

#include <algorithm>
#include <array>
#include <stdexcept>

namespace pde2d::geometry {

double signed_area(std::span<const Point> polygon) {
  // Relative to the first vertex: the products are then as large as the
  // polygon, not as its distance from the origin, so they do not cancel
  // for a small polygon far away (a unit square at 1e9 would round to 0).
  if (polygon.size() < 3)
    return 0;
  Point o = polygon[0];
  double twice = 0;
  for (std::size_t i = 1; i + 1 < polygon.size(); ++i)
    twice += cross(polygon[i] - o, polygon[i + 1] - o);
  return twice / 2;
}

int polygon_orientation(std::span<const Point> polygon) {
  const std::size_t n = polygon.size();
  if (n < 3)
    return 0;
  auto lower = [](Point p, Point q) {
    return p.y < q.y || (p.y == q.y && p.x < q.x);
  };
  std::size_t v = 0;
  for (std::size_t i = 1; i < n; ++i)
    if (lower(polygon[i], polygon[v]))
      v = i;
  // The nearest points before and after v that differ from it.
  std::size_t u = (v + n - 1) % n, w = (v + 1) % n;
  while (u != v && polygon[u] == polygon[v])
    u = (u + n - 1) % n;
  while (w != v && polygon[w] == polygon[v])
    w = (w + 1) % n;
  if (u == v || w == v)
    return 0;
  // Both neighbours lie on or above the line through v, so for a simple
  // polygon the turn u -> v -> w has the polygon's orientation.
  return orient(polygon[u], polygon[v], polygon[w]);
}

int locate_in_polygon(std::span<const Point> polygon, Point p) {
  // Crossing number with a half-open rule on y, so a ray through a vertex
  // is counted once. All decisions use exact orientation tests.
  bool inside = false;
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    Point a = polygon[i], b = polygon[(i + 1) % polygon.size()];
    if (on_segment(a, b, p))
      return 0;
    if ((a.y > p.y) != (b.y > p.y)) {
      // The edge crosses the horizontal line through p. It crosses to the
      // right of p when p is on the left of the upward-directed edge.
      int side = a.y < b.y ? orient(a, b, p) : orient(b, a, p);
      if (side > 0)
        inside = !inside;
    }
  }
  return inside ? 1 : -1;
}

namespace {

struct SweepSegment {
  Point a, b;
  BoundingBox box;
  SegmentRef ref;
};

bool adjacent(const SweepSegment &s, const SweepSegment &t,
              std::span<const std::vector<Point>> loops) {
  if (s.ref.loop != t.ref.loop)
    return false;
  std::size_t n = loops[s.ref.loop].size();
  return (s.ref.segment + 1) % n == t.ref.segment ||
         (t.ref.segment + 1) % n == s.ref.segment;
}

bool share_more_than_vertex(const SweepSegment &s, const SweepSegment &t,
                            std::size_t n) {
  // s and t are consecutive: one ends where the other starts. They overlap
  // beyond that vertex only if one's far end lies on the other.
  const SweepSegment &first = (s.ref.segment + 1) % n == t.ref.segment ? s : t;
  const SweepSegment &second = &first == &s ? t : s;
  // first: p -> q, second: q -> r.
  if (n == 3 && orient(first.a, first.b, second.b) == 0)
    return true; // a degenerate triangle
  return on_segment(second.a, second.b, first.a) ||
         on_segment(first.a, first.b, second.b);
}

} // namespace

std::optional<Crossing>
find_crossing(std::span<const std::vector<Point>> loops) {
  std::vector<SweepSegment> segments;
  for (std::size_t l = 0; l < loops.size(); ++l) {
    const auto &loop = loops[l];
    if (loop.size() < 3)
      throw std::invalid_argument("A polygon loop needs at least 3 points");
    for (std::size_t i = 0; i < loop.size(); ++i) {
      Point a = loop[i], b = loop[(i + 1) % loop.size()];
      if (!is_finite(a))
        throw std::invalid_argument("Polygon points must be finite");
      SegmentRef ref{l, i};
      if (a == b)
        return Crossing{ref, ref};
      segments.push_back({a, b, bounding_box(std::array{a, b}), ref});
    }
  }
  std::sort(segments.begin(), segments.end(),
            [](const SweepSegment &s, const SweepSegment &t) {
              if (s.box.min.x != t.box.min.x)
                return s.box.min.x < t.box.min.x;
              if (s.ref.loop != t.ref.loop)
                return s.ref.loop < t.ref.loop;
              return s.ref.segment < t.ref.segment;
            });
  std::vector<std::size_t> active;
  for (std::size_t k = 0; k < segments.size(); ++k) {
    const auto &s = segments[k];
    std::erase_if(active, [&](std::size_t j) {
      return segments[j].box.max.x < s.box.min.x;
    });
    for (std::size_t j : active) {
      const auto &t = segments[j];
      if (t.box.max.y < s.box.min.y || s.box.max.y < t.box.min.y)
        continue;
      bool hit = adjacent(s, t, loops)
                     ? share_more_than_vertex(s, t, loops[s.ref.loop].size())
                     : segments_intersect(s.a, s.b, t.a, t.b);
      if (hit)
        return Crossing{t.ref, s.ref};
    }
    active.push_back(k);
  }
  return std::nullopt;
}

} // namespace pde2d::geometry
