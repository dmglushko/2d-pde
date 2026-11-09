#pragma once
#include "boundary_discretization.hpp"
#include "mesh_types.hpp"
#include "mesh_validation.hpp"

namespace pde2d::discretization {
namespace mesh_detail {
// The polygon snapshot as a Boundary: the outer loop and its holes, split by
// snapshot_loop_sizes (empty: one loop).
inline Boundary snapshot_boundary(const BoundaryContext &context) {
  const auto &points = context.polygon_snapshot;
  if (context.snapshot_loop_sizes.empty())
    return Boundary{points, {}};
  if (context.snapshot_loop_sizes.size() < 2)
    throw std::invalid_argument("Invalid polygon snapshot loops");
  std::size_t total = 0;
  for (auto n : context.snapshot_loop_sizes) {
    if (n > points.size() - total)
      throw std::invalid_argument("Invalid polygon snapshot loops");
    total += n;
  }
  if (total != points.size())
    throw std::invalid_argument("Invalid polygon snapshot loops");
  Boundary b;
  auto at = points.begin();
  for (std::size_t k = 0; k < context.snapshot_loop_sizes.size(); ++k) {
    auto next =
        at + static_cast<std::ptrdiff_t>(context.snapshot_loop_sizes[k]);
    if (k == 0)
      b.points.assign(at, next);
    else
      b.holes.push_back({std::vector<Point2>(at, next), {}});
    at = next;
  }
  return b;
}
} // namespace mesh_detail
// A source context is an ordered, complete partition of the analytical loops
// (outer loop and holes). Every mesh boundary loop is sampled from the source
// curves of one geometry loop: curves[i].loop equals the index of that mesh
// loop in boundary_loops(mesh).
// Chord refinement may place its mesh vertices off these source curves, but
// it must not lose, duplicate, or reorder any source interval.
inline void validate_boundary_context(const TriangleMesh &mesh,
                                      const BoundaryContext &context,
                                      bool preserve_regions,
                                      bool preserve_corners = false) {
  using namespace mesh_detail;
  if (context.curves.empty() ||
      context.edge_sources.size() != mesh.boundary_edges.size())
    throw std::invalid_argument("Boundary source count mismatch");
  double extent = 0, magnitude = 1;
  auto origin = mesh.nodes.front();
  for (auto p : mesh.nodes) {
    extent = std::max(extent, distance(origin, p));
    magnitude = std::max({magnitude, std::abs(p.x), std::abs(p.y)});
  }
  double tolerance =
      1e-10 * extent + 16 * std::numeric_limits<double>::epsilon() * magnitude;
  auto close = [&](Point2 a, Point2 b) {
    return std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(b.x) &&
           std::isfinite(b.y) && distance(a, b) <= tolerance;
  };
  auto point_boxes = [](const std::vector<Point2> &points) {
    std::vector<Box> boxes;
    for (auto p : points) {
      boxes.push_back(std::isfinite(p.x) && std::isfinite(p.y)
                          ? Box::segment(p, p)
                          : Box{0, 0, 0, 0});
    }
    return boxes;
  };
  auto down = [](double x) {
    return std::nextafter(x, -std::numeric_limits<double>::infinity());
  };
  auto up = [](double x) {
    return std::nextafter(x, std::numeric_limits<double>::infinity());
  };
  auto near_box = [&](Point2 p) {
    return Box{down(p.x - tolerance), up(p.x + tolerance),
               down(p.y - tolerance), up(p.y + tolerance)};
  };
  // Keep original indices, even for malformed nonfinite snapshots.
  std::vector<Box> snapshot_boxes;
  for (auto p : context.polygon_snapshot)
    snapshot_boxes.push_back(std::isfinite(p.x) && std::isfinite(p.y)
                                 ? Box::segment(p, p)
                                 : Box{0, 0, 0, 0});
  BoxIndex snapshot_index(std::move(snapshot_boxes));
  std::set<std::uint64_t> ids;
  for (const auto &c : context.curves) {
    curve_detail::validateCurve(c.segment);
    if (!ids.insert(c.id.value).second)
      throw std::invalid_argument("Duplicate source curve ID");
  }
  std::vector<std::vector<std::pair<double, double>>> coverage(
      context.curves.size());
  std::map<std::size_t, std::size_t> outgoing;
  std::vector<Point2> source_anchors;
  std::set<std::size_t> protected_vertices;
  for (auto v : context.protected_vertices)
    if (v >= mesh.nodes.size() || !protected_vertices.insert(v).second)
      throw std::invalid_argument("Invalid protected vertex");
  for (std::size_t i = 0; i < mesh.boundary_edges.size(); ++i) {
    auto [a, b] = mesh.boundary_edges[i];
    (void)b;
    if (!outgoing.emplace(a, i).second)
      throw std::invalid_argument(
          "Source context requires a directed boundary loop");
    const auto &spans = context.edge_sources[i];
    if (spans.empty())
      throw std::invalid_argument("Missing boundary source interval");
    std::optional<Point2> previous;
    std::optional<CurveInterval> previous_span;
    for (auto s : spans) {
      if (s.curve >= context.curves.size() || !std::isfinite(s.begin) ||
          !std::isfinite(s.end) || s.begin < 0 || s.begin > 1 || s.end < 0 ||
          s.end > 1 || s.begin == s.end)
        throw std::invalid_argument("Invalid boundary source interval");
      const auto &c = context.curves[s.curve];
      if (preserve_corners && previous_span &&
          previous_span->curve != s.curve &&
          is_corner(subcurve(context.curves[previous_span->curve].segment,
                             previous_span->begin, previous_span->end),
                    subcurve(c.segment, s.begin, s.end)))
        throw std::invalid_argument(
            "Corner protection requires regenerating boundary discretization");
      auto start = curvePoint(c.segment, s.begin);
      source_anchors.push_back(start);
      source_anchors.push_back(curvePoint(c.segment, s.end));
      if (previous && !close(*previous, start))
        throw std::invalid_argument("Disconnected source intervals");
      previous = curvePoint(c.segment, s.end);
      previous_span = s;
      coverage[s.curve].push_back(std::minmax(s.begin, s.end));
      if (preserve_regions && !mesh.boundary_tags.empty() &&
          mesh.boundary_tags[i] != c.region)
        throw std::invalid_argument(
            "Boundary region disagrees with source curves");
    }
    // Chord subdivision may put new vertices off the analytical curves, but
    // vertices retained from the polygon snapshot must match their own spans.
    // Global coverage alone cannot detect cyclically shifted source records.
    bool snapshot_vertex = false;
    for (auto j : snapshot_index.candidates(near_box(mesh.nodes[a])))
      if (close(mesh.nodes[a], context.polygon_snapshot[j])) {
        snapshot_vertex = true;
        break;
      }
    if (protected_vertices.contains(a) || snapshot_vertex) {
      auto s = spans.front();
      if (!close(mesh.nodes[a],
                 curvePoint(context.curves[s.curve].segment, s.begin)))
        throw std::invalid_argument(
            "Boundary anchor disagrees with its source interval");
    }
  }
  const auto snapshot = snapshot_boundary(context);
  validate_polygon(snapshot);
  const auto loops = boundary_loops(mesh);
  if (loops.size() != loop_count(snapshot))
    throw std::invalid_argument(
        "Boundary loops disagree with the polygon snapshot");
  for (std::size_t k = 0; k < loops.size(); ++k)
    for (auto e : loops[k])
      for (auto s : context.edge_sources[e])
        if (context.curves[s.curve].loop != k)
          throw std::invalid_argument(
              "Source curve loop disagrees with its mesh boundary loop");
  BoxIndex anchor_index(point_boxes(source_anchors));
  for (auto p : context.polygon_snapshot) {
    bool anchored = false;
    for (auto j : anchor_index.candidates(near_box(p)))
      if (close(p, source_anchors[j])) {
        anchored = true;
        break;
      }
    if (!anchored)
      throw std::invalid_argument(
          "Polygon snapshot vertex is not anchored to source curves");
  }
  // Snapshot segments of every loop, each loop closing on itself.
  std::vector<std::pair<Point2, Point2>> segments;
  for (std::size_t k = 0; k < loop_count(snapshot); ++k) {
    const auto &points = loop_points(snapshot, k);
    for (std::size_t i = 0; i < points.size(); ++i)
      segments.push_back({points[i], points[(i + 1) % points.size()]});
  }
  std::vector<Box> segment_boxes;
  for (auto [p, q] : segments) {
    auto box = Box::segment(p, q);
    segment_boxes.push_back(
        {down(box.xmin - tolerance), up(box.xmax + tolerance),
         down(box.ymin - tolerance), up(box.ymax + tolerance)});
  }
  BoxIndex segment_index(std::move(segment_boxes));
  for (auto [a, b] : mesh.boundary_edges) {
    bool supported = false;
    for (auto i :
         segment_index.candidates(Box::segment(mesh.nodes[a], mesh.nodes[a]))) {
      auto [p, q] = segments[i];
      if (point_segment_distance(mesh.nodes[a], p, q) <= tolerance &&
          point_segment_distance(mesh.nodes[b], p, q) <= tolerance) {
        supported = true;
        break;
      }
    }
    if (!supported)
      throw std::invalid_argument(
          "Mesh boundary edge departed from its polygon snapshot");
  }
  for (auto v : protected_vertices)
    if (!outgoing.contains(v))
      throw std::invalid_argument("Protected vertex is not on the boundary");
  for (std::size_t i = 0; i < mesh.boundary_edges.size(); ++i) {
    auto [a, b] = mesh.boundary_edges[i];
    (void)a;
    auto next = outgoing.find(b);
    if (next == outgoing.end())
      throw std::invalid_argument("Disconnected source boundary");
    auto left = context.edge_sources[i].back(),
         right = context.edge_sources[next->second].front();
    if (!close(curvePoint(context.curves[left.curve].segment, left.end),
               curvePoint(context.curves[right.curve].segment, right.begin)))
      throw std::invalid_argument("Source boundary intervals do not join");
    if (preserve_regions &&
        context.curves[left.curve].region !=
            context.curves[right.curve].region &&
        !protected_vertices.contains(b))
      throw std::invalid_argument("Region junction is not protected");
    if (preserve_corners && left.curve != right.curve &&
        is_corner(
            subcurve(context.curves[left.curve].segment, left.begin, left.end),
            subcurve(context.curves[right.curve].segment, right.begin,
                     right.end)) &&
        !protected_vertices.contains(b))
      throw std::invalid_argument("Geometric corner is not protected");
  }
  for (auto &intervals : coverage) {
    std::sort(intervals.begin(), intervals.end());
    double end = 0;
    for (auto interval : intervals) {
      if (interval.first != end)
        throw std::invalid_argument(
            "Source curve has missing or overlapping intervals");
      end = interval.second;
    }
    if (end != 1)
      throw std::invalid_argument("Source curve is not fully represented");
  }
}
} // namespace pde2d::discretization
