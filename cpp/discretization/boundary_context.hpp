#pragma once
#include "boundary_discretization.hpp"
#include "mesh_types.hpp"
#include "mesh_validation.hpp"

namespace discretization {
// A source context is an ordered, complete partition of the analytical loop.
// Chord refinement may place its mesh vertices off these source curves, but
// it must not lose, duplicate, or reorder any source interval.
inline void validate_boundary_context(const TriangleMesh &mesh,
                                      const BoundaryContext &context,
                                      bool preserve_regions) {
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
    for (auto s : spans) {
      if (s.curve >= context.curves.size() || !std::isfinite(s.begin) ||
          !std::isfinite(s.end) || s.begin < 0 || s.begin > 1 || s.end < 0 ||
          s.end > 1 || s.begin == s.end)
        throw std::invalid_argument("Invalid boundary source interval");
      const auto &c = context.curves[s.curve];
      auto start = curvePoint(c.segment, s.begin);
      source_anchors.push_back(start);
      source_anchors.push_back(curvePoint(c.segment, s.end));
      if (previous && !close(*previous, start))
        throw std::invalid_argument("Disconnected source intervals");
      previous = curvePoint(c.segment, s.end);
      coverage[s.curve].push_back(std::minmax(s.begin, s.end));
      if (preserve_regions && !mesh.boundary_tags.empty() &&
          mesh.boundary_tags[i] != c.region)
        throw std::invalid_argument(
            "Boundary region disagrees with source curves");
    }
    // Chord subdivision may put new vertices off the analytical curves, but
    // vertices retained from the polygon snapshot must match their own spans.
    // Global coverage alone cannot detect cyclically shifted source records.
    bool snapshot_vertex = std::any_of(
        context.polygon_snapshot.begin(), context.polygon_snapshot.end(),
        [&](Point2 p) { return close(mesh.nodes[a], p); });
    if (protected_vertices.contains(a) || snapshot_vertex) {
      auto s = spans.front();
      if (!close(mesh.nodes[a],
                 curvePoint(context.curves[s.curve].segment, s.begin)))
        throw std::invalid_argument(
            "Boundary anchor disagrees with its source interval");
    }
  }
  validate_polygon(Boundary{context.polygon_snapshot, {}});
  for (auto p : context.polygon_snapshot) {
    if (std::none_of(source_anchors.begin(), source_anchors.end(),
                     [&](auto q) { return close(p, q); }))
      throw std::invalid_argument(
          "Polygon snapshot vertex is not anchored to source curves");
  }
  for (auto [a, b] : mesh.boundary_edges) {
    bool supported = false;
    for (std::size_t i = 0; i < context.polygon_snapshot.size(); ++i) {
      auto p = context.polygon_snapshot[i],
           q = context
                   .polygon_snapshot[(i + 1) % context.polygon_snapshot.size()];
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
    auto u = tangent(
        subcurve(context.curves[left.curve].segment, left.begin, left.end),
        true);
    auto v = tangent(
        subcurve(context.curves[right.curve].segment, right.begin, right.end),
        false);
    double ul = std::hypot(u.x, u.y), vl = std::hypot(v.x, v.y);
    bool corner = ul == 0 || vl == 0;
    if (!corner) {
      u = {u.x / ul, u.y / ul};
      v = {v.x / vl, v.y / vl};
      corner =
          std::abs(u.x * v.y - u.y * v.x) > 1e-10 || u.x * v.x + u.y * v.y <= 0;
    }
    if (corner && left.curve != right.curve && !protected_vertices.contains(b))
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
} // namespace discretization
