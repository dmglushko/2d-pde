#pragma once
#include "boundary_context.hpp"
#include "boundary_discretization.hpp"
#include <deque>
#include <numeric>

namespace discretization {
namespace mesh_detail {
using Triangle = std::array<std::size_t, 3>;
inline Triangle ccw(Triangle t, const std::vector<Point2> &points) {
  int sign = predicates::orient(points[t[0]], points[t[1]], points[t[2]]);
  if (sign == 0)
    throw std::runtime_error("Operation would create a degenerate triangle");
  if (sign < 0)
    std::swap(t[1], t[2]);
  return t;
}
inline bool in_triangle(Point2 p, Point2 a, Point2 b, Point2 c) {
  return predicates::orient(a, b, p) >= 0 && predicates::orient(b, c, p) >= 0 &&
         predicates::orient(c, a, p) >= 0;
}
inline std::vector<Triangle> ear_clip(const Boundary &boundary) {
  validate_polygon(boundary);
  const auto &pts = boundary.points;
  std::vector<std::size_t> ring(pts.size());
  std::iota(ring.begin(), ring.end(), 0);
  // At the lexicographically least vertex the turn determines loop orientation,
  // including outlines with collinear samples (search through those samples).
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
  if (winding < 0)
    std::reverse(ring.begin(), ring.end());
  std::vector<Triangle> triangles;
  while (ring.size() > 3) {
    bool clipped = false;
    for (std::size_t i = 0; i < ring.size(); ++i) {
      auto a = ring[(i + ring.size() - 1) % ring.size()], b = ring[i],
           c = ring[(i + 1) % ring.size()];
      if (predicates::orient(pts[a], pts[b], pts[c]) <= 0)
        continue;
      bool contains = false;
      for (auto v : ring)
        if (v != a && v != b && v != c &&
            in_triangle(pts[v], pts[a], pts[b], pts[c])) {
          contains = true;
          break;
        }
      if (contains)
        continue;
      triangles.push_back({a, b, c});
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

class Topology {
public:
  TriangleMesh &mesh;
  std::map<Edge, std::set<std::size_t>> owners;
  std::set<Edge> constrained;
  explicit Topology(TriangleMesh &m) : mesh(m) {
    for (std::size_t i = 0; i < m.elements.size(); ++i)
      add(i);
    for (auto [a, b] : m.boundary_edges)
      constrained.insert(edge_key(a, b));
  }
  void add(std::size_t i) {
    auto t = mesh.elements[i];
    for (int e = 0; e < 3; ++e)
      owners[edge_key(t[e], t[(e + 1) % 3])].insert(i);
  }
  void remove(std::size_t i) {
    auto t = mesh.elements[i];
    for (int e = 0; e < 3; ++e) {
      auto key = edge_key(t[e], t[(e + 1) % 3]);
      auto it = owners.find(key);
      it->second.erase(i);
      if (it->second.empty())
        owners.erase(it);
    }
  }
  std::size_t opposite(std::size_t t, Edge e) const {
    for (auto v : mesh.elements[t])
      if (v != e.first && v != e.second)
        return v;
    throw std::logic_error("Missing opposite vertex");
  }
  bool illegal(Edge e) const {
    auto it = owners.find(e);
    if (it == owners.end() || it->second.size() != 2 || constrained.contains(e))
      return false;
    auto i = *it->second.begin(), j = *std::next(it->second.begin()),
         c = opposite(i, e), d = opposite(j, e);
    auto a = e.first, b = e.second;
    if (predicates::orient(mesh.nodes[c], mesh.nodes[d], mesh.nodes[a]) *
            predicates::orient(mesh.nodes[c], mesh.nodes[d], mesh.nodes[b]) >=
        0)
      return false;
    if (predicates::orient(mesh.nodes[a], mesh.nodes[b], mesh.nodes[c]) < 0)
      std::swap(a, b);
    return predicates::incircle(mesh.nodes[a], mesh.nodes[b], mesh.nodes[c],
                                mesh.nodes[d]) > 0;
  }
  void legalize() {
    std::deque<Edge> queue;
    for (const auto &[e, own] : owners) {
      (void)own;
      queue.push_back(e);
    }
    std::size_t flips = 0,
                limit = std::max<std::size_t>(
                    1000, mesh.elements.size() * mesh.elements.size() * 4);
    while (!queue.empty()) {
      auto edge = queue.front();
      queue.pop_front();
      if (!illegal(edge))
        continue;
      if (++flips > limit)
        throw std::runtime_error("Delaunay legalization work budget exhausted");
      const auto own = owners.at(edge);
      auto i = *own.begin(), j = *std::next(own.begin()), c = opposite(i, edge),
           d = opposite(j, edge);
      auto first = ccw({c, d, edge.first}, mesh.nodes),
           second = ccw({d, c, edge.second}, mesh.nodes);
      remove(i);
      remove(j);
      mesh.elements[i] = first;
      mesh.elements[j] = second;
      add(i);
      add(j);
      for (auto t : {first, second})
        for (int e = 0; e < 3; ++e)
          queue.push_back(edge_key(t[e], t[(e + 1) % 3]));
    }
  }
  void split(Edge edge, Point2 point) {
    auto it = owners.find(edge);
    if (it == owners.end())
      throw std::logic_error("Missing split edge");
    auto own = it->second;
    auto n = mesh.nodes.size();
    mesh.nodes.push_back(point);
    for (auto i : own) {
      auto c = opposite(i, edge);
      auto a = ccw({edge.first, n, c}, mesh.nodes),
           b = ccw({n, edge.second, c}, mesh.nodes);
      remove(i);
      mesh.elements[i] = a;
      add(i);
      mesh.elements.push_back(b);
      add(mesh.elements.size() - 1);
    }
    if (constrained.erase(edge)) {
      constrained.insert(edge_key(edge.first, n));
      constrained.insert(edge_key(n, edge.second));
    }
  }
  void insert(Point2 point) {
    for (auto p : mesh.nodes)
      if (p.x == point.x && p.y == point.y)
        return;
    for (std::size_t i = 0; i < mesh.elements.size(); ++i) {
      auto t = mesh.elements[i];
      if (!in_triangle(point, mesh.nodes[t[0]], mesh.nodes[t[1]],
                       mesh.nodes[t[2]]))
        continue;
      for (int e = 0; e < 3; ++e)
        if (predicates::on_segment(mesh.nodes[t[e]], mesh.nodes[t[(e + 1) % 3]],
                                   point)) {
          split(edge_key(t[e], t[(e + 1) % 3]), point);
          return;
        }
      auto n = mesh.nodes.size();
      mesh.nodes.push_back(point);
      remove(i);
      mesh.elements[i] = ccw({t[0], t[1], n}, mesh.nodes);
      add(i);
      mesh.elements.push_back(ccw({t[1], t[2], n}, mesh.nodes));
      add(mesh.elements.size() - 1);
      mesh.elements.push_back(ccw({t[2], t[0], n}, mesh.nodes));
      add(mesh.elements.size() - 1);
      return;
    }
    throw std::invalid_argument("Point lies outside the polygon");
  }
};
inline TriangleMesh seed_mesh(const Boundary &boundary) {
  TriangleMesh m;
  m.nodes = boundary.points;
  m.elements = ear_clip(boundary);
  m.boundary_tags = boundary.edge_tags;
  for (std::size_t i = 0; i < m.nodes.size(); ++i)
    m.boundary_edges.emplace_back(i, (i + 1) % m.nodes.size());
  return m;
}
inline bool inside_polygon(Point2 p, const Boundary &boundary) {
  int winding = 0;
  for (std::size_t i = 0; i < boundary.points.size(); ++i) {
    auto a = boundary.points[i],
         b = boundary.points[(i + 1) % boundary.points.size()];
    if (predicates::on_segment(a, b, p))
      return true;
    if (a.y <= p.y && b.y > p.y && predicates::orient(a, b, p) > 0)
      ++winding;
    if (a.y > p.y && b.y <= p.y && predicates::orient(a, b, p) < 0)
      --winding;
  }
  return winding != 0;
}
inline std::pair<std::vector<CurveInterval>, std::vector<CurveInterval>>
bisect_sources(const BoundaryContext &context,
               const std::vector<CurveInterval> &spans) {
  std::vector<double> lengths;
  double total = 0;
  for (auto s : spans) {
    double length =
        curveLength(subcurve(context.curves[s.curve].segment, s.begin, s.end));
    lengths.push_back(length);
    total += length;
  }
  if (!(total > 0) || !std::isfinite(total))
    throw std::runtime_error("Invalid source interval length");
  double target = total / 2, acc = 0;
  // Equal-length adjacent spans can differ by a rounding bit after curve
  // subdivision. Reuse that junction instead of creating an unresolvable sliver.
  const double junction_tolerance = 16 * std::numeric_limits<double>::epsilon() * total;
  std::vector<CurveInterval> left, right;
  for (std::size_t i = 0; i < spans.size(); ++i) {
    auto s = spans[i];
    double next = acc + lengths[i];
    if (next <= target + junction_tolerance)
      left.push_back(s);
    else if (acc >= target - junction_tolerance)
      right.push_back(s);
    else {
      auto piece = subcurve(context.curves[s.curve].segment, s.begin, s.end);
      double f = curveParameterAtLengthFraction(piece,
                                                (target - acc) / lengths[i]),
             t = s.begin + (s.end - s.begin) * f;
      if (!(t > std::min(s.begin, s.end) && t < std::max(s.begin, s.end)))
        throw std::runtime_error(
            "Boundary interval cannot be resolved further");
      left.push_back({s.curve, s.begin, t});
      right.push_back({s.curve, t, s.end});
    }
    acc = next;
  }
  if (left.empty() || right.empty())
    throw std::runtime_error("Boundary split could not be resolved");
  return {left, right};
}
inline void split_boundary_record(MeshResult &result, std::size_t i,
                                  std::size_t vertex) {
  auto &m = result.mesh;
  auto edge = m.boundary_edges[i];
  m.boundary_edges[i] = {edge.first, vertex};
  m.boundary_edges.insert(m.boundary_edges.begin() + i + 1,
                          {vertex, edge.second});
  if (!m.boundary_tags.empty())
    m.boundary_tags.insert(m.boundary_tags.begin() + i + 1, m.boundary_tags[i]);
  if (result.boundary_context) {
    auto &ctx = *result.boundary_context;
    auto halves = bisect_sources(ctx, ctx.edge_sources[i]);
    ctx.edge_sources[i] = std::move(halves.first);
    ctx.edge_sources.insert(ctx.edge_sources.begin() + i + 1,
                            std::move(halves.second));
  }
}
// Rebuild after a curve boundary update. Existing interior vertices are
// reinserted if still strictly interior; topology and indices may change.
inline void rebuild_curved(MeshResult &result) {
  auto &m = result.mesh;
  auto &ctx = *result.boundary_context;
  std::map<std::size_t, std::size_t> outgoing;
  for (std::size_t i = 0; i < m.boundary_edges.size(); ++i)
    if (!outgoing.emplace(m.boundary_edges[i].first, i).second)
      throw std::invalid_argument(
          "Curve refinement needs consistently directed boundary edges");
  std::vector<std::size_t> order;
  std::size_t current = m.boundary_edges.front().first;
  do {
    auto it = outgoing.find(current);
    if (it == outgoing.end())
      throw std::invalid_argument(
          "Curve refinement needs a directed boundary loop");
    order.push_back(it->second);
    current = m.boundary_edges[it->second].second;
  } while (current != m.boundary_edges.front().first &&
           order.size() <= m.boundary_edges.size());
  if (order.size() != m.boundary_edges.size())
    throw std::invalid_argument("Invalid boundary traversal");
  Boundary polygon;
  BoundaryContext next = ctx;
  next.edge_sources.clear();
  next.protected_vertices.clear();
  std::set<std::size_t> fixed(ctx.protected_vertices.begin(),
                              ctx.protected_vertices.end()),
      boundary_vertices;
  for (auto i : order) {
    auto vertex = m.boundary_edges[i].first;
    boundary_vertices.insert(vertex);
    auto s = ctx.edge_sources[i].front();
    polygon.points.push_back(curvePoint(ctx.curves[s.curve].segment, s.begin));
    if (!m.boundary_tags.empty())
      polygon.edge_tags.push_back(m.boundary_tags[i]);
    if (fixed.contains(vertex))
      next.protected_vertices.push_back(polygon.points.size() - 1);
    next.edge_sources.push_back(ctx.edge_sources[i]);
  }
  TriangleMesh replacement = seed_mesh(polygon);
  next.polygon_snapshot = polygon.points;
  Topology topology(replacement);
  std::size_t removed = 0;
  for (std::size_t i = 0; i < m.nodes.size(); ++i)
    if (!boundary_vertices.contains(i)) {
      auto p = m.nodes[i];
      bool on_boundary = false;
      for (std::size_t e = 0; e < polygon.points.size(); ++e)
        if (predicates::on_segment(
                polygon.points[e],
                polygon.points[(e + 1) % polygon.points.size()], p)) {
          on_boundary = true;
          break;
        }
      if (!on_boundary && inside_polygon(p, polygon))
        topology.insert(p);
      else
        ++removed;
    }
  topology.legalize();
  m = std::move(replacement);
  ctx = std::move(next);
  if (removed)
    result.report.message =
        "Curve update removed " + std::to_string(removed) +
        " interior vertices outside or on the updated boundary";
}
inline MeshResult refine_result(MeshResult result, const MeshOptions &options,
                                bool subdivision) {
  validate_options(options);
  validate_mesh_geometry(result.mesh);
  if (result.boundary_context)
    validate_boundary_context(result.mesh, *result.boundary_context,
                              options.preserve_regions);
  result.options = options;
  result.report = {};
  if (!options.preserve_regions)
    result.mesh.boundary_tags.clear();
  else if (result.mesh.boundary_tags.empty() && result.boundary_context) {
    for (const auto &spans : result.boundary_context->edge_sources) {
      auto name = result.boundary_context->curves[spans.front().curve].region;
      for (auto span : spans)
        if (result.boundary_context->curves[span.curve].region != name)
          throw std::invalid_argument(
              "Enabling regions requires regenerating boundary discretization "
              "at region junctions");
      result.mesh.boundary_tags.push_back(name);
    }
  }
  if (options.boundary_treatment == BoundaryTreatment::RefineOnCurves &&
      !result.boundary_context)
    throw std::invalid_argument(
        "Curve refinement requires original boundary context");
  bool delaunay = !subdivision;
  if (options.boundary_treatment == BoundaryTreatment::RefineOnCurves &&
      options.refinement.max_edge_length > 0) {
    bool off_curve = false;
    for (std::size_t i = 0; i < result.mesh.boundary_edges.size(); ++i) {
      auto s = result.boundary_context->edge_sources[i].front();
      auto p =
          curvePoint(result.boundary_context->curves[s.curve].segment, s.begin);
      if (distance(p, result.mesh.nodes[result.mesh.boundary_edges[i].first]) >
          0)
        off_curve = true;
    }
    if (off_curve)
      rebuild_curved(result);
  }
  if (delaunay) {
    Topology topology(result.mesh);
    topology.legalize();
  }
  double h = options.refinement.max_edge_length;
  if (h > 0 && options.boundary_treatment == BoundaryTreatment::Fixed) {
    for (auto [a, b] : result.mesh.boundary_edges)
      if (distance(result.mesh.nodes[a], result.mesh.nodes[b]) >
          h * (1 + 1e-12)) {
        result.report.size_satisfied = false;
        result.report.delaunay_verified = delaunay;
        result.report.message = "Fixed boundary edge exceeds requested maximum "
                                "triangle-edge length";
        return result;
      }
  }
  for (int step = 0; h > 0; ++step) {
    Topology topology(result.mesh);
    Edge longest{};
    double length = h * (1 + 1e-12);
    bool found = false;
    for (const auto &[edge, own] : topology.owners) {
      (void)own;
      double d = distance(result.mesh.nodes[edge.first],
                          result.mesh.nodes[edge.second]);
      if (d > length) {
        longest = edge;
        length = d;
        found = true;
      }
    }
    if (!found)
      break;
    if (step >= options.refinement.max_steps) {
      result.report.size_satisfied = false;
      result.report.message = "Refinement step budget exhausted";
      break;
    }
    Point2 a = result.mesh.nodes[longest.first],
           b = result.mesh.nodes[longest.second];
    Point2 mid{std::midpoint(a.x, b.x), std::midpoint(a.y, b.y)};
    if ((mid.x == a.x && mid.y == a.y) || (mid.x == b.x && mid.y == b.y)) {
      result.report.size_satisfied = false;
      result.report.message = "Refinement reached coordinate precision limit";
      break;
    }
    auto it = std::find_if(
        result.mesh.boundary_edges.begin(), result.mesh.boundary_edges.end(),
        [&](auto e) { return edge_key(e.first, e.second) == longest; });
    bool boundary = it != result.mesh.boundary_edges.end();
    auto index =
        static_cast<std::size_t>(it - result.mesh.boundary_edges.begin());
    if (boundary &&
        options.boundary_treatment == BoundaryTreatment::RefineOnCurves) {
      // A temporary vertex supplies connectivity; rebuild samples its actual
      // curve parameter and does not reuse its midpoint coordinates.
      auto n = result.mesh.nodes.size();
      result.mesh.nodes.push_back(mid);
      split_boundary_record(result, index, n);
      rebuild_curved(result);
    } else {
      auto n = result.mesh.nodes.size();
      topology.split(longest, mid);
      if (boundary)
        split_boundary_record(result, index, n);
      if (delaunay)
        topology.legalize();
    }
    ++result.report.refinement_steps;
  }
  result.report.delaunay_verified = delaunay;
  validate_mesh_geometry(result.mesh);
  return result;
}
} // namespace mesh_detail

inline bool is_constrained_delaunay(const TriangleMesh &mesh) {
  validate_mesh_geometry(mesh);
  auto copy = mesh;
  mesh_detail::Topology topology(copy);
  for (const auto &[edge, own] : topology.owners) {
    (void)own;
    if (topology.illegal(edge))
      return false;
  }
  return true;
}
inline MeshResult generate_mesh(const DiscretizedBoundary &boundary,
                                const MeshOptions &options = {}) {
  mesh_detail::validate_options(options);
  MeshResult result;
  result.mesh = mesh_detail::seed_mesh(boundary.boundary);
  result.boundary_context = boundary.context;
  result.options = options;
  if (!options.preserve_regions)
    result.mesh.boundary_tags.clear();
  if (options.algorithm == InteriorAlgorithm::ConstrainedDelaunay)
    return mesh_detail::refine_result(std::move(result), options, false);
  validate_mesh_geometry(result.mesh);
  if (result.boundary_context)
    validate_boundary_context(result.mesh, *result.boundary_context,
                              options.preserve_regions);
  return result;
}
inline MeshResult generate_mesh(const Boundary &boundary,
                                const MeshOptions &options = {}) {
  return generate_mesh(DiscretizedBoundary{boundary, std::nullopt}, options);
}
inline MeshResult generate_mesh(const CurveGeometry &geometry,
                                const MeshOptions &options = {}) {
  mesh_detail::validate_options(options);
  return generate_mesh(
      discretize_boundary(geometry, options.boundary, options.preserve_regions),
      options);
}
inline MeshResult
refine_mesh(const TriangleMesh &mesh, const MeshOptions &options = {},
            std::optional<BoundaryContext> context = std::nullopt) {
  if (options.algorithm != InteriorAlgorithm::ConstrainedDelaunay)
    throw std::invalid_argument("refine_mesh uses constrained Delaunay; use "
                                "subdivide_mesh for subdivision");
  return mesh_detail::refine_result(
      MeshResult{mesh, std::move(context), options, {}}, options, false);
}
inline MeshResult refine_mesh(const MeshResult &previous,
                              const MeshOptions &options) {
  return refine_mesh(previous.mesh, options, previous.boundary_context);
}
inline MeshResult
subdivide_mesh(const TriangleMesh &mesh, const MeshOptions &options = {},
               std::optional<BoundaryContext> context = std::nullopt) {
  if (options.boundary_treatment == BoundaryTreatment::RefineOnCurves)
    throw std::invalid_argument("Subdivision preserves the polygon; curve "
                                "updates require Delaunay refinement");
  return mesh_detail::refine_result(
      MeshResult{mesh, std::move(context), options, {}}, options, true);
}
} // namespace discretization
