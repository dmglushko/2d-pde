#pragma once
#include "boundary_context.hpp"
#include "boundary_discretization.hpp"
#include <deque>
#include <memory>
#include <numeric>

namespace pde2d::discretization {
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
  std::vector<Box> point_boxes;
  for (auto p : pts)
    point_boxes.push_back(Box::segment(p, p));
  BoxIndex point_index(std::move(point_boxes));
  std::vector<bool> active(pts.size(), true);
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
      active[b] = false;
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
  std::set<Edge> pending;
  std::map<Edge, std::size_t> generations;
  std::size_t generation = 0;
  struct SizeEntry {
    double length;
    Edge edge;
    std::size_t generation;
    bool operator<(const SizeEntry &b) const {
      if (length != b.length)
        return length < b.length;
      return edge > b.edge; // first lexicographic edge wins equal lengths
    }
  };
  std::priority_queue<SizeEntry> sizes;
  double target = 0;
  std::set<std::pair<double, double>> coordinates;
  void queue_size(Edge e) {
    auto g = ++generation;
    generations[e] = g;
    if (target > 0) {
      double d = distance(mesh.nodes[e.first], mesh.nodes[e.second]);
      if (d > target * (1 + 1e-12))
        sizes.push({d, e, g});
    }
  }
  void initialize_size(double h) {
    target = h;
    sizes = {};
    for (const auto &[e, own] : owners) {
      (void)own;
      queue_size(e);
    }
  }
  std::optional<Edge> longest() {
    while (!sizes.empty()) {
      auto t = sizes.top();
      auto g = generations.find(t.edge);
      if (!owners.contains(t.edge) || g == generations.end() ||
          g->second != t.generation) {
        sizes.pop();
        continue;
      }
      return t.edge;
    }
    return std::nullopt;
  }
  explicit Topology(TriangleMesh &m) : mesh(m) {
    for (auto p : m.nodes)
      coordinates.emplace(p.x, p.y);
    for (std::size_t i = 0; i < m.elements.size(); ++i)
      add(i);
    for (auto [a, b] : m.boundary_edges)
      constrained.insert(edge_key(a, b));
  }
  void add(std::size_t i) {
    auto t = mesh.elements[i];
    for (int e = 0; e < 3; ++e) {
      auto key = edge_key(t[e], t[(e + 1) % 3]);
      auto &own = owners[key];
      own.insert(i);
      if (own.size() > 2)
        throw std::invalid_argument(
            "Nonmanifold mesh or incorrect boundary records");
      pending.insert(key);
      queue_size(key);
    }
  }
  void remove(std::size_t i) {
    auto t = mesh.elements[i];
    for (int e = 0; e < 3; ++e) {
      auto key = edge_key(t[e], t[(e + 1) % 3]);
      auto it = owners.find(key);
      it->second.erase(i);
      if (it->second.empty()) {
        owners.erase(it);
        generations.erase(key);
      }
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
  void legalize(bool global = true) {
    if (global)
      for (const auto &[e, own] : owners) {
        (void)own;
        pending.insert(e);
      }
    std::deque<Edge> queue(pending.begin(), pending.end());

    std::size_t flips = 0,
                limit = std::max<std::size_t>(
                    1000, mesh.elements.size() * mesh.elements.size() * 4);
    while (!queue.empty()) {
      auto edge = queue.front();
      queue.pop_front();
      pending.erase(edge);
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
      // add() records size changes, including flip-created oversized edges.
      // Queue each affected edge once, allowing later re-enqueue after popping.
      std::set<Edge> affected;
      for (auto t : {first, second})
        for (int e = 0; e < 3; ++e)
          affected.insert(edge_key(t[e], t[(e + 1) % 3]));
      for (auto e : affected)
        if (!pending.contains(e))
          queue.push_back(e);
      add(i);
      add(j);
    }
  }
  void split(Edge edge, Point2 point) {
    auto it = owners.find(edge);
    if (it == owners.end())
      throw std::logic_error("Missing split edge");
    auto own = it->second;
    auto n = mesh.nodes.size();
    mesh.nodes.push_back(point);
    coordinates.emplace(point.x, point.y);
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
    if (coordinates.contains({point.x, point.y}))
      return;
    for (std::size_t i = 0; i < mesh.elements.size(); ++i) {
      auto t = mesh.elements[i];
      auto box = Box::segment(mesh.nodes[t[0]], mesh.nodes[t[1]]);
      auto p = mesh.nodes[t[2]];
      box = {std::min(box.xmin, p.x), std::max(box.xmax, p.x),
             std::min(box.ymin, p.y), std::max(box.ymax, p.y)};
      if (!box.overlaps(Box::segment(point, point)))
        continue;
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
      coordinates.emplace(point.x, point.y);
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
inline bool inside_polygon(Point2 p, const Boundary &boundary,
                           bool strict = false) {
  int winding = 0;
  for (std::size_t i = 0; i < boundary.points.size(); ++i) {
    auto a = boundary.points[i],
         b = boundary.points[(i + 1) % boundary.points.size()];
    if (predicates::on_segment(a, b, p))
      return !strict;
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
  // subdivision. Reuse that junction instead of creating an unresolvable
  // sliver.
  const double junction_tolerance =
      16 * std::numeric_limits<double>::epsilon() * total;
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
      if (inside_polygon(p, polygon, true))
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
                              options.preserve_regions,
                              options.boundary.preserve_corners);
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
  auto topology = std::make_unique<Topology>(result.mesh);
  if (delaunay)
    topology->legalize();
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
  topology->initialize_size(h);
  for (int step = 0; h > 0; ++step) {
    auto candidate = topology->longest();
    if (!candidate)
      break;
    Edge longest = *candidate;
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
    auto it = result.mesh.boundary_edges.end();
    if (topology->constrained.contains(longest))
      it = std::find_if(
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
      topology = std::make_unique<Topology>(result.mesh);
      topology->initialize_size(h);
    } else {
      auto n = result.mesh.nodes.size();
      topology->split(longest, mid);
      if (boundary)
        split_boundary_record(result, index, n);
      if (delaunay)
        topology->legalize(false);
    }
    ++result.report.refinement_steps;
  }
  result.report.delaunay_verified = delaunay;
  validate_mesh_geometry(result.mesh);
  return result;
}
} // namespace mesh_detail

namespace mesh_detail {
// Caller has already validated geometry. Read-only verification is independent
// of legalization scheduling and never copies the complete mesh.
inline bool cdt_verified(const TriangleMesh &mesh) {
  std::map<Edge, std::vector<std::size_t>> opposites;
  std::set<Edge> constrained;
  for (auto [a, b] : mesh.boundary_edges)
    constrained.insert(edge_key(a, b));
  for (auto t : mesh.elements)
    for (int e = 0; e < 3; ++e)
      opposites[edge_key(t[e], t[(e + 1) % 3])].push_back(t[(e + 2) % 3]);
  for (const auto &[edge, own] : opposites) {
    if (own.size() != 2 || constrained.contains(edge))
      continue;
    auto [a, b] = edge;
    auto c = own[0], d = own[1];
    if (predicates::orient(mesh.nodes[c], mesh.nodes[d], mesh.nodes[a]) *
            predicates::orient(mesh.nodes[c], mesh.nodes[d], mesh.nodes[b]) >=
        0)
      continue;
    if (predicates::orient(mesh.nodes[a], mesh.nodes[b], mesh.nodes[c]) < 0)
      std::swap(a, b);
    if (predicates::incircle(mesh.nodes[a], mesh.nodes[b], mesh.nodes[c],
                             mesh.nodes[d]) > 0)
      return false;
  }
  return true;
}
} // namespace mesh_detail
inline bool is_constrained_delaunay(const TriangleMesh &mesh) {
  validate_mesh_geometry(mesh);
  return mesh_detail::cdt_verified(mesh);
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
                              options.preserve_regions,
                              options.boundary.preserve_corners);
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
} // namespace pde2d::discretization
