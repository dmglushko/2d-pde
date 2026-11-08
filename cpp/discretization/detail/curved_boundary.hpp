#pragma once
// Private (mesh_detail) part of triangulate.hpp: boundary refinement on the
// analytical source curves. Splitting a boundary record bisects its source
// intervals by arc length; rebuild_curved() re-samples the polygon on the
// curves and retriangulates, reinserting interior vertices that remain
// strictly inside.
#include "../boundary_discretization.hpp"
#include "../curve_length.hpp"
#include "../geometry_base.hpp"
#include "../mesh_types.hpp"
#include "../predicates.hpp"
#include "cdt_topology.hpp"
#include "ear_clipping.hpp"
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace pde2d::discretization::mesh_detail {
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
} // namespace pde2d::discretization::mesh_detail
