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
// Inside the outer loop and outside every hole; points on a loop count as
// inside unless strict.
inline bool inside_polygon(Point2 p, const Boundary &boundary,
                           bool strict = false) {
  int outer = point_in_loop(p, boundary.points);
  if (outer <= 0)
    return outer == 0 && !strict;
  for (const auto &hole : boundary.holes) {
    int side = point_in_loop(p, hole.points);
    if (side >= 0)
      return side == 0 && !strict;
  }
  return true;
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
  // Each loop (outer first, holes in mesh order) is traversed along its edge
  // directions from its lowest-index edge.
  Boundary polygon;
  BoundaryContext next = ctx;
  next.edge_sources.clear();
  next.protected_vertices.clear();
  next.snapshot_loop_sizes.clear();
  std::set<std::size_t> fixed(ctx.protected_vertices.begin(),
                              ctx.protected_vertices.end()),
      boundary_vertices;
  const auto loops = boundary_loops(m);
  std::size_t traversed = 0, offset = 0;
  for (std::size_t k = 0; k < loops.size(); ++k) {
    const auto first = m.boundary_edges[loops[k].front()].first;
    std::vector<std::size_t> order;
    std::size_t current = first;
    do {
      auto it = outgoing.find(current);
      if (it == outgoing.end())
        throw std::invalid_argument(
            "Curve refinement needs a directed boundary loop");
      order.push_back(it->second);
      current = m.boundary_edges[it->second].second;
    } while (current != first && order.size() <= loops[k].size());
    if (order.size() != loops[k].size())
      throw std::invalid_argument("Invalid boundary traversal");
    traversed += order.size();
    BoundaryLoop loop;
    for (auto i : order) {
      auto vertex = m.boundary_edges[i].first;
      boundary_vertices.insert(vertex);
      auto s = ctx.edge_sources[i].front();
      loop.points.push_back(curvePoint(ctx.curves[s.curve].segment, s.begin));
      if (!m.boundary_tags.empty())
        loop.edge_tags.push_back(m.boundary_tags[i]);
      if (fixed.contains(vertex))
        next.protected_vertices.push_back(offset + loop.points.size() - 1);
      next.edge_sources.push_back(ctx.edge_sources[i]);
    }
    offset += loop.points.size();
    if (k == 0) {
      polygon.points = std::move(loop.points);
      polygon.edge_tags = std::move(loop.edge_tags);
    } else {
      if (k == 1)
        next.snapshot_loop_sizes.push_back(polygon.points.size());
      next.snapshot_loop_sizes.push_back(loop.points.size());
      polygon.holes.push_back(std::move(loop));
    }
  }
  if (traversed != m.boundary_edges.size())
    throw std::invalid_argument("Invalid boundary traversal");
  TriangleMesh replacement = seed_mesh(polygon);
  next.polygon_snapshot = boundary_points(polygon);
  Topology topology(replacement);
  auto &work = result.record.work;
  topology.work = &work;
  ++work.curved_rebuilds;
  std::size_t removed = 0;
  for (std::size_t i = 0; i < m.nodes.size(); ++i)
    if (!boundary_vertices.contains(i)) {
      auto p = m.nodes[i];
      if (inside_polygon(p, polygon, true)) {
        topology.insert(p);
        ++work.reinserted_vertices;
      } else
        ++removed;
    }
  work.removed_vertices += removed;
  topology.legalize();
  m = std::move(replacement);
  ctx = std::move(next);
  if (removed)
    result.report.message =
        "Curve update removed " + std::to_string(removed) +
        " interior vertices outside or on the updated boundary";
}
} // namespace pde2d::discretization::mesh_detail
