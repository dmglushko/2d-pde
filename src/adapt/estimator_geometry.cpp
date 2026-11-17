#include "estimator_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <map>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace pde2d::adapt::detail {

using geometry::Point;

std::vector<TriangleData> triangle_data(const mesh::Mesh &mesh,
                                        std::span<const double> values,
                                        const std::string &who) {
  std::vector<TriangleData> out(mesh.triangles.size());
  for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
    const auto &v = mesh.triangles[t].nodes;
    TriangleData &d = out[t];
    for (std::size_t i = 0; i < 3; ++i)
      d.nodes[i] = mesh.nodes[v[i]];
    const auto &[p0, p1, p2] = d.nodes;
    const double det =
        (p1.x - p0.x) * (p2.y - p0.y) - (p2.x - p0.x) * (p1.y - p0.y);
    d.area = det / 2.0;
    // grad u_h = sum of u_i grad lambda_i, grad lambda_i = (pj.y - pk.y,
    // pk.x - pj.x) / det for (i, j, k) counterclockwise.
    for (std::size_t i = 0; i < 3; ++i) {
      const Point &pj = d.nodes[(i + 1) % 3], &pk = d.nodes[(i + 2) % 3];
      d.gradient.x += values[v[i]] * (pj.y - pk.y) / det;
      d.gradient.y += values[v[i]] * (pk.x - pj.x) / det;
      d.diameter = std::max(d.diameter, std::hypot(pk.x - pj.x, pk.y - pj.y));
    }
    if (!(std::isfinite(det) && det > 0.0 && std::isfinite(d.gradient.x) &&
          std::isfinite(d.gradient.y)))
      throw std::invalid_argument(std::format(
          "{}: triangle {} is too small or too large for double arithmetic: "
          "its area or the gradient of u_h is not a finite number",
          who, t));
  }
  return out;
}

EdgeTopology edge_topology(const mesh::Mesh &mesh) {
  // Every triangle edge as (smaller node, larger node, triangle, the
  // edge's first node in counterclockwise order), sorted: the two sides
  // of an interior edge are then next to each other, in triangle order.
  std::vector<std::tuple<std::size_t, std::size_t, std::size_t, std::size_t>>
      edges;
  edges.reserve(3 * mesh.triangles.size());
  for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
    const auto &v = mesh.triangles[t].nodes;
    for (std::size_t k = 0; k < 3; ++k) {
      const std::size_t a = v[(k + 1) % 3], b = v[(k + 2) % 3];
      edges.emplace_back(std::min(a, b), std::max(a, b), t, a);
    }
  }
  std::sort(edges.begin(), edges.end());

  EdgeTopology topology;
  std::map<std::pair<std::size_t, std::size_t>, std::size_t> single;
  for (std::size_t i = 0; i < edges.size();) {
    const auto &[lo, hi, t, a] = edges[i];
    if (i + 1 < edges.size() && std::get<0>(edges[i + 1]) == lo &&
        std::get<1>(edges[i + 1]) == hi) {
      const std::size_t other = std::get<2>(edges[i + 1]);
      topology.interior.push_back(
          {.a = a, .b = a == lo ? hi : lo, .first = t, .second = other});
      i += 2;
    } else {
      single.emplace(std::pair{lo, hi}, t);
      i += 1;
    }
  }
  topology.boundary_triangle.reserve(mesh.boundary_edges.size());
  for (const auto &edge : mesh.boundary_edges) {
    const auto [a, b] = edge.nodes;
    // A valid mesh (checked by MeshedProblem) has every boundary edge
    // used by exactly one triangle.
    topology.boundary_triangle.push_back(
        single.at({std::min(a, b), std::max(a, b)}));
  }
  return topology;
}

double coefficient_value(const fem::CompiledExpression &coefficient,
                         const std::string &who, const char *label,
                         const Point &p, double t) {
  if (coefficient.facts().zero)
    return 0.0;
  const double v = coefficient(p.x, p.y, t);
  if (!std::isfinite(v))
    throw std::invalid_argument(
        std::format("{}: {} is not finite at ({}, {}), t = {}: {}", who, label,
                    p.x, p.y, t, v));
  return v;
}

} // namespace pde2d::adapt::detail
