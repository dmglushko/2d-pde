#include "views.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <vector>

namespace pde2d::app {

Json report_json(const mesh::StageReport &report) {
  Json out = Json::object();
  out["stage"] = report.stage;
  out["time_ms"] = report.time.count() * 1000.0;
  out["messages"] = report.messages;
  Json metrics = Json::object();
  for (const auto &[name, value] : report.metrics)
    metrics[name] = std::isfinite(value) ? Json(value) : Json(nullptr);
  out["metrics"] = std::move(metrics);
  return out;
}

Json reports_json(std::span<const mesh::StageReport> reports) {
  Json out = Json::array();
  for (const mesh::StageReport &report : reports)
    out.push_back(report_json(report));
  return out;
}

Json mesh_json(const mesh::Mesh &mesh) {
  std::vector<double> nodes;
  nodes.reserve(2 * mesh.nodes.size());
  for (const geometry::Point &p : mesh.nodes) {
    nodes.push_back(p.x);
    nodes.push_back(p.y);
  }
  std::vector<std::size_t> triangles;
  triangles.reserve(3 * mesh.triangles.size());
  for (const mesh::Triangle &triangle : mesh.triangles)
    triangles.insert(triangles.end(), triangle.nodes.begin(),
                     triangle.nodes.end());
  std::vector<std::size_t> edges;
  edges.reserve(3 * mesh.boundary_edges.size());
  bool has_links = false;
  for (const mesh::BoundaryEdge &edge : mesh.boundary_edges) {
    edges.insert(edges.end(), {edge.nodes[0], edge.nodes[1], edge.region});
    has_links = has_links || edge.link.has_value();
  }
  Json out = Json::object();
  out["nodes"] = std::move(nodes);
  out["triangles"] = std::move(triangles);
  out["boundary_edges"] = std::move(edges);
  out["region_names"] = mesh.region_names;
  out["has_links"] = has_links;
  return out;
}

Json mesh_stats_json(const mesh::Mesh &mesh) {
  double area = 0.0;
  double shortest = std::numeric_limits<double>::infinity();
  double longest = 0.0;
  double min_angle = 180.0;
  double angle_sum = 0.0;
  for (const mesh::Triangle &triangle : mesh.triangles) {
    const std::array<geometry::Point, 3> p = {mesh.nodes[triangle.nodes[0]],
                                              mesh.nodes[triangle.nodes[1]],
                                              mesh.nodes[triangle.nodes[2]]};
    area += 0.5 * geometry::cross(p[1] - p[0], p[2] - p[0]);
    double smallest = 180.0;
    for (int i = 0; i < 3; ++i) {
      const geometry::Point a = p[(i + 1) % 3] - p[i];
      const geometry::Point b = p[(i + 2) % 3] - p[i];
      const double length = geometry::norm(a);
      shortest = std::min(shortest, length);
      longest = std::max(longest, length);
      const double angle =
          std::atan2(std::abs(geometry::cross(a, b)), geometry::dot(a, b)) *
          180.0 / std::numbers::pi;
      smallest = std::min(smallest, angle);
    }
    min_angle = std::min(min_angle, smallest);
    angle_sum += smallest;
  }
  const std::size_t count = mesh.triangles.size();
  Json out = Json::object();
  out["nodes"] = mesh.nodes.size();
  out["triangles"] = count;
  out["boundary_edges"] = mesh.boundary_edges.size();
  out["area"] = area;
  out["shortest_edge"] = count > 0 ? Json(shortest) : Json(nullptr);
  out["longest_edge"] = count > 0 ? Json(longest) : Json(nullptr);
  out["min_angle_degrees"] = count > 0 ? Json(min_angle) : Json(nullptr);
  out["mean_min_angle_degrees"] =
      count > 0 ? Json(angle_sum / static_cast<double>(count)) : Json(nullptr);
  return out;
}

Json sparsity_json(const fem::SparsityPattern &pattern, std::size_t max_bins) {
  const std::size_t size = std::max(pattern.rows, pattern.columns);
  const std::size_t bins = std::max<std::size_t>(1, std::min(size, max_bins));
  std::vector<std::size_t> counts(bins * bins, 0);
  const auto bin = [&](std::size_t index) {
    return size == 0 ? 0 : std::min(bins - 1, index * bins / size);
  };
  for (std::size_t row = 0; row < pattern.rows; ++row)
    for (auto slot = pattern.row_offsets[row];
         slot < pattern.row_offsets[row + 1]; ++slot)
      ++counts[bin(row) * bins +
               bin(static_cast<std::size_t>(
                   pattern.column_indices[static_cast<std::size_t>(slot)]))];
  Json out = Json::object();
  out["rows"] = pattern.rows;
  out["columns"] = pattern.columns;
  out["nonzeros"] = pattern.nonzeros();
  out["bins"] = bins;
  out["counts"] = std::move(counts);
  return out;
}

} // namespace pde2d::app
