#include "views.hpp"

#include "binary.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string_view>
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

namespace {

// The largest index a uint32 holds; a larger mesh cannot travel this way.
std::uint32_t index32(std::size_t index) {
  if (index > std::numeric_limits<std::uint32_t>::max())
    throw std::runtime_error("the mesh is too large for 32-bit indices");
  return static_cast<std::uint32_t>(index);
}

} // namespace

Json mesh_summary_json(const mesh::Mesh &mesh, std::uint64_t id) {
  bool has_links = false;
  for (const mesh::BoundaryEdge &edge : mesh.boundary_edges)
    has_links = has_links || edge.link.has_value();
  const std::size_t n = mesh.nodes.size();
  const std::size_t t = mesh.triangles.size();
  const std::size_t e = mesh.boundary_edges.size();
  Json parts = Json::array();
  std::size_t offset = 0;
  const auto part = [&](std::string_view name, std::string_view type,
                        std::size_t count, std::size_t size) {
    Json item = Json::object();
    item["name"] = name;
    item["type"] = type;
    item["count"] = count;
    item["offset"] = offset;
    parts.push_back(std::move(item));
    offset += count * size;
  };
  part("nodes", "float64", 2 * n, 8);
  part("triangles", "uint32", 3 * t, 4);
  part("boundary_edges", "uint32", 2 * e, 4);
  part("boundary_regions", "uint32", e, 4);
  Json arrays = Json::object();
  arrays["path"] = "/api/mesh/arrays";
  arrays["format"] = "little-endian";
  arrays["bytes"] = offset;
  arrays["parts"] = std::move(parts);

  Json out = Json::object();
  out["id"] = id;
  out["nodes"] = n;
  out["triangles"] = t;
  out["boundary_edges"] = e;
  out["region_names"] = mesh.region_names;
  out["has_links"] = has_links;
  out["arrays"] = std::move(arrays);
  return out;
}

std::string mesh_arrays_binary(const mesh::Mesh &mesh) {
  std::string out;
  out.reserve(16 * mesh.nodes.size() + 12 * mesh.triangles.size() +
              12 * mesh.boundary_edges.size());
  for (const geometry::Point &p : mesh.nodes) {
    append_float64(out, p.x);
    append_float64(out, p.y);
  }
  for (const mesh::Triangle &triangle : mesh.triangles)
    for (const std::size_t node : triangle.nodes)
      append_uint32(out, index32(node));
  for (const mesh::BoundaryEdge &edge : mesh.boundary_edges)
    for (const std::size_t node : edge.nodes)
      append_uint32(out, index32(node));
  for (const mesh::BoundaryEdge &edge : mesh.boundary_edges)
    append_uint32(out, index32(edge.region));
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
