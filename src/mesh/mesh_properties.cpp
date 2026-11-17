#include "pde2d/mesh/mesh_properties.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>

namespace pde2d::mesh {

namespace {

// The angle at `at` between the directions to p and q, in degrees.
double angle_degrees(geometry::Point at, geometry::Point p, geometry::Point q) {
  geometry::Point u = p - at, v = q - at;
  return std::atan2(std::abs(geometry::cross(u, v)), geometry::dot(u, v)) *
         180.0 / std::numbers::pi;
}

} // namespace

void validate_warning_angle(double warning_angle_degrees, const char *what) {
  if (!(warning_angle_degrees >= 0 && warning_angle_degrees <= 60))
    throw std::invalid_argument(std::string(what) +
                                " must be in [0, 60] degrees, got " +
                                std::to_string(warning_angle_degrees));
}

MeshProperties measure(const Mesh &mesh, double warning_angle_degrees) {
  validate_warning_angle(warning_angle_degrees,
                         "measure: warning_angle_degrees");
  if (mesh.triangles.empty())
    throw std::invalid_argument("measure: the mesh has no triangles");
  MeshProperties p;
  p.nodes = mesh.nodes.size();
  p.triangles = mesh.triangles.size();
  p.warning_angle_degrees = warning_angle_degrees;
  p.smallest_angle_degrees = 180.0;
  p.largest_angle_degrees = 0.0;
  p.shortest_edge = std::numeric_limits<double>::infinity();
  p.longest_edge = 0.0;
  for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
    const auto &v = mesh.triangles[t].nodes;
    for (auto i : v)
      if (i >= mesh.nodes.size())
        throw std::invalid_argument("measure: triangle " + std::to_string(t) +
                                    " uses a node index out of range");
    bool below = false;
    for (std::size_t i = 0; i < 3; ++i) {
      geometry::Point a = mesh.nodes[v[i]], b = mesh.nodes[v[(i + 1) % 3]],
                      c = mesh.nodes[v[(i + 2) % 3]];
      const double angle = angle_degrees(a, b, c);
      const double edge = geometry::distance(a, b);
      p.smallest_angle_degrees = std::min(p.smallest_angle_degrees, angle);
      p.largest_angle_degrees = std::max(p.largest_angle_degrees, angle);
      p.shortest_edge = std::min(p.shortest_edge, edge);
      p.longest_edge = std::max(p.longest_edge, edge);
      below = below || angle < warning_angle_degrees;
    }
    p.triangles_below_warning_angle += below ? 1 : 0;
  }
  return p;
}

} // namespace pde2d::mesh
