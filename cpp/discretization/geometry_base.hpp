#pragma once

#include <array>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace pde2d::discretization {

namespace boundary_detail {
// Empty tags represent untagged mesh edges; '-' is the file-format sentinel.
inline bool valid_tag(const std::string &name) {
  return name != "-" &&
         std::all_of(name.begin(), name.end(), [](unsigned char c) {
           return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                  (c >= '0' && c <= '9') || c == '_' || c == '.' || c == '-';
         });
}
} // namespace boundary_detail

struct Point2 {
  double x = 0.0, y = 0.0;
};

// One simple closed loop represented without a repeated last point.
struct BoundaryLoop {
  std::vector<Point2> points;
  // Optional tag for each edge from points[i] to points[(i+1)%size].
  std::vector<std::string> edge_tags{};
};

// A polygonal domain: one outer loop (points, edge_tags) and zero or more
// inner loops (holes). Every loop is simple and may run in either direction;
// holes lie strictly inside the outer loop and strictly outside each other
// (validate_polygon checks this). Without holes this is the historical
// single-outline boundary.
struct Boundary {
  // The outer loop, without a repeated last point.
  std::vector<Point2> points;
  // Optional tag for each edge from points[i] to points[(i+1)%size].
  std::vector<std::string> edge_tags{};
  std::vector<BoundaryLoop> holes{};
};

struct TriangleMesh {
  std::vector<Point2> nodes;
  // Every P1 triangle contains exactly three vertex indices.
  std::vector<std::array<std::size_t, 3>> elements;
  // Vertex endpoints of exterior edges, of every boundary loop (outer loop
  // and holes; see boundary_loops() in mesh_validation.hpp).
  std::vector<std::pair<std::size_t, std::size_t>> boundary_edges;
  // Empty means all edges are untagged; otherwise parallel to boundary_edges.
  std::vector<std::string> boundary_tags{};
};

class Geometry2D {
public:
  virtual ~Geometry2D() = default;
  void setCenter(double x, double y) { setCenter(Point2{x, y}); }
  void setCenter(Point2 c) {
    if (!std::isfinite(c.x) || !std::isfinite(c.y))
      throw std::invalid_argument("Center must be finite");
    center_ = c;
  }
  Point2 center() const { return center_; }
  // Rotation in radians, counterclockwise about the geometry center.
  void setRotation(double radians) {
    if (!std::isfinite(radians))
      throw std::invalid_argument("Rotation must be finite");
    rotation_ = radians;
  }
  double rotation() const { return rotation_; }

private:
  Point2 center_{};
  double rotation_ = 0.0;
};

} // namespace pde2d::discretization
