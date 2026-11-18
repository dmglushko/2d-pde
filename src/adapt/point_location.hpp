#pragma once

// Point location in a mesh for the solution transfer (private): which
// triangle contains a point, and which point of the boundary is nearest.
// A uniform grid of buckets over the mesh's bounding box holds, per cell,
// the triangles and the boundary edges whose bounding boxes meet the cell,
// in index order. Containment is decided with the exact orient predicate,
// so there is no walk (WorkingMesh::locate can cycle on some valid meshes,
// GitHub issue #22, and stops at boundary edges of a non-convex domain),
// and the answers do not depend on a starting triangle.

#include "pde2d/geometry/bounding_box.hpp"
#include "pde2d/geometry/point.hpp"
#include "pde2d/mesh/mesh.hpp"

#include <array>
#include <cstddef>
#include <optional>
#include <vector>

namespace pde2d::adapt::detail {

// A point of a mesh as weights of up to three of its nodes (unused slots
// have weight 0 and node 0): the P1 function's value there is
// sum weight_i u[node_i].
struct NodeWeights {
  std::array<std::size_t, 3> nodes{};
  std::array<double, 3> weights{};
};

class PointLocator {
public:
  // The mesh must be valid (mesh::validate) and outlive the locator.
  explicit PointLocator(const mesh::Mesh &mesh);

  // The weights of p in the triangle of lowest index that contains it
  // (closed: on an edge or a node counts), or nothing when no triangle
  // does. A point equal to a node gets weight 1 there; on an edge, the
  // two weights of the edge.
  std::optional<NodeWeights> in_triangle(geometry::Point p) const;

  // The weights of the point of the mesh boundary nearest to p (on the
  // boundary edge of lowest index among the nearest, clamped to its ends;
  // an end gets weight 1 alone).
  NodeWeights nearest_on_boundary(geometry::Point p) const;

private:
  std::array<std::size_t, 2> cell_of(geometry::Point p) const;
  std::size_t index(std::size_t i, std::size_t j) const { return j * nx_ + i; }

  const mesh::Mesh &mesh_;
  geometry::BoundingBox box_{};
  std::size_t nx_ = 1, ny_ = 1;
  double cell_width_ = 1.0, cell_height_ = 1.0;
  std::vector<std::vector<std::size_t>> triangles_; // per cell
  std::vector<std::vector<std::size_t>> edges_;     // per cell
};

} // namespace pde2d::adapt::detail
