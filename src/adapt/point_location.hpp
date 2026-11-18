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

// The boundary loop of every boundary edge of `mesh` (indexed like
// boundary_edges): the connected pieces of the boundary, which in a valid
// mesh are the closed loops (they do not touch), numbered 0, 1, ... in the
// order of their first edge.
std::vector<std::size_t> boundary_loops(const mesh::Mesh &mesh);

// The nearest point of a boundary: its weights, the boundary edge it is
// on, and the squared distance to it.
struct BoundaryPoint {
  NodeWeights weights{};
  std::size_t edge = 0;
  double distance2 = 0.0;
};

class PointLocator {
public:
  // The mesh must be valid (mesh::validate), have a boundary edge, and
  // outlive the locator.
  explicit PointLocator(const mesh::Mesh &mesh);

  // The weights of p in the triangle of lowest index that contains it
  // (closed: on an edge or a node counts), or nothing when no triangle
  // does. A point equal to a node gets weight 1 there; on an edge, the
  // two weights of the edge.
  std::optional<NodeWeights> in_triangle(geometry::Point p) const;

  // The point of the mesh boundary nearest to p (on the boundary edge of
  // lowest index among the nearest, clamped to its ends; an end gets
  // weight 1 alone). With `loop`, only the edges of that boundary loop
  // (boundary_loops; it must exist) are considered.
  BoundaryPoint
  nearest_on_boundary(geometry::Point p,
                      std::optional<std::size_t> loop = std::nullopt) const;

  // The boundary loop of each boundary edge (boundary_loops) and their
  // count.
  const std::vector<std::size_t> &edge_loops() const { return edge_loops_; }
  std::size_t loops() const { return loops_; }

private:
  std::array<std::size_t, 2> cell_of(geometry::Point p) const;
  std::size_t index(std::size_t i, std::size_t j) const { return j * nx_ + i; }

  const mesh::Mesh &mesh_;
  geometry::BoundingBox box_{};
  std::size_t nx_ = 1, ny_ = 1;
  double cell_width_ = 1.0, cell_height_ = 1.0;
  std::vector<std::vector<std::size_t>> triangles_; // per cell
  std::vector<std::vector<std::size_t>> edges_;     // per cell
  std::vector<std::size_t> edge_loops_;             // per boundary edge
  std::size_t loops_ = 0;
};

} // namespace pde2d::adapt::detail
