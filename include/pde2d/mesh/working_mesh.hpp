#pragma once

#include "pde2d/geometry/curve.hpp"
#include "pde2d/geometry/point.hpp"
#include "pde2d/mesh/boundary_polygon.hpp"
#include "pde2d/mesh/mesh.hpp"

#include <array>
#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace pde2d::mesh {

// "No such triangle / edge / node".
inline constexpr std::size_t no_index = std::numeric_limits<std::size_t>::max();

// A boundary edge of the working mesh, oriented with the domain on its
// left. The optional link says which exact curve it approximates.
struct WorkingBoundaryEdge {
  std::array<std::size_t, 2> nodes{};
  std::size_t region = 0;
  std::optional<CurveLink> link;
};

// Result of WorkingMesh::locate.
struct Location {
  enum class Kind {
    inside,  // strictly inside `triangle`
    on_edge, // on edge `index` of `triangle`
    on_node, // equal to vertex `index` of `triangle`
    blocked  // the walk had to cross boundary edge `index` of `triangle`
  };
  Kind kind = Kind::inside;
  std::size_t triangle = no_index;
  int index = -1;
};

// The mutable mesh that the meshing stages work on.
//
// Triangles are counterclockwise. Edge i of a triangle is the edge
// opposite its vertex i, from vertex (i + 1) % 3 to vertex (i + 2) % 3, so
// the triangle is on the left of it. neighbour(t, i) is the triangle across
// that edge, or no_index on the boundary. Every edge without a neighbour is
// exactly one boundary edge with the same direction.
//
// Edits keep these invariants. They never delete a triangle or a node:
// a split reuses the old triangle's index and appends new ones, a flip
// reuses both indices. Each edit returns the triangles it changed or
// created. An edit that would create an inverted triangle throws and
// leaves the mesh unchanged.
class WorkingMesh {
public:
  using TriangleNodes = std::array<std::size_t, 3>;

  // Validates everything above (orientation, conformity, boundary edges
  // match the triangulation, no unused node, boundary loops do not touch,
  // indices in range) and throws std::invalid_argument naming the problem.
  WorkingMesh(std::vector<geometry::Point> nodes,
              std::vector<TriangleNodes> triangles,
              std::vector<WorkingBoundaryEdge> boundary_edges,
              std::vector<std::string> region_names,
              std::vector<geometry::Curve> curves = {});

  // A working mesh without curve links.
  static WorkingMesh from_mesh(const Mesh &mesh);

  // --- reading -------------------------------------------------------------
  const std::vector<geometry::Point> &nodes() const { return nodes_; }
  const std::vector<TriangleNodes> &triangles() const { return triangles_; }
  const std::vector<WorkingBoundaryEdge> &boundary_edges() const {
    return boundary_edges_;
  }
  const std::vector<std::string> &region_names() const { return region_names_; }
  const std::vector<geometry::Curve> &curves() const { return curves_; }

  std::size_t neighbour(std::size_t triangle, int edge) const {
    return neighbours_[triangle][static_cast<std::size_t>(edge)];
  }
  // The boundary edge index of a triangle edge, or no_index if interior.
  std::size_t boundary_edge_index(std::size_t triangle, int edge) const;
  bool is_boundary_node(std::size_t node) const {
    return outgoing_boundary_[node] != no_index;
  }
  // The triangle and local edge of boundary edge `k`.
  std::pair<std::size_t, int> boundary_edge_location(std::size_t k) const;
  // The triangle that has the directed edge from node a to node b, and the
  // local index of that edge; {no_index, -1} if there is none.
  std::pair<std::size_t, int> find_edge(std::size_t a, std::size_t b) const;
  // All triangles that contain the node.
  std::vector<std::size_t> triangles_around(std::size_t node) const;
  // Walks from `start` towards p (a visibility walk). Stops at the triangle
  // containing p, or at the first boundary edge it would have to cross.
  Location locate(geometry::Point p, std::size_t start) const;

  // --- editing -------------------------------------------------------------
  // True if edge `edge` of `triangle` is interior and its two triangles
  // form a strictly convex quadrilateral.
  bool is_flippable(std::size_t triangle, int edge) const;
  // Replaces the edge by the other diagonal of its quadrilateral.
  std::array<std::size_t, 2> flip(std::size_t triangle, int edge);
  // Adds node p strictly inside the triangle; three triangles result.
  std::array<std::size_t, 3> insert_in_triangle(std::size_t triangle,
                                                geometry::Point p);
  // Adds node p on interior edge `edge` of `triangle`; four triangles
  // result.
  std::array<std::size_t, 4> split_interior_edge(std::size_t triangle, int edge,
                                                 geometry::Point p);
  // The point where boundary edge k is split: the curve point at the middle
  // parameter if the edge has a link to a curved (non-Line) curve, else the
  // chord midpoint.
  geometry::Point boundary_split_point(std::size_t k) const;
  // Splits boundary edge k at boundary_split_point(k). The two halves keep
  // the region; their links cover the two halves of the parameter
  // interval. Throws std::runtime_error if the curve point would invert a
  // triangle (the chord is too coarse for the curve there).
  std::array<std::size_t, 2> split_boundary_edge(std::size_t k);
  // Moves an interior node if every triangle around it stays
  // counterclockwise; returns false (and changes nothing) otherwise.
  // Boundary nodes cannot be moved (std::invalid_argument).
  bool try_move_node(std::size_t node, geometry::Point p);

  // The output mesh. Boundary edges are listed loop by loop in walking
  // order; loops in order of their lowest boundary edge index.
  Mesh to_mesh() const;

private:
  void set_node_triangles(std::size_t t);
  void replace_neighbour(std::size_t triangle, std::size_t old_neighbour,
                         std::size_t new_neighbour);
  int local_index(std::size_t triangle, std::size_t node) const;

  std::vector<geometry::Point> nodes_;
  std::vector<TriangleNodes> triangles_;
  std::vector<std::array<std::size_t, 3>> neighbours_;
  std::vector<std::size_t> node_triangle_; // one triangle containing node
  std::vector<WorkingBoundaryEdge> boundary_edges_;
  std::vector<std::size_t> outgoing_boundary_; // per node, or no_index
  std::vector<std::string> region_names_;
  std::vector<geometry::Curve> curves_;
};

} // namespace pde2d::mesh
