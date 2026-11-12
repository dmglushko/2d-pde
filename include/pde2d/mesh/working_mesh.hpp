#pragma once

#include "pde2d/geometry/point.hpp"
#include "pde2d/mesh/boundary_polygon.hpp"
#include "pde2d/mesh/curve_chain.hpp"
#include "pde2d/mesh/mesh.hpp"

#include <array>
#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace pde2d::mesh {

namespace detail {
// The steps of the library's boundary split policy (private, in
// src/mesh/boundary_split_steps.*). They edit the mesh's internals.
struct BoundarySplitSteps;
} // namespace detail

// "No such triangle / edge / node".
inline constexpr std::size_t no_index = std::numeric_limits<std::size_t>::max();

// A boundary edge of the working mesh, oriented with the domain on its
// left. The optional link says which piece of a curve chain it
// approximates.
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

// What WorkingMesh::split_boundary_edge did.
struct BoundarySplitResult {
  // False: the edge could not be split, the mesh is unchanged, `failure`
  // says why, and the edge is now marked unsplittable.
  bool split = false;
  // The simple split would have inverted a triangle, so the mesh around
  // the edge was re-triangulated.
  bool retriangulated = false;
  // Live triangles that were created or changed.
  std::vector<std::size_t> touched;
  std::size_t deleted_nodes = 0;
  std::size_t deleted_triangles = 0;
  std::string failure;
};

// The mutable mesh that the meshing stages work on.
//
// Triangles are counterclockwise. Edge i of a triangle is the edge
// opposite its vertex i, from vertex (i + 1) % 3 to vertex (i + 2) % 3, so
// the triangle is on the left of it. neighbour(t, i) is the triangle across
// that edge, or no_index on the boundary. Every edge without a neighbour is
// exactly one boundary edge with the same direction.
//
// Edits keep these invariants. A split reuses the old triangle's index and
// appends new ones, a flip reuses both indices. Splitting a curved
// boundary edge may delete triangles and nodes (see split_boundary_edge).
// A deleted triangle or node keeps its index as a tombstone
// (is_triangle_deleted, is_node_deleted), so indices held by a refiner
// stay valid; boundary edges are never deleted. compact() renumbers
// without the tombstones, and to_mesh() never contains them. Each edit
// returns the triangles it changed or created. An edit that would create
// an inverted triangle throws and leaves the mesh unchanged.
class WorkingMesh {
public:
  using TriangleNodes = std::array<std::size_t, 3>;

  // Validates everything above (orientation, conformity, boundary edges
  // match the triangulation, no unused node, boundary loops do not touch,
  // indices in range) and the geometry: no fold around a node, no
  // crossing or touching boundary edges, no node inside a triangle that
  // does not use it (so no piece lies inside another). Throws
  // std::invalid_argument naming the problem.
  WorkingMesh(std::vector<geometry::Point> nodes,
              std::vector<TriangleNodes> triangles,
              std::vector<WorkingBoundaryEdge> boundary_edges,
              std::vector<std::string> region_names,
              std::vector<CurveChain> chains = {});

  // A working mesh with the mesh's chains and curve links (none for an
  // imported mesh). Validates like the constructor.
  static WorkingMesh from_mesh(const Mesh &mesh);

  // --- reading -------------------------------------------------------------
  // Both lists may contain deleted entries until compact() is called.
  const std::vector<geometry::Point> &nodes() const { return nodes_; }
  const std::vector<TriangleNodes> &triangles() const { return triangles_; }
  bool is_triangle_deleted(std::size_t t) const {
    return triangles_[t][0] == no_index;
  }
  bool is_node_deleted(std::size_t v) const {
    return node_triangle_[v] == no_index;
  }
  bool has_deleted() const { return deleted_triangles_ > 0; }
  const std::vector<WorkingBoundaryEdge> &boundary_edges() const {
    return boundary_edges_;
  }
  const std::vector<std::string> &region_names() const { return region_names_; }
  const std::vector<CurveChain> &chains() const { return chains_; }

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
  // Walks from `start` (a live triangle) towards p (a visibility walk).
  // Stops at the triangle containing p, or at the first boundary edge it
  // would have to cross.
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
  // The point where boundary edge k is split. For a linked edge it is the
  // chain point at the arc-length middle s = (s0 + s1) / 2, except that
  // an edge lying on a single straight curve is split at its chord
  // midpoint (the same point, without the rounding of the curve). An
  // unlinked edge is split at its chord midpoint.
  geometry::Point boundary_split_point(std::size_t k) const;
  // Splits boundary edge k at boundary_split_point(k), using the
  // library's boundary split policy (fixed, internal; its steps are
  // private). The v1 policy:
  //  (a) split directly, joining the split point to the opposite vertex,
  //      if that inverts no triangle;
  //  (b) otherwise re-triangulate locally: for a point outside the mesh
  //      (the curve bulges outwards) add the triangle a, p, b on the edge;
  //      for a point inside it, insert it, force the edges a-p and p-b by
  //      flips and cut off everything between them and the old edge,
  //      including the nodes there, which lie outside the true domain;
  //  (c) if that is impossible too, change nothing, mark the edge
  //      unsplittable and say why.
  // The two halves keep the region; their links cover the two halves of
  // the arc-length interval. An edge already marked unsplittable fails at
  // once.
  BoundarySplitResult split_boundary_edge(std::size_t k);
  // True if a split of boundary edge k has failed before.
  bool is_unsplittable(std::size_t k) const { return unsplittable_.at(k); }

  // Moves an interior node if every triangle around it stays
  // counterclockwise; returns false (and changes nothing) otherwise.
  // Boundary nodes cannot be moved (std::invalid_argument).
  bool try_move_node(std::size_t node, geometry::Point p);

  // Removes deleted triangles and nodes and renumbers the rest, keeping
  // their order. Boundary edge indices do not change.
  void compact();

  // The output mesh, without deleted entries (numbered as after
  // compact()). Boundary edges are listed loop by loop in walking order;
  // loops in order of their lowest boundary edge index. The chains and the
  // edges' curve links are copied as they are.
  Mesh to_mesh() const;

private:
  friend struct detail::BoundarySplitSteps;

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
  std::vector<CurveChain> chains_;
  std::vector<bool> unsplittable_; // per boundary edge
  std::size_t deleted_triangles_ = 0;
};

} // namespace pde2d::mesh
