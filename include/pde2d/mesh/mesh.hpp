#pragma once

#include "pde2d/geometry/point.hpp"
#include "pde2d/mesh/curve_chain.hpp"

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace pde2d::mesh {

// A triangle by its three node indices, counterclockwise.
//
// THE REFINEMENT EDGE CONVENTION. Every triangle has a refinement edge:
// the edge opposite its vertex 0 (nodes[0], the "newest vertex"). The
// bisection refiners (newest_vertex_bisection.hpp and
// red_green_blue_refinement.hpp) bisect a triangle there and write the
// convention into the triangles they make (newest vertex first), so it
// persists from one refinement to the next with no extra field. The
// vertex order stays counterclockwise; only which vertex comes first
// carries the information. Code that copies a mesh keeps every vertex
// order (to_mesh/from_mesh, the .pde2d round trip and refinement do).
//
// The library delivers the order: the Mesher and Remesher::run end by
// making every triangle's longest edge its refinement edge, Gmsh import
// does it too, and Remesher::refine does it for the triangles whose order
// an edit chose (flips, Ruppert's insertions, a re-triangulating boundary
// split) while the triangles a bisection made and the untouched ones keep
// theirs. A .pde2d file keeps the order it was saved with. Only a mesh
// built or edited by hand needs set_longest_edges_as_refinement_edges()
// before newest vertex bisection or red-green-blue refinement; without
// it the mesh can still be refined, but loses their shape guarantee.
struct Triangle {
  std::array<std::size_t, 3> nodes{};
};

// An edge on the domain boundary, oriented so that the domain is on its
// left. `region` indexes Mesh::region_names. The optional link says which
// piece of a curve chain (in Mesh::chains) the edge approximates.
struct BoundaryEdge {
  std::array<std::size_t, 2> nodes{};
  std::size_t region = 0;
  std::optional<CurveLink> link;
};

// The output mesh: plain arrays, self-contained (it needs no geometry).
// Boundary edges are listed loop by loop, each loop in walking order.
// Interior edges and connected pieces are not stored; compute them when
// needed.
//
// A mesh made by the Mesher (or Remesher) also carries its curve links:
// the chains its boundary edges were sampled from, and a link per edge
// into them. A Remesher refining this mesh then puts new boundary points
// on the true curves without the geometry. An imported mesh has no chains
// and no links; its boundary edges are split at their chord midpoints.
struct Mesh {
  std::vector<geometry::Point> nodes;
  std::vector<Triangle> triangles;
  std::vector<BoundaryEdge> boundary_edges;
  std::vector<std::string> region_names;
  // The chains that BoundaryEdge::link points to; empty without links.
  std::vector<CurveChain> chains;
};

// Checks that the mesh is valid, exactly as a WorkingMesh built from it
// would (WorkingMesh::from_mesh, which the importers use): indices in
// range, finite nodes, counterclockwise triangles, conforming edges, the
// boundary edges are exactly the edges used by one triangle, every node
// used, boundary loops that do not touch, valid links; and the geometry:
// no fold around a node, no crossing or touching boundary edges, no node
// inside a triangle that does not use it (no piece inside another).
// Throws std::invalid_argument naming the problem.
void validate(const Mesh &mesh);

// Sets the refinement edge of every triangle to its longest edge: rotates
// each triangle's vertices (keeping them counterclockwise) so that the
// vertex opposite the longest edge comes first. Among equally long
// longest edges (exactly equal squared lengths) the edge with the larger
// node indices (compared as (smaller, larger) pair) is taken: one order
// of all edges that the two triangles of an edge agree on, so the
// triangles waiting for each other's refinement edges cannot form a
// cycle (the refiners use the same order). The Mesher, the Remesher and
// Gmsh import already deliver this order (see the convention above); use
// it on a mesh built or edited by hand, before the first adaptive
// refinement (Remesher::refine) by newest vertex bisection or red-green-
// blue refinement. Throws std::invalid_argument for a node index out of
// range.
void set_longest_edges_as_refinement_edges(Mesh &mesh);

} // namespace pde2d::mesh
