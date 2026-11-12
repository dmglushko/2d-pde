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

} // namespace pde2d::mesh
