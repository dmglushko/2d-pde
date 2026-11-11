#pragma once

#include "pde2d/mesh/working_mesh.hpp"

#include <cstddef>
#include <span>

// Shared Delaunay routines, plain functions that several components call.
// Boundary edges are never flipped, so the result is the constrained
// Delaunay triangulation of the boundary.
namespace pde2d::mesh {

// True if edge `edge` of `triangle` is a boundary edge, or if the vertex
// across it is not strictly inside the triangle's circumcircle.
bool is_locally_delaunay(const WorkingMesh &mesh, std::size_t triangle,
                         int edge);

// Lawson's algorithm on the whole mesh: flips edges that are not locally
// Delaunay until none is left. Returns the number of flips.
std::size_t flip_to_delaunay(WorkingMesh &mesh);

// The same, starting from the edges of the given triangles and spreading
// only as far as flips make necessary. Returns the number of flips.
std::size_t flip_to_delaunay_around(WorkingMesh &mesh,
                                    std::span<const std::size_t> triangles);

} // namespace pde2d::mesh
