#pragma once

#include "pde2d/mesh/mesh.hpp"

#include <filesystem>
#include <istream>

// Gmsh mesh import.
namespace pde2d::io {

// Reads a Gmsh .msh file, ASCII format version 4.1 or 2.2, into a Mesh.
//
// - Elements: 3-node triangles (type 2) make the mesh. 2-node lines
//   (type 1) give boundary regions; points (type 15) are ignored. Any
//   other element (quadrangles, second-order elements, 3D elements) is an
//   error.
// - Regions: each boundary edge of the triangulation (an edge of one
//   triangle only) must be covered by a line element of exactly one
//   physical group of dimension 1; that group's name from
//   $PhysicalNames is the region name ("physical_<tag>" for a group
//   without a name). Lines of no physical group, and lines inside the
//   domain, are ignored. Region names are numbered in increasing order of
//   their physical tag.
// - Nodes must lie in the plane z = 0. Nodes used by no triangle are
//   dropped; the others keep their file order. Triangles are made
//   counterclockwise and boundary edges are oriented with the domain on
//   their left. The mesh is validated like a WorkingMesh; it has no chains
//   or links, and its boundary edges are in walking order.
// - Binary files, partitioned meshes and other versions are rejected.
//
// Throws std::invalid_argument for a malformed, unsupported or invalid
// file, std::runtime_error if the file cannot be read.
mesh::Mesh import_gmsh(std::istream &in);
mesh::Mesh import_gmsh(const std::filesystem::path &path);

} // namespace pde2d::io
