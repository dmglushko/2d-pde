#pragma once

#include "pde2d/mesh/mesh.hpp"

#include <filesystem>
#include <istream>
#include <ostream>

// The PDE2D_MESH text format of the previous code, kept exactly so that
// the mesh viewer (tools/mesh_viewer) reads the files unchanged. The
// format is described in docs/formats/pde2d-mesh.md.
namespace pde2d::io {

// Writes a mesh with one boundary loop as version 2, and a mesh with
// several loops (holes, islands) as version 5 without a result trailer
// ("RESULT 0"). Boundary edges are written loop by loop, the loop through
// the lexicographically smallest boundary node first, the others in the
// order of their lowest edge index; each loop keeps its edges' relative
// order. Each edge carries its region name as its tag. Reals have 17
// significant digits.
//
// The format has no place for curve links: chains and links are dropped,
// so a mesh read back is split at chord midpoints when refined.
//
// Throws std::invalid_argument for an invalid mesh (checked like a
// WorkingMesh) or one whose coordinate range overflows, and
// std::runtime_error if the output fails.
void export_pde2d(const mesh::Mesh &mesh, std::ostream &out);
void export_pde2d(const mesh::Mesh &mesh, const std::filesystem::path &path);

// Reads versions 2 to 5. Every boundary edge must have a region tag
// (version 1 files and the "no tag" token "-" are rejected, because a Mesh
// needs a region per edge). A version 3 or 4 result trailer, or version 5
// with "RESULT 1", is checked for its structure and then ignored: the
// meshing settings, report and boundary context of the old code have no
// place in Mesh. Region names are numbered in order of first appearance.
// The mesh is validated like a WorkingMesh, and versions 2 to 4 must have
// one boundary loop; version 5 groups must each be one closed loop. The
// result has no chains or links, and its boundary edges are in walking
// order (as WorkingMesh::to_mesh lists them).
//
// Throws std::invalid_argument for a malformed or invalid file,
// std::runtime_error if the file cannot be read.
mesh::Mesh import_pde2d(std::istream &in);
mesh::Mesh import_pde2d(const std::filesystem::path &path);

} // namespace pde2d::io
