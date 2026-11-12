#pragma once

#include "pde2d/geometry/curve_geometry.hpp"
#include "pde2d/mesh/mesh.hpp"
#include "pde2d/mesh/working_mesh.hpp"

namespace pde2d::mesh {

// The size of a domain: the length that LengthBound::relative multiplies.
// The library uses one fixed definition, currently the diagonal of the
// bounding box of the domain's boundary (a private policy, so a different
// definition can replace it in one place).
//
// The size is measured on the exact curves where they are known, so every
// stage of one run sees the same number: the discretizer measures the
// geometry, and a mesh made from it carries the same curves in its chains.

// Measured on every curve of the geometry.
double domain_size(const geometry::CurveGeometry &geometry);
// Measured on the curves of the mesh's chains if it has any (it came from
// the Mesher or Remesher), else on the nodes of its boundary edges (an
// imported mesh: its boundary is its geometry, and splitting a boundary edge
// at its midpoint does not change the measure).
double domain_size(const WorkingMesh &mesh);
double domain_size(const Mesh &mesh);

} // namespace pde2d::mesh
