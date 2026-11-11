#pragma once

// Private to the mesh library.

#include "pde2d/mesh/working_mesh.hpp"

#include <cstddef>
#include <span>
#include <vector>

namespace pde2d::mesh::detail {

// Lawson flips starting from the edges of `seeds`. Every triangle that is
// a seed or is changed by a flip is appended to `touched` (if not null).
std::size_t lawson(WorkingMesh &mesh, std::span<const std::size_t> seeds,
                   std::vector<std::size_t> *touched);

} // namespace pde2d::mesh::detail
