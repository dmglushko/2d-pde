#pragma once

// Private to the mesh library.

#include "pde2d/mesh/working_mesh.hpp"

#include <cstddef>
#include <vector>

namespace pde2d::mesh::detail {

// Makes the segment between nodes a and b an edge of the mesh by flipping
// the edges that cross it (Sloan's queue: an edge that cannot be flipped
// yet is retried later). Returns the number of flips; every triangle a
// flip changes is appended to `touched` (if not null). Throws
// std::invalid_argument if a node lies on the open segment, and
// std::logic_error if the segment crosses a boundary edge or recovery
// does not finish.
std::size_t recover_edge(WorkingMesh &mesh, std::size_t a, std::size_t b,
                         std::vector<std::size_t> *touched = nullptr);

} // namespace pde2d::mesh::detail
