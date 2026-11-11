#include "pde2d/mesh/mesh.hpp"
#include "pde2d/mesh/mesh_spec.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace pde2d::mesh {

void validate(const MeshSpec &spec) {
  if (!(spec.max_edge_length > 0) || !std::isfinite(spec.max_edge_length))
    throw std::invalid_argument(
        "MeshSpec::max_edge_length must be finite and positive, got " +
        std::to_string(spec.max_edge_length));
}

} // namespace pde2d::mesh
