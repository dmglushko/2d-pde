#include "pde2d/mesh/mesh.hpp"
#include "pde2d/mesh/mesh_spec.hpp"
#include "pde2d/mesh/working_mesh.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>

namespace pde2d::mesh {

double resolve(const LengthBound &bound, double domain_size) {
  double limit = std::numeric_limits<double>::infinity();
  if (bound.absolute)
    limit = std::min(limit, *bound.absolute);
  if (bound.relative)
    limit = std::min(limit, *bound.relative * domain_size);
  return limit;
}

void validate(const LengthBound &bound, std::string_view name) {
  auto check = [&](const std::optional<double> &value, const char *which) {
    if (value && !(*value > 0 && std::isfinite(*value)))
      throw std::invalid_argument(std::string(name) + "." + which +
                                  " must be finite and positive, got " +
                                  std::to_string(*value));
  };
  check(bound.absolute, "absolute");
  check(bound.relative, "relative");
}

void validate(const MeshSpec &spec) {
  if (!spec.max_edge_length.absolute && !spec.max_edge_length.relative)
    throw std::invalid_argument(
        "MeshSpec::max_edge_length needs an absolute or a relative value");
  validate(spec.max_edge_length, "MeshSpec::max_edge_length");
}

void validate(const Mesh &mesh) { (void)WorkingMesh::from_mesh(mesh); }

} // namespace pde2d::mesh
