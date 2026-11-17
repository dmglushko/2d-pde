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
#include <tuple>

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

void set_longest_edges_as_refinement_edges(Mesh &mesh) {
  for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
    auto &v = mesh.triangles[t].nodes;
    for (auto i : v)
      if (i >= mesh.nodes.size())
        throw std::invalid_argument(
            "set_longest_edges_as_refinement_edges: triangle " +
            std::to_string(t) + " uses a node index out of range");
    // The edge opposite vertex i, ordered by squared length, then by its
    // node indices: one order for all edges, which both triangles of an
    // edge agree on (the bisection refiners use the same).
    auto opposite_key = [&](std::size_t i) {
      const std::size_t a = v[(i + 1) % 3], b = v[(i + 2) % 3];
      geometry::Point d = mesh.nodes[a] - mesh.nodes[b];
      return std::tuple{geometry::dot(d, d), std::min(a, b), std::max(a, b)};
    };
    std::size_t first = 0;
    for (std::size_t i = 1; i < 3; ++i)
      if (opposite_key(i) > opposite_key(first))
        first = i;
    // A rotation keeps the triangle counterclockwise.
    v = {v[first], v[(first + 1) % 3], v[(first + 2) % 3]};
  }
}

} // namespace pde2d::mesh
