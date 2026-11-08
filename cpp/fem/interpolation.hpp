#pragma once
// Nodal (P1) interpolation of a symbolic field in x, y and t, and nodal
// fields given either symbolically or as values (Field).
#include "discretization/geometry_base.hpp"
#include "execution.hpp"
#include "fem/expression.hpp"
#include "fem/system.hpp"
#include <stdexcept>
#include <string>
#include <variant>

namespace pde2d::fem {
// Values of `field` at every mesh node at the given time. Boundary normals are
// not defined at nodes, so a field that uses nx or ny is rejected. Symbolic
// preparation happens on the caller; workers only read the prepared
// evaluator.
inline Vector interpolate(const discretization::TriangleMesh &mesh,
                          const Expr &field, double time = 0,
                          const pde2d::ExecutionOptions &execution = {}) {
  require_symbols(field, field_symbols, "Interpolated field");
  Evaluator evaluate_field(field);
  Vector v(mesh.nodes.size());
  pde2d::parallel_for(mesh.nodes.size(), execution, [&](std::size_t i) {
    v[i] = evaluate_field({mesh.nodes[i].x, mesh.nodes[i].y, time});
  });
  return v;
}

// A scalar field on the mesh nodes: an expression in x, y, t (interpolated
// when needed) or one value per node, e.g. a solution moved to a refined
// mesh by transfer_field (transfer.hpp).
using Field = std::variant<Expr, Vector>;

// Reject a field that cannot give one finite value per node of `mesh`.
// `description` starts the message, e.g. "Initial value".
inline void validate_field(const discretization::TriangleMesh &mesh,
                           const Field &field, const std::string &description) {
  if (const auto *expression = std::get_if<Expr>(&field)) {
    require_symbols(*expression, field_symbols, description);
    return;
  }
  const auto &values = std::get<Vector>(field);
  if (values.size() != static_cast<Eigen::Index>(mesh.nodes.size()) ||
      !values.allFinite())
    throw std::invalid_argument(description +
                                " must have one finite entry per mesh node");
}
// Nodal values of a validated field at `time` (values are returned as is).
inline Vector nodal_values(const discretization::TriangleMesh &mesh,
                           const Field &field, double time = 0,
                           const pde2d::ExecutionOptions &execution = {}) {
  if (const auto *expression = std::get_if<Expr>(&field))
    return interpolate(mesh, *expression, time, execution);
  return std::get<Vector>(field);
}
} // namespace pde2d::fem
