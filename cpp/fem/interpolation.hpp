#pragma once
// Nodal (P1) interpolation of a symbolic field in x, y and t.
#include "assembly.hpp"
#include "execution.hpp"
#include "expression.hpp"

namespace pde2d::fem {
// Values of `field` at every mesh node at the given time. Boundary normals are
// not available (nx = ny = 0). Symbolic preparation happens on the caller;
// workers only read the prepared evaluator.
inline Vector interpolate(const discretization::TriangleMesh &mesh,
                          const Expr &field, double time = 0,
                          const pde2d::ExecutionOptions &execution = {}) {
  Evaluator evaluate_field(field);
  Vector v(mesh.nodes.size());
  pde2d::parallel_for(mesh.nodes.size(), execution, [&](std::size_t i) {
    v[i] = evaluate_field({mesh.nodes[i].x, mesh.nodes[i].y, time});
  });
  return v;
}
} // namespace pde2d::fem
