#pragma once
// The mesh-report invariant, enforced by type. Meshing can return a mesh
// that misses its size target (MeshReport::size_satisfied == false, for
// example a Fixed boundary conflict or an exhausted step budget). FEM entry
// points that take a discretization::MeshResult (PreparedStationary,
// solve_stationary, evolve, adapt) refuse such a result:
//
//   solve_stationary(result, problem, plan);    // throws if unsatisfied
//   solve_stationary(accept_unsatisfied(result), problem, plan);  // explicit
//   solve_stationary(result.mesh, problem, plan);  // TriangleMesh: no check
//
// The run record of an accepted result still carries its report
// (MeshProvenance), so the shortfall stays visible.
#include "discretization/mesh_types.hpp"
#include <stdexcept>
#include <string>

namespace pde2d::fem {
// A MeshResult the caller accepts although its report may be unsatisfied.
// Holds a reference: use it in the call it is made for.
class AcceptedMesh {
  const discretization::MeshResult *result_;

public:
  explicit AcceptedMesh(const discretization::MeshResult &result)
      : result_(&result) {}
  const discretization::MeshResult &result() const noexcept { return *result_; }
};
inline AcceptedMesh accept_unsatisfied(const discretization::MeshResult &r) {
  return AcceptedMesh(r);
}

// Throws std::invalid_argument unless the result met its size target.
inline void require_size_satisfied(const discretization::MeshResult &result) {
  if (!result.report.size_satisfied)
    throw std::invalid_argument(
        "MeshResult misses its size target (" +
        (result.report.message.empty() ? std::string("no message")
                                       : result.report.message) +
        "); check report.size_satisfied or pass accept_unsatisfied(result)");
}
} // namespace pde2d::fem
