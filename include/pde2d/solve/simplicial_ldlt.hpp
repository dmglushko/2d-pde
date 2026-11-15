#pragma once

#include "pde2d/solve/linear_solver.hpp"

namespace pde2d::solve {

// A direct solver for symmetric sparse matrices: an LDL^T factorization
// without pivoting, with a fill-reducing ordering (Eigen's SimplicialLDLT
// with AMD). It reads only the lower triangle, so it requires a symmetric
// matrix (requirements()). prepare() factorizes and throws
// std::runtime_error when that fails (a zero pivot: a singular matrix, or
// an indefinite one that needs pivoting); a result that is not finite
// throws std::runtime_error in solve(). It has no options.
class SimplicialLdlt final : public LinearSolver {
public:
  SimplicialLdlt() = default;

  std::string name() const override { return "simplicial_ldlt"; }
  LinearSolverRequirements requirements() const override {
    return {.symmetric_matrix = true};
  }
  std::unique_ptr<const PreparedLinearSolver>
  prepare(std::shared_ptr<const fem::CsrMatrix> matrix) const override;
};

} // namespace pde2d::solve
