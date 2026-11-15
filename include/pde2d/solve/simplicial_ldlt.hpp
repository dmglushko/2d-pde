#pragma once

#include "pde2d/solve/linear_solver.hpp"

namespace pde2d::solve {

// A direct solver for symmetric sparse matrices: an LDL^T factorization
// without pivoting, with a fill-reducing ordering (Eigen's SimplicialLDLT
// with AMD). It reads only the lower triangle, so it requires a symmetric
// matrix (requirements()). prepare() factorizes and throws
// std::runtime_error when that fails (a zero pivot: a singular matrix, or
// an indefinite one that needs pivoting).
//
// Without pivoting, a tiny pivot of an indefinite matrix passes the
// factorization but gives a wrong answer. So solve() measures the relative
// residual |b - A x| / |b| of every result and throws std::runtime_error
// when it is above residual_tolerance (or the result is not finite): it
// never returns an unchecked answer.
struct SimplicialLdltOptions {
  double residual_tolerance = 1e-8;
};

class SimplicialLdlt final : public LinearSolver {
public:
  // Throws std::invalid_argument unless 0 < residual_tolerance < 1.
  explicit SimplicialLdlt(SimplicialLdltOptions options = {});

  std::string name() const override { return "simplicial_ldlt"; }
  LinearSolverRequirements requirements() const override {
    return {.symmetric_matrix = true};
  }
  std::unique_ptr<const PreparedLinearSolver>
  prepare(std::shared_ptr<const fem::CsrMatrix> matrix) const override;

private:
  SimplicialLdltOptions options_;
};

} // namespace pde2d::solve
