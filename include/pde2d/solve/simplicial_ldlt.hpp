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
// factorization but gives a wrong answer. So solve() measures the
// normwise backward error |b - A x| / (|A| |x| + |b|) (infinity norms;
// LinearSolveResult::backward_error) of every result and throws
// std::runtime_error when it is above residual_tolerance (or the result is
// not finite): it never returns an unchecked answer. The backward error,
// not the relative residual |b - A x| / |b|, is checked because the
// residual of an accurate answer is rounding of size machine epsilon times
// |A| |x|, which a coefficient contrast of 1e6 makes larger than |b| times
// any useful tolerance. Every result also reports the ratio of the
// smallest pivot |D_ii| to the largest (pivot_ratio), which is not
// judged; below 1e-10 the result carries a warning in its messages (the
// matrix is nearly singular).
struct SimplicialLdltOptions {
  // The bound on the backward error of every solve (the name is kept from
  // when it bounded the relative residual); in (0, 1).
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
