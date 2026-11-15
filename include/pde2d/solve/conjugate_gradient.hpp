#pragma once

#include "pde2d/solve/linear_solver.hpp"

#include <cstddef>

namespace pde2d::solve {

struct ConjugateGradientOptions {
  // The solve stops when the relative residual |b - A x| / |b| is at most
  // this; in (0, 1).
  double tolerance = 1e-10;
  // The solve throws std::runtime_error when it has not converged after
  // this many iterations; at least 1.
  std::size_t max_iterations = 1000;
};

// The iterative conjugate gradient method with a diagonal (Jacobi)
// preconditioner: Eigen's ConjugateGradient, using both triangles of the
// matrix. It requires a symmetric matrix (requirements()) and converges
// only for a positive definite one; on any other it fails to reach its
// tolerance and throws std::runtime_error (the measured residual decides,
// so it never returns a wrong answer). Otherwise it behaves like
// BiCgStab: prepare() views the matrix without copying it, every solve
// starts from x = 0, the right-hand side is scaled so that a tiny one
// still converges, and a zero right-hand side gives x = 0 with no
// iterations.
class ConjugateGradient final : public LinearSolver {
public:
  // Throws std::invalid_argument for options out of range.
  explicit ConjugateGradient(ConjugateGradientOptions options = {});
  const ConjugateGradientOptions &options() const { return options_; }

  std::string name() const override { return "conjugate_gradient"; }
  LinearSolverRequirements requirements() const override {
    return {.symmetric_matrix = true};
  }
  std::unique_ptr<const PreparedLinearSolver>
  prepare(std::shared_ptr<const fem::CsrMatrix> matrix) const override;

private:
  ConjugateGradientOptions options_;
};

} // namespace pde2d::solve
