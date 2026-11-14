#pragma once

#include "pde2d/solve/linear_solver.hpp"

#include <cstddef>

namespace pde2d::solve {

struct BiCgStabOptions {
  // The solve stops when the relative residual |b - A x| / |b| is at most
  // this; in (0, 1).
  double tolerance = 1e-10;
  // The solve throws std::runtime_error when it has not converged after
  // this many iterations; at least 1.
  std::size_t max_iterations = 1000;
};

// The iterative BiCGSTAB method for any square sparse matrix (it needs no
// symmetry), with a diagonal (Jacobi) preconditioner: Eigen's BiCGSTAB.
// prepare() computes the preconditioner and views the matrix's arrays
// without copying them; every solve starts from x = 0.
class BiCgStab final : public LinearSolver {
public:
  // Throws std::invalid_argument for options out of range.
  explicit BiCgStab(BiCgStabOptions options = {});
  const BiCgStabOptions &options() const { return options_; }

  std::string name() const override { return "bicgstab"; }
  std::unique_ptr<const PreparedLinearSolver>
  prepare(std::shared_ptr<const fem::CsrMatrix> matrix) const override;

private:
  BiCgStabOptions options_;
};

} // namespace pde2d::solve
