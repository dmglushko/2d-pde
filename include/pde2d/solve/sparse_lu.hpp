#pragma once

#include "pde2d/solve/linear_solver.hpp"

namespace pde2d::solve {

// A direct solver for any square sparse matrix: an LU factorization with
// a fill-reducing column ordering (Eigen's SparseLU with COLAMD). prepare()
// factorizes; each solve is two triangular solves. A matrix that is
// singular (a pivot is exactly zero) throws std::runtime_error in
// prepare(); one that is nearly singular can pass and give a result that
// is not finite, which throws std::runtime_error in solve(). It has no
// options.
class SparseLu final : public LinearSolver {
public:
  SparseLu() = default;

  std::string name() const override { return "sparse_lu"; }
  std::unique_ptr<const PreparedLinearSolver>
  prepare(std::shared_ptr<const fem::CsrMatrix> matrix) const override;
};

} // namespace pde2d::solve
