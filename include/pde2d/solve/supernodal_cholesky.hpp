#pragma once

#include "pde2d/solve/linear_solver.hpp"

#include <cstddef>

namespace pde2d::solve {

struct SupernodalCholeskyOptions {
  // Every solve measures the relative residual |b - A x| / |b| and throws
  // std::runtime_error above this; in (0, 1).
  double residual_tolerance = 1e-8;
  // How many threads the factorization and the solves may use: 1 runs in
  // the calling thread, 0 means all hardware threads (the processors
  // available to this process, omp_get_num_procs; not OMP_NUM_THREADS).
  // At most 1024. They are the threads of the BLAS (OpenBLAS), which does
  // the dense work on the supernodes, and of CHOLMOD's own OpenMP loops.
  // The answer may differ in the last digits between thread counts; with
  // 1 thread it is the same, bit for bit, on every run.
  std::size_t threads = 1;
};

// A direct solver for symmetric positive definite sparse matrices: a
// supernodal Cholesky factorization L L^T (CHOLMOD) with a fill-reducing
// ordering (CHOLMOD's default: AMD, or METIS when AMD fills much). The
// supernodes are dense blocks, factorized with the BLAS, which is where
// the threads work. It reads only the lower triangle, so it requires a
// symmetric matrix (requirements()).
//
// prepare() factorizes and throws std::runtime_error when the matrix is
// not positive definite (a negative definite matrix, for example from an
// equation written with positive a_xx and a_yy, is not either) or is
// singular to working precision (its smallest pivot is below n machine
// epsilons times the largest). Every solve checks its relative residual
// against residual_tolerance, so it never returns an unchecked answer.
//
// Threads: OpenBLAS has one thread count for the whole process, so this
// solver sets it for the duration of each factorization and solve and
// restores it afterwards, under a lock shared with MultifrontalLu: their
// factorizations and solves that would run at once in different threads
// run one after another. Other code that calls OpenBLAS during that time
// sees this solver's count.
class SupernodalCholesky final : public LinearSolver {
public:
  // Throws std::invalid_argument for options out of range.
  explicit SupernodalCholesky(SupernodalCholeskyOptions options = {});
  const SupernodalCholeskyOptions &options() const { return options_; }

  std::string name() const override { return "supernodal_cholesky"; }
  LinearSolverRequirements requirements() const override {
    return {.symmetric_matrix = true};
  }
  std::unique_ptr<const PreparedLinearSolver>
  prepare(std::shared_ptr<const fem::CsrMatrix> matrix) const override;

private:
  SupernodalCholeskyOptions options_;
};

} // namespace pde2d::solve
