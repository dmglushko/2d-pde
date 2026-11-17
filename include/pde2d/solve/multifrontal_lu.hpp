#pragma once

#include "pde2d/solve/linear_solver.hpp"

#include <cstddef>

namespace pde2d::solve {

struct MultifrontalLuOptions {
  // Every solve measures the relative residual |b - A x| / |b| and throws
  // std::runtime_error above this; in (0, 1).
  double residual_tolerance = 1e-8;
  // How many threads the factorization and the solves may use: 1 runs in
  // the calling thread, 0 means all hardware threads (the processors
  // available to this process, omp_get_num_procs; not OMP_NUM_THREADS).
  // At most 1024. They are OpenMP threads: the fronts of independent
  // subtrees are factorized at once, and large dense updates are split
  // among threads (the BLAS itself runs in one thread meanwhile). With
  // more than one thread the order of the work is decided while it runs,
  // so the answer may differ in the last digits between thread counts and
  // between runs; with 1 thread it is the same, bit for bit, on every run.
  std::size_t threads = 1;
};

// A direct solver for any square sparse matrix: a multifrontal LU
// factorization with threshold partial pivoting (ParU, which takes its
// ordering and its symbolic analysis from UMFPACK: METIS on the pattern
// of A + A^T when the matrix is nearly symmetric in pattern, as assembled
// matrices are). The rows are scaled by their largest entry first.
//
// prepare() factorizes and throws std::runtime_error when the matrix is
// singular: when a pivot is exactly zero, and when it is singular to
// working precision (its smallest pivot is below n machine epsilons times
// the largest; a pure Neumann problem, whose constant is not fixed,
// factorizes with a last pivot of about 1e-17 n). Every solve checks its
// relative residual against residual_tolerance, so it never returns an
// unchecked answer.
//
// Threads: it sets OpenBLAS's process-wide thread count to 1 for the
// duration of each factorization and solve, under the lock shared with
// SupernodalCholesky (see there).
class MultifrontalLu final : public LinearSolver {
public:
  // Throws std::invalid_argument for options out of range.
  explicit MultifrontalLu(MultifrontalLuOptions options = {});
  const MultifrontalLuOptions &options() const { return options_; }

  std::string name() const override { return "multifrontal_lu"; }
  LinearSolverRequirements requirements() const override { return {}; }
  std::unique_ptr<const PreparedLinearSolver>
  prepare(std::shared_ptr<const fem::CsrMatrix> matrix) const override;

private:
  MultifrontalLuOptions options_;
};

} // namespace pde2d::solve
