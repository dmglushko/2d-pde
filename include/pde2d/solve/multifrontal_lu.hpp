#pragma once

#include "pde2d/solve/linear_solver.hpp"

#include <cstddef>

namespace pde2d::solve {

struct MultifrontalLuOptions {
  // Every solve measures the normwise backward error |b - A x| / (|A| |x| +
  // |b|) (infinity norms; LinearSolveResult::backward_error) and throws
  // std::runtime_error above this; in (0, 1). (The name is kept from when
  // it bounded the relative residual.)
  double residual_tolerance = 1e-8;
  // How many threads the factorization and the solves may use: 1 runs in
  // the calling thread, 0 means all hardware threads (the processors
  // available to this process, omp_get_num_procs; not OMP_NUM_THREADS).
  // At most 1024. ParU runs that many OpenMP threads (the fronts of
  // independent subtrees are factorized at once, and large dense updates
  // are split among threads), each calling the BLAS (OpenBLAS), whose own
  // pool gets the same count; so at times up to about twice `threads`
  // threads are busy. With more than one thread the order of the work is
  // decided while it runs, so the answer may differ in the last digits
  // between thread counts and between runs; with 1 thread it is the same,
  // bit for bit, on every run.
  std::size_t threads = 1;
};

// A direct solver for any square sparse matrix: a multifrontal LU
// factorization with threshold partial pivoting (ParU, which takes its
// ordering and its symbolic analysis from UMFPACK: METIS on the pattern
// of A + A^T when the matrix is nearly symmetric in pattern, as assembled
// matrices are). The rows are scaled by their largest entry first.
//
// prepare() factorizes and throws std::runtime_error when a pivot is
// exactly zero (an exactly singular matrix, a row without entries). A tiny
// pivot is accepted: the ratio of the smallest pivot of the scaled matrix
// to the largest, over every pivot (the fronts and the singletons
// eliminated before them), is reported (LinearSolveResult::pivot_ratio),
// not judged. So a pure Neumann problem, whose constant is not fixed and
// whose last pivot is rounding (a ratio of about 1e-17), gets one of its
// solutions, and a very ill-conditioned but well-posed problem is solved.
// Every solve checks its backward error against residual_tolerance, so it
// never returns an unchecked answer; the relative residual |b - A x| /
// |b| is reported but not checked (for an accurate answer it is machine
// epsilon times |A| |x| / |b|, which a coefficient contrast of 1e6 makes
// large).
//
// Threads: OpenBLAS has one thread count for the whole process, so this
// solver sets it for the duration of each factorization and solve and
// restores it afterwards, under the lock shared with SupernodalCholesky
// (see there). A program that sets OpenBLAS's count during one of these
// solves can have its setting overwritten.
//
// Linking: Ubuntu's ParU replaces the global operator new and delete of
// every program that links pde2d_solve (see SupernodalCholesky): a
// std::new_handler the program installs is skipped when memory runs out.
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
