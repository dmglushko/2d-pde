#pragma once

#include "pde2d/solve/linear_solver.hpp"

#include <cstddef>

namespace pde2d::solve {

struct SupernodalCholeskyOptions {
  // Every solve measures the normwise backward error |b - A x| / (|A| |x| +
  // |b|) (infinity norms; LinearSolveResult::backward_error) and throws
  // std::runtime_error above this; in (0, 1). (The name is kept from when
  // it bounded the relative residual.)
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
// not positive definite: a pivot is zero (an exactly singular matrix) or
// negative (a negative definite matrix, for example from an equation
// written with positive a_xx and a_yy, or an indefinite one). A tiny
// positive pivot is accepted: the ratio of the smallest pivot to the
// largest is reported (LinearSolveResult::pivot_ratio), not judged, so a
// pure Neumann problem whose last pivot rounds to a tiny positive number
// gets one of its solutions, and a very ill-conditioned but well-posed
// problem is solved. Every solve checks its backward error against
// residual_tolerance, so it never returns an unchecked answer; the
// relative residual |b - A x| / |b| is reported but not checked (for an
// accurate answer it is machine epsilon times |A| |x| / |b|, which a
// coefficient contrast of 1e6 makes large).
//
// Threads: OpenBLAS has one thread count for the whole process, so this
// solver sets it for the duration of each factorization and solve and
// restores it afterwards, under a lock shared with MultifrontalLu: their
// factorizations and solves that would run at once in different threads
// run one after another. Other code that calls OpenBLAS during that time
// sees this solver's count, and a program that sets OpenBLAS's count
// (openblas_set_num_threads) during one of these solves can have its
// setting overwritten when the solve restores the count it found.
//
// Linking: Ubuntu's SuiteSparse 7.12 builds ParU (which pde2d_solve links
// for MultifrontalLu) with its own global operator new(std::size_t) and
// operator delete(void *), so
// every program that links pde2d_solve allocates through them; on
// allocation failure they throw std::bad_alloc at once, skipping a
// std::new_handler the program installed.
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
