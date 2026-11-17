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
  // How many threads Eigen's sparse matrix-vector products may use: 1
  // runs in the calling thread, 0 means all hardware threads (the
  // processors available to this process, omp_get_num_procs; not
  // OMP_NUM_THREADS). At most 1024. Eigen uses them only for matrices with
  // more than 20000 nonzeros. The iterations and the answer are bit for
  // bit the same for every count. The count is set per calling thread, so
  // solves running at once in different threads keep their own counts,
  // unless the program sets one for all with Eigen::setNbThreads (that
  // count then wins; the answer is unchanged).
  std::size_t threads = 1;
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
  LinearSolverRequirements requirements() const override { return {}; }
  std::unique_ptr<const PreparedLinearSolver>
  prepare(std::shared_ptr<const fem::CsrMatrix> matrix) const override;

private:
  BiCgStabOptions options_;
};

} // namespace pde2d::solve
