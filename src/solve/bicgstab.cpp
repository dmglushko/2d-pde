#include "pde2d/solve/bicgstab.hpp"

#include "iterative_solve.hpp"

#include <memory>
#include <utility>

namespace pde2d::solve {

namespace {

using Solver = Eigen::BiCGSTAB<detail::RowMajorMatrix,
                               Eigen::DiagonalPreconditioner<double>>;

detail::IterativeOptions iterative(const BiCgStabOptions &options) {
  return {.tolerance = options.tolerance,
          .max_iterations = options.max_iterations};
}

} // namespace

BiCgStab::BiCgStab(BiCgStabOptions options) : options_(options) {
  detail::check_options(iterative(options_), name());
}

std::unique_ptr<const PreparedLinearSolver>
BiCgStab::prepare(std::shared_ptr<const fem::CsrMatrix> matrix) const {
  detail::check_matrix(matrix, name());
  return std::make_unique<const detail::PreparedIterative<Solver>>(
      std::move(matrix), iterative(options_), name());
}

} // namespace pde2d::solve
