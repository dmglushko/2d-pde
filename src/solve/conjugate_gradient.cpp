#include "pde2d/solve/conjugate_gradient.hpp"

#include "iterative_solve.hpp"

#include <memory>
#include <utility>

namespace pde2d::solve {

namespace {

// Lower | Upper: the product uses the whole matrix as stored, so the
// matrix need not be exactly symmetric to the last bit (assembly computes
// both triangles in full).
using Solver =
    Eigen::ConjugateGradient<detail::RowMajorMatrix, Eigen::Lower | Eigen::Upper,
                             Eigen::DiagonalPreconditioner<double>>;

detail::IterativeOptions iterative(const ConjugateGradientOptions &options) {
  return {.tolerance = options.tolerance,
          .max_iterations = options.max_iterations};
}

} // namespace

ConjugateGradient::ConjugateGradient(ConjugateGradientOptions options)
    : options_(options) {
  detail::check_options(iterative(options_), name());
}

std::unique_ptr<const PreparedLinearSolver>
ConjugateGradient::prepare(std::shared_ptr<const fem::CsrMatrix> matrix) const {
  detail::check_matrix(matrix, name());
  return std::make_unique<const detail::PreparedIterative<Solver>>(
      std::move(matrix), iterative(options_), name());
}

} // namespace pde2d::solve
