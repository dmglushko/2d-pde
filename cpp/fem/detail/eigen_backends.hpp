#pragma once
// Private (solver_detail): the built-in linear solver backends on Eigen
// (SparseLU, ConjugateGradient, GMRES and their preconditioners), the one
// dispatch on LinearSolver (with_builtin), option validation and backend
// creation. Public contracts are in linear_solver_types.hpp; the driver
// that uses these is PreparedLinearSolver (prepared_linear_solver.hpp).
#include "fem/linear_solver_types.hpp"
#include "fem/system.hpp"
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseLU>
#include <unsupported/Eigen/IterativeSolvers>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

namespace pde2d::fem {
namespace solver_detail {
// Adapts an Eigen iterative solver (and its preconditioner) to a backend.
template <class Solver>
class EigenIterativeBackend final : public LinearSolverBackend {
  Solver solver_;
  std::string setup_failure_;

public:
  explicit EigenIterativeBackend(std::string setup_failure)
      : setup_failure_(std::move(setup_failure)) {}
  std::string prepare(const SparseMatrix &matrix,
                      const SolverOptions &options) override {
    solver_.setMaxIterations(options.max_iterations);
    if constexpr (requires { solver_.set_restart(options.restart); })
      solver_.set_restart(options.restart);
    solver_.compute(matrix);
    // Diagonal and identity preconditioners always succeed.
    return solver_.info() == Eigen::Success ? std::string() : setup_failure_;
  }
  BackendSolution solve(const Vector &rhs, double tolerance) override {
    solver_.setTolerance(tolerance);
    BackendSolution result{solver_.solve(rhs)};
    result.iterations = static_cast<int>(solver_.iterations());
    return result;
  }
  BackendSolution solve_with_guess(const Vector &rhs, double tolerance,
                                   const Vector &guess) override {
    solver_.setTolerance(tolerance);
    BackendSolution result{solver_.solveWithGuess(rhs, guess)};
    result.iterations = static_cast<int>(solver_.iterations());
    result.used_guess = true;
    return result;
  }
};
template <template <class, class> class Solver, class Preconditioner>
std::unique_ptr<LinearSolverBackend> iterative(const char *setup_failure) {
  return std::make_unique<
      EigenIterativeBackend<Solver<SparseMatrix, Preconditioner>>>(
      setup_failure);
}
template <class Matrix, class Preconditioner>
using SymmetricCG =
    Eigen::ConjugateGradient<Matrix, Eigen::Lower | Eigen::Upper,
                             Preconditioner>;
template <class Matrix, class Preconditioner>
using Gmres = Eigen::GMRES<Matrix, Preconditioner>;

// Each built-in backend: its report name, the preconditioners it supports,
// matrix-dependent setup and solve.
class SparseLUBackend final : public LinearSolverBackend {
  Eigen::SparseLU<SparseMatrix> lu_;

public:
  static constexpr const char *name =
      linear_solver_method_name(LinearSolver::SparseLU);
  static bool supports(Preconditioner p) {
    return p == Preconditioner::Default;
  }
  explicit SparseLUBackend(Preconditioner) {}
  std::string prepare(const SparseMatrix &matrix,
                      const SolverOptions &) override {
    lu_.compute(matrix);
    return lu_.info() == Eigen::Success ? "" : "SparseLU factorization failed";
  }
  BackendSolution solve(const Vector &rhs, double) override {
    BackendSolution result{lu_.solve(rhs), 1};
    if (lu_.info() != Eigen::Success)
      result.failure = "SparseLU solve failed";
    return result;
  }
};

class ConjugateGradientBackend final : public LinearSolverBackend {
  std::unique_ptr<LinearSolverBackend> iterative_;

public:
  static constexpr const char *name =
      linear_solver_method_name(LinearSolver::ConjugateGradient);
  static bool supports(Preconditioner p) {
    return p == Preconditioner::Default || p == Preconditioner::Identity ||
           p == Preconditioner::Diagonal ||
           p == Preconditioner::IncompleteCholesky;
  }
  explicit ConjugateGradientBackend(Preconditioner p) {
    if (p == Preconditioner::Identity)
      iterative_ = iterative<SymmetricCG, Eigen::IdentityPreconditioner>("");
    else if (p == Preconditioner::IncompleteCholesky)
      iterative_ = iterative<SymmetricCG, Eigen::IncompleteCholesky<double>>(
          "IncompleteCholesky preconditioner failed");
    else
      iterative_ =
          iterative<SymmetricCG, Eigen::DiagonalPreconditioner<double>>("");
  }
  std::string prepare(const SparseMatrix &matrix,
                      const SolverOptions &options) override {
    if (!numerically_symmetric(matrix))
      return "ConjugateGradient requires a symmetric positive definite matrix";
    if (options.validation == SolverValidation::Checked) {
      Eigen::SimplicialLDLT<SparseMatrix> ldlt;
      ldlt.compute(matrix);
      if (ldlt.info() != Eigen::Success || !ldlt.vectorD().allFinite() ||
          ldlt.vectorD().minCoeff() <= 0)
        return "ConjugateGradient requires a positive definite matrix";
    }
    return iterative_->prepare(matrix, options);
  }
  BackendSolution solve(const Vector &rhs, double tolerance) override {
    return iterative_->solve(rhs, tolerance);
  }
  BackendSolution solve_with_guess(const Vector &rhs, double tolerance,
                                   const Vector &guess) override {
    return iterative_->solve_with_guess(rhs, tolerance, guess);
  }
};

class GmresBackend final : public LinearSolverBackend {
  std::unique_ptr<LinearSolverBackend> iterative_;

public:
  static constexpr const char *name =
      linear_solver_method_name(LinearSolver::GMRES);
  static bool supports(Preconditioner p) {
    return p == Preconditioner::Default || p == Preconditioner::Identity ||
           p == Preconditioner::Diagonal || p == Preconditioner::IncompleteLUT;
  }
  explicit GmresBackend(Preconditioner p) {
    if (p == Preconditioner::Identity)
      iterative_ = iterative<Gmres, Eigen::IdentityPreconditioner>("");
    else if (p == Preconditioner::IncompleteLUT)
      iterative_ = iterative<Gmres, Eigen::IncompleteLUT<double>>(
          "IncompleteLUT preconditioner failed");
    else
      iterative_ = iterative<Gmres, Eigen::DiagonalPreconditioner<double>>("");
  }
  std::string prepare(const SparseMatrix &matrix,
                      const SolverOptions &options) override {
    return iterative_->prepare(matrix, options);
  }
  BackendSolution solve(const Vector &rhs, double tolerance) override {
    return iterative_->solve(rhs, tolerance);
  }
  BackendSolution solve_with_guess(const Vector &rhs, double tolerance,
                                   const Vector &guess) override {
    return iterative_->solve_with_guess(rhs, tolerance, guess);
  }
};

// The only dispatch on LinearSolver: calls f(std::type_identity<Backend>).
template <class F> decltype(auto) with_builtin(LinearSolver method, F &&f) {
  switch (method) {
  case LinearSolver::SparseLU:
    return f(std::type_identity<SparseLUBackend>{});
  case LinearSolver::ConjugateGradient:
    return f(std::type_identity<ConjugateGradientBackend>{});
  case LinearSolver::GMRES:
    return f(std::type_identity<GmresBackend>{});
  }
  throw std::invalid_argument("Unsupported linear solver");
}

inline void validate(const SolverOptions &o) {
  if (!std::isfinite(o.relative_tolerance) ||
      !std::isfinite(o.absolute_tolerance) || o.relative_tolerance < 0 ||
      o.absolute_tolerance < 0 ||
      (o.relative_tolerance == 0 && o.absolute_tolerance == 0) ||
      o.max_iterations <= 0 || o.restart <= 0)
    throw std::invalid_argument("Invalid linear solver options");
  if (!o.backend)
    with_builtin(o.method, [](auto) {});
  switch (o.validation) {
  case SolverValidation::Checked:
  case SolverValidation::AssumeValid:
    break;
  default:
    throw std::invalid_argument("Unsupported solver validation policy");
  }
  switch (o.preconditioner) {
  case Preconditioner::Default:
  case Preconditioner::Identity:
  case Preconditioner::Diagonal:
  case Preconditioner::IncompleteCholesky:
  case Preconditioner::IncompleteLUT:
    break;
  default:
    throw std::invalid_argument("Unsupported preconditioner");
  }
  if (!o.backend)
    with_builtin(o.method, [&](auto backend) {
      using Backend = typename decltype(backend)::type;
      if (!Backend::supports(o.preconditioner))
        throw std::invalid_argument(
            std::string("Preconditioner is not supported by ") + Backend::name);
    });
}
inline std::unique_ptr<LinearSolverBackend>
make_backend(const SolverOptions &o) {
  if (o.backend) {
    auto backend = o.backend->create(o);
    if (!backend)
      throw std::invalid_argument("Linear solver backend factory returned "
                                  "no backend");
    return backend;
  }
  return with_builtin(
      o.method, [&](auto backend) -> std::unique_ptr<LinearSolverBackend> {
        using Backend = typename decltype(backend)::type;
        return std::make_unique<Backend>(o.preconditioner);
      });
}
} // namespace solver_detail
} // namespace pde2d::fem
