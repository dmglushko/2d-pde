#pragma once
// Public contracts of the linear solvers, without any solver implementation:
// the built-in method and preconditioner selectors, SolverOptions, the
// reports a solve returns, the backend interface (LinearSolverBackend) and
// its factory (SolverOptions::backend) and report names. The driver
// (PreparedLinearSolver, solve_linear) is in prepared_linear_solver.hpp,
// Dirichlet lifting in constraints.hpp and the built-in Eigen backends in
// detail/eigen_backends.hpp; linear_solver.hpp includes all of them.
#include "fem/system.hpp"
#include "fem/timing.hpp"
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace pde2d::fem {
enum class LinearSolver { SparseLU, ConjugateGradient, GMRES };
// Preconditioner of the iterative backends. Default keeps each backend's
// standard choice: Diagonal (Jacobi) for ConjugateGradient and GMRES, none
// for SparseLU. IncompleteCholesky is for ConjugateGradient, IncompleteLUT
// for GMRES (both with Eigen's default parameters).
enum class Preconditioner {
  Default,
  Identity,
  Diagonal,
  IncompleteCholesky,
  IncompleteLUT
};
// AssumeValid transfers responsibility for nonsingularity (and positive
// definiteness for CG) to the caller. Checked preserves numerical rank checks.
enum class SolverValidation { Checked, AssumeValid };

class LinearSolverBackendFactory;
struct SolverOptions {
  LinearSolver method = LinearSolver::SparseLU;
  double relative_tolerance = 1e-10, absolute_tolerance = 1e-12;
  int max_iterations = 2000, restart = 40;
  SolverValidation validation = SolverValidation::Checked;
  Preconditioner preconditioner = Preconditioner::Default;
  // A user backend. When set it replaces `method`; it receives these options
  // (including `preconditioner`) and interprets them itself. Compared by
  // identity, so reuse the same factory object to reuse factorizations.
  std::shared_ptr<const LinearSolverBackendFactory> backend{};
  // Every field takes part, so a new option cannot be forgotten by reuse.
  bool operator==(const SolverOptions &) const = default;
};

// What a solve did with an initial guess: none was given, the backend
// started from it (iterative backends), or the backend cannot use one and
// ignored it (direct factorizations such as SparseLU).
enum class InitialGuess { None, Used, Ignored };
inline const char *initial_guess_name(InitialGuess guess) {
  switch (guess) {
  case InitialGuess::None:
    return "None";
  case InitialGuess::Used:
    return "Used";
  case InitialGuess::Ignored:
    return "Ignored";
  }
  return "Invalid";
}

struct SolverReport {
  bool converged = false;
  int iterations = 0;
  double residual_norm = std::numeric_limits<double>::infinity();
  double relative_residual = std::numeric_limits<double>::infinity();
  std::string message;
  SolverTimings timings;
  InitialGuess initial_guess = InitialGuess::None;
};
struct LinearSolution {
  Vector values;
  SolverReport report;
};

// What a backend returns for one right-hand side.
struct BackendSolution {
  Vector values;
  int iterations = 0;
  // Nonempty when the backend itself failed. The message is reported as is
  // and the true residual is not checked.
  std::string failure{};
  // Set by solve_with_guess when the guess was actually the starting point.
  bool used_guess = false;
};

// The method-specific part of a prepared linear solver, for one matrix.
class LinearSolverBackend {
public:
  virtual ~LinearSolverBackend() = default;
  // Matrix-dependent setup. `matrix` is square, nonempty, finite, compressed
  // and, when options.validation is Checked, numerically full rank. Return
  // an empty string on success or a failure message that every later solve
  // reports (solving is then skipped).
  virtual std::string prepare(const SparseMatrix &matrix,
                              const SolverOptions &options) = 0;
  // Solve for one finite RHS. `relative_tolerance` already combines the
  // absolute and relative options for this RHS: the driver accepts the result
  // when ||A x - b|| <= relative_tolerance * ||b|| (or the absolute tolerance
  // for b = 0). Calls are sequential.
  virtual BackendSolution solve(const Vector &rhs,
                                double relative_tolerance) = 0;
  // Solve starting from `guess` (finite, one entry per row). Iterative
  // backends override this and set used_guess. The default ignores the
  // guess, so direct backends need not override it; the report then says
  // InitialGuess::Ignored.
  virtual BackendSolution solve_with_guess(const Vector &rhs,
                                           double relative_tolerance,
                                           const Vector &guess) {
    (void)guess;
    auto result = solve(rhs, relative_tolerance);
    result.used_guess = false;
    return result;
  }
};

// Creates backends; held by SolverOptions::backend.
class LinearSolverBackendFactory {
public:
  virtual ~LinearSolverBackendFactory() = default;
  // Name used in reports.
  virtual std::string name() const = 0;
  // A fresh backend for one matrix. Throw std::invalid_argument for options
  // the backend does not support.
  virtual std::unique_ptr<LinearSolverBackend>
  create(const SolverOptions &options) const = 0;
};

using LinearSolverBackendCreator =
    std::function<std::unique_ptr<LinearSolverBackend>(const SolverOptions &)>;
// A factory from a report name and a creation function.
inline std::shared_ptr<const LinearSolverBackendFactory>
make_linear_solver_backend(std::string name,
                           LinearSolverBackendCreator create) {
  class Factory final : public LinearSolverBackendFactory {
    std::string name_;
    LinearSolverBackendCreator create_;

  public:
    Factory(std::string name, LinearSolverBackendCreator create)
        : name_(std::move(name)), create_(std::move(create)) {}
    std::string name() const override { return name_; }
    std::unique_ptr<LinearSolverBackend>
    create(const SolverOptions &options) const override {
      auto backend = create_(options);
      if (!backend)
        throw std::invalid_argument("Linear solver backend factory returned "
                                    "no backend");
      return backend;
    }
  };
  if (!create)
    throw std::invalid_argument("Linear solver backend needs a creator");
  return std::make_shared<const Factory>(std::move(name), std::move(create));
}

// Report name of a built-in method, or nullptr for a value that names none.
// The built-in backends take their names from here.
constexpr const char *linear_solver_method_name(LinearSolver method) {
  switch (method) {
  case LinearSolver::SparseLU:
    return "SparseLU";
  case LinearSolver::ConjugateGradient:
    return "ConjugateGradient";
  case LinearSolver::GMRES:
    return "GMRES";
  }
  return nullptr;
}
// Report name of the selected backend (built-in or user).
inline std::string linear_solver_name(const SolverOptions &options) {
  if (options.backend)
    return options.backend->name();
  const char *name = linear_solver_method_name(options.method);
  return name ? name : "Unknown";
}
inline const char *preconditioner_name(Preconditioner p) {
  switch (p) {
  case Preconditioner::Default:
    return "Default";
  case Preconditioner::Identity:
    return "Identity";
  case Preconditioner::Diagonal:
    return "Diagonal";
  case Preconditioner::IncompleteCholesky:
    return "IncompleteCholesky";
  case Preconditioner::IncompleteLUT:
    return "IncompleteLUT";
  }
  return "Unknown";
}

inline const char *solver_validation_name(SolverValidation validation) {
  return validation == SolverValidation::Checked ? "Checked" : "AssumeValid";
}

} // namespace pde2d::fem
