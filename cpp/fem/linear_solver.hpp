#pragma once
// Linear solves for assembled systems, independent of the element type.
//
//   constrain(A, b, dirichlet)        Dirichlet lifting
//   PreparedLinearSolver(A, options)  validation, rank check, backend setup
//   prepared.solve(b)                 backend solve + true-residual check
//
// A backend (LinearSolverBackend) owns only the matrix-dependent work of one
// method: factorization or preconditioner setup, and the solve itself.
// PreparedLinearSolver keeps everything shared: option and matrix
// validation, the numerical rank check, tolerances, the true-residual check,
// timings and the reuse test. The built-in backends below are selected by
// SolverOptions::method; SolverOptions::backend plugs in a user backend
// without editing the library (see EXTENDING.md).
#include "fem/decision.hpp"
#include "fem/system.hpp"
#include "fem/timing.hpp"
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseLU>
#include <Eigen/SparseQR>
#include <unsupported/Eigen/IterativeSolvers>
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

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

struct SolverReport {
  bool converged = false;
  int iterations = 0;
  double residual_norm = std::numeric_limits<double>::infinity();
  double relative_residual = std::numeric_limits<double>::infinity();
  std::string message;
  SolverTimings timings;
  // The choices that produced this solve (solver_decisions()).
  std::vector<Decision> decisions{};
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
  static constexpr const char *name = "SparseLU";
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
  static constexpr const char *name = "ConjugateGradient";
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
};

class GmresBackend final : public LinearSolverBackend {
  std::unique_ptr<LinearSolverBackend> iterative_;

public:
  static constexpr const char *name = "GMRES";
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
inline void validate(const SparseMatrix &a, const Vector &b) {
  if (a.rows() != a.cols() || a.rows() != b.size() || a.rows() == 0)
    throw std::invalid_argument(
        "Linear system must be nonempty and square with matching RHS");
  if (!b.allFinite())
    throw std::invalid_argument("Nonfinite linear RHS");
  // An uncompressed Eigen matrix may have unused slots between columns.
  for (int column = 0; column < a.outerSize(); ++column)
    for (SparseMatrix::InnerIterator entry(a, column); entry; ++entry)
      if (!std::isfinite(entry.value()))
        throw std::invalid_argument("Nonfinite linear matrix");
}
} // namespace solver_detail

// Report name of the selected backend (built-in or user).
inline std::string linear_solver_name(const SolverOptions &options) {
  if (options.backend)
    return options.backend->name();
  try {
    return solver_detail::with_builtin(options.method, [](auto backend) {
      return std::string(decltype(backend)::type::name);
    });
  } catch (const std::invalid_argument &) {
    return "Unknown";
  }
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

// The user's linear-solver choices as decision records (decision.hpp).
inline std::vector<Decision> solver_decisions(const SolverOptions &options) {
  return {user_decision("linear solver", linear_solver_name(options),
                        options.backend ? "SolverOptions::backend"
                                        : "SolverOptions::method"),
          user_decision("preconditioner",
                        preconditioner_name(options.preconditioner),
                        "SolverOptions::preconditioner"),
          user_decision("solver validation",
                        solver_validation_name(options.validation),
                        "SolverOptions::validation")};
}

inline std::pair<SparseMatrix, Vector>
constrain(const SparseMatrix &a, const Vector &b,
          const std::vector<DirichletValue> &constraints) {
  solver_detail::validate(a, b);
  std::map<Eigen::Index, double> prescribed;
  for (const auto &d : constraints) {
    if (d.node < 0 || d.node >= a.rows() || !std::isfinite(d.value))
      throw std::invalid_argument("Invalid Dirichlet constraint");
    auto [it, inserted] = prescribed.emplace(d.node, d.value);
    if (!inserted && it->second != d.value)
      throw std::invalid_argument("Conflicting Dirichlet constraints");
  }
  Vector rhs = b;
  std::vector<Eigen::Triplet<double>> entries;
  for (int k = 0; k < a.outerSize(); ++k)
    for (SparseMatrix::InnerIterator it(a, k); it; ++it) {
      if (prescribed.contains(it.row()))
        continue;
      const auto p = prescribed.find(it.col());
      if (p != prescribed.end())
        rhs[it.row()] -= it.value() * p->second;
      else
        entries.emplace_back(it.row(), it.col(), it.value());
    }
  for (const auto &[i, g] : prescribed) {
    entries.emplace_back(i, i, 1);
    rhs[i] = g;
  }
  SparseMatrix result(a.rows(), a.cols());
  result.setFromTriplets(entries.begin(), entries.end());
  if (!rhs.allFinite())
    throw std::overflow_error("Nonfinite Dirichlet lifting");
  return {std::move(result), std::move(rhs)};
}

// Owns its matrix snapshot and all matrix-dependent preparation. Calls to solve
// are sequential; each returned report contains only that RHS's work.
class PreparedLinearSolver {
  SparseMatrix matrix_;
  SolverOptions options_;
  SolverTimings preparation_;
  std::string failure_;
  std::unique_ptr<LinearSolverBackend> backend_;

public:
  PreparedLinearSolver(const SparseMatrix &input,
                       const SolverOptions &options = {})
      : options_(options) {
    const auto started = timing::Clock::now();
    matrix_ = input;
    solver_detail::validate(options_);
    backend_ = solver_detail::make_backend(options_);
    solver_detail::validate(matrix_, Vector::Zero(matrix_.rows()));
    matrix_.makeCompressed();
    preparation_.preparations = 1;
    preparation_.validation_seconds = timing::seconds_since(started);
    auto phase = timing::Clock::now();
    if (options_.validation == SolverValidation::Checked) {
      Eigen::SparseQR<SparseMatrix, Eigen::COLAMDOrdering<int>> rank;
      rank.compute(matrix_);
      ++preparation_.rank_checks;
      preparation_.rank_check_seconds = timing::seconds_since(phase);
      if (rank.info() != Eigen::Success || rank.rank() != matrix_.rows())
        failure_ = "Singular or numerically rank-deficient linear system";
    }
    if (failure_.empty()) {
      phase = timing::Clock::now();
      ++preparation_.factorizations;
      failure_ = backend_->prepare(matrix_, options_);
      preparation_.factorization_seconds = timing::seconds_since(phase);
    }
    preparation_.total_seconds = timing::seconds_since(started);
  }
  const SparseMatrix &matrix() const noexcept { return matrix_; }
  const SolverOptions &options() const noexcept { return options_; }
  const SolverTimings &preparation_timings() const noexcept {
    return preparation_;
  }
  // Compare actual coefficients (including explicit zeros), never identities or
  // merely sparsity. A conservative false miss is safe; an approximate hit
  // isn't.
  bool matches(const SparseMatrix &a, const SolverOptions &o) const {
    if (o != options_ || a.rows() != matrix_.rows() ||
        a.cols() != matrix_.cols())
      return false;
    for (int col = 0; col < a.outerSize(); ++col) {
      SparseMatrix::InnerIterator left(a, col), right(matrix_, col);
      for (; left && right; ++left, ++right)
        if (left.row() != right.row() || left.value() != right.value())
          return false;
      if (left || right)
        return false;
    }
    return true;
  }
  LinearSolution solve(const Vector &b) {
    const auto started = timing::Clock::now();
    if (b.size() != matrix_.rows() || !b.allFinite())
      throw std::invalid_argument("Invalid linear RHS");
    LinearSolution result{Vector::Zero(b.size()), {}};
    auto &r = result.report;
    r.decisions = solver_decisions(options_);
    r.timings.rhs_solves = 1;
    const double bn = b.stableNorm();
    r.residual_norm = bn;
    r.relative_residual = bn > 0 ? 1. : 0.;
    r.timings.validation_seconds = timing::seconds_since(started);
    auto finish = [&]() {
      r.timings.total_seconds = timing::seconds_since(started);
      return std::move(result);
    };
    if (!failure_.empty()) {
      r.message = failure_;
      return finish();
    }
    const double target =
        std::max(options_.absolute_tolerance, options_.relative_tolerance * bn);
    const double tolerance = bn > 0 ? target / bn : options_.relative_tolerance;
    auto phase = timing::Clock::now();
    auto solved = backend_->solve(b, tolerance);
    result.values = std::move(solved.values);
    r.iterations = solved.iterations;
    r.timings.solve_seconds = timing::seconds_since(phase);
    if (!solved.failure.empty()) {
      r.message = std::move(solved.failure);
      return finish();
    }
    if (result.values.size() != b.size())
      throw std::runtime_error("Linear solver backend returned a solution "
                               "of the wrong size");
    phase = timing::Clock::now();
    r.residual_norm = (matrix_ * result.values - b).stableNorm();
    r.relative_residual = bn > 0 ? r.residual_norm / bn : r.residual_norm;
    r.converged = result.values.allFinite() && std::isfinite(r.residual_norm) &&
                  r.residual_norm <= target;
    r.message = r.converged ? "Converged"
                            : "Linear solver did not converge to requested "
                              "true residual tolerance";
    r.timings.residual_seconds = timing::seconds_since(phase);
    return finish();
  }
};
inline LinearSolution solve_linear(const SparseMatrix &input, const Vector &b,
                                   const SolverOptions &o = {}) {
  const auto started = timing::Clock::now();
  solver_detail::validate(o);
  solver_detail::validate(input, b);
  PreparedLinearSolver prepared(input, o);
  auto result = prepared.solve(b);
  result.report.timings += prepared.preparation_timings();
  result.report.timings.total_seconds = timing::seconds_since(started);
  return result;
}
} // namespace pde2d::fem
