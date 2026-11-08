// Linear solver backends: preconditioner choices of the built-in iterative
// backends and a user backend plugged in through SolverOptions::backend.
// Oracles: SparseLU solutions, the shared true-residual check and the
// documented reuse contract.
#include "fem/evolution.hpp"
#include "fem/report.hpp"
#include "discretization/square_mesh.hpp"
#include "support/check.hpp"
#include "support/check_eigen.hpp"
#include <Eigen/Dense>
#include <memory>
#include <sstream>

using pde2d::discretization::square_mesh;
using pde2d_test::check;
using pde2d_test::expect_throws;
namespace {
using namespace pde2d::fem;

// Dense partial-pivoting LU: an independent direct method.
class DenseLU final : public LinearSolverBackend {
  Eigen::PartialPivLU<Eigen::MatrixXd> lu_;
  std::shared_ptr<int> preparations_;

public:
  explicit DenseLU(std::shared_ptr<int> preparations)
      : preparations_(std::move(preparations)) {}
  std::string prepare(const SparseMatrix &matrix,
                      const SolverOptions &) override {
    lu_.compute(Eigen::MatrixXd(matrix));
    ++*preparations_;
    return "";
  }
  BackendSolution solve(const Vector &rhs, double) override {
    return {lu_.solve(rhs), 1};
  }
};

Problem poisson() {
  Problem p;
  p.equation.source = 1 + x * y;
  p.assign(dirichlet(x - y), {"bottom", "left"});
  p.assign(neumann(1), {"top", "right"});
  return p;
}

void preconditioners() {
  const auto mesh = square_mesh(8);
  const auto p = poisson();
  const auto reference = solve_stationary(mesh, p).values;
  struct Case {
    LinearSolver method;
    Preconditioner preconditioner;
  };
  for (auto method : {LinearSolver::ConjugateGradient, LinearSolver::GMRES}) {
    SolverOptions o;
    o.method = method;
    const auto standard = solve_stationary(mesh, p, {}, o);
    o.preconditioner = Preconditioner::Diagonal;
    const auto diagonal = solve_stationary(mesh, p, {}, o);
    // Default keeps the historical Jacobi preconditioner exactly.
    check(standard.values == diagonal.values &&
              standard.report.iterations == diagonal.report.iterations,
          "Default preconditioner changed");
    pde2d_test::near_vector(diagonal.values, reference, 1e-8);
    o.preconditioner = Preconditioner::Identity;
    pde2d_test::near_vector(solve_stationary(mesh, p, {}, o).values,
                            reference, 1e-8);
    o.preconditioner = method == LinearSolver::GMRES
                           ? Preconditioner::IncompleteLUT
                           : Preconditioner::IncompleteCholesky;
    const auto incomplete = solve_stationary(mesh, p, {}, o);
    pde2d_test::near_vector(incomplete.values, reference, 1e-8);
    check(incomplete.report.iterations < diagonal.report.iterations,
          "Incomplete factorization did not reduce iterations");
  }
  for (auto [method, preconditioner, message] :
       {std::tuple{LinearSolver::SparseLU, Preconditioner::Diagonal,
                   "not supported by SparseLU"},
        std::tuple{LinearSolver::ConjugateGradient,
                   Preconditioner::IncompleteLUT,
                   "not supported by ConjugateGradient"},
        std::tuple{LinearSolver::GMRES, Preconditioner::IncompleteCholesky,
                   "not supported by GMRES"},
        std::tuple{LinearSolver::GMRES, static_cast<Preconditioner>(99),
                   "Unsupported preconditioner"}}) {
    SolverOptions o;
    o.method = method;
    o.preconditioner = preconditioner;
    expect_throws<std::invalid_argument>(
        [&] { solve_stationary(mesh, p, {}, o); }, message);
  }
}

void user_backend() {
  const auto mesh = square_mesh(6);
  const auto p = poisson();
  auto preparations = std::make_shared<int>(0);
  SolverOptions o;
  o.backend = make_linear_solver_backend(
      "DenseLU", [preparations](const SolverOptions &) {
        return std::make_unique<DenseLU>(preparations);
      });
  const auto solution = solve_stationary(mesh, p, {}, o);
  check(*preparations == 1 && solution.report.converged &&
            solution.report.iterations == 1,
        "User backend not used");
  pde2d_test::near_vector(solution.values, solve_stationary(mesh, p).values,
                          1e-10);
  check(linear_solver_name(o) == "DenseLU", "User backend name lost");

  // Through evolve(): the constrained heat matrix is constant, so the backend
  // is prepared once and reused for every later step (performance contract).
  Problem heat = p;
  heat.equation.m1 = 1;
  TimeOptions options;
  options.end = .05;
  options.step = .01;
  options.solver = o;
  *preparations = 0;
  const auto evolution = evolve(mesh, heat, {x - y}, options);
  check(*preparations == 1 && evolution.timings.solver_reuses ==
                                  evolution.timings.time_steps - 1,
        "User backend not reused");
  TimeOptions builtin = options;
  builtin.solver = {};
  const auto expected = evolve(mesh, heat, {x - y}, builtin);
  pde2d_test::near_vector(evolution.frames.back().values,
                          expected.frames.back().values, 1e-10);
  std::ostringstream text;
  write_report(text, evolution);
  check(text.str().find("solver=DenseLU") != std::string::npos,
        "Report lost the user backend name");

  // Options compare every field, including the backend identity.
  const auto constrained =
      constrain(assemble(mesh, p).stiffness, assemble(mesh, p).load,
                assemble(mesh, p).dirichlet);
  PreparedLinearSolver prepared(constrained.first, o);
  check(prepared.matches(constrained.first, o), "Identical options missed");
  auto other = o;
  other.backend = make_linear_solver_backend(
      "DenseLU", [preparations](const SolverOptions &) {
        return std::make_unique<DenseLU>(preparations);
      });
  check(!prepared.matches(constrained.first, other),
        "Different backend factory matched");
  other = o;
  other.preconditioner = Preconditioner::Diagonal;
  check(!prepared.matches(constrained.first, other),
        "Different preconditioner matched");
}

// A backend that returns a wrong answer or fails: the shared driver still
// checks the true residual and reports the backend's own failure.
class Fixed final : public LinearSolverBackend {
  std::string failure_;
  Eigen::Index size_;

public:
  Fixed(std::string failure, Eigen::Index size)
      : failure_(std::move(failure)), size_(size) {}
  std::string prepare(const SparseMatrix &, const SolverOptions &) override {
    return "";
  }
  BackendSolution solve(const Vector &, double) override {
    return {Vector::Zero(size_), 3, failure_};
  }
};

void backend_failures() {
  Eigen::MatrixXd dense(2, 2);
  dense << 2, 1, 1, 3;
  const SparseMatrix a = dense.sparseView();
  const Vector b = Vector::Ones(2);
  auto fixed = [](std::string failure, Eigen::Index size) {
    SolverOptions o;
    o.backend = make_linear_solver_backend(
        "Fixed", [=](const SolverOptions &) {
          return std::make_unique<Fixed>(failure, size);
        });
    return o;
  };
  const auto wrong = solve_linear(a, b, fixed("", 2));
  check(!wrong.report.converged && wrong.report.iterations == 3 &&
            wrong.report.message.find("did not converge") != std::string::npos,
        "True residual check skipped for a user backend");
  const auto failed = solve_linear(a, b, fixed("backend sentinel", 2));
  check(!failed.report.converged &&
            failed.report.message == "backend sentinel",
        "User backend failure lost");
  expect_throws<std::runtime_error>(
      [&] { solve_linear(a, b, fixed("", 3)); }, "wrong size");
  expect_throws<std::invalid_argument>(
      [] { make_linear_solver_backend("None", nullptr); }, "needs a creator");
  SolverOptions empty;
  empty.backend = make_linear_solver_backend(
      "Empty", [](const SolverOptions &) {
        return std::unique_ptr<LinearSolverBackend>();
      });
  expect_throws<std::invalid_argument>([&] { solve_linear(a, b, empty); },
                                       "returned no backend");
}
} // namespace

int main() {
  return pde2d_test::run({{"preconditioners", preconditioners},
                          {"user_backend", user_backend},
                          {"backend_failures", backend_failures}});
}
