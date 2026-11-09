// PreparedStationary and initial guesses. Oracles: the one-shot pipeline
// driven through the same public components without any cache (assemble,
// constrain, solve_linear), bit for bit, and the converged solution of an
// unguessed solve.
#include "fem/report.hpp"
#include "fem/solver.hpp"
#include "discretization/square_mesh.hpp"
#include "support/check.hpp"
#include "support/check_eigen.hpp"
#include <sstream>

using pde2d::discretization::square_mesh;
using pde2d_test::check;
using pde2d_test::expect_throws;
namespace {
using namespace pde2d::fem;
const std::initializer_list<std::string> sides{"bottom", "right", "top",
                                               "left"};

Problem poisson() {
  Problem p;
  p.equation.source = 1 + x * y;
  p.assign(dirichlet(x * x - y), sides);
  return p;
}

// The stationary pipeline without PreparedStationary.
Vector uncached(const pde2d::discretization::TriangleMesh &mesh,
                const Problem &p, double time, const SolverOptions &solver) {
  const auto s = assemble(mesh, p, time);
  auto [a, b] = constrain(s.stiffness, s.load, s.dirichlet);
  const auto solution = solve_linear(a, b, solver);
  check(solution.report.converged, "Uncached reference failed");
  return solution.values;
}

void wrapper_is_the_uncached_pipeline() {
  const auto mesh = square_mesh(5);
  const auto p = poisson();
  const auto reference = uncached(mesh, p, 0, {});
  const auto wrapped = solve_stationary(mesh, p);
  PreparedStationary prepared(mesh, p);
  check(wrapped.values == reference && prepared.solve().values == reference,
        "Stationary entry points must agree bit for bit");
}

void repeated_solves_reuse_the_factorization() {
  const auto mesh = square_mesh(5);
  auto p = poisson();
  p.equation.source = 1 + sin(t) * x * y; // only the load depends on t
  PreparedStationary prepared(mesh, p);
  const std::vector<double> times{0, .5, 1, 2};
  for (double time : times) {
    const auto s = prepared.solve(time);
    check(s.values == uncached(mesh, p, time, {}),
          "Reuse changed the solution");
  }
  // Performance contract: the constrained matrix is unchanged, so it is
  // prepared once and reused by every later solve.
  const auto &st = prepared.statistics();
  check(st.solves == times.size() && st.assembly_calls == st.solves &&
            st.solver.preparations == 1 && st.solver_reuses == st.solves - 1 &&
            st.solver.rhs_solves == st.solves,
        "Factorization reuse");

  // A time-dependent matrix is prepared again for every new time.
  p.equation.c = 1 + t;
  PreparedStationary varying(mesh, p);
  for (double time : times)
    check(varying.solve(time).values == uncached(mesh, p, time, {}),
          "Time-dependent matrix");
  check(varying.statistics().solver.preparations == times.size() &&
            varying.statistics().solver_reuses == 0,
        "Changed matrices are not reused");
}

void initial_guesses() {
  const auto mesh = square_mesh(8);
  const auto p = poisson();
  SolverOptions cg;
  cg.method = LinearSolver::ConjugateGradient;
  const auto cold = solve_stationary(mesh, p, {}, cg);
  check(cold.report.initial_guess == InitialGuess::None, "No guess given");
  // Starting from the converged solution needs fewer iterations and the
  // result agrees within the solver tolerance.
  const auto warm = solve_stationary(mesh, p, {}, cg, cold.values);
  check(warm.report.initial_guess == InitialGuess::Used,
        "CG uses the initial guess");
  check(warm.report.iterations < cold.report.iterations,
        "A converged guess must save iterations");
  pde2d_test::near_vector(warm.values, cold.values, 1e-8);
  // GMRES: a rough guess still converges.
  SolverOptions gmres;
  gmres.method = LinearSolver::GMRES;
  const Vector rough = Vector::Constant(cold.values.size(), .3);
  const auto g = solve_stationary(mesh, p, {}, gmres, rough);
  check(g.report.initial_guess == InitialGuess::Used && g.report.converged,
        "GMRES uses the initial guess");
  pde2d_test::near_vector(g.values, cold.values, 1e-8);
  // A direct backend ignores the guess and says so; values are unchanged.
  const auto lu = solve_stationary(mesh, p);
  const auto lu_guess = solve_stationary(mesh, p, {}, {}, rough);
  check(lu_guess.report.initial_guess == InitialGuess::Ignored &&
            lu_guess.values == lu.values,
        "SparseLU ignores the guess");
  // The linear-algebra entry points take the same guess.
  const auto s = assemble(mesh, p);
  auto [a, b] = constrain(s.stiffness, s.load, s.dirichlet);
  const auto direct = solve_linear(a, b, cg, cold.values);
  check(direct.report.initial_guess == InitialGuess::Used,
        "solve_linear with a guess");
  PreparedLinearSolver prepared(a, cg);
  check(prepared.solve(b, cold.values).report.initial_guess ==
                InitialGuess::Used &&
            prepared.solve(b).report.initial_guess == InitialGuess::None,
        "PreparedLinearSolver guess");
  expect_throws<std::invalid_argument>(
      [&] { solve_stationary(mesh, p, {}, cg, Vector::Zero(2)); },
      "Invalid initial guess");
  Vector nonfinite = cold.values;
  nonfinite[0] = std::nan("");
  expect_throws<std::invalid_argument>([&] { prepared.solve(b, nonfinite); },
                                       "Invalid initial guess");
  std::ostringstream text;
  write_report(text, warm.report);
  check(text.str().find("initial_guess=Used") != std::string::npos,
        "Report prints the guess use");
}

} // namespace

int main() {
  return pde2d_test::run(
      {{"wrapper_is_the_uncached_pipeline", wrapper_is_the_uncached_pipeline},
       {"repeated_solves_reuse_the_factorization",
        repeated_solves_reuse_the_factorization},
       {"initial_guesses", initial_guesses}});
}
