// PreparedStationary, solve plans, initial guesses and run records.
// Oracles: the one-shot pipeline driven through the same public components
// without any cache (assemble, constrain, solve_linear), bit for bit; the
// converged solution of an unguessed solve; option-based entry points
// against their plan_from_user equivalents; and records whose fields are
// known from the inputs.
#include "fem/evolution.hpp"
#include "fem/plan.hpp"
#include "fem/record.hpp"
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

void wrapper_and_plan_are_the_uncached_pipeline() {
  const auto mesh = square_mesh(5);
  const auto p = poisson();
  const auto reference = uncached(mesh, p, 0, {});
  const auto wrapped = solve_stationary(mesh, p);
  const auto planned = solve_stationary(mesh, p, plan_from_user({}, {}));
  PreparedStationary prepared(mesh, p);
  check(wrapped.values == reference && planned.values == reference &&
            prepared.solve().values == reference,
        "Stationary entry points must agree bit for bit");
  const auto expected = solver_decisions({});
  check(wrapped.report.decisions.size() == expected.size() &&
            planned.report.decisions.size() == expected.size(),
        "Decision count");
  for (std::size_t k = 0; k < expected.size(); ++k)
    check(wrapped.report.decisions[k].choice == expected[k].choice &&
              planned.report.decisions[k].basis == expected[k].basis,
          "Plan decisions are the user's solver decisions");
  // A LinearSolution still binds to the result (slicing keeps the values).
  const LinearSolution sliced = solve_stationary(mesh, p);
  check(sliced.values == reference, "LinearSolution compatibility");
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
    check(s.record.work.assembly_calls == 1 &&
              s.record.work.linear_solves == 1 &&
              s.record.work.solver_preparations + s.record.work.solver_reuses ==
                  1,
          "Per-solve work record");
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
  // GMRES through the plan overload; a rough guess still converges.
  SolverOptions gmres;
  gmres.method = LinearSolver::GMRES;
  const Vector rough = Vector::Constant(cold.values.size(), .3);
  const auto g = solve_stationary(mesh, p, plan_from_user({}, gmres), rough);
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

void evolution_plans() {
  const auto mesh = square_mesh(3);
  Problem p;
  p.equation.m1 = 1;
  p.equation.source = sin(t) * x;
  p.assign(dirichlet(0), sides);
  TimeOptions o;
  o.method = TimeMethod::CrankNicolson;
  o.end = .05;
  o.step = .01;
  const Expr u0 = x * (1 - x) * y * (1 - y);
  const auto by_options = evolve(mesh, p, {u0}, o);
  const auto plan = plan_from_user(o);
  const auto by_plan = evolve(mesh, p, {u0}, plan, output_options(o));
  check(by_plan.frames.size() == by_options.frames.size(), "Frame count");
  for (std::size_t k = 0; k < by_plan.frames.size(); ++k)
    check(by_plan.frames[k].values == by_options.frames[k].values,
          "Plan and options paths must agree bit for bit");
  const auto &d = by_plan.metadata.decisions;
  check(d.size() == 5 && d[0].subject == "time integrator" &&
            d[0].choice == "CrankNicolson" && d[1].subject == "time steps" &&
            d[1].choice == "Uniform(step=0.01)",
        "Plan decisions");
  for (std::size_t k = 0; k < d.size(); ++k)
    check(d[k].choice == by_options.metadata.decisions[k].choice &&
              d[k].basis == by_options.metadata.decisions[k].basis,
          "Decisions agree between the paths");
  // A factory-made strategy is named by what ran.
  auto custom = plan;
  custom.time->integrator = [] {
    return std::make_unique<CrankNicolsonIntegrator>();
  };
  custom.decisions =
      plan_from_user(custom.assembly, custom.solver, *custom.time).decisions;
  check(custom.decisions[0].choice.empty(), "Unnamed before running");
  check(evolve(mesh, p, {u0}, custom).metadata.decisions[0].choice ==
            "CrankNicolson",
        "Factory strategy named after running");
  auto stationary = plan_from_user({}, {});
  expect_throws<std::invalid_argument>(
      [&] { evolve(mesh, p, {u0}, stationary); },
      "Evolution plan has no time settings");
}

void run_records() {
  const auto mesh = square_mesh(3);
  const auto s = solve_stationary(mesh, poisson());
  const auto &r = s.record;
  check(r.workload.nodes == mesh.nodes.size() &&
            r.workload.elements == mesh.elements.size() &&
            r.workload.boundary_edges == mesh.boundary_edges.size(),
        "Workload");
  check(r.execution.openmp_available == pde2d::openmp_available() &&
            r.execution.selected.backend == pde2d::ExecutionBackend::Serial,
        "Execution");
  check(r.plan.solver == SolverOptions{} && !r.plan.time &&
            r.plan.assembly.quadrature_degree == 2,
        "Plan");
  check(r.work.assembly_calls == 1 && r.work.linear_solves == 1 &&
            r.work.solver_preparations == 1 && r.work.time_steps == 0,
        "Work");
  check(r.evidence.unknown == r.plan.decisions.size() &&
            r.evidence.proven == 0 && r.evidence.weakest == Evidence::Unknown,
        "Every user decision has unknown evidence");

  Problem heat = poisson();
  heat.equation.m1 = 1;
  TimeOptions o;
  o.end = .03;
  o.step = .01;
  const auto e = evolve(mesh, heat, {x * x - y}, o);
  const auto &er = e.record;
  check(er.plan.time && er.plan.time->step == o.step &&
            er.plan.time->end == o.end,
        "Evolution plan");
  check(er.work.time_steps == e.timings.time_steps &&
            er.work.linear_solves == e.timings.linear_solves &&
            er.work.assembly_calls == e.timings.assembly_calls &&
            er.work.solver_preparations == e.timings.solver.preparations &&
            er.work.solver_reuses == e.timings.solver_reuses,
        "Evolution work");
  check(er.plan.decisions.size() == e.metadata.decisions.size() &&
            er.plan.decisions[1].choice == e.metadata.decisions[1].choice,
        "Evolution decisions");

  const std::vector<Decision> mixed{
      {DecisionSubject::LinearSolver, "1", "", Evidence::Proven,
       DecisionSource::Automatic},
      {DecisionSubject::Preconditioner, "2", "", Evidence::Sampled,
       DecisionSource::Automatic},
      user_decision(DecisionSubject::Quadrature, "3", "Option")};
  const auto summary = summarize_evidence(mixed);
  check(summary.proven == 1 && summary.sampled == 1 && summary.unknown == 1 &&
            summary.weakest == Evidence::Unknown,
        "Evidence summary");
  check(summarize_evidence({}).weakest == Evidence::Proven,
        "No decisions: nothing unsupported");

  std::ostringstream text;
  write_report(text, s);
  write_report(text, e);
  write_report(text, er);
  for (const char *token :
       {"record execution: selected_backend=Serial", "openmp_available=",
        "record workload: nodes=16", "record plan: quadrature_degree=2",
        "solver=SparseLU", "record work: assembly_calls=1",
        "record evidence: proven=0", "weakest=Unknown", "record time: method",
        "step_controller=uniform", "FEM run record:", "rejected_steps=0"})
    check(text.str().find(token) != std::string::npos,
          "Record report missing field");
}
} // namespace

int main() {
  return pde2d_test::run({{"wrapper_and_plan_are_the_uncached_pipeline",
                           wrapper_and_plan_are_the_uncached_pipeline},
                          {"repeated_solves_reuse_the_factorization",
                           repeated_solves_reuse_the_factorization},
                          {"initial_guesses", initial_guesses},
                          {"evolution_plans", evolution_plans},
                          {"run_records", run_records}});
}
