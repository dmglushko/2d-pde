// Typed plan settings (decision.hpp, plan.hpp, mesh_plan.hpp). Oracles: the
// option-based entry points (a setter must give the plan plan_from_user
// gives for the same options, and bit-identical results), the documented
// exception messages for stale and misplaced decisions, and decision texts
// known from the options.
#include "discretization/geometry.hpp"
#include "discretization/square_mesh.hpp"
#include "discretization/triangulate.hpp"
#include "fem/evolution.hpp"
#include "fem/plan.hpp"
#include "fem/record.hpp"
#include "fem/solver.hpp"
#include "support/check.hpp"
#include <memory>
#include <string>

using pde2d::discretization::square_mesh;
using pde2d_test::check;
using pde2d_test::expect_throws;
namespace {
using namespace pde2d::fem;
namespace disc = pde2d::discretization;

Problem poisson() {
  Problem p;
  p.equation.source = 1 + x * y;
  p.assign(dirichlet(x * x - y), {"bottom", "right", "top", "left"});
  return p;
}
bool same(const Decision &a, const Decision &b) {
  return a.subject == b.subject && a.choice == b.choice && a.basis == b.basis &&
         a.evidence == b.evidence && a.source == b.source;
}
bool same(const std::vector<Decision> &a, const std::vector<Decision> &b) {
  if (a.size() != b.size())
    return false;
  for (std::size_t k = 0; k < a.size(); ++k)
    if (!same(a[k], b[k]))
      return false;
  return true;
}

void subjects_are_typed() {
  for (const auto subject : decision_subjects) {
    const auto name = decision_subject_name(subject);
    check(decision_subject_from_name(name) == subject, "Name round trip");
    check(subject == name, "String comparison of the printed name");
  }
  check(!decision_subject_from_name("no such subject"), "Unknown name");
  check(user_decision("linear solver", "SparseLU", "SolverOptions::method")
                .subject == DecisionSubject::LinearSolver,
        "String subjects map to the enum");
  expect_throws<std::invalid_argument>(
      [] { user_decision("bogus", "x", "Options::x"); },
      "Unknown decision subject: bogus");
}

void setters_change_option_and_decision() {
  const auto mesh = square_mesh(6);
  const auto p = poisson();
  SolverOptions cg;
  cg.method = LinearSolver::ConjugateGradient;
  cg.preconditioner = Preconditioner::IncompleteCholesky;
  const auto by_options = plan_from_user({}, cg);
  auto by_setter = plan_from_user({}, {});
  by_setter.set_linear_solver(LinearSolver::ConjugateGradient)
      .set_preconditioner(Preconditioner::IncompleteCholesky);
  check(by_setter.solver == by_options.solver &&
            same(by_setter.decisions, by_options.decisions),
        "A setter gives the plan of the same options");
  check(solve_stationary(mesh, p, by_setter).values ==
            solve_stationary(mesh, p, by_options).values,
        "Bit-identical results");

  // A chooser's rationale.
  auto chosen = plan_from_user({}, {});
  chosen.set_linear_solver(LinearSolver::ConjugateGradient,
                           {"operator proven symmetric", Evidence::Proven,
                            DecisionSource::Automatic});
  const auto *d =
      find_decision(chosen.decisions, DecisionSubject::LinearSolver);
  check(d && d->choice == "ConjugateGradient" &&
            d->basis == "operator proven symmetric" &&
            d->evidence == Evidence::Proven &&
            d->source == DecisionSource::Automatic &&
            chosen.decisions.front().subject == DecisionSubject::LinearSolver,
        "Rationale recorded, position kept");
  // A new subject is appended.
  chosen.set_quadrature_degree(4);
  check(chosen.assembly.quadrature_degree == 4 &&
            chosen.decisions.back().subject == DecisionSubject::Quadrature &&
            chosen.decisions.back().choice == "4" &&
            chosen.decisions.back().basis ==
                "user-selected (AssemblyOptions::quadrature_degree)",
        "Quadrature decision");
  check(solve_stationary(mesh, p, chosen).report.decisions.size() == 4,
        "Accepted by the solve");

  // A backend with a descriptor.
  auto backend = make_linear_solver_backend(
      "Wrapped",
      [](const SolverOptions &) -> std::unique_ptr<LinearSolverBackend> {
        throw std::invalid_argument("not created in this test");
      },
      pde2d::StrategyDescriptor{"Wrapped", {{"ordering", "COLAMD"}}});
  auto with_backend = plan_from_user({}, {});
  with_backend.set_linear_solver_backend(backend);
  check(with_backend.solver.backend == backend &&
            with_backend.decisions[0].choice == "Wrapped" &&
            with_backend.decisions[0].basis ==
                "user-selected (SolverOptions::backend)" &&
            backend->descriptor()->parameters[0].value == "COLAMD",
        "Backend setter");
  validate_decisions(with_backend);

  // Time settings.
  TimeOptions o;
  o.end = .05;
  o.step = .01;
  o.method = TimeMethod::CrankNicolson;
  Problem heat = poisson();
  heat.equation.m1 = 1;
  auto plan = plan_from_user(TimeOptions{});
  plan.set_time_method(TimeMethod::CrankNicolson).set_time_step(.01);
  plan.time->end = .05;
  check(same(plan.decisions, plan_from_user(o).decisions),
        "Time setters give the plan of the same options");
  const auto a = evolve(square_mesh(3), heat, {x * x - y}, plan);
  const auto b = evolve(square_mesh(3), heat, {x * x - y}, o);
  check(a.frames.back().values == b.frames.back().values, "Bit-identical");
  auto stationary = plan_from_user({}, {});
  expect_throws<std::invalid_argument>([&] { stationary.set_time_step(.1); },
                                       "Plan has no time settings");
}

void stale_decisions_throw() {
  const auto mesh = square_mesh(3);
  const auto p = poisson();
  auto plan = plan_from_user({}, {});
  plan.solver.method = LinearSolver::GMRES; // option without its decision
  expect_throws<std::invalid_argument>(
      [&] { solve_stationary(mesh, p, plan); },
      "Stale decision: linear solver records \"SparseLU\" but the options "
      "give \"GMRES\"");
  expect_throws<std::invalid_argument>(
      [&] { PreparedStationary(mesh, p, plan); }, "Stale decision");
  plan.set_linear_solver(LinearSolver::GMRES);
  validate_decisions(plan);

  auto duplicate = plan_from_user({}, {});
  duplicate.decisions.push_back(duplicate.decisions.front());
  expect_throws<std::invalid_argument>(
      [&] { solve_stationary(mesh, p, duplicate); },
      "Duplicate decision: linear solver");
  auto misplaced = plan_from_user({}, {});
  misplaced.decisions.push_back(user_decision(DecisionSubject::MeshSize, "fine",
                                              "MeshOptions::refinement"));
  expect_throws<std::invalid_argument>(
      [&] { solve_stationary(mesh, p, misplaced); },
      "does not belong to a solve plan");
  auto timeless = plan_from_user({}, {});
  timeless.decisions.push_back(time_decisions(TimePlan{})[0]);
  expect_throws<std::invalid_argument>([&] { validate_decisions(timeless); },
                                       "without time settings");

  Problem heat = p;
  heat.equation.m1 = 1;
  TimeOptions o;
  o.end = .03;
  auto transient = plan_from_user(o);
  transient.time->step = .015; // the decision still says .01
  expect_throws<std::invalid_argument>(
      [&] { evolve(mesh, heat, {x * x - y}, transient); },
      "Stale decision: time steps records \"Uniform(step=0.01)\"");
  // A factory decision must name what the factory creates.
  auto factory = plan_from_user(o);
  factory.set_time_integrator(
      [] { return std::make_unique<CrankNicolsonIntegrator>(); });
  check(factory.decisions[0].choice.empty(), "Unnamed before running");
  check(evolve(mesh, heat, {x * x - y}, factory)
                .record.plan.decisions[0]
                .choice == "CrankNicolson",
        "Named after running");
  factory.decisions[0].choice = "BackwardEuler";
  expect_throws<std::invalid_argument>(
      [&] { evolve(mesh, heat, {x * x - y}, factory); },
      "Stale decision: time integrator records \"BackwardEuler\" but the "
      "options give \"CrankNicolson\"");
  // A record's plan replays: its named choices match what the factories
  // create.
  factory.decisions[0].choice.clear();
  const auto first = evolve(mesh, heat, {x * x - y}, factory);
  const auto replay = evolve(mesh, heat, {x * x - y}, first.record.plan);
  check(replay.frames.back().values == first.frames.back().values,
        "Replay from a record's plan");
}

void mesh_plans() {
  disc::MeshOptions options;
  options.boundary_treatment = disc::BoundaryTreatment::RefineOnCurves;
  options.boundary.max_curve_deviation = .05;
  options.refinement.max_edge_length = .4;
  const auto mesh_plan = mesh_plan_from_user(options);
  check(mesh_plan.decisions.size() == 4, "Four mesh decisions");
  check(mesh_plan.decisions[0].subject == DecisionSubject::MeshAlgorithm &&
            mesh_plan.decisions[0].choice == "ConstrainedDelaunay" &&
            mesh_plan.decisions[1].choice ==
                "RefineOnCurves preserve_corners=false preserve_regions=true" &&
            mesh_plan.decisions[2].choice ==
                "boundary_max_edge_length=1 max_curve_deviation=0.05 "
                "refinement_max_edge_length=0.4 max_steps=10000" &&
            mesh_plan.decisions[3].choice == "none",
        "Mesh decision texts");
  validate_decisions(mesh_plan);
  auto edited = mesh_plan;
  edited.options.refinement.max_edge_length = .3;
  expect_throws<std::invalid_argument>([&] { validate_decisions(edited); },
                                       "Stale decision: mesh size");
  auto misplaced = mesh_plan;
  misplaced.decisions.push_back(user_decision(
      DecisionSubject::LinearSolver, "SparseLU", "SolverOptions::method"));
  expect_throws<std::invalid_argument>([&] { validate_decisions(misplaced); },
                                       "does not belong to a mesh plan");

  const auto result =
      disc::generate_mesh(disc::CircleGeometry(1.0), mesh_plan.options);
  Problem p;
  p.equation.source = 1;
  p.assign(dirichlet(0), {"boundary"});
  auto plan = plan_from_user({}, {});
  plan.mesh = mesh_plan;
  const auto s = solve_stationary(result, p, plan);
  check(s.record.plan.mesh && s.record.plan.mesh->decisions.size() == 4,
        "The record carries the mesh plan");
  // A mesh plan that does not describe the mesh is rejected.
  auto other = options;
  other.refinement.max_edge_length = .3;
  plan.mesh = mesh_plan_from_user(other);
  expect_throws<std::invalid_argument>(
      [&] { solve_stationary(result, p, plan); }, "Stale decision: mesh size");
  // A stale mesh plan is rejected even on a bare mesh.
  plan.mesh = edited;
  expect_throws<std::invalid_argument>(
      [&] { solve_stationary(result.mesh, p, plan); }, "Stale decision");
  // A whole-mesh decision has no single option to compare.
  plan.mesh = mesh_plan;
  plan.mesh->decisions.push_back({DecisionSubject::Mesh, "graded",
                                  "estimated error", Evidence::Sampled,
                                  DecisionSource::Automatic});
  check(solve_stationary(result, p, plan).values == s.values,
        "Whole-mesh decision accepted");
}
} // namespace
int main() {
  return pde2d_test::run({{"subjects_are_typed", subjects_are_typed},
                          {"setters_change_option_and_decision",
                           setters_change_option_and_decision},
                          {"stale_decisions_throw", stale_decisions_throw},
                          {"mesh_plans", mesh_plans}});
}
