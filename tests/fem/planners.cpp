// Planner seams (planner.hpp) and how adapt() runs them (adaptivity.hpp).
// No planner here chooses anything clever: a test planner switches the
// linear solver after iteration 2, another changes the refinement target.
// Oracles: the same loop composed by hand from the public components with
// the plans switched at the same iterations (meshes and values bit for bit),
// the identity planner (results bit-identical to no planner), and the
// documented messages for invalid planner output. plan_mesh plans before the
// first mesh exists: its mesh is generate_mesh with the planned options.
#include "discretization/geometry.hpp"
#include "discretization/mesh_fingerprint.hpp"
#include "discretization/triangulate.hpp"
#include "fem/adaptivity.hpp"
#include "fem/estimation.hpp"
#include "fem/marking.hpp"
#include "fem/planner.hpp"
#include "fem/record_json.hpp"
#include "fem/solver.hpp"
#include "fem/transfer.hpp"
#include "support/check.hpp"
#include <memory>
#include <sstream>
#include <string>
#include <vector>

using namespace pde2d::fem;
using namespace pde2d::discretization;
using pde2d_test::check;
using pde2d_test::expect_throws;

namespace {
const Expr exact = exp(-30 * ((x - .3) * (x - .3) + (y + .2) * (y + .2)));
Problem peak_problem() {
  Problem p;
  p.equation.source = -(diff(diff(exact, x), x) + diff(diff(exact, y), y));
  p.assign(dirichlet(exact), {"boundary"});
  return p;
}
MeshOptions disk_options() {
  MeshOptions options;
  options.boundary_treatment = BoundaryTreatment::RefineOnCurves;
  options.boundary.max_curve_deviation = .02;
  options.refinement.max_edge_length = .35;
  return options;
}
MeshResult disk() {
  auto mesh = generate_mesh(CircleGeometry(1.0), disk_options());
  check(mesh.report.size_satisfied, mesh.report.message);
  return mesh;
}
bool identical(const Vector &a, const Vector &b) {
  return a.size() == b.size() && (a.array() == b.array()).all();
}
const Rationale switched{"switched after iteration 2", Evidence::Unknown,
                         DecisionSource::Automatic};

// What the test planner saw when it was called.
struct Seen {
  std::size_t iteration = 0;
  std::string mesh_fingerprint;
  std::optional<std::size_t> previous_index;
  std::optional<double> previous_estimate;
};
// Switches to conjugate gradients from iteration 3 on.
class SwitchAfterTwo final : public SolvePlanner {
public:
  std::shared_ptr<std::vector<Seen>> seen =
      std::make_shared<std::vector<Seen>>();
  std::string name() const override { return "SwitchAfterTwo"; }
  std::optional<pde2d::StrategyDescriptor> descriptor() const override {
    return pde2d::StrategyDescriptor{"SwitchAfter", {{"iteration", "2"}}};
  }
  SolvePlan plan(const Problem &, const ProblemProperties &,
                 const SolvePlan &defaults,
                 const PlanningContext &context) const override {
    Seen s;
    s.iteration = context.step;
    if (context.mesh)
      s.mesh_fingerprint = mesh_fingerprint(context.mesh->mesh);
    if (context.previous) {
      s.previous_index = context.previous->index;
      s.previous_estimate = context.previous->estimate.global;
    }
    seen->push_back(s);
    auto plan = defaults;
    if (context.step > 2 &&
        plan.solver.method != LinearSolver::ConjugateGradient)
      plan.set_linear_solver(LinearSolver::ConjugateGradient, switched);
    return plan;
  }
};

AdaptOptions loop_options(std::size_t iterations) {
  AdaptOptions o;
  o.max_iterations = iterations;
  o.transfer.max_outside_distance = .02;
  return o;
}

void solve_planner_switches_solver() {
  const auto p = peak_problem();
  const auto initial = disk();
  const auto plan = plan_from_user({}, {});
  auto options = loop_options(5);
  auto planner = std::make_shared<SwitchAfterTwo>();
  options.planners.solve = planner;
  const auto history = adapt(initial, p, plan, options);
  check(history.steps.size() == 6, "Six solves");

  // The oracle: the loop by hand, with the plan the planner gives.
  auto cg = plan;
  cg.set_linear_solver(LinearSolver::ConjugateGradient, switched);
  auto mesh = initial;
  auto u = solve_stationary(mesh, p, plan).values;
  std::vector<std::string> fingerprints{mesh_fingerprint(mesh.mesh)};
  for (std::size_t k = 1; k <= 5; ++k) {
    const auto estimate = estimate_error(mesh.mesh, p, u);
    const auto marked = mark_bulk(estimate.indicators, .5);
    auto refined = refine_mesh(mesh, marked, mesh.options);
    const auto guess =
        transfer_field(mesh.mesh, u, refined.mesh, options.transfer);
    u = PreparedStationary(refined, p, k > 2 ? cg : plan)
            .solve(0, guess.values)
            .values;
    mesh = std::move(refined);
    fingerprints.push_back(mesh_fingerprint(mesh.mesh));
  }
  check(identical(history.solution->values, u),
        "The driver with a planner reproduces the hand loop bit for bit");
  for (std::size_t k = 0; k < history.steps.size(); ++k) {
    const auto &it = history.steps[k];
    check(it.record.mesh.fingerprint == fingerprints[k], "Same meshes");
    const auto *solver =
        find_decision(it.record.plan.decisions, DecisionSubject::LinearSolver);
    if (k <= 2)
      check(solver->choice == "SparseLU" &&
                solver->source == DecisionSource::User,
            "User solver up to iteration 2");
    else
      check(solver->choice == "ConjugateGradient" &&
                solver->source == DecisionSource::Automatic &&
                solver->basis == switched.basis &&
                it.record.stationary_solve->initial_guess == InitialGuess::Used,
            "Planned solver from iteration 3");
    // Recorded per iteration: the decision and the record's strategy.
    check(!it.decisions.empty() &&
              it.decisions.front().subject == DecisionSubject::SolvePlanner &&
              it.decisions.front().choice == "SwitchAfter(iteration=2)",
          "Planner decision of the iteration");
    const auto *strategy =
        find_strategy(it.record, DecisionSubject::SolvePlanner);
    check(strategy && strategy->name == "SwitchAfterTwo" &&
              strategy->replay == StrategyReplay::Described &&
              it.record.plan.planner &&
              it.record.plan.planner->name == "SwitchAfterTwo",
          "Planner strategy in the run record");
  }
  // What the planner saw: the iteration's mesh and the previous iteration.
  check(planner->seen->size() == 6, "Called before every solve");
  for (std::size_t k = 0; k < planner->seen->size(); ++k) {
    const auto &s = (*planner->seen)[k];
    check(s.iteration == k && s.mesh_fingerprint == fingerprints[k],
          "Current mesh");
    check(k == 0
              ? !s.previous_index
              : s.previous_index == k - 1 &&
                    s.previous_estimate == history.steps[k - 1].estimate.global,
          "Previous iteration's diagnostics");
  }
  // JSON names the planner.
  std::ostringstream json;
  write_json(json, history.steps.back().record);
  check(json.str().find("\"subject\": \"solve planner\"") !=
                std::string::npos &&
            json.str().find("\"planner\": {") != std::string::npos,
        "Planner in the record JSON");
}

void identity_planner_changes_nothing() {
  const auto p = peak_problem();
  const auto initial = disk();
  const auto plan = plan_from_user({}, {});
  const auto plain = adapt(initial, p, plan, loop_options(2));
  auto options = loop_options(2);
  options.planners.solve = std::make_shared<UserPlanner>();
  options.planners.mesh = std::make_shared<UserPlanner>();
  const auto planned = adapt(initial, p, plan, options);
  check(identical(plain.solution->values, planned.solution->values) &&
            plain.steps.size() == planned.steps.size(),
        "Identity planners are bit-identical");
  const auto &record = planned.steps.back().record;
  check(record.replayable() &&
            find_strategy(record, DecisionSubject::SolvePlanner)->replay ==
                StrategyReplay::BuiltIn &&
            find_strategy(record, DecisionSubject::MeshPlanner),
        "The identity planner is built in");
  // The mesh planner's (unchanged) plan becomes the next plan.mesh.
  check(
      record.plan.mesh && record.plan.mesh->planner &&
          !mesh_options_difference(record.plan.mesh->options, initial.options),
      "Mesh plan of the refined mesh");
  check(planned.steps[0].decisions.size() == 4 &&
            planned.steps[0].decisions[1].subject == DecisionSubject::Marking &&
            planned.steps[0].decisions[2].subject ==
                DecisionSubject::MeshPlanner &&
            planned.steps[0].decisions[3].subject ==
                DecisionSubject::Refinement,
        "Decision order of an iteration");
}

// Refines with a finer constant target from the second refinement on.
class FinerTarget final : public MeshPlanner {
public:
  std::string name() const override { return "FinerTarget"; }
  std::optional<pde2d::StrategyDescriptor> descriptor() const override {
    return pde2d::StrategyDescriptor{"FinerTarget", {{"h", "0.2"}}};
  }
  MeshPlan plan(const Problem &, const ProblemProperties &,
                const MeshPlan &defaults,
                const PlanningContext &context) const override {
    auto plan = defaults;
    if (context.step >= 2)
      plan.set_max_edge_length(.2, {"finer from iteration 2", Evidence::Unknown,
                                    DecisionSource::Automatic});
    return plan;
  }
};

void mesh_planner_sets_refinement() {
  const auto p = peak_problem();
  const auto initial = disk();
  auto plan = plan_from_user({}, {});
  plan.mesh = mesh_plan_from_user(initial.options);
  auto options = loop_options(2);
  options.planners.mesh = std::make_shared<FinerTarget>();
  const auto history = adapt(initial, p, plan, options);

  // The oracle: refinement with the options the planner gives.
  auto finer = initial.options;
  finer.refinement.max_edge_length = .2;
  auto mesh = initial;
  auto u = solve_stationary(mesh, p, plan).values;
  for (std::size_t k = 1; k <= 2; ++k) {
    const auto estimate = estimate_error(mesh.mesh, p, u);
    auto refined = refine_mesh(mesh, mark_bulk(estimate.indicators, .5),
                               k >= 2 ? finer : initial.options);
    const auto guess =
        transfer_field(mesh.mesh, u, refined.mesh, options.transfer);
    u = solve_stationary(refined, p, plan_from_user({}, {}), guess.values)
            .values;
    mesh = std::move(refined);
  }
  check(identical(history.solution->values, u) &&
            history.steps.back().record.mesh.fingerprint ==
                mesh_fingerprint(mesh.mesh),
        "Planned refinement reproduces the hand loop");
  const auto &last = history.steps.back().record.plan;
  const auto *size =
      find_decision(last.mesh->decisions, DecisionSubject::MeshSize);
  check(last.mesh->options.refinement.max_edge_length == .2 && size &&
            size->source == DecisionSource::Automatic &&
            last.mesh->planner->name == "FinerTarget",
        "The planner's mesh plan is the next plan.mesh");
  const auto &refinement = history.steps[1].decisions.back();
  check(refinement.subject == DecisionSubject::Refinement &&
            refinement.source == DecisionSource::Automatic &&
            refinement.basis == "planned (FinerTarget(h=0.2))",
        "Refinement decision names the planner");
}

// AdaptOptions::refinement that differs from plan.mesh: the mesh plan of
// the refined meshes is replaced explicitly, not left to conflict.
void refinement_options_replace_the_mesh_plan() {
  const auto p = peak_problem();
  const auto initial = disk();
  auto plan = plan_from_user({}, {});
  plan.mesh = mesh_plan_from_user(initial.options);
  auto options = loop_options(1);
  auto other = initial.options;
  other.refinement.max_edge_length = .25;
  options.refinement = other;
  const auto history = adapt(initial, p, plan, options);
  const auto &first = history.steps[0].record.plan;
  const auto &second = history.steps[1].record.plan;
  check(!mesh_options_difference(first.mesh->options, initial.options) &&
            mesh_options_difference(second.mesh->options, initial.options) ==
                DecisionSubject::MeshSize &&
            !mesh_options_difference(second.mesh->options, other),
        "The refined mesh's plan describes the refinement options");
  validate_decisions(*second.mesh, history.mesh.options);
}

class Stale final : public SolvePlanner {
public:
  std::string name() const override { return "Stale"; }
  SolvePlan plan(const Problem &, const ProblemProperties &,
                 const SolvePlan &defaults,
                 const PlanningContext &) const override {
    auto plan = defaults;
    plan.solver.method = LinearSolver::GMRES; // without its decision
    return plan;
  }
};
class Timed final : public SolvePlanner {
public:
  std::string name() const override { return "Timed"; }
  SolvePlan plan(const Problem &, const ProblemProperties &,
                 const SolvePlan &defaults,
                 const PlanningContext &) const override {
    auto plan = defaults;
    plan.time = TimePlan{};
    return plan;
  }
};

void invalid_plans_are_rejected() {
  const auto p = peak_problem();
  const auto initial = disk();
  auto options = loop_options(1);
  options.planners.solve = std::make_shared<Stale>();
  expect_throws<std::invalid_argument>(
      [&] { (void)adapt(initial, p, plan_from_user({}, {}), options); },
      "Solve planner Stale returned an invalid plan: Stale decision: linear "
      "solver");
  const auto properties = analyze(p);
  expect_throws<std::invalid_argument>(
      [&] {
        (void)run_planner(Timed{}, p, properties, plan_from_user({}, {}));
      },
      "Solve planner Timed returned an invalid plan: a planner may not add "
      "or remove time settings");
  // Outside adapt(): the plan carries its planner into the record.
  const auto planned =
      run_planner(UserPlanner{}, p, properties, plan_from_user({}, {}));
  const auto solution = solve_stationary(initial, p, planned);
  check(find_strategy(solution.record, DecisionSubject::SolvePlanner) &&
            solution.record.replayable(),
        "Planner recorded outside adapt()");
}

// Plans from the geometry alone: a target from the sampled boundary.
class FromBoundary final : public MeshPlanner {
public:
  std::shared_ptr<std::size_t> boundary_points =
      std::make_shared<std::size_t>();
  std::string name() const override { return "FromBoundary"; }
  std::optional<pde2d::StrategyDescriptor> descriptor() const override {
    return pde2d::StrategyDescriptor{"FromBoundary"};
  }
  MeshPlan plan(const Problem &, const ProblemProperties &,
                const MeshPlan &defaults,
                const PlanningContext &context) const override {
    if (!context.geometry || !context.boundary || context.mesh ||
        context.previous)
      throw std::logic_error("plan_mesh plans from the geometry");
    *boundary_points = context.boundary->boundary.points.size();
    auto plan = defaults;
    plan.set_max_edge_length(.25,
                             {"from the sampled boundary", Evidence::Unknown,
                              DecisionSource::Automatic});
    return plan;
  }
};

void plan_mesh_before_the_first_mesh() {
  const auto p = peak_problem();
  const CircleGeometry geometry(1.0);
  const auto defaults = mesh_plan_from_user(disk_options());
  // The identity: the plain generate_mesh of the defaults.
  const auto same = plan_mesh(geometry, p, UserPlanner{}, defaults);
  check(
      mesh_fingerprint(same.result.mesh) ==
              mesh_fingerprint(generate_mesh(geometry, disk_options()).mesh) &&
          same.plan.planner &&
          same.plan.planner->replay == StrategyReplay::BuiltIn &&
          !mesh_options_difference(same.result.options, disk_options()),
      "UserPlanner meshes the defaults");
  // A planner that reads the geometry and its sampled boundary.
  FromBoundary planner;
  const auto planned = plan_mesh(geometry, p, planner, defaults);
  auto expected_options = disk_options();
  expected_options.refinement.max_edge_length = .25;
  check(
      *planner.boundary_points > 0 &&
          mesh_fingerprint(planned.result.mesh) ==
              mesh_fingerprint(generate_mesh(geometry, expected_options).mesh),
      "the planned options mesh the geometry");
  auto plan = plan_from_user({}, {});
  plan.mesh = planned.plan;
  const auto solution = solve_stationary(planned.result, p, plan);
  const auto *strategy =
      find_strategy(solution.record, DecisionSubject::MeshPlanner);
  const auto *size = find_decision(solution.record.plan.mesh->decisions,
                                   DecisionSubject::MeshSize);
  check(strategy && strategy->name == "FromBoundary" &&
            strategy->replay == StrategyReplay::Described && size &&
            size->source == DecisionSource::Automatic,
        "the planned mesh's plan travels into the run record");
}
} // namespace
int main() {
  return pde2d_test::run(
      {{"solve_planner_switches_solver", solve_planner_switches_solver},
       {"identity_planner_changes_nothing", identity_planner_changes_nothing},
       {"mesh_planner_sets_refinement", mesh_planner_sets_refinement},
       {"refinement_options_replace_the_mesh_plan",
        refinement_options_replace_the_mesh_plan},
       {"invalid_plans_are_rejected", invalid_plans_are_rejected},
       {"plan_mesh_before_the_first_mesh", plan_mesh_before_the_first_mesh}});
}
