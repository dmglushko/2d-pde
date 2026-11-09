// Transient adaptivity (adapt_evolution, adapt_evolution.hpp). Oracles:
//  - the loop composed by hand from the public components (evolve,
//    estimate_error, mark_bulk, refine_mesh, transfer_field and the
//    Dirichlet data of PreparedAssembly at the restart time) gives the same
//    meshes, values, estimates and records bit for bit;
//  - on the heat reference problem with a moving peak, adapting between
//    windows controls the error: it ends well below the error of the
//    unadapted coarse mesh, at about the accuracy of a uniform mesh with more
//    nodes, and the end-of-window estimates never exceed the first one;
//  - a second-order (wave) restart transfers the velocity as well and stays
//    accurate;
//  - solve and mesh planners run per window as in adapt(): the hand loop
//    with the planned solver and refinement options gives the same values;
//  - the history's JSON round-trips and invalid settings are rejected.
#include "discretization/mesh_fingerprint.hpp"
#include "discretization/triangulate.hpp"
#include "fem/adaptivity.hpp"
#include "fem/adaptivity_json.hpp"
#include "fem/assembly.hpp"
#include "fem/error_norms.hpp"
#include "fem/estimation.hpp"
#include "fem/evolution.hpp"
#include "fem/planner.hpp"
#include "fem/record_json.hpp"
#include "fem/reference/problems.hpp"
#include "fem/transfer.hpp"
#include "support/check.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace pde2d::fem;
namespace disc = pde2d::discretization;
using pde2d_test::check;
using pde2d_test::expect_throws;

namespace {
bool identical(const Vector &a, const Vector &b) {
  return a.size() == b.size() && (a.array() == b.array()).all();
}
disc::MeshResult coarse_mesh(const ReferenceProblem &r, double h) {
  auto o = r.mesh;
  o.boundary.max_edge_length = h;
  o.refinement.max_edge_length = h;
  o.boundary_treatment = disc::BoundaryTreatment::SplitPolygonEdges;
  return disc::generate_mesh(r.geometry, o);
}
SolvePlan time_plan(const ReferenceProblem &r, TimeMethod method, double step) {
  TimePlan time;
  time.method = method;
  time.start = r.transient->start;
  time.end = r.transient->end;
  time.step = step;
  return plan_from_user(AssemblyOptions{4}, SolverOptions{}, time);
}

void driver_matches_the_hand_composed_loop() {
  const auto heat = find_reference_problem("heat_moving_peak");
  const auto start = coarse_mesh(heat, .2);
  const auto plan = time_plan(heat, TimeMethod::CrankNicolson, .02);
  AdaptOptions options;
  options.windows = 3;
  const auto history = adapt_evolution(start, heat.problem,
                                       heat.transient->initial, plan, options);
  check(history.steps.size() == 3, "one entry per window");
  check(history.stop == AdaptiveStop::EndReached && !history.solution,
        "a transient history ends with its last window");

  // By hand.
  const auto stamps = adaptive_window_times(*plan.time, 3);
  check(stamps.front() == 0 && stamps.back() == heat.transient->end,
        "window stamps");
  auto mesh = start;
  auto initial = heat.transient->initial;
  auto window_plan = plan;
  for (std::size_t k = 0; k < 3; ++k) {
    window_plan.time->start = stamps[k];
    window_plan.time->end = stamps[k + 1];
    const auto e = evolve(mesh, heat.problem, initial, window_plan);
    const auto &last = e.frames.back();
    const auto estimate =
        estimate_error(mesh.mesh, heat.problem, last.values, stamps[k + 1]);
    const auto &w = history.steps[k];
    check(w.window_start == stamps[k] && w.time == stamps[k + 1],
          "window bounds");
    check(diff_records(w.record, e.record).empty(),
          "window " + std::to_string(k) + ": same run record");
    check(w.estimate.global == estimate.global, "same estimate");
    if (k == 2) {
      check(identical(history.evolution->frames.back().values, last.values),
            "same final values");
      check(disc::mesh_fingerprint(history.mesh.mesh) ==
                disc::mesh_fingerprint(mesh.mesh),
            "same final mesh");
      check(!w.marking && !w.adaptation, "no refinement after the last");
      break;
    }
    const auto marked = mark_bulk(estimate.indicators, 0.5);
    const auto refined = disc::refine_mesh(mesh, marked, mesh.options);
    check(refined.report.size_satisfied, refined.report.message);
    check(w.marking && w.marking->marked == marked.size() && w.adaptation &&
              w.adaptation->accepted &&
              w.adaptation->run.fingerprint ==
                  disc::mesh_fingerprint(refined.mesh),
          "same marking and refinement");
    auto moved = transfer_field(mesh.mesh, last.values, refined.mesh);
    const auto data =
        PreparedAssembly(refined.mesh, heat.problem, window_plan.assembly)
            .assemble(stamps[k + 1])
            .dirichlet;
    double correction = 0;
    for (const auto &d : data) {
      correction =
          std::max(correction, std::abs(moved.values[d.node] - d.value));
      moved.values[d.node] = d.value;
    }
    const auto &next = history.steps[k + 1];
    check(next.transfer && next.transfer->dirichlet_nodes == data.size() &&
              next.transfer->target_nodes == refined.mesh.nodes.size() &&
              next.transfer->max_dirichlet_correction == correction &&
              !next.velocity_transfer,
          "restart record");
    initial = InitialConditions{moved.values, std::nullopt};
    mesh = refined;
  }
  check(history.steps[1].record.initial->value.kind == "nodal",
        "restarts are nodal fields");
}

void heat_error_is_controlled() {
  const auto heat = find_reference_problem("heat_moving_peak");
  const auto plan = time_plan(heat, TimeMethod::CrankNicolson, .01);
  const auto end = heat.transient->end;
  const auto error = [&](const disc::MeshResult &m, const Vector &u) {
    return error_norms(m.mesh, u, heat.exact, end, 6).l2;
  };
  const auto coarse = coarse_mesh(heat, .2);
  const auto fixed =
      evolve(coarse, heat.problem, heat.transient->initial, plan);
  const double fixed_error = error(coarse, fixed.frames.back().values);

  AdaptOptions options;
  options.windows = 6;
  options.marker = std::make_shared<BulkMarking>(.6);
  const auto adapted = adapt_evolution(coarse, heat.problem,
                                       heat.transient->initial, plan, options);
  const double adapted_error =
      error(adapted.mesh, adapted.evolution->frames.back().values);

  // A uniform mesh of about the adapted accuracy.
  const auto uniform = coarse_mesh(heat, .1);
  const auto reference =
      evolve(uniform, heat.problem, heat.transient->initial, plan);
  const double uniform_error = error(uniform, reference.frames.back().values);
  std::cout << "fixed coarse: nodes " << coarse.mesh.nodes.size() << " L2 "
            << fixed_error << "; adapted: nodes "
            << adapted.mesh.mesh.nodes.size() << " L2 " << adapted_error
            << "; uniform h=0.1: nodes " << uniform.mesh.nodes.size() << " L2 "
            << uniform_error << '\n';
  for (const auto &w : adapted.steps)
    std::cout << "  window " << w.index << " [" << *w.window_start << ", "
              << w.time << "] nodes " << w.record.workload.nodes << " estimate "
              << w.estimate.global << '\n';
  check(adapted_error < .5 * fixed_error,
        "adapting between windows reduces the error");
  check(adapted.mesh.mesh.nodes.size() < uniform.mesh.nodes.size() &&
            adapted_error < 1.25 * uniform_error,
        "about the uniform mesh's accuracy with fewer nodes");
  // Every window refines the peak where it currently is: the estimate at
  // the end of a window never exceeds the first window's.
  for (const auto &w : adapted.steps)
    check(w.estimate.global <= adapted.steps.front().estimate.global,
          "the estimate stays controlled");

  // A tolerance above every estimate never refines.
  options.tolerance = 1e6;
  const auto still = adapt_evolution(coarse, heat.problem,
                                     heat.transient->initial, plan, options);
  check(disc::mesh_fingerprint(still.mesh.mesh) ==
                disc::mesh_fingerprint(coarse.mesh) &&
            !still.steps[0].marking,
        "nothing refined within tolerance");
  check(identical(still.evolution->frames.back().values,
                  fixed.frames.back().values) ||
            (still.evolution->frames.back().values - fixed.frames.back().values)
                    .lpNorm<Eigen::Infinity>() < 1e-9,
        "windowed restarts on one mesh continue the evolution");
}

void wave_restart_transfers_the_velocity() {
  const auto wave = find_reference_problem("wave_standing");
  const auto plan = time_plan(wave, TimeMethod::Newmark, .01);
  AdaptOptions options;
  options.windows = 2;
  const auto coarse = coarse_mesh(wave, .2);
  const auto h = adapt_evolution(coarse, wave.problem, wave.transient->initial,
                                 plan, options);
  const auto &restart = h.steps[1];
  check(restart.transfer && restart.velocity_transfer &&
            restart.velocity_transfer->target_nodes == h.mesh.mesh.nodes.size(),
        "the velocity is transferred");
  check(h.steps[1].record.initial->velocity &&
            h.steps[1].record.initial->velocity->kind == "nodal",
        "nodal velocity restart");
  const auto fixed =
      evolve(coarse, wave.problem, wave.transient->initial, plan);
  const double adapted_error =
                   error_norms(h.mesh.mesh, h.evolution->frames.back().values,
                               wave.exact, wave.transient->end, 6)
                       .l2,
               fixed_error =
                   error_norms(coarse.mesh, fixed.frames.back().values,
                               wave.exact, wave.transient->end, 6)
                       .l2;
  std::cout << "wave: fixed L2 " << fixed_error << ", adapted L2 "
            << adapted_error << '\n';
  check(adapted_error < fixed_error, "the refined second window helps");
}

const Rationale planned{"planned for the test", Evidence::Unknown,
                        DecisionSource::Automatic};
// Conjugate gradients from the second window on; records what it saw.
class SwitchToCg final : public SolvePlanner {
public:
  std::shared_ptr<std::vector<std::size_t>> previous =
      std::make_shared<std::vector<std::size_t>>();
  std::string name() const override { return "SwitchToCg"; }
  SolvePlan plan(const Problem &, const ProblemProperties &,
                 const SolvePlan &defaults,
                 const PlanningContext &context) const override {
    previous->push_back(context.previous ? context.previous->index + 1 : 0);
    auto plan = defaults;
    if (context.step >= 1 &&
        plan.solver.method != LinearSolver::ConjugateGradient)
      plan.set_linear_solver(LinearSolver::ConjugateGradient, planned);
    return plan;
  }
};
// Changes the mesh with a finer constant target.
class FinerTarget final : public MeshPlanner {
public:
  std::string name() const override { return "FinerTarget"; }
  std::optional<pde2d::StrategyDescriptor> descriptor() const override {
    return pde2d::StrategyDescriptor{"FinerTarget", {{"h", "0.15"}}};
  }
  MeshPlan plan(const Problem &, const ProblemProperties &,
                const MeshPlan &defaults,
                const PlanningContext &) const override {
    auto plan = defaults;
    plan.set_max_edge_length(.15, planned);
    return plan;
  }
};

void planners_run_per_window() {
  const auto heat = find_reference_problem("heat_moving_peak");
  const auto start = coarse_mesh(heat, .25);
  const auto plan = time_plan(heat, TimeMethod::BackwardEuler, .05);
  AdaptOptions options;
  options.windows = 3;
  auto solve_planner = std::make_shared<SwitchToCg>();
  options.planners.solve = solve_planner;
  options.planners.mesh = std::make_shared<FinerTarget>();
  const auto history = adapt_evolution(start, heat.problem,
                                       heat.transient->initial, plan, options);

  // By hand, with the plans the planners give.
  auto cg = plan;
  cg.set_linear_solver(LinearSolver::ConjugateGradient, planned);
  const auto stamps = adaptive_window_times(*plan.time, 3);
  auto mesh = start;
  auto initial = heat.transient->initial;
  Vector last_values;
  for (std::size_t k = 0; k < 3; ++k) {
    auto window_plan = k >= 1 ? cg : plan;
    window_plan.time->start = stamps[k];
    window_plan.time->end = stamps[k + 1];
    const auto e = evolve(mesh, heat.problem, initial, window_plan);
    last_values = e.frames.back().values;
    check(history.steps[k].record.mesh.fingerprint ==
              disc::mesh_fingerprint(mesh.mesh),
          "window " + std::to_string(k) + ": same mesh");
    if (k == 2)
      break;
    const auto estimate =
        estimate_error(mesh.mesh, heat.problem, last_values, stamps[k + 1]);
    auto finer = mesh.options;
    finer.refinement.max_edge_length = .15;
    const auto refined =
        disc::refine_mesh(mesh, mark_bulk(estimate.indicators, .5), finer);
    auto moved = transfer_field(mesh.mesh, last_values, refined.mesh);
    for (const auto &d :
         PreparedAssembly(refined.mesh, heat.problem, window_plan.assembly)
             .assemble(stamps[k + 1])
             .dirichlet)
      moved.values[d.node] = d.value;
    initial = InitialConditions{moved.values, std::nullopt};
    mesh = refined;
  }
  check(identical(history.evolution->frames.back().values, last_values),
        "planned windows reproduce the hand loop bit for bit");
  check(*solve_planner->previous == std::vector<std::size_t>{0, 1, 2},
        "the solve planner sees the previous window");
  for (std::size_t k = 0; k < 3; ++k) {
    const auto &step = history.steps[k];
    const auto *solver = find_decision(step.record.plan.decisions,
                                       DecisionSubject::LinearSolver);
    check(solver && (k == 0 ? solver->source == DecisionSource::User
                            : solver->source == DecisionSource::Automatic),
          "planned solver from the second window");
    check(step.decisions.front().subject == DecisionSubject::SolvePlanner,
          "solve planner decision first");
    check((k == 0) == !step.record.plan.mesh.has_value() &&
              (k == 0 || step.record.plan.mesh->planner->name == "FinerTarget"),
          "the mesh planner's plan is the next window's plan.mesh");
  }
  check(history.steps[0].decisions.size() == 4 &&
            history.steps[0].decisions[2].subject ==
                DecisionSubject::MeshPlanner &&
            history.steps[0].decisions[3].basis ==
                "planned (FinerTarget(h=0.15))",
        "decision order of a window");
}

void history_json_and_validation() {
  const auto heat = find_reference_problem("heat_moving_peak");
  const auto coarse = coarse_mesh(heat, .25);
  auto plan = time_plan(heat, TimeMethod::BackwardEuler, .05);
  AdaptOptions options;
  options.windows = 2;
  const auto h = adapt_evolution(coarse, heat.problem, heat.transient->initial,
                                 plan, options);
  std::ostringstream out;
  write_json(out, h);
  std::istringstream in(out.str());
  const auto doc = read_adaptive_history_json(in);
  check(doc.at("settings").at("windows").number() == 2 &&
            doc.at("settings").at("step").number() == .05 &&
            doc.at("kind").text == "transient" &&
            doc.at("steps").items.size() == 2 &&
            doc.at("steps").items[0].at("record").at("schema").text ==
                run_record_schema &&
            doc.at("steps").items[1].at("transfer").kind ==
                JsonValue::Kind::Object &&
            doc.at("final").at("time").number() == heat.transient->end,
        "history document");
  check(json_text(doc) == out.str(), "byte-identical re-serialization");
  check(diff_records(to_json(h), to_json(adapt_evolution(
                                     coarse, heat.problem,
                                     heat.transient->initial, plan, options)))
            .empty(),
        "identical runs: identical histories except timings");
  std::istringstream wrong("{\"schema\": \"pde2d.run_record\"}");
  expect_throws<std::invalid_argument>(
      [&] { (void)read_adaptive_history_json(wrong); },
      "Not an adaptive history");
  std::istringstream old("{\"schema\": \"pde2d.adaptive_history\", "
                         "\"schema_version\": 1}");
  expect_throws<std::invalid_argument>(
      [&] { (void)read_adaptive_history_json(old); },
      "Unsupported adaptive history schema version: 1");

  options.windows = 0;
  expect_throws<std::invalid_argument>(
      [&] {
        (void)adapt_evolution(coarse, heat.problem, heat.transient->initial,
                              plan, options);
      },
      "at least one window");
  options.windows = 2;
  expect_throws<std::invalid_argument>(
      [&] {
        (void)adapt_evolution(coarse, heat.problem, heat.transient->initial,
                              plan_from_user(AssemblyOptions{}), options);
      },
      "no time settings");
  auto controlled = plan;
  controlled.set_step_controller(
      [] { return std::make_unique<UniformStepController>(0, .6, .05); });
  expect_throws<std::invalid_argument>(
      [&] {
        (void)adapt_evolution(coarse, heat.problem, heat.transient->initial,
                              controlled, options);
      },
      "needs uniform time steps");
}
} // namespace

int main() {
  return pde2d_test::run(
      {{"driver_matches_the_hand_composed_loop",
        driver_matches_the_hand_composed_loop},
       {"heat_error_is_controlled", heat_error_is_controlled},
       {"wave_restart_transfers_the_velocity",
        wave_restart_transfers_the_velocity},
       {"planners_run_per_window", planners_run_per_window},
       {"history_json_and_validation", history_json_and_validation}});
}
