#pragma once
// Adaptive loop for a stationary problem, as one optional composition of
// public components:
//
//   solve ─▶ estimate ─▶ mark ─▶ refine ─▶ transfer ─▶ solve (warm start) ─▶ …
//   PreparedStationary  estimate_error  MarkingStrategy  refine_mesh
//                                       (marking.hpp)    (marked)
//                                                        transfer_field
//
//   AdaptOptions options;
//   options.marker = std::make_shared<BulkMarking>(0.6);
//   options.max_iterations = 8;          // refinements after the first solve
//   options.transfer.max_outside_distance = 0.02; // curved boundary
//   auto history = adapt(mesh_result, problem, plan, options);
//   history.solution.values;             // on history.mesh
//   write_json(out, history);            // adaptivity_json.hpp
//
// The driver adds nothing the components do not do: the same calls in the
// same order give the same meshes and values bit for bit (see
// tests/fem/adaptive_loop.cpp, whose oracle is the hand-composed loop). Each
// iteration's AdaptiveIteration links the RunRecord of its solve (mesh
// fingerprint and provenance included), the transfer that seeded it, a
// summary of its ErrorEstimate and, unless it was the last, what was marked
// and the refinement report of the next mesh, with the marking and
// refinement recorded as Decisions. AdaptiveHistory::stop says why the loop
// ended.
//
// Plans across iterations. The plan of iteration k is the user's plan, or
// for k > 0 the plan of iteration k - 1 updated for the new mesh:
//  - plan.analysis made on a mesh is recomputed on the new mesh with its own
//    options (refresh_analysis); a mesh-free analysis stays;
//  - plan.mesh, when set, describes the options that refined the mesh: the
//    mesh planner's MeshPlan, or, when the refinement options (AdaptOptions::
//    refinement) differ from it, mesh_plan_from_user(refinement options);
//    unchanged options keep it as is. A plan without a mesh plan gets one
//    only from a mesh planner.
//  - then the solve planner, if any, plans the solve (run_planner).
// The first iteration's plan must match its inputs: a stale analysis or a
// mesh plan that does not describe the mesh throws as in PreparedStationary.
//
// Stopping, checked after every solve and estimate in this order:
// estimate.global <= tolerance (ToleranceReached), max_iterations
// refinements done (MaxIterations), the strategy marked nothing
// (NothingMarked), the refined mesh missed its size target and
// accept_unsatisfied_refinement is false (UnsatisfiedRefinement; that mesh
// is not solved on).
#include "discretization/mesh_fingerprint.hpp"
#include "discretization/mesh_types.hpp"
#include "discretization/triangulate.hpp"
#include "fem/decision.hpp"
#include "fem/estimation.hpp"
#include "fem/assembly.hpp"
#include "fem/evolution.hpp"
#include "fem/marking.hpp"
#include "fem/analysis.hpp"
#include "fem/mesh_acceptance.hpp"
#include "fem/mesh_plan.hpp"
#include "fem/plan.hpp"
#include "fem/planner.hpp"
#include "fem/record.hpp"
#include "fem/solver.hpp"
#include "fem/transfer.hpp"
#include "provenance.hpp"
#include <cmath>
#include <cstddef>
#include <format>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace pde2d::fem {
struct AdaptOptions {
  // Estimator of every iteration; unset: GradientRecoveryEstimator with the
  // plan's execution policy.
  std::shared_ptr<const ErrorEstimator> estimator{};
  // Marking strategy; unset: BulkMarking{} (Dorfler, fraction 0.5).
  std::shared_ptr<const MarkingStrategy> marker{};
  // Largest number of refine-and-solve iterations after the first solve.
  std::size_t max_iterations = 10;
  // Stop once the global estimate is at most this; 0 never stops early.
  double tolerance = 0;
  // Options of refine_mesh(result, marked, options); unset: each result's
  // own options (MeshResult::options).
  std::optional<discretization::MeshOptions> refinement{};
  // Seed each solve with the previous solution transferred to the new mesh
  // (transfer_field; used by iterative solvers, ignored by direct ones).
  bool warm_start = true;
  TransferOptions transfer{};
  // Time at which the stationary problem is solved and estimated.
  double time = 0;
  // Keep going on a refined mesh that missed its size target (for example
  // an unrefinable marked triangle under a Fixed boundary); otherwise the
  // loop stops with UnsatisfiedRefinement.
  bool accept_unsatisfied_refinement = false;
  // Planner of the solve settings (planner.hpp), run before every solve with
  // the iteration's mesh and the previous iteration; unset: the plan stays
  // as given. Its plan (checked by run_planner) is the plan of that solve
  // and the defaults of the next call.
  std::shared_ptr<const SolvePlanner> solve_planner{};
  // Planner of the refinement options, run after marking with the current
  // mesh and the iteration just estimated. Its MeshPlan's options refine the
  // mesh (in place of `refinement`) and become plan.mesh of the next solve.
  std::shared_ptr<const MeshPlanner> mesh_planner{};
};

enum class AdaptiveStop {
  ToleranceReached,
  MaxIterations,
  NothingMarked,
  UnsatisfiedRefinement
};
// Stable names (adaptive history JSON).
inline const char *adaptive_stop_name(AdaptiveStop stop) {
  switch (stop) {
  case AdaptiveStop::ToleranceReached:
    return "tolerance reached";
  case AdaptiveStop::MaxIterations:
    return "max iterations";
  case AdaptiveStop::NothingMarked:
    return "nothing marked";
  case AdaptiveStop::UnsatisfiedRefinement:
    return "unsatisfied refinement";
  }
  return "invalid";
}

// What an iteration's ErrorEstimate said, without the indicators.
struct EstimateSummary {
  std::string estimator{}; // ErrorEstimator::name()
  std::optional<pde2d::StrategyDescriptor> descriptor{};
  double global = 0;
  std::size_t elements = 0;
  // Largest indicator and the mean (sum in element order / elements).
  double max_indicator = 0, mean_indicator = 0;
};
inline EstimateSummary summarize_estimate(const ErrorEstimate &estimate,
                                          const ErrorEstimator &estimator) {
  EstimateSummary s{estimator.name(), estimator.descriptor(), estimate.global,
                    estimate.indicators.size()};
  double sum = 0;
  for (const double eta : estimate.indicators) {
    s.max_indicator = std::max(s.max_indicator, eta);
    sum += eta;
  }
  if (s.elements > 0)
    s.mean_indicator = sum / static_cast<double>(s.elements);
  return s;
}
// Which elements an iteration marked, and by what strategy.
struct MarkingRecord {
  std::string strategy{}; // MarkingStrategy::name()
  std::optional<pde2d::StrategyDescriptor> descriptor{};
  std::size_t marked = 0, elements = 0;
};
// How the start of a solve came from the previous mesh (transfer_field).
struct TransferRecord {
  std::size_t source_nodes = 0, target_nodes = 0;
  std::size_t extrapolated_nodes = 0;
  double max_outside_distance = 0;
};

struct AdaptiveIteration {
  std::size_t index = 0;
  // The solve on this iteration's mesh: plan, mesh fingerprint and
  // provenance (options, report, marked-refinement counters), work.
  RunRecord record{};
  int solver_iterations = 0;
  InitialGuess initial_guess = InitialGuess::None;
  // The transfer that seeded this solve; empty on the first iteration and
  // without warm_start.
  std::optional<TransferRecord> transfer{};
  EstimateSummary estimate{};
  // Unless the loop stopped before marking: what was marked, and (if
  // anything was) the report of the refined mesh.
  std::optional<MarkingRecord> marking{};
  std::optional<discretization::MeshReport> refinement{};
  // The options refine_mesh was called with (the requested refinement
  // options or the mesh planner's), kept even when the refined mesh missed
  // its size target and was never solved on; and the refined result's mesh
  // run record (discretization::MeshRunRecord: fingerprints, work, quality).
  std::optional<discretization::MeshOptions> refinement_options{};
  std::optional<discretization::MeshRunRecord> refinement_run{};
  // The choices of this iteration, in this order: the solve planner that
  // planned its solve (DecisionSubject::SolvePlanner, only with
  // AdaptOptions::solve_planner), the marking (Marking), the mesh planner
  // (MeshPlanner, only with AdaptOptions::mesh_planner) and the refinement
  // (Refinement).
  std::vector<Decision> decisions{};
};

// The settings an adaptive loop ran with (AdaptOptions without its strategy
// objects, which are named by name() and descriptor()), so a history says
// how it was produced.
struct AdaptiveSettings {
  std::string estimator{}; // ErrorEstimator::name()
  std::optional<pde2d::StrategyDescriptor> estimator_descriptor{};
  bool default_estimator = true;
  std::string marker{}; // MarkingStrategy::name()
  std::optional<pde2d::StrategyDescriptor> marker_descriptor{};
  bool default_marker = true;
  std::size_t max_iterations = 0;
  double tolerance = 0;
  // AdaptOptions::refinement as requested; unset: each mesh's own options.
  std::optional<discretization::MeshOptions> refinement{};
  bool warm_start = true;
  TransferOptions transfer{};
  double time = 0;
  bool accept_unsatisfied_refinement = false;
  // name() of the planners, when set.
  std::optional<std::string> solve_planner{}, mesh_planner{};
};

struct AdaptiveHistory {
  // How the loop was configured (AdaptOptions).
  AdaptiveSettings settings{};
  std::vector<AdaptiveIteration> iterations{};
  AdaptiveStop stop = AdaptiveStop::MaxIterations;
  std::string stop_message{};
  // Final state: the last mesh solved on, its solution and full estimate.
  discretization::MeshResult mesh{};
  StationarySolution solution{};
  ErrorEstimate estimate{};
};

// The choice text of the adaptive decisions.
inline std::string marking_choice(const MarkingStrategy &marker) {
  if (const auto d = marker.descriptor())
    return pde2d::describe_strategy(*d);
  return marker.name() + " (not replayable)";
}
inline std::string refinement_choice(const discretization::MeshOptions &o) {
  return std::format(
      "marked longest-edge bisection ({}, {})",
      discretization::interior_algorithm_name(o.algorithm),
      discretization::boundary_treatment_name(o.boundary_treatment));
}

namespace adaptivity_detail {
inline void validate(const AdaptOptions &o) {
  if (!std::isfinite(o.tolerance) || o.tolerance < 0)
    throw std::invalid_argument(
        "Adaptive tolerance must be finite and nonnegative");
  if (!std::isfinite(o.time))
    throw std::invalid_argument("Adaptive solve time must be finite");
}
inline AdaptiveSettings settings_of(const AdaptOptions &o,
                                    const ErrorEstimator &estimator,
                                    const MarkingStrategy &marker) {
  AdaptiveSettings s;
  s.estimator = estimator.name();
  s.estimator_descriptor = estimator.descriptor();
  s.default_estimator = !o.estimator;
  s.marker = marker.name();
  s.marker_descriptor = marker.descriptor();
  s.default_marker = !o.marker;
  s.max_iterations = o.max_iterations;
  s.tolerance = o.tolerance;
  s.refinement = o.refinement;
  s.warm_start = o.warm_start;
  s.transfer = o.transfer;
  s.time = o.time;
  s.accept_unsatisfied_refinement = o.accept_unsatisfied_refinement;
  if (o.solve_planner)
    s.solve_planner = o.solve_planner->name();
  if (o.mesh_planner)
    s.mesh_planner = o.mesh_planner->name();
  return s;
}
inline Decision planner_decision(DecisionSubject subject,
                                 const PlannerRecord &planner,
                                 const char *option) {
  return decision_for(subject, planner_choice(planner), option, Rationale{});
}
// The properties a planner sees: the plan's (current) analysis, or an
// analysis of the problem on the mesh with default options.
inline ProblemProperties
planning_properties(const SolvePlan &plan, const Problem &problem,
                    const discretization::TriangleMesh &mesh) {
  if (plan.analysis)
    return plan.analysis->properties();
  return make_analysis_record(problem, mesh).properties();
}

inline AdaptiveHistory run(discretization::MeshResult mesh, bool accepted,
                           const Problem &problem, const SolvePlan &plan,
                           const AdaptOptions &options) {
  validate(options);
  if (!accepted)
    require_size_satisfied(mesh);
  const GradientRecoveryEstimator default_estimator(plan.assembly.execution);
  const ErrorEstimator &estimator =
      options.estimator ? *options.estimator : default_estimator;
  const BulkMarking default_marker;
  const MarkingStrategy &marker =
      options.marker ? *options.marker : default_marker;
  const auto marking_decision = decision_for(
      DecisionSubject::Marking, marking_choice(marker), "AdaptOptions::marker",
      options.marker ? Rationale{}
                     : Rationale{"default (AdaptOptions::marker unset)"});

  AdaptiveHistory history;
  history.settings = settings_of(options, estimator, marker);
  std::optional<Vector> guess;
  std::optional<TransferRecord> transfer;
  SolvePlan current = plan;
  for (std::size_t k = 0;; ++k) {
    AdaptiveIteration it;
    it.index = k;
    if (k > 0)
      refresh_analysis(current, problem, mesh.mesh);
    if (options.solve_planner) {
      const PlanningContext context{
          &mesh,
          history.iterations.empty() ? nullptr : &history.iterations.back(), k};
      current = run_planner(*options.solve_planner, problem,
                            planning_properties(current, problem, mesh.mesh),
                            current, context);
      it.decisions.push_back(planner_decision(DecisionSubject::SolvePlanner,
                                              *current.planner,
                                              "AdaptOptions::solve_planner"));
    }
    // Refined meshes reach here only if satisfied or explicitly accepted.
    auto prepared = [&] {
      if (accepted || options.accept_unsatisfied_refinement)
        return PreparedStationary(accept_unsatisfied(mesh), problem, current);
      return PreparedStationary(mesh, problem, current);
    };
    PreparedStationary stationary = prepared();
    auto solution = guess ? stationary.solve(options.time, *guess)
                          : stationary.solve(options.time);
    auto estimate = estimate_error(mesh.mesh, problem, solution.values,
                                   options.time, estimator);
    it.record = solution.record;
    it.solver_iterations = solution.report.iterations;
    it.initial_guess = solution.report.initial_guess;
    it.transfer = transfer;
    it.estimate = summarize_estimate(estimate, estimator);

    std::optional<AdaptiveStop> stop;
    std::string message;
    std::optional<discretization::MeshResult> refined;
    if (options.tolerance > 0 && estimate.global <= options.tolerance) {
      stop = AdaptiveStop::ToleranceReached;
      message = std::format("estimate {} <= tolerance {}", estimate.global,
                            options.tolerance);
    } else if (k >= options.max_iterations) {
      stop = AdaptiveStop::MaxIterations;
      message = std::format("{} refinements done", k);
    } else {
      const auto marked = mark_elements(marker, estimate, mesh.mesh);
      it.marking = MarkingRecord{marker.name(), marker.descriptor(),
                                 marked.size(), mesh.mesh.elements.size()};
      it.decisions.push_back(marking_decision);
      if (marked.empty()) {
        stop = AdaptiveStop::NothingMarked;
        message = "the marking strategy marked no element";
      } else {
        const auto &requested =
            options.refinement ? *options.refinement : mesh.options;
        // The mesh plan of the refined mesh (see the header comment): the
        // current one while it describes the requested options, otherwise
        // the requested options' own; a mesh planner starts from it.
        std::optional<MeshPlan> next_mesh_plan = current.mesh;
        if (next_mesh_plan &&
            mesh_options_difference(next_mesh_plan->options, requested))
          next_mesh_plan = mesh_plan_from_user(requested);
        Rationale why;
        const char *option = "AdaptOptions::refinement";
        if (options.mesh_planner) {
          const PlanningContext context{&mesh, &it, k + 1};
          next_mesh_plan = run_planner(
              *options.mesh_planner, problem,
              planning_properties(current, problem, mesh.mesh),
              next_mesh_plan ? *next_mesh_plan : mesh_plan_from_user(requested),
              context);
          it.decisions.push_back(planner_decision(
              DecisionSubject::MeshPlanner, *next_mesh_plan->planner,
              "AdaptOptions::mesh_planner"));
          option = "AdaptOptions::mesh_planner";
          why = {"planned (" + planner_choice(*next_mesh_plan->planner) + ")",
                 Evidence::Unknown, DecisionSource::Automatic};
        } else if (!options.refinement)
          why = {"default (the mesh's own MeshResult::options)"};
        const auto &refine_options =
            options.mesh_planner ? next_mesh_plan->options : requested;
        it.decisions.push_back(decision_for(DecisionSubject::Refinement,
                                            refinement_choice(refine_options),
                                            option, why));
        it.refinement_options = refine_options;
        refined = discretization::refine_mesh(mesh, marked, refine_options);
        current.mesh = std::move(next_mesh_plan);
        it.refinement = refined->report;
        it.refinement_run = refined->record;
        if (!refined->report.size_satisfied &&
            !options.accept_unsatisfied_refinement) {
          stop = AdaptiveStop::UnsatisfiedRefinement;
          message =
              "refined mesh misses its size target: " + refined->report.message;
        }
      }
    }
    history.iterations.push_back(std::move(it));
    if (stop) {
      history.stop = *stop;
      history.stop_message = std::move(message);
      history.mesh = std::move(mesh);
      history.solution = std::move(solution);
      history.estimate = std::move(estimate);
      return history;
    }
    if (options.warm_start) {
      auto moved = transfer_field(mesh.mesh, solution.values, refined->mesh,
                                  options.transfer);
      transfer =
          TransferRecord{mesh.mesh.nodes.size(), refined->mesh.nodes.size(),
                         moved.extrapolated_nodes, moved.max_outside_distance};
      guess = std::move(moved.values);
    }
    mesh = std::move(*refined);
  }
}
} // namespace adaptivity_detail

// Adapt `mesh` to `problem`: solve with `plan`, estimate, mark, refine,
// transfer and solve again until a stopping rule holds. The result must
// meet its size target (accept_unsatisfied to use it anyway). Throws what
// the components throw.
inline AdaptiveHistory adapt(const discretization::MeshResult &mesh,
                             const Problem &problem, const SolvePlan &plan,
                             const AdaptOptions &options = {}) {
  return adaptivity_detail::run(mesh, false, problem, plan, options);
}
inline AdaptiveHistory adapt(AcceptedMesh mesh, const Problem &problem,
                             const SolvePlan &plan,
                             const AdaptOptions &options = {}) {
  return adaptivity_detail::run(mesh.result(), true, problem, plan, options);
}
// ---------------------------------------------------------------------------
// Transient adaptivity: adapt_evolution() splits the plan's time interval
// into windows and composes, per window,
//
//   evolve ─▶ estimate (last frame) ─▶ mark ─▶ refine ─▶ transfer_field ─▶
//   restart (InitialConditions with nodal Fields) ─▶ evolve the next window
//
//   EvolutionAdaptOptions options;
//   options.windows = 4;                    // equal windows of plan.time
//   options.marker = std::make_shared<BulkMarking>(0.5);
//   auto history = adapt_evolution(mesh_result, problem, initial, plan,
//                                  options);
//   history.evolution.frames.back();        // the end state, on history.mesh
//   write_json(out, history);               // adaptivity_json.hpp
//
// Like adapt(), it adds no numerics: the same public calls in the same order
// give the same meshes and values bit for bit (tests/fem/transient_adaptivity
// .cpp composes the loop by hand as the oracle). Window k evolves on
// [adaptive_window_times(...)[k], [k + 1]] with the plan's method and step;
// the last step of a window is shortened when the window is not a whole
// number of steps (TimeGrid). After every window but the last, the estimate
// of the window's last frame (at its end time) is compared with
// `tolerance`: above it (or always, with tolerance 0) the strategy marks,
// the mesh is refined, the last frame's values (and velocity, when the
// initial conditions have one) are moved to the new mesh by transfer_field,
// and their Dirichlet nodes are set to the problem's Dirichlet data (and its
// time derivative) at the restart time, from PreparedAssembly on the new
// mesh: interpolated boundary values otherwise fail evolve()'s compatibility
// check. Without refinement (estimate within tolerance, nothing marked, or a
// refined mesh that missed its size target and accept_unsatisfied_refinement
// is false, which discards that mesh) the next window restarts on the same
// mesh from the last frame unchanged. Each window's plan is the user's plan
// with that window's start and end, plan.mesh updated as in adapt() and
// plan.analysis refreshed for every new mesh. No planners run.
struct EvolutionAdaptOptions {
  // Number of equal windows of [plan.time->start, plan.time->end].
  std::size_t windows = 4;
  // Unset: GradientRecoveryEstimator with the plan's execution policy.
  std::shared_ptr<const ErrorEstimator> estimator{};
  // Unset: BulkMarking{} (Dorfler, fraction 0.5).
  std::shared_ptr<const MarkingStrategy> marker{};
  // Refine after a window only when its end estimate exceeds this; 0
  // refines after every window but the last.
  double tolerance = 0;
  // Options of refine_mesh(result, marked, options); unset: each result's
  // own options.
  std::optional<discretization::MeshOptions> refinement{};
  TransferOptions transfer{};
  // Evolve on a refined mesh that missed its size target; otherwise that
  // mesh is discarded and the next window stays on the current mesh.
  bool accept_unsatisfied_refinement = false;
  // Output of every window's evolve() (the observer sees every accepted
  // stamp of every window; a window's first stamp repeats the previous
  // window's last).
  EvolutionOutput output{};
};

// The window boundaries t_0 = start < ... < t_windows = end, with
// t_k = start + (end - start) * k / windows (the last exactly `end`).
// Throws std::invalid_argument for zero windows or an invalid interval.
inline std::vector<double> adaptive_window_times(const TimePlan &time,
                                                 std::size_t windows) {
  if (windows == 0)
    throw std::invalid_argument("Adaptive evolution needs at least one window");
  if (!std::isfinite(time.start) || !std::isfinite(time.end) ||
      !(time.end > time.start))
    throw std::invalid_argument("Invalid time interval or step");
  std::vector<double> stamps(windows + 1);
  const double duration = time.end - time.start;
  for (std::size_t k = 0; k < windows; ++k)
    stamps[k] = time.start + duration * static_cast<double>(k) /
                                 static_cast<double>(windows);
  stamps[windows] = time.end;
  return stamps;
}

// How a window started from the previous window's last frame on a new mesh.
struct RestartRecord {
  TransferRecord values{};
  // Set when the velocity was transferred as well.
  std::optional<TransferRecord> velocity{};
  // Dirichlet nodes set to the problem's data at the restart time, and the
  // largest change that made to a transferred value (velocity: to its time
  // derivative).
  std::size_t dirichlet_nodes = 0;
  double max_dirichlet_correction = 0, max_velocity_correction = 0;
};
struct EvolutionWindow {
  std::size_t index = 0;
  double start = 0, end = 0;
  // The window's evolve(): plan (with this window's interval), mesh
  // fingerprint and provenance, initial-field fingerprints, work.
  RunRecord record{};
  // Set when this window started on a new mesh.
  std::optional<RestartRecord> restart{};
  // Of the window's last frame.
  EstimateSummary estimate{};
  // Unless this is the last window or the estimate was within tolerance.
  std::optional<MarkingRecord> marking{};
  std::optional<discretization::MeshOptions> refinement_options{};
  std::optional<discretization::MeshReport> refinement{};
  std::optional<discretization::MeshRunRecord> refinement_run{};
  // The refined mesh is the next window's mesh (false: discarded because it
  // missed its size target).
  bool refinement_accepted = false;
  // Marking and Refinement decisions, as in AdaptiveIteration.
  std::vector<Decision> decisions{};
};
// The settings of adapt_evolution (EvolutionAdaptOptions without strategy
// objects and output callbacks).
struct EvolutionAdaptSettings {
  std::size_t windows = 0;
  std::string estimator{}, marker{};
  std::optional<pde2d::StrategyDescriptor> estimator_descriptor{},
      marker_descriptor{};
  bool default_estimator = true, default_marker = true;
  double tolerance = 0;
  std::optional<discretization::MeshOptions> refinement{};
  TransferOptions transfer{};
  bool accept_unsatisfied_refinement = false;
  // The whole interval and step of the user's plan.
  double start = 0, end = 0, step = 0;
};
struct EvolutionAdaptiveHistory {
  EvolutionAdaptSettings settings{};
  std::vector<EvolutionWindow> windows{};
  // Final state: the mesh of the last window, its evolution and the
  // estimate of its last frame (at the end time).
  discretization::MeshResult mesh{};
  Evolution evolution{};
  ErrorEstimate estimate{};
};

namespace adaptivity_detail {
// Set the Dirichlet nodes of `values` to `data`; returns the largest change.
inline double impose_dirichlet(Vector &values,
                               const std::vector<DirichletValue> &data) {
  double change = 0;
  for (const auto &d : data) {
    change = std::max(change, std::abs(values[d.node] - d.value));
    values[d.node] = d.value;
  }
  return change;
}

inline EvolutionAdaptiveHistory
run_evolution(discretization::MeshResult mesh, bool accepted,
              const Problem &problem, const InitialConditions &initial,
              const SolvePlan &plan, const EvolutionAdaptOptions &options) {
  if (!plan.time)
    throw std::invalid_argument("Evolution plan has no time settings");
  if (plan.time->step_controller)
    throw std::invalid_argument(
        "adapt_evolution needs uniform time steps; TimePlan::step_controller "
        "is set");
  if (!std::isfinite(options.tolerance) || options.tolerance < 0)
    throw std::invalid_argument(
        "Adaptive tolerance must be finite and nonnegative");
  const auto stamps = adaptive_window_times(*plan.time, options.windows);
  if (!accepted)
    require_size_satisfied(mesh);
  const GradientRecoveryEstimator default_estimator(plan.assembly.execution);
  const ErrorEstimator &estimator =
      options.estimator ? *options.estimator : default_estimator;
  const BulkMarking default_marker;
  const MarkingStrategy &marker =
      options.marker ? *options.marker : default_marker;
  const auto marking_decision = decision_for(
      DecisionSubject::Marking, marking_choice(marker),
      "EvolutionAdaptOptions::marker",
      options.marker ? Rationale{}
                     : Rationale{"default (EvolutionAdaptOptions::marker "
                                 "unset)"});

  EvolutionAdaptiveHistory history;
  auto &settings = history.settings;
  settings.windows = options.windows;
  settings.estimator = estimator.name();
  settings.estimator_descriptor = estimator.descriptor();
  settings.default_estimator = !options.estimator;
  settings.marker = marker.name();
  settings.marker_descriptor = marker.descriptor();
  settings.default_marker = !options.marker;
  settings.tolerance = options.tolerance;
  settings.refinement = options.refinement;
  settings.transfer = options.transfer;
  settings.accept_unsatisfied_refinement =
      options.accept_unsatisfied_refinement;
  settings.start = plan.time->start;
  settings.end = plan.time->end;
  settings.step = plan.time->step;

  const bool with_velocity = initial.velocity.has_value();
  InitialConditions start = initial;
  std::optional<RestartRecord> restart;
  SolvePlan current = plan;
  // Meshes that reach a window are satisfied or explicitly accepted.
  const bool accept_all = accepted || options.accept_unsatisfied_refinement;
  for (std::size_t k = 0; k < options.windows; ++k) {
    EvolutionWindow window;
    window.index = k;
    window.start = stamps[k];
    window.end = stamps[k + 1];
    current.time->start = window.start;
    current.time->end = window.end;
    auto evolution =
        accept_all ? evolve(accept_unsatisfied(mesh), problem, start, current,
                            options.output)
                   : evolve(mesh, problem, start, current, options.output);
    const auto &last = evolution.frames.back();
    auto estimate =
        estimate_error(mesh.mesh, problem, last.values, window.end, estimator);
    window.record = evolution.record;
    window.restart = std::move(restart);
    restart.reset();
    window.estimate = summarize_estimate(estimate, estimator);
    const bool final_window = k + 1 == options.windows;
    std::optional<discretization::MeshResult> refined;
    if (!final_window &&
        !(options.tolerance > 0 && estimate.global <= options.tolerance)) {
      const auto marked = mark_elements(marker, estimate, mesh.mesh);
      window.marking = MarkingRecord{marker.name(), marker.descriptor(),
                                     marked.size(), mesh.mesh.elements.size()};
      window.decisions.push_back(marking_decision);
      if (!marked.empty()) {
        const auto &requested =
            options.refinement ? *options.refinement : mesh.options;
        window.decisions.push_back(decision_for(
            DecisionSubject::Refinement, refinement_choice(requested),
            "EvolutionAdaptOptions::refinement",
            options.refinement
                ? Rationale{}
                : Rationale{"default (the mesh's own MeshResult::options)"}));
        window.refinement_options = requested;
        refined = discretization::refine_mesh(mesh, marked, requested);
        window.refinement = refined->report;
        window.refinement_run = refined->record;
        window.refinement_accepted = refined->report.size_satisfied ||
                                     options.accept_unsatisfied_refinement;
        if (!window.refinement_accepted)
          refined.reset();
      }
    }
    if (final_window) {
      history.windows.push_back(std::move(window));
      history.mesh = std::move(mesh);
      history.evolution = std::move(evolution);
      history.estimate = std::move(estimate);
      return history;
    }
    if (!refined) {
      start = InitialConditions{last.values, std::nullopt};
      if (with_velocity)
        start.velocity = last.velocity;
    } else {
      RestartRecord r;
      auto values = transfer_field(mesh.mesh, last.values, refined->mesh,
                                   options.transfer);
      r.values = {mesh.mesh.nodes.size(), refined->mesh.nodes.size(),
                  values.extrapolated_nodes, values.max_outside_distance};
      const PreparedAssembly assembly(refined->mesh, problem, current.assembly);
      const auto data = assembly.assemble(window.end).dirichlet;
      r.dirichlet_nodes = data.size();
      r.max_dirichlet_correction = impose_dirichlet(values.values, data);
      start = InitialConditions{std::move(values.values), std::nullopt};
      if (with_velocity) {
        auto velocity = transfer_field(mesh.mesh, last.velocity, refined->mesh,
                                       options.transfer);
        r.velocity = TransferRecord{
            mesh.mesh.nodes.size(), refined->mesh.nodes.size(),
            velocity.extrapolated_nodes, velocity.max_outside_distance};
        r.max_velocity_correction = impose_dirichlet(
            velocity.values,
            assembly.assemble(window.end, DirichletDerivative::FirstDerivative)
                .dirichlet);
        start.velocity = std::move(velocity.values);
      }
      restart = std::move(r);
      // The mesh plan and analysis of the new mesh, as in adapt().
      if (current.mesh &&
          mesh_options_difference(current.mesh->options, refined->options))
        current.mesh = mesh_plan_from_user(refined->options);
      history.windows.push_back(std::move(window));
      mesh = std::move(*refined);
      refresh_analysis(current, problem, mesh.mesh);
      continue;
    }
    history.windows.push_back(std::move(window));
  }
  throw std::logic_error("adapt_evolution ended without a final window");
}
} // namespace adaptivity_detail

// Evolve `problem` from `initial` with `plan`, adapting the mesh between
// time windows (see above). The mesh result must meet its size target
// (accept_unsatisfied to use it anyway). Throws what the components throw,
// and std::invalid_argument for a plan without time settings or with a step
// controller.
inline EvolutionAdaptiveHistory
adapt_evolution(const discretization::MeshResult &mesh, const Problem &problem,
                const InitialConditions &initial, const SolvePlan &plan,
                const EvolutionAdaptOptions &options = {}) {
  return adaptivity_detail::run_evolution(mesh, false, problem, initial, plan,
                                          options);
}
inline EvolutionAdaptiveHistory
adapt_evolution(AcceptedMesh mesh, const Problem &problem,
                const InitialConditions &initial, const SolvePlan &plan,
                const EvolutionAdaptOptions &options = {}) {
  return adaptivity_detail::run_evolution(mesh.result(), true, problem, initial,
                                          plan, options);
}
} // namespace pde2d::fem
