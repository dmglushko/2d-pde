#pragma once
// The shared part of the adaptive drivers (adapt_stationary.hpp,
// adapt_evolution.hpp): their options, their history and the one adaptive
// step both run after every solve or window,
//
//   estimate ─▶ mark ─▶ plan the mesh ─▶ change the mesh ─▶ transfer ─▶
//   ErrorEstimator  MarkingStrategy  MeshPlanner     MeshAdaptation
//                                    (optional)      (MarkedBisection)
//   refresh plan.mesh and plan.analysis for the new mesh
//
// The step is a composition of public components and adds no numerics:
// the same calls in the same order give the same meshes and values bit for
// bit (tests/fem/adaptive_loop.cpp and transient_adaptivity.cpp compose the
// loops by hand as their oracles).
//
// Plans across steps. The plan of step k is the user's plan, or for k > 0
// the plan of step k - 1 updated for the new mesh:
//  - plan.mesh, when set, describes the options the mesh was changed with:
//    the mesh planner's MeshPlan, or, when the refinement options
//    (AdaptOptions::refinement, default the mesh's own options) differ from
//    it, mesh_plan_from_user(refinement options); unchanged options keep it
//    as is. A plan without a mesh plan gets one only from a mesh planner.
//  - plan.analysis made on a mesh is recomputed on the new mesh with its own
//    options (refresh_analysis); a mesh-free analysis stays;
//  - then the solve planner, if any, plans the next solve (run_planner).
// The first step's plan must match its inputs: a stale analysis or a mesh
// plan that does not describe the mesh throws as in PreparedStationary.
#include "discretization/mesh_types.hpp"
#include "fem/adaptivity_types.hpp"
#include "fem/analysis.hpp"
#include "fem/decision.hpp"
#include "fem/estimation.hpp"
#include "fem/evolution.hpp"
#include "fem/marking.hpp"
#include "fem/mesh_adaptation.hpp"
#include "fem/mesh_plan.hpp"
#include "fem/plan.hpp"
#include "fem/planner.hpp"
#include "fem/solver.hpp"
#include "fem/transfer.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace pde2d::fem {
// Options of both adaptive drivers. Strategies left unset use the defaults
// named below; a history keeps these options with the defaults filled in.
struct AdaptOptions {
  // Estimator of every step; unset: GradientRecoveryEstimator with the
  // plan's execution policy.
  std::shared_ptr<const ErrorEstimator> estimator{};
  // Marking strategy; unset: BulkMarking{} (Dorfler, fraction 0.5).
  std::shared_ptr<const MarkingStrategy> marker{};
  // How the marked mesh changes (mesh_adaptation.hpp); unset:
  // MarkedBisection (refine_mesh with the marked elements).
  std::shared_ptr<const MeshAdaptation> adaptation{};
  // Optional planners (planner.hpp): `solve` runs before every solve or
  // window with that mesh and the previous step, and its checked plan is
  // the plan of that solve and the defaults of the next call; `mesh` runs
  // after marking with the current mesh and the step just estimated, and
  // its MeshPlan's options change the mesh (in place of `refinement`) and
  // become plan.mesh of the next solve.
  PlannerSet planners{};
  // adapt: stop once the global estimate is at most this. adapt_evolution:
  // change the mesh after a window only when its end estimate exceeds this.
  // 0: never stop early, change after every window but the last.
  double tolerance = 0;
  // Options the mesh is changed with; unset: each result's own options
  // (MeshResult::options).
  std::optional<discretization::MeshOptions> refinement{};
  TransferOptions transfer{};
  // Use a changed mesh that missed its size target (for example an
  // unrefinable marked triangle under a Fixed boundary). Otherwise adapt
  // stops with UnsatisfiedRefinement and adapt_evolution discards it and
  // keeps the current mesh for the next window.
  bool accept_unsatisfied_refinement = false;

  // adapt only: the largest number of mesh changes after the first solve,
  // whether to seed each solve with the previous solution transferred to
  // the new mesh (used by iterative solvers, ignored by direct ones), and
  // the time the stationary problem is solved and estimated at.
  std::size_t max_iterations = 10;
  bool warm_start = true;
  double time = 0;

  // adapt_evolution only: the number of equal windows of [plan.time->start,
  // plan.time->end], and the output of every window's evolve() (the
  // observer sees every accepted stamp of every window; a window's first
  // stamp repeats the previous window's last).
  std::size_t windows = 4;
  EvolutionOutput output{};
};

// What an adaptive loop did. `steps` has one entry per solve (adapt) or
// window (adapt_evolution); the final state is the last step's mesh, its
// full estimate and its solution (adapt) or evolution (adapt_evolution).
struct AdaptiveHistory {
  // The options the loop ran with, estimator, marker and adaptation set to
  // the strategies that ran (the defaults when they were unset).
  AdaptOptions options{};
  std::vector<AdaptiveStep> steps{};
  AdaptiveStop stop = AdaptiveStop::MaxIterations;
  std::string stop_message{};
  discretization::MeshResult mesh{};
  ErrorEstimate estimate{};
  std::optional<StationarySolution> solution{};
  std::optional<Evolution> evolution{};
};

// The record of an estimate, without its indicators.
inline EstimateRecord summarize_estimate(const ErrorEstimate &estimate,
                                         const ErrorEstimator &estimator) {
  EstimateRecord s{estimator.name(), estimator.descriptor(), estimate.global,
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
// The choice text of a Marking decision.
inline std::string marking_choice(const MarkingStrategy &marker) {
  if (const auto d = marker.descriptor())
    return pde2d::describe_strategy(*d);
  return marker.name() + " (not replayable)";
}

namespace adaptivity_detail {
inline void validate_tolerance(const AdaptOptions &o) {
  if (!std::isfinite(o.tolerance) || o.tolerance < 0)
    throw std::invalid_argument(
        "Adaptive tolerance must be finite and nonnegative");
}
inline bool within_tolerance(const AdaptOptions &o,
                             const ErrorEstimate &estimate) {
  return o.tolerance > 0 && estimate.global <= o.tolerance;
}
inline Decision planner_decision(DecisionSubject subject,
                                 const StrategyRecord &planner,
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

// How the adaptive step ended for the driver.
enum class MeshChange { NothingMarked, Rejected, Changed };
struct ChangeResult {
  MeshChange change = MeshChange::NothingMarked;
  std::optional<discretization::MeshResult> mesh{};
  // The fields moved to the new mesh, in the order given, and their records.
  std::vector<Vector> fields{};
  std::vector<TransferRecord> transfers{};
};

// One adaptive loop: the options with the defaults resolved and the steps
// both drivers share.
class Loop {
public:
  Loop(const AdaptOptions &options, const Problem &problem,
       const pde2d::ExecutionOptions &execution)
      : options_(options), problem_(problem) {
    if (!options_.estimator)
      options_.estimator =
          std::make_shared<GradientRecoveryEstimator>(execution);
    if (!options_.marker)
      options_.marker = std::make_shared<BulkMarking>();
    if (!options_.adaptation)
      options_.adaptation = std::make_shared<MarkedBisection>();
    marking_decision_ = decision_for(
        DecisionSubject::Marking, marking_choice(*options_.marker),
        "AdaptOptions::marker",
        options.marker ? Rationale{}
                       : Rationale{"default (AdaptOptions::marker unset)"});
    default_refinement_ = !options.refinement;
  }
  const AdaptOptions &options() const noexcept { return options_; }

  // Run the solve planner, if any, on `current` for the step about to be
  // solved on `mesh` (previous: the last finished step).
  void plan_solve(SolvePlan &current, const discretization::MeshResult &mesh,
                  const std::vector<AdaptiveStep> &steps,
                  AdaptiveStep &step) const {
    if (!options_.planners.solve)
      return;
    PlanningContext context;
    context.mesh = &mesh;
    context.previous = steps.empty() ? nullptr : &steps.back();
    context.step = step.index;
    current = run_planner(*options_.planners.solve, problem_,
                          planning_properties(current, problem_, mesh.mesh),
                          current, context);
    step.decisions.push_back(planner_decision(DecisionSubject::SolvePlanner,
                                              *current.planner,
                                              "AdaptOptions::planners.solve"));
  }

  ErrorEstimate estimate(const discretization::MeshResult &mesh,
                         const Vector &values, double time,
                         AdaptiveStep &step) const {
    auto estimate =
        estimate_error(mesh.mesh, problem_, values, time, *options_.estimator);
    step.time = time;
    step.estimate = summarize_estimate(estimate, *options_.estimator);
    return estimate;
  }

  // Mark, plan the mesh, change it, and on an accepted change move
  // `fields` to the new mesh and update `current` for it (plan.mesh,
  // plan.analysis).
  ChangeResult change_mesh(const discretization::MeshResult &mesh,
                           const ErrorEstimate &estimate, SolvePlan &current,
                           AdaptiveStep &step,
                           const std::vector<const Vector *> &fields) const {
    const auto &marker = *options_.marker;
    const auto marked = mark_elements(marker, estimate, mesh.mesh);
    step.marking = MarkingRecord{marker.name(), marker.descriptor(),
                                 marked.size(), mesh.mesh.elements.size()};
    step.decisions.push_back(marking_decision_);
    ChangeResult result;
    if (marked.empty())
      return result;

    const auto &requested =
        options_.refinement ? *options_.refinement : mesh.options;
    // The mesh plan of the new mesh (see the header comment): the current
    // one while it describes the requested options, otherwise the requested
    // options' own; a mesh planner starts from it.
    std::optional<MeshPlan> next_mesh_plan = current.mesh;
    if (next_mesh_plan &&
        mesh_options_difference(next_mesh_plan->options, requested))
      next_mesh_plan = mesh_plan_from_user(requested);
    Rationale why;
    const char *option = "AdaptOptions::refinement";
    if (options_.planners.mesh) {
      PlanningContext context;
      context.mesh = &mesh;
      context.previous = &step;
      context.step = step.index + 1;
      next_mesh_plan = run_planner(
          *options_.planners.mesh, problem_,
          planning_properties(current, problem_, mesh.mesh),
          next_mesh_plan ? *next_mesh_plan : mesh_plan_from_user(requested),
          context);
      step.decisions.push_back(planner_decision(DecisionSubject::MeshPlanner,
                                                *next_mesh_plan->planner,
                                                "AdaptOptions::planners.mesh"));
      option = "AdaptOptions::planners.mesh";
      why = {"planned (" + planner_choice(*next_mesh_plan->planner) + ")",
             Evidence::Unknown, DecisionSource::Automatic};
    } else if (default_refinement_)
      why = {"default (the mesh's own MeshResult::options)"};
    const auto &change_options =
        options_.planners.mesh ? next_mesh_plan->options : requested;
    const auto &adaptation = *options_.adaptation;
    step.decisions.push_back(decision_for(
        DecisionSubject::Refinement,
        mesh_adaptation_choice(adaptation, change_options), option, why));
    auto changed =
        apply_mesh_adaptation(adaptation, mesh, marked, change_options);
    const bool accepted =
        changed.report.size_satisfied || options_.accept_unsatisfied_refinement;
    step.adaptation =
        MeshAdaptationRecord{adaptation.name(), adaptation.descriptor(),
                             changed.report, changed.record, accepted};
    if (!accepted) {
      result.change = MeshChange::Rejected;
      return result;
    }
    for (const Vector *field : fields) {
      auto moved =
          transfer_field(mesh.mesh, *field, changed.mesh, options_.transfer);
      result.transfers.push_back(
          TransferRecord{mesh.mesh.nodes.size(), changed.mesh.nodes.size(),
                         moved.extrapolated_nodes, moved.max_outside_distance});
      result.fields.push_back(std::move(moved.values));
    }
    current.mesh = std::move(next_mesh_plan);
    refresh_analysis(current, problem_, changed.mesh);
    result.change = MeshChange::Changed;
    result.mesh = std::move(changed);
    return result;
  }

private:
  AdaptOptions options_;
  const Problem &problem_;
  Decision marking_decision_{};
  bool default_refinement_ = true;
};
} // namespace adaptivity_detail
} // namespace pde2d::fem
