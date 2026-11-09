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
#include "fem/marking.hpp"
#include "fem/mesh_acceptance.hpp"
#include "fem/plan.hpp"
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
  // The marking and refinement choices (DecisionSubject::Marking,
  // DecisionSubject::Refinement), in that order.
  std::vector<Decision> decisions{};
};

struct AdaptiveHistory {
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
  std::optional<Vector> guess;
  std::optional<TransferRecord> transfer;
  for (std::size_t k = 0;; ++k) {
    // Refined meshes reach here only if satisfied or explicitly accepted.
    auto prepared = [&] {
      if (accepted || options.accept_unsatisfied_refinement)
        return PreparedStationary(accept_unsatisfied(mesh), problem, plan);
      return PreparedStationary(mesh, problem, plan);
    };
    PreparedStationary stationary = prepared();
    auto solution = guess ? stationary.solve(options.time, *guess)
                          : stationary.solve(options.time);
    auto estimate = estimate_error(mesh.mesh, problem, solution.values,
                                   options.time, estimator);
    AdaptiveIteration it;
    it.index = k;
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
        const auto &refine_options =
            options.refinement ? *options.refinement : mesh.options;
        it.decisions.push_back(decision_for(
            DecisionSubject::Refinement, refinement_choice(refine_options),
            "AdaptOptions::refinement",
            options.refinement
                ? Rationale{}
                : Rationale{"default (the mesh's own MeshResult::options)"}));
        refined = discretization::refine_mesh(mesh, marked, refine_options);
        it.refinement = refined->report;
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
} // namespace pde2d::fem
