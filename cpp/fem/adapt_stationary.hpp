#pragma once
// Adaptive loop for a stationary problem (the shared step is in
// adaptivity_core.hpp):
//
//   solve ─▶ estimate ─▶ mark ─▶ change mesh ─▶ transfer ─▶ solve (warm
//   start) ─▶ …
//
//   AdaptOptions options;
//   options.marker = std::make_shared<BulkMarking>(0.6);
//   options.max_iterations = 8;          // mesh changes after the first solve
//   options.transfer.max_outside_distance = 0.02; // curved boundary
//   auto history = adapt(mesh_result, problem, plan, options);
//   history.solution->values;            // on history.mesh
//   write_json(out, history);            // adaptivity_json.hpp
//
// Each step links the RunRecord of its solve (mesh fingerprint and
// provenance, initial guess, iterations), the transfer that seeded it, its
// estimate and, unless it was the last, what was marked and how the mesh
// changed, with the marking and the change recorded as Decisions.
// AdaptiveHistory::stop says why the loop ended.
//
// Stopping, checked after every solve and estimate in this order:
// estimate.global <= tolerance (ToleranceReached), max_iterations mesh
// changes done (MaxIterations), the strategy marked nothing
// (NothingMarked), the changed mesh missed its size target and
// accept_unsatisfied_refinement is false (UnsatisfiedRefinement; that mesh
// is not solved on).
#include "discretization/mesh_types.hpp"
#include "fem/adaptivity_core.hpp"
#include "fem/mesh_acceptance.hpp"
#include "fem/plan.hpp"
#include "fem/solver.hpp"
#include <cmath>
#include <format>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace pde2d::fem {
namespace adaptivity_detail {
inline AdaptiveHistory adapt_stationary(discretization::MeshResult mesh,
                                        bool accepted, const Problem &problem,
                                        const SolvePlan &plan,
                                        const AdaptOptions &options) {
  validate_tolerance(options);
  if (!std::isfinite(options.time))
    throw std::invalid_argument("Adaptive solve time must be finite");
  if (!accepted)
    require_size_satisfied(mesh);
  const Loop loop(options, problem, plan.assembly.execution);
  AdaptiveHistory history;
  history.options = loop.options();
  std::optional<Vector> guess;
  std::optional<TransferRecord> transfer;
  SolvePlan current = plan;
  // Changed meshes reach a solve only if satisfied or explicitly accepted.
  const bool accept_all = accepted || options.accept_unsatisfied_refinement;
  for (std::size_t k = 0;; ++k) {
    AdaptiveStep step;
    step.index = k;
    loop.plan_solve(current, mesh, history.steps, step);
    PreparedStationary stationary =
        accept_all
            ? PreparedStationary(accept_unsatisfied(mesh), problem, current)
            : PreparedStationary(mesh, problem, current);
    auto solution = guess ? stationary.solve(options.time, *guess)
                          : stationary.solve(options.time);
    auto estimate = loop.estimate(mesh, solution.values, options.time, step);
    step.record = solution.record;
    step.transfer = transfer;

    std::optional<AdaptiveStop> stop;
    std::string message;
    ChangeResult change;
    if (within_tolerance(options, estimate)) {
      stop = AdaptiveStop::ToleranceReached;
      message = std::format("estimate {} <= tolerance {}", estimate.global,
                            options.tolerance);
    } else if (k >= options.max_iterations) {
      stop = AdaptiveStop::MaxIterations;
      message = std::format("{} refinements done", k);
    } else {
      std::vector<const Vector *> fields;
      if (options.warm_start)
        fields.push_back(&solution.values);
      change = loop.change_mesh(mesh, estimate, current, step, fields);
      if (change.change == MeshChange::NothingMarked) {
        stop = AdaptiveStop::NothingMarked;
        message = "the marking strategy marked no element";
      } else if (change.change == MeshChange::Rejected) {
        stop = AdaptiveStop::UnsatisfiedRefinement;
        message = "refined mesh misses its size target: " +
                  step.adaptation->report.message;
      }
    }
    history.steps.push_back(std::move(step));
    if (stop) {
      history.stop = *stop;
      history.stop_message = std::move(message);
      history.mesh = std::move(mesh);
      history.solution = std::move(solution);
      history.estimate = std::move(estimate);
      return history;
    }
    if (options.warm_start) {
      transfer = change.transfers.front();
      guess = std::move(change.fields.front());
    }
    mesh = std::move(*change.mesh);
  }
}
} // namespace adaptivity_detail

// Adapt `mesh` to `problem`: solve with `plan`, estimate, mark, change the
// mesh, transfer and solve again until a stopping rule holds. The result
// must meet its size target (accept_unsatisfied to use it anyway). Throws
// what the components throw.
inline AdaptiveHistory adapt(const discretization::MeshResult &mesh,
                             const Problem &problem, const SolvePlan &plan,
                             const AdaptOptions &options = {}) {
  return adaptivity_detail::adapt_stationary(mesh, false, problem, plan,
                                             options);
}
inline AdaptiveHistory adapt(AcceptedMesh mesh, const Problem &problem,
                             const SolvePlan &plan,
                             const AdaptOptions &options = {}) {
  return adaptivity_detail::adapt_stationary(mesh.result(), true, problem, plan,
                                             options);
}
} // namespace pde2d::fem
