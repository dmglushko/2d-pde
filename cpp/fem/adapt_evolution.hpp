#pragma once
// Transient adaptivity: adapt_evolution() splits the plan's time interval
// into windows and runs, after every window but the last, the shared
// adaptive step (adaptivity_core.hpp):
//
//   evolve ─▶ estimate (last frame) ─▶ mark ─▶ change mesh ─▶ transfer_field
//   ─▶ restart (InitialConditions with nodal Fields) ─▶ evolve the next window
//
//   AdaptOptions options;
//   options.windows = 4;                    // equal windows of plan.time
//   options.marker = std::make_shared<BulkMarking>(0.5);
//   auto history = adapt_evolution(mesh_result, problem, initial, plan,
//                                  options);
//   history.evolution->frames.back();       // the end state, on history.mesh
//   write_json(out, history);               // adaptivity_json.hpp
//
// Window k evolves on [adaptive_window_times(...)[k], [k + 1]] with the
// plan's method and step; the last step of a window is shortened when the
// window is not a whole number of steps (TimeGrid). After every window but
// the last, the estimate of the window's last frame (at its end time) is
// compared with `tolerance`: above it (or always, with tolerance 0) the
// strategy marks, the mesh changes, the last frame's values (and velocity,
// when the initial conditions have one) are moved to the new mesh by
// transfer_field, and their Dirichlet nodes are set to the problem's
// Dirichlet data (and its time derivative) at the restart time, from
// PreparedAssembly on the new mesh: interpolated boundary values otherwise
// fail evolve()'s compatibility check. Without a change (estimate within
// tolerance, nothing marked, or a changed mesh that missed its size target
// and accept_unsatisfied_refinement is false, which discards that mesh) the
// next window restarts on the same mesh from the last frame unchanged. Each
// window's plan is the user's plan (or the solve planner's) with that
// window's start and end; plan.mesh and plan.analysis follow the mesh as in
// adapt(). The loop stops with EndReached after the last window.
#include "discretization/mesh_types.hpp"
#include "fem/adaptivity_core.hpp"
#include "fem/assembly.hpp"
#include "fem/evolution.hpp"
#include "fem/mesh_acceptance.hpp"
#include "fem/plan.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <format>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace pde2d::fem {
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

namespace adaptivity_detail {
// Set the Dirichlet nodes of `values` to `data`; record how many and the
// largest change.
inline void impose_dirichlet(Vector &values,
                             const std::vector<DirichletValue> &data,
                             TransferRecord &record) {
  double change = 0;
  for (const auto &d : data) {
    change = std::max(change, std::abs(values[d.node] - d.value));
    values[d.node] = d.value;
  }
  record.dirichlet_nodes = data.size();
  record.max_dirichlet_correction = change;
}

inline AdaptiveHistory adapt_transient(discretization::MeshResult mesh,
                                       bool accepted, const Problem &problem,
                                       const InitialConditions &initial,
                                       const SolvePlan &plan,
                                       const AdaptOptions &options) {
  if (!plan.time)
    throw std::invalid_argument("Evolution plan has no time settings");
  if (plan.time->step_controller)
    throw std::invalid_argument(
        "adapt_evolution needs uniform time steps; TimePlan::step_controller "
        "is set");
  validate_tolerance(options);
  const auto stamps = adaptive_window_times(*plan.time, options.windows);
  if (!accepted)
    require_size_satisfied(mesh);
  const Loop loop(options, problem, plan.assembly.execution);
  AdaptiveHistory history;
  history.options = loop.options();

  const bool with_velocity = initial.velocity.has_value();
  InitialConditions start = initial;
  std::optional<TransferRecord> transfer, velocity_transfer;
  SolvePlan current = plan;
  // Meshes that reach a window are satisfied or explicitly accepted.
  const bool accept_all = accepted || options.accept_unsatisfied_refinement;
  for (std::size_t k = 0; k < options.windows; ++k) {
    AdaptiveStep step;
    step.index = k;
    step.window_start = stamps[k];
    const auto set_window = [&] {
      current.time->start = stamps[k];
      current.time->end = stamps[k + 1];
    };
    set_window();
    loop.plan_solve(current, mesh, history.steps, step);
    set_window();
    auto evolution =
        accept_all ? evolve(accept_unsatisfied(mesh), problem, start, current,
                            options.output)
                   : evolve(mesh, problem, start, current, options.output);
    const auto &last = evolution.frames.back();
    auto estimate = loop.estimate(mesh, last.values, stamps[k + 1], step);
    step.record = evolution.record;
    step.transfer = std::exchange(transfer, std::nullopt);
    step.velocity_transfer = std::exchange(velocity_transfer, std::nullopt);
    if (k + 1 == options.windows) {
      history.steps.push_back(std::move(step));
      history.stop = AdaptiveStop::EndReached;
      history.stop_message = std::format("{} windows evolved", options.windows);
      history.mesh = std::move(mesh);
      history.evolution = std::move(evolution);
      history.estimate = std::move(estimate);
      return history;
    }
    ChangeResult change;
    if (!within_tolerance(options, estimate)) {
      std::vector<const Vector *> fields{&last.values};
      if (with_velocity)
        fields.push_back(&last.velocity);
      change = loop.change_mesh(mesh, estimate, current, step, fields);
    }
    if (change.change != MeshChange::Changed) {
      start = InitialConditions{last.values, std::nullopt};
      if (with_velocity)
        start.velocity = last.velocity;
    } else {
      auto &changed = *change.mesh;
      const PreparedAssembly assembly(changed.mesh, problem, current.assembly);
      transfer = change.transfers[0];
      impose_dirichlet(change.fields[0],
                       assembly.assemble(stamps[k + 1]).dirichlet, *transfer);
      start = InitialConditions{std::move(change.fields[0]), std::nullopt};
      if (with_velocity) {
        velocity_transfer = change.transfers[1];
        impose_dirichlet(
            change.fields[1],
            assembly
                .assemble(stamps[k + 1], DirichletDerivative::FirstDerivative)
                .dirichlet,
            *velocity_transfer);
        start.velocity = std::move(change.fields[1]);
      }
      mesh = std::move(changed);
    }
    history.steps.push_back(std::move(step));
  }
  throw std::logic_error("adapt_evolution ended without a final window");
}
} // namespace adaptivity_detail

// Evolve `problem` from `initial` with `plan`, adapting the mesh between
// time windows (see above). The mesh result must meet its size target
// (accept_unsatisfied to use it anyway). Throws what the components throw,
// and std::invalid_argument for a plan without time settings or with a step
// controller.
inline AdaptiveHistory adapt_evolution(const discretization::MeshResult &mesh,
                                       const Problem &problem,
                                       const InitialConditions &initial,
                                       const SolvePlan &plan,
                                       const AdaptOptions &options = {}) {
  return adaptivity_detail::adapt_transient(mesh, false, problem, initial, plan,
                                            options);
}
inline AdaptiveHistory adapt_evolution(AcceptedMesh mesh,
                                       const Problem &problem,
                                       const InitialConditions &initial,
                                       const SolvePlan &plan,
                                       const AdaptOptions &options = {}) {
  return adaptivity_detail::adapt_transient(mesh.result(), true, problem,
                                            initial, plan, options);
}
} // namespace pde2d::fem
