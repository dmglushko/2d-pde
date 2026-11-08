#pragma once
// Solve plan: every setting that shapes a solve, together with the decisions
// that explain it (docs/vision.md, "Decisions are explained" and "The user
// stays in control").
//
//   auto plan = plan_from_user(assembly, solver);        // stationary
//   auto plan = plan_from_user(assembly, solver, time);  // transient
//   auto plan = plan_from_user(time_options);            // (evolution.hpp)
//   solve_stationary(mesh, problem, plan);
//   PreparedStationary(mesh, problem, plan);
//   evolve(mesh, problem, initial, plan, output);
//
// SolvePlan is the one output type of a planner and the one place to
// override it: a future automatic chooser returns a SolvePlan whose
// decisions use DecisionSource::Automatic, and a user edits fields of that
// plan (and the matching decision) before solving. Nothing chooses
// automatically yet: plan_from_user records the user's options exactly as
// the option-based entry points do, so both paths give identical results.
#include "fem/assembly.hpp"
#include "fem/decision.hpp"
#include "fem/linear_solver.hpp"
#include "fem/step_control.hpp"
#include "fem/time_integration.hpp"
#include <cstddef>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace pde2d::fem {
// The time discretization of a plan (TimeOptions without its output part).
struct TimePlan {
  TimeMethod method = TimeMethod::BackwardEuler;
  double start = 0, end = 1, step = .01;
  // A user scheme; when set it replaces `method`.
  TimeIntegratorFactory integrator{};
  // A user step sequence; when set it replaces `step`.
  StepControllerFactory step_controller{};
  // Bound on consecutive rejected trials (see TimeOptions).
  std::size_t max_consecutive_rejections = default_max_consecutive_rejections;
};

struct SolvePlan {
  AssemblyOptions assembly{};
  SolverOptions solver{};
  // Present for evolutions; evolve() rejects a plan without it.
  std::optional<TimePlan> time{};
  // Why each setting has its value, in a fixed order: for evolutions
  // "time integrator", "time steps", then the solver_decisions(). A choice
  // made by a strategy object (an integrator or step controller factory) is
  // named by that object: evolve() sets those two records' `choice` to the
  // name() of what actually ran.
  std::vector<Decision> decisions{};
};

// The decisions of the time settings, as user choices.
inline std::vector<Decision> time_decisions(const TimePlan &time) {
  return {user_decision("time integrator",
                        time.integrator ? std::string()
                                        : time_method_name(time.method),
                        time.integrator ? "TimeOptions::integrator"
                                        : "TimeOptions::method"),
          user_decision("time steps",
                        time.step_controller
                            ? std::string()
                            : UniformStepController::describe(time.step),
                        time.step_controller ? "TimeOptions::step_controller"
                                             : "TimeOptions::step")};
}

// A stationary plan of the user's options.
inline SolvePlan plan_from_user(const AssemblyOptions &assembly,
                                const SolverOptions &solver = {}) {
  return {assembly, solver, std::nullopt, solver_decisions(solver)};
}
// A transient plan of the user's options.
inline SolvePlan plan_from_user(const AssemblyOptions &assembly,
                                const SolverOptions &solver,
                                const TimePlan &time) {
  auto decisions = time_decisions(time);
  for (auto &decision : solver_decisions(solver))
    decisions.push_back(std::move(decision));
  return {assembly, solver, time, std::move(decisions)};
}
} // namespace pde2d::fem
