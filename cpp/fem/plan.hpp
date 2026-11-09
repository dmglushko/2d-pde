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
// decisions use DecisionSource::Automatic, and a user overrides a setting
// with a setter (plan.set_linear_solver(...), set_time_step(...)), which
// changes the option and its decision together. Solves call
// validate_decisions at entry: a decision that no longer matches its option
// (an option edited without its decision) throws "Stale decision: ...".
// Nothing chooses automatically yet: plan_from_user records the user's
// options exactly as the option-based entry points do, so both paths give
// identical results.
//
// A plan may also carry the mesh plan of its mesh (mesh_plan.hpp) and the
// analysis a chooser based its decisions on; run records include both.
#include "fem/analysis_types.hpp"
#include "fem/assembly.hpp"
#include "fem/decision.hpp"
#include "fem/linear_solver_types.hpp"
#include "fem/mesh_plan.hpp"
#include "fem/step_control.hpp"
#include "fem/time_integration.hpp"
#include <cstddef>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace pde2d::fem {
// The time discretization of a run: the single source of truth for time
// settings. TimeOptions (evolution.hpp) is a TimePlan plus output, assembly
// and solver settings, so a new time setting is added here only.
struct TimePlan {
  TimeMethod method = TimeMethod::BackwardEuler;
  double start = 0, end = 1, step = .01;
  // A user scheme. When set, evolve() calls it once and ignores `method`.
  TimeIntegratorFactory integrator{};
  // A user step sequence on [start, end]. When set, evolve() calls it once
  // and ignores `step`; otherwise steps are uniform (UniformStepController).
  StepControllerFactory step_controller{};
  // At most this many trials in a row may be rejected; one more throws
  // std::runtime_error. 0 forbids rejections. Counts reset on acceptance.
  std::size_t max_consecutive_rejections = default_max_consecutive_rejections;
};

// The analysis a plan's decisions were based on (analysis.hpp).
struct AnalysisRecord {
  AnalysisOptions options{};
  ProblemProperties properties{};
};

struct SolvePlan {
  AssemblyOptions assembly{};
  SolverOptions solver{};
  // Present for evolutions; evolve() rejects a plan without it.
  std::optional<TimePlan> time{};
  // Why each setting has its value, in a fixed order: for evolutions
  // time integrator, time steps, then the solver_decisions(). A choice made
  // by a strategy object (an integrator or step controller factory) is named
  // by that object: evolve() sets those two records' `choice` to the name()
  // of what actually ran (empty until then).
  std::vector<Decision> decisions{};
  // The mesh plan of the mesh this plan runs on, when known. Solves given a
  // MeshResult check it against the result's options.
  std::optional<MeshPlan> mesh{};
  // The analysis a chooser based its decisions on, when one ran.
  std::optional<AnalysisRecord> analysis{};

  // Overrides: each changes one option and its decision together. The
  // default rationale records a user choice naming the option; a chooser
  // passes its basis, evidence and DecisionSource::Automatic.
  SolvePlan &set_linear_solver(LinearSolver method, const Rationale &why = {}) {
    solver.method = method;
    solver.backend = nullptr;
    set_decision(decisions, decision_for(DecisionSubject::LinearSolver,
                                         linear_solver_name(solver),
                                         "SolverOptions::method", why));
    return *this;
  }
  SolvePlan &set_linear_solver_backend(
      std::shared_ptr<const LinearSolverBackendFactory> backend,
      const Rationale &why = {}) {
    if (!backend)
      throw std::invalid_argument("Linear solver backend needs a factory");
    solver.backend = std::move(backend);
    set_decision(decisions, decision_for(DecisionSubject::LinearSolver,
                                         linear_solver_name(solver),
                                         "SolverOptions::backend", why));
    return *this;
  }
  SolvePlan &set_preconditioner(Preconditioner preconditioner,
                                const Rationale &why = {}) {
    solver.preconditioner = preconditioner;
    set_decision(decisions, decision_for(DecisionSubject::Preconditioner,
                                         preconditioner_name(preconditioner),
                                         "SolverOptions::preconditioner", why));
    return *this;
  }
  SolvePlan &set_solver_validation(SolverValidation validation,
                                   const Rationale &why = {}) {
    solver.validation = validation;
    set_decision(decisions, decision_for(DecisionSubject::SolverValidation,
                                         solver_validation_name(validation),
                                         "SolverOptions::validation", why));
    return *this;
  }
  SolvePlan &set_quadrature_degree(int degree, const Rationale &why = {}) {
    assembly.quadrature_degree = degree;
    set_decision(decisions,
                 decision_for(DecisionSubject::Quadrature,
                              std::to_string(degree),
                              "AssemblyOptions::quadrature_degree", why));
    return *this;
  }
  SolvePlan &set_time_method(TimeMethod method, const Rationale &why = {}) {
    auto &t = time_settings();
    t.method = method;
    t.integrator = {};
    set_decision(decisions, decision_for(DecisionSubject::TimeIntegrator,
                                         time_method_name(method),
                                         "TimeOptions::method", why));
    return *this;
  }
  // The choice stays empty until evolve() names what the factory created.
  SolvePlan &set_time_integrator(TimeIntegratorFactory integrator,
                                 const Rationale &why = {}) {
    if (!integrator)
      throw std::invalid_argument("Time integrator override needs a factory");
    time_settings().integrator = std::move(integrator);
    set_decision(decisions,
                 decision_for(DecisionSubject::TimeIntegrator, std::string(),
                              "TimeOptions::integrator", why));
    return *this;
  }
  SolvePlan &set_time_step(double step, const Rationale &why = {}) {
    auto &t = time_settings();
    t.step = step;
    t.step_controller = {};
    set_decision(decisions, decision_for(DecisionSubject::TimeSteps,
                                         UniformStepController::describe(step),
                                         "TimeOptions::step", why));
    return *this;
  }
  SolvePlan &set_step_controller(StepControllerFactory controller,
                                 const Rationale &why = {}) {
    if (!controller)
      throw std::invalid_argument("Step controller override needs a factory");
    time_settings().step_controller = std::move(controller);
    set_decision(decisions,
                 decision_for(DecisionSubject::TimeSteps, std::string(),
                              "TimeOptions::step_controller", why));
    return *this;
  }

private:
  TimePlan &time_settings() {
    if (!time)
      throw std::invalid_argument("Plan has no time settings");
    return *time;
  }
};

// The decisions of the time settings, as user choices.
inline std::vector<Decision> time_decisions(const TimePlan &time) {
  return {user_decision(DecisionSubject::TimeIntegrator,
                        time.integrator ? std::string()
                                        : time_method_name(time.method),
                        time.integrator ? "TimeOptions::integrator"
                                        : "TimeOptions::method"),
          user_decision(DecisionSubject::TimeSteps,
                        time.step_controller
                            ? std::string()
                            : UniformStepController::describe(time.step),
                        time.step_controller ? "TimeOptions::step_controller"
                                             : "TimeOptions::step")};
}

// The names of the strategy objects a run created (TimeIntegrator::name(),
// StepController::name()), for validating factory-made choices.
struct StrategyNames {
  std::optional<std::string> integrator{}, step_controller{};
};

// Every decision of the plan is about a solve setting, appears once and
// matches the option it explains; the mesh plan, if any, is valid too.
// Solves call this at entry. A decision about a factory-made strategy may
// be empty (named after running) or must equal the name() in `ran`; before
// the strategy exists (no name in `ran`) it is not compared. Throws
// std::invalid_argument ("Stale decision: ...", "Duplicate decision: ...").
inline void validate_decisions(const SolvePlan &plan,
                               const StrategyNames &ran = {}) {
  using decision_detail::require_current;
  decision_detail::require_unique(plan.decisions);
  auto strategy = [](const Decision &decision, bool from_factory,
                     const std::optional<std::string> &name,
                     const std::string &builtin) {
    if (!from_factory)
      require_current(decision, builtin);
    else if (name && !decision.choice.empty())
      require_current(decision, *name);
  };
  for (const auto &decision : plan.decisions) {
    const auto subject = decision.subject;
    switch (subject) {
    case DecisionSubject::LinearSolver:
      require_current(decision, linear_solver_name(plan.solver));
      break;
    case DecisionSubject::Preconditioner:
      require_current(decision,
                      preconditioner_name(plan.solver.preconditioner));
      break;
    case DecisionSubject::SolverValidation:
      require_current(decision, solver_validation_name(plan.solver.validation));
      break;
    case DecisionSubject::Quadrature:
      require_current(decision,
                      std::to_string(plan.assembly.quadrature_degree));
      break;
    case DecisionSubject::TimeIntegrator:
    case DecisionSubject::TimeSteps: {
      if (!plan.time)
        throw std::invalid_argument(
            "Stale decision: " + std::string(decision_subject_name(subject)) +
            " recorded for a plan without time settings");
      const auto &t = *plan.time;
      if (subject == DecisionSubject::TimeIntegrator)
        strategy(decision, static_cast<bool>(t.integrator), ran.integrator,
                 time_method_name(t.method));
      else
        strategy(decision, static_cast<bool>(t.step_controller),
                 ran.step_controller, UniformStepController::describe(t.step));
      break;
    }
    default:
      throw std::invalid_argument(
          "Decision subject " + std::string(decision_subject_name(subject)) +
          " does not belong to a solve plan (use SolvePlan::mesh)");
    }
  }
  if (plan.mesh)
    validate_decisions(*plan.mesh);
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
