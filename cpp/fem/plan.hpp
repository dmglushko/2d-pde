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
// SolvePlan is the one output type of a planner (SolvePlanner, planner.hpp)
// and the one place to override it: a planner returns a SolvePlan whose
// decisions use DecisionSource::Automatic, and a user overrides a setting
// with a setter (plan.set_linear_solver(...), set_time_step(...)), which
// changes the option and its decision together.
//
// Every subject a solve plan owns (DecisionOwner::Solve and Time) has one
// row in solve_plan_bindings: its option, how to read its choice from the
// plan and whether plan_from_user records it. The rows of the solver
// settings are solver_option_bindings (linear_solver_types.hpp); the time
// and assembly rows are below. plan_from_user, the setters (update) and
// validate_decisions all derive from that table, so a new setting is one
// row (plus a setter if wanted). Solves call validate_decisions at entry: a
// decision that no longer matches its option (an option edited without its
// decision) throws "Stale decision: ...". plan_from_user records the user's
// options exactly as the option-based entry points do, so both paths give
// identical results.
//
// A plan may also carry the mesh plan of its mesh (mesh_plan.hpp), the
// analysis its decisions were based on (AnalysisRecord, bound to the
// problem and mesh it was made for) and the planner that produced it; run
// records include all three.
#include "fem/analysis_types.hpp"
#include "fem/assembly_options.hpp"
#include "fem/decision.hpp"
#include "fem/linear_solver_types.hpp"
#include "fem/mesh_plan.hpp"
#include "fem/step_control.hpp"
#include "fem/time_integration.hpp"
#include <array>
#include <cstddef>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
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

namespace analysis_detail {
struct RecordFactory;
}
// The analysis a plan's decisions were based on (analysis.hpp), bound to the
// inputs it was made for: the problem fingerprint, the mesh fingerprint
// when the analysis sampled a mesh, and the sampling configuration. Only
// make_analysis_record (analysis.hpp) creates one, so the fingerprints
// always describe the properties. Solves reject a plan whose analysis was
// made for another problem or mesh ("Stale analysis"); refresh it with
// make_analysis_record on the current inputs (adapt() does so when the mesh
// changes).
class AnalysisRecord {
  AnalysisOptions options_{};
  ProblemProperties properties_{};
  std::string problem_fingerprint_{};
  std::optional<std::string> mesh_fingerprint_{};
  AnalysisRecord() = default;
  friend struct analysis_detail::RecordFactory;

public:
  // The sampling configuration (quadrature degree, sample times).
  const AnalysisOptions &options() const noexcept { return options_; }
  const ProblemProperties &properties() const noexcept { return properties_; }
  // problem_fingerprint (fingerprint.hpp) of the analyzed problem.
  const std::string &problem_fingerprint() const noexcept {
    return problem_fingerprint_;
  }
  // discretization::mesh_fingerprint of the sampled mesh; empty for an
  // analysis without a mesh, which holds for every mesh.
  const std::optional<std::string> &mesh_fingerprint() const noexcept {
    return mesh_fingerprint_;
  }
};

// The analysis was made for these inputs: the same problem and, when it
// sampled a mesh, the same mesh. Throws std::invalid_argument ("Stale
// analysis: ..."); refresh with refresh_analysis or make_analysis_record
// (analysis.hpp).
inline void require_current_analysis(const AnalysisRecord &analysis,
                                     const std::string &problem_fingerprint,
                                     const std::string &mesh_fingerprint) {
  if (analysis.problem_fingerprint() != problem_fingerprint)
    throw std::invalid_argument(
        "Stale analysis: the plan's analysis was made for problem " +
        analysis.problem_fingerprint() + ", not for this problem (" +
        problem_fingerprint +
        "); refresh it with refresh_analysis or "
        "make_analysis_record");
  if (analysis.mesh_fingerprint() &&
      *analysis.mesh_fingerprint() != mesh_fingerprint)
    throw std::invalid_argument(
        "Stale analysis: the plan's analysis sampled mesh " +
        *analysis.mesh_fingerprint() + ", not this mesh (" + mesh_fingerprint +
        "); refresh it with refresh_analysis or make_analysis_record");
}

struct SolvePlan;
namespace plan_detail {
inline void record_decision(SolvePlan &plan, DecisionSubject subject,
                            const Rationale &why);
}

struct SolvePlan {
  AssemblyOptions assembly{};
  SolverOptions solver{};
  // Present for evolutions; evolve() rejects a plan without it.
  std::optional<TimePlan> time{};
  // Why each setting has its value, in table order: for evolutions time
  // integrator, time steps, then linear solver, preconditioner, solver
  // validation. A choice made by a strategy object (an integrator or step
  // controller factory) is named by that object: evolve() sets those two
  // records' `choice` to the name() of what actually ran (empty until then).
  std::vector<Decision> decisions{};
  // The mesh plan of the mesh this plan runs on, when known. Solves given a
  // MeshResult check it against the result's options.
  std::optional<MeshPlan> mesh{};
  // The analysis a planner based its decisions on, when one ran.
  std::optional<AnalysisRecord> analysis{};
  // The planner that produced the plan (planner.hpp), when one did.
  std::optional<StrategyRecord> planner{};

  // Change options with `change(plan)` and record the decision about
  // `subject` from the result, with `why` (default: a user choice naming
  // the option; a planner passes its basis, evidence and
  // DecisionSource::Automatic). The plan is unchanged if either step throws.
  // Throws std::invalid_argument for a subject a solve plan does not own and
  // for a time subject in a plan without time settings.
  template <class Change>
  SolvePlan &update(DecisionSubject subject, Change &&change,
                    const Rationale &why = {}) {
    SolvePlan next = *this;
    std::forward<Change>(change)(next);
    plan_detail::record_decision(next, subject, why);
    *this = std::move(next);
    return *this;
  }

  SolvePlan &set_linear_solver(LinearSolver method, const Rationale &why = {}) {
    return update(
        DecisionSubject::LinearSolver,
        [&](SolvePlan &p) {
          p.solver.method = method;
          p.solver.backend = nullptr;
        },
        why);
  }
  SolvePlan &set_linear_solver_backend(
      std::shared_ptr<const LinearSolverBackendFactory> backend,
      const Rationale &why = {}) {
    if (!backend)
      throw std::invalid_argument("Linear solver backend needs a factory");
    return update(
        DecisionSubject::LinearSolver,
        [&](SolvePlan &p) { p.solver.backend = std::move(backend); }, why);
  }
  SolvePlan &set_preconditioner(Preconditioner preconditioner,
                                const Rationale &why = {}) {
    return update(
        DecisionSubject::Preconditioner,
        [&](SolvePlan &p) { p.solver.preconditioner = preconditioner; }, why);
  }
  SolvePlan &set_solver_validation(SolverValidation validation,
                                   const Rationale &why = {}) {
    return update(
        DecisionSubject::SolverValidation,
        [&](SolvePlan &p) { p.solver.validation = validation; }, why);
  }
  SolvePlan &set_quadrature_degree(int degree, const Rationale &why = {}) {
    return update(
        DecisionSubject::Quadrature,
        [&](SolvePlan &p) { p.assembly.quadrature_degree = degree; }, why);
  }
  SolvePlan &set_time_method(TimeMethod method, const Rationale &why = {}) {
    return update(
        DecisionSubject::TimeIntegrator,
        [&](SolvePlan &p) {
          auto &t = p.time_settings();
          t.method = method;
          t.integrator = {};
        },
        why);
  }
  // The choice stays empty until evolve() names what the factory created.
  SolvePlan &set_time_integrator(TimeIntegratorFactory integrator,
                                 const Rationale &why = {}) {
    if (!integrator)
      throw std::invalid_argument("Time integrator override needs a factory");
    return update(
        DecisionSubject::TimeIntegrator,
        [&](SolvePlan &p) {
          p.time_settings().integrator = std::move(integrator);
        },
        why);
  }
  SolvePlan &set_time_step(double step, const Rationale &why = {}) {
    return update(
        DecisionSubject::TimeSteps,
        [&](SolvePlan &p) {
          auto &t = p.time_settings();
          t.step = step;
          t.step_controller = {};
        },
        why);
  }
  SolvePlan &set_step_controller(StepControllerFactory controller,
                                 const Rationale &why = {}) {
    if (!controller)
      throw std::invalid_argument("Step controller override needs a factory");
    return update(
        DecisionSubject::TimeSteps,
        [&](SolvePlan &p) {
          p.time_settings().step_controller = std::move(controller);
        },
        why);
  }

private:
  TimePlan &time_settings() {
    if (!time)
      throw std::invalid_argument("Plan has no time settings");
    return *time;
  }
};

namespace plan_detail {
template <std::size_t I> constexpr DecisionBinding<SolvePlan> solver_row() {
  return {solver_option_bindings[I].subject,
          [](const SolvePlan &p) {
            return solver_option_bindings[I].option(p.solver);
          },
          [](const SolvePlan &p) {
            return solver_option_bindings[I].choice(p.solver);
          },
          [](const SolvePlan &p) {
            return solver_option_bindings[I].recorded(p.solver);
          }};
}
template <std::size_t... I>
constexpr std::array<DecisionBinding<SolvePlan>, sizeof...(I)>
solver_rows(std::index_sequence<I...>) {
  return {solver_row<I>()...};
}
} // namespace plan_detail

// The rows of the time settings (DecisionOwner::Time; recorded only in a
// plan with time settings). A factory-made integrator or step controller
// leaves the choice to the object's name() (nullopt here).
inline constexpr std::array<DecisionBinding<SolvePlan>, 2> time_plan_bindings{{
    {DecisionSubject::TimeIntegrator,
     [](const SolvePlan &p) -> std::string_view {
       return p.time && p.time->integrator ? "TimeOptions::integrator"
                                           : "TimeOptions::method";
     },
     [](const SolvePlan &p) -> std::optional<std::string> {
       if (!p.time || p.time->integrator)
         return std::nullopt;
       return time_method_name(p.time->method);
     },
     [](const SolvePlan &p) { return p.time.has_value(); }},
    {DecisionSubject::TimeSteps,
     [](const SolvePlan &p) -> std::string_view {
       return p.time && p.time->step_controller ? "TimeOptions::step_controller"
                                                : "TimeOptions::step";
     },
     [](const SolvePlan &p) -> std::optional<std::string> {
       if (!p.time || p.time->step_controller)
         return std::nullopt;
       return UniformStepController::describe(p.time->step);
     },
     [](const SolvePlan &p) { return p.time.has_value(); }},
}};
// The rows of the assembly settings. The quadrature degree is recorded only
// by its setter, so the decisions of plain options are unchanged.
inline constexpr std::array<DecisionBinding<SolvePlan>, 1>
    assembly_plan_bindings{{
        {DecisionSubject::Quadrature,
         [](const SolvePlan &) -> std::string_view {
           return "AssemblyOptions::quadrature_degree";
         },
         [](const SolvePlan &p) -> std::optional<std::string> {
           return std::to_string(p.assembly.quadrature_degree);
         },
         [](const SolvePlan &) { return false; }},
    }};
// The binding table of a solve plan: time, solver and assembly rows, in
// subject order.
inline constexpr auto solve_plan_bindings = decision_detail::concat(
    decision_detail::concat(
        time_plan_bindings,
        plan_detail::solver_rows(
            std::make_index_sequence<solver_option_bindings.size()>{})),
    assembly_plan_bindings);
static_assert(decision_detail::binds_exactly(solve_plan_bindings,
                                             {DecisionOwner::Solve,
                                              DecisionOwner::Time}),
              "solve_plan_bindings needs one row per DecisionOwner::Solve "
              "and Time subject and no other");

namespace plan_detail {
inline void require_time(const SolvePlan &plan, DecisionSubject subject) {
  if (decision_subject_owner(subject) == DecisionOwner::Time && !plan.time)
    throw std::invalid_argument("Plan has no time settings");
}
inline void record_decision(SolvePlan &plan, DecisionSubject subject,
                            const Rationale &why) {
  if (decision_detail::find_binding(solve_plan_bindings, subject))
    require_time(plan, subject);
  record_bound_decision(solve_plan_bindings, plan, plan.decisions, subject, why,
                        "solve plan");
}
inline std::vector<Decision> decisions_of(const SolvePlan &plan,
                                          DecisionOwner owner) {
  std::vector<Decision> out;
  for (auto &d : bound_user_decisions(solve_plan_bindings, plan))
    if (decision_subject_owner(d.subject) == owner)
      out.push_back(std::move(d));
  return out;
}
} // namespace plan_detail

// The decisions of the time settings, as user choices.
inline std::vector<Decision> time_decisions(const TimePlan &time) {
  SolvePlan plan;
  plan.time = time;
  return plan_detail::decisions_of(plan, DecisionOwner::Time);
}

// Every decision of the plan is owned by a solve plan, appears once and
// matches the option it explains; the mesh plan, if any, is valid too.
// Solves call this at entry. A decision about a factory-made strategy may
// be empty (named after running) or must equal the name in `ran`; before
// the strategy exists (no name in `ran`) it is not compared. Throws
// std::invalid_argument ("Stale decision: ...", "Duplicate decision: ...",
// "... does not belong to a solve plan ...").
inline void validate_decisions(const SolvePlan &plan,
                               const std::vector<StrategyRecord> &ran = {}) {
  for (const auto &decision : plan.decisions)
    if (decision_subject_owner(decision.subject) == DecisionOwner::Time &&
        !plan.time)
      throw std::invalid_argument(
          "Stale decision: " +
          std::string(decision_subject_name(decision.subject)) +
          " recorded for a plan without time settings");
  validate_bound_decisions(solve_plan_bindings, plan, plan.decisions, ran,
                           "solve plan", [](DecisionSubject subject) {
                             return std::string(
                                 decision_subject_owner(subject) ==
                                         DecisionOwner::Mesh
                                     ? "; use SolvePlan::mesh"
                                     : "");
                           });
  if (plan.mesh)
    validate_decisions(*plan.mesh);
}

// A stationary plan of the user's options.
inline SolvePlan plan_from_user(const AssemblyOptions &assembly,
                                const SolverOptions &solver = {}) {
  SolvePlan plan{assembly, solver};
  plan.decisions = bound_user_decisions(solve_plan_bindings, plan);
  return plan;
}
// A transient plan of the user's options.
inline SolvePlan plan_from_user(const AssemblyOptions &assembly,
                                const SolverOptions &solver,
                                const TimePlan &time) {
  SolvePlan plan{assembly, solver, time};
  plan.decisions = bound_user_decisions(solve_plan_bindings, plan);
  return plan;
}
} // namespace pde2d::fem
