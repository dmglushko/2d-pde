#pragma once
// Explained decisions (docs/vision.md, "Decisions are explained").
//
// A Decision records one choice that shaped a solve: what was decided (a
// typed DecisionSubject), the value chosen, what the choice was based on and
// how well that basis is established. Plans carry them (SolvePlan, MeshPlan;
// plan.hpp, mesh_plan.hpp), results copy them (SolverReport::decisions,
// Evolution::record) and write_report() prints them, so automatic choices
// added later are inspectable in the same place as user choices.
//
// A decision must describe the option it explains: solves check every
// recorded decision against the options that run (validate_decisions in
// plan.hpp) and throw on a stale one. Change an option and its decision
// together with the plan setters (SolvePlan::update, MeshPlan::update).
//
// Subjects are listed once, in decision_subject_table (stable name and
// owner). How each owned subject is read from a plan's options is one
// DecisionBinding row in that plan's binding table (solve_plan_bindings,
// mesh_plan_bindings); the user decisions, the setters and validation are
// generic functions of the table (bound_user_decisions,
// record_bound_decision, validate_bound_decisions), so there is no other
// list of subjects to keep in step.
//
// Today every decision is the user's explicit choice: source User, basis
// "user-selected (<option>)" and evidence Unknown, because nothing checked
// that the choice suits the problem. An automatic chooser would use source
// Automatic, state its basis (e.g. "operator proven symmetric") and the
// evidence of that basis.
#include "fem/evidence.hpp"
#include "provenance.hpp"
#include <array>
#include <cstddef>
#include <initializer_list>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace pde2d::fem {
enum class DecisionSource { User, Automatic };

inline const char *decision_source_name(DecisionSource source) {
  switch (source) {
  case DecisionSource::User:
    return "User";
  case DecisionSource::Automatic:
    return "Automatic";
  }
  return "Invalid";
}

// Which plan or record a decision belongs to: the solve settings of a
// SolvePlan (Solve), its time settings (Time; only in a plan with
// SolvePlan::time), the meshing options of a MeshPlan (Mesh), or the
// adaptive drivers and planners (Adapt: AdaptiveStep::decisions, never
// a plan). A decision in the wrong place is rejected ("does not belong").
enum class DecisionOwner { Solve, Time, Mesh, Adapt };
inline const char *decision_owner_name(DecisionOwner owner) {
  switch (owner) {
  case DecisionOwner::Solve:
    return "Solve";
  case DecisionOwner::Time:
    return "Time";
  case DecisionOwner::Mesh:
    return "Mesh";
  case DecisionOwner::Adapt:
    return "Adapt";
  }
  return "Invalid";
}

// What a decision is about. Each subject has one row in
// decision_subject_table below (stable name and owner), and each subject a
// plan owns has one binding in that plan's table (solve_plan_bindings in
// plan.hpp, mesh_plan_bindings in mesh_plan.hpp) saying how to read its
// choice from the options; static_asserts there fail for a subject without
// its binding. Append new values at the end; names are stable (reports and
// run record JSON use them).
enum class DecisionSubject {
  TimeIntegrator,
  TimeSteps,
  LinearSolver,
  Preconditioner,
  SolverValidation,
  Quadrature,
  MeshAlgorithm,
  BoundaryTreatment,
  MeshSize,
  SizeField,
  // A mesh chosen as a whole (for example "graded" by a preprocessor); it
  // explains MeshPlan::options together, not one field.
  Mesh,
  RefinementCriterion,
  // Adaptive loop (adaptivity.hpp): which elements an iteration marks and
  // how the marked mesh is refined.
  Marking,
  Refinement,
  // The planners an adaptive iteration ran (planner.hpp).
  SolvePlanner,
  MeshPlanner
};

struct DecisionSubjectInfo {
  DecisionSubject subject;
  std::string_view name;
  DecisionOwner owner;
};
// One row per subject, in enum order (checked below).
inline constexpr DecisionSubjectInfo decision_subject_table[] = {
    {DecisionSubject::TimeIntegrator, "time integrator", DecisionOwner::Time},
    {DecisionSubject::TimeSteps, "time steps", DecisionOwner::Time},
    {DecisionSubject::LinearSolver, "linear solver", DecisionOwner::Solve},
    {DecisionSubject::Preconditioner, "preconditioner", DecisionOwner::Solve},
    {DecisionSubject::SolverValidation, "solver validation",
     DecisionOwner::Solve},
    {DecisionSubject::Quadrature, "quadrature", DecisionOwner::Solve},
    {DecisionSubject::MeshAlgorithm, "mesh algorithm", DecisionOwner::Mesh},
    {DecisionSubject::BoundaryTreatment, "boundary treatment",
     DecisionOwner::Mesh},
    {DecisionSubject::MeshSize, "mesh size", DecisionOwner::Mesh},
    {DecisionSubject::SizeField, "size field", DecisionOwner::Mesh},
    {DecisionSubject::Mesh, "mesh", DecisionOwner::Mesh},
    {DecisionSubject::RefinementCriterion, "refinement criterion",
     DecisionOwner::Mesh},
    {DecisionSubject::Marking, "marking", DecisionOwner::Adapt},
    {DecisionSubject::Refinement, "refinement", DecisionOwner::Adapt},
    {DecisionSubject::SolvePlanner, "solve planner", DecisionOwner::Adapt},
    {DecisionSubject::MeshPlanner, "mesh planner", DecisionOwner::Adapt}};
inline constexpr std::size_t decision_subject_count =
    std::size(decision_subject_table);

namespace decision_detail {
consteval bool subject_table_in_order() {
  for (std::size_t k = 0; k < decision_subject_count; ++k)
    if (static_cast<std::size_t>(decision_subject_table[k].subject) != k ||
        decision_subject_table[k].name.empty())
      return false;
  for (std::size_t i = 0; i < decision_subject_count; ++i)
    for (std::size_t j = 0; j < i; ++j)
      if (decision_subject_table[i].name == decision_subject_table[j].name)
        return false;
  return true;
}
static_assert(subject_table_in_order(),
              "decision_subject_table needs one row per DecisionSubject, in "
              "enum order, with distinct names");
} // namespace decision_detail

// Every subject, in enum order.
inline constexpr auto decision_subjects = [] {
  std::array<DecisionSubject, decision_subject_count> subjects{};
  for (std::size_t k = 0; k < decision_subject_count; ++k)
    subjects[k] = decision_subject_table[k].subject;
  return subjects;
}();

// The table row of a subject; throws (or fails to compile in a constant
// expression) for a value that has none.
constexpr const DecisionSubjectInfo &
decision_subject_info(DecisionSubject subject) {
  const auto k = static_cast<std::size_t>(subject);
  if (k >= decision_subject_count)
    throw std::invalid_argument("Invalid decision subject");
  return decision_subject_table[k];
}
constexpr std::string_view decision_subject_name(DecisionSubject subject) {
  const auto k = static_cast<std::size_t>(subject);
  return k < decision_subject_count ? decision_subject_table[k].name
                                    : std::string_view("invalid");
}
constexpr DecisionOwner decision_subject_owner(DecisionSubject subject) {
  return decision_subject_info(subject).owner;
}
// The subject with this name, if any.
constexpr std::optional<DecisionSubject>
decision_subject_from_name(std::string_view name) {
  for (const auto &row : decision_subject_table)
    if (row.name == name)
      return row.subject;
  return std::nullopt;
}

struct Decision {
  DecisionSubject subject; // what was decided
  std::string choice;      // the chosen value, e.g. "SparseLU"
  std::string basis;       // why, e.g. "user-selected (SolverOptions::method)"
  // How well the basis supports the choice for this problem.
  Evidence evidence = Evidence::Unknown;
  DecisionSource source = DecisionSource::User;
};

// Why a setter changed a setting: the basis, evidence and source of the
// decision it records. The default is a user override naming the option.
struct Rationale {
  std::string basis{};
  Evidence evidence = Evidence::Unknown;
  DecisionSource source = DecisionSource::User;
};

// A choice taken from the user's options. `option` names the field it came
// from, so the record says where to override it.
inline Decision user_decision(DecisionSubject subject, std::string choice,
                              const std::string &option) {
  return {subject, std::move(choice), "user-selected (" + option + ")",
          Evidence::Unknown, DecisionSource::User};
}
// The decision about `subject` in `decisions`, or nullptr.
inline const Decision *find_decision(const std::vector<Decision> &decisions,
                                     DecisionSubject subject) {
  for (const auto &decision : decisions)
    if (decision.subject == subject)
      return &decision;
  return nullptr;
}
inline Decision *find_decision(std::vector<Decision> &decisions,
                               DecisionSubject subject) {
  for (auto &decision : decisions)
    if (decision.subject == subject)
      return &decision;
  return nullptr;
}
// Replace the decision about `decision.subject`, keeping its position, or
// append it.
inline void set_decision(std::vector<Decision> &decisions, Decision decision) {
  if (auto *existing = find_decision(decisions, decision.subject))
    *existing = std::move(decision);
  else
    decisions.push_back(std::move(decision));
}
// A decision for a setter: the rationale's basis, or "user-selected
// (<option>)" when it has none.
inline Decision decision_for(DecisionSubject subject, std::string choice,
                             const std::string &option, const Rationale &why) {
  if (why.basis.empty() && why.source == DecisionSource::User &&
      why.evidence == Evidence::Unknown)
    return user_decision(subject, std::move(choice), option);
  return {subject, std::move(choice),
          why.basis.empty() ? "selected (" + option + ")" : why.basis,
          why.evidence, why.source};
}

// How one decision subject is read from an options type: one row of a
// plan's binding table (solve_plan_bindings in plan.hpp, built from
// solver_option_bindings in linear_solver_types.hpp; mesh_plan_bindings in
// mesh_plan.hpp). Everything about a setting's decision derives from its
// row: the decisions of the user's options, the setters (SolvePlan::update,
// MeshPlan::update), validation ("Stale decision") and, through the
// subject, the stable name and owner.
template <class Options> struct DecisionBinding {
  DecisionSubject subject;
  // The option field(s) the decision explains, named in a user decision's
  // basis ("user-selected (SolverOptions::method)").
  std::string_view (*option)(const Options &);
  // The choice text the options give. nullopt when the options do not
  // determine it: a strategy a factory creates (named by its name() once it
  // exists, see validate_bound_decisions) or a whole-mesh decision.
  std::optional<std::string> (*choice)(const Options &);
  // The user's options record it (plan_from_user, mesh_plan_from_user);
  // otherwise only a setter or a planner records it.
  bool (*recorded)(const Options &);
};

// Whether a record rebuilds a strategy: from the options alone (built-in),
// from a descriptor only a reader can turn back into an object (described),
// or not at all (not replayable: a factory without a descriptor).
enum class StrategyReplay { BuiltIn, Described, NotReplayable };
inline const char *strategy_replay_name(StrategyReplay replay) {
  switch (replay) {
  case StrategyReplay::BuiltIn:
    return "built-in";
  case StrategyReplay::Described:
    return "described";
  case StrategyReplay::NotReplayable:
    return "not replayable";
  }
  return "invalid";
}
// One strategy object that ran or produced a plan: the decision subject it
// serves (time integrator, time steps, linear solver, size field, refinement
// criterion, solve and mesh planner), its name() and how to rebuild it.
// Run records list them (RunRecord::strategies), plans name their planner
// with one (SolvePlan::planner, MeshPlan::planner), and validation compares
// factory-made strategies' decisions with them.
struct StrategyRecord {
  DecisionSubject subject{};
  std::string name{};
  StrategyReplay replay = StrategyReplay::BuiltIn;
  std::optional<pde2d::StrategyDescriptor> descriptor{};
};
inline StrategyRecord
strategy_record(DecisionSubject subject, std::string name, bool builtin,
                std::optional<pde2d::StrategyDescriptor> descriptor) {
  const auto replay = builtin      ? StrategyReplay::BuiltIn
                      : descriptor ? StrategyReplay::Described
                                   : StrategyReplay::NotReplayable;
  return {subject, std::move(name), replay, std::move(descriptor)};
}
// The strategy that served `subject` in `strategies`, or nullptr.
inline const StrategyRecord *
find_strategy(const std::vector<StrategyRecord> &strategies,
              DecisionSubject subject) {
  for (const auto &s : strategies)
    if (s.subject == subject)
      return &s;
  return nullptr;
}

namespace decision_detail {
template <class Options, std::size_t N>
constexpr const DecisionBinding<Options> *
find_binding(const std::array<DecisionBinding<Options>, N> &table,
             DecisionSubject subject) {
  for (const auto &row : table)
    if (row.subject == subject)
      return &row;
  return nullptr;
}
// The table binds exactly the subjects of `owners`, each once, and no other.
template <class Options, std::size_t N>
consteval bool
binds_exactly(const std::array<DecisionBinding<Options>, N> &table,
              std::initializer_list<DecisionOwner> owners) {
  for (const auto &row : table) {
    if (!row.option || !row.choice || !row.recorded)
      return false;
    bool owned = false;
    for (const auto owner : owners)
      owned = owned || decision_subject_owner(row.subject) == owner;
    if (!owned)
      return false;
  }
  for (const auto &info : decision_subject_table) {
    bool owned = false;
    for (const auto owner : owners)
      owned = owned || info.owner == owner;
    std::size_t rows = 0;
    for (const auto &row : table)
      rows += row.subject == info.subject;
    if (rows != (owned ? 1u : 0u))
      return false;
  }
  return true;
}
template <class Options, std::size_t N, std::size_t M>
constexpr std::array<DecisionBinding<Options>, N + M>
concat(const std::array<DecisionBinding<Options>, N> &a,
       const std::array<DecisionBinding<Options>, M> &b) {
  std::array<DecisionBinding<Options>, N + M> out{};
  for (std::size_t k = 0; k < N; ++k)
    out[k] = a[k];
  for (std::size_t k = 0; k < M; ++k)
    out[N + k] = b[k];
  return out;
}
} // namespace decision_detail

// The decisions the user's options record, in table order.
template <class Options, std::size_t N>
std::vector<Decision>
bound_user_decisions(const std::array<DecisionBinding<Options>, N> &table,
                     const Options &options) {
  std::vector<Decision> decisions;
  for (const auto &row : table)
    if (row.recorded(options))
      decisions.push_back(user_decision(row.subject,
                                        row.choice(options).value_or(""),
                                        std::string(row.option(options))));
  return decisions;
}
// Record the decision about `subject` from the current options (a setter's
// second half). Throws std::invalid_argument for a subject the table does
// not bind.
template <class Options, std::size_t N>
void record_bound_decision(const std::array<DecisionBinding<Options>, N> &table,
                           const Options &options,
                           std::vector<Decision> &decisions,
                           DecisionSubject subject, const Rationale &why,
                           std::string_view plan_kind) {
  const auto *row = decision_detail::find_binding(table, subject);
  if (!row)
    throw std::invalid_argument(
        "Decision subject " + std::string(decision_subject_name(subject)) +
        " does not belong to a " + std::string(plan_kind));
  set_decision(decisions,
               decision_for(subject, row->choice(options).value_or(""),
                            std::string(row->option(options)), why));
}

namespace decision_detail {
// A recorded choice must equal what the options give ("stale" otherwise).
inline void require_current(const Decision &decision,
                            const std::string &actual) {
  if (decision.choice != actual)
    throw std::invalid_argument(
        "Stale decision: " +
        std::string(decision_subject_name(decision.subject)) + " records \"" +
        decision.choice + "\" but the options give \"" + actual + "\"");
}
inline void require_unique(const std::vector<Decision> &decisions) {
  for (std::size_t i = 0; i < decisions.size(); ++i)
    for (std::size_t j = 0; j < i; ++j)
      if (decisions[i].subject == decisions[j].subject)
        throw std::invalid_argument(
            "Duplicate decision: " +
            std::string(decision_subject_name(decisions[i].subject)));
}
// The owner check of a plan: a decision whose subject the table does not
// bind does not belong there ("Decision subject X does not belong to a
// <plan kind> (owner Y<hint>)").
inline void reject_foreign(const Decision &decision, std::string_view plan_kind,
                           std::string_view hint) {
  const auto owner = decision_subject_owner(decision.subject);
  throw std::invalid_argument(
      "Decision subject " +
      std::string(decision_subject_name(decision.subject)) +
      " does not belong to a " + std::string(plan_kind) + " (owner " +
      decision_owner_name(owner) + std::string(hint) + ")");
}
} // namespace decision_detail

// Every decision appears once, is bound by the table (owner check) and
// matches the choice its row reads from `options`. A row whose options do not
// determine the choice (nullopt) is compared with the name of the strategy
// in `ran` that served its subject, if there is one and the decision names a
// choice. `hint(subject)` extends the owner message. Throws
// std::invalid_argument ("Duplicate decision: ...", "... does not belong to a
// ...", "Stale decision: ...").
template <class Options, std::size_t N, class Hint>
void validate_bound_decisions(
    const std::array<DecisionBinding<Options>, N> &table,
    const Options &options, const std::vector<Decision> &decisions,
    const std::vector<StrategyRecord> &ran, std::string_view plan_kind,
    Hint &&hint) {
  decision_detail::require_unique(decisions);
  for (const auto &decision : decisions) {
    const auto *row = decision_detail::find_binding(table, decision.subject);
    if (!row)
      decision_detail::reject_foreign(decision, plan_kind,
                                      hint(decision.subject));
    if (const auto choice = row->choice(options))
      decision_detail::require_current(decision, *choice);
    else if (const auto *strategy = find_strategy(ran, decision.subject);
             strategy && !decision.choice.empty())
      decision_detail::require_current(decision, strategy->name);
  }
}
} // namespace pde2d::fem
