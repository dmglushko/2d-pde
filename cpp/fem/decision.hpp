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
// together with the SolvePlan setters.
//
// Today every decision is the user's explicit choice: source User, basis
// "user-selected (<option>)" and evidence Unknown, because nothing checked
// that the choice suits the problem. An automatic chooser would use source
// Automatic, state its basis (e.g. "operator proven symmetric") and the
// evidence of that basis.
#include "fem/evidence.hpp"
#include <cstddef>
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

// What a decision is about. Solve settings (SolvePlan) and mesh settings
// (MeshPlan) have one subject each; validate_decisions knows, for every
// subject, which option it explains. Names are stable: reports and run
// record JSON use them.
enum class DecisionSubject {
  // Solve plan (plan.hpp).
  TimeIntegrator,
  TimeSteps,
  LinearSolver,
  Preconditioner,
  SolverValidation,
  Quadrature,
  // Mesh plan (mesh_plan.hpp).
  MeshAlgorithm,
  BoundaryTreatment,
  MeshSize,
  SizeField,
  // A mesh chosen as a whole (for example "graded" by a preprocessor); it
  // explains MeshPlan::options together, not one field.
  Mesh,
  // A user refinement criterion (RefinementOptions::criterion; mesh plan).
  RefinementCriterion,
  // Adaptive loop (adaptivity.hpp): which elements an iteration marks and
  // how the marked mesh is refined. Recorded in AdaptiveIteration, never in
  // a SolvePlan or MeshPlan.
  Marking,
  Refinement
};
inline constexpr DecisionSubject decision_subjects[] = {
    DecisionSubject::TimeIntegrator,
    DecisionSubject::TimeSteps,
    DecisionSubject::LinearSolver,
    DecisionSubject::Preconditioner,
    DecisionSubject::SolverValidation,
    DecisionSubject::Quadrature,
    DecisionSubject::MeshAlgorithm,
    DecisionSubject::BoundaryTreatment,
    DecisionSubject::MeshSize,
    DecisionSubject::SizeField,
    DecisionSubject::Mesh,
    DecisionSubject::RefinementCriterion,
    DecisionSubject::Marking,
    DecisionSubject::Refinement};

inline std::string_view decision_subject_name(DecisionSubject subject) {
  switch (subject) {
  case DecisionSubject::TimeIntegrator:
    return "time integrator";
  case DecisionSubject::TimeSteps:
    return "time steps";
  case DecisionSubject::LinearSolver:
    return "linear solver";
  case DecisionSubject::Preconditioner:
    return "preconditioner";
  case DecisionSubject::SolverValidation:
    return "solver validation";
  case DecisionSubject::Quadrature:
    return "quadrature";
  case DecisionSubject::MeshAlgorithm:
    return "mesh algorithm";
  case DecisionSubject::BoundaryTreatment:
    return "boundary treatment";
  case DecisionSubject::MeshSize:
    return "mesh size";
  case DecisionSubject::SizeField:
    return "size field";
  case DecisionSubject::Mesh:
    return "mesh";
  case DecisionSubject::RefinementCriterion:
    return "refinement criterion";
  case DecisionSubject::Marking:
    return "marking";
  case DecisionSubject::Refinement:
    return "refinement";
  }
  return "invalid";
}
// The subject with this name, if any.
inline std::optional<DecisionSubject>
decision_subject_from_name(std::string_view name) {
  for (const auto subject : decision_subjects)
    if (decision_subject_name(subject) == name)
      return subject;
  return std::nullopt;
}
// Compatibility with string subjects: `decision.subject == "linear solver"`
// compares the printed name.
inline bool operator==(DecisionSubject subject, std::string_view name) {
  return decision_subject_name(subject) == name;
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
// Deprecated: string subjects. Throws std::invalid_argument for a name that
// is no DecisionSubject.
inline Decision user_decision(std::string_view subject, std::string choice,
                              const std::string &option) {
  const auto key = decision_subject_from_name(subject);
  if (!key)
    throw std::invalid_argument("Unknown decision subject: " +
                                std::string(subject));
  return user_decision(*key, std::move(choice), option);
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
} // namespace decision_detail
} // namespace pde2d::fem
