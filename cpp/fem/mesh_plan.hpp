#pragma once
// Mesh plan: the meshing options of a run together with the decisions that
// explain them, the mesh-side counterpart of SolvePlan (plan.hpp). A
// preprocessor that tailors a mesh to the problem (a MeshPlanner,
// planner.hpp) returns a MeshPlan whose decisions use
// DecisionSource::Automatic; the user overrides it with the setters, which
// change an option and its decision together.
//
//   auto mesh_plan = mesh_plan_from_user(options);
//   mesh_plan.set_size_field(h, {"Graded", {{"h0", "0.05"}}});
//   auto result = discretization::generate_mesh(geometry, mesh_plan.options);
//   auto plan = plan_from_user(assembly, solver);
//   plan.mesh = mesh_plan;
//   solve_stationary(result, problem, plan);  // checks plan.mesh against
//                                             // result.options
//
// Every decision subject a mesh plan owns (DecisionOwner::Mesh) has one row in
// mesh_plan_bindings: its option, how to read its choice from MeshOptions and
// whether the user's options record it. mesh_decisions, the setters
// (MeshPlan::update) and validate_decisions all derive from that table.
//
// It lives in the FEM module because decisions do (decision.hpp); meshing
// itself only sees discretization::MeshOptions, so the dependency direction
// (fem includes discretization, never the reverse) is kept.
#include "discretization/mesh_types.hpp"
#include "fem/decision.hpp"
#include "provenance.hpp"
#include <array>
#include <format>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace pde2d::fem {
// The choice text of the size field and refinement criterion decisions:
// "none", the descriptor, or "undescribed (not replayable)". Numbers in mesh
// choices use the shortest round-trip format, so any change of an option
// changes its text.
inline std::string
size_field_choice(const discretization::RefinementOptions &o) {
  if (!o.size_field)
    return "none";
  if (!o.size_field_descriptor)
    return "undescribed (not replayable)";
  return pde2d::describe_strategy(*o.size_field_descriptor);
}
inline std::string
refinement_criterion_choice(const discretization::RefinementOptions &o) {
  if (!o.criterion)
    return "none";
  if (!o.criterion->descriptor)
    return "undescribed (not replayable)";
  return pde2d::describe_strategy(*o.criterion->descriptor);
}

// The binding table of a mesh plan, in subject order.
inline constexpr std::array<DecisionBinding<discretization::MeshOptions>, 6>
    mesh_plan_bindings{{
        {DecisionSubject::MeshAlgorithm,
         [](const discretization::MeshOptions &) -> std::string_view {
           return "MeshOptions::algorithm";
         },
         [](const discretization::MeshOptions &o)
             -> std::optional<std::string> {
           return discretization::interior_algorithm_name(o.algorithm);
         },
         [](const discretization::MeshOptions &) { return true; }},
        {DecisionSubject::BoundaryTreatment,
         [](const discretization::MeshOptions &) -> std::string_view {
           return "MeshOptions::boundary_treatment";
         },
         [](const discretization::MeshOptions &o)
             -> std::optional<std::string> {
           return std::format(
               "{} preserve_corners={} preserve_regions={}",
               discretization::boundary_treatment_name(o.boundary_treatment),
               o.boundary.preserve_corners, o.preserve_regions);
         },
         [](const discretization::MeshOptions &) { return true; }},
        {DecisionSubject::MeshSize,
         [](const discretization::MeshOptions &) -> std::string_view {
           return "MeshOptions::boundary, MeshOptions::refinement";
         },
         [](const discretization::MeshOptions &o)
             -> std::optional<std::string> {
           return std::format(
               "boundary_max_edge_length={} "
               "max_curve_deviation={} "
               "refinement_max_edge_length={} max_steps={}",
               o.boundary.max_edge_length, o.boundary.max_curve_deviation,
               o.refinement.max_edge_length, o.refinement.max_steps);
         },
         [](const discretization::MeshOptions &) { return true; }},
        {DecisionSubject::SizeField,
         [](const discretization::MeshOptions &) -> std::string_view {
           return "RefinementOptions::size_field";
         },
         [](const discretization::MeshOptions &o)
             -> std::optional<std::string> {
           return size_field_choice(o.refinement);
         },
         [](const discretization::MeshOptions &) { return true; }},
        // The mesh as a whole: no single option to compare, never recorded
        // from options (a preprocessor records it).
        {DecisionSubject::Mesh,
         [](const discretization::MeshOptions &) -> std::string_view {
           return "MeshPlan::options";
         },
         [](const discretization::MeshOptions &) -> std::optional<std::string> {
           return std::nullopt;
         },
         [](const discretization::MeshOptions &) { return false; }},
        // Recorded only when a criterion is set, so the decisions of plain
        // options are unchanged.
        {DecisionSubject::RefinementCriterion,
         [](const discretization::MeshOptions &) -> std::string_view {
           return "RefinementOptions::criterion";
         },
         [](const discretization::MeshOptions &o)
             -> std::optional<std::string> {
           return refinement_criterion_choice(o.refinement);
         },
         [](const discretization::MeshOptions &o) {
           return o.refinement.criterion.has_value();
         }},
    }};
static_assert(decision_detail::binds_exactly(mesh_plan_bindings,
                                             {DecisionOwner::Mesh}),
              "mesh_plan_bindings needs one row per DecisionOwner::Mesh "
              "subject and no other");

// The choice text of one mesh decision from the options; nullopt for the
// whole-mesh decision. Throws for a subject a mesh plan does not own.
inline std::optional<std::string>
mesh_decision_choice(const discretization::MeshOptions &o,
                     DecisionSubject subject) {
  const auto *row = decision_detail::find_binding(mesh_plan_bindings, subject);
  if (!row)
    throw std::invalid_argument("Decision subject " +
                                std::string(decision_subject_name(subject)) +
                                " does not belong to a mesh plan");
  return row->choice(o);
}

// The user's meshing options as decision records, in table order.
inline std::vector<Decision>
mesh_decisions(const discretization::MeshOptions &o) {
  return bound_user_decisions(mesh_plan_bindings, o);
}

struct MeshPlan {
  discretization::MeshOptions options{};
  // Why each option has its value, in table order (mesh_decisions),
  // optionally a whole-mesh decision.
  std::vector<Decision> decisions{};
  // The planner that produced this plan (planner.hpp), when one did.
  std::optional<PlannerRecord> planner{};

  // Change options with `change(options)` and record the decision about
  // `subject` from the result, with `why` (default: a user choice naming the
  // option). The plan is unchanged if either step throws. Throws
  // std::invalid_argument for a subject a mesh plan does not own.
  template <class Change>
  MeshPlan &update(DecisionSubject subject, Change &&change,
                   const Rationale &why = {}) {
    auto next = options;
    std::forward<Change>(change)(next);
    auto recorded = decisions;
    record_bound_decision(mesh_plan_bindings, next, recorded, subject, why,
                          "mesh plan");
    options = std::move(next);
    decisions = std::move(recorded);
    return *this;
  }
  MeshPlan &set_interior_algorithm(discretization::InteriorAlgorithm algorithm,
                                   const Rationale &why = {}) {
    return update(
        DecisionSubject::MeshAlgorithm,
        [&](discretization::MeshOptions &o) { o.algorithm = algorithm; }, why);
  }
  MeshPlan &set_boundary_treatment(discretization::BoundaryTreatment treatment,
                                   const Rationale &why = {}) {
    return update(
        DecisionSubject::BoundaryTreatment,
        [&](discretization::MeshOptions &o) {
          o.boundary_treatment = treatment;
        },
        why);
  }
  // The constant interior target RefinementOptions::max_edge_length (0
  // disables it).
  MeshPlan &set_max_edge_length(double length, const Rationale &why = {}) {
    return update(
        DecisionSubject::MeshSize,
        [&](discretization::MeshOptions &o) {
          o.refinement.max_edge_length = length;
        },
        why);
  }
  // A graded target with the descriptor that names it in records (a
  // planner's size field must be replayable, so the descriptor is required).
  MeshPlan &set_size_field(discretization::SizeField field,
                           pde2d::StrategyDescriptor descriptor,
                           const Rationale &why = {}) {
    if (!field)
      throw std::invalid_argument("Size field override needs a field");
    return update(
        DecisionSubject::SizeField,
        [&](discretization::MeshOptions &o) {
          o.refinement.size_field = std::move(field);
          o.refinement.size_field_descriptor = std::move(descriptor);
        },
        why);
  }
  // A user refinement criterion; it must carry its descriptor.
  MeshPlan &
  set_refinement_criterion(discretization::RefinementCriterion criterion,
                           const Rationale &why = {}) {
    if (!criterion.descriptor)
      throw std::invalid_argument(
          "Refinement criterion override needs a descriptor");
    return update(
        DecisionSubject::RefinementCriterion,
        [&](discretization::MeshOptions &o) {
          o.refinement.criterion = std::move(criterion);
        },
        why);
  }
};

inline MeshPlan mesh_plan_from_user(const discretization::MeshOptions &o) {
  return {o, mesh_decisions(o)};
}

// Every decision of the plan is about a mesh option, appears once and
// matches `used` (by default the plan's own options). A whole-mesh decision
// (DecisionSubject::Mesh) explains all options together and has no single
// option to compare. Throws std::invalid_argument ("Stale decision: ...",
// "Duplicate decision: ...", "... does not belong to a mesh plan ...").
inline void validate_decisions(const MeshPlan &plan,
                               const discretization::MeshOptions &used) {
  validate_bound_decisions(mesh_plan_bindings, used, plan.decisions, {},
                           "mesh plan",
                           [](DecisionSubject) { return std::string(); });
}
inline void validate_decisions(const MeshPlan &plan) {
  validate_decisions(plan, plan.options);
}

// The first mesh option whose choice differs between two option sets, if
// any (by binding table row; the whole-mesh row compares nothing).
inline std::optional<DecisionSubject>
mesh_options_difference(const discretization::MeshOptions &a,
                        const discretization::MeshOptions &b) {
  for (const auto &row : mesh_plan_bindings)
    if (row.choice(a) != row.choice(b))
      return row.subject;
  return std::nullopt;
}
} // namespace pde2d::fem
