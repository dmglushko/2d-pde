#pragma once
// Mesh plan: the meshing options of a run together with the decisions that
// explain them, the mesh-side counterpart of SolvePlan (plan.hpp). A
// preprocessor that tailors a mesh to the problem returns a MeshPlan whose
// decisions use DecisionSource::Automatic; the user overrides it by editing
// the options and the matching decision.
//
//   auto mesh_plan = mesh_plan_from_user(options);
//   auto result = discretization::generate_mesh(geometry, mesh_plan.options);
//   auto plan = plan_from_user(assembly, solver);
//   plan.mesh = mesh_plan;
//   solve_stationary(result, problem, plan);  // checks plan.mesh against
//                                             // result.options
//
// It lives in the FEM module because decisions do (decision.hpp); meshing
// itself only sees discretization::MeshOptions, so the dependency direction
// (fem includes discretization, never the reverse) is kept.
#include "discretization/mesh_types.hpp"
#include "fem/decision.hpp"
#include "provenance.hpp"
#include <format>
#include <stdexcept>
#include <string>
#include <vector>

namespace pde2d::fem {
struct MeshPlan {
  discretization::MeshOptions options{};
  // Why each option has its value: mesh algorithm, boundary treatment, mesh
  // size, size field (mesh_decisions), optionally a whole-mesh decision.
  std::vector<Decision> decisions{};
};

// The choice text of each mesh decision, from the options. Numbers use the
// shortest round-trip format, so any change of an option changes its text.
inline std::string
size_field_choice(const discretization::RefinementOptions &o) {
  if (!o.size_field)
    return "none";
  if (!o.size_field_descriptor)
    return "undescribed (not replayable)";
  return pde2d::describe_strategy(*o.size_field_descriptor);
}
// The user refinement criterion: "none", its descriptor, or "undescribed
// (not replayable)".
inline std::string
refinement_criterion_choice(const discretization::RefinementOptions &o) {
  if (!o.criterion)
    return "none";
  if (!o.criterion->descriptor)
    return "undescribed (not replayable)";
  return pde2d::describe_strategy(*o.criterion->descriptor);
}
inline std::string mesh_decision_choice(const discretization::MeshOptions &o,
                                        DecisionSubject subject) {
  switch (subject) {
  case DecisionSubject::MeshAlgorithm:
    return discretization::interior_algorithm_name(o.algorithm);
  case DecisionSubject::BoundaryTreatment:
    return std::format(
        "{} preserve_corners={} preserve_regions={}",
        discretization::boundary_treatment_name(o.boundary_treatment),
        o.boundary.preserve_corners, o.preserve_regions);
  case DecisionSubject::MeshSize:
    return std::format("boundary_max_edge_length={} max_curve_deviation={} "
                       "refinement_max_edge_length={} max_steps={}",
                       o.boundary.max_edge_length,
                       o.boundary.max_curve_deviation,
                       o.refinement.max_edge_length, o.refinement.max_steps);
  case DecisionSubject::SizeField:
    return size_field_choice(o.refinement);
  case DecisionSubject::RefinementCriterion:
    return refinement_criterion_choice(o.refinement);
  default:
    throw std::invalid_argument("Decision subject " +
                                std::string(decision_subject_name(subject)) +
                                " does not belong to a mesh plan");
  }
}

// The user's meshing options as decision records, in a fixed order.
// A refinement criterion decision follows only when a criterion is set, so
// the decisions of plain options are unchanged.
inline std::vector<Decision>
mesh_decisions(const discretization::MeshOptions &o) {
  std::vector<Decision> decisions{
      user_decision(DecisionSubject::MeshAlgorithm,
                    mesh_decision_choice(o, DecisionSubject::MeshAlgorithm),
                    "MeshOptions::algorithm"),
      user_decision(DecisionSubject::BoundaryTreatment,
                    mesh_decision_choice(o, DecisionSubject::BoundaryTreatment),
                    "MeshOptions::boundary_treatment"),
      user_decision(DecisionSubject::MeshSize,
                    mesh_decision_choice(o, DecisionSubject::MeshSize),
                    "MeshOptions::boundary, MeshOptions::refinement"),
      user_decision(DecisionSubject::SizeField,
                    mesh_decision_choice(o, DecisionSubject::SizeField),
                    "RefinementOptions::size_field")};
  if (o.refinement.criterion)
    decisions.push_back(user_decision(
        DecisionSubject::RefinementCriterion,
        mesh_decision_choice(o, DecisionSubject::RefinementCriterion),
        "RefinementOptions::criterion"));
  return decisions;
}
inline MeshPlan mesh_plan_from_user(const discretization::MeshOptions &o) {
  return {o, mesh_decisions(o)};
}

// Every decision of the plan is about a mesh option, appears once and
// matches `used` (by default the plan's own options). A whole-mesh decision
// (DecisionSubject::Mesh) explains all options together and has no single
// option to compare. Throws std::invalid_argument ("Stale decision: ...",
// "Duplicate decision: ...").
inline void validate_decisions(const MeshPlan &plan,
                               const discretization::MeshOptions &used) {
  decision_detail::require_unique(plan.decisions);
  for (const auto &decision : plan.decisions)
    if (decision.subject != DecisionSubject::Mesh)
      decision_detail::require_current(
          decision, mesh_decision_choice(used, decision.subject));
}
inline void validate_decisions(const MeshPlan &plan) {
  validate_decisions(plan, plan.options);
}
} // namespace pde2d::fem
