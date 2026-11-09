#pragma once
// Comparing plans on a reference problem (reference/problems.hpp): run each
// plan, measure its error against the exact solution and record its work,
// so a choice can be judged in accuracy for the work spent (docs/vision.md).
//
//   auto reference = find_reference_problem("annulus_mixed");
//   auto mesh_plan = mesh_plan_from_user(reference.mesh);
//   auto c = compare_plans(reference, mesh_plan, {lu_plan, cg_plan});
//   c.entries[k].errors.l2;          // L2 / H1-seminorm error (error_norms)
//   c.entries[k].work;               // nodes, solves, iterations, mesh work
//   c.entries[k].record;             // the run's full RunRecord
//   write_json(out, c);              // schema "pde2d.plan_comparison"
//
//   compare_mesh_plans(reference, {coarse, fine}, plan);  // one plan, meshes
//
// compare_plans meshes the reference geometry once (generate_mesh with the
// mesh plan's options) and solves it with every solve plan;
// compare_mesh_plans meshes it with every mesh plan and solves each with the
// one solve plan. Each solve gets the mesh plan it ran on (plan.mesh is
// replaced) and, when the plan carries a mesh-bound analysis, an analysis
// refreshed for that mesh (refresh_analysis), so records describe what ran.
// Stationary problems are solved at t = 0; transient ones are evolved over
// the reference window, which the plan's time settings must match, and
// compared at its end. Nothing here changes a numerical result: every entry
// is the plain public solve of its inputs.
#include "discretization/mesh_record_json.hpp"
#include "discretization/triangulate.hpp"
#include "fem/analysis.hpp"
#include "fem/error_norms.hpp"
#include "fem/evolution.hpp"
#include "fem/json.hpp"
#include "fem/mesh_acceptance.hpp"
#include "fem/mesh_plan.hpp"
#include "fem/plan.hpp"
#include "fem/record.hpp"
#include "fem/record_json.hpp"
#include "fem/reference/problems.hpp"
#include "fem/solver.hpp"
#include <cstddef>
#include <format>
#include <istream>
#include <limits>
#include <ostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace pde2d::fem {
struct ComparisonOptions {
  // Quadrature exactness degree of the error norms (error_norms.hpp).
  int error_degree = 8;
  // Solve on meshes that miss their size target (accept_unsatisfied);
  // otherwise such a mesh throws as in every MeshResult entry point.
  bool accept_unsatisfied = false;
};

// The error of one run against the exact solution at `time`.
struct ComparisonErrors {
  double l2 = 0, h1_seminorm = 0;
  double time = 0;
};
// The work of one run, summarized from its records.
struct ComparisonWork {
  std::size_t nodes = 0, elements = 0;
  std::size_t assembly_calls = 0, linear_solves = 0, solver_preparations = 0,
              solver_iterations = 0, time_steps = 0;
  // Of the meshing call (discretization::MeshRunRecord::work).
  std::size_t mesh_refinement_steps = 0, mesh_edge_flips = 0;
};
// Wall-clock seconds. Never reproducible (diff_records skips them).
struct ComparisonTimings {
  double mesh_seconds = 0, solve_seconds = 0;
};
struct ComparisonEntry {
  std::size_t index = 0;
  // A readable name of the plan (comparison_label).
  std::string label{};
  RunRecord record{};
  ComparisonErrors errors{};
  ComparisonWork work{};
  ComparisonTimings timings{};
};
// What varied between the entries.
enum class ComparisonKind { SolvePlans, MeshPlans };
inline const char *comparison_kind_name(ComparisonKind kind) {
  return kind == ComparisonKind::SolvePlans ? "solve plans" : "mesh plans";
}
struct PlanComparison {
  std::string reference{}; // ReferenceProblem::name
  std::string notes{};
  ComparisonKind kind = ComparisonKind::SolvePlans;
  std::vector<ComparisonEntry> entries{};
};

// "SparseLU", "ConjugateGradient / CrankNicolson h=0.01", ...: the solver
// (and time scheme) of a solve plan.
inline std::string comparison_label(const SolvePlan &plan) {
  auto text = linear_solver_name(plan.solver);
  if (plan.solver.backend == nullptr &&
      plan.solver.preconditioner != Preconditioner::Default)
    text +=
        std::string(" + ") + preconditioner_name(plan.solver.preconditioner);
  if (plan.time) {
    text += std::string(" / ") + (plan.time->integrator
                                      ? std::string("custom integrator")
                                      : time_method_name(plan.time->method));
    if (!plan.time->step_controller)
      text += std::format(" h={}", plan.time->step);
  }
  return text;
}
// "ConstrainedDelaunay, Fixed, h=0.1": the meshing of a mesh plan.
inline std::string comparison_label(const MeshPlan &plan) {
  const auto &o = plan.options;
  auto text = std::format(
      "{}, {}", discretization::interior_algorithm_name(o.algorithm),
      discretization::boundary_treatment_name(o.boundary_treatment));
  if (o.refinement.max_edge_length > 0)
    text += std::format(", h={}", o.refinement.max_edge_length);
  if (o.refinement.size_field)
    text += ", size field";
  if (o.refinement.criterion)
    text += ", criterion";
  return text;
}

namespace comparison_detail {
inline discretization::MeshResult mesh_of(const ReferenceProblem &reference,
                                          const MeshPlan &mesh_plan) {
  validate_decisions(mesh_plan);
  return discretization::generate_mesh(reference.geometry, mesh_plan.options);
}
inline ComparisonEntry run(const ReferenceProblem &reference,
                           const discretization::MeshResult &mesh,
                           const MeshPlan &mesh_plan, SolvePlan plan,
                           const ComparisonOptions &options) {
  plan.mesh = mesh_plan;
  if (plan.analysis && plan.analysis->mesh_fingerprint())
    refresh_analysis(plan, reference.problem, mesh.mesh);
  if (!options.accept_unsatisfied)
    require_size_satisfied(mesh);
  const auto accepted = accept_unsatisfied(mesh);
  ComparisonEntry entry;
  Vector values;
  if (const auto &window = reference.transient) {
    if (!plan.time)
      throw std::invalid_argument("Reference problem " + reference.name +
                                  " is transient; the plan has no time "
                                  "settings");
    if (plan.time->start != window->start || plan.time->end != window->end)
      throw std::invalid_argument(std::format(
          "Plan time window [{}, {}] does not match reference problem {} "
          "window [{}, {}]",
          plan.time->start, plan.time->end, reference.name, window->start,
          window->end));
    EvolutionOutput output;
    output.frame_stride = std::numeric_limits<std::size_t>::max();
    output.store_reports = false;
    auto evolution =
        evolve(accepted, reference.problem, window->initial, plan, output);
    values = std::move(evolution.frames.back().values);
    entry.errors.time = evolution.frames.back().time;
    entry.record = std::move(evolution.record);
  } else {
    if (plan.time)
      throw std::invalid_argument("Reference problem " + reference.name +
                                  " is stationary; the plan has time "
                                  "settings");
    auto solution = PreparedStationary(accepted, reference.problem, plan)
                        .solve(entry.errors.time);
    values = std::move(solution.values);
    entry.record = std::move(solution.record);
  }
  const auto norms =
      PreparedErrorNorms(mesh.mesh, reference.exact, options.error_degree)
          .evaluate(values, entry.errors.time, plan.assembly.execution);
  entry.errors.l2 = norms.l2;
  entry.errors.h1_seminorm = norms.h1_seminorm;
  const auto &w = entry.record.work;
  entry.work = {mesh.mesh.nodes.size(),
                mesh.mesh.elements.size(),
                w.assembly_calls,
                w.linear_solves,
                w.solver_preparations,
                w.solver_iterations,
                w.time_steps,
                mesh.record.work.refinement_steps,
                mesh.record.work.edge_flips};
  entry.timings = {mesh.record.total_seconds,
                   entry.record.timings.total_seconds};
  return entry;
}
} // namespace comparison_detail

// Mesh the reference geometry with `mesh_plan` and solve it with each plan.
// Throws what the solves throw, and std::invalid_argument for a plan whose
// time settings do not fit the problem (none for a transient problem, any
// for a stationary one, another window).
inline PlanComparison compare_plans(const ReferenceProblem &reference,
                                    const MeshPlan &mesh_plan,
                                    const std::vector<SolvePlan> &plans,
                                    const ComparisonOptions &options = {}) {
  PlanComparison comparison{reference.name, reference.notes,
                            ComparisonKind::SolvePlans};
  const auto mesh = comparison_detail::mesh_of(reference, mesh_plan);
  for (std::size_t k = 0; k < plans.size(); ++k) {
    auto entry =
        comparison_detail::run(reference, mesh, mesh_plan, plans[k], options);
    entry.index = k;
    entry.label = comparison_label(plans[k]);
    comparison.entries.push_back(std::move(entry));
  }
  return comparison;
}
// Mesh the reference geometry with each mesh plan and solve with `plan`.
inline PlanComparison
compare_mesh_plans(const ReferenceProblem &reference,
                   const std::vector<MeshPlan> &meshes, const SolvePlan &plan,
                   const ComparisonOptions &options = {}) {
  PlanComparison comparison{reference.name, reference.notes,
                            ComparisonKind::MeshPlans};
  for (std::size_t k = 0; k < meshes.size(); ++k) {
    const auto mesh = comparison_detail::mesh_of(reference, meshes[k]);
    auto entry =
        comparison_detail::run(reference, mesh, meshes[k], plan, options);
    entry.index = k;
    entry.label = comparison_label(meshes[k]);
    comparison.entries.push_back(std::move(entry));
  }
  return comparison;
}

inline constexpr const char *plan_comparison_schema = "pde2d.plan_comparison";
inline constexpr int plan_comparison_schema_version = 1;

// The JSON document of a comparison (conventions of record_json.hpp; each
// entry embeds its run record under "record").
inline JsonValue to_json(const PlanComparison &c) {
  auto v = JsonValue::object();
  v.set("schema", json_string(plan_comparison_schema));
  v.set("schema_version", json_number(plan_comparison_schema_version));
  v.set("reference", json_string(c.reference));
  v.set("notes", json_string(c.notes));
  v.set("kind", json_string(comparison_kind_name(c.kind)));
  auto entries = JsonValue::array();
  for (const auto &e : c.entries) {
    auto entry = JsonValue::object();
    entry.set("index", json_number(e.index));
    entry.set("label", json_string(e.label));
    auto errors = JsonValue::object();
    errors.set("l2", json_number(e.errors.l2));
    errors.set("h1_seminorm", json_number(e.errors.h1_seminorm));
    errors.set("time", json_number(e.errors.time));
    entry.set("errors", std::move(errors));
    const auto &w = e.work;
    auto work = JsonValue::object();
    work.set("nodes", json_number(w.nodes));
    work.set("elements", json_number(w.elements));
    work.set("assembly_calls", json_number(w.assembly_calls));
    work.set("linear_solves", json_number(w.linear_solves));
    work.set("solver_preparations", json_number(w.solver_preparations));
    work.set("solver_iterations", json_number(w.solver_iterations));
    work.set("time_steps", json_number(w.time_steps));
    work.set("mesh_refinement_steps", json_number(w.mesh_refinement_steps));
    work.set("mesh_edge_flips", json_number(w.mesh_edge_flips));
    entry.set("work", std::move(work));
    auto timings = JsonValue::object();
    timings.set("mesh_seconds", json_number(e.timings.mesh_seconds));
    timings.set("solve_seconds", json_number(e.timings.solve_seconds));
    entry.set("timings", std::move(timings));
    entry.set("record", to_json(e.record));
    entries.push(std::move(entry));
  }
  v.set("entries", std::move(entries));
  return v;
}
inline std::ostream &write_json(std::ostream &out, const PlanComparison &c) {
  return write_json(out, to_json(c));
}
// Parse a comparison document and check its schema. Throws
// std::invalid_argument ("Invalid JSON: ...", "Not a plan comparison",
// "Unsupported plan comparison schema version: N").
inline JsonValue read_plan_comparison_json(std::istream &in) {
  auto document = parse_json(in);
  const auto *schema = document.kind == JsonValue::Kind::Object
                           ? document.find("schema")
                           : nullptr;
  if (!schema || schema->kind != JsonValue::Kind::String ||
      schema->text != plan_comparison_schema)
    throw std::invalid_argument("Not a plan comparison");
  const auto *version = document.find("schema_version");
  if (!version || version->kind != JsonValue::Kind::Number ||
      version->text != std::to_string(plan_comparison_schema_version))
    throw std::invalid_argument(
        "Unsupported plan comparison schema version: " +
        (version ? version->text : std::string("missing")));
  return document;
}
} // namespace pde2d::fem
