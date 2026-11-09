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
//   compare_planners(reference, {{mesh_planner, solve_planner}}, plan);
//
// compare_planners judges planners (planner.hpp): entry 0 is always the
// identity UserPlanner (the baseline: the defaults as given), then one entry
// per PlannerSet (an unset planner is the identity). Each entry plans and
// meshes with plan_mesh (its geometry in the PlanningContext), plans the
// solve on that mesh (run_planner) and solves; it records the planners' own
// cost (ComparisonTimings::planning_seconds, wall clock without meshing),
// the decisions they changed (planner_decisions) and its error and work
// relative to the baseline (ComparisonEntry::relative).
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
#include "fem/planner.hpp"
#include "fem/record.hpp"
#include "fem/record_json.hpp"
#include "fem/reference/problems.hpp"
#include "fem/solver.hpp"
#include <algorithm>
#include <chrono>
#include <cstddef>
#include <format>
#include <istream>
#include <limits>
#include <optional>
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
  // compare_planners: the planners' own time (analysis and plan calls,
  // without meshing), and the entry's total (mesh + planning + solve)
  // divided by the baseline's.
  double planning_seconds = 0;
  std::optional<double> relative_seconds{};
};
// An entry divided by the baseline entry (compare_planners: the identity
// planner), error by error and count by count; 1 when both are 0, inf when
// only the baseline's is.
struct ComparisonRelative {
  double l2 = 1, h1_seminorm = 1;
  double nodes = 1, linear_solves = 1, solver_iterations = 1;
};
struct ComparisonEntry {
  std::size_t index = 0;
  // A readable name of the plan (comparison_label).
  std::string label{};
  RunRecord record{};
  ComparisonErrors errors{};
  ComparisonWork work{};
  ComparisonTimings timings{};
  // compare_planners: the decisions the planners changed (of the mesh plan,
  // then of the solve plan; those that differ from the defaults' or are
  // new), and the entry relative to the baseline.
  std::vector<Decision> planner_decisions{};
  std::optional<ComparisonRelative> relative{};
};
// What varied between the entries.
enum class ComparisonKind { SolvePlans, MeshPlans, Planners };
inline const char *comparison_kind_name(ComparisonKind kind) {
  switch (kind) {
  case ComparisonKind::SolvePlans:
    return "solve plans";
  case ComparisonKind::MeshPlans:
    return "mesh plans";
  case ComparisonKind::Planners:
    return "planners";
  }
  return "invalid";
}
struct PlanComparison {
  std::string reference{}; // ReferenceProblem::name
  std::string notes{};
  ComparisonKind kind = ComparisonKind::SolvePlans;
  std::vector<ComparisonEntry> entries{};
  // The entry the others are relative to (compare_planners: 0).
  std::optional<std::size_t> baseline{};
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

// "mesh: FinerTarget, solve: User": the planners of a set (unset: User).
inline std::string comparison_label(const PlannerSet &planners) {
  return "mesh: " + (planners.mesh ? planners.mesh->name() : "User") +
         ", solve: " + (planners.solve ? planners.solve->name() : "User");
}

namespace comparison_detail {
inline double ratio(double value, double baseline) {
  if (baseline > 0)
    return value / baseline;
  return value == 0 ? 1.0 : std::numeric_limits<double>::infinity();
}
inline double seconds_since(std::chrono::steady_clock::time_point start) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
      .count();
}
// The decisions of `planned` that are not in `defaults` as they are.
inline void changed_decisions(const std::vector<Decision> &defaults,
                              const std::vector<Decision> &planned,
                              std::vector<Decision> &out) {
  for (const auto &d : planned) {
    const auto *was = find_decision(defaults, d.subject);
    if (!was || was->choice != d.choice || was->basis != d.basis ||
        was->evidence != d.evidence || was->source != d.source)
      out.push_back(d);
  }
}
inline ComparisonEntry
run_planners(const ReferenceProblem &reference, const MeshPlanner &mesh_planner,
             const SolvePlanner &solve_planner, const MeshPlan &mesh_defaults,
             const SolvePlan &defaults, const ComparisonOptions &options) {
  const auto planning_started = std::chrono::steady_clock::now();
  auto planned = plan_mesh(reference.geometry, reference.problem, mesh_planner,
                           mesh_defaults);
  // plan_mesh times meshing in the mesh run record; the rest is planning.
  double planning =
      seconds_since(planning_started) - planned.result.record.total_seconds;
  const auto solve_started = std::chrono::steady_clock::now();
  auto solve_defaults = defaults;
  solve_defaults.mesh = planned.plan;
  refresh_analysis(solve_defaults, reference.problem, planned.result.mesh);
  PlanningContext context;
  context.geometry = &reference.geometry;
  context.mesh = &planned.result;
  const auto properties =
      solve_defaults.analysis
          ? solve_defaults.analysis->properties()
          : make_analysis_record(reference.problem, planned.result.mesh)
                .properties();
  const auto plan = run_planner(solve_planner, reference.problem, properties,
                                solve_defaults, context);
  planning += seconds_since(solve_started);
  auto entry = run(reference, planned.result, planned.plan, plan, options);
  entry.timings.planning_seconds = std::max(planning, 0.0);
  changed_decisions(mesh_defaults.decisions, planned.plan.decisions,
                    entry.planner_decisions);
  changed_decisions(defaults.decisions, plan.decisions,
                    entry.planner_decisions);
  return entry;
}
} // namespace comparison_detail

// Compare planner sets on a reference problem against the identity
// planner: entry 0 plans nothing (UserPlanner for mesh and solve), entry
// k + 1 runs planners[k]. Meshing starts from defaults.mesh (unset: the
// reference's suggested options, mesh_plan_from_user(reference.mesh)), the
// solve from `defaults`. Throws what planning, meshing and the solves
// throw, and as compare_plans for time settings that do not fit.
inline PlanComparison compare_planners(const ReferenceProblem &reference,
                                       const std::vector<PlannerSet> &planners,
                                       const SolvePlan &defaults,
                                       const ComparisonOptions &options = {}) {
  PlanComparison comparison{reference.name, reference.notes,
                            ComparisonKind::Planners};
  comparison.baseline = 0;
  const auto mesh_defaults =
      defaults.mesh ? *defaults.mesh : mesh_plan_from_user(reference.mesh);
  const UserPlanner identity;
  for (std::size_t k = 0; k <= planners.size(); ++k) {
    const PlannerSet set = k == 0 ? PlannerSet{} : planners[k - 1];
    auto entry = comparison_detail::run_planners(
        reference, set.mesh ? *set.mesh : identity,
        set.solve ? *set.solve : identity, mesh_defaults, defaults, options);
    entry.index = k;
    entry.label =
        k == 0 ? "User (baseline)" : comparison_label(planners[k - 1]);
    comparison.entries.push_back(std::move(entry));
  }
  using comparison_detail::ratio;
  const auto &base = comparison.entries.front();
  const auto total = [](const ComparisonEntry &e) {
    return e.timings.mesh_seconds + e.timings.planning_seconds +
           e.timings.solve_seconds;
  };
  for (auto &e : comparison.entries) {
    e.relative = ComparisonRelative{
        ratio(e.errors.l2, base.errors.l2),
        ratio(e.errors.h1_seminorm, base.errors.h1_seminorm),
        ratio(static_cast<double>(e.work.nodes),
              static_cast<double>(base.work.nodes)),
        ratio(static_cast<double>(e.work.linear_solves),
              static_cast<double>(base.work.linear_solves)),
        ratio(static_cast<double>(e.work.solver_iterations),
              static_cast<double>(base.work.solver_iterations))};
    e.timings.relative_seconds = ratio(total(e), total(base));
  }
  return comparison;
}
// A stationary reference with the default solve settings
// (plan_from_user(AssemblyOptions{})).
inline PlanComparison compare_planners(const ReferenceProblem &reference,
                                       const std::vector<PlannerSet> &planners,
                                       const ComparisonOptions &options = {}) {
  return compare_planners(reference, planners,
                          plan_from_user(AssemblyOptions{}), options);
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
    timings.set("planning_seconds", json_number(e.timings.planning_seconds));
    timings.set("relative_seconds",
                e.timings.relative_seconds
                    ? json_number(*e.timings.relative_seconds)
                    : JsonValue::null());
    entry.set("timings", std::move(timings));
    entry.set("planner_decisions",
              record_json_detail::decisions(e.planner_decisions));
    if (const auto &r = e.relative) {
      auto relative = JsonValue::object();
      relative.set("l2", json_number(r->l2));
      relative.set("h1_seminorm", json_number(r->h1_seminorm));
      relative.set("nodes", json_number(r->nodes));
      relative.set("linear_solves", json_number(r->linear_solves));
      relative.set("solver_iterations", json_number(r->solver_iterations));
      entry.set("relative", std::move(relative));
    } else
      entry.set("relative", JsonValue::null());
    entry.set("record", to_json(e.record));
    entries.push(std::move(entry));
  }
  v.set("entries", std::move(entries));
  v.set("baseline", c.baseline ? json_number(*c.baseline) : JsonValue::null());
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
