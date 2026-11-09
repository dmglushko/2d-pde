#pragma once
// Planner seams (docs/vision.md, goals 2 and 3): the interfaces of a future
// automatic chooser of solve settings (SolvePlanner) and of meshing options
// (MeshPlanner), and the identity planner UserPlanner that keeps the user's
// plan. Nothing here chooses automatically; this header only defines where
// a chooser plugs in and how its output is checked and recorded.
//
//   auto planned = run_planner(planner, problem, properties, defaults);
//   solve_stationary(mesh, problem, planned);   // planned.planner names it
//
//   auto [mesh, mesh_plan] = plan_mesh(geometry, problem, mesh_planner,
//                                      mesh_plan_from_user(options));
//   plan.mesh = mesh_plan;                      // before the first mesh
//
//   AdaptOptions options;                       // adaptivity.hpp
//   options.planners.solve = std::make_shared<MyPlanner>();
//   adapt(result, problem, plan, options);      // planned before every solve
//
// A planner returns its `defaults` with the settings it chooses changed
// through the setters (SolvePlan::update, MeshPlan::update or the named
// ones), each with a Rationale (basis, evidence, DecisionSource::Automatic),
// so every choice is explained and the user can override it. run_planner
// validates what it returns before anything runs ("Solve planner <name>
// returned an invalid plan: ...") and stamps the plan with the planner
// (SolvePlan::planner, MeshPlan::planner, a StrategyRecord); run records
// list it among their strategies (DecisionSubject::SolvePlanner,
// MeshPlanner). Planners see the previous adaptive step as an AdaptiveStep
// (adaptivity_types.hpp, a light record header), not the drivers.
#include "discretization/boundary_discretization.hpp"
#include "discretization/curve_geometry.hpp"
#include "discretization/mesh_fingerprint.hpp"
#include "discretization/mesh_types.hpp"
#include "discretization/triangulate.hpp"
#include "fem/adaptivity_types.hpp"
#include "fem/analysis.hpp"
#include "fem/analysis_types.hpp"
#include "fem/decision.hpp"
#include "fem/fingerprint.hpp"
#include "fem/mesh_plan.hpp"
#include "fem/plan.hpp"
#include "fem/problem.hpp"
#include "provenance.hpp"
#include <cstddef>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace pde2d::fem {
// What a planner may look at besides the problem and its analysis.
struct PlanningContext {
  // The geometry being meshed, when known: set by plan_mesh (before the
  // first mesh exists) and compare_planners; nullptr inside adapt().
  const discretization::CurveGeometry *geometry = nullptr;
  // plan_mesh: the geometry's boundary sampled with the defaults' boundary
  // options (a planner that changes them gets a new sampling).
  const discretization::DiscretizedBoundary *boundary = nullptr;
  // The mesh the plan is for: for a SolvePlanner the mesh about to be solved
  // on, for a MeshPlanner inside adapt() the mesh about to be changed;
  // nullptr before the first mesh.
  const discretization::MeshResult *mesh = nullptr;
  // Inside adapt() and adapt_evolution(): the previous step (its run
  // record, transfer, estimate, marking); for a MeshPlanner the step just
  // estimated and marked (its `adaptation` is still empty). nullptr before
  // the first solve and outside the drivers.
  const AdaptiveStep *previous = nullptr;
  // Inside the drivers: the index of the step being planned.
  std::size_t step = 0;
};

class SolvePlanner {
public:
  virtual ~SolvePlanner() = default;
  // Name used in records and decisions.
  virtual std::string name() const = 0;
  // Stable description (name + parameters); without one the record says
  // the planner is "not replayable".
  virtual std::optional<pde2d::StrategyDescriptor> descriptor() const {
    return std::nullopt;
  }
  // The plan to run: `defaults` (the user's plan, or inside the adaptive
  // drivers the plan of the previous step) with the settings this planner
  // chooses changed through the setters. Must keep the time settings present or
  // absent as in `defaults`.
  virtual SolvePlan plan(const Problem &problem,
                         const ProblemProperties &properties,
                         const SolvePlan &defaults,
                         const PlanningContext &context) const = 0;
};

class MeshPlanner {
public:
  virtual ~MeshPlanner() = default;
  virtual std::string name() const = 0;
  virtual std::optional<pde2d::StrategyDescriptor> descriptor() const {
    return std::nullopt;
  }
  // The meshing options to use, with their decisions: `defaults` changed
  // through the MeshPlan setters. A size field or refinement criterion it
  // sets must carry a descriptor (set_size_field, set_refinement_criterion
  // require one).
  virtual MeshPlan plan(const Problem &problem,
                        const ProblemProperties &properties,
                        const MeshPlan &defaults,
                        const PlanningContext &context) const = 0;
};

// The identity planner: returns its defaults unchanged.
class UserPlanner final : public SolvePlanner, public MeshPlanner {
public:
  std::string name() const override { return "User"; }
  std::optional<pde2d::StrategyDescriptor> descriptor() const override {
    return pde2d::StrategyDescriptor{"User"};
  }
  SolvePlan plan(const Problem &, const ProblemProperties &,
                 const SolvePlan &defaults,
                 const PlanningContext &) const override {
    return defaults;
  }
  MeshPlan plan(const Problem &, const ProblemProperties &,
                const MeshPlan &defaults,
                const PlanningContext &) const override {
    return defaults;
  }
};

// The planners of a run: the mesh planner (before meshing, or after
// marking inside the adaptive drivers) and the solve planner (before every
// solve). Unset: the plan stays as given.
struct PlannerSet {
  std::shared_ptr<const MeshPlanner> mesh{};
  std::shared_ptr<const SolvePlanner> solve{};
};

// The planner's choice as a decision of an adaptive step
// (DecisionSubject::SolvePlanner or MeshPlanner): its descriptor, or
// "<name> (not replayable)".
inline std::string planner_choice(const StrategyRecord &planner) {
  if (planner.descriptor)
    return pde2d::describe_strategy(*planner.descriptor);
  return planner.name + " (not replayable)";
}

namespace planner_detail {
template <class Planner>
StrategyRecord record(DecisionSubject subject, const Planner &planner) {
  return strategy_record(subject, planner.name(),
                         dynamic_cast<const UserPlanner *>(&planner) != nullptr,
                         planner.descriptor());
}
template <class F>
void checked(const std::string &kind, const std::string &name, F &&check) {
  try {
    check();
  } catch (const std::invalid_argument &error) {
    throw std::invalid_argument(kind + " planner " + name +
                                " returned an invalid plan: " + error.what());
  }
}
} // namespace planner_detail

// Run a solve planner and check its plan before anything uses it: valid,
// current decisions (validate_decisions), time settings present exactly when
// they are in `defaults`, and an analysis (if any) made for `problem` and,
// with context.mesh, that mesh. Returns the plan with `planner` set. Throws
// std::invalid_argument ("Solve planner <name> returned an invalid plan:
// <reason>").
inline SolvePlan run_planner(const SolvePlanner &planner,
                             const Problem &problem,
                             const ProblemProperties &properties,
                             const SolvePlan &defaults,
                             const PlanningContext &context = {}) {
  auto planned = planner.plan(problem, properties, defaults, context);
  planner_detail::checked("Solve", planner.name(), [&] {
    if (planned.time.has_value() != defaults.time.has_value())
      throw std::invalid_argument(
          "a planner may not add or remove time settings");
    validate_decisions(planned);
    if (planned.analysis)
      require_current_analysis(
          *planned.analysis, problem_fingerprint(problem),
          context.mesh ? discretization::mesh_fingerprint(context.mesh->mesh)
                       : planned.analysis->mesh_fingerprint().value_or(""));
  });
  planned.planner =
      planner_detail::record(DecisionSubject::SolvePlanner, planner);
  return planned;
}
// Run a mesh planner and check its plan (validate_decisions). Returns the
// plan with `planner` set. Throws std::invalid_argument ("Mesh planner
// <name> returned an invalid plan: <reason>").
inline MeshPlan run_planner(const MeshPlanner &planner, const Problem &problem,
                            const ProblemProperties &properties,
                            const MeshPlan &defaults,
                            const PlanningContext &context = {}) {
  auto planned = planner.plan(problem, properties, defaults, context);
  planner_detail::checked("Mesh", planner.name(),
                          [&] { validate_decisions(planned); });
  planned.planner =
      planner_detail::record(DecisionSubject::MeshPlanner, planner);
  return planned;
}

// A mesh made from a plan: the meshing result and the mesh plan that made
// it (result.options are plan.options). Put `plan` in SolvePlan::mesh so
// solves and records know how the mesh was chosen.
struct PlannedMesh {
  discretization::MeshResult result{};
  MeshPlan plan{};
};
// Plan the meshing options of `geometry` for `problem` before any mesh
// exists, then mesh: run_planner(planner, problem, mesh-free analysis,
// defaults, {geometry, boundary sampled with the defaults' options}) and
// generate_mesh(geometry, planned options). With UserPlanner the result is
// generate_mesh(geometry, defaults.options). Throws what run_planner and
// generate_mesh throw. The result's report must still be checked.
[[nodiscard]] inline PlannedMesh
plan_mesh(const discretization::CurveGeometry &geometry, const Problem &problem,
          const MeshPlanner &planner, const MeshPlan &defaults,
          const AnalysisOptions &analysis = {}) {
  validate_decisions(defaults);
  const auto boundary = discretization::discretize_boundary(
      geometry, defaults.options.boundary, defaults.options.preserve_regions);
  PlanningContext context;
  context.geometry = &geometry;
  context.boundary = &boundary;
  auto plan = run_planner(planner, problem,
                          make_analysis_record(problem, analysis).properties(),
                          defaults, context);
  auto result = discretization::generate_mesh(geometry, plan.options);
  return {std::move(result), std::move(plan)};
}
} // namespace pde2d::fem
