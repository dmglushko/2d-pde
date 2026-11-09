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
//   AdaptOptions options;                       // adaptivity.hpp
//   options.solve_planner = std::make_shared<MyPlanner>();
//   adapt(result, problem, plan, options);      // planned before every solve
//
// A planner returns its `defaults` with the settings it chooses changed
// through the setters (SolvePlan::update, MeshPlan::update or the named
// ones), each with a Rationale (basis, evidence, DecisionSource::Automatic),
// so every choice is explained and the user can override it. run_planner
// validates what it returns before anything runs ("Solve planner <name>
// returned an invalid plan: ...") and stamps the plan with the planner
// (SolvePlan::planner, MeshPlan::planner); run records list it among their
// strategies (DecisionSubject::SolvePlanner, MeshPlanner).
#include "discretization/mesh_fingerprint.hpp"
#include "discretization/mesh_types.hpp"
#include "fem/analysis_types.hpp"
#include "fem/decision.hpp"
#include "fem/fingerprint.hpp"
#include "fem/mesh_plan.hpp"
#include "fem/plan.hpp"
#include "fem/problem.hpp"
#include "provenance.hpp"
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>

namespace pde2d::fem {
struct AdaptiveIteration; // adaptivity.hpp

// What a planner may look at besides the problem and its analysis.
struct PlanningContext {
  // The mesh the plan is for: for a SolvePlanner the mesh about to be solved
  // on, for a MeshPlanner inside adapt() the mesh about to be refined.
  const discretization::MeshResult *mesh = nullptr;
  // Inside adapt(): the previous iteration's diagnostics (its run record,
  // solver iterations, transfer, estimate summary, marking); for a
  // MeshPlanner the iteration just estimated and marked (its `refinement` is
  // still empty). nullptr outside adapt() and before the first solve.
  // Include fem/adaptivity.hpp to read it.
  const AdaptiveIteration *previous = nullptr;
  // Inside adapt(): the index of the iteration being planned.
  std::size_t iteration = 0;
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
  // The plan to run: `defaults` (the user's plan, or inside adapt() the
  // plan of the previous iteration) with the settings this planner chooses
  // changed through the setters. Must keep the time settings present or
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

// The planner's choice as a decision of an adaptive iteration
// (DecisionSubject::SolvePlanner or MeshPlanner): its descriptor, or
// "<name> (not replayable)".
inline std::string planner_choice(const PlannerRecord &planner) {
  if (planner.descriptor)
    return pde2d::describe_strategy(*planner.descriptor);
  return planner.name + " (not replayable)";
}

namespace planner_detail {
template <class Planner> PlannerRecord record(const Planner &planner) {
  return {planner.name(), planner.descriptor(),
          dynamic_cast<const UserPlanner *>(&planner) != nullptr};
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
  planned.planner = planner_detail::record(planner);
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
  planned.planner = planner_detail::record(planner);
  return planned;
}
} // namespace pde2d::fem
