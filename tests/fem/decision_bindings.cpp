// The binding tables of plans (solve_plan_bindings in plan.hpp,
// mesh_plan_bindings in mesh_plan.hpp): every decision a plan records, its
// setter and its validation come from one row per subject. Oracles:
//  - for every bound option, an edit of the raw options without its decision
//    is detected ("Stale decision: <subject name>"); the test's list of edits
//    must cover every row, so a new row without a test fails here;
//  - the same edit through update() keeps the plan valid and records a user
//    decision naming the row's option;
//  - for every subject, a decision in a plan that does not own it is
//    rejected ("does not belong to a ... plan"), and update() refuses it.
#include "discretization/size_criterion.hpp"
#include "fem/mesh_plan.hpp"
#include "fem/plan.hpp"
#include "support/check.hpp"
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

using pde2d_test::check;
using pde2d_test::expect_throws;
namespace {
using namespace pde2d::fem;
namespace disc = pde2d::discretization;

std::string name(DecisionSubject subject) {
  return std::string(decision_subject_name(subject));
}

// A transient plan whose decisions cover every solve-plan row.
SolvePlan full_solve_plan() {
  auto plan = plan_from_user({}, {}, TimePlan{});
  plan.set_quadrature_degree(3);
  return plan;
}

using SolveEdit = std::function<void(SolvePlan &)>;
// Raw edits of each bound solve option (several per row where a row reads
// several fields).
std::map<DecisionSubject, std::vector<SolveEdit>> solve_edits() {
  return {
      {DecisionSubject::TimeIntegrator,
       {[](SolvePlan &p) { p.time->method = TimeMethod::CrankNicolson; },
        [](SolvePlan &p) { p.time->method = TimeMethod::Newmark; }}},
      {DecisionSubject::TimeSteps, {[](SolvePlan &p) { p.time->step = .02; }}},
      {DecisionSubject::LinearSolver,
       {[](SolvePlan &p) { p.solver.method = LinearSolver::GMRES; },
        [](SolvePlan &p) {
          p.solver.backend = make_linear_solver_backend(
              "Other",
              [](const SolverOptions &)
                  -> std::unique_ptr<LinearSolverBackend> {
                throw std::invalid_argument("not created in this test");
              });
        }}},
      {DecisionSubject::Preconditioner, {[](SolvePlan &p) {
         p.solver.preconditioner = Preconditioner::Identity;
       }}},
      {DecisionSubject::SolverValidation, {[](SolvePlan &p) {
         p.solver.validation = SolverValidation::AssumeValid;
       }}},
      {DecisionSubject::Quadrature,
       {[](SolvePlan &p) { p.assembly.quadrature_degree = 4; }}},
  };
}

void every_solve_option_is_checked() {
  const auto edits = solve_edits();
  check(edits.size() == solve_plan_bindings.size(),
        "One entry of edits per solve-plan row");
  const auto base = full_solve_plan();
  check(base.decisions.size() == solve_plan_bindings.size(),
        "The base plan records every row");
  validate_decisions(base);
  for (const auto &row : solve_plan_bindings) {
    const auto at = edits.find(row.subject);
    check(at != edits.end() && !at->second.empty(),
          "No edit for row " + name(row.subject));
    for (const auto &edit : at->second) {
      auto raw = base;
      edit(raw);
      expect_throws<std::invalid_argument>([&] { validate_decisions(raw); },
                                           "Stale decision: " +
                                               name(row.subject));
      // The same edit through the table-driven setter stays valid.
      auto updated = base;
      updated.update(row.subject, edit);
      validate_decisions(updated);
      const auto *d = find_decision(updated.decisions, row.subject);
      check(d && d->choice == row.choice(updated).value_or("") &&
                d->basis == "user-selected (" +
                                std::string(row.option(updated)) + ")" &&
                d->source == DecisionSource::User,
            "update() records the row's decision: " + name(row.subject));
      check(updated.decisions.size() == base.decisions.size() &&
                d - updated.decisions.data() ==
                    find_decision(base.decisions, row.subject) -
                        base.decisions.data(),
            "update() keeps the decision's position");
    }
  }
  // Factory-made strategies: the plan cannot name them, but removing the
  // factory behind the decision's back is detected.
  auto factory = base;
  factory.set_time_integrator(
      [] { return std::make_unique<CrankNicolsonIntegrator>(); });
  validate_decisions(factory);
  auto removed = factory;
  removed.time->integrator = {};
  expect_throws<std::invalid_argument>([&] { validate_decisions(removed); },
                                       "Stale decision: time integrator");
  // After running, a recorded name must match what the factory made.
  factory.decisions[0].choice = "BackwardEuler";
  expect_throws<std::invalid_argument>(
      [&] {
        validate_decisions(
            factory, {{{DecisionSubject::TimeIntegrator, "CrankNicolson"}}});
      },
      "Stale decision: time integrator records \"BackwardEuler\"");
}

using MeshEdit = std::function<void(disc::MeshOptions &)>;
std::map<DecisionSubject, std::vector<MeshEdit>> mesh_edits() {
  return {
      {DecisionSubject::MeshAlgorithm, {[](disc::MeshOptions &o) {
         o.algorithm = disc::InteriorAlgorithm::EarClipping;
       }}},
      {DecisionSubject::BoundaryTreatment,
       {[](disc::MeshOptions &o) {
          o.boundary_treatment = disc::BoundaryTreatment::SplitPolygonEdges;
        },
        [](disc::MeshOptions &o) { o.boundary.preserve_corners = true; },
        [](disc::MeshOptions &o) { o.preserve_regions = false; }}},
      {DecisionSubject::MeshSize,
       {[](disc::MeshOptions &o) { o.boundary.max_edge_length = .5; },
        [](disc::MeshOptions &o) { o.boundary.max_curve_deviation = .01; },
        [](disc::MeshOptions &o) { o.refinement.max_edge_length = .2; },
        [](disc::MeshOptions &o) { o.refinement.max_steps = 7; }}},
      {DecisionSubject::SizeField,
       // A new callable under the same descriptor is not visible (callables
       // cannot be compared; the descriptor is what names a size field).
       {[](disc::MeshOptions &o) { o.refinement.size_field = {}; },
        [](disc::MeshOptions &o) {
          o.refinement.size_field_descriptor =
              pde2d::StrategyDescriptor{"Other"};
        }}},
      // The whole-mesh decision explains every option together: no raw
      // edit makes it stale.
      {DecisionSubject::Mesh, {}},
      {DecisionSubject::RefinementCriterion, {[](disc::MeshOptions &o) {
         o.refinement.criterion->descriptor =
             pde2d::StrategyDescriptor{"Other"};
       }}},
  };
}

void every_mesh_option_is_checked() {
  disc::MeshOptions options;
  options.refinement.size_field = [](pde2d::discretization::Point2) {
    return .3;
  };
  options.refinement.size_field_descriptor =
      pde2d::StrategyDescriptor{"Constant", {{"h", "0.3"}}};
  auto base = mesh_plan_from_user(options);
  disc::RefinementCriterion criterion;
  criterion.size = disc::constant_size_criterion(.4);
  criterion.descriptor = pde2d::StrategyDescriptor{"Constant", {{"h", "0.4"}}};
  base.set_refinement_criterion(criterion);
  base.decisions.push_back({DecisionSubject::Mesh, "graded", "test",
                            Evidence::Unknown, DecisionSource::Automatic});
  check(base.decisions.size() == mesh_plan_bindings.size(),
        "The base mesh plan records every row");
  validate_decisions(base);
  const auto edits = mesh_edits();
  check(edits.size() == mesh_plan_bindings.size(),
        "One entry of edits per mesh-plan row");
  for (const auto &row : mesh_plan_bindings) {
    const auto at = edits.find(row.subject);
    check(at != edits.end(), "No edit for row " + name(row.subject));
    check(at->second.empty() == !row.choice(base.options).has_value(),
          "Only the whole-mesh row has no edit");
    for (const auto &edit : at->second) {
      auto raw = base;
      edit(raw.options);
      expect_throws<std::invalid_argument>([&] { validate_decisions(raw); },
                                           "Stale decision: " +
                                               name(row.subject));
      // Against the options a mesh was made with, too.
      expect_throws<std::invalid_argument>(
          [&] { validate_decisions(base, raw.options); },
          "Stale decision: " + name(row.subject));
      check(mesh_options_difference(base.options, raw.options) == row.subject,
            "The difference names the row");
      auto updated = base;
      updated.update(row.subject, edit);
      validate_decisions(updated);
      check(find_decision(updated.decisions, row.subject)->basis ==
                "user-selected (" + std::string(row.option(updated.options)) +
                    ")",
            "update() records the row's decision: " + name(row.subject));
    }
  }
  // The named setters are update() with one row each.
  auto by_setter = mesh_plan_from_user({});
  by_setter.set_interior_algorithm(disc::InteriorAlgorithm::EarClipping)
      .set_boundary_treatment(disc::BoundaryTreatment::RefineOnCurves)
      .set_max_edge_length(.25);
  disc::MeshOptions direct;
  direct.algorithm = disc::InteriorAlgorithm::EarClipping;
  direct.boundary_treatment = disc::BoundaryTreatment::RefineOnCurves;
  direct.refinement.max_edge_length = .25;
  const auto expected = mesh_decisions(direct);
  check(by_setter.decisions.size() == expected.size(), "Setter decisions");
  for (std::size_t k = 0; k < expected.size(); ++k)
    check(by_setter.decisions[k].choice == expected[k].choice,
          "Setters give the decisions of the same options");
  // A planner's strategies must be replayable: the setters require a
  // descriptor.
  expect_throws<std::invalid_argument>(
      [&] { by_setter.set_refinement_criterion(disc::RefinementCriterion{}); },
      "Refinement criterion override needs a descriptor");
  by_setter.set_size_field([](pde2d::discretization::Point2) { return .1; },
                           pde2d::StrategyDescriptor{"Constant"});
  check(
      find_decision(by_setter.decisions, DecisionSubject::SizeField)->choice ==
          "Constant",
      "Size field setter");
}

void owners_are_enforced() {
  for (const auto subject : decision_subjects) {
    const auto owner = decision_subject_owner(subject);
    const Decision foreign{subject, "x", "test", Evidence::Unknown,
                           DecisionSource::User};
    if (owner != DecisionOwner::Solve && owner != DecisionOwner::Time) {
      auto plan = plan_from_user({}, {}, TimePlan{});
      plan.decisions.push_back(foreign);
      expect_throws<std::invalid_argument>(
          [&] { validate_decisions(plan); },
          "Decision subject " + name(subject) +
              " does not belong to a solve plan (owner " +
              decision_owner_name(owner));
      expect_throws<std::invalid_argument>(
          [&] { plan.update(subject, [](SolvePlan &) {}); },
          "does not belong to a solve plan");
    }
    if (owner != DecisionOwner::Mesh) {
      auto mesh_plan = mesh_plan_from_user({});
      mesh_plan.decisions.push_back(foreign);
      expect_throws<std::invalid_argument>(
          [&] { validate_decisions(mesh_plan); },
          "Decision subject " + name(subject) +
              " does not belong to a mesh plan (owner " +
              decision_owner_name(owner));
      expect_throws<std::invalid_argument>(
          [&] { mesh_plan.update(subject, [](disc::MeshOptions &) {}); },
          "does not belong to a mesh plan");
    }
  }
  // A time subject needs time settings.
  auto stationary = plan_from_user({}, {});
  expect_throws<std::invalid_argument>(
      [&] {
        stationary.update(DecisionSubject::TimeSteps, [](SolvePlan &) {});
      },
      "Plan has no time settings");
  // Tables hold exactly the subjects their plans own.
  std::size_t solve = 0, mesh = 0;
  for (const auto subject : decision_subjects) {
    const auto owner = decision_subject_owner(subject);
    solve += owner == DecisionOwner::Solve || owner == DecisionOwner::Time;
    mesh += owner == DecisionOwner::Mesh;
  }
  check(solve == solve_plan_bindings.size() &&
            mesh == mesh_plan_bindings.size(),
        "Table sizes");
}

void failed_update_keeps_the_plan() {
  auto plan = plan_from_user({}, {});
  const auto before = plan.solver;
  expect_throws<std::invalid_argument>(
      [&] {
        plan.update(DecisionSubject::Marking, [](SolvePlan &p) {
          p.solver.method = LinearSolver::GMRES;
        });
      },
      "does not belong to a solve plan");
  check(plan.solver == before, "A refused update changes nothing");
  validate_decisions(plan);
}
} // namespace
int main() {
  return pde2d_test::run(
      {{"every_solve_option_is_checked", every_solve_option_is_checked},
       {"every_mesh_option_is_checked", every_mesh_option_is_checked},
       {"owners_are_enforced", owners_are_enforced},
       {"failed_update_keeps_the_plan", failed_update_keeps_the_plan}});
}
