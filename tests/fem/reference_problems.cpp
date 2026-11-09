// The reference problem library (fem/reference/problems.hpp) and plan
// comparisons on it (fem/reference/compare.hpp). Oracles:
//  - each exact solution satisfies its problem: the strong-form residual
//    m2 u_tt + m1 u_t + a_xx u_xx + a_xy u_xy + a_yy u_yy + b.grad u + c u
//    - source, with the derivatives of u taken by fourth-order finite
//    differences of the evaluated exact solution (independent of the
//    symbolic manufacturing), vanishes at interior sample points and times;
//    every boundary condition holds at points sampled on the source curves
//    with the outward normal of the curve; initial values (and velocities)
//    match u (and u_t) at the start;
//  - every problem converges: halving the mesh size reduces the L2 error;
//  - comparing a direct and an iterative solver gives the same errors to the
//    solver tolerance and different work, and the records differ only in
//    the solver settings and their consequences;
//  - comparing planners: the identity baseline equals the plain comparison
//    of the default plans bit for bit, a planner's entry equals the plain
//    comparison of the plans it chose, and both are reported relative to
//    the baseline with the planners' decisions.
#include "discretization/geometry.hpp"
#include "discretization/triangulate.hpp"
#include "fem/evolution.hpp"
#include "fem/expression.hpp"
#include "fem/mesh_plan.hpp"
#include "fem/plan.hpp"
#include "fem/planner.hpp"
#include "fem/record_json.hpp"
#include "fem/reference/compare.hpp"
#include "fem/reference/problems.hpp"
#include "support/check.hpp"
#include <cmath>
#include <memory>
#include <iostream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

using namespace pde2d::fem;
namespace disc = pde2d::discretization;
using pde2d_test::check;
using pde2d_test::expect_throws;

namespace {
// Fourth-order central differences of an evaluated field.
struct Derivatives {
  double u, ux, uy, uxx, uyy, uxy, ut, utt;
};
Derivatives derivatives(const Evaluator &u, double px, double py, double pt) {
  const auto at = [&](double a, double b, double c) {
    return u(EvaluationPoint{a, b, c});
  };
  const double h = 1e-3;
  const auto first = [&](auto f) {
    return (-f(2 * h) + 8 * f(h) - 8 * f(-h) + f(-2 * h)) / (12 * h);
  };
  const auto second = [&](auto f) {
    return (-f(2 * h) + 16 * f(h) - 30 * f(0.) + 16 * f(-h) - f(-2 * h)) /
           (12 * h * h);
  };
  const auto along_x = [&](double d) { return at(px + d, py, pt); };
  const auto along_y = [&](double d) { return at(px, py + d, pt); };
  const auto along_t = [&](double d) { return at(px, py, pt + d); };
  Derivatives r{};
  r.u = at(px, py, pt);
  r.ux = first(along_x);
  r.uy = first(along_y);
  r.uxx = second(along_x);
  r.uyy = second(along_y);
  r.ut = first(along_t);
  r.utt = second(along_t);
  r.uxy = (at(px + h, py + h, pt) - at(px + h, py - h, pt) -
           at(px - h, py + h, pt) + at(px - h, py - h, pt)) /
          (4 * h * h);
  return r;
}
double value(const Expr &e, EvaluationPoint p) { return Evaluator(e)(p); }

std::vector<double> sample_times(const ReferenceProblem &r) {
  if (!r.transient)
    return {0};
  const auto &w = *r.transient;
  return {w.start, (w.start + w.end) / 2, w.end};
}

void exact_solutions_satisfy_the_pde() {
  for (const auto &r : reference_problems()) {
    const auto mesh = disc::generate_mesh(r.geometry, r.mesh);
    check(mesh.report.size_satisfied, r.name + ": suggested mesh");
    const Evaluator u(r.exact);
    const auto &e = r.problem.equation;
    std::size_t samples = 0;
    double worst = 0;
    for (std::size_t k = 0; k < mesh.mesh.elements.size(); k += 3) {
      const auto &tri = mesh.mesh.elements[k];
      double px = 0, py = 0;
      for (int j = 0; j < 3; ++j) {
        px += mesh.mesh.nodes[tri[j]].x / 3;
        py += mesh.mesh.nodes[tri[j]].y / 3;
      }
      if (std::hypot(px, py) < .2 && r.name == "l_shape_corner")
        continue; // the singular corner
      for (const double pt : sample_times(r)) {
        const auto d = derivatives(u, px, py, pt);
        const EvaluationPoint p{px, py, pt};
        const double terms[] = {
            value(e.m2, p) * d.utt,   value(e.m1, p) * d.ut,
            value(e.a_xx, p) * d.uxx, value(e.a_xy, p) * d.uxy,
            value(e.a_yy, p) * d.uyy, value(e.b_x, p) * d.ux,
            value(e.b_y, p) * d.uy,   value(e.c, p) * d.u};
        const double source = value(e.source, p);
        double residual = -source, scale = std::abs(source);
        for (const double term : terms) {
          residual += term;
          scale += std::abs(term);
        }
        worst = std::max(worst, std::abs(residual) / std::max(1.0, scale));
        ++samples;
      }
    }
    std::cout << r.name << ": " << samples
              << " interior samples, worst relative residual " << worst << '\n';
    check(samples > 10, r.name + ": interior samples");
    check(worst < 1e-6, r.name + ": the exact solution solves the equation");
  }
}

// Outward normal of the domain at a curve point: the loop's orientation
// (shoelace of its sampled curves) decides the side; holes face inwards.
void boundary_conditions_hold() {
  for (const auto &r : reference_problems()) {
    const auto &curves = r.geometry.curves();
    std::vector<double> area(r.geometry.loopCount(), 0.0);
    for (const auto &c : curves)
      for (int k = 0; k < 64; ++k) {
        const auto a = disc::curvePoint(c.segment, k / 64.0),
                   b = disc::curvePoint(c.segment, (k + 1) / 64.0);
        area[c.loop] += a.x * b.y - b.x * a.y;
      }
    const Evaluator u(r.exact);
    const auto &e = r.problem.equation;
    double worst = 0;
    std::set<std::string> regions;
    for (const auto &c : curves) {
      const BoundaryCondition *bc = nullptr;
      for (const auto &a : r.problem.boundaries)
        for (const auto &name : a.regions)
          if (name == c.region)
            bc = &a.condition;
      check(bc != nullptr,
            r.name + ": region " + c.region + " has a condition");
      regions.insert(c.region);
      for (const double s : {.13, .37, .61, .87}) {
        const auto p = disc::curvePoint(c.segment, s);
        const auto d = disc::curveDerivative(c.segment, s);
        const double length = std::hypot(d.x, d.y);
        // Right of the direction of travel.
        double nx_ = d.y / length, ny_ = -d.x / length;
        const bool ccw = area[c.loop] > 0;
        // Outer loop: outward is right of a counterclockwise loop. Holes:
        // the domain's outward normal points into the hole, left of it.
        if ((c.loop == 0) != ccw) {
          nx_ = -nx_;
          ny_ = -ny_;
        }
        for (const double pt : sample_times(r)) {
          const auto du = derivatives(u, p.x, p.y, pt);
          const EvaluationPoint q{p.x, p.y, pt, nx_, ny_};
          const double axx = value(e.a_xx, q), axy = value(e.a_xy, q),
                       ayy = value(e.a_yy, q);
          const double q_n = -(nx_ * (axx * du.ux + axy / 2 * du.uy) +
                               ny_ * (axy / 2 * du.ux + ayy * du.uy));
          const double terms[] = {value(bc->alpha, q) * du.u,
                                  value(bc->gamma, q) * du.ut,
                                  value(bc->beta_flux, q) * q_n,
                                  value(bc->beta_normal, q) *
                                      (nx_ * du.ux + ny_ * du.uy),
                                  value(bc->beta_x, q) * du.ux,
                                  value(bc->beta_y, q) * du.uy};
          const double target = value(bc->value, q);
          double residual = -target, scale = std::abs(target);
          for (const double term : terms) {
            residual += term;
            scale += std::abs(term);
          }
          worst = std::max(worst, std::abs(residual) / std::max(1.0, scale));
        }
      }
    }
    std::cout << r.name << ": " << regions.size()
              << " regions, worst relative boundary residual " << worst << '\n';
    for (const auto &a : r.problem.boundaries)
      for (const auto &name : a.regions)
        check(regions.contains(name), r.name + ": region " + name + " sampled");
    check(worst < 1e-7, r.name + ": the exact solution meets its conditions");
  }
}

void initial_conditions_match() {
  std::size_t transient = 0;
  for (const auto &r : reference_problems()) {
    check(r.time_dependent() == r.exact_time_dependent(),
          r.name + ": only transient exact solutions depend on t");
    if (!r.transient)
      continue;
    ++transient;
    const auto &w = *r.transient;
    const Evaluator u(r.exact);
    const auto &value_field = std::get<Expr>(w.initial.value);
    for (const double px : {.1, .45, .8})
      for (const double py : {.2, .5, .7}) {
        const EvaluationPoint p{px, py, w.start};
        check(std::abs(value(value_field, p) - u(p)) < 1e-14,
              r.name + ": initial value");
        if (!is_zero(r.problem.equation.m2)) {
          check(w.initial.velocity.has_value(),
                r.name + ": second order needs a velocity");
          const auto d = derivatives(u, px, py, w.start);
          check(std::abs(value(std::get<Expr>(*w.initial.velocity), p) - d.ut) <
                    1e-8,
                r.name + ": initial velocity");
        }
      }
  }
  check(transient == 2, "a heat and a wave problem");
  expect_throws<std::invalid_argument>(
      [] { (void)find_reference_problem("nope"); },
      "Unknown reference problem: nope");
  check(find_reference_problem("annulus_mixed").name == "annulus_mixed");
}

SolvePlan default_plan(const ReferenceProblem &r) {
  if (!r.transient)
    return plan_from_user(AssemblyOptions{6});
  TimePlan time;
  time.start = r.transient->start;
  time.end = r.transient->end;
  time.step = .01;
  time.method = !is_zero(r.problem.equation.m2) ? TimeMethod::Newmark
                                                : TimeMethod::CrankNicolson;
  return plan_from_user(AssemblyOptions{6}, SolverOptions{}, time);
}

void every_problem_converges() {
  for (const auto &r : reference_problems()) {
    auto fine = r.mesh;
    fine.boundary.max_edge_length /= 2;
    fine.refinement.max_edge_length /= 2;
    const auto c = compare_mesh_plans(
        r, {mesh_plan_from_user(r.mesh), mesh_plan_from_user(fine)},
        default_plan(r));
    check(c.kind == ComparisonKind::MeshPlans && c.entries.size() == 2);
    const auto &coarse_entry = c.entries[0], &fine_entry = c.entries[1];
    std::cout << r.name << ": nodes " << coarse_entry.work.nodes << " -> "
              << fine_entry.work.nodes << ", L2 " << coarse_entry.errors.l2
              << " -> " << fine_entry.errors.l2 << ", H1 "
              << coarse_entry.errors.h1_seminorm << " -> "
              << fine_entry.errors.h1_seminorm << '\n';
    check(fine_entry.work.nodes > 2 * coarse_entry.work.nodes,
          r.name + ": the finer mesh has more nodes");
    check(fine_entry.errors.l2 < .7 * coarse_entry.errors.l2,
          r.name + ": the L2 error decreases with the mesh size");
    if (r.name == "l_shape_corner") {
      // The corner singularity limits uniform meshes to H1 rate 2/3 (the
      // mesh size halves).
      const double rate = std::log2(coarse_entry.errors.h1_seminorm /
                                    fine_entry.errors.h1_seminorm);
      check(rate > .5 && rate < .85, "L-shape: H1 rate near 2/3");
    }
    const double end = r.transient ? r.transient->end : 0;
    check(coarse_entry.errors.time == end, r.name + ": compared at the end");
    check(
        coarse_entry.record.mesh.provenance && coarse_entry.record.plan.mesh &&
            coarse_entry.record.plan.mesh->options.refinement.max_edge_length ==
                r.mesh.refinement.max_edge_length,
        r.name + ": the record carries the mesh plan it ran on");
    check(coarse_entry.work.mesh_refinement_steps ==
              coarse_entry.record.mesh.provenance->run.work.refinement_steps,
          r.name + ": mesh work from the mesh run record");
  }
}

void direct_and_iterative_solvers() {
  const auto r = find_reference_problem("annulus_mixed");
  const auto lu = plan_from_user(AssemblyOptions{6});
  SolverOptions cg;
  cg.method = LinearSolver::ConjugateGradient;
  const auto iterative = plan_from_user(AssemblyOptions{6}, cg);
  const auto c = compare_plans(r, mesh_plan_from_user(r.mesh), {lu, iterative});
  check(c.reference == "annulus_mixed" &&
        c.kind == ComparisonKind::SolvePlans && c.entries.size() == 2);
  const auto &a = c.entries[0], &b = c.entries[1];
  std::cout << a.label << ": L2 " << a.errors.l2 << " iterations "
            << a.work.solver_iterations << "; " << b.label << ": L2 "
            << b.errors.l2 << " iterations " << b.work.solver_iterations
            << '\n';
  check(a.label == "SparseLU" && b.label == "ConjugateGradient", "labels");
  // The solver tolerance (relative residual 1e-10) bounds the difference
  // far below the discretization error.
  check(std::abs(a.errors.l2 - b.errors.l2) <= 1e-7 * a.errors.l2 &&
            std::abs(a.errors.h1_seminorm - b.errors.h1_seminorm) <=
                1e-7 * a.errors.h1_seminorm,
        "same errors to the solver tolerance");
  check(b.work.solver_iterations > 10 &&
            b.work.solver_iterations != a.work.solver_iterations,
        "different work");
  check(a.work.nodes == b.work.nodes &&
            a.record.mesh.fingerprint == b.record.mesh.fingerprint,
        "one mesh");
  // The records differ in the solver and its consequences only.
  for (const auto &d : diff_records(a.record, b.record))
    check(d.path.starts_with("plan.solver.") ||
              d.path.starts_with("plan.decisions") ||
              d.path.starts_with("strategies") ||
              d.path == "work.solver_iterations",
          "unexpected difference " + d.path);
  bool outcome = false;
  for (const auto &d :
       diff_records(a.record, b.record, {.ignore_outcomes = false}))
    outcome = outcome || d.path == "stationary_solve.outcome.iterations";
  check(outcome, "outcomes differ when compared");

  // JSON: schema, embedded records, exact round trip.
  std::ostringstream out;
  write_json(out, c);
  std::istringstream in(out.str());
  const auto doc = read_plan_comparison_json(in);
  check(doc.at("reference").text == "annulus_mixed" &&
            doc.at("entries").items.size() == 2 &&
            doc.at("entries").items[1].at("record").at("schema").text ==
                run_record_schema,
        "comparison document");
  check(doc.at("entries").items[0].at("errors").at("l2").number() ==
            a.errors.l2,
        "errors round-trip exactly");
  check(json_text(doc) == out.str(), "byte-identical re-serialization");
  std::istringstream wrong("{\"schema\": \"x\"}");
  expect_throws<std::invalid_argument>(
      [&] { (void)read_plan_comparison_json(wrong); }, "Not a plan comparison");
  check(diff_records(to_json(c),
                     to_json(compare_plans(r, mesh_plan_from_user(r.mesh),
                                           {lu, iterative})))
            .empty(),
        "a repeated comparison differs only in timings");
}

const Rationale planned{"planned for the test", Evidence::Unknown,
                        DecisionSource::Automatic};
class HalveTarget final : public MeshPlanner {
public:
  std::string name() const override { return "HalveTarget"; }
  MeshPlan plan(const Problem &, const ProblemProperties &,
                const MeshPlan &defaults,
                const PlanningContext &context) const override {
    if (!context.geometry || !context.boundary || context.mesh)
      throw std::logic_error("planning before the first mesh");
    auto plan = defaults;
    plan.set_max_edge_length(defaults.options.refinement.max_edge_length / 2,
                             planned);
    return plan;
  }
};
class UseCg final : public SolvePlanner {
public:
  std::string name() const override { return "UseCg"; }
  std::optional<pde2d::StrategyDescriptor> descriptor() const override {
    return pde2d::StrategyDescriptor{"UseCg"};
  }
  SolvePlan plan(const Problem &, const ProblemProperties &,
                 const SolvePlan &defaults,
                 const PlanningContext &context) const override {
    if (!context.geometry || !context.mesh)
      throw std::logic_error("planning on the planned mesh");
    auto plan = defaults;
    plan.set_linear_solver(LinearSolver::ConjugateGradient, planned);
    return plan;
  }
};

void planners_against_the_baseline() {
  const auto r = find_reference_problem("annulus_mixed");
  const auto defaults = plan_from_user(AssemblyOptions{6});
  const auto c = compare_planners(r,
                                  {{std::make_shared<HalveTarget>(), nullptr},
                                   {nullptr, std::make_shared<UseCg>()}},
                                  defaults);
  check(c.kind == ComparisonKind::Planners && c.entries.size() == 3 &&
            c.baseline == 0,
        "baseline plus one entry per planner set");
  const auto &base = c.entries[0], &finer = c.entries[1], &cg = c.entries[2];
  check(base.label == "User (baseline)" &&
            finer.label == "mesh: HalveTarget, solve: User" &&
            cg.label == "mesh: User, solve: UseCg",
        "labels");
  // Oracles: the plain comparisons of the plans the planners chose.
  auto halved = mesh_plan_from_user(r.mesh);
  halved.set_max_edge_length(r.mesh.refinement.max_edge_length / 2, planned);
  auto cg_plan = defaults;
  cg_plan.set_linear_solver(LinearSolver::ConjugateGradient, planned);
  const auto plain =
      compare_mesh_plans(r, {mesh_plan_from_user(r.mesh), halved}, defaults);
  const auto plain_cg =
      compare_plans(r, mesh_plan_from_user(r.mesh), {cg_plan});
  check(base.errors.l2 == plain.entries[0].errors.l2 &&
            base.errors.h1_seminorm == plain.entries[0].errors.h1_seminorm &&
            finer.errors.l2 == plain.entries[1].errors.l2 &&
            cg.errors.l2 == plain_cg.entries[0].errors.l2,
        "entries are the plain solves of the planned plans, bit for bit");
  check(base.planner_decisions.empty() && base.relative &&
            base.relative->l2 == 1 && base.relative->nodes == 1 &&
            base.timings.relative_seconds == 1.0,
        "the baseline is relative 1 and changed nothing");
  check(finer.planner_decisions.size() == 1 &&
            finer.planner_decisions[0].subject == DecisionSubject::MeshSize &&
            finer.planner_decisions[0].source == DecisionSource::Automatic,
        "the mesh planner's decision");
  check(finer.relative->nodes > 2 && finer.relative->l2 < .5,
        "a finer mesh: more nodes, smaller error");
  std::cout << "planners: CG relative l2 " << cg.relative->l2 << " iterations "
            << cg.relative->solver_iterations << " (LU "
            << base.work.solver_iterations << "), decisions "
            << cg.planner_decisions.size() << "; finer nodes "
            << finer.relative->nodes << " l2 " << finer.relative->l2 << '\n';
  check(cg.planner_decisions.size() == 1 &&
            cg.planner_decisions[0].subject == DecisionSubject::LinearSolver &&
            std::abs(cg.relative->l2 - 1) < 1e-6 && cg.relative->nodes == 1 &&
            cg.relative->solver_iterations > 10,
        "the solve planner's decision; many more iterations than LU");
  check(find_strategy(cg.record, DecisionSubject::SolvePlanner)->name ==
                "UseCg" &&
            find_strategy(finer.record, DecisionSubject::MeshPlanner)->name ==
                "HalveTarget",
        "the planners are in the run records");
  for (const auto &e : c.entries)
    check(e.timings.planning_seconds >= 0 &&
              e.timings.relative_seconds.has_value(),
          "planning cost");
  std::ostringstream out;
  write_json(out, c);
  std::istringstream in(out.str());
  const auto doc = read_plan_comparison_json(in);
  check(doc.at("kind").text == "planners" && doc.at("baseline").number() == 0 &&
            doc.at("entries")
                    .items[2]
                    .at("relative")
                    .at("solver_iterations")
                    .number() == cg.relative->solver_iterations &&
            doc.at("entries").items[1].at("planner_decisions").items.size() ==
                1,
        "comparison JSON");
}

void plans_must_fit_the_problem() {
  const auto heat = find_reference_problem("heat_moving_peak");
  const auto mesh_plan = mesh_plan_from_user(heat.mesh);
  expect_throws<std::invalid_argument>(
      [&] {
        (void)compare_plans(heat, mesh_plan,
                            {plan_from_user(AssemblyOptions{})});
      },
      "is transient");
  auto shifted = default_plan(heat);
  shifted.time->end = .5;
  expect_throws<std::invalid_argument>(
      [&] { (void)compare_plans(heat, mesh_plan, {shifted}); },
      "does not match reference problem heat_moving_peak window");
  const auto annulus = find_reference_problem("annulus_mixed");
  expect_throws<std::invalid_argument>(
      [&] {
        (void)compare_plans(annulus, mesh_plan_from_user(annulus.mesh),
                            {default_plan(heat)});
      },
      "is stationary");
}
} // namespace

int main() {
  return pde2d_test::run(
      {{"exact_solutions_satisfy_the_pde", exact_solutions_satisfy_the_pde},
       {"boundary_conditions_hold", boundary_conditions_hold},
       {"initial_conditions_match", initial_conditions_match},
       {"every_problem_converges", every_problem_converges},
       {"direct_and_iterative_solvers", direct_and_iterative_solvers},
       {"planners_against_the_baseline", planners_against_the_baseline},
       {"plans_must_fit_the_problem", plans_must_fit_the_problem}});
}
