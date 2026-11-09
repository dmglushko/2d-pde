// Complete records of warm-started and adaptive runs (record.hpp,
// solver.hpp, adaptivity.hpp, adaptivity_json.hpp). Oracles: values known
// from the inputs (the fingerprint of the guess computed separately, the
// solver report of the same solve, the AdaptOptions that were passed),
// relations between counters (summed iterations of an evolution equal the
// sum over its step reports), the JSON documents, and diffs between two
// runs that differ in one setting (exactly that setting's paths differ).
#include "discretization/geometry.hpp"
#include "discretization/mesh_fingerprint.hpp"
#include "discretization/square_mesh.hpp"
#include "discretization/triangulate.hpp"
#include "fem/adaptivity.hpp"
#include "fem/adaptivity_json.hpp"
#include "fem/evolution.hpp"
#include "fem/fingerprint.hpp"
#include "fem/record_json.hpp"
#include "fem/solver.hpp"
#include "support/check.hpp"
#include <algorithm>
#include <cmath>
#include <set>
#include <sstream>
#include <string>

using namespace pde2d::fem;
namespace disc = pde2d::discretization;
using pde2d_test::check;

namespace {
const std::initializer_list<std::string> sides{"bottom", "right", "top",
                                               "left"};
const Expr peak = exp(-40 * ((x + .2) * (x + .2) + (y - .1) * (y - .1)));
Problem peak_problem() {
  Problem p;
  p.equation.source = -(diff(diff(peak, x), x) + diff(diff(peak, y), y));
  p.assign(dirichlet(peak), sides);
  return p;
}
SolvePlan cg_plan() {
  SolverOptions cg;
  cg.method = LinearSolver::ConjugateGradient;
  return plan_from_user(AssemblyOptions{4}, cg);
}
std::set<std::string> paths(const std::vector<RecordDifference> &diff) {
  std::set<std::string> out;
  for (const auto &d : diff)
    out.insert(d.path);
  return out;
}
// The unit square centered at the origin (RectangleGeometry).
disc::MeshResult square(double h) {
  disc::MeshOptions o;
  o.boundary.max_edge_length = h;
  o.refinement.max_edge_length = h;
  o.boundary_treatment = disc::BoundaryTreatment::SplitPolygonEdges;
  return disc::generate_mesh(disc::RectangleGeometry(1, 1), o);
}

void warm_started_solve_records_its_guess() {
  const auto mesh = disc::square_mesh(8);
  const auto problem = peak_problem();
  PreparedStationary solver(mesh, problem, cg_plan());
  const auto cold = solver.solve();
  const auto &c = cold.record.stationary_solve;
  check(c && c->initial_guess == InitialGuess::None && !c->guess,
        "a cold solve has no guess");
  check(c->iterations == cold.report.iterations && c->converged &&
            c->residual_norm == cold.report.residual_norm &&
            c->relative_residual == cold.report.relative_residual,
        "iterations and final residual of the report");
  check(cold.record.work.solver_iterations ==
            static_cast<std::size_t>(cold.report.iterations),
        "work counts the iterations");

  Vector guess = cold.values;
  guess.array() += 1e-3;
  const auto warm = solver.solve(0, guess);
  const auto &w = warm.record.stationary_solve;
  check(w && w->initial_guess == InitialGuess::Used && w->guess &&
            w->guess->kind == "nodal" &&
            w->guess->fingerprint == field_fingerprint(Field(guess)),
        "the guess as passed, by fingerprint");
  check(w->iterations == warm.report.iterations, "warm iterations");

  // Default diff: inputs, settings and work; outcomes only on request.
  const auto inputs = paths(diff_records(cold.record, warm.record));
  check(inputs.contains("stationary_solve.initial_guess") &&
            inputs.contains("stationary_solve.guess.fingerprint") &&
            inputs.contains("stationary_solve.guess.kind"),
        "the guess differs");
  check(std::ranges::none_of(inputs,
                             [](const std::string &p) {
                               return p.find("outcome") != std::string::npos;
                             }),
        "outcomes are skipped by default");
  const auto all =
      paths(diff_records(cold.record, warm.record, {.ignore_outcomes = false}));
  check(all.contains("stationary_solve.outcome.residual_norm"),
        "outcomes compared on request");

  // A direct solver ignores the guess but records it.
  const auto lu = solve_stationary(mesh, problem,
                                   plan_from_user(AssemblyOptions{4}), guess);
  check(lu.record.stationary_solve->initial_guess == InitialGuess::Ignored &&
            lu.record.stationary_solve->guess->fingerprint ==
                w->guess->fingerprint,
        "an ignored guess is recorded");

  // JSON keys.
  std::ostringstream out;
  write_json(out, warm.record);
  std::istringstream in(out.str());
  const auto doc = read_run_record_json(in);
  const auto &solve = doc.at("stationary_solve");
  check(solve.at("initial_guess").text == "Used" &&
            solve.at("guess").at("fingerprint").text == w->guess->fingerprint &&
            solve.at("outcome").at("iterations").number() ==
                warm.report.iterations &&
            solve.at("outcome").at("relative_residual").number() ==
                warm.report.relative_residual,
        "stationary solve JSON");
  check(doc.at("work").at("solver_iterations").number() ==
            warm.report.iterations,
        "work JSON");

  // Evolutions sum the iterations of every solve.
  Problem heat = problem;
  heat.equation.m1 = 1;
  TimeOptions o;
  o.end = .05;
  o.solver.method = LinearSolver::ConjugateGradient;
  const auto e = evolve(mesh, heat, {peak}, o);
  std::size_t sum = 0;
  for (const auto &r : e.reports)
    sum += static_cast<std::size_t>(r.iterations);
  check(e.record.work.solver_iterations == sum && sum > 0 &&
            e.timings.solver_iterations == sum,
        "evolution iterations");
  check(!e.record.stationary_solve, "no stationary solve in an evolution");
}

void mesh_run_record_in_the_run_record() {
  const auto result = disc::generate_mesh(disc::CircleGeometry(1), [] {
    disc::MeshOptions o;
    o.boundary.max_edge_length = .3;
    o.refinement.max_edge_length = .3;
    return o;
  }());
  Problem p;
  p.equation.source = 1;
  p.assign(dirichlet(0), {"boundary"});
  const auto s = solve_stationary(result, p);
  const auto &run = s.record.mesh.provenance->run;
  check(run.operation == disc::MeshOperation::Generate &&
            run.fingerprint == s.record.mesh.fingerprint &&
            run.work.edge_flips == result.record.work.edge_flips,
        "the run record embeds the mesh run record");
  std::ostringstream out;
  write_json(out, s.record);
  std::istringstream in(out.str());
  const auto doc = read_run_record_json(in);
  const auto &json_run = doc.at("mesh").at("provenance").at("run");
  check(json_run.at("schema").text == disc::mesh_run_record_schema &&
            json_run.at("fingerprint").text == run.fingerprint,
        "mesh run record JSON inside the run record");
  // Identical runs differ only in timings, also those of the mesh record.
  const auto again = solve_stationary(
      disc::generate_mesh(disc::CircleGeometry(1), result.options), p);
  check(diff_records(s.record, again.record).empty(),
        "mesh timings are ignored like the others");
  check(paths(diff_records(s.record, again.record, {.ignore_timings = false}))
            .contains("mesh.provenance.run.timings.total_seconds"),
        "and present");
}

AdaptOptions adapt_options() {
  AdaptOptions o;
  o.max_iterations = 2;
  o.tolerance = 1e-6; // never reached here
  o.transfer.max_outside_distance = 0;
  return o;
}

void adaptive_history_keeps_its_settings() {
  const auto mesh = square(.25);
  const auto problem = peak_problem();
  auto o = adapt_options();
  auto requested = mesh.options;
  requested.refinement.max_steps = 5000;
  o.refinement = requested;
  const auto h = adapt(mesh, problem, cg_plan(), o);
  const auto &s = h.options;
  check(s.max_iterations == 2 && s.tolerance == 1e-6 && s.warm_start &&
            !s.accept_unsatisfied_refinement && s.time == 0 &&
            s.estimator->name() == "gradient recovery (ZZ)" &&
            s.marker->name() == "bulk (Dorfler)" &&
            s.adaptation->name() == "marked longest-edge bisection" &&
            s.refinement && s.refinement->refinement.max_steps == 5000 &&
            !s.planners.solve,
        "settings of the loop");
  check(h.steps[0].decisions[0].basis == "default (AdaptOptions::marker unset)",
        "the default marker is recorded as the default");
  for (std::size_t k = 0; k + 1 < h.steps.size(); ++k) {
    const auto &it = h.steps[k];
    check(it.adaptation &&
              it.adaptation->run.options.refinement.max_steps == 5000 &&
              it.adaptation->run.operation ==
                  disc::MeshOperation::MarkedRefine &&
              it.adaptation->run.source_fingerprint ==
                  it.record.mesh.fingerprint &&
              it.adaptation->run.fingerprint ==
                  h.steps[k + 1].record.mesh.fingerprint,
          "each refinement: requested options and mesh run record");
  }
  std::ostringstream out;
  write_json(out, h);
  std::istringstream in(out.str());
  const auto doc = read_adaptive_history_json(in);
  const auto &settings = doc.at("settings");
  check(
      settings.at("tolerance").number() == 1e-6 &&
          settings.at("max_iterations").number() == 2 &&
          settings.at("warm_start").boolean &&
          settings.at("transfer").at("max_outside_distance").number() == 0 &&
          settings.at("refinement").at("refinement").at("max_steps").number() ==
              5000 &&
          settings.at("estimator").at("name").text == "gradient recovery (ZZ)",
      "settings JSON");
  check(
      doc.at("steps").items[0].at("adaptation").at("run").at("options").kind ==
              JsonValue::Kind::Object &&
          doc.at("steps")
                  .items[0]
                  .at("adaptation")
                  .at("run")
                  .at("schema")
                  .text == disc::mesh_run_record_schema,
      "step JSON");

  // One setting changed: exactly its path differs.
  auto other = o;
  other.tolerance = 2e-6;
  const auto diff = paths(diff_records(
      to_json(h), to_json(adapt(mesh, problem, cg_plan(), other))));
  check(diff == std::set<std::string>{"settings.tolerance"},
        "a tolerance change is one difference");
  other = o;
  other.warm_start = false;
  const auto cold = paths(diff_records(
      to_json(h), to_json(adapt(mesh, problem, cg_plan(), other))));
  check(cold.contains("settings.warm_start") &&
            cold.contains("steps[1].transfer.source_nodes"),
        "warm start: setting and transfer differ");
}

void failed_refinement_keeps_its_options() {
  const auto mesh = square(.25);
  auto o = adapt_options();
  auto budget = mesh.options;
  budget.refinement.max_steps = 0;
  o.refinement = budget;
  const auto h = adapt(mesh, peak_problem(), cg_plan(), o);
  check(h.stop == AdaptiveStop::UnsatisfiedRefinement && h.steps.size() == 1,
        "the refined mesh missed its target");
  const auto &it = h.steps.back();
  check(it.adaptation && it.adaptation->run.options.refinement.max_steps == 0 &&
            !it.adaptation->report.size_satisfied && !it.adaptation->accepted,
        "the requested options are kept although nothing was solved on them");
  check(h.options.refinement->refinement.max_steps == 0, "and in settings");
  const auto doc = to_json(h);
  check(doc.at("steps")
                .items[0]
                .at("adaptation")
                .at("run")
                .at("options")
                .at("refinement")
                .at("max_steps")
                .number() == 0,
        "JSON of the failed refinement");
}
} // namespace

int main() {
  return pde2d_test::run(
      {{"warm_started_solve_records_its_guess",
        warm_started_solve_records_its_guess},
       {"mesh_run_record_in_the_run_record", mesh_run_record_in_the_run_record},
       {"adaptive_history_keeps_its_settings",
        adaptive_history_keeps_its_settings},
       {"failed_refinement_keeps_its_options",
        failed_refinement_keeps_its_options}});
}
