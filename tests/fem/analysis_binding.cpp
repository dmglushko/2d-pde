// Analysis evidence bound to its inputs (AnalysisRecord, plan.hpp;
// make_analysis_record and refresh_analysis, analysis.hpp). Oracles: the
// record's fingerprints are those of problem_fingerprint and
// mesh_fingerprint of the inputs it was made from; solve, evolve and adapt
// refuse evidence for other inputs with the documented message; a refresh
// gives exactly the record made directly on the new inputs; and adapt()
// carries a mesh-bound analysis that matches every iteration's mesh.
#include "discretization/geometry.hpp"
#include "discretization/mesh_fingerprint.hpp"
#include "discretization/square_mesh.hpp"
#include "discretization/triangulate.hpp"
#include "fem/adaptivity.hpp"
#include "fem/analysis.hpp"
#include "fem/evolution.hpp"
#include "fem/fingerprint.hpp"
#include "fem/record_json.hpp"
#include "fem/report.hpp"
#include "fem/solver.hpp"
#include "support/check.hpp"
#include <sstream>
#include <string>

using namespace pde2d::fem;
namespace disc = pde2d::discretization;
using pde2d::discretization::square_mesh;
using pde2d_test::check;
using pde2d_test::expect_throws;

namespace {
Problem poisson(const Expr &source) {
  Problem p;
  p.equation.source = source;
  p.assign(dirichlet(0), {"bottom", "right", "top", "left"});
  return p;
}
bool same_properties(const AnalysisRecord &a, const AnalysisRecord &b) {
  std::ostringstream left, right;
  write_report(left, a.properties());
  write_report(right, b.properties());
  return left.str() == right.str();
}

void records_carry_their_inputs() {
  const auto mesh = square_mesh(4);
  const auto p = poisson(1 + x);
  AnalysisOptions options;
  options.sample_times = {0., .5};
  const auto record = make_analysis_record(p, mesh, options);
  check(record.problem_fingerprint() == problem_fingerprint(p) &&
            record.mesh_fingerprint() == disc::mesh_fingerprint(mesh) &&
            record.options().sample_times == options.sample_times,
        "Fingerprints and sampling configuration");
  const auto free = make_analysis_record(p);
  check(!free.mesh_fingerprint() &&
            free.problem_fingerprint() == problem_fingerprint(p),
        "Mesh-free record");
  auto plan = plan_from_user({}, {});
  plan.analysis = record;
  std::ostringstream json;
  write_json(json, solve_stationary(mesh, p, plan).record);
  check(json.str().find("\"problem_fingerprint\": \"" + problem_fingerprint(p) +
                        "\"") != std::string::npos &&
            json.str().find("\"mesh_fingerprint\": \"" +
                            disc::mesh_fingerprint(mesh) + "\"") !=
                std::string::npos,
        "JSON binds the analysis");
}

void mismatches_are_rejected() {
  const auto mesh = square_mesh(4);
  const auto other_mesh = square_mesh(5);
  const auto p = poisson(1 + x);
  const auto q = poisson(2 + x);
  auto plan = plan_from_user({}, {});
  plan.analysis = make_analysis_record(p, mesh);
  const auto reference = solve_stationary(mesh, p, plan_from_user({}, {}));
  check(solve_stationary(mesh, p, plan).values == reference.values,
        "Current evidence is accepted and changes nothing");
  expect_throws<std::invalid_argument>(
      [&] { solve_stationary(mesh, q, plan); },
      "Stale analysis: the plan's analysis was made for problem " +
          problem_fingerprint(p) + ", not for this problem");
  expect_throws<std::invalid_argument>(
      [&] { PreparedStationary(other_mesh, p, plan); },
      "Stale analysis: the plan's analysis sampled mesh " +
          disc::mesh_fingerprint(mesh) + ", not this mesh");
  // A mesh-free analysis holds on every mesh, but not for another problem.
  plan.analysis = make_analysis_record(p);
  solve_stationary(other_mesh, p, plan);
  expect_throws<std::invalid_argument>(
      [&] { solve_stationary(other_mesh, q, plan); }, "Stale analysis");

  // Evolutions check it too.
  Problem heat = p;
  heat.equation.m1 = 1;
  TimeOptions o;
  o.end = .02;
  auto transient = plan_from_user(o);
  transient.analysis = make_analysis_record(heat, mesh);
  (void)evolve(mesh, heat, {Expr(0)}, transient);
  expect_throws<std::invalid_argument>(
      [&] { (void)evolve(other_mesh, heat, {Expr(0)}, transient); },
      "Stale analysis: the plan's analysis sampled mesh");
  expect_throws<std::invalid_argument>(
      [&] { (void)evolve(mesh, p, {Expr(0)}, transient); },
      "Stale analysis: the plan's analysis was made for problem");
}

void explicit_refresh() {
  const auto mesh = square_mesh(4);
  const auto other_mesh = square_mesh(6);
  const auto p = poisson(1 + x * y);
  const auto q = poisson(t + x * y);
  AnalysisOptions options;
  options.quadrature_degree = 3;
  auto plan = plan_from_user({}, {});
  plan.analysis = make_analysis_record(p, mesh, options);
  check(!refresh_analysis(plan, p, mesh), "Current: nothing to do");
  check(refresh_analysis(plan, p, other_mesh), "New mesh: recomputed");
  const auto direct = make_analysis_record(p, other_mesh, options);
  check(plan.analysis->mesh_fingerprint() == direct.mesh_fingerprint() &&
            plan.analysis->options().quadrature_degree == 3 &&
            same_properties(*plan.analysis, direct),
        "Refresh equals a record made on the new mesh");
  solve_stationary(other_mesh, p, plan);
  // Another problem: the properties change with it.
  check(refresh_analysis(plan, q, other_mesh), "New problem: recomputed");
  check(plan.analysis->properties().time_dependent.value[weak::Source] &&
            same_properties(*plan.analysis,
                            make_analysis_record(q, other_mesh, options)),
        "Refresh follows the problem");
  // A mesh-free analysis stays mesh-free.
  plan.analysis = make_analysis_record(p, options);
  check(!refresh_analysis(plan, p, other_mesh) &&
            !plan.analysis->mesh_fingerprint(),
        "Mesh-free evidence needs no refresh for a mesh");
}

void adapt_refreshes_on_new_meshes() {
  disc::MeshOptions mesh_options;
  mesh_options.boundary_treatment = disc::BoundaryTreatment::RefineOnCurves;
  mesh_options.refinement.max_edge_length = .4;
  const auto initial =
      disc::generate_mesh(disc::CircleGeometry(1.0), mesh_options);
  check(initial.report.size_satisfied, initial.report.message);
  Problem p;
  p.equation.source = exp(-20 * (x * x + y * y));
  p.assign(dirichlet(0), {"boundary"});
  AdaptOptions options;
  options.max_iterations = 2;
  options.transfer.max_outside_distance =
      mesh_options.boundary.max_curve_deviation;
  const auto plain = adapt(initial, p, plan_from_user({}, {}), options);
  auto plan = plan_from_user({}, {});
  AnalysisOptions sampling;
  sampling.sample_times = {0., 1.};
  plan.analysis = make_analysis_record(p, initial.mesh, sampling);
  const auto history = adapt(initial, p, plan, options);
  check(history.solution.values == plain.solution.values,
        "Analysis evidence changes no numbers");
  for (const auto &it : history.iterations) {
    const auto &a = it.record.plan.analysis;
    check(a && a->mesh_fingerprint() == it.record.mesh.fingerprint &&
              a->options().sample_times == sampling.sample_times,
          "Each iteration's analysis matches its mesh");
  }
  check(history.iterations.size() == 3 &&
            history.iterations[1].record.plan.analysis->mesh_fingerprint() !=
                history.iterations[0].record.plan.analysis->mesh_fingerprint(),
        "Refreshed when the mesh changed");
  // Evidence for another mesh is refused on the first iteration.
  auto coarser = mesh_options;
  coarser.refinement.max_edge_length = .6;
  plan.analysis = make_analysis_record(
      p, disc::generate_mesh(disc::CircleGeometry(1.0), coarser).mesh,
      sampling);
  expect_throws<std::invalid_argument>(
      [&] { (void)adapt(initial, p, plan, options); },
      "Stale analysis: the plan's analysis sampled mesh");
}
} // namespace
int main() {
  return pde2d_test::run(
      {{"records_carry_their_inputs", records_carry_their_inputs},
       {"mismatches_are_rejected", mismatches_are_rejected},
       {"explicit_refresh", explicit_refresh},
       {"adapt_refreshes_on_new_meshes", adapt_refreshes_on_new_meshes}});
}
