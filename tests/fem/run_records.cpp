// Complete, portable run records (record.hpp, record_json.hpp).
// Oracles: values known from the inputs (workload, options, fingerprints of
// separately built inputs, the mesh run record of the meshing call), the
// JSON round trip (parsed numbers are the run's doubles bit for bit,
// re-serialization is byte-identical, the caller's stream locale is
// ignored), two runs of the same inputs (identical records except timings),
// one changed input (exactly the matching fingerprint changes) and replay
// (the record's options rerun the solve bit for bit).
#include "discretization/geometry.hpp"
#include "discretization/mesh_fingerprint.hpp"
#include "discretization/square_mesh.hpp"
#include "discretization/triangulate.hpp"
#include "fem/evolution.hpp"
#include "fem/fingerprint.hpp"
#include "fem/record_json.hpp"
#include "fem/report.hpp"
#include "fem/solver.hpp"
#include "fem/time_integration.hpp"
#include "support/check.hpp"
#include <Eigen/SparseLU>
#include <locale>
#include <memory>
#include <sstream>
#include <string>

using pde2d::discretization::square_mesh;
using pde2d_test::check;
using pde2d_test::expect_throws;
namespace {
using namespace pde2d::fem;
namespace disc = pde2d::discretization;

Problem poisson(Expr source = 1 + x * y) {
  Problem p;
  p.equation.source = std::move(source);
  p.assign(dirichlet(x * x - y), {"bottom", "top"});
  p.assign(neumann(0), {"right", "left"});
  return p;
}
Problem heat() {
  Problem p = poisson();
  p.equation.m1 = 1;
  return p;
}
std::string json(const RunRecord &record) {
  std::ostringstream out;
  write_json(out, record);
  return out.str();
}
bool only_timings(const std::vector<pde2d::RecordDifference> &diff) {
  for (const auto &d : diff)
    if (!d.path.starts_with("timings."))
      return false;
  return true;
}
// A decimal comma, to show that the caller's locale does not leak in.
struct Comma : std::numpunct<char> {
  char do_decimal_point() const override { return ','; }
  std::string do_grouping() const override { return "\3"; }
};

// A user backend: Eigen's SparseLU behind the public interface.
class UserLU final : public LinearSolverBackend {
  Eigen::SparseLU<SparseMatrix> lu_;

public:
  std::string prepare(const SparseMatrix &matrix,
                      const SolverOptions &) override {
    lu_.compute(matrix);
    return lu_.info() == Eigen::Success ? "" : "factorization failed";
  }
  BackendSolution solve(const Vector &rhs, double) override {
    return {lu_.solve(rhs), 1};
  }
};

disc::MeshResult disk() {
  disc::MeshOptions options;
  options.boundary_treatment = disc::BoundaryTreatment::RefineOnCurves;
  options.boundary.max_curve_deviation = .05;
  options.refinement.max_edge_length = .4;
  auto result = disc::generate_mesh(disc::CircleGeometry(1.0), options);
  check(result.report.size_satisfied, result.report.message);
  return result;
}

void json_round_trip() {
  const auto mesh = square_mesh(3);
  TimeOptions o;
  o.method = TimeMethod::Newmark;
  o.end = .1;
  o.step = .1 / 3; // no short decimal: needs all 17 digits
  o.frame_stride = 2;
  Problem wave = poisson();
  wave.equation.m2 = 1;
  // Nodal, and zero on the Dirichlet sides like the data's time derivative.
  Vector velocity(mesh.nodes.size());
  for (std::size_t k = 0; k < mesh.nodes.size(); ++k)
    velocity[k] = .25 * mesh.nodes[k].y * (1 - mesh.nodes[k].y);
  const auto e = evolve(mesh, wave, {x * x - y, velocity}, o);
  const auto text = json(e.record);

  std::ostringstream comma;
  comma.imbue(std::locale(std::locale::classic(), new Comma));
  comma.precision(2);
  write_json(comma, e.record);
  check(comma.str() == text, "Stream locale and precision must not matter");

  std::istringstream in(text);
  const auto doc = read_run_record_json(in);
  check(json_text(doc) == text, "Re-serialization is byte-identical");
  check(doc.at("schema").text == "pde2d.run_record" &&
            doc.at("schema_version").number() == run_record_schema_version &&
            doc.at("kind").text == "evolution",
        "Schema");
  const auto &time = doc.at("options").at("time");
  check(time.at("step").number() == o.step && time.at("end").number() == o.end,
        "Doubles round-trip bit for bit");
  check(time.at("method").text == "Newmark" &&
            time.at("integrator").at("name").text == "Newmark" &&
            !time.at("integrator").at("user_supplied").boolean &&
            time.at("frame_stride").number() == 2,
        "Time settings");
  check(
      doc.at("options").at("solver").at("name").text == "SparseLU" &&
          doc.at("options").at("solver").at("relative_tolerance").number() ==
              o.solver.relative_tolerance &&
          doc.at("options").at("assembly").at("execution").at("backend").text ==
              "Serial",
      "Assembly and solver options");
  check(doc.at("workload").at("nodes").number() == mesh.nodes.size(),
        "Workload");
  check(doc.at("work").at("time_steps").number() == e.timings.time_steps &&
            doc.at("work").at("linear_solves").number() ==
                e.timings.linear_solves,
        "Work counters");
  check(doc.at("mesh").at("fingerprint").text == disc::mesh_fingerprint(mesh),
        "Mesh fingerprint");
  check(doc.at("mesh").at("run").kind == pde2d::JsonValue::Kind::Null,
        "No mesh run record unless attached");
  check(doc.at("problem").at("fingerprint").text == problem_fingerprint(wave),
        "Problem fingerprint");
  const auto &initial = doc.at("initial");
  check(initial.at("value").at("kind").text == "expression" &&
            initial.at("value").at("fingerprint").text ==
                field_fingerprint(Field{x * x - y}) &&
            initial.at("velocity").at("kind").text == "nodal" &&
            initial.at("velocity").at("fingerprint").text ==
                field_fingerprint(Field{velocity}),
        "Initial fields");
  check(doc.at("stationary").kind == pde2d::JsonValue::Kind::Null,
        "No stationary part for an evolution");
  check(doc.at("versions").at("pde2d").text ==
                pde2d::library_version().version &&
            doc.at("versions").at("symengine").text == SYMENGINE_VERSION &&
            doc.at("execution").at("openmp_available").boolean ==
                pde2d::openmp_available(),
        "Versions and execution");
  check(doc.at("replayable").boolean, "Built-in strategies replay");

  // A stationary record with a guess and an attached mesh run record.
  const auto result = disk();
  Problem p;
  p.equation.source = 1;
  p.assign(dirichlet(0), {"boundary"});
  SolverOptions cg;
  cg.method = LinearSolver::ConjugateGradient;
  const Vector guess = Vector::Zero(result.mesh.nodes.size());
  auto s = PreparedStationary(result.mesh, p, {}, cg).solve(.75, guess);
  attach_mesh_record(s.record, result);
  const auto stationary = json(s.record);
  std::istringstream sin(stationary);
  const auto sdoc = read_run_record_json(sin);
  check(json_text(sdoc) == stationary, "Stationary round trip");
  const auto &solve = sdoc.at("stationary");
  check(sdoc.at("kind").text == "stationary" &&
            solve.at("time").number() == .75 &&
            solve.at("initial_guess").text == "Used" &&
            solve.at("guess").at("fingerprint").text ==
                field_fingerprint(Field{guess}) &&
            solve.at("outcome").at("iterations").number() ==
                s.report.iterations &&
            solve.at("outcome").at("residual_norm").number() ==
                s.report.residual_norm,
        "Stationary time, guess and outcome");
  check(sdoc.at("work").at("solver_iterations").number() == s.report.iterations,
        "Iterations as work");
  const auto &run = sdoc.at("mesh").at("run");
  check(run.at("schema").text == disc::mesh_run_record_schema &&
            run.at("operation").text == "generate" &&
            run.at("options").at("boundary_treatment").text ==
                "RefineOnCurves" &&
            run.at("options").at("refinement").at("max_edge_length").number() ==
                .4 &&
            run.at("work").at("refinement_steps").number() ==
                result.report.refinement_steps,
        "Attached mesh run record");
  auto edited = result;
  edited.mesh.nodes[0].x += 1e-3;
  edited.record.fingerprint = disc::mesh_fingerprint(edited.mesh);
  expect_throws<std::invalid_argument>(
      [&] { attach_mesh_record(s.record, edited); },
      "Mesh run record describes another mesh");

  // The reader checks the schema.
  std::istringstream other("{\"schema\": \"something\"}");
  expect_throws<std::invalid_argument>([&] { read_run_record_json(other); },
                                       "Not a run record");
  auto newer = text;
  newer.replace(newer.find("\"schema_version\": 1"), 19,
                "\"schema_version\": 2");
  std::istringstream future(newer);
  expect_throws<std::invalid_argument>([&] { read_run_record_json(future); },
                                       "Unsupported run record schema version");
  std::istringstream broken(text.substr(0, text.size() / 2));
  expect_throws<std::invalid_argument>([&] { read_run_record_json(broken); },
                                       "Invalid JSON");
}

void identical_inputs_identical_records() {
  const auto mesh = square_mesh(4);
  const auto a = solve_stationary(mesh, poisson());
  const auto b = solve_stationary(square_mesh(4), poisson());
  check(diff_records(a.record, b.record).empty(),
        "Same inputs: identical records except timings");
  check(
      only_timings(diff_records(a.record, b.record, {.ignore_timings = false})),
      "Only timings may differ");
  check(diff_records(a.record, b.record, {.ignore_outcomes = false}).empty(),
        "Outcomes are reproducible");

  TimeOptions o;
  o.end = .05;
  const auto e1 = evolve(mesh, heat(), {x * x - y}, o);
  const auto e2 = evolve(square_mesh(4), heat(), {x * x - y}, o);
  check(diff_records(e1.record, e2.record).empty(), "Evolutions too");
  check(only_timings(
            diff_records(e1.record, e2.record, {.ignore_timings = false})),
        "Only timings may differ");
  check(pde2d::diff_records(to_json(e1.record), to_json(e2.record)).empty(),
        "Documents compare like records");
}

void one_input_changes_one_fingerprint() {
  const auto mesh = square_mesh(4);
  const auto base = solve_stationary(mesh, poisson());
  // One coefficient.
  const auto changed = solve_stationary(mesh, poisson(1 + 2 * x * y));
  check(base.record.problem_fingerprint != changed.record.problem_fingerprint,
        "A coefficient changes the problem fingerprint");
  const auto diff = diff_records(base.record, changed.record);
  check(diff.size() == 1 && diff[0].path == "problem.fingerprint",
        "Nothing else differs");
  // Built separately, the same problem has the same fingerprint.
  check(problem_fingerprint(poisson()) == base.record.problem_fingerprint,
        "Fingerprints identify content");
  // Boundary conditions and assignment order count.
  auto robin = poisson();
  robin.boundaries[1].condition.alpha = 1;
  check(problem_fingerprint(robin) != base.record.problem_fingerprint,
        "A condition changes the fingerprint");
  Problem swapped;
  swapped.equation.source = 1 + x * y;
  swapped.assign(neumann(0), {"right", "left"});
  swapped.assign(dirichlet(x * x - y), {"bottom", "top"});
  check(problem_fingerprint(swapped) != base.record.problem_fingerprint,
        "Assignment order counts");
  // Another mesh changes only the mesh fields.
  const auto finer = solve_stationary(square_mesh(5), poisson());
  for (const auto &d : diff_records(base.record, finer.record))
    check(d.path.starts_with("mesh.") || d.path.starts_with("workload."),
          "Only mesh and workload differ");
  // Initial fields.
  TimeOptions o;
  o.end = .02;
  const auto e1 = evolve(mesh, heat(), {x * x - y}, o);
  const auto e2 = evolve(mesh, heat(), {x * x - y + 1e-3 * y * (1 - y)}, o);
  const auto ediff = diff_records(e1.record, e2.record);
  check(ediff.size() == 1 && ediff[0].path == "initial.value.fingerprint",
        "An initial value changes its fingerprint only");
  check(field_fingerprint(Field{Vector::Zero(3)}) !=
            field_fingerprint(Field{Expr(0)}),
        "Nodal and symbolic fields differ");
}

void records_replay_and_count_work() {
  const auto mesh = square_mesh(6);
  SolverOptions gmres;
  gmres.method = LinearSolver::GMRES;
  gmres.relative_tolerance = 1e-11;
  AssemblyOptions assembly;
  assembly.quadrature_degree = 4;
  PreparedStationary prepared(mesh, poisson(), assembly, gmres);
  const auto first = prepared.solve(0);
  const auto second = prepared.solve(0);
  // Performance contract: the second solve reuses the preparation.
  check(first.record.work.solver_preparations == 1 &&
            first.record.work.solver_reuses == 0 &&
            second.record.work.solver_preparations == 0 &&
            second.record.work.solver_reuses == 1,
        "Per-solve preparation and reuse");
  check(first.record.work.solver_iterations ==
            static_cast<std::size_t>(first.report.iterations),
        "Iterations of the solve");
  // The record's options rerun the solve bit for bit.
  const auto replay = solve_stationary(mesh, poisson(), first.record.assembly,
                                       first.record.solver);
  check(replay.values == first.values &&
            first.record.assembly.quadrature_degree == 4 &&
            first.record.solver == gmres,
        "Replay from the record");

  TimeOptions o;
  o.method = TimeMethod::CrankNicolson;
  o.end = .04;
  o.solver = gmres;
  const auto e = evolve(mesh, heat(), {x * x - y}, o);
  std::size_t iterations = 0;
  for (const auto &r : e.reports)
    iterations += static_cast<std::size_t>(r.iterations);
  // Every solve is a time step here (no start-up solve), so the stored
  // reports cover every iteration.
  check(e.record.work.linear_solves == e.timings.time_steps &&
            e.record.work.solver_iterations == iterations &&
            e.record.work.assembly_calls == e.timings.assembly_calls &&
            e.record.work.solver_reuses == e.timings.solver_reuses,
        "Evolution work");
  auto replayed = o;
  replayed.method = e.record.time->method;
  replayed.step = e.record.time->step;
  replayed.solver = e.record.solver;
  replayed.assembly = e.record.assembly;
  const auto again = evolve(mesh, heat(), {x * x - y}, replayed);
  check(again.frames.back().values == e.frames.back().values,
        "Evolution replay from the record");
}

void user_strategies_are_named() {
  const auto mesh = square_mesh(3);
  TimeOptions o;
  o.end = .03;
  o.solver.backend = make_linear_solver_backend(
      "MyLU",
      [](const SolverOptions &) -> std::unique_ptr<LinearSolverBackend> {
        return std::make_unique<UserLU>();
      });
  o.integrator = [] { return make_time_integrator(TimeMethod::CrankNicolson); };
  const auto e = evolve(mesh, heat(), {x * x - y}, o);
  const auto &r = e.record;
  check(r.linear_solver.name == "MyLU" && r.linear_solver.user_supplied &&
            r.time->integrator.name == "CrankNicolson" &&
            r.time->integrator.user_supplied && !r.replayable(),
        "User strategies: named, not replayable");
  const auto doc = to_json(r);
  const auto &solver = doc.at("options").at("solver");
  check(solver.at("name").text == "MyLU" &&
            solver.at("user_supplied").boolean &&
            doc.at("options")
                .at("time")
                .at("integrator")
                .at("user_supplied")
                .boolean &&
            !doc.at("replayable").boolean,
        "JSON names them");
  std::ostringstream report;
  write_report(report, r);
  check(report.str().find("linear_solver=MyLU user_supplied=true "
                          "time_integrator=CrankNicolson "
                          "user_supplied=true") != std::string::npos &&
            report.str().find("replayable=false") != std::string::npos,
        "Report names them");
  const auto s = solve_stationary(mesh, poisson(), {}, o.solver);
  check(s.record.linear_solver.user_supplied && !s.record.replayable(),
        "Stationary user backend");
}
} // namespace
int main() {
  return pde2d_test::run(
      {{"json_round_trip", json_round_trip},
       {"identical_inputs_identical_records",
        identical_inputs_identical_records},
       {"one_input_changes_one_fingerprint", one_input_changes_one_fingerprint},
       {"records_replay_and_count_work", records_replay_and_count_work},
       {"user_strategies_are_named", user_strategies_are_named}});
}
