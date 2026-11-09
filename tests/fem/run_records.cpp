// Complete, portable run records (record.hpp, record_json.hpp).
// Oracles: values known from the inputs (workload, options, mesh options and
// report, fingerprints of separately built inputs), the JSON round trip
// (parsed numbers are the run's doubles bit for bit, re-serialization is
// byte-identical, the caller's stream locale is ignored), two runs of the
// same inputs (identical records except timings) and one changed input
// (exactly the matching fingerprint changes).
#include "discretization/geometry.hpp"
#include "discretization/mesh_fingerprint.hpp"
#include "discretization/square_mesh.hpp"
#include "discretization/triangulate.hpp"
#include "fem/analysis.hpp"
#include "fem/evolution.hpp"
#include "fem/fingerprint.hpp"
#include "fem/record_json.hpp"
#include "fem/report.hpp"
#include "fem/solver.hpp"
#include "support/check.hpp"
#include <locale>
#include <sstream>
#include <string>

using pde2d::discretization::square_mesh;
using pde2d_test::check;
using pde2d_test::expect_throws;
namespace {
using namespace pde2d::fem;
namespace disc = pde2d::discretization;
const std::initializer_list<std::string> sides{"bottom", "right", "top",
                                               "left"};

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
bool only_timings(const std::vector<RecordDifference> &diff) {
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
            doc.at("schema_version").number() == run_record_schema_version,
        "Schema");
  const auto &time = doc.at("plan").at("time");
  check(time.at("step").number() == o.step && time.at("end").number() == o.end,
        "Doubles round-trip bit for bit");
  check(time.at("method").text == "Newmark", "Time method");
  check(doc.at("workload").at("nodes").number() == mesh.nodes.size(),
        "Workload");
  check(doc.at("mesh").at("fingerprint").text == disc::mesh_fingerprint(mesh),
        "Mesh fingerprint");
  check(doc.at("mesh").at("provenance").kind == JsonValue::Kind::Null,
        "No provenance for a bare mesh");
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
  check(doc.at("stationary_time").kind == JsonValue::Kind::Null,
        "No stationary time for an evolution");
  const auto &decisions = doc.at("plan").at("decisions");
  check(decisions.items.size() == e.record.plan.decisions.size() &&
            decisions.items[0].at("subject").text == "time integrator" &&
            decisions.items[0].at("choice").text == "Newmark",
        "Decisions");
  check(doc.at("versions").at("pde2d").text ==
                pde2d::library_version().version &&
            doc.at("versions").at("symengine").text == SYMENGINE_VERSION,
        "Versions");
  check(doc.at("replayable").boolean, "Built-in strategies replay");

  // A stationary record with mesh provenance, a mesh plan and an analysis.
  const auto result = disk();
  Problem p;
  p.equation.source = 1;
  p.assign(dirichlet(0), {"boundary"});
  auto plan = plan_from_user({}, {});
  plan.mesh = mesh_plan_from_user(result.options);
  AnalysisOptions analysis_options;
  analysis_options.sample_times = {0., .5};
  plan.analysis = make_analysis_record(p, result.mesh, analysis_options);
  const auto s = PreparedStationary(result, p, plan).solve(.75);
  const auto stationary = json(s.record);
  std::istringstream sin(stationary);
  const auto sdoc = read_run_record_json(sin);
  check(json_text(sdoc) == stationary, "Stationary round trip");
  check(sdoc.at("stationary_time").number() == .75, "Stationary time");
  const auto &provenance = sdoc.at("mesh").at("provenance");
  check(provenance.at("options").at("boundary_treatment").text ==
                "RefineOnCurves" &&
            provenance.at("options")
                    .at("refinement")
                    .at("max_edge_length")
                    .number() == .4 &&
            provenance.at("report").at("size_satisfied").boolean ==
                result.report.size_satisfied &&
            provenance.at("report").at("refinement_steps").number() ==
                result.report.refinement_steps &&
            provenance.at("boundary_context").boolean,
        "Mesh provenance");
  check(sdoc.at("plan").at("mesh").at("decisions").items.size() == 4 &&
            sdoc.at("plan")
                    .at("analysis")
                    .at("options")
                    .at("sample_times")
                    .items.size() == 2 &&
            sdoc.at("plan")
                .at("analysis")
                .at("properties")
                .at("has_dirichlet")
                .at("value")
                .boolean,
        "Mesh plan and analysis");

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
  check(parse_json("\"a\\u00e9\\ud83d\\ude00\"").text ==
            "a\xc3\xa9\xf0\x9f\x98\x80",
        "Unicode escapes decode to UTF-8");
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

  TimeOptions o;
  o.end = .05;
  const auto e1 = evolve(mesh, heat(), {x * x - y}, o);
  const auto e2 = evolve(square_mesh(4), heat(), {x * x - y}, o);
  check(diff_records(e1.record, e2.record).empty(), "Evolutions too");
  check(only_timings(
            diff_records(e1.record, e2.record, {.ignore_timings = false})),
        "Only timings may differ");
  check(diff_records(to_json(e1.record), to_json(e2.record)).empty(),
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

void mesh_provenance_and_strategies() {
  const auto result = disk();
  Problem p;
  p.equation.source = 1;
  p.assign(dirichlet(0), {"boundary"});
  const auto s = solve_stationary(result, p);
  const auto &r = s.record;
  check(r.mesh.fingerprint == disc::mesh_fingerprint(result.mesh) &&
            r.mesh.provenance &&
            r.mesh.provenance->options.boundary_treatment ==
                disc::BoundaryTreatment::RefineOnCurves &&
            r.mesh.provenance->report.message == result.report.message,
        "Provenance from a MeshResult");
  // Bit-identical to the bare-mesh solve.
  check(s.values == solve_stationary(result.mesh, p).values,
        "Provenance does not change the solve");
  check(r.strategies.size() == 1 &&
            r.strategies[0].subject == DecisionSubject::LinearSolver &&
            r.strategies[0].replay == StrategyReplay::BuiltIn && r.replayable(),
        "Built-in solver");

  // A size field without a descriptor: recorded, not replayable.
  auto options = result.options;
  options.refinement.size_field = [](disc::Point2 q) {
    return .2 + .3 * std::abs(q.x);
  };
  auto graded = disc::generate_mesh(disc::CircleGeometry(1.0), options);
  const auto g = solve_stationary(graded, p);
  check(g.record.strategies.size() == 2 &&
            g.record.strategies[1].subject == DecisionSubject::SizeField &&
            g.record.strategies[1].replay == StrategyReplay::NotReplayable &&
            !g.record.replayable(),
        "Undescribed size field");
  check(json(g.record).find("\"replay\": \"not replayable\"") !=
            std::string::npos,
        "JSON says not replayable");
  options.refinement.size_field_descriptor =
      pde2d::StrategyDescriptor{"Linear", {{"base", ".2"}, {"slope", ".3"}}};
  graded = disc::generate_mesh(disc::CircleGeometry(1.0), options);
  const auto described = solve_stationary(graded, p);
  check(described.record.strategies[1].replay == StrategyReplay::Described &&
            described.record.strategies[1].name == "Linear(base=.2, slope=.3)",
        "Described size field");

  // User backend, integrator and controller.
  const auto mesh = square_mesh(3);
  TimeOptions o;
  o.end = .03;
  o.solver.backend = make_linear_solver_backend(
      "MyLU",
      [](const SolverOptions &) -> std::unique_ptr<LinearSolverBackend> {
        return std::make_unique<UserLU>();
      });
  o.integrator = [] { return std::make_unique<CrankNicolsonIntegrator>(); };
  o.step_controller = [&] {
    return std::make_unique<UniformStepController>(o.start, o.end, o.step);
  };
  const auto e = evolve(mesh, heat(), {x * x - y}, o);
  const auto &st = e.record.strategies;
  check(st.size() == 3 && st[0].subject == DecisionSubject::TimeIntegrator &&
            st[0].replay == StrategyReplay::Described &&
            st[0].descriptor->name == "CrankNicolson" &&
            st[1].subject == DecisionSubject::TimeSteps &&
            st[1].replay == StrategyReplay::Described &&
            st[1].descriptor->parameters.size() == 3 &&
            st[2].subject == DecisionSubject::LinearSolver &&
            st[2].name == "MyLU" &&
            st[2].replay == StrategyReplay::NotReplayable &&
            !e.record.replayable(),
        "Factory strategies");
  std::ostringstream report;
  write_report(report, e.record);
  check(report.str().find("record strategy: linear solver=MyLU "
                          "replay=not replayable") != std::string::npos,
        "Report names the replay status");
}

void one_source_for_time_settings() {
  const auto mesh = square_mesh(3);
  TimeOptions o;
  o.method = TimeMethod::CrankNicolson;
  o.end = .04;
  o.solver.method = LinearSolver::GMRES;
  // TimeOptions is a TimePlan plus output: one source for time settings.
  const TimePlan &time = o;
  check(&time.step == &o.step, "Time settings are the TimePlan's");
  const auto plan = plan_from_user(o);
  check(plan.time->method == o.method && plan.time->step == o.step,
        "Plan from the same fields");
  const auto e = evolve(mesh, heat(), {x * x - y}, o);
  const auto &r = e.record;
  check(r.workload.nodes == mesh.nodes.size() &&
            r.plan.time->method == TimeMethod::CrankNicolson &&
            find_strategy(r, DecisionSubject::TimeIntegrator)->name ==
                "CrankNicolson" &&
            r.plan.time->step == o.step && r.plan.solver == o.solver &&
            r.plan.decisions.size() == 5 &&
            r.plan.decisions[0].choice == "CrankNicolson",
        "The record holds the time settings that ran");
}
} // namespace
int main() {
  return pde2d_test::run(
      {{"json_round_trip", json_round_trip},
       {"identical_inputs_identical_records",
        identical_inputs_identical_records},
       {"one_input_changes_one_fingerprint", one_input_changes_one_fingerprint},
       {"mesh_provenance_and_strategies", mesh_provenance_and_strategies},
       {"one_source_for_time_settings", one_source_for_time_settings}});
}
