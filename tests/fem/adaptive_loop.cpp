// The solve -> estimate -> mark -> refine -> transfer/re-solve loop on a
// stationary problem with a curved boundary. The loop is composed twice:
// by hand from the public components (the oracle) and by the adapt()
// driver (adaptivity.hpp), which must reproduce it bit for bit: the same
// meshes, values, estimates, marked counts, refinement reports, transfer
// statistics and run records. The transferred solution seeds the next
// (conjugate gradient) solve as its initial guess. Oracles of the loop
// itself: the exact H1-seminorm error of a manufactured solution must
// decrease in every iteration, and the seeded solve must agree with an
// unseeded solve within the solver tolerance. No performance claim is made
// beyond reporting the iteration counts.
#include "discretization/geometry.hpp"
#include "discretization/mesh_fingerprint.hpp"
#include "discretization/triangulate.hpp"
#include "fem/adaptivity.hpp"
#include "fem/adaptivity_json.hpp"
#include "fem/error_norms.hpp"
#include "fem/estimation.hpp"
#include "fem/marking.hpp"
#include "fem/record_json.hpp"
#include "fem/solver.hpp"
#include "fem/transfer.hpp"
#include "support/check.hpp"
#include "support/check_eigen.hpp"
#include <cmath>
#include <iostream>
#include <memory>
#include <sstream>
using namespace pde2d::fem;
using namespace pde2d::discretization;
using pde2d_test::check;
using pde2d_test::expect_throws;

namespace {
// A sharp peak inside the unit disk: -Laplace(u) = f, u = exact on the
// circle.
const Expr exact = exp(-60 * ((x - .4) * (x - .4) + (y + .3) * (y + .3)));
Problem peak_problem() {
  Problem p;
  p.equation.source = -(diff(diff(exact, x), x) + diff(diff(exact, y), y));
  p.assign(dirichlet(exact), {"boundary"});
  return p;
}
MeshOptions disk_options() {
  MeshOptions options;
  options.boundary_treatment = BoundaryTreatment::RefineOnCurves;
  options.boundary.max_curve_deviation = .02;
  options.refinement.max_edge_length = .3;
  return options;
}
SolvePlan cg_plan() {
  SolverOptions cg;
  cg.method = LinearSolver::ConjugateGradient;
  return plan_from_user(AssemblyOptions{8}, cg);
}
bool identical(const Vector &a, const Vector &b) {
  return a.size() == b.size() && (a.array() == b.array()).all();
}
bool same_report(const MeshReport &a, const MeshReport &b) {
  if (a.size_satisfied != b.size_satisfied ||
      a.delaunay_verified != b.delaunay_verified ||
      a.refinement_steps != b.refinement_steps || a.message != b.message ||
      a.marked.has_value() != b.marked.has_value())
    return false;
  if (!a.marked)
    return true;
  const auto &m = *a.marked, &n = *b.marked;
  return m.marked_elements == n.marked_elements &&
         m.marked_edges == n.marked_edges &&
         m.bisected_edges == n.bisected_edges &&
         m.absorbed_edges == n.absorbed_edges &&
         m.unrefinable_elements == n.unrefinable_elements;
}

// What the hand-composed loop observed in one iteration.
struct Observed {
  std::string fingerprint;
  RunRecord record;
  double estimate = 0;
  std::size_t marked = 0;
  std::optional<MeshReport> refinement;
  std::optional<FieldTransfer> transfer;
};

void adaptive_loop_reduces_error() {
  const auto p = peak_problem();
  const auto options = disk_options();
  auto mesh = generate_mesh(CircleGeometry(1.0), options);
  check(mesh.report.size_satisfied, mesh.report.message);
  const auto initial = mesh;
  const auto plan = cg_plan();

  // The oracle: the loop composed by hand from the public components.
  std::vector<Observed> observed;
  auto first = solve_stationary(mesh, p, plan);
  auto u = first.values;
  observed.push_back({mesh_fingerprint(mesh.mesh), first.record});
  double error = error_norms(mesh.mesh, u, exact).h1_seminorm;
  const double first_error = error;
  for (int iteration = 0; iteration < 8; ++iteration) {
    auto estimate = estimate_error(mesh.mesh, p, u);
    auto marked = mark_bulk(estimate.indicators, .6);
    auto refined = refine_mesh(mesh, marked, mesh.options);
    check(refined.report.size_satisfied, refined.report.message);
    check(refined.report.marked->bisected_edges > 0);
    observed.back().estimate = estimate.global;
    observed.back().marked = marked.size();
    observed.back().refinement = refined.report;
    // The previous solution moved to the new mesh; boundary nodes on the
    // circle lie outside the old chords by at most the curve deviation.
    TransferOptions transfer;
    transfer.max_outside_distance = options.boundary.max_curve_deviation;
    auto guess = transfer_field(mesh.mesh, u, refined.mesh, transfer);
    check(guess.values.allFinite());
    // One preparation serves the seeded and the unseeded solve.
    PreparedStationary stationary(refined, p, plan);
    const auto seeded = stationary.solve(0, guess.values);
    const auto cold = stationary.solve(0);
    check(seeded.report.initial_guess == InitialGuess::Used,
          "the transferred field must seed the solve");
    check(stationary.statistics().solver.preparations == 1 &&
              stationary.statistics().solver_reuses == 1,
          "the second solve on one mesh must reuse the preparation");
    pde2d_test::near_vector(seeded.values, cold.values, 1e-7);
    observed.push_back(
        {mesh_fingerprint(refined.mesh), seeded.record, 0, 0, {}, guess});
    auto next = seeded.values;
    const double next_error =
        error_norms(refined.mesh, next, exact).h1_seminorm;
    std::cout << "iteration " << iteration << ": nodes "
              << mesh.mesh.nodes.size() << " -> " << refined.mesh.nodes.size()
              << ", marked " << marked.size() << ", estimate "
              << estimate.global << ", H1 error " << error << " -> "
              << next_error << ", change from transferred "
              << (next - guess.values).lpNorm<Eigen::Infinity>()
              << ", CG iterations seeded " << seeded.report.iterations
              << " / unseeded " << cold.report.iterations << '\n';
    check(next_error < error, "adaptive iteration did not reduce the error");
    mesh = std::move(refined);
    u = std::move(next);
    error = next_error;
  }
  observed.back().estimate = estimate_error(mesh.mesh, p, u).global;
  check(error < .5 * first_error,
        "eight adaptive iterations gained too little");
  for (auto [a, b] : mesh.mesh.boundary_edges) {
    (void)b;
    auto q = mesh.mesh.nodes[a];
    check(std::abs(std::hypot(q.x, q.y) - 1) < 1e-12);
  }

  // The driver: the same loop in one call.
  AdaptOptions adapt_options;
  adapt_options.marker = std::make_shared<BulkMarking>(.6);
  adapt_options.max_iterations = 8;
  adapt_options.transfer.max_outside_distance =
      options.boundary.max_curve_deviation;
  const auto history = adapt(initial, p, plan, adapt_options);
  check(history.stop == AdaptiveStop::MaxIterations, history.stop_message);
  check(history.iterations.size() == observed.size(),
        "one entry per solve (eight refinements after the first)");
  check(mesh_fingerprint(history.mesh.mesh) == mesh_fingerprint(mesh.mesh),
        "the driver reaches the hand-composed mesh");
  check(identical(history.solution.values, u),
        "the driver's final values equal the hand-composed ones bit for bit");
  for (std::size_t k = 0; k < observed.size(); ++k) {
    const auto &it = history.iterations[k];
    const auto &o = observed[k];
    check(it.index == k);
    check(it.record.mesh.fingerprint == o.fingerprint, "mesh fingerprint");
    check(it.record.mesh.provenance.has_value(), "mesh provenance");
    check(diff_records(it.record, o.record).empty(),
          "the iteration's run record equals the hand-composed one");
    check(it.estimate.global == o.estimate, "estimate bit for bit");
    check(it.estimate.estimator == "gradient recovery (ZZ)");
    const bool last = k + 1 == observed.size();
    check(it.marking.has_value() == !last && it.refinement.has_value() == !last,
          "the last iteration stops before marking");
    check(it.transfer.has_value() == (k > 0),
          "transfer into every refined mesh");
    if (k > 0) {
      check(it.transfer->extrapolated_nodes == o.transfer->extrapolated_nodes &&
                it.transfer->max_outside_distance ==
                    o.transfer->max_outside_distance,
            "transfer statistics");
      check(it.initial_guess == InitialGuess::Used, "warm start");
    }
    if (!last) {
      check(it.marking->marked == o.marked, "marked count");
      check(it.marking->strategy == "bulk (Dorfler)");
      check(same_report(*it.refinement, *o.refinement), "refinement report");
      check(it.decisions.size() == 2 &&
                it.decisions[0].subject == DecisionSubject::Marking &&
                it.decisions[0].choice == "Bulk(fraction=0.6)" &&
                it.decisions[1].subject == DecisionSubject::Refinement &&
                it.decisions[1].choice ==
                    "marked longest-edge bisection (ConstrainedDelaunay, "
                    "RefineOnCurves)",
            "marking and refinement decisions");
    }
  }
}

// The history serializes with the run-record conventions and round-trips.
void history_json_round_trip() {
  const auto p = peak_problem();
  auto mesh = generate_mesh(CircleGeometry(1.0), disk_options());
  AdaptOptions o;
  o.max_iterations = 2;
  o.transfer.max_outside_distance = .02;
  const auto history = adapt(mesh, p, cg_plan(), o);
  check(history.iterations.size() == 3);
  check(history.iterations[0].decisions[0].basis ==
            "default (AdaptOptions::marker unset)",
        "an unset marker is recorded as the default");
  std::stringstream out;
  write_json(out, history);
  const auto document = read_adaptive_history_json(out);
  check(diff_records(document, to_json(history)).empty(), "JSON round trip");
  check(document.at("schema").text == "pde2d.adaptive_history", "schema");
  check(document.at("stop").at("reason").text == "max iterations");
  const auto &iterations = document.at("iterations");
  check(iterations.items.size() == 3);
  const auto &second = iterations.items[1];
  check(second.at("record").at("schema").text == "pde2d.run_record",
        "each iteration embeds its run record");
  check(second.at("marking")
                .at("descriptor")
                .at("parameters")
                .at("fraction")
                .text == "0.5",
        "default Dorfler fraction");
  check(second.at("refinement").at("marked").at("bisected_edges").kind ==
            pde2d::fem::JsonValue::Kind::Number,
        "marked-refinement counters");
  check(second.at("transfer").at("target_nodes").text ==
            std::to_string(history.iterations[1].record.workload.nodes),
        "transfer into the second mesh");
  check(iterations.items[2].at("marking").kind ==
            pde2d::fem::JsonValue::Kind::Null,
        "no marking after the last solve");
  check(document.at("final").at("mesh_fingerprint").text ==
            mesh_fingerprint(history.mesh.mesh),
        "final mesh");
  // Two identical runs differ only in timings, also nested in the records.
  const auto again = adapt(mesh, p, cg_plan(), o);
  check(diff_records(to_json(history), to_json(again)).empty(),
        "repeat runs give the same history");
  std::stringstream bad("{\"schema\":\"pde2d.run_record\"}");
  expect_throws<std::invalid_argument>(
      [&] { (void)read_adaptive_history_json(bad); },
      "Not an adaptive history");
  std::stringstream future(
      "{\"schema\":\"pde2d.adaptive_history\",\"schema_version\":99}");
  expect_throws<std::invalid_argument>(
      [&] { (void)read_adaptive_history_json(future); },
      "Unsupported adaptive history schema version: 99");
}

class NoMarking final : public MarkingStrategy {
public:
  std::string name() const override { return "none"; }
  std::vector<std::size_t> mark(const ErrorEstimate &,
                                const TriangleMesh &) const override {
    return {};
  }
};
class Unsorted final : public MarkingStrategy {
public:
  std::string name() const override { return "unsorted"; }
  std::vector<std::size_t> mark(const ErrorEstimate &,
                                const TriangleMesh &) const override {
    return {1, 0};
  }
};

// Each stopping rule ends the loop with its reason.
void stopping_rules() {
  const auto p = peak_problem();
  auto mesh = generate_mesh(CircleGeometry(1.0), disk_options());
  const auto plan = cg_plan();
  AdaptOptions o;
  o.transfer.max_outside_distance = .02;

  o.tolerance = 1e9;
  auto h = adapt(mesh, p, plan, o);
  check(h.stop == AdaptiveStop::ToleranceReached && h.iterations.size() == 1 &&
            !h.iterations[0].marking,
        "a loose tolerance stops after the first solve");
  check(mesh_fingerprint(h.mesh.mesh) == mesh_fingerprint(mesh.mesh));

  o.tolerance = 0;
  o.max_iterations = 0;
  h = adapt(mesh, p, plan, o);
  check(h.stop == AdaptiveStop::MaxIterations && h.iterations.size() == 1);
  // The driver's single solve and estimate are the components' results.
  const auto direct = solve_stationary(mesh, p, plan);
  check(identical(h.solution.values, direct.values));
  check(h.estimate.global ==
        estimate_error(mesh.mesh, p, direct.values).global);

  o.max_iterations = 3;
  o.marker = std::make_shared<NoMarking>();
  h = adapt(mesh, p, plan, o);
  check(h.stop == AdaptiveStop::NothingMarked && h.iterations.size() == 1 &&
            h.iterations[0].marking->marked == 0 &&
            h.iterations[0].decisions.size() == 1,
        "nothing marked");
  check(h.iterations[0].decisions[0].choice == "none (not replayable)",
        "an undescribed strategy is named but not replayable");

  o.marker = std::make_shared<Unsorted>();
  expect_throws<std::runtime_error>([&] { (void)adapt(mesh, p, plan, o); },
                                    "returned invalid element indices");

  // A refinement that misses its size target stops the loop unless
  // accepted.
  o.marker = nullptr;
  auto budget = mesh.options;
  budget.refinement.max_steps = 0;
  o.refinement = budget;
  h = adapt(mesh, p, plan, o);
  check(h.stop == AdaptiveStop::UnsatisfiedRefinement &&
            h.iterations.size() == 1 &&
            !h.iterations[0].refinement->size_satisfied,
        "an unsatisfied refinement stops the loop");
  check(h.stop_message.find("Refinement step budget exhausted") !=
            std::string::npos,
        h.stop_message);
  o.accept_unsatisfied_refinement = true;
  o.max_iterations = 1;
  h = adapt(mesh, p, plan, o);
  check(h.stop == AdaptiveStop::MaxIterations && h.iterations.size() == 2 &&
            !h.iterations[1].record.mesh.provenance->report.size_satisfied,
        "accepted unsatisfied refinements continue and stay recorded");

  // Invalid options and an unsatisfied initial mesh.
  AdaptOptions bad;
  bad.tolerance = -1;
  expect_throws<std::invalid_argument>([&] { (void)adapt(mesh, p, plan, bad); },
                                       "Adaptive tolerance");
  auto unsatisfied = mesh;
  unsatisfied.report.size_satisfied = false;
  unsatisfied.report.message = "test shortfall";
  AdaptOptions once;
  once.max_iterations = 0;
  expect_throws<std::invalid_argument>(
      [&] { (void)adapt(unsatisfied, p, plan, once); },
      "misses its size target (test shortfall)");
  h = adapt(accept_unsatisfied(unsatisfied), p, plan, once);
  check(identical(h.solution.values, direct.values),
        "an accepted initial mesh is solved as is");
}
} // namespace

int main() {
  return pde2d_test::run(
      {{"adaptive_loop_reduces_error", adaptive_loop_reduces_error},
       {"history_json_round_trip", history_json_round_trip},
       {"stopping_rules", stopping_rules}});
}
