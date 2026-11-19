// The adaptive loop for static problems: on the L-shape with the corner
// singularity u = r^(2/3) sin(2 theta / 3), adaptive refinement reaches
// the optimal rate N^(-1/2) of the energy error, uniform refinement only
// about N^(-1/3); the loop stops on its tolerance, on max_cycles, on the
// node budget and when nothing changes, and says so; invalid
// configurations and targets throw.

#include "adapt/estimator_support.hpp"
#include "solve/solve_support.hpp"
#include "support/check.hpp"

#include "pde2d/adapt/adaptive_static_solver.hpp"
#include "pde2d/adapt/dorfler_marking.hpp"
#include "pde2d/adapt/kelly_estimator.hpp"
#include "pde2d/adapt/maximum_marking.hpp"
#include "pde2d/adapt/mesh_refinement.hpp"
#include "pde2d/adapt/recovery_estimator.hpp"
#include "pde2d/adapt/residual_estimator.hpp"
#include "pde2d/geometry/curve_geometry.hpp"
#include "pde2d/mesh/cdt.hpp"
#include "pde2d/mesh/chord_length_discretizer.hpp"
#include "pde2d/mesh/curve_chain.hpp"
#include "pde2d/mesh/mesher.hpp"
#include "pde2d/mesh/newest_vertex_bisection.hpp"
#include "pde2d/solve/sparse_lu.hpp"

#include <cmath>
#include <iostream>
#include <memory>
#include <numbers>
#include <vector>

using namespace pde2d;
using namespace pde2d::adapt;
using fem::Expression;
using pde2d::test::check;
using pde2d::test::expect_throws;

namespace {

constexpr double pi = std::numbers::pi;

// The L-shape (-1, 1)^2 without [0, 1] x [-1, 0], re-entrant corner at
// the origin, one region per side.
std::shared_ptr<const mesh::Mesh> l_shape_mesh(double h) {
  const std::vector<geometry::Point> p{{-1, -1}, {0, -1}, {0, 0},
                                       {1, 0},   {1, 1},  {-1, 1}};
  const std::vector<std::string> regions{"bottom", "corner_y", "corner_x",
                                         "right",  "top",      "left"};
  geometry::Loop loop;
  for (std::size_t i = 0; i < p.size(); ++i)
    loop.curves.push_back(
        {geometry::Line{p[i], p[(i + 1) % p.size()]}, regions[i]});
  mesh::Mesher mesher(std::make_shared<mesh::ChordLengthDiscretizer>(),
                      std::make_shared<mesh::Cdt>());
  mesh::Mesh m = mesher
                     .run(geometry::CurveGeometry({loop}),
                          {.max_edge_length = {.absolute = h}})
                     .mesh;
  // The Mesher delivers the longest edges as refinement edges; newest
  // vertex bisection then keeps them from cycle to cycle.
  return std::make_shared<const mesh::Mesh>(std::move(m));
}

// -Laplace u = 0 with u = r^(2/3) sin(2 theta / 3) on the boundary
// (theta in [0, 3 pi / 2], measured from the positive x axis), written
// per side: zero on the two sides at the corner.
fem::Problem singular_problem() {
  const auto dirichlet = [](const char *g) {
    return fem::BoundaryCondition{.alpha = Expression("1"), .g = Expression(g)};
  };
  return {.equation = {.a_xx = Expression("-1"), .a_yy = Expression("-1")},
          .boundary_conditions = {
              {"corner_x", dirichlet("0")},
              {"corner_y", dirichlet("0")},
              {"right", dirichlet("(1 + y^2)^(1/3)*sin(2/3*atan(y))")},
              {"top", dirichlet("(1 + x^2)^(1/3)*sin(2/3*(pi/2 - atan(x)))")},
              {"left", dirichlet("(1 + y^2)^(1/3)*sin(2/3*(pi - atan(y)))")},
              {"bottom",
               dirichlet("(1 + x^2)^(1/3)*sin(2/3*(3*pi/2 + atan(x)))")}}};
}

// grad u = 2/3 r^(-1/3) (sin(-theta / 3), cos(-theta / 3)).
geometry::Point exact_gradient(double x, double y) {
  double theta = std::atan2(y, x);
  if (theta < 0)
    theta += 2 * pi;
  const double r = std::hypot(x, y);
  const double s = 2.0 / 3.0 * std::pow(r, -1.0 / 3.0);
  return {s * std::sin(-theta / 3), s * std::cos(-theta / 3)};
}

double true_error(const AdaptiveCycle &cycle) {
  return test::energy_error(*cycle.solution.mesh, cycle.solution.values,
                            exact_gradient, test::identity_weight,
                            geometry::Point{0, 0});
}

AdaptiveStaticSolver loop(std::shared_ptr<const ErrorEstimator> estimator,
                          std::shared_ptr<const MarkingStrategy> marking) {
  auto refinement = std::make_shared<MeshRefinement>(
      std::move(estimator), std::move(marking),
      std::make_shared<mesh::Remesher>(
          std::make_shared<mesh::NewestVertexBisection>()));
  return AdaptiveStaticSolver(
      solve::StaticSolver(std::make_shared<solve::SparseLu>()), {refinement});
}

std::shared_ptr<const fem::MeshedProblem> start(double h = 0.5) {
  return std::make_shared<const fem::MeshedProblem>(l_shape_mesh(h),
                                                    singular_problem());
}

// The slope of log(error) against log(N) by least squares over the cycles
// from `first` on.
double slope(const std::vector<double> &n, const std::vector<double> &error,
             std::size_t first) {
  double sx = 0, sy = 0, sxx = 0, sxy = 0, k = 0;
  for (std::size_t i = first; i < n.size(); ++i) {
    const double x = std::log(n[i]), y = std::log(error[i]);
    sx += x;
    sy += y;
    sxx += x * x;
    sxy += x * y;
    k += 1;
  }
  return (k * sxy - sx * sy) / (k * sxx - sx * sx);
}

struct Run {
  std::vector<double> nodes, errors, estimates;
};

Run run(const AdaptiveStaticSolver &solver, std::size_t max_nodes,
        const char *name) {
  const AdaptiveStaticResult result =
      solver.solve(start(), {.max_nodes = max_nodes});
  check(result.stop_reason == StopReason::node_budget,
        "stops on the node budget");
  Run out;
  for (const AdaptiveCycle &cycle : result.cycles) {
    out.nodes.push_back(static_cast<double>(cycle.solution.mesh->nodes.size()));
    out.errors.push_back(true_error(cycle));
    out.estimates.push_back(cycle.estimate->total);
    std::cout << "INFO " << name << ": " << out.nodes.back() << " nodes, error "
              << out.errors.back() << ", estimate " << out.estimates.back()
              << ", effectivity " << out.estimates.back() / out.errors.back()
              << '\n';
  }
  return out;
}

// Adaptive refinement (residual estimator, Dörfler 0.5, NVB) against
// uniform refinement (every triangle marked) up to 20000 nodes.
void l_shape_rates() {
  const Run adaptive = run(loop(std::make_shared<ResidualEstimator>(),
                                std::make_shared<DorflerMarking>(
                                    DorflerMarkingOptions{.theta = 0.5})),
                           20000, "adaptive");
  const Run uniform = run(loop(std::make_shared<ResidualEstimator>(),
                               std::make_shared<MaximumMarking>(
                                   MaximumMarkingOptions{.theta = 0.0})),
                          20000, "uniform");
  // Rates over the second half of the cycles (pre-asymptotic first).
  const double a =
      slope(adaptive.nodes, adaptive.errors, adaptive.nodes.size() / 2);
  const double u =
      slope(uniform.nodes, uniform.errors, uniform.nodes.size() / 2);
  const double ae =
      slope(adaptive.nodes, adaptive.estimates, adaptive.nodes.size() / 2);
  std::cout << "INFO rates: adaptive error " << a << ", adaptive estimate "
            << ae << ", uniform error " << u << '\n';
  check(a < -0.42 && a > -0.6, "adaptive: about N^(-1/2)");
  check(ae < -0.42 && ae > -0.6, "adaptive estimate: about N^(-1/2)");
  check(u > -0.4 && u < -0.27, "uniform: about N^(-1/3)");
  check(adaptive.errors.back() < 0.5 * uniform.errors.back(),
        "adaptive is clearly better at the same budget");
  // The residual estimator stays equivalent to the error on the graded
  // meshes too.
  for (std::size_t i = 0; i < adaptive.nodes.size(); ++i) {
    const double effectivity = adaptive.estimates[i] / adaptive.errors[i];
    check(effectivity > 1.0 && effectivity < 10.0, "effectivity in a band");
  }
}

// The same with the Kelly and recovery estimators: also the optimal rate.
void l_shape_other_estimators() {
  for (const auto &[estimator, name] :
       {std::pair<std::shared_ptr<const ErrorEstimator>, const char *>{
            std::make_shared<KellyEstimator>(), "kelly"},
        std::pair<std::shared_ptr<const ErrorEstimator>, const char *>{
            std::make_shared<RecoveryEstimator>(), "recovery"}}) {
    const Run r = run(loop(estimator, std::make_shared<DorflerMarking>(
                                          DorflerMarkingOptions{.theta = 0.5})),
                      8000, name);
    const double a = slope(r.nodes, r.errors, r.nodes.size() / 2);
    std::cout << "INFO " << name << " adaptive rate " << a << '\n';
    check(a < -0.42 && a > -0.6, std::string(name) + ": about N^(-1/2)");
  }
}

AdaptiveStaticSolver dorfler_loop() {
  return loop(std::make_shared<ResidualEstimator>(),
              std::make_shared<DorflerMarking>());
}

void stops_on_tolerance() {
  // Relative: a tenth of the first estimate.
  const AdaptiveStaticResult r =
      dorfler_loop().solve(start(), {.tolerance = {.relative = 0.1}});
  check(r.stop_reason == StopReason::tolerance_met, "tolerance met");
  const double tolerance = 0.1 * r.cycles.front().estimate->total;
  check(r.cycles.back().estimate->total <= tolerance, "last within");
  for (std::size_t i = 0; i + 1 < r.cycles.size(); ++i)
    check(r.cycles[i].estimate->total > tolerance, "earlier ones not");
  test::near(r.report.metrics.at("tolerance"), tolerance, 0.0, "reported");
  check(r.report.messages.back().starts_with("stopped: tolerance_met"),
        "the report says why");
  check(r.problem->mesh() == r.cycles.back().solution.mesh,
        "the problem of the last cycle");
  // Absolute, with both: the smaller applies.
  const double absolute = 0.5 * tolerance;
  const AdaptiveStaticResult both = dorfler_loop().solve(
      start(), {.tolerance = {.absolute = absolute, .relative = 0.1}});
  check(both.cycles.back().estimate->total <= absolute, "the smaller one");
  check(both.cycles.size() > r.cycles.size(), "more cycles");
}

void stops_on_max_cycles() {
  const AdaptiveStaticResult r = dorfler_loop().solve(
      start(), {.tolerance = {.absolute = 1e-9}, .max_cycles = 3});
  check(r.stop_reason == StopReason::max_cycles, "max cycles");
  check(r.cycles.size() == 3, "three cycles");
  check(r.report.metrics.at("cycles") == 3.0, "reported");
  check(r.report.messages.back().starts_with("stopped: max_cycles"), "why");
  // Every cycle but the last adapted: estimator, mesh_refinement, Remesher.
  for (std::size_t i = 0; i < 3; ++i) {
    bool refined = false;
    for (const auto &report : r.cycles[i].reports)
      refined = refined || report.stage == "mesh_refinement";
    check(refined == (i < 2), "adapted after every cycle but the last");
  }
  // Later cycles start with the report of the problem the loop built.
  check(r.cycles[1].reports.front().stage == "meshed_problem", "rebuilt");
  check(r.cycles[0].reports.front().stage == "assembly", "given problem");
  const AdaptiveStaticResult once =
      dorfler_loop().solve(start(), {.max_cycles = 1});
  check(once.cycles.size() == 1 && once.stop_reason == StopReason::max_cycles,
        "one cycle");
}

// The cycle observer sees every cycle with its estimate and problem, and
// stops the loop; the tolerance and max_cycles come first.
void observer_sees_and_stops_cycles() {
  std::vector<double> estimates;
  std::vector<std::size_t> nodes;
  const AdaptiveStaticResult watched = dorfler_loop().solve(
      start(), {.max_cycles = 3}, 0.0,
      [&](const AdaptiveCycle &cycle, const fem::MeshedProblem &problem) {
        estimates.push_back(cycle.estimate->total);
        nodes.push_back(problem.mesh()->nodes.size());
        check(cycle.solution.mesh == problem.mesh(), "the cycle's problem");
        return true;
      });
  check(watched.stop_reason == StopReason::max_cycles && estimates.size() == 3,
        "every cycle seen");
  for (std::size_t i = 0; i < 3; ++i)
    check(estimates[i] == watched.cycles[i].estimate->total &&
              nodes[i] == watched.cycles[i].solution.mesh->nodes.size(),
          "as the result has it");
  const AdaptiveStaticResult stopped = dorfler_loop().solve(
      start(), {.max_cycles = 5}, 0.0,
      [](const AdaptiveCycle &, const fem::MeshedProblem &) { return false; });
  check(stopped.stop_reason == StopReason::stopped &&
            stopped.cycles.size() == 1,
        "stopped after the first cycle");
  check(stopped.problem->mesh() == stopped.cycles.back().solution.mesh,
        "the problem of the last cycle");
  check(stopped.report.messages.back().starts_with("stopped: stopped"),
        "the report says why");
  bool adapted = false;
  for (const auto &report : stopped.cycles[0].reports)
    adapted = adapted || report.stage == "mesh_refinement";
  check(!adapted, "no adaptation after a stop");
  check(to_string(StopReason::stopped) == "stopped", "its name");
  const AdaptiveStaticResult last = dorfler_loop().solve(
      start(), {.max_cycles = 1}, 0.0,
      [](const AdaptiveCycle &, const fem::MeshedProblem &) { return false; });
  check(last.stop_reason == StopReason::max_cycles,
        "the last allowed cycle stops for max_cycles");
}

void stops_on_node_budget() {
  const std::size_t budget = 300;
  const AdaptiveStaticResult r =
      dorfler_loop().solve(start(), {.max_nodes = budget});
  check(r.stop_reason == StopReason::node_budget, "node budget");
  for (const auto &cycle : r.cycles)
    check(cycle.solution.mesh->nodes.size() <= budget, "within the budget");
  check(r.report.messages.back().starts_with("stopped: node_budget"), "why");
  check(r.report.messages.back().find("more than max_nodes = 300") !=
            std::string::npos,
        "the message gives the budget");
  // The last cycle's refinement was done and discarded.
  bool refined = false;
  for (const auto &report : r.cycles.back().reports)
    refined = refined || report.stage == "mesh_refinement";
  check(refined, "the discarded refinement is reported");
}

// u = x + y on two triangles, every node Dirichlet: u_h is exact, every
// indicator is exactly zero, nothing is marked.
void stops_when_nothing_changes() {
  const fem::BoundaryCondition c{.alpha = Expression("1"),
                                 .g = Expression("x + y")};
  const fem::Problem problem{
      .equation = {.a_xx = Expression("-1"), .a_yy = Expression("-1")},
      .boundary_conditions = {
          {"bottom", c}, {"right", c}, {"top", c}, {"left", c}}};
  const auto meshed =
      std::make_shared<const fem::MeshedProblem>(test::unit_square(1), problem);
  const AdaptiveStaticResult r =
      dorfler_loop().solve(meshed, {.max_cycles = 5});
  check(r.stop_reason == StopReason::nothing_changed, "nothing changed");
  check(r.cycles.size() == 1, "one cycle");
  check(r.cycles.front().estimate->total == 0.0, "zero estimate");
  check(r.problem == meshed, "the given problem");
}

// Review regression: one triangle whose bottom edge is linked to an arc
// bulging in past the opposite vertex, so the boundary split policy cannot
// split it and the refiner leaves the marked triangle as it is. The
// adaptation reports no new mesh and the loop stops with nothing_changed,
// with only a tolerance (before the fix it repeated without end).
void stops_when_refinement_is_blocked() {
  mesh::Mesh m;
  m.nodes = {{0, 0}, {2, 0}, {1, 0.2}};
  m.triangles = {{{0, 1, 2}}};
  m.region_names = {"r"};
  // (0,0) over (1,1) to (2,0).
  m.chains = {mesh::CurveChain(std::vector<geometry::Curve>{
      geometry::EllipticalArc{{1, 0}, 1, 1, 0, pi, -pi}})};
  m.boundary_edges = {
      mesh::BoundaryEdge{
          {0, 1}, 0, mesh::CurveLink{0, 0, m.chains[0].length()}},
      mesh::BoundaryEdge{{1, 2}, 0, std::nullopt},
      mesh::BoundaryEdge{{2, 0}, 0, std::nullopt}};
  mesh::set_longest_edges_as_refinement_edges(m);
  const fem::Problem problem{
      .equation = {.a_xx = Expression("-1"),
                   .a_yy = Expression("-1"),
                   .f = Expression("1")},
      .boundary_conditions = {
          {"r", {.alpha = Expression("1"), .g = Expression("0")}}}};
  const auto meshed = std::make_shared<const fem::MeshedProblem>(
      std::make_shared<const mesh::Mesh>(m), problem);
  const AdaptiveStaticResult r =
      dorfler_loop().solve(meshed, {.tolerance = {.absolute = 1e-9}});
  check(r.stop_reason == StopReason::nothing_changed, "nothing changed");
  check(r.cycles.size() == 1, "one cycle");
  check(r.cycles.front().estimate->total > 0.0, "a positive estimate");
  bool said = false;
  for (const auto &report : r.cycles.back().reports)
    for (const auto &message : report.messages)
      said = said || message.starts_with("refinement changed nothing");
  check(said, "the report says why");
}

// An adaptation that neither estimates nor changes anything.
class Idle final : public Adaptation {
public:
  std::string name() const override { return "idle"; }
  AdaptationCapabilities capabilities() const override { return {}; }
  std::optional<ErrorEstimate> estimate(const SolvedState &) const override {
    return std::nullopt;
  }
  AdaptationResult adapt(const SolvedState &,
                         const ErrorEstimate *) const override {
    return {};
  }
};

// Adaptations that break their declarations.
class NoEstimate final : public Adaptation {
public:
  std::string name() const override { return "no_estimate"; }
  AdaptationCapabilities capabilities() const override {
    return {.estimates_error = true};
  }
  std::optional<ErrorEstimate> estimate(const SolvedState &) const override {
    return std::nullopt;
  }
  AdaptationResult adapt(const SolvedState &,
                         const ErrorEstimate *) const override {
    return {};
  }
};

class UndeclaredMesh final : public Adaptation {
public:
  std::string name() const override { return "undeclared_mesh"; }
  AdaptationCapabilities capabilities() const override { return {}; }
  std::optional<ErrorEstimate> estimate(const SolvedState &) const override {
    return std::nullopt;
  }
  AdaptationResult adapt(const SolvedState &state,
                         const ErrorEstimate *) const override {
    return {.mesh = state.problem.mesh()};
  }
};

void invalid_configurations() {
  const solve::StaticSolver solver(std::make_shared<solve::SparseLu>());
  const auto estimator = std::make_shared<ResidualEstimator>();
  const auto marking = std::make_shared<DorflerMarking>();
  const auto remesher = std::make_shared<mesh::Remesher>(
      std::make_shared<mesh::NewestVertexBisection>());
  expect_throws<std::invalid_argument>(
      [&] { MeshRefinement(nullptr, marking, remesher); },
      "mesh_refinement: the estimator is null");
  expect_throws<std::invalid_argument>(
      [&] { MeshRefinement(estimator, nullptr, remesher); },
      "mesh_refinement: the marking strategy is null");
  expect_throws<std::invalid_argument>(
      [&] { MeshRefinement(estimator, marking, nullptr); },
      "mesh_refinement: the remesher is null");
  const auto refinement =
      std::make_shared<MeshRefinement>(estimator, marking, remesher);
  expect_throws<std::invalid_argument>(
      [&] { AdaptiveStaticSolver(solver, {}); },
      "at least one adaptation is needed");
  expect_throws<std::invalid_argument>(
      [&] { AdaptiveStaticSolver(solver, {refinement, nullptr}); },
      "adaptation 1 is null");
  expect_throws<std::invalid_argument>(
      [&] { AdaptiveStaticSolver(solver, {refinement, refinement}); },
      "at most one adaptation may change the mesh");

  const AdaptiveStaticSolver good(solver, {refinement});
  const auto problem = start();
  expect_throws<std::invalid_argument>(
      [&] { (void)good.solve(nullptr, {.max_cycles = 1}); },
      "the problem is null");
  expect_throws<std::invalid_argument>(
      [&] { (void)good.solve(problem, {.max_cycles = 1}, std::nan("")); },
      "t is not finite");
  expect_throws<std::invalid_argument>(
      [&] { (void)good.solve(problem, {}); },
      "the target needs a tolerance, max_cycles or max_nodes");
  expect_throws<std::invalid_argument>(
      [&] { (void)good.solve(problem, {.tolerance = {.absolute = -1.0}}); },
      "tolerance.absolute must be finite and positive");
  expect_throws<std::invalid_argument>(
      [&] {
        (void)good.solve(problem, {.tolerance = {.relative = std::nan("")}});
      },
      "tolerance.relative must be finite and positive");
  expect_throws<std::invalid_argument>(
      [&] { (void)good.solve(problem, {.max_cycles = 0}); },
      "max_cycles must be at least 1");
  expect_throws<std::invalid_argument>(
      [&] { (void)good.solve(problem, {.max_nodes = 0}); },
      "max_nodes must be at least 1");
  expect_throws<std::invalid_argument>(
      [&] { (void)good.solve(problem, {.max_nodes = 5}); },
      "more than max_nodes = 5");
  const AdaptiveStaticSolver idle(solver, {std::make_shared<Idle>()});
  expect_throws<std::invalid_argument>(
      [&] { (void)idle.solve(problem, {.tolerance = {.absolute = 1.0}}); },
      "a tolerance needs an adaptation that estimates the error");
  // Without a tolerance it runs: nothing estimates, nothing changes.
  const AdaptiveStaticResult r = idle.solve(problem, {.max_cycles = 3});
  check(r.stop_reason == StopReason::nothing_changed, "idle changes nothing");
  check(!r.cycles.front().estimate, "no estimate");
  expect_throws<std::runtime_error>(
      [&] {
        (void)AdaptiveStaticSolver(solver, {std::make_shared<NoEstimate>()})
            .solve(problem, {.max_cycles = 2});
      },
      "no_estimate declares estimates_error but gave no estimate");
  expect_throws<std::runtime_error>(
      [&] {
        (void)AdaptiveStaticSolver(solver, {std::make_shared<UndeclaredMesh>()})
            .solve(problem, {.max_cycles = 2});
      },
      "undeclared_mesh returned a mesh but does not declare changes_mesh");
  // An adaptation that does not change the mesh may join one that does.
  const AdaptiveStaticSolver both(solver,
                                  {std::make_shared<Idle>(), refinement});
  const AdaptiveStaticResult b =
      both.solve(problem, {.tolerance = {.relative = 0.5}});
  check(b.stop_reason == StopReason::tolerance_met,
        "the estimate comes from the first adaptation that estimates");
}

} // namespace

int main() {
  return test::run({
      {"the cycle observer sees every cycle and can stop the loop",
       observer_sees_and_stops_cycles},
      {"L-shape: adaptive N^(-1/2), uniform N^(-1/3)", l_shape_rates},
      {"L-shape with the Kelly and recovery estimators",
       l_shape_other_estimators},
      {"stops on the tolerance", stops_on_tolerance},
      {"stops on max_cycles", stops_on_max_cycles},
      {"stops on the node budget", stops_on_node_budget},
      {"stops when nothing changes", stops_when_nothing_changes},
      {"stops when refinement is blocked", stops_when_refinement_is_blocked},
      {"invalid configurations and targets", invalid_configurations},
  });
}
