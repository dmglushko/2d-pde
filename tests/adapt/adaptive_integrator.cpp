// The adaptive loop for time-dependent problems. A steep bump moving
// through a heat equation (manufactured solution): the adaptive run
// refines where the bump is and ends with a clearly smaller energy error
// than uniform refinement with at least as many nodes. Adaptation times
// follow the interval and step rules exactly (whichever comes first); a
// run that never refines equals one Integrator run that lands on the same
// times, bit for bit for the first-order methods; a wave equation (time
// order 2) runs with its velocities transferred; the node budget stops
// adapting but not the run; invalid targets throw.

#include "adapt/estimator_support.hpp"
#include "solve/solve_support.hpp"
#include "support/check.hpp"
#include "support/shapes.hpp"

#include "pde2d/adapt/adaptive_integrator.hpp"
#include "pde2d/adapt/dorfler_marking.hpp"
#include "pde2d/adapt/kelly_estimator.hpp"
#include "pde2d/adapt/mesh_refinement.hpp"
#include "pde2d/adapt/residual_estimator.hpp"
#include "pde2d/geometry/curve_geometry.hpp"
#include "pde2d/mesh/cdt.hpp"
#include "pde2d/mesh/chord_length_discretizer.hpp"
#include "pde2d/mesh/mesher.hpp"
#include "pde2d/mesh/newest_vertex_bisection.hpp"
#include "pde2d/mesh/remesher.hpp"
#include "pde2d/mesh/ruppert.hpp"
#include "pde2d/solve/fixed_step.hpp"
#include "pde2d/solve/implicit_euler.hpp"
#include "pde2d/solve/implicit_midpoint.hpp"
#include "pde2d/solve/newmark.hpp"
#include "pde2d/solve/sparse_lu.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

using namespace pde2d;
using namespace pde2d::adapt;
using fem::Expression;
using geometry::Point;
using pde2d::test::check;
using pde2d::test::expect_throws;
using pde2d::test::near;

namespace {

constexpr double pi = std::numbers::pi;

solve::Integrator integrator(std::shared_ptr<const solve::TimeMethod> method,
                             double dt) {
  return solve::Integrator(
      std::move(method),
      std::make_shared<solve::FixedStep>(solve::FixedStepOptions{.dt = dt}),
      std::make_shared<solve::SparseLu>());
}

std::shared_ptr<const Adaptation>
refinement(std::shared_ptr<const ErrorEstimator> estimator =
               std::make_shared<ResidualEstimator>()) {
  return std::make_shared<MeshRefinement>(
      std::move(estimator),
      std::make_shared<DorflerMarking>(DorflerMarkingOptions{.theta = 0.5}),
      std::make_shared<mesh::Remesher>(
          std::make_shared<mesh::NewestVertexBisection>()));
}

std::shared_ptr<const mesh::Mesh> unit_square_mesh(double h) {
  const mesh::Mesher mesher(std::make_shared<mesh::ChordLengthDiscretizer>(),
                            std::make_shared<mesh::Cdt>(),
                            std::make_shared<mesh::Ruppert>());
  return std::make_shared<const mesh::Mesh>(
      mesher
          .run(geometry::CurveGeometry(
                   {test::square_loop({0, 0}, 1, "boundary")}),
               {.max_edge_length = {.absolute = h}})
          .mesh);
}

// --- the moving bump --------------------------------------------------------
//
// u = exp(-r^2 / s^2), r the distance to (0.3 + 0.4 t, 0.5), s = 0.08, for
// u_t - (u_xx + u_yy) = f on the unit square, u = exact on the boundary.
constexpr double s2 = 0.0064;
double bump_center(double t) { return 0.3 + 0.4 * t; }

fem::Problem bump_problem() {
  const std::string dx = "(x - 0.3 - 0.4*t)", dy = "(y - 0.5)";
  const std::string r2 = "(" + dx + "^2 + " + dy + "^2)";
  const std::string u = "exp(-" + r2 + "/0.0064)";
  // f = u_t - Laplace u = u (0.8 dx / s^2 - 4 r^2 / s^4 + 4 / s^2).
  const std::string f =
      u + "*(0.8*" + dx + "/0.0064 - 4*" + r2 + "/0.0064^2 + 4/0.0064)";
  return {.equation = {.a_xx = Expression("-1"),
                       .a_yy = Expression("-1"),
                       .m1 = Expression("1"),
                       .f = Expression(f)},
          .boundary_conditions = {{"boundary",
                                   {.alpha = Expression("1"),
                                    .g = Expression(u)}}},
          .initial = {.u = Expression(u)}};
}

Point bump_gradient(double x, double y, double t) {
  const double dx = x - bump_center(t), dy = y - 0.5;
  const double u = std::exp(-(dx * dx + dy * dy) / s2);
  return {-2 * dx / s2 * u, -2 * dy / s2 * u};
}

double bump_error(const solve::Solution &s) {
  return test::energy_error(*s.mesh, s.values, [&](double x, double y) {
    return bump_gradient(x, y, s.time);
  });
}

// The nodes per unit area within `radius` of p, and beyond `outside`.
std::pair<double, double> densities(const mesh::Mesh &m, Point p,
                                    double radius, double outside) {
  double near_count = 0, far_count = 0;
  for (const Point &q : m.nodes) {
    const double d = distance(p, q);
    if (d < radius)
      near_count += 1;
    else if (d > outside)
      far_count += 1;
  }
  // The far area: the unit square minus the disk of radius `outside`
  // around p (inside the square for the points used here).
  return {near_count / (pi * radius * radius),
          far_count / (1.0 - pi * outside * outside)};
}

void moving_bump() {
  const double dt = 1.0 / 64, t1 = 0.5;
  const auto start = std::make_shared<const fem::MeshedProblem>(
      unit_square_mesh(0.1), bump_problem());
  const AdaptiveIntegrator adaptive(
      integrator(std::make_shared<solve::ImplicitEuler>(), dt),
      {refinement()});
  const AdaptiveIntegrationResult r = adaptive.integrate(
      start, 0.0, t1, {.every = {.steps = 2}, .max_nodes = 6000});
  check(r.final.time == t1, "reaches t1");
  check(r.final.mesh == r.pieces.back().problem->mesh(),
        "the final solution is on the last mesh");
  const std::size_t nodes = r.final.mesh->nodes.size();
  std::size_t refined = 0;
  for (const auto &piece : r.pieces)
    if (piece.adaptation &&
        piece.adaptation->outcome == AdaptationOutcome::refined)
      ++refined;
  check(refined >= 8, "refined at most adaptation times");
  check(r.report.metrics.at("refinements") == static_cast<double>(refined),
        "refinements reported");
  const double adaptive_error = bump_error(r.final);

  // Uniform refinement of the same starting mesh up to at least as many
  // nodes, integrated with the same method and step.
  std::shared_ptr<const mesh::Mesh> uniform = start->mesh();
  const mesh::Remesher nvb(std::make_shared<mesh::NewestVertexBisection>());
  std::shared_ptr<const mesh::Mesh> coarser = uniform;
  while (uniform->nodes.size() < nodes) {
    std::vector<std::size_t> all(uniform->triangles.size());
    for (std::size_t t = 0; t < all.size(); ++t)
      all[t] = t;
    coarser = uniform;
    uniform = std::make_shared<const mesh::Mesh>(nvb.refine(*uniform, all).mesh);
  }
  const auto plain = integrator(std::make_shared<solve::ImplicitEuler>(), dt);
  const auto uniform_run = plain.integrate(
      std::make_shared<const fem::MeshedProblem>(uniform, bump_problem()), 0.0,
      t1);
  const auto coarser_run = plain.integrate(
      std::make_shared<const fem::MeshedProblem>(coarser, bump_problem()), 0.0,
      t1);
  const auto start_run = plain.integrate(start, 0.0, t1);
  const double uniform_error = bump_error(uniform_run.final);
  const double coarser_error = bump_error(coarser_run.final);
  std::cout << "INFO starting mesh: " << start->mesh()->nodes.size()
            << " nodes, error " << bump_error(start_run.final) << '\n'
            << "INFO adaptive: " << nodes << " nodes, error " << adaptive_error
            << ", " << refined << " refinements\n"
            << "INFO uniform: " << uniform->nodes.size() << " nodes, error "
            << uniform_error << "; " << coarser->nodes.size()
            << " nodes, error " << coarser_error << '\n';
  check(uniform->nodes.size() >= nodes, "uniform has at least as many nodes");
  check(adaptive_error < 0.6 * uniform_error,
        "adaptive is clearly more accurate with no more nodes");

  // The nodes are where the bump went, densest where it is at t1.
  const auto [dense, sparse] =
      densities(*r.final.mesh, {bump_center(t1), 0.5}, 0.12, 0.45);
  std::cout << "INFO node density near the bump " << dense << ", far "
            << sparse << '\n';
  check(dense > 8 * sparse, "refined where the bump is");

  // The midpoint method does not damp stiff components, so the
  // interpolation error each transfer leaves on the new mesh's fine modes
  // stays (measured, not a requirement; see adaptive_integrator.hpp).
  const auto midpoint =
      AdaptiveIntegrator(
          integrator(std::make_shared<solve::ImplicitMidpoint>(), dt),
          {refinement()})
          .integrate(start, 0.0, t1, {.every = {.steps = 2}, .max_nodes = 6000});
  const auto midpoint_uniform =
      integrator(std::make_shared<solve::ImplicitMidpoint>(), dt)
          .integrate(std::make_shared<const fem::MeshedProblem>(
                         uniform, bump_problem()),
                     0.0, t1);
  std::cout << "INFO implicit midpoint: adaptive "
            << midpoint.final.mesh->nodes.size() << " nodes, error "
            << bump_error(midpoint.final) << "; uniform "
            << uniform->nodes.size() << " nodes, error "
            << bump_error(midpoint_uniform.final) << '\n';
  // The largest indicator at the last adaptation time is at the bump.
  const AdaptationPoint *last = nullptr;
  for (const auto &piece : r.pieces)
    if (piece.adaptation)
      last = &*piece.adaptation;
  check(last && last->estimate, "the last adaptation time has an estimate");
  const auto &eta = last->estimate->indicators;
  const std::size_t worst = static_cast<std::size_t>(
      std::max_element(eta.begin(), eta.end()) - eta.begin());
  // The estimate is of the mesh integrated before that time: the piece
  // that ends there.
  const mesh::Mesh *before = nullptr;
  for (const auto &piece : r.pieces)
    if (piece.adaptation && &*piece.adaptation == last)
      before = piece.problem->mesh().get();
  const auto &tri = before->triangles[worst].nodes;
  const Point centroid = (1.0 / 3) * (before->nodes[tri[0]] +
                                      before->nodes[tri[1]] +
                                      before->nodes[tri[2]]);
  check(distance(centroid, {bump_center(last->time), 0.5}) < 0.2,
        "the largest indicator is at the bump");
}

// One uniform refinement at t = 1/32 (Dorfler with theta = 1 marks every
// positive indicator; the budget then stops adapting), against a run on the
// refined mesh from t0. Implicit Euler damps what the transfer left in the
// fine modes within a few steps: the two agree (to 2e-5 two steps after
// the transfer, to rounding later). The midpoint method
// does not: the transferred run's error decays only slowly towards the
// fresh one (measured).
void transfer_error_and_damping() {
  const double dt = 1.0 / 64;
  const auto start = std::make_shared<const fem::MeshedProblem>(
      unit_square_mesh(0.1), bump_problem());
  const auto uniform_refinement = std::make_shared<MeshRefinement>(
      std::make_shared<ResidualEstimator>(),
      std::make_shared<DorflerMarking>(DorflerMarkingOptions{.theta = 1.0}),
      std::make_shared<mesh::Remesher>(
          std::make_shared<mesh::NewestVertexBisection>()));
  std::vector<double> times;
  for (int k = 4; k <= 32; k += 4)
    times.push_back(k * dt);
  for (const auto &method :
       std::vector<std::shared_ptr<const solve::TimeMethod>>{
           std::make_shared<solve::ImplicitEuler>(),
           std::make_shared<solve::ImplicitMidpoint>()}) {
    const auto r =
        AdaptiveIntegrator(integrator(method, dt), {uniform_refinement})
            .integrate(start, 0.0, 0.5,
                       {.every = {.steps = 2}, .max_nodes = 1000},
                       {.snapshot_times = times});
    check(r.report.metrics.at("refinements") == 1.0, "one refinement");
    check(r.pieces[0].adaptation->time == 2 * dt, "at t = 1/32");
    const auto fresh = integrator(method, dt).integrate(
        std::make_shared<const fem::MeshedProblem>(r.final.mesh,
                                                   bump_problem()),
        0.0, 0.5, {.snapshot_times = times});
    for (std::size_t k = 0; k < times.size(); ++k) {
      const double transferred = bump_error(r.snapshots[k]);
      const double direct = bump_error(fresh.snapshots[k]);
      std::cout << "INFO " << method->name() << " t = " << times[k]
                << ": transferred " << transferred << ", fresh " << direct
                << '\n';
      if (method->name() == "implicit_euler")
        near(transferred, direct, 1e-4 * direct,
             "Euler: the transfer is forgotten");
    }
  }
}

// --- schedules and continuation --------------------------------------------
//
// u_t - (u_xx + u_yy) = 0 on the unit square, u = 0 on the boundary,
// u(0) = sin(pi x) sin(pi y).
fem::Problem decay_problem() {
  const fem::BoundaryCondition zero{.alpha = Expression("1"),
                                    .g = Expression("0")};
  return {.equation = {.a_xx = Expression("-1"),
                       .a_yy = Expression("-1"),
                       .m1 = Expression("1")},
          .boundary_conditions = {{"bottom", zero},
                                  {"right", zero},
                                  {"top", zero},
                                  {"left", zero}},
          .initial = {.u = Expression("sin(pi*x)*sin(pi*y)")}};
}

// An adaptation that changes nothing and records when it was asked.
class Recorder final : public Adaptation {
public:
  struct Call {
    double time;
    std::vector<double> values, u_t, u_tt;
  };
  std::string name() const override { return "recorder"; }
  AdaptationCapabilities capabilities() const override { return {}; }
  std::optional<ErrorEstimate> estimate(const SolvedState &) const override {
    return std::nullopt;
  }
  AdaptationResult adapt(const SolvedState &state,
                         const ErrorEstimate *) const override {
    calls.push_back({state.solution.time,
                     state.solution.values,
                     {state.derivatives.u_t.begin(), state.derivatives.u_t.end()},
                     {state.derivatives.u_tt.begin(),
                      state.derivatives.u_tt.end()}});
    return {};
  }
  mutable std::vector<Call> calls;
};

struct Schedule {
  AdaptationSchedule every;
  double dt;
  double t1;
  std::vector<double> snapshots;
  std::vector<double> times;       // expected adaptation times
  std::vector<std::size_t> steps;  // expected steps up to each
  std::size_t total_steps;
};

void follows_schedules() {
  const auto problem = std::make_shared<const fem::MeshedProblem>(
      test::unit_square(4), decay_problem());
  const std::vector<Schedule> cases{
      // Interval only: every 1/8, two steps of 1/16 each.
      {{.interval = 0.125}, 1.0 / 16, 1.0, {},
       {0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875}, {2, 4, 6, 8, 10, 12, 14},
       16},
      // Steps only: every 3 steps of 1/16; the last piece has one step.
      {{.steps = 3}, 1.0 / 16, 1.0, {},
       {3.0 / 16, 6.0 / 16, 9.0 / 16, 12.0 / 16, 15.0 / 16}, {3, 6, 9, 12, 15},
       16},
      // Both, the steps first (3/16 < 1/4).
      {{.interval = 0.25, .steps = 3}, 1.0 / 16, 1.0, {},
       {3.0 / 16, 6.0 / 16, 9.0 / 16, 12.0 / 16, 15.0 / 16}, {3, 6, 9, 12, 15},
       16},
      // Both, the interval first (1/8 < 3/16).
      {{.interval = 0.125, .steps = 3}, 1.0 / 16, 1.0, {},
       {0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875}, {2, 4, 6, 8, 10, 12, 14},
       16},
      // Both, each winning once: snapshot landings add steps, so 4 steps
      // come before 0.2 (0.02, 0.04, 0.06, 0.15); then the interval
      // (0.24, 0.33, 0.35 and 0.44, 0.53, 0.55); the last piece to 0.6
      // is one shortened step.
      {{.interval = 0.2, .steps = 4}, 0.09, 0.6, {0.02, 0.04, 0.06},
       {0.15, 0.35, 0.55}, {4, 7, 10}, 11},
  };
  for (const Schedule &c : cases) {
    for (const auto &method :
         std::vector<std::shared_ptr<const solve::TimeMethod>>{
             std::make_shared<solve::ImplicitEuler>(),
             std::make_shared<solve::ImplicitMidpoint>()}) {
      const auto recorder = std::make_shared<Recorder>();
      const AdaptiveIntegrator adaptive(integrator(method, c.dt), {recorder});
      const auto r = adaptive.integrate(
          problem, 0.0, c.t1, {.every = c.every},
          {.keep_step_history = true, .snapshot_times = c.snapshots});
      check(r.final.time == c.t1, "reaches t1");
      check(recorder->calls.size() == c.times.size(), "adaptation count");
      check(r.pieces.size() == c.times.size() + 1, "one more piece");
      check(r.history.size() == c.total_steps, "every step recorded");
      check(r.report.metrics.at("steps") ==
                static_cast<double>(c.total_steps),
            "steps reported");
      check(r.snapshots.size() == c.snapshots.size(), "every snapshot");
      for (std::size_t k = 0; k < c.times.size(); ++k) {
        near(recorder->calls[k].time, c.times[k], 1e-12, "adaptation time");
        const auto &point = *r.pieces[k].adaptation;
        check(point.time == recorder->calls[k].time, "the point's time");
        check(point.steps == c.steps[k], "steps up to the adaptation time");
        check(point.outcome == AdaptationOutcome::nothing_changed,
              "nothing changed");
        check(r.pieces[k].t_end == point.time &&
                  r.pieces[k + 1].t_start == point.time,
              "pieces meet at the adaptation time");
      }
      check(!r.pieces.back().adaptation, "no adaptation at t1");

      // The pieces continue the run exactly: one Integrator run landing on
      // the same times (the adaptation times as snapshot times) gives the
      // same values, bit for bit.
      std::vector<double> lands = c.snapshots;
      for (const auto &call : recorder->calls)
        lands.push_back(call.time);
      std::ranges::sort(lands);
      const auto whole = integrator(method, c.dt)
                             .integrate(problem, 0.0, c.t1,
                                        {.keep_step_history = true,
                                         .snapshot_times = lands});
      check(whole.final.values == r.final.values,
            "the same final values, bit for bit");
      check(whole.history.size() == r.history.size(), "the same steps");
      for (std::size_t k = 0; k < r.history.size(); ++k)
        check(whole.history[k].t == r.history[k].t &&
                  whole.history[k].dt == r.history[k].dt,
              "the same step times");
      for (std::size_t k = 0; k < c.times.size(); ++k) {
        const auto &call = recorder->calls[k];
        // The state at the adaptation time is the long run's there; u_t is
        // the difference quotient of its last step, from the long run's
        // state one step before (its history and snapshots).
        const auto at = std::ranges::find_if(
            whole.snapshots, [&](const auto &s) { return s.time == call.time; });
        check(at != whole.snapshots.end() && at->values == call.values,
              "the state at the adaptation time");
        check(call.u_tt.empty(), "no u_tt for time order 1");
        check(call.u_t.size() == call.values.size(), "u_t at every node");
        const double h = whole.history[c.steps[k] - 1].dt;
        const auto before = integrator(method, c.dt).integrate(
            problem, 0.0, c.t1, {.snapshot_times = lands}, nullptr,
            c.steps[k] - 1);
        for (std::size_t v = 0; v < call.u_t.size(); ++v)
          near(call.u_t[v], (call.values[v] - before.final.values[v]) / h,
               1e-12, "u_t = (u(t) - u(t - dt)) / dt");
      }
    }
  }
}

// A target that never refines: the run equals one Integrator run that
// lands on the adaptation times, also with a real estimator.
void never_refining_equals_plain_run() {
  const auto problem = std::make_shared<const fem::MeshedProblem>(
      unit_square_mesh(0.2), bump_problem());
  for (const auto &method :
       std::vector<std::shared_ptr<const solve::TimeMethod>>{
           std::make_shared<solve::ImplicitEuler>(),
           std::make_shared<solve::ImplicitMidpoint>()}) {
    const AdaptiveIntegrator adaptive(integrator(method, 0.1), {refinement()});
    const auto r = adaptive.integrate(
        problem, 0.0, 1.0,
        {.every = {.interval = 0.25}, .tolerance = {.absolute = 1e300}},
        {.snapshot_times = {0.3}});
    for (const auto &piece : r.pieces) {
      check(piece.problem == problem, "the same problem throughout");
      if (piece.adaptation) {
        check(piece.adaptation->outcome == AdaptationOutcome::tolerance_met,
              "the tolerance is met");
        check(piece.adaptation->estimate.has_value(), "estimated");
        check(piece.adaptation->reports.empty(), "nothing adapted");
      }
    }
    check(r.pieces.size() == 4, "pieces to 0.25, 0.5, 0.75 and 1");
    const auto whole = integrator(method, 0.1).integrate(
        problem, 0.0, 1.0, {.snapshot_times = {0.25, 0.3, 0.5, 0.75}});
    check(whole.final.values == r.final.values, "bit for bit");
    check(r.snapshots.size() == 1 &&
              r.snapshots[0].values == whole.snapshots[1].values,
          "the snapshot at 0.3");
    check(r.report.metrics.at("tolerance") == 1e300, "the tolerance");
  }
}

// --- time order 2 -----------------------------------------------------------
//
// A wave whose solution is linear in x and y at every t (from
// tests/solve/time_integration.cpp): u = p cos(2t) + q sin(3t), p = 1 + x +
// 2y, q = x - y, for u_tt + 0.1 u_t - (u_xx + u_yy) = f, Dirichlet on the
// bottom and the left, u_n = g on the right and the top. P1 elements and
// interpolation reproduce it, so an adaptive run is as accurate as a plain
// one (to the time error) only if values and velocities are carried over
// exactly.
fem::Problem linear_wave_problem() {
  const std::string p = "(1 + x + 2*y)", q = "(x - y)";
  const std::string u = p + "*cos(2*t) + " + q + "*sin(3*t)";
  const std::string u_t = "-2*" + p + "*sin(2*t) + 3*" + q + "*cos(3*t)";
  const std::string u_tt = "-4*" + p + "*cos(2*t) - 9*" + q + "*sin(3*t)";
  const fem::BoundaryCondition dirichlet{.alpha = Expression("1"),
                                         .g = Expression(u)};
  return {
      .equation = {.a_xx = Expression("-1"),
                   .a_yy = Expression("-1"),
                   .m1 = Expression("0.1"),
                   .m2 = Expression("1"),
                   .f = Expression(u_tt + " + 0.1*(" + u_t + ")")},
      .boundary_conditions =
          {{"bottom", dirichlet},
           {"left", dirichlet},
           {"right",
            {.beta_n = Expression("1"), .g = Expression("cos(2*t) + sin(3*t)")}},
           {"top",
            {.beta_n = Expression("1"),
             .g = Expression("2*cos(2*t) - sin(3*t)")}}},
      .initial = {.u = Expression(u), .u_t = Expression(u_t)}};
}

// The largest nodal errors of u and u_t of a linear wave solution.
std::pair<double, double> linear_wave_errors(const solve::Solution &s) {
  double u = 0, v = 0;
  const double t = s.time;
  for (std::size_t k = 0; k < s.mesh->nodes.size(); ++k) {
    const Point n = s.mesh->nodes[k];
    const double p = 1 + n.x + 2 * n.y, q = n.x - n.y;
    u = std::max(u, std::abs(s.values[k] -
                             (p * std::cos(2 * t) + q * std::sin(3 * t))));
    v = std::max(v, std::abs(s.velocities[k] - (-2 * p * std::sin(2 * t) +
                                                3 * q * std::cos(3 * t))));
  }
  return {u, v};
}

void wave_transfers_velocities() {
  mesh::Mesh square = *test::unit_square(4);
  mesh::set_longest_edges_as_refinement_edges(square);
  const auto start = std::make_shared<const fem::MeshedProblem>(
      std::make_shared<const mesh::Mesh>(std::move(square)),
      linear_wave_problem());
  for (const auto &method :
       std::vector<std::shared_ptr<const solve::TimeMethod>>{
           std::make_shared<solve::Newmark>(),
           std::make_shared<solve::ImplicitMidpoint>()}) {
    const double dt = 1.0 / 64;
    const AdaptiveIntegrator adaptive(integrator(method, dt), {refinement()});
    const auto r =
        adaptive.integrate(start, 0.0, 1.0, {.every = {.interval = 0.125}});
    const auto plain = integrator(method, dt).integrate(start, 0.0, 1.0);
    check(r.final.time == 1.0, "reaches t1");
    check(r.report.metrics.at("refinements") >= 3, "refined");
    check(r.final.velocities.size() == r.final.mesh->nodes.size(),
          "velocities on the final mesh");
    for (const auto &piece : r.pieces)
      if (piece.adaptation &&
          piece.adaptation->outcome == AdaptationOutcome::refined) {
        bool transfer = false;
        for (const auto &report : piece.adaptation->reports)
          transfer = transfer || report.stage == "solution_transfer";
        check(transfer, "the state was transferred");
      }
    const auto [u, v] = linear_wave_errors(r.final);
    const auto [u0, v0] = linear_wave_errors(plain.final);
    std::cout << "INFO linear wave " << method->name() << ": "
              << r.final.mesh->nodes.size() << " nodes, errors " << u << ", "
              << v << "; plain run on " << start->mesh()->nodes.size()
              << " nodes " << u0 << ", " << v0 << '\n';
    check(u0 < 1e-2 && v0 < 1e-2, "the plain run is accurate");
    // The time error depends on the mesh (through the discrete operator),
    // so the runs differ in it; a value or velocity not carried over would
    // make an error of order 1 (|u_t| reaches about 5).
    check(u < 2 * u0 && v < 4 * v0,
          "as accurate as the plain run to its time error: values and "
          "velocities carried over");
  }
}

// u_tt - (u_xx + u_yy) = 0, u = sin(pi x) sin(pi y) cos(sqrt(2) pi t): a
// measurement. A transfer puts the old mesh's interpolation error into the
// new mesh's fine modes, which a wave keeps (Newmark and the midpoint
// method do not damp them; implicit Euler does): u stays accurate, u_t at
// the nodes less so than on the starting mesh.
void standing_wave() {
  const fem::BoundaryCondition zero{.alpha = Expression("1"),
                                    .g = Expression("0")};
  const fem::Problem wave{
      .equation = {.a_xx = Expression("-1"),
                   .a_yy = Expression("-1"),
                   .m2 = Expression("1")},
      .boundary_conditions = {{"boundary", zero}},
      .initial = {.u = Expression("sin(pi*x)*sin(pi*y)"),
                  .u_t = Expression("0")}};
  const auto start = std::make_shared<const fem::MeshedProblem>(
      unit_square_mesh(0.25), wave);
  const double w = std::sqrt(2.0) * pi;
  const auto errors = [&](const solve::Solution &s) {
    double u = 0, v = 0;
    for (std::size_t k = 0; k < s.mesh->nodes.size(); ++k) {
      const Point p = s.mesh->nodes[k];
      const double shape = std::sin(pi * p.x) * std::sin(pi * p.y);
      u = std::max(u, std::abs(s.values[k] - shape * std::cos(w * s.time)));
      v = std::max(v, std::abs(s.velocities[k] +
                               shape * w * std::sin(w * s.time)));
    }
    return std::pair{u, v};
  };
  for (const auto &method :
       std::vector<std::shared_ptr<const solve::TimeMethod>>{
           std::make_shared<solve::Newmark>(),
           std::make_shared<solve::ImplicitEuler>()}) {
    const double dt = 1.0 / 128;
    const AdaptiveIntegrator adaptive(integrator(method, dt), {refinement()});
    const auto r =
        adaptive.integrate(start, 0.0, 0.5, {.every = {.interval = 0.0625}});
    const auto plain = integrator(method, dt).integrate(start, 0.0, 0.5);
    const auto [u, v] = errors(r.final);
    const auto [u0, v0] = errors(plain.final);
    std::cout << "INFO standing wave " << method->name() << ": "
              << r.final.mesh->nodes.size() << " nodes, errors " << u << ", "
              << v << "; starting mesh " << start->mesh()->nodes.size()
              << " nodes " << u0 << ", " << v0 << " (|u_t| up to " << w
              << ")\n";
    check(r.final.time == 0.5, "reaches t1");
    check(u < 0.05, "u stays accurate");
    check(v < 0.5 * w, "u_t is carried over (lost, it would be off by ~w)");
  }
}

// --- the node budget and invalid input -------------------------------------

void node_budget() {
  const auto start = std::make_shared<const fem::MeshedProblem>(
      unit_square_mesh(0.1), bump_problem());
  const std::size_t budget = start->mesh()->nodes.size() + 60;
  const AdaptiveIntegrator adaptive(
      integrator(std::make_shared<solve::ImplicitEuler>(), 1.0 / 32),
      {refinement()});
  const auto r = adaptive.integrate(start, 0.0, 0.5,
                                    {.every = {.steps = 2}, .max_nodes = budget});
  check(r.final.time == 0.5, "reaches t1 after the budget stopped adapting");
  std::size_t at = 0;
  for (std::size_t k = 0; k < r.pieces.size(); ++k)
    if (r.pieces[k].adaptation &&
        r.pieces[k].adaptation->outcome == AdaptationOutcome::node_budget)
      at = k + 1;
  check(at > 0, "the budget was hit");
  check(at + 1 == r.pieces.size(), "then one piece to t1");
  check(r.pieces.back().t_end == 0.5, "the last piece ends at t1");
  check(r.final.mesh->nodes.size() <= budget, "never above the budget");
  check(r.report.metrics.at("max_nodes_used") <= static_cast<double>(budget),
        "max_nodes_used");
  check(!r.report.messages.empty() &&
            r.report.messages[0].find("node budget") != std::string::npos,
        "the report says so");
  check(r.report.metrics.at("steps") == 16.0, "every step taken");
}

void rejects_bad_input() {
  const auto problem = std::make_shared<const fem::MeshedProblem>(
      test::unit_square(4), decay_problem());
  const auto i = integrator(std::make_shared<solve::ImplicitEuler>(), 0.1);
  expect_throws<std::invalid_argument>(
      [&] { (void)AdaptiveIntegrator(i, {}); }, "at least one adaptation");
  expect_throws<std::invalid_argument>(
      [&] { (void)AdaptiveIntegrator(i, {nullptr}); }, "adaptation 0 is null");
  expect_throws<std::invalid_argument>(
      [&] { (void)AdaptiveIntegrator(i, {refinement(), refinement()}); },
      "at most one adaptation may change the mesh");
  const AdaptiveIntegrator adaptive(i, {refinement()});
  const AdaptiveIntegrator recorder(i, {std::make_shared<Recorder>()});
  const AdaptiveTimeTarget good{.every = {.interval = 0.5}};
  const auto fails = [&](const AdaptiveTimeTarget &target,
                         std::string_view fragment) {
    expect_throws<std::invalid_argument>(
        [&] { (void)adaptive.integrate(problem, 0.0, 1.0, target); },
        fragment);
  };
  fails({}, "the schedule needs an interval, a number of steps or both");
  fails({.every = {.interval = 0.0}}, "every.interval must be finite");
  fails({.every = {.interval = std::numeric_limits<double>::infinity()}},
        "every.interval must be finite");
  fails({.every = {.steps = 0}}, "every.steps must be at least 1");
  fails({.every = {.steps = 1}, .tolerance = {.absolute = -1.0}},
        "tolerance.absolute must be finite");
  fails({.every = {.steps = 1}, .tolerance = {.relative = 0.0}},
        "tolerance.relative must be finite");
  fails({.every = {.steps = 1}, .max_nodes = 0}, "max_nodes must be at least 1");
  fails({.every = {.steps = 1}, .max_nodes = 3}, "the starting mesh has 25");
  expect_throws<std::invalid_argument>(
      [&] {
        (void)recorder.integrate(problem, 0.0, 1.0,
                                 {.every = {.steps = 1},
                                  .tolerance = {.absolute = 1.0}});
      },
      "a tolerance needs an adaptation that estimates");
  expect_throws<std::invalid_argument>(
      [&] { (void)adaptive.integrate(nullptr, 0.0, 1.0, good); },
      "the problem is null");
  expect_throws<std::invalid_argument>(
      [&] { (void)adaptive.integrate(problem, 1.0, 1.0, good); },
      "t1 must be greater than t0");
  expect_throws<std::invalid_argument>(
      [&] {
        (void)adaptive.integrate(problem, 0.0, 1.0, good,
                                 {.snapshot_every = 2});
      },
      "snapshot_every is not supported");
  expect_throws<std::invalid_argument>(
      [&] {
        (void)adaptive.integrate(problem, 0.0, 1.0, good,
                                 {.snapshot_times = {0.5, 0.25}});
      },
      "strictly increasing");
  expect_throws<std::invalid_argument>(
      [&] {
        (void)adaptive.integrate(problem, 0.0, 1.0, good,
                                 {.snapshot_times = {1.5}});
      },
      "not within [t0, t1]");
}

} // namespace

int main() {
  return test::run({
      {"a moving bump: refined where it is, better than uniform",
       moving_bump},
      {"after a transfer, implicit Euler agrees with a fresh run",
       transfer_error_and_damping},
      {"adaptation times follow the interval and the step count",
       follows_schedules},
      {"a target that never refines equals one integrator run",
       never_refining_equals_plain_run},
      {"a wave equation transfers its velocities", wave_transfers_velocities},
      {"a standing wave on adapted meshes (measured)", standing_wave},
      {"the node budget stops adapting, not the run", node_budget},
      {"invalid configurations and targets throw", rejects_bad_input},
  });
}
