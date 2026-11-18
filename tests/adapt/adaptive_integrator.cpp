// The adaptive loop for time-dependent problems. A steep bump moving
// through a heat equation (manufactured solution): the adaptive run
// refines where the bump is and ends with a clearly smaller energy error
// than uniform refinement with at least as many nodes. Adaptation times
// follow the interval and step rules exactly (whichever comes first); a
// run that never refines equals one Integrator run that lands on the same
// times, bit for bit for the first-order methods; a wave equation (time
// order 2) runs with its velocities transferred; the node budget stops
// adapting but not the run, and so does max_refinements; a snapshot at an
// adaptation time adds no step; the time derivatives an estimator gets
// are checked, also for boundary data whose rate is not finite at t0;
// invalid targets throw.

#include "adapt/estimator_support.hpp"
#include "solve/solve_support.hpp"
#include "support/check.hpp"
#include "support/shapes.hpp"

#include "pde2d/adapt/adaptive_integrator.hpp"
#include "pde2d/adapt/dorfler_marking.hpp"
#include "pde2d/adapt/kelly_estimator.hpp"
#include "pde2d/adapt/mesh_refinement.hpp"
#include "pde2d/adapt/residual_estimator.hpp"
#include "pde2d/fem/assembler.hpp"
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
#include "pde2d/solve/step_controller.hpp"

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
  return {
      .equation = {.a_xx = Expression("-1"),
                   .a_yy = Expression("-1"),
                   .m1 = Expression("1"),
                   .f = Expression(f)},
      .boundary_conditions = {{"boundary",
                               {.alpha = Expression("1"), .g = Expression(u)}}},
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
std::pair<double, double> densities(const mesh::Mesh &m, Point p, double radius,
                                    double outside) {
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
      integrator(std::make_shared<solve::ImplicitEuler>(), dt), {refinement()});
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
    uniform =
        std::make_shared<const mesh::Mesh>(nvb.refine(*uniform, all).mesh);
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
  std::cout << "INFO node density near the bump " << dense << ", far " << sparse
            << '\n';
  check(dense > 8 * sparse, "refined where the bump is");

  // The midpoint method does not damp stiff components, so the
  // interpolation error each transfer leaves on the new mesh's fine modes
  // stays (measured, not a requirement; see adaptive_integrator.hpp).
  const auto midpoint =
      AdaptiveIntegrator(
          integrator(std::make_shared<solve::ImplicitMidpoint>(), dt),
          {refinement()})
          .integrate(start, 0.0, t1,
                     {.every = {.steps = 2}, .max_nodes = 6000});
  const auto midpoint_uniform =
      integrator(std::make_shared<solve::ImplicitMidpoint>(), dt)
          .integrate(std::make_shared<const fem::MeshedProblem>(uniform,
                                                                bump_problem()),
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
  const Point centroid =
      (1.0 / 3) *
      (before->nodes[tri[0]] + before->nodes[tri[1]] + before->nodes[tri[2]]);
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
  for (const std::size_t damping : {0u, 2u}) {
    for (const auto &method :
         std::vector<std::shared_ptr<const solve::TimeMethod>>{
             std::make_shared<solve::ImplicitEuler>(),
             std::make_shared<solve::ImplicitMidpoint>()}) {
      if (damping > 0 && method->name() == "implicit_euler")
        continue;
      const auto r =
          AdaptiveIntegrator(
              integrator(method, dt), {uniform_refinement},
              {.damping_steps = damping,
               .damping_method = std::make_shared<solve::ImplicitEuler>()})
              .integrate(start, 0.0, 0.5,
                         {.every = {.steps = 2}, .max_nodes = 1000},
                         {.snapshot_times = times});
      check(r.report.metrics.at("refinements") == 1.0, "one refinement");
      check(r.pieces[0].adaptation->time == 2 * dt, "at t = 1/32");
      const auto fresh =
          integrator(method, dt)
              .integrate(std::make_shared<const fem::MeshedProblem>(
                             r.final.mesh, bump_problem()),
                         0.0, 0.5, {.snapshot_times = times});
      for (std::size_t k = 0; k < times.size(); ++k) {
        const double transferred = bump_error(r.snapshots[k]);
        const double direct = bump_error(fresh.snapshots[k]);
        std::cout << "INFO " << method->name() << " (" << damping
                  << " damping steps) t = " << times[k] << ": transferred "
                  << transferred << ", fresh " << direct << '\n';
        if (method->name() == "implicit_euler")
          near(transferred, direct, 1e-4 * direct,
               "Euler: the transfer is forgotten");
        else if (damping > 0)
          check(transferred < 1.001 * direct,
                "damping steps: no worse than a run on the refined mesh from "
                "t0");
      }
    }
  }
}

// The moving bump with the midpoint method, adapting every 4 steps of
// 1/128 (so the midpoint method takes about half the steps even with
// damping steps): without damping steps each transfer's error stays and
// the adaptive run is worse than uniform refinement; with 2 steps of
// implicit Euler after each transfer it is clearly better than both
// (measured: 3532 nodes, error 0.325 without; 2514 nodes, 0.0877 with;
// uniform 2927 nodes, 0.262). With 0 damping steps the run is unchanged
// bit for bit, whatever the damping method.
void damping_steps_after_transfers() {
  const double dt = 1.0 / 128, t1 = 0.5;
  const auto start = std::make_shared<const fem::MeshedProblem>(
      unit_square_mesh(0.1), bump_problem());
  const auto midpoint = std::make_shared<solve::ImplicitMidpoint>();
  const auto euler = std::make_shared<solve::ImplicitEuler>();
  const AdaptiveTimeTarget target{.every = {.steps = 4}, .max_nodes = 6000};
  const auto plain =
      AdaptiveIntegrator(integrator(midpoint, dt), {refinement()})
          .integrate(start, 0.0, t1, target);
  const auto off =
      AdaptiveIntegrator(integrator(midpoint, dt), {refinement()},
                         {.damping_steps = 0, .damping_method = euler})
          .integrate(start, 0.0, t1, target);
  check(off.final.values == plain.final.values &&
            off.pieces.size() == plain.pieces.size() &&
            off.report.metrics.at("damping_steps") == 0.0,
        "0 damping steps: unchanged bit for bit");
  const auto damped =
      AdaptiveIntegrator(integrator(midpoint, dt), {refinement()},
                         {.damping_steps = 2, .damping_method = euler})
          .integrate(start, 0.0, t1, target);
  check(damped.final.time == t1, "reaches t1");

  // Uniform refinement to at least as many nodes as the damped run.
  std::shared_ptr<const mesh::Mesh> uniform = start->mesh();
  const mesh::Remesher nvb(std::make_shared<mesh::NewestVertexBisection>());
  while (uniform->nodes.size() < damped.final.mesh->nodes.size()) {
    std::vector<std::size_t> all(uniform->triangles.size());
    for (std::size_t t = 0; t < all.size(); ++t)
      all[t] = t;
    uniform =
        std::make_shared<const mesh::Mesh>(nvb.refine(*uniform, all).mesh);
  }
  const auto uniform_run =
      integrator(midpoint, dt)
          .integrate(std::make_shared<const fem::MeshedProblem>(uniform,
                                                                bump_problem()),
                     0.0, t1);
  const double plain_error = bump_error(plain.final);
  const double damped_error = bump_error(damped.final);
  const double uniform_error = bump_error(uniform_run.final);
  std::cout << "INFO midpoint, every 4 steps of 1/128: no damping "
            << plain.final.mesh->nodes.size() << " nodes, error " << plain_error
            << "; 2 damping steps of implicit Euler "
            << damped.final.mesh->nodes.size() << " nodes, error "
            << damped_error << " (" << damped.report.metrics.at("damping_steps")
            << " of " << damped.report.metrics.at("steps")
            << " steps); uniform " << uniform->nodes.size() << " nodes, error "
            << uniform_error << '\n';
  check(damped_error < 0.5 * plain_error, "clearly better than undamped");
  check(damped_error < 0.5 * uniform_error,
        "clearly better than uniform with at least as many nodes");

  // The pieces: after every refinement a damping piece of 2 steps, then
  // the midpoint method to the adaptation time (4 steps after the last).
  const double refinements = damped.report.metrics.at("refinements");
  check(damped.report.metrics.at("damping_steps") == 2 * refinements,
        "2 damping steps per refinement");
  std::size_t damping_pieces = 0;
  for (std::size_t k = 0; k < damped.pieces.size(); ++k) {
    const IntegrationPiece &piece = damped.pieces[k];
    const bool after_refinement =
        k > 0 && damped.pieces[k - 1].adaptation &&
        damped.pieces[k - 1].adaptation->outcome == AdaptationOutcome::refined;
    check(piece.damping == after_refinement,
          "a damping piece exactly after each refinement");
    if (!piece.damping)
      continue;
    ++damping_pieces;
    check(piece.report.metrics.at("steps") == 2.0, "two damping steps");
    check(!piece.adaptation, "no adaptation after the damping steps");
    check(k + 1 < damped.pieces.size() &&
              damped.pieces[k + 1].problem == piece.problem &&
              damped.pieces[k + 1].report.metrics.at("steps") == 2.0 &&
              (damped.pieces[k + 1].adaptation ||
               damped.pieces[k + 1].t_end == t1),
          "the midpoint method takes the other 2 steps to the adaptation "
          "time");
  }
  check(static_cast<double>(damping_pieces) == refinements,
        "one damping piece per refinement");
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

// An adaptation that changes nothing and records when it was asked and
// what it was given: in adapt(), or, when it estimates (an estimate of
// zero), in estimate(), the state an estimator gets.
class Recorder final : public Adaptation {
public:
  struct Call {
    double time;
    std::vector<double> values, u_t, u_tt;
  };
  explicit Recorder(bool estimates = false) : estimates_(estimates) {}
  std::string name() const override { return "recorder"; }
  AdaptationCapabilities capabilities() const override {
    return {.estimates_error = estimates_};
  }
  std::optional<ErrorEstimate>
  estimate(const SolvedState &state) const override {
    if (!estimates_)
      return std::nullopt;
    record(state);
    ErrorEstimate estimate;
    estimate.indicators.assign(state.problem.mesh()->triangles.size(), 0.0);
    estimate.report.stage = "recorder";
    return estimate;
  }
  AdaptationResult adapt(const SolvedState &state,
                         const ErrorEstimate *) const override {
    if (!estimates_)
      record(state);
    return {};
  }
  mutable std::vector<Call> calls;

private:
  void record(const SolvedState &state) const {
    calls.push_back(
        {state.solution.time,
         state.solution.values,
         {state.derivatives.u_t.begin(), state.derivatives.u_t.end()},
         {state.derivatives.u_tt.begin(), state.derivatives.u_tt.end()}});
  }
  bool estimates_;
};

struct Schedule {
  AdaptationSchedule every;
  double dt;
  double t1;
  std::vector<double> snapshots;
  std::vector<double> times;      // expected adaptation times
  std::vector<std::size_t> steps; // expected steps up to each
  std::size_t total_steps;
};

void follows_schedules() {
  const auto problem = std::make_shared<const fem::MeshedProblem>(
      test::unit_square(4), decay_problem());
  const std::vector<Schedule> cases{
      // Interval only: every 1/8, two steps of 1/16 each.
      {{.interval = 0.125},
       1.0 / 16,
       1.0,
       {},
       {0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875},
       {2, 4, 6, 8, 10, 12, 14},
       16},
      // Steps only: every 3 steps of 1/16; the last piece has one step.
      {{.steps = 3},
       1.0 / 16,
       1.0,
       {},
       {3.0 / 16, 6.0 / 16, 9.0 / 16, 12.0 / 16, 15.0 / 16},
       {3, 6, 9, 12, 15},
       16},
      // Both, the steps first (3/16 < 1/4).
      {{.interval = 0.25, .steps = 3},
       1.0 / 16,
       1.0,
       {},
       {3.0 / 16, 6.0 / 16, 9.0 / 16, 12.0 / 16, 15.0 / 16},
       {3, 6, 9, 12, 15},
       16},
      // Both, the interval first (1/8 < 3/16).
      {{.interval = 0.125, .steps = 3},
       1.0 / 16,
       1.0,
       {},
       {0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875},
       {2, 4, 6, 8, 10, 12, 14},
       16},
      // Both, each winning once: snapshot landings add steps, so 4 steps
      // come before 0.2 (0.02, 0.04, 0.06, 0.15); then the interval
      // (0.24, 0.33, 0.35 and 0.44, 0.53, 0.55); the last piece to 0.6
      // is one shortened step.
      {{.interval = 0.2, .steps = 4},
       0.09,
       0.6,
       {0.02, 0.04, 0.06},
       {0.15, 0.35, 0.55},
       {4, 7, 10},
       11},
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
      check(r.report.metrics.at("steps") == static_cast<double>(c.total_steps),
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
      // the same times (the interval's adaptation times as snapshot
      // times; those of the step rule are not landings, the pieces
      // continue the stretch of steps there) gives the same values, bit
      // for bit.
      std::vector<double> lands = c.snapshots;
      for (std::size_t k = 0; k < c.times.size(); ++k) {
        const std::size_t since = c.steps[k] - (k == 0 ? 0 : c.steps[k - 1]);
        if (!(c.every.steps && since == *c.every.steps))
          lands.push_back(recorder->calls[k].time);
      }
      std::ranges::sort(lands);
      const auto whole =
          integrator(method, c.dt)
              .integrate(problem, 0.0, c.t1,
                         {.keep_step_history = true, .snapshot_times = lands});
      check(whole.final.values == r.final.values,
            "the same final values, bit for bit");
      check(whole.history.size() == r.history.size(), "the same steps");
      for (std::size_t k = 0; k < r.history.size(); ++k)
        check(whole.history[k].t == r.history[k].t &&
                  whole.history[k].dt == r.history[k].dt,
              "the same step times");
      for (std::size_t k = 0; k < c.times.size(); ++k) {
        const auto &call = recorder->calls[k];
        // The state at the adaptation time is the long run's there (its
        // first c.steps[k] steps); u_t is the difference quotient of its
        // last step, from the long run's state one step before.
        const auto at =
            integrator(method, c.dt)
                .integrate(problem, 0.0, c.t1, {.snapshot_times = lands},
                           nullptr, c.steps[k]);
        check(at.final.time == call.time && at.final.values == call.values,
              "the state at the adaptation time");
        check(call.u_tt.empty(), "no u_tt for time order 1");
        check(call.u_t.size() == call.values.size(), "u_t at every node");
        const double h = whole.history[c.steps[k] - 1].dt;
        const auto before =
            integrator(method, c.dt)
                .integrate(problem, 0.0, c.t1, {.snapshot_times = lands},
                           nullptr, c.steps[k] - 1);
        for (std::size_t v = 0; v < call.u_t.size(); ++v)
          near(call.u_t[v], (call.values[v] - before.final.values[v]) / h,
               1e-12, "u_t = (u(t) - u(t - dt)) / dt");
      }
    }
  }
}

// An adaptation time that rounds next to a snapshot time is that time: dt
// = 0.1 and an interval of 0.1, snapshots at 0.3 (0.2 + 0.1 is just above
// it) and 0.8 (0.7 + 0.1 just below): ten steps of 0.1, none of rounding
// size, and the snapshots at their exact times.
void snapshot_at_an_adaptation_time() {
  const auto problem = std::make_shared<const fem::MeshedProblem>(
      test::unit_square(4), decay_problem());
  check(0.2 + 0.1 > 0.3 && 0.7 + 0.1 < 0.8, "the sums round off both sides");
  for (const auto &method :
       std::vector<std::shared_ptr<const solve::TimeMethod>>{
           std::make_shared<solve::ImplicitEuler>(),
           std::make_shared<solve::ImplicitMidpoint>()}) {
    const auto recorder = std::make_shared<Recorder>();
    const auto r =
        AdaptiveIntegrator(integrator(method, 0.1), {recorder})
            .integrate(
                problem, 0.0, 1.0, {.every = {.interval = 0.1}},
                {.keep_step_history = true, .snapshot_times = {0.3, 0.8}});
    check(r.history.size() == 10 && r.report.metrics.at("steps") == 10.0,
          "ten steps");
    for (std::size_t k = 0; k < r.history.size(); ++k) {
      near(r.history[k].t, 0.1 * static_cast<double>(k + 1), 1e-15,
           "the step times");
      near(r.history[k].dt, 0.1, 1e-15, "steps of 0.1");
    }
    check(r.snapshots.size() == 2 && r.snapshots[0].time == 0.3 &&
              r.snapshots[1].time == 0.8,
          "the snapshots at their exact times");
    check(recorder->calls.size() == 9, "nine adaptation times");
    check(recorder->calls[2].time == 0.3 && recorder->calls[7].time == 0.8,
          "the adaptation times are the snapshot times");
    check(r.final.time == 1.0, "reaches t1");
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
    const auto whole =
        integrator(method, 0.1)
            .integrate(problem, 0.0, 1.0,
                       {.snapshot_times = {0.25, 0.3, 0.5, 0.75}});
    check(whole.final.values == r.final.values, "bit for bit");
    check(r.snapshots.size() == 1 &&
              r.snapshots[0].values == whole.snapshots[1].values,
          "the snapshot at 0.3");
    check(r.report.metrics.at("tolerance") == 1e300, "the tolerance");
  }
}

// The anchor goes across every piece on the same problem: adapting every
// 3 steps of 0.1 (not exact in binary) without refining takes exactly the
// step times of one integrate() call that lands nowhere, (k + 1) * 0.1
// from t0, also where a new stretch from the piece's start would round
// differently (0.30000000000000004 + 0.1 != 0.4).
void anchor_across_pieces() {
  const auto problem = std::make_shared<const fem::MeshedProblem>(
      test::unit_square(4), decay_problem());
  for (const auto &method :
       std::vector<std::shared_ptr<const solve::TimeMethod>>{
           std::make_shared<solve::ImplicitEuler>(),
           std::make_shared<solve::ImplicitMidpoint>()}) {
    const auto whole =
        integrator(method, 0.1)
            .integrate(problem, 0.0, 2.0, {.keep_step_history = true});
    const auto recorder = std::make_shared<Recorder>(true);
    const auto r = AdaptiveIntegrator(integrator(method, 0.1), {recorder})
                       .integrate(problem, 0.0, 2.0, {.every = {.steps = 3}},
                                  {.keep_step_history = true});
    check(recorder->calls.size() == 6 && r.pieces.size() == 7,
          "adaptation times after 3, 6, ..., 18 steps");
    check(r.history.size() == whole.history.size(), "as many steps");
    bool differs = false;
    for (std::size_t k = 0;
         k < std::min(r.history.size(), whole.history.size()); ++k) {
      check(r.history[k].t == whole.history[k].t &&
                r.history[k].dt == whole.history[k].dt,
            "the step times of one run");
      if (k >= 3)
        differs =
            differs || whole.history[2].t + static_cast<double>(k - 2) * 0.1 !=
                           whole.history[k].t;
    }
    check(differs, "a new stretch at each piece would give other times");
    for (std::size_t k = 0; k < recorder->calls.size(); ++k)
      check(recorder->calls[k].time == whole.history[3 * k + 2].t,
            "the adaptation times are the uninterrupted run's");
    check(r.final.values == whole.final.values, "bit for bit");
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
  return {.equation = {.a_xx = Expression("-1"),
                       .a_yy = Expression("-1"),
                       .m1 = Expression("0.1"),
                       .m2 = Expression("1"),
                       .f = Expression(u_tt + " + 0.1*(" + u_t + ")")},
          .boundary_conditions = {{"bottom", dirichlet},
                                  {"left", dirichlet},
                                  {"right",
                                   {.beta_n = Expression("1"),
                                    .g = Expression("cos(2*t) + sin(3*t)")}},
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
    u = std::max(
        u, std::abs(s.values[k] - (p * std::cos(2 * t) + q * std::sin(3 * t))));
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
  // Newmark with 2 damping steps of implicit Euler after each transfer:
  // the linear wave has no transfer error, so the damping steps only add
  // their (first-order) time error (measured).
  {
    const double dt = 1.0 / 64;
    const auto r =
        AdaptiveIntegrator(
            integrator(std::make_shared<solve::Newmark>(), dt), {refinement()},
            {.damping_steps = 2,
             .damping_method = std::make_shared<solve::ImplicitEuler>()})
            .integrate(start, 0.0, 1.0, {.every = {.interval = 0.125}});
    const auto [u, v] = linear_wave_errors(r.final);
    std::cout << "INFO linear wave newmark, 2 damping steps of implicit "
                 "Euler: "
              << r.final.mesh->nodes.size() << " nodes, errors " << u << ", "
              << v << '\n';
    check(r.final.time == 1.0 && u < 0.05 && v < 0.1,
          "the damping steps keep the run close");
  }
}

// Newmark keeps its acceleration across pieces: a target that never
// refines gives one Integrator run that lands on the adaptation times,
// bit for bit (the pieces do not call Newmark's start again).
void newmark_pieces_continue_exactly() {
  mesh::Mesh square = *test::unit_square(4);
  mesh::set_longest_edges_as_refinement_edges(square);
  const auto problem = std::make_shared<const fem::MeshedProblem>(
      std::make_shared<const mesh::Mesh>(std::move(square)),
      linear_wave_problem());
  const auto newmark = integrator(std::make_shared<solve::Newmark>(), 1.0 / 16);
  const auto r = AdaptiveIntegrator(newmark, {refinement()})
                     .integrate(problem, 0.0, 1.0,
                                {.every = {.interval = 0.25},
                                 .tolerance = {.absolute = 1e300}});
  check(r.pieces.size() == 4, "four pieces");
  const auto whole = newmark.integrate(problem, 0.0, 1.0,
                                       {.snapshot_times = {0.25, 0.5, 0.75}});
  check(r.final.values == whole.final.values &&
            r.final.velocities == whole.final.velocities &&
            r.final.accelerations == whole.final.accelerations,
        "bit for bit");
  for (std::size_t k = 1; k < r.pieces.size(); ++k)
    check(r.pieces[k].report.metrics.at("assemblies") ==
              r.pieces[k].report.metrics.at("steps"),
          "no start in a continued piece");
}

// u_tt - (u_xx + u_yy) = 0, u = sin(pi x) sin(pi y) cos(sqrt(2) pi t): a
// measurement. A transfer puts the old mesh's interpolation error into the
// new mesh's fine modes, which a wave keeps (Newmark and the midpoint
// method do not damp them; implicit Euler does): u stays accurate, u_t at
// the nodes less so than on the starting mesh.
void standing_wave() {
  const fem::BoundaryCondition zero{.alpha = Expression("1"),
                                    .g = Expression("0")};
  const fem::Problem wave{.equation = {.a_xx = Expression("-1"),
                                       .a_yy = Expression("-1"),
                                       .m2 = Expression("1")},
                          .boundary_conditions = {{"boundary", zero}},
                          .initial = {.u = Expression("sin(pi*x)*sin(pi*y)"),
                                      .u_t = Expression("0")}};
  const auto start =
      std::make_shared<const fem::MeshedProblem>(unit_square_mesh(0.25), wave);
  const double w = std::sqrt(2.0) * pi;
  const auto errors = [&](const solve::Solution &s) {
    double u = 0, v = 0;
    for (std::size_t k = 0; k < s.mesh->nodes.size(); ++k) {
      const Point p = s.mesh->nodes[k];
      const double shape = std::sin(pi * p.x) * std::sin(pi * p.y);
      u = std::max(u, std::abs(s.values[k] - shape * std::cos(w * s.time)));
      v = std::max(
          v, std::abs(s.velocities[k] + shape * w * std::sin(w * s.time)));
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
  // Newmark with 1 or 2 damping steps of implicit Euler after each
  // transfer (measured). Newmark continues after them from the
  // acceleration of the last damping step: no start(), so nothing is
  // assembled at the piece's first time.
  for (const std::size_t damping : {1u, 2u}) {
    const double dt = 1.0 / 128;
    const AdaptiveIntegrator adaptive(
        integrator(std::make_shared<solve::Newmark>(), dt), {refinement()},
        {.damping_steps = damping,
         .damping_method = std::make_shared<solve::ImplicitEuler>()});
    const auto r =
        adaptive.integrate(start, 0.0, 0.5, {.every = {.interval = 0.0625}});
    const auto [u, v] = errors(r.final);
    std::cout << "INFO standing wave newmark, " << damping
              << " damping steps of implicit Euler: "
              << r.final.mesh->nodes.size() << " nodes, errors " << u << ", "
              << v << '\n';
    check(r.final.time == 0.5, "reaches t1");
    check(r.report.metrics.at("damping_steps") ==
              static_cast<double>(damping) * r.report.metrics.at("refinements"),
          "damping steps after each refinement");
    for (std::size_t k = 1; k < r.pieces.size(); ++k)
      if (r.pieces[k - 1].damping)
        check(r.pieces[k].report.metrics.at("assemblies") ==
                  r.pieces[k].report.metrics.at("steps"),
              "Newmark continues from the damping steps' acceleration");
  }
}

// The derivatives an estimator gets at an adaptation time one step after
// t0, for a wave (time order 2) with boundary data that moves: u_t is the
// integrator's velocities (at the Dirichlet nodes the rate at t), u_tt
// the difference quotient of the velocities at the free nodes and the
// acceleration of the boundary data at the Dirichlet nodes. Checked
// against one plain step and the initial and boundary data written out.
fem::Problem moving_wave_problem(const std::string &g) {
  const fem::BoundaryCondition dirichlet{.alpha = Expression("1"),
                                         .g = Expression(g)};
  return {.equation = {.a_xx = Expression("-1"),
                       .a_yy = Expression("-1"),
                       .m2 = Expression("1")},
          .boundary_conditions = {{"bottom", dirichlet},
                                  {"right", dirichlet},
                                  {"top", dirichlet},
                                  {"left", dirichlet}},
          .initial = {.u = Expression("y"), .u_t = Expression("sin(x) + y")}};
}

void second_order_derivatives() {
  const auto problem = std::make_shared<const fem::MeshedProblem>(
      test::unit_square(4), moving_wave_problem("x*t^2 + y"));
  const double dt = 0.1;
  const auto plain =
      integrator(std::make_shared<solve::ImplicitEuler>(), dt)
          .integrate(problem, 0.0, dt, {.keep_before_last_step = true});
  const auto &free_nodes = problem->free_nodes();
  const auto &nodes = problem->mesh()->nodes;
  // The state before the step is the initial one, in the free unknowns.
  const solve::FreeState &before = *plain.before_last_step;
  check(before.time == 0.0 && before.velocities.size() == free_nodes.size(),
        "the free velocities at t0");
  for (std::size_t i = 0; i < free_nodes.size(); ++i) {
    const Point p = nodes[free_nodes[i]];
    near(before.velocities[i], std::sin(p.x) + p.y, 1e-15,
         "the initial velocity u_t(x, 0)");
    near(before.values[i], p.y, 1e-15, "the initial value u(x, 0)");
  }

  const auto recorder = std::make_shared<Recorder>(true);
  const auto r =
      AdaptiveIntegrator(
          integrator(std::make_shared<solve::ImplicitEuler>(), dt), {recorder})
          .integrate(problem, 0.0, 2 * dt, {.every = {.steps = 1}});
  check(recorder->calls.size() == 1 && recorder->calls[0].time == dt,
        "one adaptation, after the first step");
  const auto &call = recorder->calls[0];
  check(call.values == plain.final.values, "the solution after one step");
  check(call.u_t == plain.final.velocities, "u_t: the velocities");
  std::vector<bool> dirichlet(nodes.size(), true);
  for (const std::size_t v : free_nodes)
    dirichlet[v] = false;
  std::size_t dirichlet_nodes = 0;
  for (std::size_t v = 0; v < nodes.size(); ++v) {
    const Point p = nodes[v];
    if (dirichlet[v]) {
      ++dirichlet_nodes;
      near(call.u_t[v], 2 * p.x * dt, 1e-14, "Dirichlet: g_t(t)");
      near(call.u_tt[v], 2 * p.x, 1e-14, "Dirichlet: g_tt(t)");
    } else {
      near(call.u_tt[v],
           (plain.final.velocities[v] - (std::sin(p.x) + p.y)) / dt, 1e-12,
           "free: (u_t(t) - u_t(t - dt)) / dt");
    }
  }
  check(dirichlet_nodes == 16, "the boundary of the 4 x 4 square");
  check(r.final.time == 2 * dt, "reaches t1");
}

// A wave at rest with u = sqrt(t) on the boundary, whose rate is not finite
// at t0 = 0: implicit Euler never evaluates it there, and neither may the
// adaptive run, also when it adapts after the first step (u_tt at the
// Dirichlet nodes is g_tt(t), at t > t0).
void boundary_rate_not_finite_at_t0() {
  fem::Problem wave = moving_wave_problem("sqrt(t)");
  wave.initial = {.u = Expression("0"), .u_t = Expression("0")};
  const auto problem =
      std::make_shared<const fem::MeshedProblem>(test::unit_square(4), wave);
  const double dt = 0.1;
  const auto plain = integrator(std::make_shared<solve::ImplicitEuler>(), dt)
                         .integrate(problem, 0.0, dt);
  for (const AdaptationSchedule &every :
       {AdaptationSchedule{.interval = 1.0}, AdaptationSchedule{.steps = 1},
        AdaptationSchedule{.steps = 3}}) {
    const auto recorder = std::make_shared<Recorder>();
    const auto r = AdaptiveIntegrator(
                       integrator(std::make_shared<solve::ImplicitEuler>(), dt),
                       {recorder})
                       .integrate(problem, 0.0, dt, {.every = every});
    check(recorder->calls.empty(), "no adaptation at t1");
    check(r.final.values == plain.final.values &&
              r.final.velocities == plain.final.velocities,
          "the plain run's step");
  }
  const auto recorder = std::make_shared<Recorder>();
  const auto r =
      AdaptiveIntegrator(
          integrator(std::make_shared<solve::ImplicitEuler>(), dt), {recorder})
          .integrate(problem, 0.0, 2 * dt, {.every = {.steps = 1}});
  check(recorder->calls.size() == 1, "an adaptation after the first step");
  const auto &call = recorder->calls[0];
  for (const auto &d : problem->dirichlet_nodes())
    near(call.u_tt[d.node], -0.25 * std::pow(dt, -1.5), 1e-12,
         "u_tt = g_tt(t) at a Dirichlet node");
  check(r.final.time == 2 * dt, "reaches t1");
}

// --- the node budget and invalid input -------------------------------------

void node_budget() {
  const auto start = std::make_shared<const fem::MeshedProblem>(
      unit_square_mesh(0.1), bump_problem());
  const std::size_t budget = start->mesh()->nodes.size() + 60;
  const AdaptiveIntegrator adaptive(
      integrator(std::make_shared<solve::ImplicitEuler>(), 1.0 / 32),
      {refinement()});
  const auto r = adaptive.integrate(
      start, 0.0, 0.5, {.every = {.steps = 2}, .max_nodes = budget});
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

// max_refinements: after the second refinement adapting stops, and the
// rest is one piece to t1.
void refinement_limit() {
  const auto start = std::make_shared<const fem::MeshedProblem>(
      unit_square_mesh(0.1), bump_problem());
  const AdaptiveIntegrator adaptive(
      integrator(std::make_shared<solve::ImplicitEuler>(), 1.0 / 32),
      {refinement()});
  const auto r = adaptive.integrate(
      start, 0.0, 0.5, {.every = {.steps = 2}, .max_refinements = 2});
  check(r.final.time == 0.5, "reaches t1");
  check(r.report.metrics.at("refinements") == 2.0, "two refinements");
  std::size_t second = 0, refined = 0;
  for (std::size_t k = 0; k < r.pieces.size(); ++k)
    if (r.pieces[k].adaptation &&
        r.pieces[k].adaptation->outcome == AdaptationOutcome::refined &&
        ++refined == 2)
      second = k;
  check(refined == 2 && second + 2 == r.pieces.size(), "then one piece to t1");
  check(!r.pieces.back().adaptation && r.pieces.back().t_end == 0.5,
        "the last piece ends at t1 without adapting");
  check(r.report.metrics.at("adaptation_times") ==
            static_cast<double>(second + 1),
        "no adaptation time after the limit");
  check(r.report.metrics.at("steps") == 16.0, "every step taken");
  check(!r.report.messages.empty() &&
            r.report.messages[0].find("refinement limit") != std::string::npos,
        "the report says so");
}

void rejects_bad_input() {
  const auto problem = std::make_shared<const fem::MeshedProblem>(
      test::unit_square(4), decay_problem());
  const auto i = integrator(std::make_shared<solve::ImplicitEuler>(), 0.1);
  expect_throws<std::invalid_argument>([&] { (void)AdaptiveIntegrator(i, {}); },
                                       "at least one adaptation");
  expect_throws<std::invalid_argument>(
      [&] { (void)AdaptiveIntegrator(i, {nullptr}); }, "adaptation 0 is null");
  expect_throws<std::invalid_argument>(
      [&] { (void)AdaptiveIntegrator(i, {refinement(), refinement()}); },
      "at most one adaptation may change the mesh");
  expect_throws<std::invalid_argument>(
      [&] {
        (void)AdaptiveIntegrator(i, {refinement()}, {.damping_steps = 2});
      },
      "2 damping steps need a damping method");
  // The decay problem has time order 1, which Newmark does not support.
  expect_throws<std::invalid_argument>(
      [&] {
        (void)AdaptiveIntegrator(
            i, {refinement()},
            {.damping_steps = 1,
             .damping_method = std::make_shared<solve::Newmark>()})
            .integrate(problem, 0.0, 1.0, {.every = {.interval = 0.5}});
      },
      "the damping method newmark does not support time order 1");
  const AdaptiveIntegrator adaptive(i, {refinement()});
  const AdaptiveIntegrator recorder(i, {std::make_shared<Recorder>()});
  const AdaptiveTimeTarget good{.every = {.interval = 0.5}};
  const auto fails = [&](const AdaptiveTimeTarget &target,
                         std::string_view fragment) {
    expect_throws<std::invalid_argument>(
        [&] { (void)adaptive.integrate(problem, 0.0, 1.0, target); }, fragment);
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
  fails({.every = {.steps = 1}, .max_nodes = 0},
        "max_nodes must be at least 1");
  fails({.every = {.steps = 1}, .max_nodes = 3}, "the starting mesh has 25");
  fails({.every = {.steps = 1}, .max_refinements = 0},
        "max_refinements must be at least 1");
  // t + interval rounds to t at t0 = 1e6 (the spacing of doubles there is
  // 1.2e-10).
  expect_throws<std::runtime_error>(
      [&] {
        (void)recorder.integrate(problem, 1e6, 1e6 + 1.0,
                                 {.every = {.interval = 1e-12}});
      },
      "the adaptation interval 1e-12 is too small to change t = 1e+06");
  expect_throws<std::invalid_argument>(
      [&] {
        (void)recorder.integrate(
            problem, 0.0, 1.0,
            {.every = {.steps = 1}, .tolerance = {.absolute = 1.0}});
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

// --- the state after damping steps -----------------------------------------

// Refines every triangle with a positive indicator.
std::shared_ptr<const Adaptation> uniform_refinement() {
  return std::make_shared<MeshRefinement>(
      std::make_shared<ResidualEstimator>(),
      std::make_shared<DorflerMarking>(DorflerMarkingOptions{.theta = 1.0}),
      std::make_shared<mesh::Remesher>(
          std::make_shared<mesh::NewestVertexBisection>()));
}

// u_tt - (u_xx + u_yy) = t on the unit square with u_n = 0, from rest:
// the spatially constant mode u = t^3 / 6, u_t = t^2 / 2, u_tt = t, which
// a transfer keeps exactly and the methods follow up to rounding (Newmark
// and implicit Euler give u_tt = t at the end of a step, Newmark u_t
// exactly; the review's case).
fem::Problem constant_mode_problem() {
  return {.equation = {.a_xx = Expression("-1"),
                       .a_yy = Expression("-1"),
                       .m2 = Expression("1"),
                       .f = Expression("t")},
          .boundary_conditions = {{"boundary", {.beta_n = Expression("1")}}},
          .initial = {.u = Expression("0"), .u_t = Expression("0")}};
}

// Newmark with steps of 0.25, a refinement at 0.25 and then one damping
// step, to 0.5 (or to t1 when it is closer).
AdaptiveIntegrationResult
constant_mode_run(std::shared_ptr<const solve::TimeMethod> method,
                  std::shared_ptr<const solve::TimeMethod> damping, double t1,
                  std::vector<double> snapshots = {}) {
  const auto start = std::make_shared<const fem::MeshedProblem>(
      unit_square_mesh(0.5), constant_mode_problem());
  return AdaptiveIntegrator(
             integrator(std::move(method), 0.25), {uniform_refinement()},
             {.damping_steps = 1, .damping_method = std::move(damping)})
      .integrate(start, 0.0, t1,
                 {.every = {.interval = 0.25}, .max_refinements = 1},
                 {.snapshot_times = std::move(snapshots)});
}

double max_deviation(const std::vector<double> &values, double expected) {
  double deviation = 0.0;
  for (const double v : values)
    deviation = std::max(deviation, std::abs(v - expected));
  return values.empty() ? std::numeric_limits<double>::infinity() : deviation;
}

// After damping steps Newmark continues from the acceleration that holds
// at their end: implicit Euler's velocity quotient (declared), or the
// damping method's own acceleration when it keeps one (Newmark as
// damping method; the quotient there is the average of the two ends,
// 0.375 instead of 0.5). The midpoint method's quotient is the
// acceleration at the middle of the step: refused ahead of Newmark, not
// ahead of a method that keeps none.
void damping_hands_over_the_acceleration() {
  const auto newmark = std::make_shared<solve::Newmark>();
  const auto euler = std::make_shared<solve::ImplicitEuler>();
  const auto midpoint = std::make_shared<solve::ImplicitMidpoint>();
  check(euler->velocity_quotient_is_acceleration(2) &&
            !euler->velocity_quotient_is_acceleration(1) &&
            !midpoint->velocity_quotient_is_acceleration(2) &&
            !newmark->velocity_quotient_is_acceleration(2),
        "the declarations");

  // Implicit Euler: u_tt = 0.5 at 0.5, then a Newmark step to 0.75 adds
  // 0.25 (0.5 + 0.75) / 2 to u_t.
  const auto by_euler = constant_mode_run(newmark, euler, 0.75, {0.5});
  check(by_euler.pieces.size() == 3 && by_euler.pieces[1].damping &&
            by_euler.pieces[1].t_end == 0.5,
        "a refinement at 0.25, one damping step to 0.5");
  const solve::Solution &at_half = by_euler.snapshots.at(0);
  check(at_half.time == 0.5 &&
            max_deviation(at_half.accelerations, 0.5) < 1e-12,
        "Euler: the snapshot at the end of the damping step has u_tt = t");
  check(max_deviation(by_euler.final.velocities,
                      at_half.velocities[0] + 0.15625) < 1e-12 &&
            max_deviation(by_euler.final.accelerations, 0.75) < 1e-12,
        "Newmark continues from u_tt = 0.5");

  // Newmark as damping method keeps its own acceleration: u_t at 0.75 is
  // exact, 0.75^2 / 2 (0.265625 from the quotient 0.375).
  const auto by_newmark = constant_mode_run(newmark, newmark, 0.75, {0.5});
  check(max_deviation(by_newmark.snapshots.at(0).accelerations, 0.5) < 1e-12,
        "Newmark: its own u_tt = t at the end of the damping step");
  check(max_deviation(by_newmark.final.velocities, 0.28125) < 1e-12,
        "Newmark: u_t exact at 0.75");

  // The midpoint method gives no acceleration at the end of its steps.
  for (const char *fragment :
       {"newmark continues after the damping steps from the acceleration",
        "the damping method implicit_midpoint gives none at time order 2"})
    expect_throws<std::invalid_argument>(
        [&] { (void)constant_mode_run(newmark, midpoint, 0.75); }, fragment);
  // A method that keeps none needs none.
  const auto euler_after_midpoint = constant_mode_run(euler, midpoint, 0.75);
  check(euler_after_midpoint.final.time == 0.75 &&
            euler_after_midpoint.pieces[1].damping,
        "midpoint damping steps ahead of implicit Euler");

  // The damping step lands on t1 = 0.5000000002 with its nominal length
  // 0.25: the quotient divides by that, not by t1 - 0.25 (which gives
  // 0.4999999998).
  const double t1 = 0.5000000002;
  const auto landing = constant_mode_run(newmark, euler, t1);
  check(landing.final.time == t1 && landing.pieces.back().damping,
        "the damping step lands on t1");
  check(max_deviation(landing.final.accelerations, t1) < 1e-13,
        "the quotient uses the step's length");
}

// A wave with coefficients that depend on t and moving Dirichlet data: the
// acceleration handed over after an implicit Euler damping step satisfies
// M2 u_tt + B u_t + A u = b at its end (assembled here), also where M2
// vanishes there (m2 = (t - 0.5)^2 at t = 0.5, where Newmark's start
// cannot solve for it); a snapshot at that time carries it, and Newmark
// continued from the snapshot equals the adaptive run, bit for bit.
void acceleration_after_damping_satisfies_the_equation() {
  for (const std::string m2 : {"1 + t", "(t - 0.5)^2"}) {
    const fem::BoundaryCondition dirichlet{.alpha = Expression("1"),
                                           .g = Expression("x*t^2 + y")};
    const fem::Problem wave{
        .equation = {.a_xx = Expression("-(1 + t)"),
                     .a_yy = Expression("-(1 + t)"),
                     .m1 = Expression("t"),
                     .m2 = Expression(m2),
                     .f = Expression("x*y*t + 1")},
        .boundary_conditions = {{"boundary", dirichlet}},
        .initial = {.u = Expression("y"), .u_t = Expression("sin(x) + y")}};
    const auto start = std::make_shared<const fem::MeshedProblem>(
        unit_square_mesh(0.25), wave);
    const auto newmark = integrator(std::make_shared<solve::Newmark>(), 0.25);
    const auto r =
        AdaptiveIntegrator(
            newmark, {uniform_refinement()},
            {.damping_steps = 1,
             .damping_method = std::make_shared<solve::ImplicitEuler>()})
            .integrate(start, 0.0, 1.0,
                       {.every = {.interval = 0.25}, .max_refinements = 1},
                       {.snapshot_times = {0.5}});
    check(r.pieces.size() == 3 && r.pieces[1].damping &&
              r.pieces[1].t_end == 0.5 && r.snapshots.size() == 1,
          m2 + ": one damping step to the snapshot at 0.5");
    const auto &problem = r.pieces[1].problem;
    const solve::Solution &s = r.snapshots[0];
    check(s.mesh == problem->mesh() &&
              s.accelerations.size() == s.values.size(),
          m2 + ": the snapshot has accelerations");

    // The residual of the equation at the free nodes, assembled at 0.5.
    const auto assembled = fem::Assembler().assemble(problem, 0.5);
    const auto &free_nodes = problem->free_nodes();
    std::vector<double> u, v, w;
    for (const std::size_t node : free_nodes) {
      u.push_back(s.values[node]);
      v.push_back(s.velocities[node]);
      w.push_back(s.accelerations[node]);
    }
    const auto mw = assembled.mass().free.multiply(w);
    const auto bv = assembled.damping().free.multiply(v);
    const auto au = assembled.stiffness().free.multiply(u);
    const auto b = assembled.right_hand_side();
    double residual = 0.0, scale = 0.0, mass_part = 0.0;
    for (std::size_t i = 0; i < free_nodes.size(); ++i) {
      residual = std::max(residual, std::abs(b[i] - mw[i] - bv[i] - au[i]));
      scale = std::max(scale, std::abs(b[i]) + std::abs(mw[i]) +
                                  std::abs(bv[i]) + std::abs(au[i]));
      mass_part = std::max(mass_part, std::abs(mw[i]));
    }
    std::cout << "INFO " << m2 << ": endpoint residual " << residual << " of "
              << scale << ", |M2 u_tt| " << mass_part << '\n';
    check(residual <= 1e-10 * scale,
          m2 + ": M2 u_tt + B u_t + A u = b at the end of the damping step");
    if (m2 == "1 + t")
      check(residual <= 1e-9 * mass_part,
            "far below the acceleration term (a wrong u_tt would show)");
    const auto &dirichlet_nodes = problem->dirichlet_nodes();
    for (std::size_t k = 0; k < dirichlet_nodes.size(); ++k)
      check(s.accelerations[dirichlet_nodes[k].node] ==
                problem->dirichlet_acceleration(k, 0.5),
            m2 + ": the boundary data's acceleration at the Dirichlet nodes");

    // Newmark continued from the snapshot: no start (it would fail where
    // M2 vanishes), the same run as the adaptive one.
    const auto continued = newmark.integrate(problem, 0.5, 1.0, {}, &s);
    check(continued.report.metrics.at("assemblies") ==
              continued.report.metrics.at("steps"),
          m2 + ": no start");
    check(continued.final.values == r.final.values &&
              continued.final.velocities == r.final.velocities &&
              continued.final.accelerations == r.final.accelerations,
          m2 + ": the adaptive run's continuation, bit for bit");
  }
}

// Changes the step at t = 0.4: a split where only the method changes keeps
// the stretch of steps (an uninterrupted Euler run steps 0.35, 0.4, then
// 0.425; restarting at 0.35 would give 0.39999999999999997 and another
// step of 0.05).
class SteppedController final : public solve::StepController {
public:
  std::string name() const override { return "stepped"; }
  double next_step(double t) const override { return t < 0.4 ? 0.05 : 0.025; }
};

void damping_keeps_the_step_times() {
  mesh::Mesh square = *test::unit_square(4);
  mesh::set_longest_edges_as_refinement_edges(square);
  const auto start = std::make_shared<const fem::MeshedProblem>(
      std::make_shared<const mesh::Mesh>(std::move(square)), decay_problem());
  const auto euler = std::make_shared<solve::ImplicitEuler>();
  const solve::Integrator stepped(euler, std::make_shared<SteppedController>(),
                                  std::make_shared<solve::SparseLu>());
  const AdaptiveTimeTarget target{.every = {.interval = 0.3},
                                  .max_refinements = 1};
  const solve::IntegrationOutput history{.keep_step_history = true};
  const auto plain = AdaptiveIntegrator(stepped, {refinement()})
                         .integrate(start, 0.0, 0.5, target, history);
  const auto damped =
      AdaptiveIntegrator(stepped, {refinement()},
                         {.damping_steps = 1, .damping_method = euler})
          .integrate(start, 0.0, 0.5, target, history);
  check(plain.report.metrics.at("refinements") == 1.0 &&
            damped.pieces.size() == 3 && damped.pieces[1].damping &&
            damped.pieces[1].t_end == 0.35,
        "a refinement at 0.3, then one damping step");
  bool reaches = false;
  for (std::size_t k = 0; k + 1 < plain.history.size(); ++k)
    reaches = reaches || (plain.history[k].t == 0.4 &&
                          plain.history[k + 1].t == 0.4 + 0.025);
  check(reaches && 0.35 + 0.05 < 0.4,
        "the uninterrupted run steps 0.4, then 0.425 (from 0.35 a new "
        "stretch would not reach 0.4)");
  check(damped.history.size() == plain.history.size(), "as many steps");
  for (std::size_t k = 0;
       k < std::min(plain.history.size(), damped.history.size()); ++k)
    check(damped.history[k].t == plain.history[k].t &&
              damped.history[k].dt == plain.history[k].dt,
          "the same step times");
  check(damped.final.values == plain.final.values,
        "Euler as both methods: the same run, bit for bit");
}

// The estimator's u_t after a step that landed within 1e-9 dt of the
// adaptation time: the quotient divides by the step's length 0.25, as
// the method did, not by the difference of the times.
void derivatives_use_the_step_length() {
  fem::Problem moving = decay_problem();
  for (auto &[region, condition] : moving.boundary_conditions)
    condition.g = Expression("x*t");
  const auto problem =
      std::make_shared<const fem::MeshedProblem>(test::unit_square(4), moving);
  const double at = 0.5000000002;
  const auto euler = integrator(std::make_shared<solve::ImplicitEuler>(), 0.25);
  const auto plain =
      euler.integrate(problem, 0.0, at, {.keep_before_last_step = true});
  const solve::FreeState &before = *plain.before_last_step;
  check(plain.final.time == at && before.time == 0.25 && before.dt == 0.25,
        "the step landed with its nominal length");
  const auto recorder = std::make_shared<Recorder>(true);
  (void)AdaptiveIntegrator(euler, {recorder})
      .integrate(problem, 0.0, 1.0, {.every = {.interval = at}});
  check(recorder->calls.size() == 1 && recorder->calls[0].time == at,
        "one adaptation, at the landing time");
  const auto &u_t = recorder->calls[0].u_t;
  const auto &free_nodes = problem->free_nodes();
  for (std::size_t i = 0; i < free_nodes.size(); ++i)
    check(u_t[free_nodes[i]] ==
              (recorder->calls[0].values[free_nodes[i]] - before.values[i]) /
                  0.25,
          "free: (u(t) - u(t - dt)) / dt with the step's dt");
  for (const auto &d : problem->dirichlet_nodes()) {
    const double x = problem->mesh()->nodes[d.node].x;
    near(u_t[d.node], (x * at - x * 0.25) / 0.25, 1e-14,
         "Dirichlet: the boundary data's quotient over the step's dt");
  }
}

} // namespace

int main() {
  return test::run({
      {"a moving bump: refined where it is, better than uniform", moving_bump},
      {"after a transfer, implicit Euler (and damping steps) forget it",
       transfer_error_and_damping},
      {"damping steps after each transfer (midpoint method)",
       damping_steps_after_transfers},
      {"adaptation times follow the interval and the step count",
       follows_schedules},
      {"a snapshot at an adaptation time adds no step",
       snapshot_at_an_adaptation_time},
      {"a target that never refines equals one integrator run",
       never_refining_equals_plain_run},
      {"the step anchor goes across every piece on the same problem",
       anchor_across_pieces},
      {"a wave equation transfers its velocities", wave_transfers_velocities},
      {"a standing wave on adapted meshes (measured)", standing_wave},
      {"Newmark: pieces continue exactly", newmark_pieces_continue_exactly},
      {"time order 2: the u_t and u_tt an estimator gets",
       second_order_derivatives},
      {"a boundary rate not finite at t0 is never evaluated there",
       boundary_rate_not_finite_at_t0},
      {"the node budget stops adapting, not the run", node_budget},
      {"max_refinements stops adapting, not the run", refinement_limit},
      {"invalid configurations and targets throw", rejects_bad_input},
      {"after damping steps: the acceleration at their end",
       damping_hands_over_the_acceleration},
      {"after damping steps: the endpoint equation holds",
       acceleration_after_damping_satisfies_the_equation},
      {"damping steps keep the step times", damping_keeps_the_step_times},
      {"the derivatives use the step's length",
       derivatives_use_the_step_length},
  });
}
