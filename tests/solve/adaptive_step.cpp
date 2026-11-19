// The error-controlled step (AdaptiveStep) and the methods' local error
// estimates. The problems have solutions linear in x and y at every t,
// u = p g(t) with p = 1 + x + 2y: P1 elements reproduce them and every
// quadrature assembly uses is exact for them, so the error at the nodes is
// the time discretization error alone (as in time_integration.cpp). The
// time profile g decides how hard the run is: a smooth decay, a front
// (tanh) that changes abruptly at t = 1/2.

#include "solve_support.hpp"

#include "pde2d/fem/assembler.hpp"
#include "pde2d/solve/adaptive_step.hpp"
#include "pde2d/solve/fixed_step.hpp"
#include "pde2d/solve/implicit_euler.hpp"
#include "pde2d/solve/implicit_midpoint.hpp"
#include "pde2d/solve/integrator.hpp"
#include "pde2d/solve/newmark.hpp"
#include "pde2d/solve/sparse_lu.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <functional>
#include <iostream>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

using namespace pde2d::fem;
using namespace pde2d::solve;
using pde2d::test::check;
using pde2d::test::expect_throws;
using pde2d::test::near;

namespace {

Expression e(const std::string &text) { return Expression(text); }

const std::string p = "(1 + x + 2*y)";

// A time profile: g and its first and second derivatives, as text and as
// a function for the exact solution.
struct Profile {
  std::string g, g_t, g_tt;
  std::function<double(double)> value;
};

// The front: g = tanh(40 (t - 1/2)), which changes from -1 to 1 within
// about 0.1 around t = 1/2 and is nearly constant elsewhere.
Profile front() {
  const std::string th = "tanh(40*(t - 0.5))";
  return {th, "40*(1 - " + th + "^2)", "-3200*" + th + "*(1 - " + th + "^2)",
          [](double t) { return std::tanh(40.0 * (t - 0.5)); }};
}

// A fast decay to a constant: g = 2 - exp(-10 t).
Profile decay() {
  return {"(2 - exp(-10*t))", "10*exp(-10*t)", "-100*exp(-10*t)",
          [](double t) { return 2.0 - std::exp(-10.0 * t); }};
}

// A wave profile: g = cos(2 pi t) + 0.5 tanh(k (t - 1)), a smooth
// oscillation with a step of steepness k in it.
Profile wave(double k = 20.0) {
  const std::string ks = std::format("{}", k);
  const std::string th = "tanh(" + ks + "*(t - 1))";
  return {"(cos(2*pi*t) + 0.5*" + th + ")",
          "(-2*pi*sin(2*pi*t) + 0.5*" + ks + "*(1 - " + th + "^2))",
          "(-4*pi^2*cos(2*pi*t) - " + ks + "*" + ks + "*" + th + "*(1 - " + th +
              "^2))",
          [k](double t) {
            return std::cos(2.0 * std::numbers::pi * t) +
                   0.5 * std::tanh(k * (t - 1.0));
          }};
}

// u_t - (u_xx + u_yy) = f, u = p g(t), Dirichlet on every side.
Problem heat(const Profile &profile) {
  const std::string u = p + "*" + profile.g;
  const BoundaryCondition dirichlet{.alpha = e("1"), .g = e(u)};
  return {.equation = {.a_xx = e("-1"),
                       .a_yy = e("-1"),
                       .m1 = e("1"),
                       .f = e(p + "*" + profile.g_t)},
          .boundary_conditions = {{"bottom", dirichlet},
                                  {"right", dirichlet},
                                  {"top", dirichlet},
                                  {"left", dirichlet}},
          .initial = {.u = e(u)}};
}

// u_tt - (u_xx + u_yy) = f, u = p g(t), Dirichlet on every side.
Problem wave_equation(const Profile &profile) {
  const std::string u = p + "*" + profile.g;
  const BoundaryCondition dirichlet{.alpha = e("1"), .g = e(u)};
  return {.equation = {.a_xx = e("-1"),
                       .a_yy = e("-1"),
                       .m2 = e("1"),
                       .f = e(p + "*" + profile.g_tt)},
          .boundary_conditions = {{"bottom", dirichlet},
                                  {"right", dirichlet},
                                  {"top", dirichlet},
                                  {"left", dirichlet}},
          .initial = {.u = e(u), .u_t = e(p + "*" + profile.g_t)}};
}

std::shared_ptr<const MeshedProblem> meshed(const Problem &problem) {
  return std::make_shared<const MeshedProblem>(pde2d::test::unit_square(4),
                                               problem);
}

// The largest nodal error of a solution against u = p g(t).
double nodal_error(const Solution &s, const Profile &profile) {
  double worst = 0.0;
  for (std::size_t k = 0; k < s.values.size(); ++k) {
    const auto &n = s.mesh->nodes[k];
    worst = std::max(worst, std::abs(s.values[k] - (1 + n.x + 2 * n.y) *
                                                       profile.value(s.time)));
  }
  return worst;
}

Integrator adaptive(std::shared_ptr<const TimeMethod> method,
                    AdaptiveStepOptions options) {
  return Integrator(std::move(method), std::make_shared<AdaptiveStep>(options),
                    std::make_shared<SparseLu>());
}

Integrator fixed(std::shared_ptr<const TimeMethod> method, double dt) {
  return Integrator(std::move(method),
                    std::make_shared<FixedStep>(FixedStepOptions{.dt = dt}),
                    std::make_shared<SparseLu>());
}

double metric(const IntegrationResult &r, const std::string &name) {
  return r.report.metrics.at(name);
}

// The largest nodal error over the snapshots and the final solution.
double run_error(const IntegrationResult &r, const Profile &profile) {
  double worst = nodal_error(r.final, profile);
  for (const auto &s : r.snapshots)
    worst = std::max(worst, nodal_error(s, profile));
  return worst;
}

std::vector<double> sample_times(double t1, std::size_t n) {
  std::vector<double> times;
  for (std::size_t k = 1; k < n; ++k)
    times.push_back(t1 * static_cast<double>(k) / static_cast<double>(n));
  return times;
}

// --- the options and the controller alone ----------------------------------

void rejects_bad_options() {
  const AdaptiveStepOptions good{.initial_dt = 0.01};
  check(AdaptiveStep(good).options().rtol == 1e-3, "the defaults");
  const auto bad = [&](auto change, const char *fragment) {
    AdaptiveStepOptions o = good;
    change(o);
    expect_throws<std::invalid_argument>([&] { AdaptiveStep{o}; }, fragment);
  };
  bad([](auto &o) { o.initial_dt = 0.0; }, "initial_dt");
  bad([](auto &o) { o.initial_dt = INFINITY; }, "initial_dt");
  bad([](auto &o) { o.atol = 0.0; }, "atol");
  bad([](auto &o) { o.atol = NAN; }, "atol");
  bad([](auto &o) { o.rtol = -1e-3; }, "rtol");
  bad([](auto &o) { o.safety = 0.0; }, "safety");
  bad([](auto &o) { o.safety = 1.1; }, "safety");
  bad([](auto &o) { o.min_factor = 1.0; }, "min_factor");
  bad([](auto &o) { o.min_factor = 0.0; }, "min_factor");
  bad([](auto &o) { o.max_factor = 1.0; }, "max_factor");
  bad([](auto &o) { o.max_factor = INFINITY; }, "max_factor");
  bad([](auto &o) { o.min_keep_factor = 0.1; }, "min_keep_factor");
  bad([](auto &o) { o.min_keep_factor = 1.1; }, "min_keep_factor");
  bad([](auto &o) { o.max_keep_factor = 0.9; }, "max_keep_factor");
  bad([](auto &o) { o.max_keep_factor = 6.0; }, "max_keep_factor");
  bad([](auto &o) { o.min_dt = -1.0; }, "min_dt");
  bad([](auto &o) { o.max_dt = 0.0; }, "max_dt");
  bad([](auto &o) { o.min_dt = 0.1; }, "initial_dt");
  bad([](auto &o) { o.max_dt = 0.001; }, "initial_dt");
  bad(
      [](auto &o) {
        o.min_dt = 0.1;
        o.max_dt = 0.05;
      },
      "above max_dt");
  expect_throws<std::invalid_argument>(
      [] {
        Integrator(std::make_shared<ImplicitEuler>(), nullptr,
                   std::make_shared<SparseLu>());
      },
      "step controller is null");
}

void error_norm() {
  const AdaptiveStep step({.initial_dt = 0.1, .atol = 1e-2, .rtol = 0.1});
  // Scales 0.01 + 0.1 max(|before|, |after|): 0.01 + 0.1 * 2 = 0.21 and
  // 0.01 + 0.1 * 0.4 = 0.05.
  const std::vector<double> before{1.0, -0.4}, after{2.0, 0.1},
      error{0.21, -0.1};
  // Scaled 1 and -2: sqrt((1 + 4) / 2).
  near(step.error_norm(before, after, error), std::sqrt(2.5), 1e-14,
       "the root mean square of the scaled entries");
  check(step.error_norm({}, {}, {}) == 0.0, "no entries: 0");
  expect_throws<std::invalid_argument>(
      [&] {
        const std::vector<double> one{1.0};
        (void)step.error_norm(before, after, one);
      },
      "entries");
}

// judge() with a made-up error: one entry, atol 1, rtol 0, so the norm is
// |error|.
StepDecision judge(const AdaptiveStep &controller, double error, double dt,
                   double length, int order = 1) {
  const std::vector<double> zero{0.0}, err{error};
  return controller.judge({.t = 1.0,
                           .dt = dt,
                           .length = length,
                           .order = order,
                           .values_before = zero,
                           .values_after = zero,
                           .local_error = std::span<const double>(err)});
}

void controller_decisions() {
  const AdaptiveStep c({.initial_dt = 0.1, .atol = 1.0, .rtol = 0.0});
  // Accepted, err = 0.25, order 1: 0.1 * 0.9 * 2 = 0.18 (outside the keep
  // band [0.09, 0.15]).
  StepDecision d = judge(c, 0.25, 0.1, 0.1);
  check(d.accept && d.error_norm && *d.error_norm == 0.25, "accepted");
  near(d.next_dt, 0.18, 1e-15, "dt safety (1 / err)^(1 / (p + 1))");
  // Order 2: 0.1 * 0.9 * 0.064^(-1/3) = 0.1 * 0.9 * 2.5.
  near(judge(c, 0.064, 0.1, 0.1, 2).next_dt, 0.225, 1e-15,
       "the exponent 1 / (p + 1)");
  // Inside the band: 0.9 * (1 / 0.64)^(1/2) = 1.125: dt kept, exactly.
  check(judge(c, 0.64, 0.1, 0.1).next_dt == 0.1, "a small change keeps dt");
  // err = 0.95 is accepted: 0.9 / sqrt(0.95) = 0.923, inside: kept.
  d = judge(c, 0.95, 0.1, 0.1);
  check(d.accept && d.next_dt == 0.1, "accepted near 1, dt kept");
  // At most max_factor, at least min_factor (a tiny error, 0).
  near(judge(c, 1e-12, 0.1, 0.1).next_dt, 0.5, 1e-15, "at most 5 dt");
  near(judge(c, 0.0, 0.1, 0.1).next_dt, 0.5, 1e-15, "error 0: 5 dt");
  // Rejected: err = 4 gives 0.9 * 0.5 = 0.45 of the length.
  d = judge(c, 4.0, 0.1, 0.1);
  check(!d.accept && *d.error_norm == 4.0, "rejected above 1");
  near(d.next_dt, 0.045, 1e-15, "the rejection shrinks");
  // A huge error: at least min_factor; not a number: min_factor.
  near(judge(c, 1e12, 0.1, 0.1).next_dt, 0.02, 1e-15, "at least 0.2 length");
  d = judge(c, NAN, 0.1, 0.1);
  check(!d.accept, "NaN rejected");
  near(d.next_dt, 0.02, 1e-15, "NaN: min_factor");
  // err just above 1 still shrinks (safety), within the keep band: a
  // rejection always shrinks.
  d = judge(c, 1.01, 0.1, 0.1);
  check(!d.accept && d.next_dt < 0.1, "a rejection always shrinks");
  // A landing step of length 0.01 (dt 0.1) with a small error: the
  // proposal from its length, limited relative to dt, not to the length.
  d = judge(c, 1e-6, 0.1, 0.01);
  near(d.next_dt, 0.5, 1e-15, "a short landing does not shrink dt");
  d = judge(c, 0.25 * 0.25, 0.1, 0.01); // 0.01 * 0.9 * 4 = 0.036
  near(d.next_dt, 0.036, 1e-15, "a landing near its tolerance shrinks");
  // No estimate yet: accepted, dt kept.
  d = c.judge({.t = 0.0, .dt = 0.1, .length = 0.05, .order = 2});
  check(d.accept && d.next_dt == 0.1 && !d.error_norm, "no estimate");
  // The dt bounds.
  const AdaptiveStep bounded({.initial_dt = 0.1,
                              .atol = 1.0,
                              .rtol = 0.0,
                              .min_dt = 0.05,
                              .max_dt = 0.2});
  near(judge(bounded, 1e-6, 0.1, 0.1).next_dt, 0.2, 1e-15, "at most max_dt");
  near(judge(bounded, 0.9, 0.2, 0.2).next_dt, 0.2, 1e-15, "kept at max_dt");
  // A rejection whose proposal (0.02) is below min_dt tries min_dt; one
  // of a step not longer than min_dt (also a short landing) throws.
  d = judge(bounded, 100.0, 0.1, 0.1);
  check(!d.accept && d.next_dt == 0.05, "a rejection tries min_dt");
  expect_throws<std::runtime_error>(
      [&] { (void)judge(bounded, 100.0, 0.05, 0.05); },
      "not longer than min_dt = 0.05");
  expect_throws<std::runtime_error>(
      [&] { (void)judge(bounded, 1.5, 0.1, 0.04); },
      "not longer than min_dt = 0.05");
}

// --- the methods' estimates -----------------------------------------------

void method_declarations() {
  const ImplicitEuler euler;
  const ImplicitMidpoint midpoint;
  const Newmark newmark;
  check(!euler.error_estimate(0), "Euler: none at time order 0");
  check(euler.error_estimate(1)->order == 1 &&
            euler.error_estimate(1)->previous_steps == 1,
        "Euler at order 1: p = 1, one earlier step");
  check(euler.error_estimate(2)->order == 1 &&
            euler.error_estimate(2)->previous_steps == 0,
        "Euler at order 2: p = 1, no earlier step");
  check(midpoint.error_estimate(1)->order == 2 &&
            midpoint.error_estimate(1)->previous_steps == 2 &&
            midpoint.error_estimate(2)->order == 2 &&
            midpoint.error_estimate(2)->previous_steps == 1,
        "midpoint: p = 2, two earlier steps at order 1, one at order 2");
  check(newmark.error_estimate(2)->order == 2 &&
            newmark.error_estimate(2)->previous_steps == 0,
        "Newmark: p = 2, no earlier step");

  // The formulas on a scalar u = t^3 sampled at unequal times (the
  // estimates against the actual errors of steps: further below).
  const auto u = [](double t) { return t * t * t; };
  // Euler at order 1: the times 1, 1.2 (previous), 1.5 (this step).
  {
    const TimeState before{.values = {u(1.2)}}, after{.values = {u(1.5)}};
    const std::vector<PreviousStep> previous{{.values = {u(1.0)}, .dt = 0.2}};
    const double d1 = (u(1.5) - u(1.2)) / 0.3, d0 = (u(1.2) - u(1.0)) / 0.2;
    near(euler.local_error(before, after, 0.3, previous)[0],
         0.5 * 0.3 * (d1 - d0), 1e-14, "Euler: dt (d1 - d0) / 2");
  }
  // Midpoint: the times 1, 1.1, 1.3, 1.6; U[...] of t^3 is 1.
  {
    const TimeState before{.values = {u(1.3)}}, after{.values = {u(1.6)}};
    const std::vector<PreviousStep> previous{{.values = {u(1.0)}, .dt = 0.1},
                                             {.values = {u(1.1)}, .dt = 0.2}};
    near(midpoint.local_error(before, after, 0.3, previous)[0],
         0.3 * 0.3 * 0.3 / 2.0, 1e-12, "midpoint: dt^3 / 12 * 6 U[...]");
    expect_throws<std::invalid_argument>(
        [&] {
          (void)midpoint.local_error(before, after, 0.3,
                                     std::span(previous).subspan(1));
        },
        "earlier steps");
  }
  // Euler at order 2 and Newmark from the velocities and accelerations.
  {
    const TimeState before{
        .values = {0.0}, .velocities = {1.0}, .accelerations = {2.0}},
        after{.values = {0.0}, .velocities = {3.0}, .accelerations = {5.0}};
    near(euler.local_error(before, after, 0.2, {})[0], 0.1 * 2.0, 1e-15,
         "Euler at order 2: dt / 2 (V1 - V0)");
    near(newmark.local_error(before, after, 0.2, {})[0], 0.04 / 12.0 * 3.0,
         1e-15, "Newmark: dt^2 / 12 (W1 - W0) (Zienkiewicz-Xie)");
  }
}

// A time method that declares no error estimate (implicit Euler's steps
// under another name).
class NoEstimate final : public TimeMethod {
public:
  std::string name() const override { return "no_estimate"; }
  TimeMethodCapabilities capabilities() const override {
    return euler_.capabilities();
  }
  double evaluation_fraction() const override { return 1.0; }
  StepMatrixWeights step_matrix_weights(double dt) const override {
    return euler_.step_matrix_weights(dt);
  }
  StartNeeds start_needs(int order) const override {
    return euler_.start_needs(order);
  }
  StepSolveInfo advance(const AssembledProblem &problem, double dt,
                        const PreparedLinearSolver &solver,
                        TimeState &state) const override {
    return euler_.advance(problem, dt, solver, state);
  }

private:
  ImplicitEuler euler_;
};

void refuses_methods_without_an_estimate() {
  const auto problem = meshed(heat(decay()));
  expect_throws<std::invalid_argument>(
      [&] {
        (void)adaptive(std::make_shared<NoEstimate>(), {.initial_dt = 0.01})
            .integrate(problem, 0.0, 1.0);
      },
      "no_estimate declares no local error estimate");
  // Euler at time order 0 (a static problem in time): no time error.
  Problem steady = heat(decay());
  steady.equation.m1 = Expression();
  expect_throws<std::invalid_argument>(
      [&] {
        (void)adaptive(std::make_shared<ImplicitEuler>(), {.initial_dt = 0.01})
            .integrate(meshed(steady), 0.0, 1.0);
      },
      "implicit_euler declares no local error estimate at time order 0");
  // A fixed step needs none.
  check(fixed(std::make_shared<NoEstimate>(), 0.1)
                .integrate(problem, 0.0, 1.0)
                .final.time == 1.0,
        "FixedStep takes any method");
}

// --- runs -------------------------------------------------------------------

// The achieved error follows the tolerance. The controller bounds the
// error per step (not per unit step), so the global error behaves like
// tol^(p / (p + 1)) (Hairer, Norsett and Wanner, II.4): about 10^(1/2) =
// 3.2 times smaller per tenfold tolerance for implicit Euler, 10^(2/3) =
// 4.6 for the midpoint method; checked with 70 % of that.
void error_follows_the_tolerance() {
  const Profile profile = front();
  const auto problem = meshed(heat(profile));
  const std::vector<double> samples = sample_times(1.0, 40);
  for (const auto &method : std::vector<std::shared_ptr<const TimeMethod>>{
           std::make_shared<ImplicitEuler>(),
           std::make_shared<ImplicitMidpoint>()}) {
    std::vector<double> errors, steps;
    const double order = method->error_estimate(1)->order;
    const double ratio = 0.7 * std::pow(10.0, order / (order + 1.0));
    for (const double rtol : {1e-2, 1e-3, 1e-4}) {
      const auto r =
          adaptive(method,
                   {.initial_dt = 1e-3, .atol = rtol * 1e-2, .rtol = rtol})
              .integrate(problem, 0.0, 1.0, {.snapshot_times = samples});
      errors.push_back(run_error(r, profile));
      steps.push_back(metric(r, "steps"));
      std::cout << "INFO " << method->name() << " rtol " << rtol << ": error "
                << errors.back() << ", steps " << metric(r, "steps")
                << ", rejected " << metric(r, "rejected_steps") << ", prepares "
                << metric(r, "prepares") << "\n";
    }
    for (std::size_t k = 0; k + 1 < errors.size(); ++k) {
      check(errors[k + 1] < errors[k] / ratio,
            "a smaller tolerance, smaller error");
      check(steps[k + 1] > steps[k], "and more steps");
    }
  }
}

// The front: steps are rejected where it starts, dt is small inside it and
// large on both sides, and every snapshot time and t1 is landed on.
void rejects_at_the_front_and_grows_after() {
  const Profile profile = front();
  const auto problem = meshed(heat(profile));
  const std::vector<double> snapshots{0.25, 0.5, 0.75};
  for (const auto &method : std::vector<std::shared_ptr<const TimeMethod>>{
           std::make_shared<ImplicitEuler>(),
           std::make_shared<ImplicitMidpoint>()}) {
    const auto r =
        adaptive(method, {.initial_dt = 0.01, .atol = 1e-6, .rtol = 1e-3})
            .integrate(
                problem, 0.0, 1.0,
                {.keep_step_history = true, .snapshot_times = snapshots});
    std::size_t rejected = 0, accepted = 0;
    double dt_front = INFINITY, dt_before = 0.0, dt_after = 0.0;
    for (const auto &record : r.history) {
      if (!record.accepted) {
        ++rejected;
        check(record.error_norm && *record.error_norm > 1.0,
              "a rejected step's error norm is above 1");
        check(record.t - record.dt > 0.2 && record.t - record.dt < 0.7,
              "rejections near the front");
        continue;
      }
      ++accepted;
      check(!record.error_norm || *record.error_norm <= 1.0,
            "an accepted step's error norm is at most 1");
      if (std::abs(record.t - 0.5) < 0.05)
        dt_front = std::min(dt_front, record.dt);
      if (record.t < 0.25)
        dt_before = std::max(dt_before, record.dt);
      if (record.t > 0.8)
        dt_after = std::max(dt_after, record.dt);
    }
    std::cout << "INFO " << method->name() << ": " << accepted << " accepted, "
              << rejected << " rejected, dt " << dt_before << " before, "
              << dt_front << " in, " << dt_after << " after the front\n";
    check(rejected > 0 &&
              metric(r, "rejected_steps") == static_cast<double>(rejected),
          "rejections, counted in the report");
    check(metric(r, "steps") == static_cast<double>(accepted),
          "steps counts the accepted steps");
    check(dt_before > 10.0 * dt_front && dt_after > 10.0 * dt_front,
          "small steps in the front, large ones around it");
    // The landings.
    check(r.snapshots.size() == 3, "every snapshot");
    for (std::size_t k = 0; k < 3; ++k)
      check(r.snapshots[k].time == snapshots[k], "landed exactly");
    check(r.final.time == 1.0 && r.history.back().t == 1.0, "landed on t1");
    // The rejected step is taken again from the same time, shorter.
    for (std::size_t k = 0; k + 1 < r.history.size(); ++k)
      if (!r.history[k].accepted)
        check(r.history[k + 1].dt < r.history[k].dt &&
                  r.history[k + 1].t - r.history[k + 1].dt <
                      r.history[k].t - r.history[k].dt + 1e-12,
              "the same step again, shorter");
  }
}

// A smooth decay: dt grows as the solution settles, and the keep band
// keeps the number of preparations below the number of steps.
void grows_when_smooth() {
  const Profile profile = decay();
  const auto problem = meshed(heat(profile));
  const auto r = adaptive(std::make_shared<ImplicitMidpoint>(),
                          {.initial_dt = 1e-3, .atol = 1e-6, .rtol = 1e-4})
                     .integrate(problem, 0.0, 3.0, {.keep_step_history = true});
  const auto &h = r.history;
  std::cout << "INFO decay: " << metric(r, "steps") << " steps, "
            << metric(r, "rejected_steps") << " rejected, "
            << metric(r, "prepares") << " prepares, dt from " << h.front().dt
            << " to " << metric(r, "max_dt") << "\n";
  check(metric(r, "max_dt") > 100.0 * 1e-3, "dt grows by far");
  check(metric(r, "rejected_steps") < 0.2 * metric(r, "steps"),
        "few rejections");
  check(metric(r, "prepares") < metric(r, "steps"),
        "kept steps need no new preparation");
  near(nodal_error(r.final, profile), 0.0, 1e-3, "accurate at t1");
}

// The run that continues with its anchor equals one long run, bit for bit,
// with the rejections: the next dt and the earlier steps come with the
// anchor.
void continues_with_the_anchor() {
  const Profile profile = front();
  const auto problem = meshed(heat(profile));
  for (const auto &method : std::vector<std::shared_ptr<const TimeMethod>>{
           std::make_shared<ImplicitEuler>(),
           std::make_shared<ImplicitMidpoint>()}) {
    const Integrator integrator =
        adaptive(method, {.initial_dt = 0.01, .atol = 1e-6, .rtol = 1e-3});
    const IntegrationOutput history{.keep_step_history = true};
    const auto whole = integrator.integrate(problem, 0.0, 1.0, history);
    std::vector<StepRecord> pieces;
    IntegrationResult part = integrator.integrate(problem, 0.0, 1.0, history,
                                                  nullptr, std::size_t{7});
    pieces.insert(pieces.end(), part.history.begin(), part.history.end());
    std::size_t calls = 1;
    while (part.final.time < 1.0) {
      const Solution state = part.final;
      const StepAnchor anchor = part.anchor;
      check(anchor.next_dt > 0.0, "the anchor carries the next dt");
      part = integrator.integrate(problem, state.time, 1.0, history, &state,
                                  std::size_t{7}, &anchor);
      pieces.insert(pieces.end(), part.history.begin(), part.history.end());
      ++calls;
    }
    check(calls > 3, "several pieces");
    check(pieces.size() == whole.history.size(), "the same steps");
    for (std::size_t k = 0; k < std::min(pieces.size(), whole.history.size());
         ++k)
      check(pieces[k].t == whole.history[k].t &&
                pieces[k].dt == whole.history[k].dt &&
                pieces[k].accepted == whole.history[k].accepted,
            "the same steps and rejections");
    check(part.final.values == whole.final.values, "bit for bit");
    // Without the anchor the pieces start again from initial_dt.
    const Solution half = integrator.integrate(problem, 0.0, 0.5).final;
    const auto restart =
        integrator.integrate(problem, 0.5, 1.0, history, &half);
    check(restart.history.front().dt == 0.01, "no anchor: initial_dt");
    // An anchor from another problem's run is refused.
    StepAnchor wrong = whole.anchor;
    wrong.previous_steps.assign(1, {.values = {1.0}, .dt = 0.1});
    expect_throws<std::invalid_argument>(
        [&] {
          (void)integrator.integrate(problem, 1.0, 1.5, {}, &whole.final, {},
                                     &wrong);
        },
        "previous step of the step anchor");
  }
}

// min_dt: a front too steep for it stops the run with a clear message
// (a rejected step at min_dt).
void stops_below_min_dt() {
  const auto problem = meshed(heat(front()));
  expect_throws<std::runtime_error>(
      [&] {
        (void)adaptive(
            std::make_shared<ImplicitEuler>(),
            {.initial_dt = 0.01, .atol = 1e-8, .rtol = 1e-6, .min_dt = 0.005})
            .integrate(problem, 0.0, 1.0);
      },
      "not longer than min_dt = 0.005");
}

// --- estimates against the actual error of a step --------------------------
//
// Problems with A = 0, so every free unknown follows its own ODE in time
// (the load and the Dirichlet lifting are M times the data): the exact
// evolution from any state a step starts from is known in closed form,
// and the actual local error of a step is its end state minus that
// evolution from its start state. The estimates are compared with it in
// the controller's norm.

// The values (and velocities) of a solution at the free unknowns.
std::vector<double> free_part(const MeshedProblem &problem,
                              const std::vector<double> &all) {
  std::vector<double> part;
  for (const std::size_t node : problem.free_nodes())
    part.push_back(all[node]);
  return part;
}

// The controller's norm of the actual local error of the step from
// `start` to `end`, given the exact evolution of one value (and velocity)
// over [t0, t1].
double actual_norm(const AdaptiveStep &controller, const MeshedProblem &problem,
                   const Solution &start, const Solution &end,
                   const std::function<double(double u0, double v0, double t0,
                                              double t1)> &evolve) {
  const std::vector<double> before = free_part(problem, start.values),
                            after = free_part(problem, end.values);
  const std::vector<double> v0 = start.velocities.empty()
                                     ? std::vector<double>(before.size(), 0.0)
                                     : free_part(problem, start.velocities);
  std::vector<double> actual(before.size());
  for (std::size_t i = 0; i < before.size(); ++i)
    actual[i] = after[i] - evolve(before[i], v0[i], start.time, end.time);
  return controller.error_norm(before, after, actual);
}

// u_t = 2t (u = t^2 on the boundary and at t = 0): every free unknown
// follows U' = 2t, U(t1) = U(t0) + t1^2 - t0^2.
Problem rate_2t() {
  const BoundaryCondition dirichlet{.alpha = e("1"), .g = e("t^2")};
  return {.equation = {.m1 = e("1"), .f = e("2*t")},
          .boundary_conditions = {{"bottom", dirichlet},
                                  {"right", dirichlet},
                                  {"top", dirichlet},
                                  {"left", dirichlet}},
          .initial = {.u = e("t^2")}};
}

// u_tt = c (u = c t^2 / 2): U(t1) = U0 + V0 h + c h^2 / 2, h = t1 - t0.
// u_tt = t (u = t^3 / 6): U(t1) = U0 + V0 h + t0 h^2 / 2 + h^3 / 6.
Problem acceleration(const std::string &u, const std::string &u_t,
                     const std::string &u_tt) {
  const BoundaryCondition dirichlet{.alpha = e("1"), .g = e(u)};
  return {.equation = {.m2 = e("1"), .f = e(u_tt)},
          .boundary_conditions = {{"bottom", dirichlet},
                                  {"right", dirichlet},
                                  {"top", dirichlet},
                                  {"left", dirichlet}},
          .initial = {.u = e(u), .u_t = e(u_t)}};
}

// Implicit Euler at time order 1, a step of another length than the one
// before: the quotients d = (U1 - U0) / h are the rates at the steps'
// ends, a step's length apart, whatever the length of the step before
// (review finding: the estimate was off by 2h / (h + k)). Here the actual
// local error is h^2, and the estimate h (d1 - d0) / 2 is exact.
void euler_after_a_step_of_another_length() {
  const auto problem = meshed(rate_2t());
  struct Case {
    double first, t1, atol;
  };
  // A landing of 0.02 after a step of 0.1; a step of 0.1 after a landing
  // of 0.01. atol so that the step is accepted (norms 0.4 and 0.5).
  for (const Case &c : {Case{0.1, 0.12, 1e-3}, Case{0.01, 0.11, 2e-2}}) {
    const AdaptiveStepOptions options{
        .initial_dt = 0.1, .atol = c.atol, .rtol = 0.0};
    const auto r = adaptive(std::make_shared<ImplicitEuler>(), options)
                       .integrate(problem, 0.0, c.t1,
                                  {.keep_step_history = true,
                                   .snapshot_times = {c.first}});
    check(r.history.size() == 2 && r.history[1].accepted &&
              r.history[1].error_norm.has_value(),
          "two steps, the second estimated and accepted");
    const double actual =
        actual_norm(AdaptiveStep(options), *problem, r.snapshots.at(0), r.final,
                    [](double u0, double, double t0, double t1) {
                      return u0 + t1 * t1 - t0 * t0;
                    });
    near(*r.history[1].error_norm, actual, 1e-9 * actual,
         std::format("Euler, {} after {}: the estimate is the actual error",
                     c.t1 - c.first, c.first));
  }
}

// The midpoint method at time order 2 on u_tt = t with steps of 0.01, 0.1
// and 0.01: V is exact (the midpoint rule integrates the linear u_tt
// exactly) and U takes the trapezoidal rule of V, so the actual local
// error is h^3 / 12 u''' = h^3 / 12. The estimate from the velocities is
// exact; the one from the third divided difference of U was suppressed by
// the errors U carries from the long step (review finding: 0.521 reported
// against an actual 2.08).
void midpoint_second_order_on_unequal_steps() {
  const auto problem = meshed(acceleration("t^3/6", "t^2/2", "t"));
  const AdaptiveStepOptions options{
      .initial_dt = 0.1, .atol = 1e-4, .rtol = 0.0};
  const auto r = adaptive(std::make_shared<ImplicitMidpoint>(), options)
                     .integrate(problem, 0.0, 0.12,
                                {.keep_step_history = true,
                                 .snapshot_times = {0.01, 0.11}});
  check(r.history.size() == 3 &&
            std::ranges::all_of(r.history,
                                [](const auto &s) { return s.accepted; }),
        "three accepted steps");
  near(r.history[1].dt, 0.1, 1e-12, "the long step");
  const auto evolve = [](double u0, double v0, double t0, double t1) {
    const double h = t1 - t0;
    return u0 + v0 * h + t0 * h * h / 2.0 + h * h * h / 6.0;
  };
  const Solution *starts[] = {&r.snapshots.at(0), &r.snapshots.at(1)};
  const Solution *ends[] = {&r.snapshots.at(1), &r.final};
  for (std::size_t k = 0; k < 2; ++k) {
    const auto &record = r.history[k + 1];
    check(record.error_norm.has_value(),
          std::format("step {} is estimated", k + 2));
    const double actual = actual_norm(AdaptiveStep(options), *problem,
                                      *starts[k], *ends[k], evolve);
    near(*record.error_norm, actual, 1e-8 * actual,
         std::format("midpoint, step {} of length {}: the estimate is the "
                     "actual error",
                     k + 2, record.dt));
  }
}

// The midpoint method at time order 2 continued with a hand-made anchor
// whose earlier step was not one of its own: the state at its start is
// the exact one (u = t^3 / 6, u_t = t^2 / 2 at t = 0, the step 0.3 long).
// The estimate reads the stored velocity V_-1 = 0, so u''' = 2 V[0, 0.3,
// 0.4] = 1 is exact and the estimate is the actual error h^3 / 12; the
// velocity inferred from a midpoint update, 2 (U0 - U_-1) / dt_-1 - V0 =
// -0.015, gave another estimate (review finding).
void midpoint_after_a_step_of_another_method() {
  const auto problem = meshed(acceleration("t^3/6", "t^2/2", "t"));
  const std::size_t nodes = problem->mesh()->nodes.size(),
                    free = problem->free_nodes().size();
  const double start = 0.3;
  const Solution initial{
      .mesh = problem->mesh(),
      .time = start,
      .values = std::vector<double>(nodes, start * start * start / 6),
      .velocities = std::vector<double>(nodes, start * start / 2)};
  const StepAnchor anchor{
      .time = start,
      .next_dt = 0.1,
      .previous_steps = {{.values = std::vector<double>(free, 0.0),
                          .dt = 0.3,
                          .velocities = std::vector<double>(free, 0.0)}}};
  const AdaptiveStepOptions options{
      .initial_dt = 0.1, .atol = 1e-3, .rtol = 0.0};
  const Integrator integrator =
      adaptive(std::make_shared<ImplicitMidpoint>(), options);
  const auto r = integrator.integrate(
      problem, start, 0.4, {.keep_step_history = true}, &initial, {}, &anchor);
  check(r.history.size() == 1 && r.history[0].accepted &&
            r.history[0].error_norm.has_value(),
        "one step, estimated from the anchor's earlier step");
  const double actual =
      actual_norm(AdaptiveStep(options), *problem, initial, r.final,
                  [](double u0, double v0, double t0, double t1) {
                    const double h = t1 - t0;
                    return u0 + v0 * h + t0 * h * h / 2.0 + h * h * h / 6.0;
                  });
  near(actual, 1.0 / 12.0, 1e-9, "the actual error is h^3 / 12 = atol / 12");
  near(*r.history[0].error_norm, actual, 1e-8 * actual,
       "the estimate is the actual error");
  // An earlier step at time order 2 without its velocities is refused.
  StepAnchor without = anchor;
  without.previous_steps[0].velocities.clear();
  expect_throws<std::invalid_argument>(
      [&] {
        (void)integrator.integrate(problem, start, 0.4, {}, &initial, {},
                                   &without);
      },
      "0 velocities");
}

// min_dt: a rejected step longer than min_dt is taken again at min_dt when
// the proposal is below it (review finding: the run stopped although the
// step at min_dt passes). Implicit Euler at time order 2 on u_tt = 2: the
// actual local error of a step of length h from any state is h^2.
void retries_at_min_dt() {
  const auto problem = meshed(acceleration("t^2", "2*t", "2"));
  // The step of 0.1 has the norm 1.1; the proposal 0.1 * 0.9 / sqrt(1.1)
  // = 0.0858 is below min_dt = 0.09, where the norm is 0.891.
  const AdaptiveStepOptions options{
      .initial_dt = 0.1, .atol = 0.01 / 1.1, .rtol = 0.0, .min_dt = 0.09};
  const AdaptiveStep controller(options);
  const auto evolve = [](double u0, double v0, double t0, double t1) {
    const double h = t1 - t0;
    return u0 + v0 * h + h * h;
  };
  // The rejected step, as a fixed step of 0.1: its actual norm is above 1.
  const Solution initial =
      fixed(std::make_shared<ImplicitEuler>(), 0.1)
          .integrate(problem, 0.0, 0.1, {.snapshot_times = {0.0}})
          .snapshots.at(0);
  const double rejected =
      actual_norm(controller, *problem, initial,
                  fixed(std::make_shared<ImplicitEuler>(), 0.1)
                      .integrate(problem, 0.0, 0.1)
                      .final,
                  evolve);
  near(rejected, 1.1, 1e-9, "the step of 0.1 fails the tolerance");
  IntegrationResult r;
  try {
    r = adaptive(std::make_shared<ImplicitEuler>(), options)
            .integrate(problem, 0.0, 0.3,
                       {.keep_step_history = true, .snapshot_every = 1});
  } catch (const std::runtime_error &error) {
    check(false, std::string("the run stopped: ") + error.what());
    return;
  }
  check(r.history.size() >= 2 && !r.history[0].accepted &&
            r.history[1].accepted && r.history[1].dt == 0.09,
        "rejected at 0.1, taken again at min_dt");
  near(*r.history[0].error_norm, rejected, 1e-9, "the rejected step's norm");
  const double actual =
      actual_norm(controller, *problem, initial, r.snapshots.at(0), evolve);
  check(actual <= 1.0, "the step at min_dt passes");
  near(*r.history[1].error_norm, actual, 1e-9, "its estimate is its error");
  check(r.final.time == 0.3, "the run reaches t1");
  for (const auto &record : r.history)
    check(!record.accepted || record.dt >= 0.09 || record.t == 0.3,
          "no step below min_dt but the landing");
}

// A mesh whose every node is a Dirichlet node: no free unknowns, so every
// step's local error is empty (norm 0) for every method and time order
// with an estimate (review finding: implicit Euler at time order 2 threw).
void estimates_without_free_unknowns() {
  const auto square = pde2d::test::unit_square(1);
  struct Case {
    std::shared_ptr<const TimeMethod> method;
    Problem problem;
  };
  const std::vector<Case> cases{
      {std::make_shared<ImplicitEuler>(), heat(decay())},
      {std::make_shared<ImplicitEuler>(), wave_equation(wave())},
      {std::make_shared<ImplicitMidpoint>(), heat(decay())},
      {std::make_shared<ImplicitMidpoint>(), wave_equation(wave())},
      {std::make_shared<Newmark>(), wave_equation(wave())}};
  for (const Case &c : cases) {
    const auto problem =
        std::make_shared<const MeshedProblem>(square, c.problem);
    check(problem->free_nodes().empty(), "no free unknowns");
    const int order = Assembler().time_order(*problem);
    const std::string what =
        std::format("{} at order {}", c.method->name(), order);
    IntegrationResult r;
    try {
      r = adaptive(c.method, {.initial_dt = 0.1})
              .integrate(problem, 0.0, 1.0, {.keep_step_history = true});
    } catch (const std::exception &error) {
      check(false, what + ": " + error.what());
      continue;
    }
    check(r.final.time == 1.0, what + ": reaches t1");
    const auto estimated = std::ranges::count_if(
        r.history, [](const auto &s) { return s.error_norm.has_value(); });
    check(estimated > 0 && std::ranges::all_of(r.history,
                                               [](const auto &s) {
                                                 return s.accepted &&
                                                        (!s.error_norm ||
                                                         *s.error_norm == 0.0);
                                               }),
          what + ": every estimate 0, every step accepted");
    // The method alone: an empty error, whatever earlier steps it reads.
    const std::vector<PreviousStep> previous(
        c.method->error_estimate(order)->previous_steps,
        PreviousStep{.values = {}, .dt = 0.1});
    check(c.method->local_error({}, {}, 0.1, previous).empty(),
          what + ": local_error is empty");
  }
}

// Time order 2: Newmark (and implicit Euler and the midpoint method) on a
// wave with a step in its profile; the error follows the tolerance.
void wave_with_newmark() {
  const Profile profile = wave();
  const auto problem = meshed(wave_equation(profile));
  const std::vector<double> samples = sample_times(2.0, 40);
  for (const auto &method : std::vector<std::shared_ptr<const TimeMethod>>{
           std::make_shared<Newmark>(), std::make_shared<ImplicitMidpoint>(),
           std::make_shared<ImplicitEuler>()}) {
    std::vector<double> errors;
    for (const double rtol : {1e-3, 1e-4, 1e-5}) {
      const auto r =
          adaptive(method, {.initial_dt = 1e-3, .atol = rtol, .rtol = rtol})
              .integrate(problem, 0.0, 2.0, {.snapshot_times = samples});
      errors.push_back(run_error(r, profile));
      std::cout << "INFO wave, " << method->name() << " rtol " << rtol
                << ": error " << errors.back() << ", steps "
                << metric(r, "steps") << ", rejected "
                << metric(r, "rejected_steps") << ", prepares "
                << metric(r, "prepares") << "\n";
      check(r.final.time == 2.0, "reaches t1");
    }
    for (std::size_t k = 0; k + 1 < errors.size(); ++k)
      check(errors[k + 1] < errors[k] / 2.0,
            "a smaller tolerance, smaller error");
  }
}

// The cost against a fixed step of about the same accuracy: the number of
// steps and of linear-solver preparations (measured, printed). The error
// is the largest nodal error at t1 / 4, t1 / 2, 3 t1 / 4 and t1 (few
// landings: each landing that shortens a step costs two preparations).
// The fixed step is the largest of t1 / (4 m) whose error is at most the
// adaptive run's. The adaptive runs have max_dt = t1 / 20: an
// estimate is made from the values a method computed, so a step long
// enough to jump over the onset of the front (the midpoint rule samples
// the load only at the middle of the step) is not seen as wrong (without
// the bound, the midpoint run at rtol 1e-4 steps from 0.25 to 0.46 with
// an error norm of 0.23 and ends 0.17 wrong at t = 0.5). Where the solution has
// a fast phase (a front, a steep step in a wave) the adaptive run needs fewer
// steps; on a smooth oscillation it does not, and it prepares the linear solver
// far more often than a fixed step (once).
void cost_against_a_fixed_step() {
  struct Case {
    std::string name;
    std::shared_ptr<const MeshedProblem> problem;
    Profile profile;
    std::shared_ptr<const TimeMethod> method;
    double t1, rtol;
    bool adaptive_wins;
  };
  const std::vector<Case> cases{
      {"heat (front), midpoint", meshed(heat(front())), front(),
       std::make_shared<ImplicitMidpoint>(), 1.0, 1e-4, true},
      {"heat (front), Euler", meshed(heat(front())), front(),
       std::make_shared<ImplicitEuler>(), 1.0, 1e-3, true},
      {"wave (oscillation, mild step), Newmark",
       meshed(wave_equation(wave(20.0))), wave(20.0),
       std::make_shared<Newmark>(), 2.0, 1e-4, false},
      {"wave (oscillation, steep step), Newmark",
       meshed(wave_equation(wave(400.0))), wave(400.0),
       std::make_shared<Newmark>(), 2.0, 1e-4, true},
  };
  for (const Case &c : cases) {
    const std::vector<double> samples = sample_times(c.t1, 4);
    const auto r =
        adaptive(c.method, {.initial_dt = 1e-3,
                            .atol = c.rtol * 1e-2,
                            .rtol = c.rtol,
                            .max_dt = c.t1 / 20.0})
            .integrate(c.problem, 0.0, c.t1, {.snapshot_times = samples});
    const double error = run_error(r, c.profile);
    // Fixed steps t1 / (4 m), m growing by about 10 % (so the steps land
    // on the samples without shortening), until the error is at most the
    // adaptive run's.
    double dt = 0.0;
    IntegrationResult f;
    double fixed_error = INFINITY;
    for (std::size_t m = 5; fixed_error > error && m < 100000;
         m = std::max(m + 1, m * 11 / 10)) {
      dt = c.t1 / static_cast<double>(4 * m);
      f = fixed(c.method, dt)
              .integrate(c.problem, 0.0, c.t1, {.snapshot_times = samples});
      fixed_error = run_error(f, c.profile);
    }
    std::cout << "INFO cost, " << c.name << ": adaptive error " << error
              << " with " << metric(r, "steps") << " steps ("
              << metric(r, "rejected_steps") << " rejected), "
              << metric(r, "prepares") << " prepares; fixed dt " << dt
              << " error " << fixed_error << " with " << metric(f, "steps")
              << " steps, " << metric(f, "prepares") << " prepares\n";
    if (c.adaptive_wins)
      check(metric(r, "steps") + metric(r, "rejected_steps") <
                metric(f, "steps"),
            "fewer steps than a fixed step of the same accuracy");
  }
}

} // namespace

int main() {
  return pde2d::test::run({
      {"options are checked", rejects_bad_options},
      {"the error norm", error_norm},
      {"the controller's decisions", controller_decisions},
      {"the methods' estimates", method_declarations},
      {"a method without an estimate is refused",
       refuses_methods_without_an_estimate},
      {"the error follows the tolerance", error_follows_the_tolerance},
      {"rejections at a front, larger steps after",
       rejects_at_the_front_and_grows_after},
      {"dt grows when the solution is smooth", grows_when_smooth},
      {"a run continued with its anchor equals one run",
       continues_with_the_anchor},
      {"below min_dt the run stops", stops_below_min_dt},
      {"implicit Euler (order 1) after a step of another length",
       euler_after_a_step_of_another_length},
      {"the midpoint method (order 2) on unequal steps",
       midpoint_second_order_on_unequal_steps},
      {"the midpoint method (order 2) after another method's step",
       midpoint_after_a_step_of_another_method},
      {"a rejected step is taken again at min_dt", retries_at_min_dt},
      {"no free unknowns: empty estimates", estimates_without_free_unknowns},
      {"time order 2: Newmark on a wave", wave_with_newmark},
      {"the cost against a fixed step (measured)", cost_against_a_fixed_step},
  });
}
