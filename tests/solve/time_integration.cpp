#include "solve_support.hpp"

#include "pde2d/fem/assembler.hpp"
#include "pde2d/solve/bicgstab.hpp"
#include "pde2d/solve/fixed_step.hpp"
#include "pde2d/solve/implicit_euler.hpp"
#include "pde2d/solve/implicit_midpoint.hpp"
#include "pde2d/solve/integrator.hpp"
#include "pde2d/solve/newmark.hpp"
#include "pde2d/solve/sparse_lu.hpp"
#include "pde2d/solve/static_solver.hpp"

#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

// Manufactured solutions that are linear in x and y at every t. P1
// elements reproduce them exactly, and every quadrature used by assembly
// is exact for them, so the semi-discrete solution is the exact solution
// at the nodes: the error at the nodes is the time discretization error
// alone, and its rate is the method's order.

using namespace pde2d::fem;
using namespace pde2d::solve;
using pde2d::test::check;
using pde2d::test::expect_throws;
using pde2d::test::near;

namespace {

Expression e(const std::string &text) { return Expression(text); }

const std::string p = "(1 + x + 2*y)";
const std::string q = "(x - y)";

// --- time order 1 -----------------------------------------------------------
//
// u = p cos(2t) + q sin(t) for
//   -(1 + t) (u_xx + u_yy) + u + (1 + t/2) u_t = f
// (a stiffness and a damping that depend on t), with moving Dirichlet data
// on the bottom and the left and u_n = g on the right and the top.
double heat_exact(double x, double y, double t) {
  return (1 + x + 2 * y) * std::cos(2 * t) + (x - y) * std::sin(t);
}

Problem heat_problem() {
  const std::string u = p + "*cos(2*t) + " + q + "*sin(t)";
  const std::string u_t = "-2*" + p + "*sin(2*t) + " + q + "*cos(t)";
  const BoundaryCondition dirichlet{.alpha = e("1"), .g = e(u)};
  return {.equation = {.a_xx = e("-(1 + t)"),
                       .a_yy = e("-(1 + t)"),
                       .c = e("1"),
                       .m1 = e("1 + t/2"),
                       .f = e(u + " + (1 + t/2)*(" + u_t + ")")},
          .boundary_conditions =
              {{"bottom", dirichlet},
               {"left", dirichlet},
               {"right", {.beta_n = e("1"), .g = e("cos(2*t) + sin(t)")}},
               {"top", {.beta_n = e("1"), .g = e("2*cos(2*t) - sin(t)")}}},
          .initial = {.u = e(u)}};
}

// The same equation with constant coefficients and a load that depends
// on t: u = p (1 + t) + q t^2 solves u_t - (u_xx + u_yy) = f.
Problem constant_heat_problem() {
  const std::string u = p + "*(1 + t) + " + q + "*t^2";
  const BoundaryCondition dirichlet{.alpha = e("1"), .g = e(u)};
  return {.equation = {.a_xx = e("-1"),
                       .a_yy = e("-1"),
                       .m1 = e("1"),
                       .f = e(p + " + 2*t*" + q)},
          .boundary_conditions =
              {{"bottom", dirichlet},
               {"left", dirichlet},
               {"right", {.beta_n = e("1"), .g = e("(1 + t) + t^2")}},
               {"top", {.beta_n = e("1"), .g = e("2*(1 + t) - t^2")}}},
          .initial = {.u = e(u)}};
}

// The solution of heat_problem for u_xx + u_yy = 0 with m1 = 0: only the
// dynamic condition u_n + u_t = g on the right has a time derivative, so
// the damping matrix B is singular (zero rows at the other unknowns).
Problem dynamic_boundary_problem() {
  const std::string u = p + "*cos(2*t) + " + q + "*sin(t)";
  const std::string u_t = "-2*" + p + "*sin(2*t) + " + q + "*cos(t)";
  const BoundaryCondition dirichlet{.alpha = e("1"), .g = e(u)};
  return {.equation = {.a_xx = e("-1"), .a_yy = e("-1")},
          .boundary_conditions =
              {{"bottom", dirichlet},
               {"left", dirichlet},
               {"right",
                {.beta_n = e("1"),
                 .gamma = e("1"),
                 .g = e("cos(2*t) + sin(t) + " + u_t)}},
               {"top", {.beta_n = e("1"), .g = e("2*cos(2*t) - sin(t)")}}},
          .initial = {.u = e(u)}};
}

// --- time order 2 -----------------------------------------------------------
//
// u = p cos(2t) + q sin(3t) for u_tt + 0.1 u_t - (u_xx + u_yy) = f, with
// moving Dirichlet data on the bottom and the left.
double wave_exact(double x, double y, double t) {
  return (1 + x + 2 * y) * std::cos(2 * t) + (x - y) * std::sin(3 * t);
}

Problem wave_problem() {
  const std::string u = p + "*cos(2*t) + " + q + "*sin(3*t)";
  const std::string u_t = "-2*" + p + "*sin(2*t) + 3*" + q + "*cos(3*t)";
  const std::string u_tt = "-4*" + p + "*cos(2*t) - 9*" + q + "*sin(3*t)";
  const BoundaryCondition dirichlet{.alpha = e("1"), .g = e(u)};
  return {.equation = {.a_xx = e("-1"),
                       .a_yy = e("-1"),
                       .m1 = e("0.1"),
                       .m2 = e("1"),
                       .f = e(u_tt + " + 0.1*(" + u_t + ")")},
          .boundary_conditions =
              {{"bottom", dirichlet},
               {"left", dirichlet},
               {"right", {.beta_n = e("1"), .g = e("cos(2*t) + sin(3*t)")}},
               {"top", {.beta_n = e("1"), .g = e("2*cos(2*t) - sin(3*t)")}}},
          .initial = {.u = e(u), .u_t = e(u_t)}};
}

std::shared_ptr<const MeshedProblem> meshed(const Problem &problem,
                                            std::size_t n = 4) {
  return std::make_shared<const MeshedProblem>(pde2d::test::unit_square(n),
                                               problem);
}

Integrator integrator(std::shared_ptr<const TimeMethod> method, double dt,
                      std::shared_ptr<const LinearSolver> linear_solver =
                          std::make_shared<SparseLu>()) {
  return Integrator(std::move(method),
                    std::make_shared<FixedStep>(FixedStepOptions{.dt = dt}),
                    std::move(linear_solver));
}

using Exact = std::function<double(double, double, double)>;

// The time convergence rates of `method` on [0, 1] with dt = 0.05, 0.025,
// 0.0125, from the largest nodal error at t = 0.25, 0.5, 0.75 and 1 (one
// time alone can sit near a zero of the error).
void time_rate(const std::shared_ptr<const TimeMethod> &method,
               const Problem &problem, const Exact &exact, double low,
               double high) {
  const auto m = meshed(problem);
  std::vector<double> errors;
  for (const double dt : {0.05, 0.025, 0.0125}) {
    const auto result = integrator(method, dt).integrate(
        m, 0.0, 1.0, {.snapshot_times = {0.25, 0.5, 0.75, 1.0}});
    check(result.final.time == 1.0, "ends at t1");
    double error = 0.0;
    for (const auto &s : result.snapshots)
      error = std::max(error, pde2d::test::max_nodal_error(
                                  *m->mesh(), s.values, [&](double x, double y) {
                                    return exact(x, y, s.time);
                                  }));
    errors.push_back(error);
  }
  for (std::size_t i = 1; i < errors.size(); ++i) {
    const double rate = std::log2(errors[i - 1] / errors[i]);
    std::cout << method->name() << ": error " << errors[i - 1] << " -> "
              << errors[i] << ", rate " << rate << '\n';
    check(rate > low && rate < high, "the method's order");
  }
}

// E = V^T M2 V / 2 + U^T A U / 2 in the free unknowns.
double energy(const AssembledProblem &a, const MeshedProblem &m,
              const Solution &s) {
  std::vector<double> u, v;
  for (const std::size_t node : m.free_nodes()) {
    u.push_back(s.values[node]);
    v.push_back(s.velocities[node]);
  }
  const auto mv = a.mass().free.multiply(v);
  const auto au = a.stiffness().free.multiply(u);
  double sum = 0.0;
  for (std::size_t i = 0; i < u.size(); ++i)
    sum += v[i] * mv[i] + u[i] * au[i];
  return sum / 2;
}

void energy_is_conserved() {
  const BoundaryCondition zero{.alpha = e("1")};
  const auto m = meshed(
      {.equation = {.a_xx = e("-1"), .a_yy = e("-1"), .m2 = e("1")},
       .boundary_conditions = {{"bottom", zero},
                               {"right", zero},
                               {"top", zero},
                               {"left", zero}},
       .initial = {.u = e("sin(pi*x)*sin(pi*y)"), .u_t = e("0")}},
      8);
  const auto result =
      integrator(std::make_shared<Newmark>(), 0.05)
          .integrate(m, 0.0, 5.0,
                     {.snapshot_times = {0.0}, .snapshot_every = 1});
  check(result.snapshots.size() == 101, "the start and a snapshot per step");
  const AssembledProblem a = Assembler().assemble(m, 0.0);
  const double e0 = energy(a, *m, result.snapshots[0]);
  double largest = 0.0;
  for (const auto &s : result.snapshots)
    largest = std::max(largest, std::abs(energy(a, *m, s) - e0) / e0);
  std::cout << "newmark: largest relative energy change " << largest << '\n';
  check(e0 > 0.0, "positive energy");
  check(largest < 1e-10, "energy conserved");
}

void counts_prepares() {
  // Constant matrices, a load and Dirichlet data in t: prepared once.
  for (const auto &method : std::vector<std::shared_ptr<const TimeMethod>>{
           std::make_shared<ImplicitEuler>(),
           std::make_shared<ImplicitMidpoint>()}) {
    const auto solver = std::make_shared<pde2d::test::CountingSolver>();
    const auto result = integrator(method, 0.1, solver)
                            .integrate(meshed(constant_heat_problem()), 0.0,
                                       1.0);
    check(result.report.metrics.at("steps") == 10, "10 steps");
    check(result.report.metrics.at("prepares") == 1, "prepared once");
    check(solver->count() == 1, "the solver saw one prepare");
    check(result.report.metrics.at("assemblies") == 11, "11 assemblies");
  }
  // Stiffness and damping depend on t: prepared every step.
  const auto solver = std::make_shared<pde2d::test::CountingSolver>();
  const auto result = integrator(std::make_shared<ImplicitEuler>(), 0.1,
                                 solver)
                          .integrate(meshed(heat_problem()), 0.0, 1.0);
  check(result.report.metrics.at("prepares") == 10, "prepared every step");
  check(solver->count() == 10, "the solver saw ten prepares");
  // Newmark prepares once more in start(), for M2.
  const auto wave_solver = std::make_shared<pde2d::test::CountingSolver>();
  const auto wave = integrator(std::make_shared<Newmark>(), 0.1, wave_solver)
                        .integrate(meshed(wave_problem()), 0.0, 1.0);
  check(wave.report.metrics.at("prepares") == 1, "one step matrix");
  check(wave_solver->count() == 2, "and one mass matrix");
}

void refuses_time_orders() {
  expect_throws<std::invalid_argument>(
      [] {
        (void)integrator(std::make_shared<Newmark>(), 0.1)
            .integrate(meshed(heat_problem()), 0.0, 1.0);
      },
      "newmark does not support time order 1");
  expect_throws<std::invalid_argument>(
      [] {
        (void)integrator(std::make_shared<ImplicitEuler>(), 0.1)
            .integrate(meshed(wave_problem()), 0.0, 1.0);
      },
      "implicit_euler does not support time order 2");
  expect_throws<std::invalid_argument>(
      [] {
        (void)integrator(std::make_shared<ImplicitMidpoint>(), 0.1)
            .integrate(meshed(wave_problem()), 0.0, 1.0);
      },
      "implicit_midpoint does not support time order 2");
  Problem steady = constant_heat_problem();
  steady.equation.m1 = Expression();
  expect_throws<std::invalid_argument>(
      [&] {
        (void)integrator(std::make_shared<ImplicitEuler>(), 0.1)
            .integrate(meshed(steady), 0.0, 1.0);
      },
      "time order 0");
}

void lands_on_snapshots() {
  const auto m = meshed(constant_heat_problem());
  const auto result = integrator(std::make_shared<ImplicitEuler>(), 0.1)
                          .integrate(m, 0.0, 1.0,
                                     {.keep_step_history = true,
                                      .snapshot_times = {0.0, 0.25, 0.3, 1.0}});
  check(result.snapshots.size() == 4, "four snapshots");
  const std::vector<double> times = {0.0, 0.25, 0.3, 1.0};
  for (std::size_t i = 0; i < 4; ++i)
    check(result.snapshots[i].time == times[i], "exactly at the time");
  check(result.snapshots[0].values == *m->initial_values(0.0),
        "the snapshot at t0 is the starting state");
  check(result.snapshots[3].values == result.final.values,
        "the snapshot at t1 is the final solution");
  // 0.1, 0.2, 0.25, 0.3, then 0.4 ... 1.0.
  check(result.history.size() == 11, "11 steps");
  check(result.history[2].t == 0.25 && result.history[3].t == 0.3,
        "steps land on the snapshot times");
  near(result.history[2].dt, 0.05, 1e-15, "a shortened step");
  for (std::size_t i = 4; i < 11; ++i)
    near(result.history[i].t, 0.3 + 0.1 * static_cast<double>(i - 3), 1e-14,
         "full steps after the snapshot");
  check(result.history.back().t == 1.0, "the last step lands on t1");
  check(result.report.metrics.at("snapshots") == 4, "reported");
  check(result.report.metrics.at("min_dt") < 0.0500001, "min_dt");
  check(result.report.metrics.at("max_dt") == 0.1, "max_dt");

  // Every third step, and once at 0.6 although both rules ask for it.
  const auto every =
      integrator(std::make_shared<ImplicitEuler>(), 0.1)
          .integrate(m, 0.0, 1.0,
                     {.snapshot_times = {0.6}, .snapshot_every = 3});
  check(every.snapshots.size() == 3, "steps 3, 6 and 9");
  check(every.snapshots[1].time == 0.6, "the snapshot time");
  near(every.snapshots[0].time, 0.3, 1e-14, "step 3");
  near(every.snapshots[2].time, 0.9, 1e-14, "step 9");
  check(every.history.empty(), "no history unless asked");
}

void continues_runs() {
  // First order: bit for bit, when the long run lands on the split time.
  for (const auto &method : std::vector<std::shared_ptr<const TimeMethod>>{
           std::make_shared<ImplicitEuler>(),
           std::make_shared<ImplicitMidpoint>()}) {
    for (const auto &problem : {heat_problem(), constant_heat_problem()}) {
      const auto m = meshed(problem);
      const auto i = integrator(method, 0.1);
      const auto whole = i.integrate(m, 0.0, 1.0, {.snapshot_times = {0.5}});
      const auto first = i.integrate(m, 0.0, 0.5);
      const auto second = i.integrate(m, 0.5, 1.0, {}, &first.final);
      check(first.final.values == whole.snapshots[0].values, "first half");
      check(second.final.values == whole.final.values,
            "continued run equals one long run");
    }
  }
  // Newmark: the acceleration is computed again from the equation at the
  // split time, so equal to rounding.
  const auto m = meshed(wave_problem());
  const auto i = integrator(std::make_shared<Newmark>(), 0.1);
  const auto whole = i.integrate(m, 0.0, 1.0, {.snapshot_times = {0.5}});
  const auto first = i.integrate(m, 0.0, 0.5);
  check(!first.final.velocities.empty(), "the final solution has velocities");
  const auto second = i.integrate(m, 0.5, 1.0, {}, &first.final);
  for (std::size_t k = 0; k < whole.final.values.size(); ++k) {
    near(second.final.values[k], whole.final.values[k], 1e-12, "values");
    near(second.final.velocities[k], whole.final.velocities[k], 1e-11,
         "velocities");
  }
}

void checks_input() {
  const auto m = meshed(constant_heat_problem());
  const auto i = integrator(std::make_shared<ImplicitEuler>(), 0.1);
  expect_throws<std::invalid_argument>(
      [&] { (void)i.integrate(nullptr, 0.0, 1.0); }, "problem is null");
  expect_throws<std::invalid_argument>(
      [&] { (void)i.integrate(m, 1.0, 1.0); }, "greater than t0");
  expect_throws<std::invalid_argument>(
      [&] { (void)i.integrate(m, 0.0, std::nan("")); }, "finite");
  expect_throws<std::invalid_argument>(
      [&] { (void)i.integrate(m, 0.0, 1.0, {.snapshot_times = {1.5}}); },
      "not within [t0, t1]");
  expect_throws<std::invalid_argument>(
      [&] {
        (void)i.integrate(m, 0.0, 1.0, {.snapshot_times = {0.5, 0.5}});
      },
      "strictly increasing");
  // An initial solution on another mesh object (equal contents).
  const auto other = meshed(constant_heat_problem());
  const auto start = i.integrate(other, 0.0, 0.1).final;
  expect_throws<std::invalid_argument>(
      [&] { (void)i.integrate(m, 0.1, 0.2, {}, &start); }, "another mesh");
  // Time order 2 needs velocities: a static solution has none.
  const auto wave = meshed(wave_problem());
  const auto steady = StaticSolver(std::make_shared<SparseLu>()).solve(wave);
  expect_throws<std::invalid_argument>(
      [&] {
        (void)integrator(std::make_shared<Newmark>(), 0.1)
            .integrate(wave, 0.0, 1.0, {}, &steady.solution);
      },
      "velocities");
  expect_throws<std::invalid_argument>(
      [] { FixedStep({.dt = 0.0}); }, "dt must be finite and positive");
  expect_throws<std::invalid_argument>(
      [] {
        Integrator(nullptr, std::make_shared<FixedStep>(FixedStepOptions{.dt = 1}),
                   std::make_shared<SparseLu>());
      },
      "time method is null");
}

} // namespace

int main() {
  return pde2d::test::run({
      {"implicit Euler converges at first order in time",
       [] {
         time_rate(std::make_shared<ImplicitEuler>(), heat_problem(),
                   heat_exact, 0.85, 1.2);
       }},
      {"implicit midpoint converges at second order in time",
       [] {
         time_rate(std::make_shared<ImplicitMidpoint>(), heat_problem(),
                   heat_exact, 1.85, 2.2);
       }},
      {"a singular damping matrix (m1 = 0, a dynamic condition)",
       [] {
         time_rate(std::make_shared<ImplicitEuler>(),
                   dynamic_boundary_problem(), heat_exact, 0.85, 1.2);
         time_rate(std::make_shared<ImplicitMidpoint>(),
                   dynamic_boundary_problem(), heat_exact, 1.85, 2.5);
       }},
      {"Newmark converges at second order in time",
       [] {
         time_rate(std::make_shared<Newmark>(), wave_problem(), wave_exact,
                   1.85, 2.2);
       }},
      {"BiCgStab gives the same time integration as SparseLu",
       [] {
         const auto m = meshed(heat_problem());
         const auto lu = integrator(std::make_shared<ImplicitMidpoint>(), 0.1)
                             .integrate(m, 0.0, 1.0);
         const auto it =
             integrator(std::make_shared<ImplicitMidpoint>(), 0.1,
                        std::make_shared<BiCgStab>(
                            BiCgStabOptions{.tolerance = 1e-13}))
                 .integrate(m, 0.0, 1.0);
         for (std::size_t k = 0; k < lu.final.values.size(); ++k)
           near(it.final.values[k], lu.final.values[k], 1e-10, "same values");
         check(it.report.metrics.at("linear_iterations") > 0, "iterations");
       }},
      {"Newmark conserves the energy of an undamped wave",
       energy_is_conserved},
      {"the solver is prepared only when the step matrix changes",
       counts_prepares},
      {"methods are refused for time orders they do not support",
       refuses_time_orders},
      {"snapshots land exactly on their times", lands_on_snapshots},
      {"a continued run equals one long run", continues_runs},
      {"the integrator checks its input", checks_input},
  });
}
