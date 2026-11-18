#include "solve_support.hpp"

#include "pde2d/fem/assembler.hpp"
#include "pde2d/solve/bicgstab.hpp"
#include "pde2d/solve/conjugate_gradient.hpp"
#include "pde2d/solve/fixed_step.hpp"
#include "pde2d/solve/implicit_euler.hpp"
#include "pde2d/solve/implicit_midpoint.hpp"
#include "pde2d/solve/integrator.hpp"
#include "pde2d/solve/newmark.hpp"
#include "pde2d/solve/simplicial_ldlt.hpp"
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
          .boundary_conditions = {{"bottom", dirichlet},
                                  {"left", dirichlet},
                                  {"right",
                                   {.beta_n = e("1"),
                                    .gamma = e("1"),
                                    .g = e("cos(2*t) + sin(t) + " + u_t)}},
                                  {"top",
                                   {.beta_n = e("1"),
                                    .g = e("2*cos(2*t) - sin(t)")}}},
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

// The same u for sin(t)^2 u_tt + 0.1 u_t - (u_xx + u_yy) = f: M2 is zero
// at t = 0, so Newmark cannot compute a starting acceleration.
Problem vanishing_mass_problem() {
  Problem problem = wave_problem();
  const std::string u_t = "-2*" + p + "*sin(2*t) + 3*" + q + "*cos(3*t)";
  const std::string u_tt = "-4*" + p + "*cos(2*t) - 9*" + q + "*sin(3*t)";
  problem.equation.m2 = e("sin(t)^2");
  problem.equation.f = e("sin(t)^2*(" + u_tt + ") + 0.1*(" + u_t + ")");
  return problem;
}

// Time order 1 as written, 0 as assembled: m1 = 0 and the u_t term is in
// a dynamic condition, but A = 0, so the boundary terms (D among them)
// vanish. u + 0 u_t = 2 + t, so u = 2 + t everywhere.
Problem vanished_time_terms_problem() {
  const BoundaryCondition natural{.beta_n = e("1")};
  return {
      .equation = {.c = e("1"), .f = e("2 + t")},
      .boundary_conditions = {{"bottom", {.beta_n = e("1"), .gamma = e("1")}},
                              {"right", natural},
                              {"top", natural},
                              {"left", natural}},
      .initial = {.u = e("2")}};
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

// The time convergence rates of `method` on [0, 1] with the steps `dts`
// (by default 0.05, 0.025, 0.0125), from the largest nodal error at
// t = 0.25, 0.5, 0.75 and 1 (one time alone can sit near a zero of the
// error).
void time_rate(const std::shared_ptr<const TimeMethod> &method,
               const Problem &problem, const Exact &exact, double low,
               double high,
               const std::vector<double> &dts = {0.05, 0.025, 0.0125}) {
  const auto m = meshed(problem);
  std::vector<double> errors;
  for (const double dt : dts) {
    const auto result =
        integrator(method, dt)
            .integrate(m, 0.0, 1.0, {.snapshot_times = {0.25, 0.5, 0.75, 1.0}});
    check(result.final.time == 1.0, "ends at t1");
    double error = 0.0;
    for (const auto &s : result.snapshots)
      error =
          std::max(error, pde2d::test::max_nodal_error(
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

void energy_is_conserved(const std::shared_ptr<const TimeMethod> &method) {
  const BoundaryCondition zero{.alpha = e("1")};
  const auto m =
      meshed({.equation = {.a_xx = e("-1"), .a_yy = e("-1"), .m2 = e("1")},
              .boundary_conditions = {{"bottom", zero},
                                      {"right", zero},
                                      {"top", zero},
                                      {"left", zero}},
              .initial = {.u = e("sin(pi*x)*sin(pi*y)"), .u_t = e("0")}},
             8);
  const auto result =
      integrator(method, 0.05)
          .integrate(m, 0.0, 5.0,
                     {.snapshot_times = {0.0}, .snapshot_every = 1});
  check(result.snapshots.size() == 101, "the start and a snapshot per step");
  const AssembledProblem a = Assembler().assemble(m, 0.0);
  const double e0 = energy(a, *m, result.snapshots[0]);
  double largest = 0.0;
  for (const auto &s : result.snapshots)
    largest = std::max(largest, std::abs(energy(a, *m, s) - e0) / e0);
  std::cout << method->name() << ": largest relative energy change " << largest
            << '\n';
  check(e0 > 0.0, "positive energy");
  check(largest < 1e-10, "energy conserved");
}

void counts_prepares() {
  // Constant matrices, a load and Dirichlet data in t: prepared once.
  for (const auto &method : std::vector<std::shared_ptr<const TimeMethod>>{
           std::make_shared<ImplicitEuler>(),
           std::make_shared<ImplicitMidpoint>()}) {
    const auto solver = std::make_shared<pde2d::test::CountingSolver>();
    const auto result =
        integrator(method, 0.1, solver)
            .integrate(meshed(constant_heat_problem()), 0.0, 1.0);
    check(result.report.metrics.at("steps") == 10, "10 steps");
    check(result.report.metrics.at("prepares") == 1, "prepared once");
    check(solver->count() == 1, "the solver saw one prepare");
    // One per step: their start reads nothing, so nothing is assembled at
    // t0.
    check(result.report.metrics.at("assemblies") == 10, "10 assemblies");
  }
  // Stiffness and damping depend on t: prepared every step.
  const auto solver = std::make_shared<pde2d::test::CountingSolver>();
  const auto result = integrator(std::make_shared<ImplicitEuler>(), 0.1, solver)
                          .integrate(meshed(heat_problem()), 0.0, 1.0);
  check(result.report.metrics.at("prepares") == 10, "prepared every step");
  check(solver->count() == 10, "the solver saw ten prepares");
  // Newmark prepares once more in start(), for M2.
  const auto wave_solver = std::make_shared<pde2d::test::CountingSolver>();
  const auto wave = integrator(std::make_shared<Newmark>(), 0.1, wave_solver)
                        .integrate(meshed(wave_problem()), 0.0, 1.0);
  check(wave.report.metrics.at("prepares") == 1, "one step matrix");
  check(wave_solver->count() == 2, "and one mass matrix");
  check(wave.report.metrics.at("assemblies") == 11,
        "Newmark's start reads the whole problem at t0: 11 assemblies");
}

void refuses_time_orders() {
  check(ImplicitEuler().capabilities().supports(0) &&
            ImplicitEuler().capabilities().supports(1) &&
            ImplicitEuler().capabilities().supports(2),
        "implicit_euler: 0, 1 and 2");
  check(!ImplicitMidpoint().capabilities().supports(0) &&
            ImplicitMidpoint().capabilities().supports(1) &&
            ImplicitMidpoint().capabilities().supports(2),
        "implicit_midpoint: 1 and 2");
  check(!Newmark().capabilities().supports(0) &&
            !Newmark().capabilities().supports(1) &&
            Newmark().capabilities().supports(2),
        "newmark: 2");
  check(!ImplicitEuler().capabilities().supports(3) &&
            !ImplicitEuler().capabilities().supports(-1),
        "no other orders");
  expect_throws<std::invalid_argument>(
      [] {
        (void)integrator(std::make_shared<Newmark>(), 0.1)
            .integrate(meshed(heat_problem()), 0.0, 1.0);
      },
      "integrator: newmark does not support time order 1");
  // Time order 0 as written: no word about vanished terms.
  Problem steady = constant_heat_problem();
  steady.equation.m1 = Expression();
  steady.initial = {};
  for (const auto &method : std::vector<std::shared_ptr<const TimeMethod>>{
           std::make_shared<ImplicitMidpoint>(), std::make_shared<Newmark>()})
    try {
      (void)integrator(method, 0.1).integrate(meshed(steady), 0.0, 1.0);
      check(false, "time order 0 is refused");
    } catch (const std::invalid_argument &error) {
      const std::string what = error.what();
      check(what.find(method->name() + " does not support time order 0") !=
                std::string::npos,
            "names the method and the order");
      check(what.find("vanished") == std::string::npos,
            "nothing vanished: the problem is written with order 0");
    }
  // Time order 1 as written, 0 as assembled.
  const auto vanished = meshed(vanished_time_terms_problem());
  check(time_order(*vanished) == 1 &&
            Assembler().assemble(vanished, 0.0).time_order() == 0,
        "order 1 as written, 0 as assembled");
  for (const auto &method : std::vector<std::shared_ptr<const TimeMethod>>{
           std::make_shared<ImplicitMidpoint>(), std::make_shared<Newmark>()})
    expect_throws<std::invalid_argument>(
        [&] { (void)integrator(method, 0.1).integrate(vanished, 0.0, 1.0); },
        method->name() +
            " does not support time order 0; the problem is written with "
            "time order 1, but its time terms vanished in assembly");
}

// Time order 0 with implicit Euler: each step is a static solve at its
// end time, and the state at t0 is the static solution there.
void order_zero_is_static() {
  Problem steady = constant_heat_problem();
  steady.equation.m1 = Expression();
  steady.initial = {};
  for (const auto &problem : {steady, vanished_time_terms_problem()}) {
    const auto m = meshed(problem);
    const auto result =
        integrator(std::make_shared<ImplicitEuler>(), 0.1)
            .integrate(m, 0.0, 1.0,
                       {.snapshot_times = {0.0}, .snapshot_every = 1});
    check(result.report.metrics.at("time_order") == 0, "time order 0");
    check(result.snapshots.size() == 11, "t0 and ten steps");
    const StaticSolver static_solver(std::make_shared<SparseLu>());
    for (const auto &snapshot : result.snapshots) {
      const auto expected = static_solver.solve(m, snapshot.time).solution;
      for (std::size_t k = 0; k < expected.values.size(); ++k)
        near(snapshot.values[k], expected.values[k], 1e-12,
             "a static solve at t = " + std::to_string(snapshot.time));
    }
  }
  // The second problem's solution is u = 2 + t.
  const auto m = meshed(vanished_time_terms_problem());
  const auto result = integrator(std::make_shared<ImplicitEuler>(), 0.25)
                          .integrate(m, 0.0, 1.0);
  for (const double value : result.final.values)
    near(value, 3.0, 1e-12, "u = 2 + t at t = 1");
}

// M2 is zero at t0 = 0 (m2 = sin(t)^2): Newmark cannot start and says
// why; the midpoint method needs no acceleration and keeps its order.
void vanishing_mass() {
  try {
    (void)integrator(std::make_shared<Newmark>(), 0.05)
        .integrate(meshed(vanishing_mass_problem()), 0.0, 1.0);
    check(false, "Newmark cannot start");
  } catch (const std::runtime_error &error) {
    const std::string what = error.what();
    std::cout << "newmark: " << what << '\n';
    for (const char *part :
         {"newmark: the starting acceleration cannot be computed",
          "M2 is zero at t0 = 0", "ImplicitMidpoint", "ImplicitEuler",
          "need no acceleration"})
      check(what.find(part) != std::string::npos,
            std::string("the message says ") + part);
  }
  // M2 not zero but singular at t0: m2 is zero for x >= 0.3 at t = 0, so
  // the rows of the nodes at x = 0.75 and 1 are zero and SparseLu fails.
  Problem singular = vanishing_mass_problem();
  singular.equation.m2 = e("Piecewise((1 + t, x < 0.3), (t, True))");
  try {
    (void)integrator(std::make_shared<Newmark>(), 0.05)
        .integrate(meshed(singular), 0.0, 1.0);
    check(false, "Newmark cannot start");
  } catch (const std::runtime_error &error) {
    const std::string what = error.what();
    std::cout << "newmark: " << what << '\n';
    check(what.find("starting acceleration cannot be computed") !=
                  std::string::npos &&
              what.find("M2 is singular at t0 = 0 (solving with it failed: "
                        "sparse_lu") != std::string::npos &&
              what.find("ImplicitMidpoint") != std::string::npos,
          "names the cause, the solver's error and the other methods");
  }
  // Near t = 0, B dt / M2 is about 0.4 / dt: the step is stiff there, and
  // midpoint (not L-stable) shows irregular rates (1.3 to 2.5) down to
  // dt = 0.003; below it the rate is 2.
  time_rate(std::make_shared<ImplicitMidpoint>(), vanishing_mass_problem(),
            wave_exact, 1.9, 2.1, {0.003125, 0.0015625, 0.00078125});
  time_rate(std::make_shared<ImplicitEuler>(), vanishing_mass_problem(),
            wave_exact, 0.85, 1.3);
}

// The step of the second-order midpoint method with M2 = 0 is the
// first-order midpoint step: m2 = sin(t)^2 assembled at t = 0 (M2 is
// zero there, but not a zero operator) against the same problem without
// m2, one step from the same U0.
void midpoint_without_mass() {
  Problem first_order = vanishing_mass_problem();
  first_order.equation.m2 = Expression();
  first_order.initial.u_t.reset();
  const auto second = meshed(vanishing_mass_problem());
  const auto first = meshed(first_order);
  const AssembledProblem a2 = Assembler().assemble(second, 0.0);
  const AssembledProblem a1 = Assembler().assemble(first, 0.0);
  check(a2.time_order() == 2 && a1.time_order() == 1, "orders 2 and 1");
  const ImplicitMidpoint method;
  const double dt = 0.1;
  const StepMatrixWeights w = method.step_matrix_weights(dt);
  // weights.mass M2 + weights.damping B + weights.stiffness A, by hand.
  const auto matrix = [&](const AssembledProblem &a) {
    auto values = a.stiffness().free.values;
    for (std::size_t s = 0; s < values.size(); ++s)
      values[s] = w.mass * a.mass().free.values[s] +
                  w.damping * a.damping().free.values[s] +
                  w.stiffness * a.stiffness().free.values[s];
    return std::make_shared<const CsrMatrix>(
        CsrMatrix{.pattern = a.stiffness().free.pattern, .values = values});
  };
  const std::size_t n = second->free_nodes().size();
  TimeState s2{.values = std::vector<double>(n, 0.0),
               .velocities = std::vector<double>(n, 0.0)};
  for (std::size_t i = 0; i < n; ++i) {
    s2.values[i] = std::sin(static_cast<double>(i));
    s2.velocities[i] = std::cos(static_cast<double>(i));
  }
  TimeState s1{.values = s2.values};
  (void)method.advance(a2, dt, *SparseLu().prepare(matrix(a2)), s2);
  (void)method.advance(a1, dt, *SparseLu().prepare(matrix(a1)), s1);
  check(s2.values == s1.values, "the same U1, bit for bit");
}

// Linear solvers that require symmetry: accepted on the symmetric heat
// problems, refused (before preparing) on a non-symmetric one.
void symmetric_solvers() {
  for (const auto &problem : {heat_problem(), constant_heat_problem()}) {
    const auto m = meshed(problem);
    check(Assembler().assemble(m, 0.0).stiffness().always_symmetric,
          "isotropic heat is symmetric");
    const auto lu = integrator(std::make_shared<ImplicitMidpoint>(), 0.1)
                        .integrate(m, 0.0, 1.0);
    for (const std::shared_ptr<const LinearSolver> &solver :
         std::vector<std::shared_ptr<const LinearSolver>>{
             std::make_shared<ConjugateGradient>(
                 ConjugateGradientOptions{.tolerance = 1e-13}),
             std::make_shared<SimplicialLdlt>()}) {
      const auto other =
          integrator(std::make_shared<ImplicitMidpoint>(), 0.1, solver)
              .integrate(m, 0.0, 1.0);
      for (std::size_t k = 0; k < lu.final.values.size(); ++k)
        near(other.final.values[k], lu.final.values[k], 1e-10,
             solver->name() + " gives the SparseLu result");
    }
  }
  // Newmark with CG: M2 (start) and the step matrix are symmetric.
  const auto wave =
      integrator(std::make_shared<Newmark>(), 0.1,
                 std::make_shared<ConjugateGradient>(
                     ConjugateGradientOptions{.tolerance = 1e-13}))
          .integrate(meshed(wave_problem()), 0.0, 1.0);
  const auto wave_lu = integrator(std::make_shared<Newmark>(), 0.1)
                           .integrate(meshed(wave_problem()), 0.0, 1.0);
  for (std::size_t k = 0; k < wave.final.values.size(); ++k)
    near(wave.final.values[k], wave_lu.final.values[k], 1e-9, "Newmark, CG");
  // Convection makes A, and so every step matrix, non-symmetric.
  Problem convected = constant_heat_problem();
  convected.equation.b_x = e("1");
  const auto solver = std::make_shared<pde2d::test::CountingSolver>();
  expect_throws<std::invalid_argument>(
      [&] {
        (void)integrator(std::make_shared<ImplicitEuler>(), 0.1,
                         std::make_shared<SimplicialLdlt>())
            .integrate(meshed(convected), 0.0, 1.0);
      },
      "integrator: the linear solver simplicial_ldlt requires a symmetric "
      "matrix, but the step matrix of implicit_euler is not symmetric");
}

// An anisotropic diagonal A (a_xx = -1, a_yy = -3) with natural conditions
// on the unit square: A is not symmetric by the facts (T is not proven
// zero), but T vanishes on the axis-aligned edges, so every step matrix is
// symmetric by its values and CG is accepted. With a_xy or a convection it
// is refused.
Problem anisotropic_heat_problem(const std::string &a_xx = "-1") {
  const BoundaryCondition robin{
      .alpha = e("1"), .beta_n = e("1"), .g = e("1 + t")};
  return {.equation = {.a_xx = e(a_xx),
                       .a_yy = e("-3"),
                       .c = e("1"),
                       .m1 = e("1"),
                       .f = e("1 + sin(3*t)*x")},
          .boundary_conditions = {{"bottom", robin},
                                  {"right", robin},
                                  {"top", {.beta_n = e("1"), .g = e("y")}},
                                  {"left", robin}},
          .initial = {.u = e("x*y")}};
}

void symmetric_now_solvers() {
  const auto cg = std::make_shared<ConjugateGradient>(
      ConjugateGradientOptions{.tolerance = 1e-13});
  // Constant A (prepared once) and A in t (prepared every step).
  for (const char *a_xx : {"-1", "-(1 + t)"}) {
    const auto m = meshed(anisotropic_heat_problem(a_xx));
    const auto assembled = Assembler().assemble(m, 0.5);
    check(!assembled.stiffness().always_symmetric &&
              assembled.stiffness().symmetric_now,
          "A: not always symmetric, symmetric now");
    for (const auto &method : std::vector<std::shared_ptr<const TimeMethod>>{
             std::make_shared<ImplicitEuler>(),
             std::make_shared<ImplicitMidpoint>()}) {
      const auto lu = integrator(method, 0.1).integrate(m, 0.0, 1.0);
      const auto other = integrator(method, 0.1, cg).integrate(m, 0.0, 1.0);
      for (std::size_t k = 0; k < lu.final.values.size(); ++k)
        near(other.final.values[k], lu.final.values[k], 1e-10,
             method->name() + " with CG gives the SparseLu result");
    }
  }
  // a_xy or a convection: the step matrix is not symmetric by its values.
  Problem cross = anisotropic_heat_problem();
  cross.equation.a_xy = e("0.5");
  Problem convected = anisotropic_heat_problem();
  convected.equation.b_y = e("1");
  for (const auto &problem : {cross, convected})
    expect_throws<std::invalid_argument>(
        [&] {
          (void)integrator(std::make_shared<ImplicitEuler>(), 0.1, cg)
              .integrate(meshed(problem), 0.0, 1.0);
        },
        "integrator: the linear solver conjugate_gradient requires a "
        "symmetric matrix, but the step matrix of implicit_euler is not "
        "symmetric, neither by assembly's facts nor by its values");
}

// u = sqrt(t) + x solves u_t - (u_xx + u_yy) = 1 / (2 sqrt(t)), with
// Dirichlet data on the bottom and the left, from t0 = 0. d/dt sqrt(t) is
// not finite at 0: the methods whose start reads nothing at t0 must never
// assemble there.
double root_exact(double x, double, double t) { return std::sqrt(t) + x; }

Problem root_problem() {
  const BoundaryCondition dirichlet{.alpha = e("1"), .g = e("sqrt(t) + x")};
  return {.equation = {.a_xx = e("-1"),
                       .a_yy = e("-1"),
                       .m1 = e("1"),
                       .f = e("1/(2*sqrt(t))")},
          .boundary_conditions = {{"bottom", dirichlet},
                                  {"left", dirichlet},
                                  {"right", {.beta_n = e("1"), .g = e("1")}},
                                  {"top", {.beta_n = e("1"), .g = e("0")}}},
          .initial = {.u = e("sqrt(t) + x")}};
}

void starts_without_assembling() {
  const auto m = meshed(root_problem());
  // A full assembly at t0 would evaluate U_D'(0).
  expect_throws<std::invalid_argument>(
      [&] { (void)Assembler().assemble(m, 0.0); }, "not finite");
  check(ImplicitEuler().start_needs(1) == StartNeeds::nothing &&
            ImplicitEuler().start_needs(2) == StartNeeds::nothing &&
            ImplicitEuler().start_needs(0) == StartNeeds::static_part &&
            ImplicitMidpoint().start_needs(1) == StartNeeds::nothing &&
            ImplicitMidpoint().start_needs(2) == StartNeeds::nothing &&
            Newmark().start_needs(2) == StartNeeds::whole_problem,
        "the declared start needs");
  for (const auto &method : std::vector<std::shared_ptr<const TimeMethod>>{
           std::make_shared<ImplicitEuler>(),
           std::make_shared<ImplicitMidpoint>()}) {
    std::vector<double> errors;
    for (const double dt : {0.01, 0.005}) {
      const auto result =
          integrator(method, dt)
              .integrate(m, 0.0, 1.0, {.snapshot_times = {0.0}});
      check(result.report.metrics.at("assemblies") ==
                result.report.metrics.at("steps"),
            method->name() + ": one assembly per step, none at t0");
      near(pde2d::test::max_nodal_error(
               *m->mesh(), result.snapshots[0].values,
               [](double x, double y) { return root_exact(x, y, 0.0); }),
           0.0, 1e-15, method->name() + ": the state at t0 is u(0)");
      errors.push_back(pde2d::test::max_nodal_error(
          *m->mesh(), result.final.values,
          [](double x, double y) { return root_exact(x, y, 1.0); }));
    }
    std::cout << method->name() << " from t0 = 0 with u = sqrt(t): error at "
              << "t = 1 " << errors[0] << " -> " << errors[1] << '\n';
    check(errors[0] < 5e-3, method->name() + ": close to sqrt(t) + x");
    check(errors[1] < errors[0], method->name() + ": better with smaller dt");
  }
  // Newmark's start reads the whole problem at t0, so a second-order
  // problem with the same data cannot start at 0 (U_D'' is not finite).
  Problem wave = root_problem();
  wave.equation.m2 = e("1");
  wave.initial.u_t = e("0");
  expect_throws<std::invalid_argument>(
      [&] {
        (void)integrator(std::make_shared<Newmark>(), 0.1)
            .integrate(meshed(wave), 0.0, 1.0);
      },
      "not finite");
}

// sin(t) u_tt + 0.1 u_t - (u_xx + u_yy) = f with the wave's u: M2 is zero at
// t0 = 0, though not a zero operator (time order 2 also at t = 0). Newmark
// refuses at its start; the midpoint method starts without reading M2.
void mass_zero_at_start() {
  Problem problem = wave_problem();
  const std::string u_t = "-2*" + p + "*sin(2*t) + 3*" + q + "*cos(3*t)";
  const std::string u_tt = "-4*" + p + "*cos(2*t) - 9*" + q + "*sin(3*t)";
  problem.equation.m2 = e("sin(t)");
  problem.equation.f = e("sin(t)*(" + u_tt + ") + 0.1*(" + u_t + ")");
  const auto m = meshed(problem);
  const auto at_zero = Assembler().assemble(m, 0.0);
  check(Assembler().time_order(*m) == 2 && at_zero.time_order() == 2 &&
            !at_zero.mass().always_zero && at_zero.mass().zero_now,
        "time order 2, M2 zero now at t = 0");
  try {
    (void)integrator(std::make_shared<Newmark>(), 0.01).integrate(m, 0.0, 1.0);
    check(false, "Newmark cannot start");
  } catch (const std::runtime_error &error) {
    const std::string what = error.what();
    for (const char *part :
         {"newmark: the starting acceleration cannot be computed",
          "M2 is zero at t0 = 0", "implicit_midpoint", "implicit_euler"})
      check(what.find(part) != std::string::npos,
            std::string("the message says ") + part);
  }
  const auto result = integrator(std::make_shared<ImplicitMidpoint>(), 0.001)
                          .integrate(m, 0.0, 1.0);
  const double error = pde2d::test::max_nodal_error(
      *m->mesh(), result.final.values,
      [](double x, double y) { return wave_exact(x, y, 1.0); });
  std::cout << "implicit_midpoint with m2 = sin(t) from t0 = 0: error at t = 1 "
            << error << '\n';
  check(error < 1e-3, "the midpoint method follows the solution");
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

// max_steps ends a run early; before_last_step is the state one step
// before the end. Steps of 1/16 are exact in binary, so the times compare
// exactly.
void stops_after_max_steps() {
  for (const auto &problem : {heat_problem(), wave_problem()}) {
    const auto m = meshed(problem);
    const auto i = integrator(std::make_shared<ImplicitMidpoint>(), 1.0 / 16);
    const auto whole = i.integrate(
        m, 0.0, 1.0, {.keep_step_history = true, .snapshot_every = 1});
    const auto part = i.integrate(m, 0.0, 1.0,
                                  {.keep_step_history = true,
                                   .snapshot_times = {0.25, 0.75},
                                   .keep_before_last_step = true},
                                  nullptr, 5);
    check(part.final.time == 5.0 / 16, "ends after 5 steps");
    check(part.history.size() == 5, "5 step records");
    check(part.report.metrics.at("t_end") == 5.0 / 16, "t_end");
    check(part.report.metrics.at("t1") == 1.0, "t1 as asked");
    check(part.snapshots.size() == 1 && part.snapshots[0].time == 0.25,
          "only the snapshot times reached");
    check(part.final.values == whole.snapshots[4].values,
          "the same state as the long run after 5 steps");
    check(part.before_last_step && part.before_last_step->time == 4.0 / 16,
          "the state before the last step");
    // In the free unknowns: no boundary data is evaluated for it.
    const auto &free_nodes = m->free_nodes();
    const Solution &after_4 = whole.snapshots[3];
    const FreeState &before = *part.before_last_step;
    check(before.values.size() == free_nodes.size() &&
              before.velocities.size() ==
                  (after_4.velocities.empty() ? 0 : free_nodes.size()),
          "free values, and free velocities for time order 2 only");
    for (std::size_t k = 0; k < free_nodes.size(); ++k) {
      check(before.values[k] == after_4.values[free_nodes[k]],
            "the values after 4 steps");
      if (!before.velocities.empty())
        check(before.velocities[k] == after_4.velocities[free_nodes[k]],
              "the velocities after 4 steps");
    }
    // Reaching t1 first: max_steps does not matter; before_last_step is
    // the start of the step that landed on t1.
    const auto reached =
        i.integrate(m, 0.0, 1.0, {.keep_before_last_step = true}, nullptr, 100);
    check(reached.final.time == 1.0, "t1 reached before max_steps");
    check(reached.before_last_step->time == 15.0 / 16, "the last step's start");
    check(reached.final.values == whole.final.values, "the same final state");
    check(!whole.before_last_step, "not kept unless asked");
    expect_throws<std::invalid_argument>(
        [&] { (void)i.integrate(m, 0.0, 1.0, {}, nullptr, 0); },
        "max_steps must be at least 1");
  }
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
  // Newmark keeps its acceleration in the Solutions it returns, and a run
  // continued from one takes it instead of calling start(): bit for bit,
  // like the first-order methods.
  const auto m = meshed(wave_problem());
  const auto i = integrator(std::make_shared<Newmark>(), 0.1);
  const auto whole = i.integrate(m, 0.0, 1.0, {.snapshot_times = {0.5}});
  const auto first = i.integrate(m, 0.0, 0.5);
  check(!first.final.velocities.empty(), "the final solution has velocities");
  check(first.final.accelerations.size() == m->mesh()->nodes.size() &&
            whole.snapshots[0].accelerations.size() == m->mesh()->nodes.size(),
        "and accelerations, also the snapshots");
  // At the Dirichlet nodes the boundary data's acceleration.
  for (std::size_t k = 0; k < m->dirichlet_nodes().size(); ++k)
    check(first.final.accelerations[m->dirichlet_nodes()[k].node] ==
              m->dirichlet_acceleration(k, 0.5),
          "the Dirichlet acceleration");
  const auto second = i.integrate(m, 0.5, 1.0, {}, &first.final);
  check(second.final.values == whole.final.values &&
            second.final.velocities == whole.final.velocities &&
            second.final.accelerations == whole.final.accelerations,
        "Newmark: the continued run equals one long run");
  // No start: nothing is assembled at the continuation time.
  check(second.report.metrics.at("assemblies") ==
            second.report.metrics.at("steps"),
        "one assembly per step, none at t0");
  // Without the accelerations, start() computes them again (to rounding).
  Solution without = first.final;
  without.accelerations.clear();
  const auto restarted = i.integrate(m, 0.5, 1.0, {}, &without);
  check(restarted.report.metrics.at("assemblies") ==
            restarted.report.metrics.at("steps") + 1,
        "start() assembles at t0");
  for (std::size_t k = 0; k < whole.final.values.size(); ++k) {
    near(restarted.final.values[k], whole.final.values[k], 1e-12, "values");
    near(restarted.final.velocities[k], whole.final.velocities[k], 1e-11,
         "velocities");
  }
  // A method that keeps no acceleration returns none, and ignores given
  // ones.
  const auto euler = integrator(std::make_shared<ImplicitEuler>(), 0.1);
  const auto euler_first = euler.integrate(m, 0.0, 0.5);
  check(euler_first.final.accelerations.empty(),
        "implicit Euler returns no accelerations");
  check(euler.integrate(m, 0.5, 1.0, {}, &first.final).final.values ==
            euler.integrate(m, 0.5, 1.0, {}, &without).final.values,
        "implicit Euler ignores accelerations");
}

// The review case: (t - 0.5)^2 u_tt + 0.1 u_t - (u_xx + u_yy) = f with the
// wave's u. M2 is zero at t = 0.5, so Newmark could not start there; a run
// continued at 0.5 takes the carried acceleration and equals one long run.
void newmark_continues_where_m2_vanishes() {
  Problem problem = wave_problem();
  const std::string u_t = "-2*" + p + "*sin(2*t) + 3*" + q + "*cos(3*t)";
  const std::string u_tt = "-4*" + p + "*cos(2*t) - 9*" + q + "*sin(3*t)";
  problem.equation.m2 = e("(t - 0.5)^2");
  problem.equation.f = e("(t - 0.5)^2*(" + u_tt + ") + 0.1*(" + u_t + ")");
  const auto m = meshed(problem);
  const auto i = integrator(std::make_shared<Newmark>(), 1.0 / 16);
  const auto whole = i.integrate(m, 0.0, 1.0, {.snapshot_times = {0.5}});
  const auto first = i.integrate(m, 0.0, 0.5);
  const auto second = i.integrate(m, 0.5, 1.0, {}, &first.final);
  check(second.final.values == whole.final.values &&
            second.final.velocities == whole.final.velocities,
        "continued at t = 0.5 equals one long run");
  const double error = pde2d::test::max_nodal_error(
      *m->mesh(), whole.final.values,
      [](double x, double y) { return wave_exact(x, y, 1.0); });
  std::cout << "newmark with m2 = (t - 0.5)^2, dt = 1/16: error at t = 1 "
            << error << '\n';
  check(error < 0.05, "Newmark follows the solution through m2 = 0");
  // Restarting there needs M2 at t = 0.5, which is zero.
  Solution without = first.final;
  without.accelerations.clear();
  expect_throws<std::runtime_error>(
      [&] { (void)i.integrate(m, 0.5, 1.0, {}, &without); },
      "M2 is zero at t0 = 0.5");
}

void checks_input() {
  const auto m = meshed(constant_heat_problem());
  const auto i = integrator(std::make_shared<ImplicitEuler>(), 0.1);
  expect_throws<std::invalid_argument>(
      [&] { (void)i.integrate(nullptr, 0.0, 1.0); }, "problem is null");
  expect_throws<std::invalid_argument>([&] { (void)i.integrate(m, 1.0, 1.0); },
                                       "greater than t0");
  expect_throws<std::invalid_argument>(
      [&] { (void)i.integrate(m, 0.0, std::nan("")); }, "finite");
  expect_throws<std::invalid_argument>(
      [&] { (void)i.integrate(m, 0.0, 1.0, {.snapshot_times = {1.5}}); },
      "not within [t0, t1]");
  expect_throws<std::invalid_argument>(
      [&] { (void)i.integrate(m, 0.0, 1.0, {.snapshot_times = {0.5, 0.5}}); },
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
  // Accelerations of the wrong size or not finite, for a method that keeps
  // them.
  const auto newmark = integrator(std::make_shared<Newmark>(), 0.1);
  Solution wave_start = newmark.integrate(wave, 0.0, 0.1).final;
  wave_start.accelerations.pop_back();
  expect_throws<std::invalid_argument>(
      [&] { (void)newmark.integrate(wave, 0.1, 0.2, {}, &wave_start); },
      "accelerations for");
  wave_start.accelerations.push_back(0.0);
  wave_start.accelerations[wave->free_nodes()[0]] = std::nan("");
  expect_throws<std::invalid_argument>(
      [&] { (void)newmark.integrate(wave, 0.1, 0.2, {}, &wave_start); },
      "not finite");
  expect_throws<std::invalid_argument>([] { FixedStep({.dt = 0.0}); },
                                       "dt must be finite and positive");
  expect_throws<std::invalid_argument>(
      [] {
        Integrator(nullptr,
                   std::make_shared<FixedStep>(FixedStepOptions{.dt = 1}),
                   std::make_shared<SparseLu>());
      },
      "time method is null");
}

// The members that existed before before_last_step keep their order: a
// positional initialization written then means the same (review finding).
void integration_result_keeps_its_order() {
  Solution final{.time = 1.0, .values = {2.0}};
  std::vector<Solution> snapshots(2);
  std::vector<StepRecord> history(3);
  pde2d::mesh::StageReport report;
  report.stage = "integrator";
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
  const IntegrationResult r{final, snapshots, history, report};
#pragma GCC diagnostic pop
  check(r.final.time == 1.0 && r.final.values == std::vector<double>{2.0} &&
            r.snapshots.size() == 2 && r.history.size() == 3 &&
            r.report.stage == "integrator",
        "final, snapshots, history, report");
  check(!r.before_last_step, "the new member keeps its default");
  // Solution: mesh, time, values, velocities, then accelerations.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
  const Solution s{nullptr, 2.0, {1.0}, {3.0}};
#pragma GCC diagnostic pop
  check(s.time == 2.0 && s.values == std::vector<double>{1.0} &&
            s.velocities == std::vector<double>{3.0} && s.accelerations.empty(),
        "Solution keeps its positional order");
}

} // namespace

int main() {
  return pde2d::test::run({
      {"IntegrationResult and Solution keep their positional order",
       integration_result_keeps_its_order},
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
      {"implicit Euler integrates time order 2 at first order",
       [] {
         time_rate(std::make_shared<ImplicitEuler>(), wave_problem(),
                   wave_exact, 0.85, 1.2);
       }},
      {"implicit midpoint integrates time order 2 at second order",
       [] {
         time_rate(std::make_shared<ImplicitMidpoint>(), wave_problem(),
                   wave_exact, 1.85, 2.2);
       }},
      {"M2 zero at t0: Newmark refuses, midpoint and Euler converge",
       vanishing_mass},
      {"with M2 = 0 the midpoint step is the first-order step",
       midpoint_without_mass},
      {"implicit Euler at time order 0 solves statically at each step",
       order_zero_is_static},
      {"CG and LDLT where the step matrix is symmetric, refused elsewhere",
       symmetric_solvers},
      {"BiCgStab gives the same time integration as SparseLu",
       [] {
         const auto m = meshed(heat_problem());
         const auto lu = integrator(std::make_shared<ImplicitMidpoint>(), 0.1)
                             .integrate(m, 0.0, 1.0);
         const auto it = integrator(std::make_shared<ImplicitMidpoint>(), 0.1,
                                    std::make_shared<BiCgStab>(
                                        BiCgStabOptions{.tolerance = 1e-13}))
                             .integrate(m, 0.0, 1.0);
         for (std::size_t k = 0; k < lu.final.values.size(); ++k)
           near(it.final.values[k], lu.final.values[k], 1e-10, "same values");
         check(it.report.metrics.at("linear_iterations") > 0, "iterations");
       }},
      {"Newmark conserves the energy of an undamped wave",
       [] { energy_is_conserved(std::make_shared<Newmark>()); }},
      {"implicit midpoint conserves the energy of an undamped wave",
       [] { energy_is_conserved(std::make_shared<ImplicitMidpoint>()); }},
      {"the solver is prepared only when the step matrix changes",
       counts_prepares},
      {"methods are refused for time orders they do not support",
       refuses_time_orders},
      {"CG where the step matrix is symmetric by its values only",
       symmetric_now_solvers},
      {"u = sqrt(t) from t0 = 0: Euler and midpoint never assemble at t0",
       starts_without_assembling},
      {"m2 = sin(t) from t0 = 0: Newmark refuses, midpoint runs",
       mass_zero_at_start},
      {"snapshots land exactly on their times", lands_on_snapshots},
      {"a continued run equals one long run", continues_runs},
      {"Newmark continues at a time where M2 vanishes",
       newmark_continues_where_m2_vanishes},
      {"max_steps ends a run early; the state before the last step",
       stops_after_max_steps},
      {"the integrator checks its input", checks_input},
  });
}
