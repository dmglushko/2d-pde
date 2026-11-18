// The time methods: ImplicitEuler, ImplicitMidpoint and Newmark, and the
// default TimeMethod::start. Each advance() builds the right-hand side of
// its step from the assembled problem's free parts and the state, solves,
// and updates the state. The Dirichlet lifting is already in
// AssembledProblem::right_hand_side().

#include "pde2d/solve/implicit_euler.hpp"
#include "pde2d/solve/implicit_midpoint.hpp"
#include "pde2d/solve/newmark.hpp"

#include "requirements.hpp"
#include "vectors.hpp"

#include <algorithm>
#include <format>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace pde2d::solve {

void TimeMethod::start(const fem::AssembledProblem &, const LinearSolver &,
                       TimeState &) const {}

bool TimeMethod::keeps_acceleration(int) const { return false; }

namespace {

StepSolveInfo info(const LinearSolveResult &solved) {
  return {.linear_iterations = solved.iterations,
          .residual = solved.residual,
          .linear_threads = solved.threads};
}

} // namespace

// --- ImplicitEuler ----------------------------------------------------------

StepMatrixWeights ImplicitEuler::step_matrix_weights(double dt) const {
  // M2 / dt^2 + B / dt + A; a zero operator adds nothing, so these are the
  // weights of every time order.
  return {.mass = 1.0 / (dt * dt), .damping = 1.0 / dt, .stiffness = 1.0};
}

void ImplicitEuler::start(const fem::AssembledProblem &problem,
                          const LinearSolver &linear_solver,
                          TimeState &state) const {
  // Only called for time order 0 (start_needs), with the static part at
  // t0: there is no initial state, the state at t0 is the solution of
  // A(t0) U0 = b(t0). This solve is not counted in the integrator's
  // `prepares`.
  const fem::Operator &stiffness = problem.stiffness();
  detail::check_requirements(
      linear_solver, stiffness.always_symmetric || stiffness.symmetric_now,
      name(), "A (the stiffness)");
  state.values =
      linear_solver
          .prepare(std::make_shared<const fem::CsrMatrix>(stiffness.free))
          ->solve(problem.right_hand_side())
          .values;
}

StepSolveInfo ImplicitEuler::advance(const fem::AssembledProblem &problem,
                                     double dt,
                                     const PreparedLinearSolver &solver,
                                     TimeState &state) const {
  // (M2 / dt^2 + B / dt + A) U1 = b(t1) + M2 (U0 / dt^2 + V0 / dt)
  //                               + B U0 / dt
  // Time order 1 has no M2 (and no V), time order 0 neither M2 nor B.
  const bool second_order = !problem.mass().always_zero;
  std::vector<double> rhs = detail::right_hand_side(problem);
  if (second_order) {
    std::vector<double> mass_part(state.values.size());
    for (std::size_t i = 0; i < mass_part.size(); ++i)
      mass_part[i] = state.values[i] / (dt * dt) + state.velocities[i] / dt;
    detail::add_product(rhs, 1.0, problem.mass(), mass_part);
  }
  detail::add_product(rhs, 1.0 / dt, problem.damping(), state.values);
  LinearSolveResult solved = solver.solve(rhs);
  if (second_order) {
    // V1 = (U1 - U0) / dt.
    for (std::size_t i = 0; i < state.velocities.size(); ++i)
      state.velocities[i] = (solved.values[i] - state.values[i]) / dt;
  }
  state.values = std::move(solved.values);
  return info(solved);
}

// --- ImplicitMidpoint -------------------------------------------------------

StepMatrixWeights ImplicitMidpoint::step_matrix_weights(double dt) const {
  // 2 M2 / dt^2 + B / dt + A / 2, for both time orders.
  return {.mass = 2.0 / (dt * dt), .damping = 1.0 / dt, .stiffness = 0.5};
}

StepSolveInfo ImplicitMidpoint::advance(const fem::AssembledProblem &problem,
                                        double dt,
                                        const PreparedLinearSolver &solver,
                                        TimeState &state) const {
  // (2 M2 / dt^2 + B / dt + A / 2) U1
  //     = b(tm) + M2 (2 U0 / dt^2 + 2 V0 / dt) + (B / dt - A / 2) U0,
  // all at tm. Time order 1 has no M2 (and no V).
  const bool second_order = !problem.mass().always_zero;
  std::vector<double> rhs = detail::right_hand_side(problem);
  if (second_order) {
    std::vector<double> mass_part(state.values.size());
    for (std::size_t i = 0; i < mass_part.size(); ++i)
      mass_part[i] =
          2.0 * state.values[i] / (dt * dt) + 2.0 * state.velocities[i] / dt;
    detail::add_product(rhs, 1.0, problem.mass(), mass_part);
  }
  detail::add_product(rhs, 1.0 / dt, problem.damping(), state.values);
  detail::add_product(rhs, -0.5, problem.stiffness(), state.values);
  LinearSolveResult solved = solver.solve(rhs);
  if (second_order) {
    // (U1 - U0) / dt = (V1 + V0) / 2, so V1 = 2 (U1 - U0) / dt - V0.
    for (std::size_t i = 0; i < state.velocities.size(); ++i)
      state.velocities[i] =
          2.0 * (solved.values[i] - state.values[i]) / dt - state.velocities[i];
  }
  state.values = std::move(solved.values);
  return info(solved);
}

// --- Newmark ----------------------------------------------------------------

namespace {
// Average acceleration.
constexpr double beta = 0.25;
constexpr double gamma = 0.5;
} // namespace

StepMatrixWeights Newmark::step_matrix_weights(double dt) const {
  return {.mass = 1.0 / (beta * dt * dt),
          .damping = gamma / (beta * dt),
          .stiffness = 1.0};
}

void Newmark::start(const fem::AssembledProblem &problem,
                    const LinearSolver &linear_solver, TimeState &state) const {
  // No fallback to another method (docs/decisions.md): the error says why
  // and which methods need no acceleration.
  const auto cannot_start = [&](const std::string &cause) {
    return std::runtime_error(
        std::format("newmark: the starting acceleration cannot be computed, "
                    "since {}; ImplicitMidpoint (implicit_midpoint) and "
                    "ImplicitEuler (implicit_euler) need no acceleration",
                    cause));
  };
  const fem::Operator &mass = problem.mass();
  // First the fact from the numbers: M2 with only zero entries at t0
  // (m2 = sin(t) from t0 = 0) cannot be solved with, whatever the solver.
  // A problem without free unknowns has nothing to solve for.
  if (mass.zero_now && mass.free.rows() > 0)
    throw cannot_start(std::format("M2 is zero at t0 = {} (m2 vanishes there)",
                                   problem.time()));
  detail::check_requirements(
      linear_solver, mass.always_symmetric || mass.symmetric_now, name(), "M2");
  // M2(t0) W0 = b(t0) - B(t0) V0 - A(t0) U0
  std::vector<double> rhs = detail::right_hand_side(problem);
  detail::add_product(rhs, -1.0, problem.damping(), state.velocities);
  detail::add_product(rhs, -1.0, problem.stiffness(), state.values);
  try {
    const auto prepared = linear_solver.prepare(
        std::make_shared<const fem::CsrMatrix>(mass.free));
    state.accelerations = prepared->solve(rhs).values;
  } catch (const std::runtime_error &error) {
    throw cannot_start(std::format(
        "M2 is singular at t0 = {} (solving with it failed: {}), or the "
        "linear solver cannot handle it",
        problem.time(), error.what()));
  }
}

StepSolveInfo Newmark::advance(const fem::AssembledProblem &problem, double dt,
                               const PreparedLinearSolver &solver,
                               TimeState &state) const {
  const std::vector<double> &u0 = state.values;
  const std::vector<double> &v0 = state.velocities;
  const std::vector<double> &w0 = state.accelerations;
  const std::size_t n = u0.size();

  // Solving the update formulas for W1 and V1 in terms of U1:
  //   W1 = (U1 - U0) / (beta dt^2) - V0 / (beta dt) - (1/(2 beta) - 1) W0
  //   V1 = V0 + dt ((1 - gamma) W0 + gamma W1)
  // and putting them into the equation at t1 gives the step matrix and
  //   rhs = b + M2 (U0 / (beta dt^2) + V0 / (beta dt) + (1/(2 beta) - 1) W0)
  //           + B (gamma U0 / (beta dt) + (gamma / beta - 1) V0
  //                + dt (gamma / (2 beta) - 1) W0).
  std::vector<double> mass_part(n), damping_part(n);
  for (std::size_t i = 0; i < n; ++i) {
    mass_part[i] = u0[i] / (beta * dt * dt) + v0[i] / (beta * dt) +
                   (1.0 / (2.0 * beta) - 1.0) * w0[i];
    damping_part[i] = gamma * u0[i] / (beta * dt) +
                      (gamma / beta - 1.0) * v0[i] +
                      dt * (gamma / (2.0 * beta) - 1.0) * w0[i];
  }
  std::vector<double> rhs = detail::right_hand_side(problem);
  detail::add_product(rhs, 1.0, problem.mass(), mass_part);
  detail::add_product(rhs, 1.0, problem.damping(), damping_part);
  LinearSolveResult solved = solver.solve(rhs);

  const std::vector<double> &u1 = solved.values;
  std::vector<double> w1(n), v1(n);
  for (std::size_t i = 0; i < n; ++i) {
    w1[i] = (u1[i] - u0[i]) / (beta * dt * dt) - v0[i] / (beta * dt) -
            (1.0 / (2.0 * beta) - 1.0) * w0[i];
    v1[i] = v0[i] + dt * ((1.0 - gamma) * w0[i] + gamma * w1[i]);
  }
  const StepSolveInfo result = info(solved);
  state.values = std::move(solved.values);
  state.velocities = std::move(v1);
  state.accelerations = std::move(w1);
  return result;
}

} // namespace pde2d::solve
