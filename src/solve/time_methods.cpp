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
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>

namespace pde2d::solve {

void TimeMethod::start(const fem::AssembledProblem &, const LinearSolver &,
                       TimeState &) const {}

bool TimeMethod::keeps_acceleration(int) const { return false; }

bool TimeMethod::velocity_quotient_is_acceleration(int) const { return false; }

std::optional<ErrorEstimateDeclaration> TimeMethod::error_estimate(int) const {
  return std::nullopt;
}

std::vector<double>
TimeMethod::local_error(const TimeState &, const TimeState &, double,
                        std::span<const PreviousStep>) const {
  throw std::logic_error(name() + " declares no local error estimate");
}

namespace {

// Checks the earlier steps a local error estimate reads: as many as
// declared, each with a value per free unknown and a positive length.
void check_previous(const std::string &method,
                    std::span<const PreviousStep> previous, std::size_t count,
                    std::size_t size) {
  if (previous.size() != count)
    throw std::invalid_argument(
        std::format("{}: the local error estimate reads {} earlier steps, "
                    "given {}",
                    method, count, previous.size()));
  for (const PreviousStep &step : previous)
    if (step.values.size() != size || !(step.dt > 0.0))
      throw std::invalid_argument(std::format(
          "{}: an earlier step has {} values for {} unknowns or a length "
          "that is not positive",
          method, step.values.size(), size));
}

// Entry by entry, Newton's divided difference U[s0, ..., sk] of the values
// at the increasing points s (one vector per point).
std::vector<double> divided_difference(std::vector<std::vector<double>> table,
                                       const std::vector<double> &s) {
  for (std::size_t level = 1; level < s.size(); ++level)
    for (std::size_t j = 0; j + level < s.size(); ++j)
      for (std::size_t i = 0; i < table[j].size(); ++i)
        table[j][i] = (table[j + 1][i] - table[j][i]) / (s[j + level] - s[j]);
  return std::move(table[0]);
}

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

std::optional<ErrorEstimateDeclaration>
ImplicitEuler::error_estimate(int time_order) const {
  // Time order 0 has no time error; time order 2 has u_t in the state,
  // time order 1 needs the previous step's quotient.
  if (time_order == 1)
    return ErrorEstimateDeclaration{.order = 1, .previous_steps = 1};
  if (time_order == 2)
    return ErrorEstimateDeclaration{.order = 1, .previous_steps = 0};
  return std::nullopt;
}

std::vector<double>
ImplicitEuler::local_error(const TimeState &before, const TimeState &after,
                           double dt,
                           std::span<const PreviousStep> previous) const {
  const std::size_t n = before.values.size();
  // No free unknowns (every node a Dirichlet node): nothing to estimate,
  // and at time order 2 no velocities to tell the order by.
  if (n == 0)
    return {};
  std::vector<double> error(n);
  if (!before.velocities.empty()) {
    // Time order 2: dt^2 / 2 u'' with u'' = (V1 - V0) / dt.
    check_previous(name(), previous, 0, n);
    for (std::size_t i = 0; i < n; ++i)
      error[i] = 0.5 * dt * (after.velocities[i] - before.velocities[i]);
    return error;
  }
  // Time order 1. The step's error is dt^2 / 2 u'': Euler takes
  // U1 = U0 + dt u'(t1) = U0 + dt u' + dt^2 u'' + ..., the exact solution
  // through U0 is U0 + dt u' + dt^2 / 2 u'' + ... (u' and u'' at t0). The
  // step solves B d1 + A U1 = b(t1) with d1 = (U1 - U0) / dt, so d1 is the
  // rate at the step's end t1, and the earlier step's quotient
  // d0 = (U0 - U_-1) / dt_-1 is the rate at t0, whatever that step's
  // length: the two are dt apart, so u'' = (d1 - d0) / dt and
  // e = dt (d1 - d0) / 2 (one earlier step).
  check_previous(name(), previous, 1, n);
  const PreviousStep &last = previous[0];
  for (std::size_t i = 0; i < n; ++i) {
    const double d1 = (after.values[i] - before.values[i]) / dt;
    const double d0 = (before.values[i] - last.values[i]) / last.dt;
    error[i] = 0.5 * dt * (d1 - d0);
  }
  return error;
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

std::optional<ErrorEstimateDeclaration>
ImplicitMidpoint::error_estimate(int time_order) const {
  // Time order 1 from the values of the last three steps, time order 2
  // from the velocities, which need one earlier step.
  if (time_order == 1)
    return ErrorEstimateDeclaration{.order = 2, .previous_steps = 2};
  if (time_order == 2)
    return ErrorEstimateDeclaration{.order = 2, .previous_steps = 1};
  return std::nullopt;
}

std::vector<double>
ImplicitMidpoint::local_error(const TimeState &before, const TimeState &after,
                              double dt,
                              std::span<const PreviousStep> previous) const {
  const std::size_t n = before.values.size();
  // No free unknowns (every node a Dirichlet node): nothing to estimate,
  // and at time order 2 no velocities to tell the order by.
  if (n == 0)
    return {};
  if (!before.velocities.empty()) {
    // Time order 2. The displacement update (U1 - U0) / dt = (V0 + V1) / 2
    // is the trapezoidal rule for U' = V, whose error is dt^3 / 12 V'' =
    // dt^3 / 12 u''' (an error of V1 enters U1 only times dt / 2, at
    // O(dt^4)). u''' = V'' is estimated from the endpoint velocities, not
    // from the values, which carry the errors of earlier (possibly much
    // longer) steps: a1 = (V1 - V0) / dt and a0 = (V0 - V_-1) / dt_-1 are
    // u'' at the middles of this step and the one before, (dt + dt_-1) / 2
    // apart, so u''' = 2 (a1 - a0) / (dt + dt_-1) and
    // e = dt^3 (a1 - a0) / (6 (dt + dt_-1)). The velocity V_-1 at the start
    // of the earlier step is not kept; that step was one of this method's
    // (no other method keeps earlier steps at time order 2, so none comes
    // with an anchor), so its own update gives it back:
    // V_-1 = 2 (U0 - U_-1) / dt_-1 - V0.
    check_previous(name(), previous, 1, n);
    const PreviousStep &last = previous[0];
    std::vector<double> error(n);
    for (std::size_t i = 0; i < n; ++i) {
      const double v0 = before.velocities[i];
      const double v_last =
          2.0 * (before.values[i] - last.values[i]) / last.dt - v0;
      const double a1 = (after.velocities[i] - v0) / dt;
      const double a0 = (v0 - v_last) / last.dt;
      error[i] = dt * dt * dt * (a1 - a0) / (6.0 * (dt + last.dt));
    }
    return error;
  }
  // Time order 1: dt^3 / 12 u''' with u''' = 6 U[s0, s1, s2, s3], the
  // points the ends of the two earlier steps and of this one, placed by
  // the steps' lengths.
  check_previous(name(), previous, 2, n);
  const double s1 = previous[0].dt;
  const double s2 = s1 + previous[1].dt;
  const double s3 = s2 + dt;
  std::vector<double> error = divided_difference(
      {previous[0].values, previous[1].values, before.values, after.values},
      {0.0, s1, s2, s3});
  for (double &e : error)
    e *= 0.5 * dt * dt * dt;
  return error;
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

std::optional<ErrorEstimateDeclaration>
Newmark::error_estimate(int time_order) const {
  if (time_order == 2)
    return ErrorEstimateDeclaration{.order = 2, .previous_steps = 0};
  return std::nullopt;
}

std::vector<double>
Newmark::local_error(const TimeState &before, const TimeState &after, double dt,
                     std::span<const PreviousStep> previous) const {
  // Zienkiewicz and Xie (1991): dt^2 (beta - 1/6) (W1 - W0).
  const std::size_t n = before.values.size();
  // No free unknowns (every node a Dirichlet node): nothing to estimate.
  if (n == 0)
    return {};
  check_previous(name(), previous, 0, n);
  std::vector<double> error(n);
  for (std::size_t i = 0; i < n; ++i)
    error[i] = dt * dt * (beta - 1.0 / 6.0) *
               (after.accelerations[i] - before.accelerations[i]);
  return error;
}

} // namespace pde2d::solve
