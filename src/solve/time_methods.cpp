// The time methods: ImplicitEuler, ImplicitMidpoint and Newmark, and the
// default TimeMethod::start. Each advance() builds the right-hand side of
// its step from the assembled problem's free parts and the state, solves,
// and updates the state. The Dirichlet lifting is already in
// AssembledProblem::right_hand_side().

#include "pde2d/solve/implicit_euler.hpp"
#include "pde2d/solve/implicit_midpoint.hpp"
#include "pde2d/solve/newmark.hpp"

#include "vectors.hpp"

#include <memory>
#include <utility>

namespace pde2d::solve {

void TimeMethod::start(const fem::AssembledProblem &, const LinearSolver &,
                       TimeState &) const {}

namespace {

StepSolveInfo info(const LinearSolveResult &solved) {
  return {.linear_iterations = solved.iterations, .residual = solved.residual};
}

} // namespace

// --- ImplicitEuler ----------------------------------------------------------

StepMatrixWeights ImplicitEuler::step_matrix_weights(double dt) const {
  return {.mass = 0.0, .damping = 1.0 / dt, .stiffness = 1.0};
}

StepSolveInfo ImplicitEuler::advance(const fem::AssembledProblem &problem,
                                     double dt,
                                     const PreparedLinearSolver &solver,
                                     TimeState &state) const {
  // (B / dt + A) U1 = b(t1) + B U0 / dt
  std::vector<double> rhs = detail::right_hand_side(problem);
  detail::add_product(rhs, 1.0 / dt, problem.damping(), state.values);
  LinearSolveResult solved = solver.solve(rhs);
  state.values = std::move(solved.values);
  return info(solved);
}

// --- ImplicitMidpoint -------------------------------------------------------

StepMatrixWeights ImplicitMidpoint::step_matrix_weights(double dt) const {
  return {.mass = 0.0, .damping = 1.0 / dt, .stiffness = 0.5};
}

StepSolveInfo ImplicitMidpoint::advance(const fem::AssembledProblem &problem,
                                        double dt,
                                        const PreparedLinearSolver &solver,
                                        TimeState &state) const {
  // (B / dt + A / 2) U1 = b(tm) + (B / dt - A / 2) U0, all at tm.
  std::vector<double> rhs = detail::right_hand_side(problem);
  detail::add_product(rhs, 1.0 / dt, problem.damping(), state.values);
  detail::add_product(rhs, -0.5, problem.stiffness(), state.values);
  LinearSolveResult solved = solver.solve(rhs);
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
  // M2(t0) W0 = b(t0) - B(t0) V0 - A(t0) U0
  std::vector<double> rhs = detail::right_hand_side(problem);
  detail::add_product(rhs, -1.0, problem.damping(), state.velocities);
  detail::add_product(rhs, -1.0, problem.stiffness(), state.values);
  const auto prepared = linear_solver.prepare(
      std::make_shared<const fem::CsrMatrix>(problem.mass().free));
  state.accelerations = prepared->solve(rhs).values;
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
