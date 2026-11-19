#pragma once

#include "pde2d/solve/time_method.hpp"

namespace pde2d::solve {

// Newmark's method with average acceleration (beta = 1/4, gamma = 1/2) for
// time order 2: second-order accurate, no numerical damping, one linear
// solve per step. The state is U, V = U' and the acceleration W = U''. The
// equation holds at the end of the step, t1 = t0 + dt:
//
//   M2 W1 + B V1 + A U1 = b(t1)
//   U1 = U0 + dt V0 + dt^2 ((1/2 - beta) W0 + beta W1)
//   V1 = V0 + dt ((1 - gamma) W0 + gamma W1)
//
// Solved for U1, the step matrix is M2 / (beta dt^2) + gamma B / (beta dt)
// + A. start() computes W0 from the equation at the start time:
// M2(t0) W0 = b(t0) - B(t0) V0 - A(t0) U0, so it reads the whole problem
// at t0 (StartNeeds::whole_problem), preparing the linear solver for M2,
// which must therefore not be singular. When M2 has only zero entries at
// t0 (Operator::zero_now, m2 = sin(t) from t0 = 0), start() throws
// std::runtime_error saying so before trying; when the solve with M2
// fails, it throws std::runtime_error saying that M2 is singular at t0.
// Both name ImplicitMidpoint and ImplicitEuler, which need no
// acceleration; there is no fallback (docs/decisions.md). It keeps the
// acceleration (keeps_acceleration): the Integrator returns W in its
// Solutions, and a run continued from one with accelerations does not call
// start(), so it continues exactly, also from a time where M2 is singular.
// No options.
class Newmark final : public TimeMethod {
public:
  std::string name() const override { return "newmark"; }
  TimeMethodCapabilities capabilities() const override {
    return {.time_order_2 = true};
  }
  double evaluation_fraction() const override { return 1.0; }
  StepMatrixWeights step_matrix_weights(double dt) const override;
  StartNeeds start_needs(int) const override {
    return StartNeeds::whole_problem;
  }
  // The state carries W, and start() computes only W0.
  bool keeps_acceleration(int) const override { return true; }
  // The local error estimate, order p = 2, from the change of the
  // acceleration over the step (Zienkiewicz and Xie, "A simple error
  // estimator and adaptive time stepping procedure for dynamic analysis",
  // Earthquake Eng. Struct. Dyn. 20 (1991) 871-887):
  // e = dt^2 (beta - 1/6) (W1 - W0) = dt^2 / 12 (W1 - W0), no earlier step.
  // No free unknowns: an empty error.
  std::optional<ErrorEstimateDeclaration>
  error_estimate(int time_order) const override;
  std::vector<double>
  local_error(const TimeState &before, const TimeState &after, double dt,
              std::span<const PreviousStep> previous) const override;
  void start(const fem::AssembledProblem &problem,
             const LinearSolver &linear_solver,
             TimeState &state) const override;
  StepSolveInfo advance(const fem::AssembledProblem &problem, double dt,
                        const PreparedLinearSolver &solver,
                        TimeState &state) const override;
};

} // namespace pde2d::solve
