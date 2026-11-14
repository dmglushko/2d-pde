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
// M2(t0) W0 = b(t0) - B(t0) V0 - A(t0) U0, preparing the linear solver for
// M2, which must therefore not be singular. No options.
class Newmark final : public TimeMethod {
public:
  std::string name() const override { return "newmark"; }
  TimeMethodCapabilities capabilities() const override {
    return {.supports_first_order = false, .supports_second_order = true};
  }
  double evaluation_fraction() const override { return 1.0; }
  StepMatrixWeights step_matrix_weights(double dt) const override;
  void start(const fem::AssembledProblem &problem,
             const LinearSolver &linear_solver,
             TimeState &state) const override;
  StepSolveInfo advance(const fem::AssembledProblem &problem, double dt,
                        const PreparedLinearSolver &solver,
                        TimeState &state) const override;
};

} // namespace pde2d::solve
