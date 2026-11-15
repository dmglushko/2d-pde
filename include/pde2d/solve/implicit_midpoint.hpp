#pragma once

#include "pde2d/solve/time_method.hpp"

namespace pde2d::solve {

// The implicit midpoint method for time orders 1 and 2, second-order
// accurate. The matrices and the right-hand side are evaluated once, at
// the middle of the step, tm = t0 + dt / 2, and U there is the average of
// U0 and U1.
//
// Time order 1:
//   B(tm) (U1 - U0) / dt + A(tm) (U0 + U1) / 2 = b(tm)
//   (B / dt + A / 2) U1 = b(tm) + (B / dt - A / 2) U0
//
// Time order 2, stepped as the pair U' = V, M2 V' + B V + A U = b:
//   (U1 - U0) / dt = (V1 + V0) / 2
//   M2 (V1 - V0) / dt + B (V1 + V0) / 2 + A (U1 + U0) / 2 = b(tm)
// The first gives V1 = 2 (U1 - U0) / dt - V0; put into the second:
//   (2 M2 / dt^2 + B / dt + A / 2) U1
//       = b(tm) + M2 (2 U0 / dt^2 + 2 V0 / dt) + B U0 / dt - A U0 / 2
// With M2 = 0 this is the first-order step. No acceleration is needed, so
// M2 may be singular or zero at some times (m2 = sin(t)^2).
//
// Time order 0 is not supported: with B = 0 the step is
// U1 = 2 A^-1 b - U0, which oscillates around the solution.
//
// One set of weights (2 / dt^2, 1 / dt, 1/2) serves both orders. (It is
// not called Crank-Nicolson, which also names averaging A U and b over
// the two ends.) No options.
class ImplicitMidpoint final : public TimeMethod {
public:
  std::string name() const override { return "implicit_midpoint"; }
  TimeMethodCapabilities capabilities() const override {
    return {.time_order_1 = true, .time_order_2 = true};
  }
  double evaluation_fraction() const override { return 0.5; }
  StepMatrixWeights step_matrix_weights(double dt) const override;
  StepSolveInfo advance(const fem::AssembledProblem &problem, double dt,
                        const PreparedLinearSolver &solver,
                        TimeState &state) const override;
};

} // namespace pde2d::solve
