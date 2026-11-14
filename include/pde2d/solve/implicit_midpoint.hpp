#pragma once

#include "pde2d/solve/time_method.hpp"

namespace pde2d::solve {

// The implicit midpoint method for time order 1, second-order accurate.
// The matrices and the right-hand side are evaluated once, at the middle
// of the step, tm = t0 + dt / 2, and U there is the average of U0 and U1:
//
//   B(tm) (U1 - U0) / dt + A(tm) (U0 + U1) / 2 = b(tm)
//   (B / dt + A / 2) U1 = b(tm) + (B / dt - A / 2) U0
//
// (It is not called Crank-Nicolson, which also names averaging A U and b
// over the two ends.) No options.
class ImplicitMidpoint final : public TimeMethod {
public:
  std::string name() const override { return "implicit_midpoint"; }
  TimeMethodCapabilities capabilities() const override {
    return {.supports_first_order = true, .supports_second_order = false};
  }
  double evaluation_fraction() const override { return 0.5; }
  StepMatrixWeights step_matrix_weights(double dt) const override;
  StepSolveInfo advance(const fem::AssembledProblem &problem, double dt,
                        const PreparedLinearSolver &solver,
                        TimeState &state) const override;
};

} // namespace pde2d::solve
