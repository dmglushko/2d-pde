#pragma once

#include "pde2d/solve/time_method.hpp"

namespace pde2d::solve {

// Implicit (backward) Euler for time order 1, first-order accurate. With
// everything assembled at the end of the step, t1 = t0 + dt:
//
//   B (U1 - U0) / dt + A U1 = b(t1)
//   (B / dt + A) U1 = b(t1) + B U0 / dt
//
// B may be singular (only some unknowns have a time derivative); the step
// matrix must not be. No options.
class ImplicitEuler final : public TimeMethod {
public:
  std::string name() const override { return "implicit_euler"; }
  TimeMethodCapabilities capabilities() const override {
    return {.supports_first_order = true, .supports_second_order = false};
  }
  double evaluation_fraction() const override { return 1.0; }
  StepMatrixWeights step_matrix_weights(double dt) const override;
  StepSolveInfo advance(const fem::AssembledProblem &problem, double dt,
                        const PreparedLinearSolver &solver,
                        TimeState &state) const override;
};

} // namespace pde2d::solve
