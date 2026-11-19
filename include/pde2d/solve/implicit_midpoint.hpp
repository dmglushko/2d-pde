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
// The method does not damp. While M2 is zero (at some nodes or for a while),
// V there comes only from V1 = 2 (U1 - U0) / dt - V0, so an error in V0
// stays as an oscillation of sign (+, -, +, ...) and enters U again when M2
// becomes non-zero. Near such times, and on coarse steps, the rate can drop
// below 2; ImplicitEuler damps it but is first order.
//
// Time order 0 is not supported: with B = 0 the step is
// U1 = 2 A^-1 b - U0, which oscillates around the solution.
//
// The start reads nothing at t0 (StartNeeds::nothing): every assembly is
// at a middle of a step, so U_D' and U_D'' are never evaluated at t0
// (u = sqrt(t) from t0 = 0 works).
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
  StartNeeds start_needs(int) const override { return StartNeeds::nothing; }
  // The local error estimate, order p = 2, at time order 2 only. The
  // values are the trapezoidal rule of the velocities, so the error of a
  // step is dt^3 / 12 u''' with u''' = V'' from the endpoint velocities:
  // a1 = (V1 - V0) / dt and a0 = (V0 - V_-1) / dt_-1 are u'' at the
  // middles of this step and the one before, and
  // e = dt^3 (a1 - a0) / (6 (dt + dt_-1)) (one earlier step; V_-1 is the
  // velocity stored with it, PreviousStep::velocities).
  // Time order 1 has no estimate, so an error-controlled step controller
  // refuses the method there: besides dt^3 / 12 u''' the step has the
  // error of averaging the state, h^3 / 8 times the problem's rate times
  // u'', and no estimate from the computed states can see it (with
  // u' = -20 u + 2t + 20t^2 and dt = 0.1 the states are exactly those of
  // u' = 2t, whose steps have no error). An estimate from the step's
  // residual, at the cost of one more solve: issue #44.
  // No free unknowns: an empty error.
  std::optional<ErrorEstimateDeclaration>
  error_estimate(int time_order) const override;
  std::vector<double>
  local_error(const TimeState &before, const TimeState &after, double dt,
              std::span<const PreviousStep> previous) const override;
  StepSolveInfo advance(const fem::AssembledProblem &problem, double dt,
                        const PreparedLinearSolver &solver,
                        TimeState &state) const override;
};

} // namespace pde2d::solve
