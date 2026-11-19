#pragma once

#include "pde2d/solve/time_method.hpp"

namespace pde2d::solve {

// Implicit (backward) Euler for time orders 0, 1 and 2, first-order
// accurate. Everything is assembled at the end of the step, t1 = t0 + dt.
//
// Time order 1:
//   B (U1 - U0) / dt + A U1 = b(t1)
//   (B / dt + A) U1 = b(t1) + B U0 / dt
// B may be singular (only some unknowns have a time derivative); the step
// matrix must not be.
//
// Time order 2, stepped as the pair U' = V, M2 V' + B V + A U = b:
//   (U1 - U0) / dt = V1,   M2 (V1 - V0) / dt + B V1 + A U1 = b(t1)
// Putting V1 = (U1 - U0) / dt into the second equation leaves one system
// of the usual size:
//   (M2 / dt^2 + B / dt + A) U1 = b(t1) + M2 (U0 / dt^2 + V0 / dt)
//                                 + B U0 / dt
// and then V1 = (U1 - U0) / dt. No acceleration is needed, so M2 may be
// singular or zero at some times (m2 = sin(t)^2).
//
// Time order 0: the same formula with M2 = B = 0, A U1 = b(t1), a static
// solve at each step time.
//
// One set of weights (1 / dt^2, 1 / dt, 1) serves every order: a zero
// operator adds nothing. No options.
//
// The start reads nothing at t0 for time orders 1 and 2: the steps are
// assembled at their end times, so U_D' and U_D'' are never evaluated at
// t0 (u = sqrt(t) from t0 = 0 works). For time order 0 there is no
// initial state: start() solves A(t0) U0 = b(t0) from the static part
// assembled at t0 (StartNeeds::static_part), for a snapshot at t0.
class ImplicitEuler final : public TimeMethod {
public:
  std::string name() const override { return "implicit_euler"; }
  TimeMethodCapabilities capabilities() const override {
    return {.time_order_0 = true, .time_order_1 = true, .time_order_2 = true};
  }
  double evaluation_fraction() const override { return 1.0; }
  StepMatrixWeights step_matrix_weights(double dt) const override;
  StartNeeds start_needs(int time_order) const override {
    return time_order == 0 ? StartNeeds::static_part : StartNeeds::nothing;
  }
  // At time order 2 the step is M2 (V1 - V0) / dt + B V1 + A U1 = b(t1),
  // so (V1 - V0) / dt is the acceleration at the step's end.
  bool velocity_quotient_is_acceleration(int time_order) const override {
    return time_order == 2;
  }
  // The local error estimate, order p = 1, at time orders 1 and 2 (none at
  // time order 0, which has no time error): the error of a step is
  // dt^2 / 2 u'', with u'' from the change of u_t over the step
  // (Hairer, Norsett and Wanner, Solving ODEs I, II.4; Gresho and Sani,
  // Incompressible Flow and the FEM, 3.16.4). Time order 2:
  // e = dt / 2 (V1 - V0), no earlier step. Time order 1: u_t is not in the
  // state, so the quotients over this step and the previous one are used,
  // d1 = (U1 - U0) / dt, d0 = (U0 - U_-1) / dt_-1: Euler's quotients are
  // the rates at the steps' ends, dt apart whatever dt_-1 is, so
  // u'' = (d1 - d0) / dt and e = dt (d1 - d0) / 2 (one earlier step). No
  // free unknowns: an empty error.
  std::optional<ErrorEstimateDeclaration>
  error_estimate(int time_order) const override;
  std::vector<double>
  local_error(const TimeState &before, const TimeState &after, double dt,
              std::span<const PreviousStep> previous) const override;
  // Called only for time order 0: the state at t0 is the static solution.
  void start(const fem::AssembledProblem &problem,
             const LinearSolver &linear_solver,
             TimeState &state) const override;
  StepSolveInfo advance(const fem::AssembledProblem &problem, double dt,
                        const PreparedLinearSolver &solver,
                        TimeState &state) const override;
};

} // namespace pde2d::solve
