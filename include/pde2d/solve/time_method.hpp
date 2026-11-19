#pragma once

#include "pde2d/fem/assembled_problem.hpp"
#include "pde2d/solve/linear_solver.hpp"

#include <cstddef>
#include <optional>
#include <span>
#include <string>
#include <vector>

// The time method interface. The Integrator integrates the assembled system
//
//   M2 U'' + B U' + A U = b
//
// (AssembledProblem: mass(), damping(), stiffness(), right_hand_side(),
// all in the free unknowns, with the Dirichlet lifting already in b). A
// time method declares what it supports and how its step matrix is made;
// the Integrator assembles, prepares the linear solver and calls
// advance() once per step.
namespace pde2d::solve {

// The time orders a method can integrate. The Integrator checks them
// against AssembledProblem::time_order() before the first step. That is
// the order assembly gives, which can be lower than the order as written
// (time_order(MeshedProblem)): a dynamic condition's u_t term vanishes
// where A is zero.
struct TimeMethodCapabilities {
  bool time_order_0 = false; // A U = b at each step time
  bool time_order_1 = false; // B U' + A U = b
  bool time_order_2 = false; // M2 U'' + B U' + A U = b

  // Whether `time_order` (0, 1 or 2) is supported; false for any other.
  bool supports(int time_order) const {
    return (time_order == 0 && time_order_0) ||
           (time_order == 1 && time_order_1) ||
           (time_order == 2 && time_order_2);
  }
};

// What a method's start() reads of the problem at the start time t0
// (docs/decisions.md, "A time method declares what its start reads at
// t0"). The Integrator assembles at t0 only that much, so a problem whose
// time derivatives of the boundary data are not finite at t0 (u = sqrt(t)
// from t0 = 0) can be integrated by a method that does not read them:
// - nothing: start() is not called and nothing is assembled at t0;
// - static_part: the problem assembled at t0 with max_time_order 0 (A and
//   the load, with the lifting by U_D only);
// - whole_problem: the problem assembled at t0 with max_time_order 2.
enum class StartNeeds { nothing, static_part, whole_problem };

// The matrix of one step's linear system, as weights of the three
// operators' free parts: mass * M2 + damping * B + stiffness * A. It
// depends only on dt, so the Integrator can tell when it changes.
struct StepMatrixWeights {
  double mass = 0.0;
  double damping = 0.0;
  double stiffness = 0.0;
};

// The state of a time integration, in the free unknowns
// (MeshedProblem::free_nodes()).
struct TimeState {
  std::vector<double> values{};        // U
  std::vector<double> velocities{};    // U', for time order 2, else empty
  std::vector<double> accelerations{}; // U'', if the method keeps it
};

// What one step reports about its linear solve.
struct StepSolveInfo {
  std::size_t linear_iterations = 0;
  double residual = 0.0;
  // The threads the linear solver used (LinearSolveResult::threads).
  std::size_t linear_threads = 1;
};

// What a method declares about its estimate of a step's local error
// (TimeMethod::error_estimate), for an error-controlling StepController.
struct ErrorEstimateDeclaration {
  // The method's order p: its local error behaves like dt^(p + 1).
  int order = 1;
  // How many accepted steps before the current one the estimate reads
  // (TimeMethod::local_error's `previous`); a step with fewer has no
  // estimate.
  std::size_t previous_steps = 0;
};

// An accepted step before the current one: the values at its start, in
// the free unknowns, its length, and the velocities at its start (time
// order 2; empty when the run has no velocities), so that no estimate
// infers them from which method took the step. (The velocities come
// last, so that positional initializations keep their meaning.)
struct PreviousStep {
  std::vector<double> values{};
  double dt = 0.0;
  std::vector<double> velocities{};
};

class TimeMethod {
public:
  virtual ~TimeMethod() = default;
  virtual std::string name() const = 0;
  virtual TimeMethodCapabilities capabilities() const = 0;

  // Where in a step from t to t + dt the problem is assembled, as a
  // fraction in [0, 1]: the step uses the AssembledProblem at
  // t + fraction * dt (1 for the end of the step, 1/2 for its middle).
  virtual double evaluation_fraction() const = 0;
  // The weights of the step matrix for a step of length dt.
  virtual StepMatrixWeights step_matrix_weights(double dt) const = 0;

  // What start() reads at t0 for a problem of this time order (one the
  // method supports; see StartNeeds).
  virtual StartNeeds start_needs(int time_order) const = 0;

  // Whether, at this time order (one the method supports), the method
  // keeps the acceleration U'' in its state (TimeState::accelerations) and
  // its start() computes nothing else. The Integrator then returns the
  // accelerations in its Solutions (Solution::accelerations), and a run
  // given an initial Solution with accelerations takes them instead of
  // calling start() (and assembles nothing at t0 for it), so a continued
  // run equals one long run and does not need M2 to be invertible at the
  // time it continues from. The default is false: no accelerations are
  // returned, and start() runs as start_needs says.
  virtual bool keeps_acceleration(int time_order) const;

  // Whether, at this time order (one the method supports), the quotient
  // (V1 - V0) / dt of a step's velocities, dt the step's length as
  // advance() received it, satisfies the equation M2 U'' + B U' + A U = b
  // at the end of the step, with the problem assembled there: then it is
  // the acceleration a method that keeps one (keeps_acceleration) can
  // continue from without its start. Only time order 2 has velocities.
  // ImplicitEuler: true at time order 2 (its step is that equation with
  // U'' = (V1 - V0) / dt). The midpoint method's quotient is the
  // acceleration at the middle of the step, Newmark's the average of the
  // two ends, so theirs is false (Newmark keeps its own). The adaptive
  // loop reads it after damping steps. The default is false.
  virtual bool velocity_quotient_is_acceleration(int time_order) const;

  // Called once before the first step, and only when start_needs(time
  // order) is not StartNeeds::nothing (and not when keeps_acceleration
  // holds and the initial Solution has accelerations), with the problem
  // assembled at the start time as start_needs says and the state at the
  // start (values, and velocities for time order 2; empty for time order
  // 0). A method
  // that needs more state (Newmark's accelerations, the solution at t0
  // for time order 0) computes it here; it may prepare `linear_solver`
  // for its own matrix, after checking the solver's requirements against
  // that matrix's facts. The default does nothing.
  virtual void start(const fem::AssembledProblem &problem,
                     const LinearSolver &linear_solver, TimeState &state) const;

  // Whether, at this time order (one the method supports), the method
  // estimates the local error of a step's values from states it already
  // has, with no extra solve (docs/decisions.md, "Adaptive time steps:
  // each method estimates its own error"): its order and how many earlier
  // steps the estimate reads; none when it does not. An error-controlling
  // StepController refuses a method that declares none. The default is
  // none.
  virtual std::optional<ErrorEstimateDeclaration>
  error_estimate(int time_order) const;

  // The estimate of the local error of the step from `before` to `after`
  // (advance() with length dt), one entry per free unknown, for a time
  // order at which error_estimate() declares one. `previous` holds the
  // accepted steps before it, oldest first, exactly as many as declared;
  // the last one ends at `before`. Its sign does not matter (the norm
  // takes absolute values). The default throws std::logic_error.
  virtual std::vector<double>
  local_error(const TimeState &before, const TimeState &after, double dt,
              std::span<const PreviousStep> previous) const;

  // Advances `state` by one step of length dt. `problem` is assembled at
  // t + evaluation_fraction() * dt, and `solver` is prepared for the step
  // matrix step_matrix_weights(dt) of that problem.
  virtual StepSolveInfo advance(const fem::AssembledProblem &problem, double dt,
                                const PreparedLinearSolver &solver,
                                TimeState &state) const = 0;
};

} // namespace pde2d::solve
