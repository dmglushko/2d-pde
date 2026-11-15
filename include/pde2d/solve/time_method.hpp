#pragma once

#include "pde2d/fem/assembled_problem.hpp"
#include "pde2d/solve/linear_solver.hpp"

#include <cstddef>
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

  // Called once before the first step, and only when start_needs(time
  // order) is not StartNeeds::nothing, with the problem assembled at the
  // start time as start_needs says and the state at the start (values,
  // and velocities for time order 2; empty for time order 0). A method
  // that needs more state (Newmark's accelerations, the solution at t0
  // for time order 0) computes it here; it may prepare `linear_solver`
  // for its own matrix, after checking the solver's requirements against
  // that matrix's facts. The default does nothing.
  virtual void start(const fem::AssembledProblem &problem,
                     const LinearSolver &linear_solver, TimeState &state) const;

  // Advances `state` by one step of length dt. `problem` is assembled at
  // t + evaluation_fraction() * dt, and `solver` is prepared for the step
  // matrix step_matrix_weights(dt) of that problem.
  virtual StepSolveInfo advance(const fem::AssembledProblem &problem, double dt,
                                const PreparedLinearSolver &solver,
                                TimeState &state) const = 0;
};

} // namespace pde2d::solve
