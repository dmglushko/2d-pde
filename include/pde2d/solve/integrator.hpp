#pragma once

#include "pde2d/fem/meshed_problem.hpp"
#include "pde2d/mesh/report.hpp"
#include "pde2d/solve/linear_solver.hpp"
#include "pde2d/solve/solution.hpp"
#include "pde2d/solve/step_controller.hpp"
#include "pde2d/solve/time_method.hpp"

#include <chrono>
#include <cstddef>
#include <memory>
#include <vector>

namespace pde2d::solve {

// What an integrate() call keeps besides the final solution.
struct IntegrationOutput {
  // Keep the record of every step (IntegrationResult::history).
  bool keep_step_history = false;
  // Times at which a Solution is kept; the steps are shortened so that
  // they land exactly on them. Strictly increasing, within [t0, t1]; a
  // time equal to t0 keeps the starting state.
  std::vector<double> snapshot_times{};
  // Keep a Solution after every n-th step of the call (0: never). A step
  // that also lands on a snapshot time gives one snapshot.
  std::size_t snapshot_every = 0;
};

// The record of one step.
struct StepRecord {
  double t = 0.0;  // the time at the end of the step
  double dt = 0.0; // its length
  bool accepted = true;
  std::size_t linear_iterations = 0;
  double residual = 0.0;
  std::chrono::duration<double> time{}; // the time taken
};

struct IntegrationResult {
  // The solution at t1 (with velocities for time order 2).
  Solution final{};
  // In time order; see IntegrationOutput.
  std::vector<Solution> snapshots{};
  // Every step's record, if keep_step_history was set; else empty.
  std::vector<StepRecord> history{};
  // Stage "integrator", the summary of the step records: the time taken
  // and the metrics t0, t1, time_order, steps, rejected_steps, min_dt,
  // max_dt, linear_iterations (the sum), max_linear_iterations,
  // max_residual, assemblies, prepares (how often the linear solver was
  // prepared for a step matrix; not counting start() or the solve at t0
  // for time order 0) and snapshots.
  mesh::StageReport report{};
};

// Integrates a MeshedProblem in time (docs/decisions.md, "Solvers and time
// integration"). The components are configured here; the job is given to
// integrate().
class Integrator {
public:
  // Throws std::invalid_argument for a null component.
  Integrator(std::shared_ptr<const TimeMethod> method,
             std::shared_ptr<const StepController> step_controller,
             std::shared_ptr<const LinearSolver> linear_solver);

  // Integrates from t0 to t1 > t0:
  //
  // 1. assembles the problem at t0 and checks its time order
  //    (AssembledProblem::time_order()) against the method's declared
  //    capabilities (TimeMethodCapabilities::supports); when the assembled
  //    order is lower than the order as written (time_order(meshed)), the
  //    error says that time terms vanished in assembly;
  // 2. takes the starting state, in the free unknowns, from `initial` if
  //    given (the Dirichlet nodes are ignored; for time order 2 it must
  //    have velocities), else from MeshedProblem::initial_values(t0) and,
  //    for time order 2, initial_velocities(t0); calls the method's
  //    start(). Time order 0 has no initial state: the state at t0 is the
  //    solution of A(t0) U = b(t0), and `initial` is only checked;
  // 3. repeats until t1: asks the step controller for dt, shortens the
  //    step to land on the next snapshot time or t1 (a step that ends
  //    within 1e-9 dt of one lands on it), assembles the problem at the
  //    method's evaluation point (passing the previous result, so parts
  //    that do not depend on t are reused), prepares the linear solver
  //    for the step matrix only when it changed (dt changed, or an
  //    operator in it depends on t), and advances the state.
  //
  // Before the linear solver is prepared for a matrix, its requirements
  // (LinearSolver::requirements) are checked against assembly's facts:
  // the step matrix is symmetric when every operator with a non-zero
  // weight is symmetric or zero (fem::Operator::symmetric).
  //
  // Times on a stretch of equal steps are computed as start + k dt (not
  // summed), so a run that lands on a time and one that starts there take
  // the same steps. Continuing a run: integrate(meshed, t1, t2, output,
  // &previous.final).
  //
  // Throws std::invalid_argument for a null problem, times that are not
  // finite or t1 <= t0, bad output options, a time order the method does
  // not support, a linear solver whose requirements a matrix does not
  // meet, and an `initial` on another mesh object, of the wrong size, with
  // values that are not finite or without the velocities time order 2
  // needs. Passes on the exceptions of assembly, of the method's start()
  // and of the linear solver.
  [[nodiscard]] IntegrationResult
  integrate(std::shared_ptr<const fem::MeshedProblem> meshed_problem, double t0,
            double t1, const IntegrationOutput &output = {},
            const Solution *initial = nullptr) const;

private:
  std::shared_ptr<const TimeMethod> method_;
  std::shared_ptr<const StepController> step_controller_;
  std::shared_ptr<const LinearSolver> linear_solver_;
};

} // namespace pde2d::solve
