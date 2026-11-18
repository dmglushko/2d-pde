#pragma once

#include "pde2d/fem/assembler.hpp"
#include "pde2d/fem/meshed_problem.hpp"
#include "pde2d/mesh/report.hpp"
#include "pde2d/solve/linear_solver.hpp"
#include "pde2d/solve/solution.hpp"
#include "pde2d/solve/step_controller.hpp"
#include "pde2d/solve/time_method.hpp"

#include <chrono>
#include <cstddef>
#include <memory>
#include <optional>
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
  // Keep the state at the start of the last step, in the free unknowns
  // (IntegrationResult::before_last_step), for example to form a
  // difference quotient of the final state. No boundary data is evaluated
  // for it.
  bool keep_before_last_step = false;
};

// The integration state at one time in the free unknowns
// (MeshedProblem::free_nodes(): entry i is node free_nodes()[i]), without
// the Dirichlet nodes, so that keeping it evaluates no boundary data (the
// rate of u = sqrt(t) is not finite at t = 0).
struct FreeState {
  double time = 0.0;
  std::vector<double> values{};
  // For time order 2, else empty.
  std::vector<double> velocities{};
  // For IntegrationResult::before_last_step: the length of the step taken
  // from this state, as the method used it (StepRecord::dt). A step that
  // lands on a target time within 1e-9 dt of its nominal end keeps the
  // nominal dt, so the end time minus `time` can differ from it by
  // rounding; a difference quotient of the step divides by this. (After
  // the members above, so that positional initializations keep their
  // meaning: new members go at the end.)
  double dt = 0.0;
};

// Where the steps of a run stand: the stretch of equal steps it is on.
// Step k of the stretch ends at time + k dt (computed so, not summed), and
// `steps` of them were taken, so the run stands at time + steps dt. After
// a step that landed on a target (a snapshot time or t1) the stretch
// starts there: time is the landing time and steps 0. Given back to
// integrate() (its `anchor`), a run continues the same stretch, so that
// it takes the step times one run would have taken: integrate() stopped
// by max_steps and continued from its final state with its anchor equals
// one longer run, also when the step controller's answer depends on t.
struct StepAnchor {
  double time = 0.0;
  // The nominal step of the stretch (the step controller's); 0 before any
  // step.
  double dt = 0.0;
  std::size_t steps = 0;
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
  // The solution where the run ended (with velocities for time order 2,
  // and accelerations for a method that keeps them): at t1, or earlier
  // when integrate() stopped after max_steps steps.
  Solution final{};
  // In time order; see IntegrationOutput.
  std::vector<Solution> snapshots{};
  // Every step's record, if keep_step_history was set; else empty.
  std::vector<StepRecord> history{};
  // Stage "integrator", the summary of the step records: the time taken
  // and the metrics t0, t1 (as asked), t_end (where the run ended: t1, or
  // earlier after max_steps), time_order, steps, rejected_steps, min_dt,
  // max_dt, linear_iterations (the sum), max_linear_iterations,
  // max_residual, assemblies, prepares (how often the linear solver was
  // prepared for a step matrix; not counting start(), which also does the
  // solve at t0 for time order 0), assemblies (counting the one at t0 only
  // when the method's start reads it), snapshots, assembly_threads (the
  // assembler's thread count, 0 resolved; see the "assembly" report) and
  // linear_threads (the most threads the linear solver used in a step),
  // max_backward_error (the largest LinearSolveResult::backward_error)
  // and min_pivot_ratio (the smallest measured pivot_ratio, above 0; 0
  // when no solve measured one, as with an iterative solver), both over
  // every solve of the call, start()'s included.
  // Its messages are the linear solver's warnings
  // (LinearSolveResult::messages) of every solve of the call, the steps'
  // and those a method's start() does: each distinct message once, in the
  // order they first appeared (SupernodalCholesky's, when it factored -A,
  // once however many steps it solved; a direct solver's pivot ratio
  // warning once, also when the step matrix is prepared again at every
  // step: its text holds the threshold, not the ratio). The three are
  // read from the log of solves at once, so they agree.
  mesh::StageReport report{};
  // The state at the start of the last step (with velocities for time
  // order 2), if keep_before_last_step was set; else empty. (After the
  // members above, so that positional initializations keep their meaning:
  // new members go at the end.)
  std::optional<FreeState> before_last_step{};
  // Where the steps stand at the end of the run, to continue the same
  // stretch of steps (integrate()'s `anchor`).
  StepAnchor anchor{};
};

// Integrates a MeshedProblem in time (docs/decisions.md, "Solvers and time
// integration"). The components are configured here; the job is given to
// integrate().
class Integrator {
public:
  // The assembler is made here with `assembler_options` (its thread
  // count). Throws std::invalid_argument for a null component and
  // assembler options out of range.
  Integrator(std::shared_ptr<const TimeMethod> method,
             std::shared_ptr<const StepController> step_controller,
             std::shared_ptr<const LinearSolver> linear_solver,
             fem::AssemblerOptions assembler_options = {});

  // Integrates from t0 to t1 > t0:
  //
  // 1. gets the time order from assembly's formula facts, without
  //    assembling or evaluating anything (fem::Assembler::time_order, the
  //    order AssembledProblem::time_order() gives at every t), and checks
  //    it against the method's declared capabilities
  //    (TimeMethodCapabilities::supports); when it is lower than the
  //    order as written (time_order(meshed)), the error says that time
  //    terms vanished in assembly;
  // 2. takes the starting state, in the free unknowns, from `initial` if
  //    given (the Dirichlet nodes are ignored; for time order 2 it must
  //    have velocities; for a method that keeps the acceleration,
  //    TimeMethod::keeps_acceleration, its accelerations when it has
  //    them), else from MeshedProblem::initial_free_state(t0)
  //    (the free nodes only, so no boundary data is evaluated at t0). Time
  //    order 0 has no initial state (`initial` is only checked): the method's
  //    start computes it. Then it assembles the problem at t0 only as far as
  //    the method's start reads it (TimeMethod::start_needs: nothing,
  //    max_time_order 0 or 2) and calls start() with it; for
  //    StartNeeds::nothing, and when the accelerations of a method that
  //    keeps them came from `initial`, it neither assembles at t0 nor
  //    calls start(), so boundary data whose time derivative is not
  //    finite at t0 (u = sqrt(t) from t0 = 0) can be integrated;
  // 3. repeats until t1: asks the step controller for dt, shortens the
  //    step to land on the next snapshot time or t1 (a step that ends
  //    within 1e-9 dt of one lands on it), assembles the problem at the
  //    method's evaluation point (passing the previous result, so parts
  //    that do not depend on t are reused), prepares the linear solver
  //    for the step matrix only when it changed (dt changed, or an
  //    operator in it depends on t), and advances the state.
  //
  // Before the linear solver is prepared for a matrix, its requirements
  // (LinearSolver::requirements) are checked: the step matrix meets a
  // symmetry requirement when it is always symmetric by assembly's facts
  // (every operator with a non-zero weight is always symmetric or always
  // zero: fem::Operator::always_symmetric, always_zero), else when the
  // step matrix about to be prepared is symmetric by its values
  // (fem::is_symmetric).
  //
  // Times on a stretch of equal steps are computed as start + k dt (not
  // summed), so a run that lands on a time and one that starts there take
  // the same steps. A run starts a new stretch at t0, unless `anchor` is
  // given (IntegrationResult::anchor of the run that ended at t0): then
  // its first step continues that stretch when the step controller gives
  // the same dt, so a run continued after max_steps takes the step times
  // of one long run. Continuing a run: integrate(meshed, t1, t2, output,
  // &previous.final); the returned Solutions carry the accelerations of a
  // method that keeps them (Newmark), so such a run continues without its
  // start and equals one long run that lands on t1, bit for bit.
  //
  // `max_steps`, when given, ends the run after that many steps if t1 is
  // not reached first: final.time then says where it ended (the last step
  // was not shortened, so it is not a landing; a run continued from there
  // takes its steps from that time), and snapshot times after it are not
  // taken.
  //
  // Throws std::invalid_argument for a null problem, times that are not
  // finite or t1 <= t0, bad output options, max_steps 0, a time order the
  // method does not support, a linear solver whose requirements a matrix does
  // not meet, and an `initial` on another mesh object, of the wrong size, with
  // values that are not finite, without the velocities time order 2 needs
  // or (for a method that keeps the acceleration) with accelerations of
  // the wrong size or not finite, and an `anchor` that does not stand at
  // t0 (time + steps dt != t0, or a dt that is not finite and positive with
  // steps > 0). Throws std::runtime_error when the method's start() leaves
  // no state at t0 (a method of time order 0 must compute it), or no
  // acceleration for a method that keeps it. Passes on the exceptions of
  // assembly, of the method's start() and of the linear
  // solver.
  [[nodiscard]] IntegrationResult
  integrate(std::shared_ptr<const fem::MeshedProblem> meshed_problem, double t0,
            double t1, const IntegrationOutput &output = {},
            const Solution *initial = nullptr,
            std::optional<std::size_t> max_steps = std::nullopt,
            const StepAnchor *anchor = nullptr) const;

  // The time method, so that a driver can read its declarations.
  [[nodiscard]] const std::shared_ptr<const TimeMethod> &method() const {
    return method_;
  }

  // The same step controller, linear solver and assembler options with
  // another time method (for example the damping steps of the adaptive
  // loop). Throws std::invalid_argument for a null method.
  [[nodiscard]] Integrator
  with_method(std::shared_ptr<const TimeMethod> method) const;

private:
  std::shared_ptr<const TimeMethod> method_;
  std::shared_ptr<const StepController> step_controller_;
  std::shared_ptr<const LinearSolver> linear_solver_;
  fem::Assembler assembler_;
};

} // namespace pde2d::solve
