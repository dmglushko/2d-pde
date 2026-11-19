#pragma once

#include "pde2d/adapt/adaptation.hpp"
#include "pde2d/adapt/adaptive_static_solver.hpp"
#include "pde2d/adapt/error_estimator.hpp"
#include "pde2d/fem/meshed_problem.hpp"
#include "pde2d/mesh/report.hpp"
#include "pde2d/solve/integrator.hpp"
#include "pde2d/solve/solution.hpp"

#include <cstddef>
#include <memory>
#include <optional>
#include <string_view>
#include <vector>

namespace pde2d::adapt {

// When a time-dependent run adapts: after `interval` of time or after
// `steps` steps since the last adaptation (or t0), whichever comes first.
// Both optional, at least one set (like mesh::LengthBound).
struct AdaptationSchedule {
  std::optional<double> interval{};   // finite and positive
  std::optional<std::size_t> steps{}; // at least 1
};

// The target of an adaptive time-dependent run.
struct AdaptiveTimeTarget {
  // When to adapt; required.
  AdaptationSchedule every{};
  // At an adaptation time, adapt only when the estimated error is above
  // this (absolute, relative to the estimate at the first adaptation
  // time, or both: the smaller applies). Not set: adapt every time.
  ErrorTolerance tolerance{};
  // Never integrate on a mesh with more nodes than this (at least 1). The
  // mesh only grows (there is no coarsening yet): a refinement above it is
  // discarded and adapting stops; the run continues to t1 on the last
  // mesh.
  std::optional<std::size_t> max_nodes{};
  // At most this many refinements (adaptation times that changed the mesh;
  // at least 1). After the last one adapting stops: no more estimates, and
  // the run continues to t1 on that mesh as one piece. Counted in
  // refinements, not adaptation times: the schedule already fixes how
  // often the run checks, while each refinement builds a new
  // MeshedProblem and leaves a transfer error in the solution.
  std::optional<std::size_t> max_refinements{};
};

// What happened at one adaptation time.
enum class AdaptationOutcome {
  refined,         // a new mesh; the state was transferred to it
  tolerance_met,   // the estimate was at most the tolerance: no adaptation
  nothing_changed, // the adaptations returned no mesh (nothing marked)
  node_budget,     // the new mesh had more nodes than max_nodes: discarded,
                   // and the run adapts no more
};

// "refined", "tolerance_met", "nothing_changed", "node_budget".
std::string_view to_string(AdaptationOutcome outcome);

// One adaptation time: the estimate of the solution there and what the
// adaptations did with it.
struct AdaptationPoint {
  double time = 0.0;
  // Steps of the run from t0 up to this time.
  std::size_t steps = 0;
  // The estimate the tolerance is tested with (the first adaptation that
  // declares estimates_error); nothing when no adaptation estimates.
  std::optional<ErrorEstimate> estimate{};
  AdaptationOutcome outcome = AdaptationOutcome::nothing_changed;
  // In order: each adaptation's reports (none when the tolerance was
  // met), then for a new mesh "meshed_problem" (the problem the driver
  // built on it) and "solution_transfer".
  std::vector<mesh::StageReport> reports{};
};

// One piece of the run: an integration on one mesh, from t_start to
// t_end, followed by the adaptation at t_end (none after the last piece,
// which ends at t1).
struct IntegrationPiece {
  // The problem integrated, on its mesh (problem->mesh()); pieces after
  // an adaptation that changed nothing share their problem.
  std::shared_ptr<const fem::MeshedProblem> problem{};
  double t_start = 0.0;
  double t_end = 0.0;
  // The integrator's report of this piece ("integrator").
  mesh::StageReport report{};
  std::optional<AdaptationPoint> adaptation{};
  // Whether this piece is the damping steps after a transfer
  // (AdaptiveIntegratorOptions), integrated by the damping method. It ends
  // after those steps, and the next piece continues on the same problem
  // with the integrator's method up to the same adaptation time; it has
  // an adaptation only when it reached that time itself. (After the
  // members above, so that positional initializations keep their
  // meaning: new members go at the end.)
  bool damping = false;
};

struct AdaptiveIntegrationResult {
  // The solution at t1 on the last mesh (with velocities for time order
  // 2, and accelerations when the last piece's state has them).
  solve::Solution final{};
  // At the asked snapshot times, in time order, each on the mesh of its
  // piece.
  std::vector<solve::Solution> snapshots{};
  // Every step's record, of all pieces in order, if keep_step_history was
  // set; else empty.
  std::vector<solve::StepRecord> history{};
  // Every piece, in time order; the meshes used are their problems'.
  std::vector<IntegrationPiece> pieces{};
  // "adaptive_integrator": the time taken; the metrics t0, t1, pieces,
  // adaptation_times, refinements, steps, damping_steps (those of the
  // damping method, included in steps), nodes and triangles (of the
  // last mesh), max_nodes_used (the most nodes any piece integrated on,
  // the last mesh's since meshes only grow) and tolerance (resolved, when
  // set); a message when the node budget or max_refinements stopped
  // adapting, or when the caller stopped the run.
  mesh::StageReport report{};
  // Whether IntegrationOutput::on_step stopped the run before t1: `final`
  // is then where it stood, the last piece has no adaptation, and the
  // snapshots are those up to there. (After the members above, so that
  // positional initializations keep their meaning: new members go at the
  // end.)
  bool stopped = false;
};

// The settings of the adaptive loop in time that are the same for every
// job (docs/decisions.md, "Where a setting goes").
struct AdaptiveIntegratorOptions {
  // After each transfer to a new mesh, take this many steps with
  // damping_method before the integrator's method continues (Rannacher
  // smoothing, docs/decisions.md "Damping steps after a transfer"): a
  // method that does not damp (the midpoint method, Newmark) keeps the
  // interpolation error a transfer leaves in the finest modes, implicit
  // Euler damps it within a few steps. 0: off. The steps are the
  // integrator's (its step controller, linear solver and assembler
  // options) with the damping method, and they count as steps of the
  // schedule.
  std::size_t damping_steps = 0;
  // Required when damping_steps > 0; typically solve::ImplicitEuler. It
  // must support the problem's time order (checked by integrate() from
  // its capabilities before the first step), and when the integrator's
  // method keeps the acceleration (Newmark) it must hand one over: keep
  // its own (keeps_acceleration) or declare that its velocity quotient is
  // the acceleration at the end of a step
  // (velocity_quotient_is_acceleration: implicit Euler), else integrate()
  // throws before the first step. Never used for anything else: no method
  // is switched to on a failure.
  std::shared_ptr<const solve::TimeMethod> damping_method{};
};

// The adaptive loop for time-dependent problems: a driver above the
// Integrator (docs/decisions.md, "An adaptive loop is a driver above the
// solvers").
//
//   piece k: integrate from t_k until the next adaptation time
//            (Integrator::integrate continued from the last state)
//         -> estimate there -> above the tolerance? -> adapt once
//         -> a new MeshedProblem from the same Problem on the new mesh,
//            the state transferred to it (transfer_solution) -> piece k+1
//
// The adaptation times follow the schedule: the piece ends at t_k +
// interval (the integrator lands on it; a time within 1e-9 interval of t1
// is t1, and of a snapshot time that time, so a snapshot that coincides
// with an adaptation time adds no step) or after `steps` steps
// (Integrator::integrate's max_steps), whichever comes first, and the run
// ends at t1, where it does not adapt.
// Each adaptation time refines at most once, without integrating the
// piece again: the mesh follows the solution one adaptation time behind,
// and adapting more often follows it more closely.
//
// The estimate at an adaptation time is that of the solution there with
// its time derivatives from the last step, at every node: time order 1
// (or 0) u_t = (u(t) - u(t - dt)) / dt (at the Dirichlet nodes from the
// boundary data's values); time order 2 u_t = the integrator's
// velocities, u_tt = (u_t(t) - u_t(t - dt)) / dt at the free nodes and
// the boundary data's acceleration at t at the Dirichlet nodes (its rate
// at t - dt may not exist: u = sqrt(t) at t0 = 0). dt is the length the
// method used for the step (solve::FreeState::dt), which differs from the
// difference of the times by rounding when the step landed on a time.
// The state before the last step (IntegrationOutput::keep_before_last_step:
// the free unknowns, so no boundary data is evaluated for it) is kept only
// for a piece that can end in an adaptation; the boundary data above is
// evaluated only at an adaptation time, so a run that never adapts
// evaluates nothing a plain Integrator run would not. The cycle's estimate
// is that of the first adaptation that declares estimates_error, which is
// then given it. When the estimate is at most the tolerance nothing adapts;
// otherwise every adaptation adapts in order. A new mesh with more nodes
// than max_nodes is discarded and the run adapts no more (the mesh only
// grows); after max_refinements refinements it adapts no more either; the
// rest is one piece to t1. So a run always reaches t1 unless something
// throws.
//
// The state passed on is the previous piece's final solution with its
// step anchor (solve::StepAnchor), on the same problem when nothing
// changed (so the pieces continue the run exactly: the step times are
// those of one integrate() call that lands on the interval's adaptation
// times and the snapshot times, the times of the "every n steps" rule
// included without landing there; with a first-order method bit for bit;
// Newmark continues from its carried acceleration, so bit for bit too),
// else transferred to the new mesh, where a new stretch of steps starts
// (values, velocities for time order 2 and accelerations when the state
// has them; Dirichlet nodes from the new problem).
//
// Damping steps (AdaptiveIntegratorOptions): after each transfer the next
// damping_steps steps are a piece of their own with the damping method
// (IntegrationPiece::damping), then the integrator's method continues to
// the same adaptation time (the schedule counts the damping steps too;
// remaining damping steps carry over an adaptation time that changed
// nothing, and a new transfer starts them again). Where only the method
// changes, the integrator's method continues the damping steps' stretch
// of steps (solve::StepAnchor), so the step times are those of one
// uninterrupted run with the same step controller. For time order 2 the
// state after damping steps carries an acceleration, so a method that
// keeps one continues from it without its start, as the damping method
// declares: its own when it keeps one (Newmark as damping method), else,
// when it declares velocity_quotient_is_acceleration (implicit Euler),
// the quotient of its last step, u_tt = (u_t(t) - u_t(t - dt)) / dt at the
// free nodes (it satisfies M2 u_tt + B u_t + A u = b at t, the equation
// Newmark's start would solve, without needing M2 to be invertible) and
// the boundary data's acceleration at the Dirichlet nodes; a snapshot at
// that time carries it too. A damping method that gives neither, ahead of
// a method that keeps the acceleration, is refused before the first step
// (the midpoint method's quotient is the acceleration at the middle of the
// step).
//
// A transfer leaves the old mesh's interpolation error in the new mesh's
// fine modes. Implicit Euler damps them within a step; the midpoint method
// and Newmark (beta = 1/4) do not, so with them each refinement's error
// stays in the solution unless damping steps are asked for
// (tests/adapt/adaptive_integrator.cpp, a moving bump: implicit Euler
// adaptive 2518 nodes, energy error 0.088 against uniform 2927 nodes,
// 0.25; the midpoint method adaptive 3699 nodes, 0.51, against uniform
// 0.27; adapting every 4 steps of 1/128, the midpoint method gives 3532
// nodes, 0.325, and with 2 damping steps of implicit Euler after each
// transfer 2514 nodes, 0.088, against uniform 2927 nodes, 0.26). For a
// smooth wave damping steps cost accuracy (each adds implicit Euler's
// first-order time error), so they are an option, off by default.
//
// Output: snapshot_times as for the Integrator (the pieces land on them),
// keep_step_history concatenates the pieces' records; snapshot_every is
// not supported (it counts the steps of one integrate() call).
//
// At most one adaptation may change the mesh (declared by changes_mesh):
// all act on the same solved state. Refinement edges as for the
// AdaptiveStaticSolver: the loop never reorders triangles.
class AdaptiveIntegrator {
public:
  // Throws std::invalid_argument without adaptations, for a null one,
  // when more than one declares changes_mesh, and for damping steps
  // without a damping method.
  AdaptiveIntegrator(solve::Integrator integrator,
                     std::vector<std::shared_ptr<const Adaptation>> adaptations,
                     AdaptiveIntegratorOptions options = {});

  // Integrates `problem` (its mesh is the starting mesh) from t0 to t1,
  // adapting as the target says; `initial` as for Integrator::integrate
  // (on the problem's mesh). Throws std::invalid_argument for a null
  // problem, times that are not finite or t1 <= t0, a schedule with
  // nothing set or a value that is not finite and positive (steps at
  // least 1), a tolerance value, max_nodes or max_refinements that is not
  // finite and positive, a tolerance when no adaptation estimates, a
  // damping method that does not support the problem's time order or that
  // gives no acceleration at the end of its steps when the integrator's
  // method keeps one (naming both methods), a starting mesh
  // with more nodes than max_nodes, snapshot times not strictly
  // increasing within [t0, t1], and snapshot_every; throws
  // std::runtime_error when an adaptation breaks its declarations and
  // when t + interval rounds to t (an interval too small at the current
  // time); passes
  // on the exceptions of the integrator, the adaptations, the transfer
  // and the MeshedProblem constructor.
  [[nodiscard]] AdaptiveIntegrationResult
  integrate(std::shared_ptr<const fem::MeshedProblem> problem, double t0,
            double t1, const AdaptiveTimeTarget &target,
            const solve::IntegrationOutput &output = {},
            const solve::Solution *initial = nullptr) const;

private:
  solve::Integrator integrator_;
  std::vector<std::shared_ptr<const Adaptation>> adaptations_;
  AdaptiveIntegratorOptions options_;
  // The integrator with the damping method, when damping steps are asked
  // for.
  std::optional<solve::Integrator> damping_;
};

} // namespace pde2d::adapt
