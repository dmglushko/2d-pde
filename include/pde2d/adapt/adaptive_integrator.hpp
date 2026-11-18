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
  std::optional<double> interval{};     // finite and positive
  std::optional<std::size_t> steps{};   // at least 1
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
};

struct AdaptiveIntegrationResult {
  // The solution at t1 on the last mesh (with velocities for time order
  // 2).
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
  // adaptation_times, refinements, steps, nodes and triangles (of the
  // last mesh), max_nodes_used (the most nodes any piece integrated on,
  // the last mesh's since meshes only grow) and tolerance (resolved, when
  // set); a message when the node budget stopped adapting.
  mesh::StageReport report{};
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
// is t1) or after `steps` steps (Integrator::integrate's max_steps),
// whichever comes first, and the run ends at t1, where it does not adapt.
// Each adaptation time refines at most once, without integrating the
// piece again: the mesh follows the solution one adaptation time behind,
// and adapting more often follows it more closely.
//
// The estimate at an adaptation time is that of the solution there with
// its time derivatives from the last step: time order 1 (or 0) u_t =
// (u(t) - u(t - dt)) / dt; time order 2 u_t = the integrator's
// velocities, u_tt = (u_t(t) - u_t(t - dt)) / dt, all at every node (the
// Dirichlet nodes from the boundary data). The cycle's estimate is that of
// the first adaptation that declares estimates_error, which is then given
// it. When the estimate is at most the tolerance nothing adapts;
// otherwise every adaptation adapts in order. A new mesh with more nodes
// than max_nodes is discarded and the run adapts no more (the mesh only
// grows); the rest is one piece to t1. So a run always reaches t1 unless
// something throws.
//
// The state passed on is the previous piece's final solution, on the
// same problem when nothing changed (so the pieces continue the run
// exactly: with a first-order method bit for bit like one integrate() call
// that lands on the same times), else transferred to the new mesh
// (values, and velocities for time order 2; Dirichlet nodes from the new
// problem).
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
  // Throws std::invalid_argument without adaptations, for a null one, and
  // when more than one declares changes_mesh.
  AdaptiveIntegrator(solve::Integrator integrator,
                     std::vector<std::shared_ptr<const Adaptation>> adaptations);

  // Integrates `problem` (its mesh is the starting mesh) from t0 to t1,
  // adapting as the target says; `initial` as for Integrator::integrate
  // (on the problem's mesh). Throws std::invalid_argument for a null
  // problem, times that are not finite or t1 <= t0, a schedule with
  // nothing set or a value that is not finite and positive (steps at
  // least 1), a tolerance value or max_nodes that is not finite and
  // positive, a tolerance when no adaptation estimates, a starting mesh
  // with more nodes than max_nodes, snapshot times not strictly
  // increasing within [t0, t1], and snapshot_every; throws
  // std::runtime_error when an adaptation breaks its declarations; passes
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
};

} // namespace pde2d::adapt
