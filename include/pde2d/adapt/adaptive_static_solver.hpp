#pragma once

#include "pde2d/adapt/adaptation.hpp"
#include "pde2d/adapt/error_estimator.hpp"
#include "pde2d/fem/meshed_problem.hpp"
#include "pde2d/mesh/report.hpp"
#include "pde2d/solve/solution.hpp"
#include "pde2d/solve/static_solver.hpp"

#include <cstddef>
#include <memory>
#include <optional>
#include <string_view>
#include <vector>

namespace pde2d::adapt {

// A tolerance for the estimated error (ErrorEstimate::total): absolute,
// relative to the first cycle's estimate, or both; with both, the smaller
// one applies (like mesh::LengthBound). Neither set is no tolerance.
struct ErrorTolerance {
  std::optional<double> absolute{};
  std::optional<double> relative{}; // times the first cycle's estimate
};

// The job's target. At least one of the three must be set.
struct AdaptiveTarget {
  // Stop when the estimated error is at most this.
  ErrorTolerance tolerance{};
  // At most this many cycles (solves), at least 1.
  std::optional<std::size_t> max_cycles{};
  // No solve on a mesh with more nodes than this, at least 1.
  std::optional<std::size_t> max_nodes{};
};

enum class StopReason {
  tolerance_met,   // the estimated error reached the tolerance
  max_cycles,      // the last allowed cycle was solved
  node_budget,     // the adapted mesh would exceed max_nodes (not solved)
  nothing_changed, // no adaptation changed the mesh (nothing was marked)
};

// "tolerance_met", "max_cycles", "node_budget", "nothing_changed".
std::string_view to_string(StopReason reason);

// One cycle: a solve on one mesh, its estimate, and the adaptations that
// followed it (none in the last cycle, unless their mesh was discarded
// for the node budget).
struct AdaptiveCycle {
  solve::Solution solution{};
  // The estimate the target is tested with (see AdaptiveStaticSolver);
  // nothing when no adaptation estimates.
  std::optional<ErrorEstimate> estimate{};
  // In order: "meshed_problem" (cycles after the first, whose problem the
  // driver built), the StaticSolver's reports, then each adaptation's
  // reports.
  std::vector<mesh::StageReport> reports{};
};

struct AdaptiveStaticResult {
  // Every cycle, in order; the last one holds the final solution.
  std::vector<AdaptiveCycle> cycles{};
  // The problem of the last cycle (on its mesh).
  std::shared_ptr<const fem::MeshedProblem> problem{};
  StopReason stop_reason = StopReason::max_cycles;
  // "adaptive_static_solver": the time taken; the metrics cycles, nodes
  // and triangles (of the last cycle), estimate (its total, when there is
  // one) and tolerance (resolved, when set); a message saying why it
  // stopped.
  mesh::StageReport report{};
};

// The adaptive loop for static problems: a driver above the StaticSolver.
//
//   cycle k: solve (StaticSolver at t) -> estimate -> stop? -> adapt
//            -> a new MeshedProblem from the same Problem on the new mesh
//
// The cycle's estimate is that of the first adaptation that declares
// estimates_error; the adaptation is then given it, so it is computed
// once. A static solution is a steady state, so the derivatives passed
// are zeros. Stop, in this order:
// 1. the estimate's total is at most the tolerance (tolerance_met);
// 2. the cycle was the max_cycles-th (max_cycles);
// 3. then every adaptation acts, in order, on the cycle's solved state;
//    none returned a mesh (nothing_changed);
// 4. the new mesh has more nodes than max_nodes: it is discarded and not
//    solved (node_budget).
// So the last cycle's mesh never has more than max_nodes nodes.
//
// Until a solution can be transferred between meshes, at most one
// adaptation may change the mesh (declared by changes_mesh): all act on
// the same solved state.
//
// Refinement edges: the loop never reorders triangles. For newest vertex
// bisection or red-green-blue, call
// mesh::set_longest_edges_as_refinement_edges once on the starting mesh,
// before building the MeshedProblem passed to solve().
class AdaptiveStaticSolver {
public:
  // Throws std::invalid_argument without adaptations, for a null one, and
  // when more than one declares changes_mesh.
  AdaptiveStaticSolver(
      solve::StaticSolver solver,
      std::vector<std::shared_ptr<const Adaptation>> adaptations);

  // Runs the loop from `problem` (its mesh is the starting mesh) at time
  // t. Throws std::invalid_argument for a null problem, a t that is not
  // finite, a target with nothing set, a tolerance value or budget that
  // is not finite and positive (max_cycles, max_nodes at least 1), a
  // tolerance when no adaptation estimates, and a starting mesh with more
  // nodes than max_nodes; throws std::runtime_error when an adaptation
  // breaks its declarations (declares estimates_error but gives no
  // estimate, returns a mesh without declaring changes_mesh); passes on
  // the exceptions of the solver, the adaptations and the MeshedProblem
  // constructor.
  [[nodiscard]] AdaptiveStaticResult
  solve(std::shared_ptr<const fem::MeshedProblem> problem,
        const AdaptiveTarget &target, double t = 0.0) const;

private:
  solve::StaticSolver solver_;
  std::vector<std::shared_ptr<const Adaptation>> adaptations_;
};

} // namespace pde2d::adapt
