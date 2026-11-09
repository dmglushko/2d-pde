#pragma once
// Records of an adaptive loop (adaptivity.hpp), apart from the drivers so
// planners (planner.hpp) can read the previous step without the adaptive
// implementation. One entry type serves both drivers: AdaptiveStep is one
// solve (adapt) or one time window (adapt_evolution) with what was
// estimated, marked and changed afterwards; window data is set only for
// windows. Contract header: no assembly, symbolic or estimator headers.
#include "discretization/mesh_types.hpp"
#include "fem/decision.hpp"
#include "fem/record.hpp"
#include "provenance.hpp"
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace pde2d::fem {
// Why an adaptive loop ended. EndReached: adapt_evolution evolved its last
// window (it has no other stop).
enum class AdaptiveStop {
  ToleranceReached,
  MaxIterations,
  NothingMarked,
  UnsatisfiedRefinement,
  EndReached
};
// Stable names (adaptive history JSON).
inline const char *adaptive_stop_name(AdaptiveStop stop) {
  switch (stop) {
  case AdaptiveStop::ToleranceReached:
    return "tolerance reached";
  case AdaptiveStop::MaxIterations:
    return "max iterations";
  case AdaptiveStop::NothingMarked:
    return "nothing marked";
  case AdaptiveStop::UnsatisfiedRefinement:
    return "unsatisfied refinement";
  case AdaptiveStop::EndReached:
    return "end reached";
  }
  return "invalid";
}

// How one nodal field moved from the previous mesh (transfer_field). On a
// transient restart its Dirichlet nodes were then set to the problem's data
// at the restart time: how many, and the largest change that made.
struct TransferRecord {
  std::size_t source_nodes = 0, target_nodes = 0;
  std::size_t extrapolated_nodes = 0;
  double max_outside_distance = 0;
  std::size_t dirichlet_nodes = 0;
  double max_dirichlet_correction = 0;
};
// What a step's ErrorEstimate said, without the indicators.
struct EstimateRecord {
  std::string estimator{}; // ErrorEstimator::name()
  std::optional<pde2d::StrategyDescriptor> descriptor{};
  double global = 0;
  std::size_t elements = 0;
  // Largest indicator and the mean (sum in element order / elements).
  double max_indicator = 0, mean_indicator = 0;
};
// Which elements a step marked, and by what strategy.
struct MarkingRecord {
  std::string strategy{}; // MarkingStrategy::name()
  std::optional<pde2d::StrategyDescriptor> descriptor{};
  std::size_t marked = 0, elements = 0;
};
// How a step changed the mesh (MeshAdaptation, mesh_adaptation.hpp): the
// strategy, the report of the new mesh and its mesh run record (operation,
// fingerprints, the options it was called with, work, quality), and whether
// the next solve uses it (false: it missed its size target and
// AdaptOptions::accept_unsatisfied_refinement is false).
struct MeshAdaptationRecord {
  std::string strategy{}; // MeshAdaptation::name()
  std::optional<pde2d::StrategyDescriptor> descriptor{};
  discretization::MeshReport report{};
  discretization::MeshRunRecord run{};
  bool accepted = false;
};

struct AdaptiveStep {
  std::size_t index = 0;
  // The solve or evolution on this step's mesh: plan, mesh fingerprint and
  // provenance, initial guess or initial fields, work.
  RunRecord record{};
  // The time of the estimate: the stationary solve time, or the end of the
  // window. Windows also set their start.
  double time = 0;
  std::optional<double> window_start{};
  // The transfers that seeded this step from the previous mesh: the warm
  // start of a solve (adapt with warm_start), or a restart's values and
  // velocity (adapt_evolution). Empty on the first step and on steps that
  // kept the previous mesh.
  std::optional<TransferRecord> transfer{}, velocity_transfer{};
  EstimateRecord estimate{};
  // Unless the loop stopped (or skipped refinement) before marking: what was
  // marked, and, if anything was, how the mesh changed.
  std::optional<MarkingRecord> marking{};
  std::optional<MeshAdaptationRecord> adaptation{};
  // The choices of this step, in this order: the solve planner that
  // planned its solve (DecisionSubject::SolvePlanner, only with a solve
  // planner), the marking (Marking), the mesh planner (MeshPlanner, only
  // with a mesh planner) and the mesh change (Refinement).
  std::vector<Decision> decisions{};
};
} // namespace pde2d::fem
