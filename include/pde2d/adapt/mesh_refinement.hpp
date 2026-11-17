#pragma once

#include "pde2d/adapt/adaptation.hpp"
#include "pde2d/adapt/error_estimator.hpp"
#include "pde2d/adapt/marking.hpp"
#include "pde2d/mesh/remesher.hpp"

#include <memory>

namespace pde2d::adapt {

// Adaptive mesh refinement as one adaptation: estimate, mark, refine.
// Each part is a component configured with its own options (the
// estimator's, the marking strategy's theta, the Remesher's refiner and
// improvers).
//
// adapt(): the estimator's indicators on the solution's mesh (or the given
// estimate), the marking strategy's marked triangles, and
// Remesher::refine of the solution's mesh. Nothing marked (all indicators
// zero) changes nothing: the result has no mesh and the Remesher is not
// called. Reports, in order: the estimator's, "mesh_refinement" (metrics
// marked and triangles; a message naming the marking strategy), then
// the Remesher's (pipeline, refiner, final passes).
//
// The refinement edges of newest vertex bisection and red-green-blue are
// the stored vertex order (mesh.hpp): call
// mesh::set_longest_edges_as_refinement_edges once on the starting mesh,
// before building the first MeshedProblem; this adaptation never resets
// them, since between cycles the order is the method's memory.
class MeshRefinement final : public Adaptation {
public:
  // Throws std::invalid_argument for a null component.
  MeshRefinement(std::shared_ptr<const ErrorEstimator> estimator,
                 std::shared_ptr<const MarkingStrategy> marking,
                 std::shared_ptr<const mesh::Remesher> remesher);

  std::string name() const override { return "mesh_refinement"; }
  AdaptationCapabilities capabilities() const override {
    return {.estimates_error = true, .changes_mesh = true};
  }

  // The estimator's estimate of the state.
  [[nodiscard]] std::optional<ErrorEstimate>
  estimate(const SolvedState &state) const override;

  // Throws std::invalid_argument when the solution is not on the
  // problem's mesh or a given estimate does not have one indicator per
  // triangle, and passes on the exceptions of its components.
  [[nodiscard]] AdaptationResult
  adapt(const SolvedState &state, const ErrorEstimate *estimate) const override;

  const ErrorEstimator &estimator() const { return *estimator_; }
  const MarkingStrategy &marking() const { return *marking_; }
  const mesh::Remesher &remesher() const { return *remesher_; }

private:
  std::shared_ptr<const ErrorEstimator> estimator_;
  std::shared_ptr<const MarkingStrategy> marking_;
  std::shared_ptr<const mesh::Remesher> remesher_;
};

} // namespace pde2d::adapt
