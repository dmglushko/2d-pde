#include "pde2d/adapt/mesh_refinement.hpp"

#include <chrono>
#include <format>
#include <stdexcept>
#include <utility>

namespace pde2d::adapt {

MeshRefinement::MeshRefinement(std::shared_ptr<const ErrorEstimator> estimator,
                               std::shared_ptr<const MarkingStrategy> marking,
                               std::shared_ptr<const mesh::Remesher> remesher)
    : estimator_(std::move(estimator)), marking_(std::move(marking)),
      remesher_(std::move(remesher)) {
  if (!estimator_)
    throw std::invalid_argument("mesh_refinement: the estimator is null");
  if (!marking_)
    throw std::invalid_argument(
        "mesh_refinement: the marking strategy is null");
  if (!remesher_)
    throw std::invalid_argument("mesh_refinement: the remesher is null");
}

std::optional<ErrorEstimate>
MeshRefinement::estimate(const SolvedState &state) const {
  return estimator_->estimate(state.problem, state.solution, state.derivatives);
}

AdaptationResult MeshRefinement::adapt(const SolvedState &state,
                                       const ErrorEstimate *estimate) const {
  const mesh::Mesh &mesh = *state.problem.mesh();
  if (state.solution.mesh != state.problem.mesh())
    throw std::invalid_argument(
        "mesh_refinement: the solution is not on the problem's mesh");
  AdaptationResult result;
  result.estimate = estimate
                        ? *estimate
                        : estimator_->estimate(state.problem, state.solution,
                                               state.derivatives);
  if (result.estimate->indicators.size() != mesh.triangles.size())
    throw std::invalid_argument(std::format(
        "mesh_refinement: the estimate has {} indicators for {} triangles",
        result.estimate->indicators.size(), mesh.triangles.size()));
  result.reports.push_back(result.estimate->report);

  const auto start = std::chrono::steady_clock::now();
  const std::vector<std::size_t> marked =
      marking_->mark(mesh, result.estimate->indicators);
  mesh::StageReport report;
  report.stage = name();
  report.messages.push_back("marking: " + marking_->name());
  report.metrics["triangles"] = static_cast<double>(mesh.triangles.size());
  report.metrics["marked"] = static_cast<double>(marked.size());
  report.time = std::chrono::steady_clock::now() - start;
  if (marked.empty()) {
    report.messages.push_back("nothing marked: the mesh is unchanged");
    result.reports.push_back(std::move(report));
    return result;
  }
  result.reports.push_back(std::move(report));

  mesh::RefinementResult refined = remesher_->refine(mesh, marked);
  for (auto &r : refined.reports)
    result.reports.push_back(std::move(r));
  result.mesh = std::make_shared<const mesh::Mesh>(std::move(refined.mesh));
  result.properties = refined.properties;
  result.parents = std::move(refined.parents);
  return result;
}

} // namespace pde2d::adapt
