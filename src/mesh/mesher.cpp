#include "pde2d/mesh/mesher.hpp"

#include "stage.hpp"

#include <stdexcept>

namespace pde2d::mesh {

Mesher::Mesher(std::shared_ptr<const BoundaryDiscretizer> discretizer,
               std::shared_ptr<const InitialTriangulator> triangulator,
               std::shared_ptr<const Refiner> refiner,
               std::vector<std::shared_ptr<const Improver>> improvers)
    : discretizer_(std::move(discretizer)),
      triangulator_(std::move(triangulator)),
      remesher_(std::move(refiner), std::move(improvers)) {
  if (!discretizer_)
    throw std::invalid_argument("Mesher needs a boundary discretizer");
  if (!triangulator_)
    throw std::invalid_argument("Mesher needs an initial triangulator");
}

MeshResult Mesher::run(const geometry::CurveGeometry &geometry,
                       const MeshSpec &spec) const {
  validate(spec);
  // Component against input, before any stage runs.
  std::size_t holes = 0, pieces = 0;
  for (std::size_t l = 0; l < geometry.loops().size(); ++l)
    (geometry.depth(l) % 2 == 0 ? pieces : holes) += 1;
  auto caps = triangulator_->capabilities();
  if (holes > 0 && !caps.supports_holes)
    throw std::invalid_argument(triangulator_->name() +
                                " does not support holes; the geometry has " +
                                std::to_string(holes));
  if (pieces > 1 && !caps.supports_several_pieces)
    throw std::invalid_argument(
        triangulator_->name() +
        " does not support several pieces; the geometry has " +
        std::to_string(pieces));

  StageReport boundary_report;
  BoundaryPolygon polygon =
      detail::timed_stage(boundary_report, discretizer_->name(), [&](auto &r) {
        return discretizer_->run(geometry, spec, r);
      });
  try {
    validate(polygon);
  } catch (const std::invalid_argument &e) {
    throw std::runtime_error(
        discretizer_->name() +
        " produced an invalid boundary polygon: " + e.what());
  }
  StageReport triangulation_report;
  WorkingMesh working = detail::timed_stage(
      triangulation_report, triangulator_->name(),
      [&](auto &r) { return triangulator_->run(polygon, r); });

  MeshResult result = remesher_.run(std::move(working), spec);
  result.reports.insert(
      result.reports.begin(),
      {std::move(boundary_report), std::move(triangulation_report)});
  return result;
}

} // namespace pde2d::mesh
