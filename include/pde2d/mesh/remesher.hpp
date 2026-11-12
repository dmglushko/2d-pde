#pragma once

#include "pde2d/geometry/curve_geometry.hpp"
#include "pde2d/mesh/components.hpp"
#include "pde2d/mesh/mesh.hpp"
#include "pde2d/mesh/mesh_spec.hpp"
#include "pde2d/mesh/report.hpp"
#include "pde2d/mesh/working_mesh.hpp"

#include <memory>
#include <vector>

namespace pde2d::mesh {

// Refines and improves a mesh that already exists: the optional refiner
// (calling the local improvers after each change), then the final passes
// in order. Checks "target met" on the final mesh.
//
// The first report of every run is the "pipeline" report: the plan
// resolved at construction and its warnings (see pipeline_report()).
class Remesher {
public:
  // Resolves the improvers' use modes against the refiner (plan_pipeline).
  // A mode that cannot be honoured is a warning in the pipeline report.
  // The refiner may be null; a null improver throws
  // std::invalid_argument.
  explicit Remesher(
      std::shared_ptr<const Refiner> refiner,
      std::vector<std::shared_ptr<const Improver>> improvers = {});

  // Which improvers run locally inside the refiner and which as final
  // passes, and the warnings found when the plan was made.
  const PipelinePlan &plan() const { return plan_; }

  // A mesh without geometry: split boundary edges get their new point at
  // the chord midpoint.
  [[nodiscard]] MeshResult run(const Mesh &mesh, const MeshSpec &spec) const;

  // A mesh that lies on `geometry`: each boundary edge is first linked to
  // the curve of its region that contains both its end points, so split
  // boundary edges get their new point on the curve. An edge whose end
  // points lie on two different curves (across a junction, or across the
  // start of a closed curve) stays unlinked; the "curve_links" report says
  // how many. Throws std::invalid_argument if a mesh region is not in the
  // geometry or a boundary node is not on a curve of its region.
  [[nodiscard]] MeshResult run(const Mesh &mesh,
                               const geometry::CurveGeometry &geometry,
                               const MeshSpec &spec) const;

  // The working mesh as it is, with whatever links it has. The Mesher
  // calls this after triangulation.
  [[nodiscard]] MeshResult run(WorkingMesh mesh, const MeshSpec &spec) const;

  // The "pipeline" report that starts every run's reports. Messages are
  // the plan's warnings. Metrics: "local_improvers" and "final_passes"
  // (counts); "refiner.<name>" = 1 if there is a refiner; one
  // "local.<k>.<name>" per local improver and one "final.<k>.<name>" per
  // final pass, where k is the position in call order and the value is
  // the improver's index in the improver list.
  const StageReport &pipeline_report() const { return pipeline_report_; }

private:
  std::shared_ptr<const Refiner> refiner_;
  std::vector<std::shared_ptr<const Improver>> improvers_;
  PipelinePlan plan_;
  std::vector<const LocalImprover *> local_improvers_;
  StageReport pipeline_report_;
};

} // namespace pde2d::mesh
