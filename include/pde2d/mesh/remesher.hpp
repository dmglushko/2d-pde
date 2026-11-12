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

  // A mesh as it is: boundary edges with a curve link (a mesh made by the
  // Mesher or Remesher carries its chains and links) get their new point
  // on the true curve; edges without one (an imported mesh) at their chord
  // midpoint.
  [[nodiscard]] MeshResult run(const Mesh &mesh, const MeshSpec &spec) const;

  // A mesh that lies on `geometry`, for a mesh without links of its own
  // (any links it has are replaced). The geometry's loops are first
  // joined into chains exactly as the ChordLengthDiscretizer does (a
  // junction is a break where the region changes or the boundary turns by
  // more than corner_angle_degrees, in [0, 180]); then each boundary edge
  // is linked to the chain of its region that contains both its end
  // points, so split boundary edges get their new point on the curve. An
  // edge whose end points lie on two different chains (across a corner,
  // a region change, or the start of a closed loop) stays unlinked; the
  // "curve_links" report (right after "pipeline") says how many. Throws
  // std::invalid_argument for an angle outside [0, 180], a mesh region
  // that is not in the geometry, or a boundary node that is not on a curve
  // of its region.
  [[nodiscard]] MeshResult
  run(const Mesh &mesh, const geometry::CurveGeometry &geometry,
      const MeshSpec &spec,
      double corner_angle_degrees = default_corner_angle_degrees) const;

  // The working mesh as it is, with whatever links it has. The Mesher
  // calls this after triangulation.
  [[nodiscard]] MeshResult run(WorkingMesh mesh, const MeshSpec &spec) const;

  // The "pipeline" report that starts every run's reports. Its first
  // message is the plan in words, for example "plan: refiner ruppert;
  // local: none; final: delaunay_flips, laplacian_smoothing" ("no refiner"
  // without one); the plan's warnings follow as further messages. Metrics:
  // "local_improvers" and "final_passes" (counts). Its time is the time
  // the plan took to resolve.
  const StageReport &pipeline_report() const { return pipeline_report_; }

private:
  std::shared_ptr<const Refiner> refiner_;
  std::vector<std::shared_ptr<const Improver>> improvers_;
  PipelinePlan plan_;
  std::vector<const LocalImprover *> local_improvers_;
  std::vector<const FinalImprover *> final_improvers_;
  StageReport pipeline_report_;
};

} // namespace pde2d::mesh
