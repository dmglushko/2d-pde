#pragma once

#include "pde2d/geometry/curve_geometry.hpp"
#include "pde2d/mesh/components.hpp"
#include "pde2d/mesh/mesh.hpp"
#include "pde2d/mesh/mesh_properties.hpp"
#include "pde2d/mesh/mesh_spec.hpp"
#include "pde2d/mesh/report.hpp"
#include "pde2d/mesh/working_mesh.hpp"

#include <cstddef>
#include <memory>
#include <span>
#include <vector>

namespace pde2d::mesh {

struct RemesherOptions {
  // The final mesh's properties count the triangles with an angle below
  // this (degrees, in [0, 60]; 0 counts none), and the pipeline report
  // warns when there are any. Nothing is refused.
  double warning_angle_degrees = default_warning_angle_degrees;
};

// Refines and improves a mesh that already exists: the optional refiner
// (calling the local improvers after each change), then the final passes
// in order. Two ways: run() refines towards a MeshSpec and checks "target
// met"; refine() refines marked triangles once (adaptive refinement). Both
// measure the final mesh once (MeshProperties, after the last improver).
//
// Both deliver the refinement-edge order (mesh.hpp) after the final
// passes: run() makes every triangle's longest edge its refinement edge;
// refine() does so only for the triangles whose vertex order an edit chose
// (made or changed by a flip, an insertion such as Ruppert's, or a
// re-triangulating boundary split, during the refiner or a final pass),
// while the triangles a bisection made and those nothing touched keep
// their order, newest vertex bisection's memory between cycles. Moving a
// node (smoothing) keeps a triangle's order. The pipeline report's metric
// "refinement_edges_set" counts the triangles whose vertex 0 changed.
//
// The first report of every run is the "pipeline" report: the plan
// resolved at construction and its warnings (see pipeline_report()), and
// after the run the warnings about the final mesh: triangles below the
// warning angle, and (refine) why the parents were dropped.
class Remesher {
public:
  // Resolves the improvers' use modes against the refiner (plan_pipeline).
  // A mode that cannot be honoured is a warning in the pipeline report.
  // The refiner may be null; a null improver or a warning angle outside
  // [0, 60] throws std::invalid_argument.
  explicit Remesher(std::shared_ptr<const Refiner> refiner,
                    std::vector<std::shared_ptr<const Improver>> improvers = {},
                    RemesherOptions options = {});
  const RemesherOptions &options() const { return options_; }

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

  // Refines the marked triangles of the mesh once, with the refiner's
  // refine() (indices into mesh.triangles, any order, repetitions
  // allowed), then runs the final passes like run(); links are used as in
  // run(mesh, spec). The result carries the parents only when no
  // component made them untrue: the refiner declares keeps_parents and
  // kept them in this run, and every local improver and final pass of the
  // plan declares keeps_parents; otherwise the pipeline report names the
  // components that dropped them. Final passes get a MeshSpec without an
  // effective length bound (refinement has no length target). Reports:
  // pipeline, refiner, final passes. Throws std::invalid_argument without
  // a refiner, for an invalid mesh (validate) or a marked index out of
  // range, and std::runtime_error if the refined mesh overlaps itself (a
  // new curve point on a boundary edge reached another part of the mesh).
  [[nodiscard]] RefinementResult
  refine(const Mesh &mesh, std::span<const std::size_t> marked_triangles) const;

  // The "pipeline" report that starts every run's reports. Its first
  // message is the plan in words, for example "plan: refiner ruppert;
  // local: none; final: delaunay_flips, laplacian_smoothing" ("no refiner"
  // without one); the plan's warnings follow as further messages. Metrics:
  // "local_improvers" and "final_passes" (counts); a run adds
  // "refinement_edges_set". Its time is the time the plan took to
  // resolve.
  const StageReport &pipeline_report() const { return pipeline_report_; }

private:
  // Measures the final mesh and adds its warnings to the pipeline report
  // (reports[0]).
  MeshProperties finish(const Mesh &mesh, StageReport &pipeline) const;

  std::shared_ptr<const Refiner> refiner_;
  std::vector<std::shared_ptr<const Improver>> improvers_;
  RemesherOptions options_;
  PipelinePlan plan_;
  std::vector<const LocalImprover *> local_improvers_;
  std::vector<const FinalImprover *> final_improvers_;
  StageReport pipeline_report_;
};

} // namespace pde2d::mesh
