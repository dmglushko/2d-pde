#pragma once

#include "pde2d/geometry/curve_geometry.hpp"
#include "pde2d/mesh/components.hpp"
#include "pde2d/mesh/mesh_properties.hpp"
#include "pde2d/mesh/mesh_spec.hpp"
#include "pde2d/mesh/remesher.hpp"
#include "pde2d/mesh/report.hpp"

#include <memory>
#include <vector>

namespace pde2d::mesh {

struct MesherOptions {
  // As RemesherOptions::warning_angle_degrees, for the internal Remesher.
  double warning_angle_degrees = default_warning_angle_degrees;
};

// Meshes a geometry with a fixed sequence of stages:
//   BoundaryDiscretizer -> InitialTriangulator -> Refiner (optional)
//   -> Improvers.
// The last two run in an internal Remesher, which resolves the improvers'
// use modes; the reports of a run start with its "pipeline" report.
class Mesher {
public:
  // Resolves the improvers' use modes against the refiner (warnings go to
  // the pipeline report) and throws std::invalid_argument for a null
  // discretizer, triangulator or improver, or a warning angle outside
  // [0, 60].
  Mesher(std::shared_ptr<const BoundaryDiscretizer> discretizer,
         std::shared_ptr<const InitialTriangulator> triangulator,
         std::shared_ptr<const Refiner> refiner = nullptr,
         std::vector<std::shared_ptr<const Improver>> improvers = {},
         MesherOptions options = {});

  // Before any stage runs, checks the spec and the components against the
  // input (for example a triangulator without hole support against a
  // geometry with holes) and throws std::invalid_argument on a mismatch.
  // Stage failures throw with a specific message. The result says whether
  // every final edge is at most spec.max_edge_length, resolved against
  // domain_size (MeshResult::max_edge_length), and the properties of the
  // final mesh (MeshResult::properties; the pipeline report warns about
  // triangles below the warning angle). Reports, in order: pipeline,
  // discretizer, triangulator, refiner, final passes.
  [[nodiscard]] MeshResult run(const geometry::CurveGeometry &geometry,
                               const MeshSpec &spec) const;

  // The plan of the internal Remesher.
  const PipelinePlan &plan() const { return remesher_.plan(); }
  const MesherOptions &options() const { return options_; }

private:
  std::shared_ptr<const BoundaryDiscretizer> discretizer_;
  std::shared_ptr<const InitialTriangulator> triangulator_;
  MesherOptions options_;
  Remesher remesher_;
};

} // namespace pde2d::mesh
