#pragma once

#include "pde2d/geometry/curve_geometry.hpp"
#include "pde2d/mesh/components.hpp"
#include "pde2d/mesh/mesh_spec.hpp"
#include "pde2d/mesh/remesher.hpp"
#include "pde2d/mesh/report.hpp"

#include <memory>
#include <vector>

namespace pde2d::mesh {

// Meshes a geometry with a fixed sequence of stages:
//   BoundaryDiscretizer -> InitialTriangulator -> Refiner (optional)
//   -> Improvers.
// The last two run in an internal Remesher.
class Mesher {
public:
  // Checks the components against each other and throws
  // std::invalid_argument on a mismatch or a null discretizer/triangulator.
  Mesher(std::shared_ptr<const BoundaryDiscretizer> discretizer,
         std::shared_ptr<const InitialTriangulator> triangulator,
         std::shared_ptr<const Refiner> refiner = nullptr,
         std::vector<std::shared_ptr<const Improver>> improvers = {});

  // Before any stage runs, checks the spec and the components against the
  // input (for example a triangulator without hole support against a
  // geometry with holes) and throws std::invalid_argument on a mismatch.
  // Stage failures throw with a specific message. The result says whether
  // every final edge is at most spec.max_edge_length.
  [[nodiscard]] MeshResult run(const geometry::CurveGeometry &geometry,
                               const MeshSpec &spec) const;

private:
  std::shared_ptr<const BoundaryDiscretizer> discretizer_;
  std::shared_ptr<const InitialTriangulator> triangulator_;
  Remesher remesher_;
};

} // namespace pde2d::mesh
