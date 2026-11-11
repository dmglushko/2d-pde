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

// Refines and improves a mesh that already exists: the optional refiner,
// then the improvers in order. Checks "target met" on the final mesh.
class Remesher {
public:
  // Checks the components against each other (check_compatibility) and
  // throws std::invalid_argument on a mismatch. The refiner may be null.
  explicit Remesher(
      std::shared_ptr<const Refiner> refiner,
      std::vector<std::shared_ptr<const Improver>> improvers = {});

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

private:
  std::shared_ptr<const Refiner> refiner_;
  std::vector<std::shared_ptr<const Improver>> improvers_;
};

} // namespace pde2d::mesh
