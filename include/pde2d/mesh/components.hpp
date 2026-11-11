#pragma once

#include "pde2d/geometry/curve_geometry.hpp"
#include "pde2d/mesh/boundary_polygon.hpp"
#include "pde2d/mesh/mesh_spec.hpp"
#include "pde2d/mesh/report.hpp"
#include "pde2d/mesh/working_mesh.hpp"

#include <cstddef>
#include <memory>
#include <span>
#include <string>
#include <vector>

// The interfaces of the meshing stages. The stages always run in this
// order:
//   BoundaryDiscretizer -> InitialTriangulator -> Refiner (optional)
//   -> Improvers (a list, in order).
// Every component has a name (used as StageReport::stage and in error
// messages) and declares capabilities; compatibility is decided from the
// declarations, never from names.
namespace pde2d::mesh {

// --- capabilities -----------------------------------------------------------

// What an initial triangulator can handle. Checked against the geometry
// when a Mesher run starts.
struct TriangulatorCapabilities {
  bool supports_holes = false;          // loops of odd depth
  bool supports_several_pieces = false; // more than one loop of even depth
};

// What an improver (whole-mesh or local) does to a mesh.
struct ImproverEffects {
  bool flips_edges = false;
  bool moves_nodes = false;
  bool makes_delaunay = false; // the mesh is Delaunay afterwards
};

// What a refiner tolerates from the local improver it is given, and what
// it already maintains in its output.
struct RefinerCapabilities {
  bool tolerates_flips = false;
  bool tolerates_node_moves = false;
  bool maintains_delaunay = false;
};

// --- interfaces -------------------------------------------------------------

class BoundaryDiscretizer {
public:
  virtual ~BoundaryDiscretizer() = default;
  virtual std::string name() const = 0;
  // Replaces each loop of exact curves by straight edges.
  virtual BoundaryPolygon run(const geometry::CurveGeometry &geometry,
                              const MeshSpec &spec,
                              StageReport &report) const = 0;
};

class InitialTriangulator {
public:
  virtual ~InitialTriangulator() = default;
  virtual std::string name() const = 0;
  virtual TriangulatorCapabilities capabilities() const = 0;
  // Triangulates a valid polygon (see validate(BoundaryPolygon)). Every
  // polygon edge becomes a boundary edge with its region and link.
  virtual WorkingMesh run(const BoundaryPolygon &polygon,
                          StageReport &report) const = 0;
};

// Called by a refiner after each change, on the triangles it touched.
class LocalImprover {
public:
  virtual ~LocalImprover() = default;
  virtual std::string name() const = 0;
  virtual ImproverEffects effects() const = 0;
  // Returns the number of changes made (flips, moves, ...).
  virtual std::size_t
  improve_around(WorkingMesh &mesh,
                 std::span<const std::size_t> touched_triangles) const = 0;
};

class Refiner {
public:
  virtual ~Refiner() = default;
  virtual std::string name() const = 0;
  virtual RefinerCapabilities capabilities() const = 0;
  // The local improver this refiner was given in its options, if any.
  virtual const LocalImprover *local_improver() const { return nullptr; }
  // Adds nodes until the spec (and the refiner's own goals) are met.
  // Updates boundary edges (regions and links) as it splits them.
  virtual void run(WorkingMesh &mesh, const MeshSpec &spec,
                   StageReport &report) const = 0;
};

// A pass over the whole mesh.
class Improver {
public:
  virtual ~Improver() = default;
  virtual std::string name() const = 0;
  virtual ImproverEffects effects() const = 0;
  virtual void run(WorkingMesh &mesh, const MeshSpec &spec,
                   StageReport &report) const = 0;
};

// Component-against-component check, done when a Remesher (and therefore a
// Mesher) is constructed. Throws std::invalid_argument with the reason
// when
// - the refiner's local improver has an effect the refiner does not
//   tolerate (for example node moves);
// - an improver would make the mesh Delaunay while it already is: right
//   after a refiner that maintains Delaunay ("ruppert already maintains
//   Delaunay"), or after another improver that made it Delaunay with no
//   flip or move in between;
// - a component pointer is null.
void check_compatibility(
    const Refiner *refiner,
    const std::vector<std::shared_ptr<const Improver>> &improvers);

} // namespace pde2d::mesh
