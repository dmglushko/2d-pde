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

// What a refiner tolerates from local improvers during refinement, and
// what it already maintains in its output. When the pipeline is
// assembled, these decide which improvers the refiner accepts as local
// improvers (see plan_pipeline).
struct RefinerCapabilities {
  bool tolerates_flips = false;
  bool tolerates_node_moves = false;
  bool maintains_delaunay = false;
};

// How an improver given to the Mesher (or Remesher) wants to be used:
// locally, inside the refiner after each change, as a final pass over the
// whole mesh, or both. Each improver carries it in its options (`use`).
enum class ImproverUse { local_and_final, local_only, final_only };

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

// Called by a refiner after each change, on the triangles it touched. The
// refiner gets its local improvers from the Mesher (or Remesher), never
// from its own options.
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
  // Adds nodes until the spec (and the refiner's own goals) are met.
  // Updates boundary edges (regions and links) as it splits them. After
  // each change it calls every local improver, in the given order, on the
  // triangles it touched, and adds their returned counts to its report.
  // The Mesher (or Remesher) passes the local improvers it resolved when
  // it was built; they stay alive for the whole run.
  virtual void run(WorkingMesh &mesh, const MeshSpec &spec,
                   std::span<const LocalImprover *const> local_improvers,
                   StageReport &report) const = 0;
};

// A pass over the whole mesh. An improver that can also work locally
// implements LocalImprover as well and returns itself from
// as_local_improver().
class Improver {
public:
  virtual ~Improver() = default;
  virtual std::string name() const = 0;
  virtual ImproverEffects effects() const = 0;
  // The use mode from the improver's options.
  virtual ImproverUse use() const = 0;
  // Its local version, or null if it has none.
  virtual const LocalImprover *as_local_improver() const { return nullptr; }
  virtual void run(WorkingMesh &mesh, const MeshSpec &spec,
                   StageReport &report) const = 0;
};

// How a Remesher (and so a Mesher) uses its improvers, decided once when
// it is constructed.
struct PipelinePlan {
  // Indices into the improver list: the improvers the refiner calls after
  // each change, in this order.
  std::vector<std::size_t> local_improvers;
  // Indices into the improver list: the whole-mesh passes after the
  // refiner, in this order.
  std::vector<std::size_t> final_passes;
  // Modes that could not be honoured, and passes that change nothing.
  std::vector<std::string> warnings;
};

// Resolves the improvers' use modes against the refiner (which may be
// null). An improver runs locally if its mode allows local use, it has a
// local version, and the refiner tolerates its effects (no flips for a
// refiner that does not tolerate flips, no node moves likewise, and no
// Delaunay maker for a refiner that already maintains Delaunay). It runs
// as a final pass if its mode allows final use. Final passes keep the list
// order, and so do local improvers.
//
// A mode that cannot be honoured is a warning, not an error, for example
// "delaunay_flips: ruppert already maintains Delaunay, so it runs only as
// a final pass". So is a final pass that would make the mesh Delaunay
// while it already is (after a refiner that maintains Delaunay, or after
// another such pass with no flip or move in between); it still runs.
// Throws std::invalid_argument only for a null improver.
PipelinePlan
plan_pipeline(const Refiner *refiner,
              const std::vector<std::shared_ptr<const Improver>> &improvers);

} // namespace pde2d::mesh
