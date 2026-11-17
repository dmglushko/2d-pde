#pragma once

#include "pde2d/mesh/components.hpp"

#include <cstddef>

namespace pde2d::mesh {

struct RuppertOptions {
  // Smallest angle the refined triangles may have, in degrees, in (0, 34].
  // Termination is proven up to about 20.7 degrees for inputs without
  // angles below 60 degrees, and usual in practice up to about 33.
  double min_angle_degrees = 20.0;
  // Refinement stops with an exception after inserting this many points.
  std::size_t max_points = 10'000'000;
};

// Ruppert's Delaunay refinement. First makes its input (constrained)
// Delaunay by flips, then repeats:
//   - split a boundary edge that is encroached (a node lies inside its
//     diametral circle) or longer than max_edge_length;
//   - for a triangle with an angle below the bound or an edge longer than
//     max_edge_length, insert its circumcenter, unless that point would
//     encroach boundary edges or lies beyond one, in which case those
//     boundary edges are split instead.
// Every insertion is followed by flips, so the mesh stays constrained
// Delaunay. A boundary edge with a curve link is split at the curve point.
// Local improvers it is given run after every insertion and its flips.
// max_edge_length is resolved against domain_size(mesh).
//
// refine() (for adaptive refinement) inserts the circumcenter of every
// marked triangle (as it was when refine() was called), with the same
// rules: the input is first made Delaunay; a circumcenter that would
// encroach boundary edges, or lies beyond one, is not inserted and those
// boundary edges are split instead; after each insertion the boundary
// edges a new point encroaches are split; flips keep the mesh Delaunay.
// There is no angle or length goal here, so nothing else is refined. It
// never keeps the parents (it flips).
//
// Metrics: "initial_flips", "flips", "boundary_splits", "circumcenters",
// "retriangulated_splits", "deleted_nodes", "unsplittable_edges",
// "local_improver_changes" and "local_improver_changes.<name>"; run()
// adds "domain_size", "max_edge_length" (resolved) and
// "abandoned_triangles", refine() adds "marked", "skipped_circumcenters"
// (one already a node, for example the shared circumcenter of two marked
// triangles of a rectangle, or cut off with its triangle) and
// "blocked_circumcenters" (needing an unsplittable boundary edge).
class Ruppert final : public Refiner {
public:
  explicit Ruppert(RuppertOptions options = {});
  const RuppertOptions &options() const { return options_; }

  std::string name() const override { return "ruppert"; }
  RefinerCapabilities capabilities() const override {
    // It keeps its own Delaunay invariant, so it accepts no outside
    // changes during refinement: only local improvers that neither flip
    // nor move nor make the mesh Delaunay.
    // It flips edges, so it never keeps parents.
    return {.tolerates_flips = false,
            .tolerates_node_moves = false,
            .maintains_delaunay = true,
            .keeps_parents = false};
  }
  void run(WorkingMesh &mesh, const MeshSpec &spec,
           std::span<const LocalImprover *const> local_improvers,
           StageReport &report) const override;
  std::optional<Parents>
  refine(WorkingMesh &mesh, std::span<const std::size_t> marked_triangles,
         std::span<const LocalImprover *const> local_improvers,
         StageReport &report) const override;

private:
  RuppertOptions options_;
};

} // namespace pde2d::mesh
