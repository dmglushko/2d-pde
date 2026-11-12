#pragma once

#include "pde2d/mesh/components.hpp"

namespace pde2d::mesh {

struct DelaunayFlipsOptions {
  // Local inside a refiner that accepts it, and a final pass.
  ImproverUse use = ImproverUse::local_and_final;
};

// Flips edges until every interior edge is locally Delaunay (Lawson's
// algorithm, flip_to_delaunay). As an Improver it runs on the whole mesh;
// as a LocalImprover it starts from the touched triangles. Boundary edges
// are never flipped. Metric: "flips".
class DelaunayFlips final : public Improver, public LocalImprover {
public:
  explicit DelaunayFlips(DelaunayFlipsOptions options = {})
      : options_(options) {}
  const DelaunayFlipsOptions &options() const { return options_; }

  std::string name() const override { return "delaunay_flips"; }
  ImproverEffects effects() const override {
    return {.flips_edges = true, .moves_nodes = false, .makes_delaunay = true};
  }
  ImproverUse use() const override { return options_.use; }
  const LocalImprover *as_local_improver() const override { return this; }
  void run(WorkingMesh &mesh, const MeshSpec &spec,
           StageReport &report) const override;
  std::size_t
  improve_around(WorkingMesh &mesh,
                 std::span<const std::size_t> touched_triangles) const override;

private:
  DelaunayFlipsOptions options_;
};

} // namespace pde2d::mesh
