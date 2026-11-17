#pragma once

#include "pde2d/mesh/components.hpp"

namespace pde2d::mesh {

struct LaplacianSmoothingOptions {
  // Number of passes over all interior nodes. Must be at least 1.
  int iterations = 3;
  // Smoothing has no local version, so a final pass is the only use that
  // can be honoured.
  ImproverUse use = ImproverUse::final_only;
};

// Moves each interior node to the average of its neighbours, one node at a
// time in index order (Gauss-Seidel). A move that would invert a triangle
// is skipped. Boundary nodes never move. It has only the final role (no
// local version). Metrics: "moves", "rejected_moves".
class LaplacianSmoothing final : public Improver, public FinalImprover {
public:
  explicit LaplacianSmoothing(LaplacianSmoothingOptions options = {});
  const LaplacianSmoothingOptions &options() const { return options_; }

  std::string name() const override { return "laplacian_smoothing"; }
  ImproverEffects effects() const override {
    return {.flips_edges = false,
            .moves_nodes = true,
            .makes_delaunay = false,
            .keeps_parents = false};
  }
  ImproverUse use() const override { return options_.use; }
  const FinalImprover *as_final() const override { return this; }
  void run(WorkingMesh &mesh, const MeshSpec *spec,
           StageReport &report) const override;

private:
  LaplacianSmoothingOptions options_;
};

} // namespace pde2d::mesh
