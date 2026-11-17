#include "pde2d/mesh/laplacian_smoothing.hpp"

#include <algorithm>
#include <stdexcept>
#include <vector>

namespace pde2d::mesh {

LaplacianSmoothing::LaplacianSmoothing(LaplacianSmoothingOptions options)
    : options_(options) {
  if (options_.iterations < 1)
    throw std::invalid_argument(
        "LaplacianSmoothingOptions::iterations must be at least 1");
}

void LaplacianSmoothing::run(WorkingMesh &mesh, const MeshSpec *,
                             StageReport &report) const {
  std::size_t moves = 0, rejected = 0;
  for (int pass = 0; pass < options_.iterations; ++pass)
    for (std::size_t node = 0; node < mesh.nodes().size(); ++node) {
      if (mesh.is_node_deleted(node) || mesh.is_boundary_node(node))
        continue;
      std::vector<std::size_t> neighbours;
      for (auto t : mesh.triangles_around(node))
        for (auto v : mesh.triangles()[t])
          if (v != node)
            neighbours.push_back(v);
      // Sorted so the floating-point sum is the same however the
      // triangles are stored.
      std::sort(neighbours.begin(), neighbours.end());
      neighbours.erase(std::unique(neighbours.begin(), neighbours.end()),
                       neighbours.end());
      geometry::Point sum{};
      for (auto v : neighbours)
        sum = sum + mesh.nodes()[v];
      auto target = (1.0 / static_cast<double>(neighbours.size())) * sum;
      if (mesh.try_move_node(node, target))
        ++moves;
      else
        ++rejected;
    }
  report.metrics["moves"] = static_cast<double>(moves);
  report.metrics["rejected_moves"] = static_cast<double>(rejected);
}

} // namespace pde2d::mesh
