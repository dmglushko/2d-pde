#pragma once
// Transfer of a P1 nodal field between meshes of the same domain: the value
// at each target node is the source P1 interpolant there (point location
// plus barycentric interpolation), so affine fields transfer exactly up to
// rounding. Target nodes outside the source mesh follow the rule of
// discretization/point_location.hpp: within max_outside_distance they use
// linear extrapolation of the triangle owning the nearest source boundary
// edge; farther nodes are an error. This is interpolation, not an L2
// projection: it does not conserve integrals.
#include "discretization/point_location.hpp"
#include "execution.hpp"
#include "fem/system.hpp"
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace pde2d::fem {
struct TransferOptions {
  // Largest accepted distance of a target node outside the source mesh. For
  // curved domains a bound on the source chord deviation (for example
  // BoundaryOptions::max_curve_deviation) is a natural choice.
  double max_outside_distance = 0.0;
  pde2d::ExecutionOptions execution{};
};
struct FieldTransfer {
  Vector values;
  // Target nodes outside the source mesh, and the largest such distance.
  std::size_t extrapolated_nodes = 0;
  double max_outside_distance = 0.0;
};

inline FieldTransfer
transfer_field(const discretization::PointLocator &source, const Vector &values,
               const discretization::TriangleMesh &target,
               const pde2d::ExecutionOptions &execution = {}) {
  const auto &mesh = source.mesh();
  if (values.size() != static_cast<Eigen::Index>(mesh.nodes.size()) ||
      !values.allFinite())
    throw std::invalid_argument(
        "Transferred values must have one finite entry per source node");
  std::vector<std::optional<discretization::PointLocation>> found(
      target.nodes.size());
  pde2d::parallel_for(target.nodes.size(), execution, [&](std::size_t i) {
    found[i] = source.locate(target.nodes[i]);
  });
  FieldTransfer result;
  result.values.resize(static_cast<Eigen::Index>(target.nodes.size()));
  for (std::size_t i = 0; i < found.size(); ++i) {
    if (!found[i])
      throw std::invalid_argument(
          "Transfer target node " + std::to_string(i) +
          " lies outside the source mesh beyond the allowed distance");
    const auto &loc = *found[i];
    const auto &t = mesh.elements[loc.element];
    double v = 0;
    for (int j = 0; j < 3; ++j)
      v += loc.barycentric[j] * values[static_cast<Eigen::Index>(t[j])];
    result.values[static_cast<Eigen::Index>(i)] = v;
    if (!loc.inside) {
      ++result.extrapolated_nodes;
      result.max_outside_distance =
          std::max(result.max_outside_distance, loc.distance);
    }
  }
  return result;
}
inline FieldTransfer transfer_field(const discretization::TriangleMesh &source,
                                    const Vector &values,
                                    const discretization::TriangleMesh &target,
                                    const TransferOptions &options = {}) {
  return transfer_field(
      discretization::PointLocator(
          source, {.max_outside_distance = options.max_outside_distance}),
      values, target, options.execution);
}
} // namespace pde2d::fem
