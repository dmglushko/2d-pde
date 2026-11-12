#include "pde2d/mesh/domain_size.hpp"

#include "domain_size_policy.hpp"
#include "pde2d/geometry/bounding_box.hpp"

#include <cstddef>
#include <stdexcept>
#include <vector>

namespace pde2d::mesh {
namespace detail {
namespace {

class BoundingBoxDiagonal final : public DomainSizePolicy {
public:
  double of_curves(const std::vector<geometry::Curve> &curves) const override {
    return geometry::diagonal(geometry::bounding_box(curves));
  }
  double of_points(const std::vector<geometry::Point> &points) const override {
    return geometry::diagonal(geometry::bounding_box(points));
  }
};

} // namespace

const DomainSizePolicy &domain_size_policy() {
  static const BoundingBoxDiagonal policy;
  return policy;
}

} // namespace detail

namespace {

double of_chains(const std::vector<CurveChain> &chains) {
  std::vector<geometry::Curve> curves;
  for (const auto &chain : chains)
    curves.insert(curves.end(), chain.curves().begin(), chain.curves().end());
  return detail::domain_size_policy().of_curves(curves);
}

template <typename Edges>
double of_boundary_nodes(const std::vector<geometry::Point> &nodes,
                         const Edges &edges) {
  if (edges.empty())
    throw std::invalid_argument("domain_size: the mesh has no boundary edges");
  std::vector<geometry::Point> points;
  points.reserve(2 * edges.size());
  for (const auto &e : edges)
    for (std::size_t v : e.nodes)
      points.push_back(nodes.at(v));
  return detail::domain_size_policy().of_points(points);
}

} // namespace

double domain_size(const geometry::CurveGeometry &geometry) {
  std::vector<geometry::Curve> curves;
  for (const auto &loop : geometry.loops())
    for (const auto &c : loop.curves)
      curves.push_back(c.curve);
  return detail::domain_size_policy().of_curves(curves);
}

double domain_size(const WorkingMesh &mesh) {
  if (!mesh.chains().empty())
    return of_chains(mesh.chains());
  return of_boundary_nodes(mesh.nodes(), mesh.boundary_edges());
}

double domain_size(const Mesh &mesh) {
  if (!mesh.chains.empty())
    return of_chains(mesh.chains);
  return of_boundary_nodes(mesh.nodes, mesh.boundary_edges);
}

} // namespace pde2d::mesh
