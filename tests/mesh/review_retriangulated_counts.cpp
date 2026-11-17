// From the review of the shared refiners: closure must count an unmarked
// triangle that the boundary policy re-triangulates, even when parents have
// been dropped.
#include "support/check.hpp"
#include "support/mesh_checks.hpp"

#include "pde2d/mesh/newest_vertex_bisection.hpp"
#include "pde2d/mesh/remesher.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <numbers>
#include <vector>

using namespace pde2d::mesh;
using namespace pde2d::test;

namespace {
void retriangulation_counts_unmarked_neighbour() {
  // The same ordinary curved-boundary fixture as refinement_curved.cpp:
  // the curve point (1, .8) lies across the marked triangle's interior
  // edge, so the unmarked neighbour must also be re-triangulated.
  Mesh m;
  m.nodes = {{0, 0}, {2, 0}, {2.2, 0.4}, {1, 2}};
  m.triangles = {{{2, 0, 1}}, {{0, 2, 3}}};
  m.region_names = {"r"};
  const double k = 0.225, radius = std::hypot(1.0, k);
  const double pi = std::numbers::pi;
  m.chains = {CurveChain(std::vector<pde2d::geometry::Curve>{
      pde2d::geometry::EllipticalArc{{1, -k},
                                     radius,
                                     radius,
                                     0,
                                     pi - std::atan(k),
                                     -(pi - 2 * std::atan(k))}})};
  m.boundary_edges = {{{0, 1}, 0, CurveLink{0, 0, m.chains[0].length()}},
                      {{1, 2}, 0, std::nullopt},
                      {{2, 3}, 0, std::nullopt},
                      {{3, 0}, 0, std::nullopt}};
  check_valid_mesh(m);
  const auto result = Remesher(std::make_shared<NewestVertexBisection>())
                          .refine(m, std::vector<std::size_t>{0});
  check_valid_mesh(result.mesh);
  check(!result.parents, "re-triangulation drops parents");
  auto old_neighbour = m.triangles[1].nodes;
  std::sort(old_neighbour.begin(), old_neighbour.end());
  for (auto triangle : result.mesh.triangles) {
    std::sort(triangle.nodes.begin(), triangle.nodes.end());
    check(triangle.nodes != old_neighbour, "unmarked neighbour was replaced");
  }
  const double closure = result.reports[1].metrics.at("refined_by_closure");
  std::cout << "INFO refined_by_closure " << closure
            << ", one unmarked input triangle was replaced\n";
  near(closure, 1, 0, "closure counts the re-triangulated unmarked neighbour");
}
} // namespace

int main() {
  return run({{"retriangulation_counts_unmarked_neighbour",
               retriangulation_counts_unmarked_neighbour}});
}
