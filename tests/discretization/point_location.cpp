// PointLocator: containing element and barycentric coordinates. Oracles: a
// brute-force scan over all triangles with exact predicates (the box index is
// what is under test), reconstruction p = sum lambda_i x_i, and the
// documented outside rule.
#include "discretization/geometry.hpp"
#include "discretization/point_location.hpp"
#include "discretization/predicates.hpp"
#include "discretization/square_mesh.hpp"
#include "discretization/triangulate.hpp"
#include "support/check.hpp"
#include <cmath>
#include <cstdint>
#include <optional>
using namespace pde2d::discretization;
using pde2d_test::check;
using pde2d_test::expect_throws;
using pde2d_test::near_abs;

namespace {
std::optional<std::size_t> brute_force(const TriangleMesh &m, Point2 p) {
  for (std::size_t i = 0; i < m.elements.size(); ++i) {
    auto t = m.elements[i];
    auto a = m.nodes[t[0]], b = m.nodes[t[1]], c = m.nodes[t[2]];
    if (predicates::orient(a, b, p) >= 0 && predicates::orient(b, c, p) >= 0 &&
        predicates::orient(c, a, p) >= 0)
      return i;
  }
  return std::nullopt;
}
void check_reconstructs(const TriangleMesh &m, const PointLocation &loc,
                        Point2 p) {
  const auto &t = m.elements[loc.element];
  double x = 0, y = 0, sum = 0;
  for (int j = 0; j < 3; ++j) {
    x += loc.barycentric[j] * m.nodes[t[j]].x;
    y += loc.barycentric[j] * m.nodes[t[j]].y;
    sum += loc.barycentric[j];
  }
  near_abs(x, p.x, 1e-14);
  near_abs(y, p.y, 1e-14);
  near_abs(sum, 1, 1e-14);
}

void matches_brute_force() {
  MeshOptions o;
  o.boundary_treatment = BoundaryTreatment::RefineOnCurves;
  o.refinement.max_edge_length = .15;
  auto mesh = generate_mesh(CircleGeometry(1.0), o).mesh;
  PointLocator locator(mesh);
  std::uint64_t state = 12345;
  auto next = [&] {
    state = state * 6364136223846793005ull + 1442695040888963407ull;
    return static_cast<double>(state >> 11) * 0x1.0p-53 * 2.4 - 1.2;
  };
  int inside = 0;
  for (int k = 0; k < 2000; ++k) {
    Point2 p{next(), next()};
    auto expected = brute_force(mesh, p);
    auto found = locator.locate(p);
    check(found.has_value() == expected.has_value(), "containment differs");
    if (!found)
      continue;
    ++inside;
    check(found->element == *expected, "lowest containing element expected");
    check(found->inside && found->distance == 0);
    for (double l : found->barycentric)
      check(l >= -1e-14 && l <= 1 + 1e-14);
    check_reconstructs(mesh, *found, p);
  }
  check(inside > 1000);
  // Mesh vertices and points on shared edges go to the lowest-index owner.
  for (std::size_t v = 0; v < mesh.nodes.size(); v += 7) {
    auto found = locator.locate(mesh.nodes[v]);
    check(found && found->element == *brute_force(mesh, mesh.nodes[v]));
  }
}

void outside_points() {
  auto mesh = square_mesh(4);
  // Default: only the closed mesh.
  check(locate_point(mesh, {1, .3}).has_value(), "boundary point is inside");
  check(!locate_point(mesh, {1.05, .3}).has_value());
  PointLocator near(mesh, {.max_outside_distance = .1});
  auto found = near.locate({1.05, .3});
  check(found && !found->inside);
  near_abs(found->distance, .05, 1e-15);
  // Extrapolated from the triangle owning the nearest boundary edge.
  const auto &t = mesh.elements[found->element];
  int on_right = 0;
  for (auto v : t)
    on_right += mesh.nodes[v].x == 1;
  check(on_right == 2, "owner of the nearest boundary edge expected");
  check_reconstructs(mesh, *found, {1.05, .3});
  // Beyond the distance: nothing.
  check(!near.locate({1.2, .3}).has_value());
  check(!near.locate({-.5, -.5}).has_value());
  // Corner region: nearest boundary edge is still within the distance.
  auto corner = near.locate({1.03, 1.04});
  check(corner && !corner->inside);
  near_abs(corner->distance, std::hypot(.03, .04), 1e-15);
  check_reconstructs(mesh, *corner, {1.03, 1.04});
  expect_throws<std::invalid_argument>(
      [&] { PointLocator(mesh, {.max_outside_distance = -1}); },
      "outside distance must be finite and nonnegative");
  expect_throws<std::invalid_argument>([&] { near.locate({std::nan(""), 0}); },
                                       "finite point");
  auto broken = mesh;
  broken.elements.pop_back();
  expect_throws<std::invalid_argument>([&] { PointLocator{broken}; });
}
} // namespace

int main() {
  return pde2d_test::run({{"matches_brute_force", matches_brute_force},
                          {"outside_points", outside_points}});
}
