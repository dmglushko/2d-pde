// transfer_field: P1 interpolation of a nodal field onto another mesh.
// Oracles: affine fields are reproduced exactly (to rounding) on refined and
// unrelated meshes; nodes shared by both meshes keep their values; the
// documented outside rule on a curved domain.
#include "discretization/geometry.hpp"
#include "discretization/square_mesh.hpp"
#include "discretization/triangulate.hpp"
#include "fem/interpolation.hpp"
#include "fem/transfer.hpp"
#include "support/check.hpp"
#include <cmath>
#include <iostream>
using namespace pde2d::fem;
using namespace pde2d::discretization;
using pde2d_test::check;
using pde2d_test::expect_throws;

namespace {
const Expr affine = 1 + 2 * x - 3 * y;
double max_difference(const Vector &a, const Vector &b) {
  return (a - b).lpNorm<Eigen::Infinity>();
}

void affine_onto_refined_and_unrelated_meshes() {
  RectangleGeometry g(1, 1);
  g.setCenter(.5, .5);
  MeshOptions o;
  o.boundary_treatment = BoundaryTreatment::SplitPolygonEdges;
  o.refinement.max_edge_length = .2;
  auto coarse = generate_mesh(g, o);
  auto source = interpolate(coarse.mesh, affine);
  // Refined mesh of the same domain (keeps the coarse nodes).
  o.refinement.max_edge_length = .07;
  auto fine = refine_mesh(coarse, o);
  auto onto_fine = transfer_field(coarse.mesh, source, fine.mesh);
  check(onto_fine.extrapolated_nodes == 0);
  check(max_difference(onto_fine.values, interpolate(fine.mesh, affine)) <
        1e-13);
  // Coarse nodes are nodes of the refined mesh too: same values.
  for (std::size_t i = 0; i < coarse.mesh.nodes.size(); ++i)
    for (std::size_t j = 0; j < fine.mesh.nodes.size(); ++j)
      if (fine.mesh.nodes[j].x == coarse.mesh.nodes[i].x &&
          fine.mesh.nodes[j].y == coarse.mesh.nodes[i].y)
        check(std::abs(onto_fine.values[j] - source[i]) < 1e-14);
  // Unrelated structured mesh, and back.
  auto square = square_mesh(7);
  auto onto_square = transfer_field(coarse.mesh, source, square);
  check(max_difference(onto_square.values, interpolate(square, affine)) <
        1e-13);
  auto back = transfer_field(square, onto_square.values, coarse.mesh);
  check(max_difference(back.values, source) < 1e-13);
  // A nonlinear field is interpolated, not reproduced: error O(h^2).
  const Expr smooth = sin(3 * x) * cos(2 * y);
  auto onto = transfer_field(square_mesh(32),
                             interpolate(square_mesh(32), smooth), square);
  auto error = max_difference(onto.values, interpolate(square, smooth));
  check(error < 1e-2, "interpolation error too large");
}

void curved_domain_nodes_outside_source() {
  MeshOptions o;
  o.boundary_treatment = BoundaryTreatment::RefineOnCurves;
  o.boundary.max_curve_deviation = .02;
  o.refinement.max_edge_length = .3;
  auto coarse = generate_mesh(CircleGeometry(1.0), o);
  o.refinement.max_edge_length = .1;
  auto fine = refine_mesh(coarse, o);
  auto source = interpolate(coarse.mesh, affine);
  // New boundary nodes lie on the circle, outside the coarse chords.
  expect_throws<std::invalid_argument>(
      [&] { transfer_field(coarse.mesh, source, fine.mesh); },
      "lies outside the source mesh beyond the allowed distance");
  TransferOptions options;
  options.max_outside_distance = o.boundary.max_curve_deviation;
  auto moved = transfer_field(coarse.mesh, source, fine.mesh, options);
  std::cout << "extrapolated " << moved.extrapolated_nodes << " nodes, max "
            << moved.max_outside_distance << '\n';
  check(moved.extrapolated_nodes > 0);
  check(moved.max_outside_distance > 0 &&
        moved.max_outside_distance <= options.max_outside_distance);
  // Linear extrapolation keeps affine fields exact outside as well.
  check(max_difference(moved.values, interpolate(fine.mesh, affine)) < 1e-13);
}

void validation_and_execution() {
  auto mesh = square_mesh(4);
  expect_throws<std::invalid_argument>(
      [&] { transfer_field(mesh, Vector::Zero(3), mesh); },
      "one finite entry per source node");
  Vector bad = Vector::Zero(static_cast<Eigen::Index>(mesh.nodes.size()));
  bad[0] = std::nan("");
  expect_throws<std::invalid_argument>([&] { transfer_field(mesh, bad, mesh); },
                                       "finite entry");
  if (!pde2d::openmp_available())
    return;
  auto source = square_mesh(20);
  auto values = interpolate(source, sin(5 * x * y));
  TransferOptions parallel;
  parallel.execution = {pde2d::ExecutionBackend::OpenMP, 4, 1};
  auto target = square_mesh(13);
  auto a = transfer_field(source, values, target);
  auto b = transfer_field(source, values, target, parallel);
  check(a.values == b.values, "serial and OpenMP transfers differ");
}
} // namespace

int main() {
  return pde2d_test::run(
      {{"affine_onto_refined_and_unrelated_meshes",
        affine_onto_refined_and_unrelated_meshes},
       {"curved_domain_nodes_outside_source",
        curved_domain_nodes_outside_source},
       {"validation_and_execution", validation_and_execution}});
}
