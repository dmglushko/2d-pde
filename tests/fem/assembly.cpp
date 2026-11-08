#include "fem/assembly.hpp"
#include "discretization/square_mesh.hpp"
using pde2d::discretization::square_mesh;
#include "support/check.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <limits>
#include <stdexcept>

using pde2d_test::check;
using pde2d_test::expect_throws;
namespace {
using namespace pde2d::fem;
void near(double a, double b, double tolerance = 2e-11) {
  pde2d_test::near(a, b, tolerance);
}
pde2d::discretization::TriangleMesh triangle() {
  return {{{0, 0}, {1, 0}, {0, 1}},
          {{0, 1, 2}},
          {{1, 0}, {2, 1}, {0, 2}},
          {"bottom", "diagonal", "left"}};
}
Problem zero_flux() {
  Problem p;
  p.assign(flux_neumann(0), {"bottom", "diagonal", "left"});
  return p;
}
void element_matrices() {
  auto mesh = triangle();
  auto p = zero_flux();
  p.equation.a_xx = -2;
  p.equation.a_xy = -2;
  p.equation.a_yy = -3;
  p.equation.m1 = 2;
  p.equation.m2 = 3;
  p.equation.source = 6;
  const auto s = assemble(mesh, p);
  // D=[[2,1],[1,3]], area=1/2, grad(phi)=[(-1,-1),(1,0),(0,1)].
  const double expected[3][3] = {{3.5, -1.5, -2}, {-1.5, 1, .5}, {-2, .5, 1.5}};
  for (int i = 0; i < 3; ++i) {
    near(s.load[i], 1);
    for (int j = 0; j < 3; ++j) {
      near(s.stiffness.coeff(i, j), expected[i][j]);
      near(s.first_mass.coeff(i, j), 2. * (i == j ? 2 : 1) / 24);
      near(s.second_mass.coeff(i, j), 3. * (i == j ? 2 : 1) / 24);
    }
  }
  check(s.dirichlet.empty(), "Pure Neumann must remain unconstrained");
  near((s.stiffness * Vector::Ones(3)).norm(), 0);
  p.equation.c = 4;
  const auto reaction = assemble(mesh, p);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      near(reaction.stiffness.coeff(i, j) - s.stiffness.coeff(i, j),
           4. * (i == j ? 2 : 1) / 24);
}
void boundary_matrices() {
  auto mesh = triangle();
  Problem p;
  p.assign(dynamic_flux_robin(3, 2, 2, 8), {"bottom"});
  p.assign(flux_neumann(0), {"diagonal", "left"});
  auto s = assemble(mesh, p);
  auto base = assemble(mesh, zero_flux());
  for (int i = 0; i < 3; ++i) {
    near(s.load[i], i < 2 ? 2 : 0);
    for (int j = 0; j < 3; ++j) {
      const double edge_mass =
          (i < 2 && j < 2) ? (i == j ? 1. / 3 : 1. / 6) : 0;
      near(s.stiffness.coeff(i, j) - base.stiffness.coeff(i, j), edge_mass);
      near(s.first_mass.coeff(i, j), 1.5 * edge_mass);
    }
  }
  // For D=2I, du_dn=q_n/2. Compare both representations exactly.
  p.equation.a_xx = -2;
  p.equation.a_yy = -2;
  p.boundaries[0].condition = dynamic_robin(3, 2, 4, 8);
  auto normal = assemble(mesh, p);
  p.boundaries[0].condition = dynamic_flux_robin(3, 2, 2, 8);
  auto flux = assemble(mesh, p);
  near((normal.stiffness - flux.stiffness).norm(), 0);
  near((normal.first_mass - flux.first_mass).norm(), 0);
  near((normal.load - flux.load).norm(), 0);
  // A general derivative vector is accepted only with a symbolic identity.
  p.boundaries[0].condition.beta_flux = 0;
  p.boundaries[0].condition.beta_x = 4 * nx;
  p.boundaries[0].condition.beta_y = 4 * ny;
  auto general = assemble(mesh, p);
  near((general.stiffness - flux.stiffness).norm(), 0);
  near((general.first_mass - flux.first_mass).norm(), 0);
  near((general.load - flux.load).norm(), 0);
}
void affine_patch() {
  auto mesh = square_mesh(3);
  for (auto &edge : mesh.boundary_edges)
    std::swap(edge.first, edge.second);
  Problem p;
  auto &e = p.equation;
  e.a_xx = -(2 + x);
  e.a_xy = -(1 + x + y);
  e.a_yy = -(3 + y);
  e.b_x = 1 + y;
  e.b_y = 2 - x;
  e.c = 1 + x;
  const Expr u = 1 + 2 * x - 3 * y;
  e.source = 2 * e.b_x - 3 * e.b_y + e.c * u;
  const Expr q =
      -nx * (2 * e.a_xx - 3 * e.a_xy / 2) - ny * (e.a_xy - 3 * e.a_yy);
  p.assign(flux_neumann(q), {"bottom", "left"});
  p.assign(flux_robin(2, 3, 2 * u + 3 * q), {"top", "right"});
  auto s = assemble(mesh, p, 0, AssemblyOptions{6});
  Vector exact(mesh.nodes.size());
  for (std::size_t i = 0; i < mesh.nodes.size(); ++i)
    exact[i] = 1 + 2 * mesh.nodes[i].x - 3 * mesh.nodes[i].y;
  near((s.stiffness * exact - s.load).norm(), 0);
  // Manufactured time-dependent affine solution, including dynamic boundary.
  e.m1 = 1 + x;
  e.m2 = 2 - y;
  const Expr ut = u * (1 + t + t * t), dt = u * (1 + 2 * t), dtt = 2 * u;
  e.source = (1 + t + t * t) * (2 * e.b_x - 3 * e.b_y + e.c * u) + e.m1 * dt +
             e.m2 * dtt;
  p.boundaries.clear();
  p.assign(
      dynamic_flux_robin(2, 3, 4, 2 * dt + 3 * ut + 4 * (1 + t + t * t) * q),
      {"bottom", "top", "left", "right"});
  const double time = .4;
  s = assemble(mesh, p, time, AssemblyOptions{6});
  near((s.stiffness * (exact * (1 + time + time * time)) +
        s.first_mass * (exact * (1 + 2 * time)) + s.second_mass * (2 * exact) -
        s.load)
           .norm(),
       0);
}
void dirichlet_and_errors() {
  auto mesh = triangle();
  auto p = zero_flux();
  p.boundaries.clear();
  p.assign(dirichlet(1 + x + 2 * y + t), {"bottom", "diagonal", "left"});
  auto s = assemble(mesh, p, .5);
  check(s.dirichlet.size() == 3, "Missing strong Dirichlet nodes");
  for (auto d : s.dirichlet)
    near(d.value, 1.5 + mesh.nodes[d.node].x + 2 * mesh.nodes[d.node].y);
  // Assembly itself never performs row elimination.
  near(s.stiffness.coeff(0, 1), -.5);
  p.boundaries.clear();
  p.assign(dirichlet(0), {"bottom"});
  p.assign(dirichlet(1), {"left"});
  p.assign(flux_neumann(0), {"diagonal"});
  expect_throws([&] { assemble(mesh, p); }, "Contradictory");
  p = zero_flux();
  p.boundaries[0].regions = {"bottom", "left"};
  expect_throws([&] { assemble(mesh, p); }, "Missing");
  p = zero_flux();
  p.assign(flux_neumann(0), {"bottom"});
  expect_throws([&] { assemble(mesh, p); }, "Overlapping");
  p = zero_flux();
  p.assign(flux_neumann(0), {"absent"});
  expect_throws([&] { assemble(mesh, p); }, "Absent");
  p = zero_flux();
  auto no_tags = mesh;
  no_tags.boundary_tags.clear();
  expect_throws([&] { assemble(no_tags, p); }, "tag");
  auto bad = mesh;
  bad.elements[0] = {0, 2, 1};
  expect_throws([&] { assemble(bad, p); }, "positive area");
  bad = mesh;
  bad.boundary_edges[0] = {0, 2};
  expect_throws([&] { assemble(bad, p); });
  p = zero_flux();
  p.boundaries[0].condition = neumann(0);
  p.equation.a_xx = -2;
  p.equation.a_yy = -3;
  expect_throws([&] { assemble(mesh, p); }, "not algebraically proportional");
  // Off-diagonal diffusion makes even an axis-aligned normal derivative oblique
  // relative to flux. This must fail before trying to integrate that boundary.
  p = zero_flux();
  p.equation.a_xy = -1;
  p.boundaries[0].regions = {"diagonal", "left"};
  p.assign(neumann(0), {"bottom"});
  expect_throws([&] { assemble(mesh, p); }, "not algebraically proportional");
  // General oblique derivative not aligned with flux must not be projected.
  p = zero_flux();
  p.boundaries[0].condition = {};
  p.boundaries[0].condition.beta_x = 1;
  expect_throws([&] { assemble(mesh, p); }, "Unsupported oblique");
  p = zero_flux();
  p.boundaries[0].condition = {};
  p.boundaries[0].condition.gamma = 1;
  expect_throws([&] { assemble(mesh, p); }, "Derivative-free dynamic");
  p = zero_flux();
  p.boundaries[0].condition = {};
  expect_throws([&] { assemble(mesh, p); }, "no nonzero operator");
  p = zero_flux();
  p.equation.source = log(Expr(-1));
  expect_throws([&] { assemble(mesh, p); });
  p = zero_flux();
  p.equation.source = nx;
  expect_throws([&] { assemble(mesh, p); }, "boundary normals");
  p = zero_flux();
  p.boundaries[0].condition.value = log(x);
  expect_throws([&] { assemble(mesh, p); });
  p = zero_flux();
  expect_throws(
      [&] { assemble(mesh, p, std::numeric_limits<double>::infinity()); },
      "Invalid FEM assembly time");
  expect_throws(
      [&] {
        PreparedAssembly(mesh, p).assemble(
            std::numeric_limits<double>::quiet_NaN(), 1);
      },
      "Invalid FEM assembly time");
  expect_throws([&] { assemble(mesh, p, 0, AssemblyOptions{2, -1}); },
                "Invalid FEM time or boundary tolerance");
  expect_throws([&] { assemble(mesh, p, 0, AssemblyOptions{-1}); });
}
void boundary_restriction_regressions() {
  auto mesh = triangle();
  auto p = zero_flux();
  p.boundaries[0].regions = {"diagonal", "left"};
  p.assign(flux_robin(1, 2 * x - 1, 1), {"bottom"});
  expect_throws([&] { assemble(mesh, p); }, "vanishes or changes sign");
  p.boundaries.back().condition = flux_robin(1, x - .3, 1);
  expect_throws([&] { assemble(mesh, p); }, "vanishes or changes sign");
  // This cross coefficient vanishes on the bottom edge, so du_dn=q_n there.
  p.equation.a_xy = y;
  p.boundaries.back().condition = neumann(1);
  const auto normal = assemble(mesh, p);
  p.boundaries.back().condition = flux_neumann(1);
  const auto natural = assemble(mesh, p);
  near((normal.load - natural.load).norm(), 0);
  // The x flux component vanishes on the edge. Select the regular y pivot.
  p.equation.a_xy = 2 * y;
  p.equation.a_yy = -2;
  BoundaryCondition general;
  general.beta_x = x * y + y;
  general.beta_y = -2 * x - 2;
  general.value = 1;
  p.boundaries.back().condition = general;
  const auto converted = assemble(mesh, p);
  p.boundaries.back().condition = flux_robin(0, x + 1, 1);
  const auto reference = assemble(mesh, p);
  near((converted.load - reference.load).norm(), 0);
  p.equation.a_xy = 2 * x;
  general.beta_x = x * (x + 1);
  general.beta_y = -2 * (x + 1);
  p.boundaries.back().condition = general;
  const auto crossing_pivot = assemble(mesh, p);
  p.boundaries.back().condition = flux_robin(0, x + 1, 1);
  const auto crossing_reference = assemble(mesh, p);
  near((crossing_pivot.load - crossing_reference.load).norm(), 0);
}
void slanted_normal_boundaries() {
  // Unequal normal components exposed floating-point cancellation in the
  // symbolic proportionality test, even for constant isotropic diffusion.
  pde2d::discretization::TriangleMesh mesh{{{0, 0}, {1, 0}, {.3, .7}},
                                           {{0, 1, 2}},
                                           {{0, 1}, {1, 2}, {2, 0}},
                                           {"b", "b", "b"}};
  for (const Expr scale : {Expr(10), Expr(.1), Expr(10) * (1 + x + y + t)}) {
    for (int kind = 0; kind < 3; ++kind) {
      Problem p;
      p.equation.a_xx = p.equation.a_yy = -scale;
      BoundaryCondition bc = kind == 0   ? neumann(2)
                             : kind == 1 ? robin(3, 4, 5)
                                         : dynamic_robin(2, 3, 4, 5);
      p.assign(bc, {"b"});
      const auto normal = assemble(mesh, p, .4);
      // For -scale*Laplacian, q_n=scale*du/dn on every edge.
      bc.beta_flux = bc.beta_normal / scale;
      bc.beta_normal = 0;
      p.boundaries[0].condition = bc;
      const auto flux = assemble(mesh, p, .4);
      near((normal.stiffness - flux.stiffness).norm(), 0);
      near((normal.first_mass - flux.first_mass).norm(), 0);
      near((normal.load - flux.load).norm(), 0);
    }
  }
  // No numeric tolerance should admit an actually oblique condition, even
  // with a tiny tangential component or only on the interior of an edge.
  Problem p;
  p.equation.a_xx = p.equation.a_yy = -10;
  auto bc = neumann(0);
  bc.beta_x = 1e-14;
  p.assign(bc, {"b"});
  expect_throws([&] { assemble(mesh, p); }, "not algebraically proportional");
  p.boundaries[0].condition = neumann(0);
  p.equation.a_yy = -11;
  expect_throws([&] { assemble(mesh, p); }, "not algebraically proportional");
}
} // namespace
int main() {
  return pde2d_test::run(
      {{"element_matrices", element_matrices},
       {"boundary_matrices", boundary_matrices},
       {"affine_patch", affine_patch},
       {"dirichlet_and_errors", dirichlet_and_errors},
       {"boundary_restriction_regressions", boundary_restriction_regressions},
       {"slanted_normal_boundaries", slanted_normal_boundaries}});
}
