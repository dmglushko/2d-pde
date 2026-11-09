// The shortest complete FEM program: geometry -> mesh -> Problem ->
// solve_stationary / evolve, checked against an exact solution. A CTest smoke
// test (example_fem_minimal).
#include "discretization/geometry.hpp"
#include "discretization/triangulate.hpp"
#include "fem/error_norms.hpp"
#include "fem/evolution.hpp"
#include "fem/solver.hpp"
#include <iostream>

int main() {
  namespace disc = pde2d::discretization;
  using namespace pde2d::fem;
  disc::MeshOptions mesh_options;
  mesh_options.boundary.max_edge_length = 0.1; // boundary and interior
  mesh_options.refinement.max_edge_length = 0.1;
  const auto mesh =
      disc::generate_mesh(disc::RectangleGeometry(2.0, 1.0), mesh_options);
  if (!mesh.report.size_satisfied) { // meshing may miss its size target
    std::cerr << mesh.report.message << '\n';
    return 1;
  }

  // -Laplace(u) = -4 with u = x^2 + y^2 on the whole boundary.
  const Expr exact = x * x + y * y;
  Problem problem;
  problem.equation.source = -4;
  problem.assign(dirichlet(exact), {"bottom", "right", "top", "left"});
  const auto solution = solve_stationary(mesh.mesh, problem);
  std::cout << "stationary: nodes " << mesh.mesh.nodes.size() << ", L2 error "
            << error_norms(mesh.mesh, solution.values, exact).l2 << '\n';

  // u_t - Laplace(u) = -4 from the same field: it stays a steady state.
  Problem heat = problem;
  heat.equation.m1 = 1;
  TimeOptions time;
  time.method = TimeMethod::CrankNicolson;
  time.end = 0.1;
  time.step = 0.01;
  const auto evolution =
      evolve(mesh.mesh, heat, InitialConditions{exact}, time);
  const double error =
      error_norms(mesh.mesh, evolution.frames.back().values, exact, time.end)
          .l2;
  std::cout << "evolution: " << evolution.frames.size() << " frames, L2 error "
            << error << '\n';
  return error < 1e-2 ? 0 : 1;
}
