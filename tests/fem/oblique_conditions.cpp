// General oblique-derivative boundary conditions (ObliqueEdge,
// detail/edge_forms/oblique.hpp) through the public API. Oracles:
// manufactured solutions with an oblique condition whose direction is not
// proportional to the natural flux (anisotropic, variable A and variable
// tangential coefficients) and their L2/H1 convergence rates on a square and
// on a disk; the tangential edge block computed by hand in world
// coordinates; time-dependent coefficients against the same problem with t
// substituted (no caching split); serial against OpenMP assembly; and the
// solver's own symmetry check.
#include "discretization/geometry.hpp"
#include "discretization/square_mesh.hpp"
#include "discretization/triangulate.hpp"
#include "fem/assembly.hpp"
#include "fem/error_norms.hpp"
#include "fem/solver.hpp"
#include "support/check.hpp"
#include "support/edge_forms.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using pde2d::discretization::square_mesh;
using pde2d_test::check;
using pde2d_test::expect_throws;
using pde2d_test::near;
namespace {
using namespace pde2d::fem;

// alpha u + beta_flux q_n + du/dn + r du/ds = value, with s = (-ny, nx) the
// counterclockwise tangent, written through beta_x and beta_y. The value is
// that of the exact solution, so the condition holds on every straight edge.
BoundaryCondition oblique(const Equation &eq, const Expr &exact, Expr alpha,
                          Expr beta_flux, const Expr &r) {
  BoundaryCondition bc;
  bc.alpha = alpha;
  bc.beta_flux = beta_flux;
  bc.beta_normal = 1;
  bc.beta_x = -r * ny;
  bc.beta_y = r * nx;
  const Expr ux = diff(exact, x), uy = diff(exact, y);
  const Expr q_n = -nx * (eq.a_xx * ux + eq.a_xy / 2 * uy) -
                   ny * (eq.a_xy / 2 * ux + eq.a_yy * uy);
  bc.value = alpha * exact + beta_flux * q_n + (nx + bc.beta_x) * ux +
             (ny + bc.beta_y) * uy;
  return bc;
}

// Anisotropic variable diffusion (negative definite A) with drift and
// reaction, and its manufactured source.
Equation anisotropic(const Expr &exact) {
  Equation eq;
  eq.a_xx = -(2 + x);
  eq.a_xy = -(.6 + .2 * y);
  eq.a_yy = -(1 + y * y);
  eq.b_x = .3;
  eq.c = 1;
  eq.source = eq.a_xx * diff(diff(exact, x), x) +
              eq.a_xy * diff(diff(exact, x), y) +
              eq.a_yy * diff(diff(exact, y), y) + eq.b_x * diff(exact, x) +
              eq.c * exact;
  return eq;
}

double rate(double coarse, double fine, double ratio) {
  return std::log(coarse / fine) / std::log(ratio);
}

void square_convergence() {
  // One oblique condition on three sides, so its direction varies smoothly
  // along the boundary and only turns with the normal at the corners. (Where
  // the obliqueness jumps at a corner the adjoint problem has a corner
  // singularity and the L2 rate drops below 2; H1 stays first order. See
  // detail/edge_forms/oblique.hpp.)
  const Expr exact = sin(1.3 * x + .4) * exp(.7 * y) + .5 * x * y;
  Problem p;
  p.equation = anisotropic(exact);
  const Expr r = .5 * (1 + x * y);
  p.assign(oblique(p.equation, exact, 1, .3, r), {"bottom", "right", "top"});
  p.assign(dirichlet(exact), {"left"});
  // On the coarsest mesh every edge of the oblique regions is oblique.
  const auto a = pde2d_test::region_forms(square_mesh(4), p);
  for (std::size_t k : {0, 1, 2})
    check(a[k].oblique == 4 && a[k].edges == 4,
          "Oblique regions of the manufactured problem");
  std::vector<ErrorNorms> errors;
  for (int n : {4, 8, 16, 32}) {
    const auto mesh = square_mesh(n);
    const auto s = solve_stationary(mesh, p, AssemblyOptions{6});
    check(s.report.converged, "Oblique stationary solve");
    errors.push_back(error_norms(mesh, s.values, exact));
    std::cout << "square n=" << n << " L2=" << errors.back().l2
              << " H1=" << errors.back().h1_seminorm << '\n';
  }
  for (std::size_t k = 1; k < errors.size(); ++k) {
    const double l2 = rate(errors[k - 1].l2, errors[k].l2, 2),
                 h1 = rate(errors[k - 1].h1_seminorm, errors[k].h1_seminorm, 2);
    std::cout << "square rates L2=" << l2 << " H1=" << h1 << '\n';
    check(l2 > 1.85, "Oblique square: second-order L2 convergence");
    check(h1 > .9, "Oblique square: first-order H1 convergence");
  }
}

void disk_convergence() {
  // Curved boundary: every polygon edge has its own normal and tangent, and
  // the whole boundary is oblique (with a Robin term and a reaction).
  const Expr exact = exp(.5 * x) * cos(y) + x * y;
  Problem p;
  p.equation = anisotropic(exact);
  p.assign(oblique(p.equation, exact, 1, 0, .4 * (1 + .5 * x)), {"boundary"});
  std::vector<double> sizes, l2;
  for (double h : {.4, .2, .1, .05}) {
    pde2d::discretization::MeshOptions options;
    options.boundary.max_edge_length = h;
    options.refinement.max_edge_length = h;
    const auto mesh = pde2d::discretization::generate_mesh(
        pde2d::discretization::CircleGeometry(1.0), options);
    check(mesh.report.size_satisfied, mesh.report.message);
    const auto s = solve_stationary(mesh.mesh, p, AssemblyOptions{6});
    check(s.report.converged, "Oblique disk solve");
    const auto e = error_norms(mesh.mesh, s.values, exact);
    std::cout << "disk h=" << h << " nodes=" << mesh.mesh.nodes.size()
              << " L2=" << e.l2 << " H1=" << e.h1_seminorm << '\n';
    sizes.push_back(h);
    l2.push_back(e.l2);
  }
  const double overall =
      rate(l2.front(), l2.back(), sizes.front() / sizes.back());
  std::cout << "disk L2 rate over " << sizes.size() << " meshes: " << overall
            << '\n';
  check(overall > 1.8, "Oblique disk: second-order L2 convergence");
}

// Bottom edge of the reference triangle, stored from node 1 = (1,0) to
// node 0 = (0,0): the tangent s = (1,0) points against the edge's order.
pde2d::discretization::TriangleMesh triangle() {
  return {{{0, 0}, {1, 0}, {0, 1}},
          {{0, 1, 2}},
          {{1, 0}, {2, 1}, {0, 2}},
          {"bottom", "diagonal", "left"}};
}

void tangential_block_by_hand() {
  // -Laplace u with du/dn + r du/ds = 0 on the bottom: beta = 1, tau = r,
  // and du/ds = u_x there. The tangential term adds
  //   K_ij = int_0^1 r phi_i d(phi_j)/dx dx,  phi_0 = 1 - x, phi_1 = x,
  // = r/2 [[-1, 1], [-1, 1]] in rows/columns (0, 1), independently of the
  // order in which the mesh stores the edge.
  const double r = .5;
  const auto mesh = triangle();
  Problem natural;
  natural.assign(flux_neumann(0), {"diagonal", "left"});
  natural.assign(neumann(0), {"bottom"});
  Problem tangential = natural;
  tangential.boundaries.back().condition.beta_x = -r * ny;
  tangential.boundaries.back().condition.beta_y = r * nx;
  for (const auto &edges :
       {mesh.boundary_edges, std::vector<std::pair<std::size_t, std::size_t>>{
                                 {0, 1}, {2, 1}, {0, 2}}}) {
    auto m = mesh;
    m.boundary_edges = edges;
    const SparseMatrix difference =
        assemble(m, tangential).stiffness - assemble(m, natural).stiffness;
    const double expected[3][3] = {
        {-r / 2, r / 2, 0}, {-r / 2, r / 2, 0}, {0, 0, 0}};
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        near(difference.coeff(i, j), expected[i][j], 1e-15);
  }
  // A variable r = 1 + x: K_ij = int (1 + x) phi_i d(phi_j)/dx, exact with
  // the default edge rule (polynomial degree 2).
  tangential.boundaries.back().condition.beta_x = -(1 + x) * ny;
  tangential.boundaries.back().condition.beta_y = (1 + x) * nx;
  const SparseMatrix difference =
      assemble(mesh, tangential).stiffness - assemble(mesh, natural).stiffness;
  // int (1+x)(1-x) = 2/3, int (1+x) x = 5/6.
  const double row0 = 2. / 3, row1 = 5. / 6;
  near(difference.coeff(0, 0), -row0, 1e-14);
  near(difference.coeff(0, 1), row0, 1e-14);
  near(difference.coeff(1, 0), -row1, 1e-14);
  near(difference.coeff(1, 1), row1, 1e-14);
}

void time_dependent_coefficients_match_substitution() {
  // Prepared assembly caches the time-independent part and evaluates the
  // rest per time; the same problem with t substituted has no time
  // dependence (everything cached) and must agree.
  const auto mesh = square_mesh(4);
  const auto problem_at = [](const Expr &time) {
    Problem p;
    p.equation.a_xx = -(2 + x);
    p.equation.a_xy = -.5;
    p.equation.a_yy = -(1 + time * y);
    p.equation.m1 = 1;
    BoundaryCondition bc;
    bc.alpha = 1 + time;
    bc.gamma = 2;
    bc.beta_normal = 1;
    bc.beta_x = -(.3 + time * x) * ny;
    bc.beta_y = (.3 + time * x) * nx;
    bc.value = sin(time + x + y);
    p.assign(bc, {"bottom", "right"});
    p.assign(dirichlet(time * x), {"top", "left"});
    return p;
  };
  const PreparedAssembly prepared(mesh, problem_at(t), {4});
  for (double time : {0., .3, 1.7}) {
    const auto dynamic = prepared.assemble(time);
    const auto fixed = assemble(mesh, problem_at(Expr(time)), 0, {4});
    near((dynamic.stiffness - fixed.stiffness).norm(), 0, 1e-13);
    near((dynamic.first_mass - fixed.first_mass).norm(), 0, 1e-13);
    near((dynamic.load - fixed.load).norm(), 0, 1e-13);
  }
  // Serial and OpenMP evaluation are bit-identical.
  if (pde2d::openmp_available()) {
    AssemblyOptions options{4};
    options.execution = {pde2d::ExecutionBackend::OpenMP, 4, 0};
    const PreparedAssembly parallel(mesh, problem_at(t), options);
    for (double time : {0., .3}) {
      const auto a = prepared.assemble(time), b = parallel.assemble(time);
      check((a.stiffness - b.stiffness).norm() == 0 &&
                (a.first_mass - b.first_mass).norm() == 0 &&
                (a.load - b.load).norm() == 0,
            "Serial and OpenMP oblique assembly differ");
    }
  }
}

void nonsymmetry_is_seen_by_the_solver() {
  const auto mesh = square_mesh(4);
  // Self-adjoint interior (-Laplace u + u): only the boundary is nonsymmetric.
  const Expr exact = x * x + y;
  Problem p;
  p.equation.c = 1;
  p.equation.source = -2 + exact;
  p.assign(oblique(p.equation, exact, 1, 0, Expr(.5)), {"bottom"});
  p.assign(dirichlet(exact), {"right", "top", "left"});
  check(!numerically_symmetric(assemble(mesh, p).stiffness),
        "Assembled oblique stiffness is nonsymmetric");
  SolverOptions cg;
  cg.method = LinearSolver::ConjugateGradient;
  expect_throws<std::runtime_error>([&] { solve_stationary(mesh, p, {}, cg); },
                                    "symmetric positive");
  SolverOptions gmres;
  gmres.method = LinearSolver::GMRES;
  check(solve_stationary(mesh, p, {}, gmres).report.converged,
        "GMRES solves the oblique problem");
}

void degenerate_edges_are_rejected() {
  const auto mesh = square_mesh(2);
  // Tangential derivative only: no normal component.
  Problem p;
  BoundaryCondition tangential;
  tangential.alpha = 1;
  tangential.beta_x = -ny;
  tangential.beta_y = nx;
  p.assign(tangential, {"bottom"});
  p.assign(dirichlet(0), {"right", "top", "left"});
  expect_throws<std::invalid_argument>([&] { assemble(mesh, p); },
                                       "tangential to the edge");
  // A normal component that changes sign on the edge (sampled divisor).
  p.boundaries[0].condition.beta_normal = x - .3;
  expect_throws<std::invalid_argument>(
      [&] { assemble(mesh, p); },
      "Oblique flux multiplier vanishes or changes sign");
}
} // namespace

int main() {
  return pde2d_test::run(
      {{"tangential_block_by_hand", tangential_block_by_hand},
       {"square_convergence", square_convergence},
       {"disk_convergence", disk_convergence},
       {"time_dependent_coefficients_match_substitution",
        time_dependent_coefficients_match_substitution},
       {"nonsymmetry_is_seen_by_the_solver", nonsymmetry_is_seen_by_the_solver},
       {"degenerate_edges_are_rejected", degenerate_edges_are_rejected}});
}
