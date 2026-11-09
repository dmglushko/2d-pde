// FEM on a multiply connected domain (annulus). Oracles: a manufactured
// stationary solution with Dirichlet data on the outer circle and a Robin
// condition on the hole (outward normal pointing into the hole), measured
// second-order L2 convergence; normals checked against the circle geometry.
#include "discretization/geometry.hpp"
#include "discretization/triangulate.hpp"
#include "fem/boundary_assignment.hpp"
#include "fem/error_norms.hpp"
#include "fem/interpolation.hpp"
#include "fem/solver.hpp"
#include "support/check.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

using namespace pde2d::fem;
using namespace pde2d::discretization;
using pde2d_test::check;

namespace {
const double R = 1.0, r_in = 0.4;
MeshResult annulus_mesh(double h) {
  AnnulusGeometry g(R, r_in);
  MeshOptions o;
  o.boundary.max_edge_length = h;
  o.boundary_treatment = BoundaryTreatment::RefineOnCurves;
  o.refinement.max_edge_length = h;
  auto result = generate_mesh(g, o);
  if (!result.report.size_satisfied)
    throw std::runtime_error(result.report.message);
  return result;
}
const Expr exact = exp(0.5 * x) * sin(1.2 * y) + x * y;
const Expr du_dn = nx * diff(exact, x) + ny * diff(exact, y);
Problem dirichlet_outside_robin_inside() {
  Problem p;
  p.equation.source = -(diff(diff(exact, x), x) + diff(diff(exact, y), y));
  p.assign(dirichlet(exact), {"outer"});
  // 2u + du/dn = g with n the outward normal of the domain, i.e. pointing
  // into the hole.
  p.assign(robin(2, 1, 2 * exact + du_dn), {"inner"});
  return p;
}

void manufactured_convergence_on_annulus() {
  const auto p = dirichlet_outside_robin_inside();
  ErrorNorms previous{};
  std::size_t previous_nodes = 0;
  for (double h : {0.2, 0.1, 0.05}) {
    const auto mesh = annulus_mesh(h).mesh;
    auto s = solve_stationary(mesh, p, AssemblyOptions{6});
    check(s.report.converged, "Stationary solve converged");
    auto e = error_norms(mesh, s.values, exact);
    std::cout << "h=" << h << " nodes=" << mesh.nodes.size() << " L2=" << e.l2
              << " H1=" << e.h1_seminorm;
    if (previous_nodes) {
      // Rates in the mesh size h ~ N^(-1/2).
      const double scale =
          0.5 * std::log(double(mesh.nodes.size()) / previous_nodes);
      const double l2_rate = std::log(previous.l2 / e.l2) / scale,
                   h1_rate =
                       std::log(previous.h1_seminorm / e.h1_seminorm) / scale;
      std::cout << " L2 rate=" << l2_rate << " H1 rate=" << h1_rate;
      check(l2_rate > 1.8, "Second-order L2 convergence on the annulus");
      check(h1_rate > 0.85, "First-order H1 convergence on the annulus");
    }
    std::cout << '\n';
    previous = e;
    previous_nodes = mesh.nodes.size();
  }
}

void hole_normals_point_into_the_hole() {
  const auto mesh = annulus_mesh(0.2).mesh;
  const auto p = dirichlet_outside_robin_inside();
  const auto binding = bind_boundary(mesh, p);
  check(binding.regions == std::vector<std::string>{"outer", "inner"});
  std::size_t inner = 0;
  for (const auto &edge : binding.edges) {
    const auto &f = edge.frame;
    const double mx = (f.a.x + f.b.x) / 2, my = (f.a.y + f.b.y) / 2,
                 radial =
                     (f.normal_x * mx + f.normal_y * my) / std::hypot(mx, my);
    if (binding.regions[edge.region] == "inner") {
      ++inner;
      check(radial < -0.99, "Hole normals point towards the center");
    } else
      check(radial > 0.99, "Outer normals point away from the center");
  }
  check(inner == boundary_loops(mesh)[1].size(), "Inner edges are the hole");
}

} // namespace

int main() {
  return pde2d_test::run(
      {{"manufactured_convergence_on_annulus",
        manufactured_convergence_on_annulus},
       {"hole_normals_point_into_the_hole", hole_normals_point_into_the_hole}});
}
