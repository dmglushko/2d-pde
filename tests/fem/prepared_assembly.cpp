#include "discretization/square_mesh.hpp"
using pde2d::discretization::square_mesh;
#include "fem/assembly.hpp"
#include "support/check.hpp"
#include <iostream>
#include <stdexcept>
using namespace pde2d::fem;
using pde2d_test::check;
namespace {
void near(double a, double b) { pde2d_test::near(a, b, 3e-10); }
void same(const AssembledSystem &a, const AssembledSystem &b) {
  near((a.stiffness - b.stiffness).norm(), 0);
  near((a.first_mass - b.first_mass).norm(), 0);
  near((a.second_mass - b.second_mass).norm(), 0);
  near((a.load - b.load).norm(), 0);
  check(a.dirichlet.size() == b.dirichlet.size());
  for (std::size_t k = 0; k < a.dirichlet.size(); ++k) {
    check(a.dirichlet[k].node == b.dirichlet[k].node);
    near(a.dirichlet[k].value, b.dirichlet[k].value);
  }
}
void known() {
  pde2d::discretization::TriangleMesh mesh{{{0, 0}, {1, 0}, {0, 1}},
                                           {{0, 1, 2}},
                                           {{0, 1}, {1, 2}, {2, 0}},
                                           {"b", "b", "b"}};
  Problem p;
  p.equation.a_xx = -2;
  p.equation.a_xy = -2;
  p.equation.a_yy = -3;
  p.equation.m1 = 2 + t;
  p.equation.m2 = 3 + 2 * t;
  p.equation.c = 4 * t;
  p.equation.source = 6 * (1 + t);
  p.assign(flux_neumann(0), {"b"});
  PreparedAssembly a(mesh, p);
  const double k[3][3] = {{3.5, -1.5, -2}, {-1.5, 1, .5}, {-2, .5, 1.5}};
  for (double time : {0., .3, 1., .3}) {
    auto s = a.assemble(time);
    for (int i = 0; i < 3; ++i) {
      near(s.load[i], 1 + time);
      for (int j = 0; j < 3; ++j) {
        double m = (i == j ? 2. : 1.) / 24;
        near(s.stiffness.coeff(i, j), k[i][j] + 4 * time * m);
        near(s.first_mass.coeff(i, j), (2 + time) * m);
        near(s.second_mass.coeff(i, j), (3 + 2 * time) * m);
      }
    }
  }
  auto before = a.assemble(.3);
  mesh.nodes[0].x = 50;
  mesh.elements.clear();
  p.equation.m1 = 99;
  p.boundaries.clear();
  same(before, a.assemble(.3));
  check(a.mesh().nodes[0].x == 0);
}
void patch() {
  auto mesh = square_mesh(4);
  Problem p;
  auto &e = p.equation;
  e.a_xx = -(2 + x + t);
  e.a_xy = -(1 + x + y + t);
  e.a_yy = -(3 + y + 2 * t);
  e.b_x = 1 + y + t;
  e.b_y = 2 - x + t;
  e.c = 1 + x + t;
  e.m1 = 1 + x + t;
  e.m2 = 2 - y + t * t;
  Expr u = 1 + 2 * x - 3 * y, f = 1 + t + t * t, ut = u * f,
       dt = u * (1 + 2 * t), dtt = 2 * u;
  Expr q = -nx * (2 * e.a_xx - 3 * e.a_xy / 2) - ny * (e.a_xy - 3 * e.a_yy);
  e.source = f * (2 * e.b_x - 3 * e.b_y + e.c * u) + e.m1 * dt + e.m2 * dtt;
  p.assign(dynamic_flux_robin(2 + t, 3 + x, 4 + t,
                              (2 + t) * dt + (3 + x) * ut + (4 + t) * f * q),
           {"bottom", "top", "left", "right"});
  PreparedAssembly a(mesh, p, AssemblyOptions{6});
  Vector exact(mesh.nodes.size());
  for (std::size_t i = 0; i < mesh.nodes.size(); ++i)
    exact[i] = 1 + 2 * mesh.nodes[i].x - 3 * mesh.nodes[i].y;
  for (double time : {.2, .7, .2}) {
    auto s = a.assemble(time);
    near((s.stiffness * (exact * (1 + time + time * time)) +
          s.first_mass * (exact * (1 + 2 * time)) +
          s.second_mass * (2 * exact) - s.load)
             .norm(),
         0);
    same(s, assemble(mesh, p, time, AssemblyOptions{6}));
  }
  // Isolate partial boundary caching from the time-dependent interior.
  Problem fixed;
  fixed.assign(dynamic_flux_robin(2, 3, 4, t),
               {"bottom", "top", "left", "right"});
  PreparedAssembly partial(mesh, fixed);
  auto s0 = partial.assemble(.2), s1 = partial.assemble(.7);
  near((s0.stiffness - s1.stiffness).norm(), 0);
  near((s0.first_mass - s1.first_mass).norm(), 0);
  near((s1.load - s0.load).sum(), .5);
  near(s0.first_mass.sum(), 2);
}
void moving() {
  auto mesh = square_mesh(2);
  Problem p;
  BoundaryCondition bc;
  bc.alpha = 2 + t;
  bc.value = (2 + t) * (1 + x + y + t * t * t);
  p.assign(bc, {"bottom", "top", "left", "right"});
  PreparedAssembly a(mesh, p);
  for (int order = 0; order < 3; ++order)
    for (double time : {0., .4, 1.}) {
      auto s = a.assemble(time, order);
      for (auto d : s.dirichlet) {
        double expected = order == 0
                              ? 1 + mesh.nodes[d.node].x +
                                    mesh.nodes[d.node].y + time * time * time
                          : order == 1 ? 3 * time * time
                                       : 6 * time;
        near(d.value, expected);
      }
    }
  // Time zero need not be in the domain of a time-dependent coefficient.
  p.equation.source = 1 / t;
  PreparedAssembly singular(mesh, p);
  near(singular.assemble(1).load.sum(), 1);
  // Edge restriction removes this derivative term before Dirichlet reduction.
  p.boundaries.clear();
  bc.beta_x = y;
  p.assign(bc, {"bottom"});
  p.assign(flux_neumann(0), {"top", "left", "right"});
  PreparedAssembly reduced(mesh, p);
  for (const auto &d : reduced.assemble(.4, 2).dirichlet)
    near(d.value, 2.4);
  bool rejected = false;
  try {
    singular.assemble(0);
  } catch (const std::domain_error &) {
    rejected = true;
  }
  check(rejected);
}
} // namespace
int main() {
  return pde2d_test::run(
      {{"known", known}, {"patch", patch}, {"moving", moving}});
}
