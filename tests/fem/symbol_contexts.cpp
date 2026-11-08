// Symbol validation by context: boundary normals exist only on boundary
// edges, so fields evaluated at nodes or quadrature points (interpolated
// fields, initial conditions, exact solutions, interior coefficients) must
// reject nx and ny instead of silently evaluating them as zero.
#include "fem/error_norms.hpp"
#include "fem/evolution.hpp"
#include "discretization/square_mesh.hpp"
#include "support/check.hpp"

using pde2d::discretization::square_mesh;
using pde2d_test::check;
using pde2d_test::expect_throws;
namespace {
using namespace pde2d::fem;

void reusable_validation() {
  require_symbols(sin(x) * y + t, field_symbols, "Field");
  require_symbols(x * nx + ny, boundary_symbols, "Boundary value");
  expect_throws<std::invalid_argument>(
      [] { require_symbols(x * nx, field_symbols, "Field"); },
      "Field cannot depend on boundary normals (nx)");
  const std::array<Expr, 2> space{x, y};
  expect_throws<std::invalid_argument>(
      [&] { require_symbols(x + t, space, "Steady field"); },
      "Steady field cannot depend on t");
  // Unknown symbols are left to Evaluator's established message.
  const Expr z{SymEngine::symbol("z")};
  require_symbols(z, field_symbols, "Field");
  expect_throws<std::invalid_argument>([&] { Evaluator{z}; },
                                       "Unbound symbol in FEM expression: z");
}

void field_contexts() {
  const auto mesh = square_mesh(2);
  expect_throws<std::invalid_argument>(
      [&] { interpolate(mesh, x + nx); },
      "Interpolated field cannot depend on boundary normals (nx)");
  const auto values = interpolate(mesh, x * y + t, .5);
  check(values[4] == .25 + .5, "Field interpolation changed");
  expect_throws<std::invalid_argument>(
      [&] { PreparedErrorNorms(mesh, y * ny); },
      "Exact solution cannot depend on boundary normals (ny)");
  expect_throws<std::invalid_argument>(
      [&] { error_norms(mesh, values, x + nx); }, "Exact solution");

  Problem p;
  p.equation.m1 = 1;
  p.assign(dirichlet(0), {"bottom", "left"});
  // Boundary expressions keep the outward normal.
  p.assign(neumann(nx + ny), {"top", "right"});
  TimeOptions o;
  o.end = .02;
  o.step = .01;
  check(evolve(mesh, p, {x * y}, o).frames.size() == 3,
        "Boundary normals must stay available on boundary conditions");
  expect_throws<std::invalid_argument>(
      [&] { evolve(mesh, p, {x * y + nx}, o); },
      "Initial value cannot depend on boundary normals (nx)");
  p.equation.m1 = 0;
  p.equation.m2 = 1;
  o.method = TimeMethod::Newmark;
  expect_throws<std::invalid_argument>(
      [&] { evolve(mesh, p, {x * y, ny}, o); },
      "Initial velocity cannot depend on boundary normals (ny)");
  p.equation.source = nx;
  expect_throws<std::invalid_argument>(
      [&] { assemble(mesh, p); },
      "Interior coefficients cannot depend on boundary normals");
}
} // namespace

int main() {
  return pde2d_test::run({{"reusable_validation", reusable_validation},
                          {"field_contexts", field_contexts}});
}
