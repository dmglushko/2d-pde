#include "fem/error_norms.hpp"
#include "discretization/square_mesh.hpp"
using pde2d::discretization::square_mesh;
#include <iostream>
#include <stdexcept>

#include "support/error_norms_reference.hpp"
#include "support/check.hpp"
using pde2d_test::check;
void error_norm_diagnostics() {
  using namespace pde2d::fem;
  auto mesh = square_mesh(20);
  Vector v = Vector::Zero(mesh.nodes.size());
  const Expr exact = sin(2 * x) * cos(y) + t * x * y;
  PreparedErrorNorms prepared(mesh, exact);
  for (double time : {0., .3, 2.}) {
    for (std::size_t i = 0; i < mesh.nodes.size(); i++)
      v[i] = std::sin(mesh.nodes[i].x) + time;
    const auto old = reference(mesh, v, exact, time),
               now = prepared.evaluate(v, time),
               compat = error_norms(mesh, v, exact, time);
    check(std::abs(old.l2 - now.l2) < 1e-13 &&
          std::abs(old.h1_seminorm - now.h1_seminorm) < 1e-13);
    check(compat.l2 == now.l2);
    if (pde2d::openmp_available()) {
      auto par =
          prepared.evaluate(v, time, {pde2d::ExecutionBackend::OpenMP, 2, 0});
      check(par.l2 == now.l2 && par.h1_seminorm == now.h1_seminorm);
    }
  }
  const auto snapshot = prepared.evaluate(v, .2);
  const double original_x = mesh.nodes[0].x;
  mesh.nodes[0].x = 100;
  check(prepared.evaluate(v, .2).l2 == snapshot.l2);
  mesh.nodes[0].x = original_x;
  auto transformed = mesh;
  for (auto &node : transformed.nodes) {
    node.x = 2 * node.x + .3 * node.y + 4;
    node.y = 3 * node.y - 2;
  }
  const auto expected = reference(transformed, v, exact, .7),
             physical = PreparedErrorNorms(transformed, exact).evaluate(v, .7);
  check(std::abs(expected.l2 - physical.l2) < 1e-12 &&
        std::abs(expected.h1_seminorm - physical.h1_seminorm) < 1e-12);
  const Expr affine = 2 * x - 3 * y + t;
  for (std::size_t i = 0; i < mesh.nodes.size(); i++)
    v[i] = 2 * mesh.nodes[i].x - 3 * mesh.nodes[i].y + .4;
  auto zero = PreparedErrorNorms(mesh, affine).evaluate(v, .4);
  check(zero.l2 < 1e-14 && zero.h1_seminorm < 1e-13);
  if (!pde2d::openmp_available()) {
    bool rejected = false;
    try {
      prepared.evaluate(v, 0, {pde2d::ExecutionBackend::OpenMP, 2, 0});
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    check(rejected);
  }
  bool rejected = false;
  try {
    prepared.evaluate(Vector::Zero(1));
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  check(rejected);
  v.setZero();
  auto analytic = PreparedErrorNorms(mesh, x * x).evaluate(v);
  check(std::abs(analytic.l2 - std::sqrt(.2)) < 1e-13 &&
        std::abs(analytic.h1_seminorm - std::sqrt(4. / 3)) < 1e-13);
}
int main() {
  return pde2d_test::run({{"error_norm_diagnostics", error_norm_diagnostics}});
}
