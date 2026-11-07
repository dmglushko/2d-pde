#include "fem/assembly.hpp"
#include "discretization/square_mesh.hpp"
#include "support/check.hpp"
#include <iostream>
#include <optional>
#include <stdexcept>

void dirichlet_time_derivatives() {
  using namespace fem;
  const auto mesh = square_mesh(2);
  const Expr boundary = Expr(SymEngine::abs(t.get_basic()));
  Problem p;
  p.assign(dirichlet(boundary), {"bottom", "right", "top", "left"});
  PreparedAssembly prepared(mesh, p);
  // Ordinary value conditions do not require differentiability in time.
  for (double time : {-1., 0., 1.}) {
    auto s = prepared.assemble(time);
    for (const auto &d : s.dirichlet)
      if (d.value != std::abs(time))
        throw std::runtime_error(
            "Unused derivative rejected or changed a value condition");
  }
  // Support depends on the installed symbolic library. When an evaluator for
  // a requested derivative cannot be prepared, never silently substitute
  // zero.
  for (int order : {1, 2}) {
    bool supported = true;
    double expected = 0;
    try {
      Expr derivative = boundary;
      for (int k = 0; k < order; ++k)
        derivative = diff(derivative, t);
      expected = evaluate(derivative, {0, 0, 1});
    } catch (const std::exception &) {
      supported = false;
    }
    bool rejected = false;
    std::optional<AssembledSystem> result;
    try {
      result.emplace(prepared.assemble(1, order));
    } catch (const std::exception &) {
      rejected = true;
    }
    if (rejected == supported)
      throw std::runtime_error(
          "Requested derivative capability was not preserved");
    if (supported)
      for (const auto &d : result->dirichlet)
        if (d.value != expected)
          throw std::runtime_error(
              "Requested derivative differs from the symbolic library");
  }
  // Repeated and copied preparation must keep the same deferred errors.
  auto copy = prepared;
  for (const auto &d : copy.assemble(1).dirichlet)
    if (d.value != 1)
      throw std::runtime_error("Prepared copy changed boundary value");
}
int main() {
  return pde2d_test::run(
      {{"dirichlet_time_derivatives", dirichlet_time_derivatives}});
}
