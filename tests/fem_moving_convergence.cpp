#include "../examples/fem/square_mesh.hpp"
#include "fem/evolution.hpp"
#include <iostream>
using namespace fem;
int main() {
  auto mesh = square_mesh(4);
  const Expr base = 1 + x + y, exact = base * exp(t);
  for (auto method : {TimeMethod::BackwardEuler, TimeMethod::CrankNicolson,
                      TimeMethod::Newmark}) {
    bool wave = method == TimeMethod::Newmark;
    Problem p;
    p.equation.m1 = wave ? .3 : 1;
    p.equation.m2 = wave ? 1 : 0;
    p.equation.source = (wave ? 1.3 : 1) * exact;
    p.assign(dirichlet(exact), {"left", "bottom"});
    p.assign(dynamic_flux_robin(.4, 1, 1, 1.4 * exact + exp(t) * (nx + ny)),
             {"right", "top"});
    double previous = 0;
    for (double dt : {.1, .05, .025}) {
      TimeOptions o;
      o.method = method;
      o.end = .4;
      o.step = dt;
      o.assembly.quadrature_degree = 6;
      auto result = evolve(mesh, p, base, base, o);
      auto &last = result.frames.back();
      if (last.time != o.end || !last.values.allFinite() || !last.velocity.allFinite())
        throw std::runtime_error("Nonfinite or incomplete moving boundary solution");
      double error = 0;
      for (std::size_t i = 0; i < mesh.nodes.size(); ++i)
        error = std::max(error, std::abs(last.values[i] - (1 + mesh.nodes[i].x +
                                                           mesh.nodes[i].y) *
                                                              std::exp(.4)));
      std::cout << "moving mixed method=" << int(method) << " dt=" << dt
                << " max_error=" << error << '\n';
      if (!std::isfinite(error) || error <= 0)
        throw std::runtime_error("Invalid moving boundary error");
      if (previous &&
          previous / error <
              (wave || method == TimeMethod::CrankNicolson ? 3.4 : 1.7))
        throw std::runtime_error("Moving boundary convergence failed");
      previous = error;
    }
  }
}
