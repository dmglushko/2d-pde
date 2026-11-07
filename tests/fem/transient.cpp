#include "fem/evolution.hpp"
#include "discretization/square_mesh.hpp"
#include "fem/error_norms.hpp"
#include "support/check.hpp"
#include <iostream>
#include <stdexcept>

void dynamic_boundary_convergence() {
  using namespace fem;
  const auto mesh = square_mesh(3);
  const Expr base = 1 + x + y, exact = exp(-t) * base;
  Problem problem;
  problem.assign(dynamic_robin(.4, 1, 1, .6 * exact + exp(-t) * (nx + ny)),
                 {"bottom", "right", "top", "left"});
  for (const auto method : {TimeMethod::BackwardEuler,
                            TimeMethod::CrankNicolson, TimeMethod::Newmark}) {
    problem.equation.m1 = method == TimeMethod::Newmark ? Expr(.2) : Expr(1);
    problem.equation.m2 = method == TimeMethod::Newmark ? Expr(1) : Expr(0);
    problem.equation.source =
        method == TimeMethod::Newmark ? .8 * exact : -exact;
    double previous = 0;
    for (double dt : {.1, .05, .025}) {
      TimeOptions options;
      options.method = method;
      options.end = .4;
      options.step = dt;
      const auto solution = evolve(mesh, problem, base, -base, options);
      if (solution.frames.size() !=
          static_cast<std::size_t>(std::lround(.4 / dt)) + 1)
        throw std::runtime_error(
            "Accumulated roundoff created a spurious time step");
      const auto &last = solution.frames.back();
      if (std::abs(last.time - .4) > 1e-14)
        throw std::runtime_error("Incorrect final time");
      const double error = error_norms(mesh, last.values, exact, last.time).l2;
      std::cout << "dynamic boundary method=" << static_cast<int>(method)
                << " dt=" << dt << " L2=" << error << '\n';
      if (previous > 0 &&
          previous / error < (method == TimeMethod::BackwardEuler ? 1.8 : 3.5))
        throw std::runtime_error("Lost time accuracy with boundary u_t");
      previous = error;
    }
  }
  // Moving essential data: exactly representable spatially and quadratic in
  // time. Newmark should reproduce it, including the shorter final step.
  const Expr moving = base * (1 + t + t * t);
  problem = Problem{};
  problem.equation.m2 = 1;
  problem.equation.m1 = .3;
  problem.equation.source = base * (2 + .3 * (1 + 2 * t));
  problem.assign(dirichlet(moving), {"bottom", "right", "top", "left"});
  TimeOptions options;
  options.method = TimeMethod::Newmark;
  options.end = .25;
  options.step = .1;
  auto result = evolve(mesh, problem, base, base, options);
  if (std::abs(result.frames.back().time - .25) > 1e-14)
    throw std::runtime_error("Short final time step missed endpoint");
  for (const auto &frame : result.frames)
    if (error_norms(mesh, frame.values, moving, frame.time).l2 > 1e-10)
      throw std::runtime_error(
          "Moving Dirichlet lifting corrupted an exact quadratic solution");
  // General derivative terms can cancel to a value condition. Initialization
  // must differentiate the reduced condition, not inspect only raw fields.
  auto &general = problem.boundaries.front().condition;
  general.beta_flux = 1;
  general.beta_normal = -1;
  result = evolve(mesh, problem, base, base, options);
  for (const auto &frame : result.frames)
    if (error_norms(mesh, frame.values, moving, frame.time).l2 > 1e-10)
      throw std::runtime_error(
          "Reduced Dirichlet boundary lost its time derivatives");
}
int main() {
  return pde2d_test::run(
      {{"dynamic_boundary_convergence", dynamic_boundary_convergence}});
}
