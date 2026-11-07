#include "fem/evolution.hpp"
#include "discretization/square_mesh.hpp"
using pde2d::discretization::square_mesh;
#include <iostream>
int main() {
  using namespace pde2d::fem;
  auto mesh = square_mesh(10);
  Problem p;
  p.equation.m1 = 1;
  p.assign(dirichlet(0), {"bottom", "right", "top", "left"});
  for (auto method : {TimeMethod::BackwardEuler, TimeMethod::Newmark}) {
    p.equation.m1 = method == TimeMethod::Newmark ? .2 : 1;
    p.equation.m2 = method == TimeMethod::Newmark ? 1 : 0;
    TimeOptions o;
    o.method = method;
    o.end = 2;
    o.step = .01;
    auto e =
        evolve(mesh, p,
               {sin(3.141592653589793 * x) * sin(3.141592653589793 * y), 0}, o);
    std::cout << "method=" << int(method)
              << " total=" << e.timings.total_seconds
              << " assembly=" << e.timings.assembly_seconds
              << " solver=" << e.timings.solver_seconds
              << " rank=" << e.timings.solver.rank_check_seconds
              << " factor=" << e.timings.solver.factorization_seconds
              << " solves=" << e.timings.linear_solves
              << " assemblies=" << e.timings.assembly_calls
              << " final_norm=" << e.frames.back().values.stableNorm()
#ifdef SOLVER6_PREPARED
              << " preparations=" << e.timings.solver.preparations
              << " reuses=" << e.timings.solver_reuses
#endif
              << '\n';
  }
}
