// A program that links pde2d_solve and includes Eigen itself (review
// finding): it must be compiled with OpenMP too, so its copies of Eigen's
// inline functions (Eigen::nbThreads()) are the library's. Compiled at -O0
// so those functions are not inlined away here. With a serial copy the
// linker may keep, the solvers silently ran on one thread.

#include "support/check.hpp"
#include "support/l_shape_mesh.hpp"

#include "pde2d/fem/assembler.hpp"
#include "pde2d/solve/conjugate_gradient.hpp"

#include <Eigen/Core>

#include <memory>

using namespace pde2d::fem;
using namespace pde2d::solve;
using pde2d::test::check;

int main() {
  return pde2d::test::run({
      {"a consumer of pde2d_solve is compiled with OpenMP",
       [] {
#ifndef _OPENMP
         check(false, "pde2d_solve passes OpenMP on to its consumers");
#endif
         check(Eigen::nbThreads() >= 1, "Eigen's inline function is used");
       }},
      {"the solver uses the threads it was given",
       [] {
         BoundaryCondition zero{.alpha = Expression("1"), .g = Expression("0")};
         const auto problem = std::make_shared<const MeshedProblem>(
             pde2d::test::refined_l_shape(0.04),
             Problem{.equation = {.a_xx = Expression("-1"),
                                  .a_yy = Expression("-1"),
                                  .f = Expression("1")},
                     .boundary_conditions = {{"bottom", zero},
                                             {"right", zero},
                                             {"inner", zero},
                                             {"top", zero},
                                             {"left", zero}}});
         const auto assembled = Assembler().assemble(problem, 0.0, nullptr, 0);
         const auto prepared =
             ConjugateGradient({.max_iterations = 20000, .threads = 4})
                 .prepare(std::make_shared<const CsrMatrix>(
                     assembled.stiffness().free));
         const auto result = prepared->solve(assembled.right_hand_side());
         check(result.threads == 4, "4 threads");
       }},
  });
}
