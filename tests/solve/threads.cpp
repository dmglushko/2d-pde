// The iterative solvers, the StaticSolver and the Integrator with several
// threads (docs/decisions.md, "Parallel assembly and iterative solvers are
// built now"): the answers, iteration counts and residuals are bit for bit
// the same for every thread count. The matrices have more than 20000
// nonzeros, so Eigen's matrix-vector product really runs in parallel. At
// most 4 threads.

#include "support/check.hpp"
#include "support/l_shape_mesh.hpp"

#include "pde2d/fem/assembler.hpp"
#include "pde2d/solve/bicgstab.hpp"
#include "pde2d/solve/conjugate_gradient.hpp"
#include "pde2d/solve/fixed_step.hpp"
#include "pde2d/solve/implicit_euler.hpp"
#include "pde2d/solve/integrator.hpp"
#include "pde2d/solve/sparse_lu.hpp"
#include "pde2d/solve/static_solver.hpp"

#include <algorithm>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>

using namespace pde2d::fem;
using namespace pde2d::solve;
using pde2d::test::check;
using pde2d::test::expect_throws;

namespace {

Expression e(const char *text) { return Expression(text); }

bool same_bits(std::span<const double> x, std::span<const double> y) {
  return std::ranges::equal(x, y, [](double a, double b) {
    return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b);
  });
}

bool same(const LinearSolveResult &a, const LinearSolveResult &b) {
  return same_bits(a.values, b.values) && a.iterations == b.iterations &&
         std::bit_cast<std::uint64_t>(a.residual) ==
             std::bit_cast<std::uint64_t>(b.residual);
}

std::shared_ptr<const pde2d::mesh::Mesh> l_shape() {
  static const auto mesh = pde2d::test::refined_l_shape(0.04);
  return mesh;
}

// -div(grad u) + b . grad u + m1 u_t = f with u = 0 on the whole
// boundary: A = -identity, so the stiffness K is positive definite; `b_x`
// 0 gives a symmetric matrix.
std::shared_ptr<const MeshedProblem> poisson(const char *b_x,
                                             const char *m1 = "0") {
  BoundaryCondition zero{.alpha = e("1"), .g = e("0")};
  return std::make_shared<const MeshedProblem>(
      l_shape(), Problem{.equation = {.a_xx = e("-1"),
                                      .a_yy = e("-1"),
                                      .b_x = e(b_x),
                                      .m1 = e(m1),
                                      .f = e("1 + sin(t)*x")},
                         .boundary_conditions = {{"bottom", zero},
                                                 {"right", zero},
                                                 {"inner", zero},
                                                 {"top", zero},
                                                 {"left", zero}},
                         .initial = {.u = e("x*y")}});
}

// Solves the static system of `problem` with `solver`.
LinearSolveResult solve_with(const LinearSolver &solver,
                             const std::shared_ptr<const MeshedProblem> &problem) {
  const auto assembled = Assembler().assemble(problem, 0.0, nullptr, 0);
  check(assembled.stiffness().free.values.size() > 20000,
        "more than 20000 nonzeros: Eigen's product runs in parallel");
  const auto prepared = solver.prepare(
      std::make_shared<const CsrMatrix>(assembled.stiffness().free));
  return prepared->solve(assembled.right_hand_side());
}

template <class Solver, class Options>
void identical_for_thread_counts(const std::shared_ptr<const MeshedProblem> &p,
                                 const std::string &what) {
  const auto serial = solve_with(Solver(Options{.max_iterations = 5000}), p);
  check(serial.iterations > 10, what + ": a real iterative solve");
  check(serial.threads == 1, what + ": 1 thread reported");
  for (const std::size_t threads : {2, 4}) {
    const auto parallel = solve_with(
        Solver(Options{.max_iterations = 5000, .threads = threads}), p);
    check(same(parallel, serial),
          what + ", " + std::to_string(threads) +
              " threads: the same values, iterations and residual");
    check(parallel.threads == threads, what + ": thread count reported");
  }
}

} // namespace

int main() {
  return pde2d::test::run({
      {"conjugate gradients: the same bits for 1, 2 and 4 threads",
       [] {
         identical_for_thread_counts<ConjugateGradient,
                                     ConjugateGradientOptions>(poisson("0"),
                                                               "CG");
       }},
      {"BiCGSTAB: the same bits for 1, 2 and 4 threads",
       [] {
         identical_for_thread_counts<BiCgStab, BiCgStabOptions>(poisson("5"),
                                                                "BiCGSTAB");
       }},
      {"a direct solver reports one thread",
       [] {
         const auto problem = poisson("0");
         const auto assembled = Assembler().assemble(problem, 0.0, nullptr, 0);
         const auto solved =
             SparseLu()
                 .prepare(std::make_shared<const CsrMatrix>(
                     assembled.stiffness().free))
                 ->solve(assembled.right_hand_side());
         check(solved.threads == 1, "1 thread");
       }},
      {"StaticSolver with threads: the same solution",
       [] {
         const auto problem = poisson("0");
         const auto serial =
             StaticSolver(std::make_shared<ConjugateGradient>(
                              ConjugateGradientOptions{.max_iterations = 5000}))
                 .solve(problem);
         const auto parallel =
             StaticSolver(std::make_shared<ConjugateGradient>(
                              ConjugateGradientOptions{.max_iterations = 5000,
                                                       .threads = 4}),
                          {.threads = 4})
                 .solve(problem);
         check(same_bits(parallel.solution.values, serial.solution.values),
               "the same bits");
         check(parallel.reports.at(0).metrics.at("threads") == 4 &&
                   serial.reports.at(0).metrics.at("threads") == 1,
               "assembly threads reported");
         check(parallel.reports.at(1).metrics.at("linear_threads") == 4 &&
                   serial.reports.at(1).metrics.at("linear_threads") == 1,
               "linear solver threads reported");
         check(parallel.reports.at(1).metrics.at("linear_iterations") ==
                   serial.reports.at(1).metrics.at("linear_iterations"),
               "the same iterations");
       }},
      {"Integrator with threads: the same solution",
       [] {
         const auto problem = poisson("0", "1");
         const auto run = [&](std::size_t threads) {
           return Integrator(std::make_shared<ImplicitEuler>(),
                             std::make_shared<FixedStep>(
                                 FixedStepOptions{.dt = 0.05}),
                             std::make_shared<ConjugateGradient>(
                                 ConjugateGradientOptions{
                                     .max_iterations = 5000,
                                     .threads = threads}),
                             {.threads = threads})
               .integrate(problem, 0.0, 0.2, {.keep_step_history = true});
         };
         const auto serial = run(1);
         const auto parallel = run(4);
         check(same_bits(parallel.final.values, serial.final.values),
               "the same bits");
         check(parallel.history.size() == serial.history.size(), "steps");
         for (std::size_t i = 0; i < serial.history.size(); ++i)
           check(parallel.history[i].linear_iterations ==
                     serial.history[i].linear_iterations,
                 "the same iterations in step " + std::to_string(i));
         check(parallel.report.metrics.at("assembly_threads") == 4 &&
                   serial.report.metrics.at("assembly_threads") == 1,
               "assembly threads reported");
       }},
      {"thread counts are checked",
       [] {
         const std::size_t too_many = 1025;
         const std::size_t negative = std::numeric_limits<std::size_t>::max();
         for (const std::size_t threads : {too_many, negative}) {
           expect_throws<std::invalid_argument>(
               [&] { (void)ConjugateGradient({.threads = threads}); },
               "conjugate_gradient: threads must be at most 1024");
           expect_throws<std::invalid_argument>(
               [&] { (void)BiCgStab({.threads = threads}); },
               "bicgstab: threads must be at most 1024");
           expect_throws<std::invalid_argument>(
               [&] {
                 (void)StaticSolver(std::make_shared<SparseLu>(),
                                    {.threads = threads});
               },
               "assembler: threads must be at most 1024");
           expect_throws<std::invalid_argument>(
               [&] {
                 (void)Integrator(std::make_shared<ImplicitEuler>(),
                                  std::make_shared<FixedStep>(
                                      FixedStepOptions{.dt = 0.1}),
                                  std::make_shared<SparseLu>(),
                                  {.threads = threads});
               },
               "assembler: threads must be at most 1024");
         }
         check(ConjugateGradient({.threads = 1024}).options().threads == 1024 &&
                   BiCgStab({.threads = 0}).options().threads == 0,
               "1024 and 0 are accepted");
       }},
  });
}
