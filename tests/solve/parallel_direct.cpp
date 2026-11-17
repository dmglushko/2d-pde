// The parallel direct solvers (SupernodalCholesky, MultifrontalLu) on
// assembled problems (docs/decisions.md, "Parallel direct solvers come from
// SuiteSparse"): their solutions agree with the Eigen direct solvers', they
// refuse what they cannot handle, and the thread count changes the answer
// at most in the last digits, never with 1 thread. At most 4 threads.

#include "support/check.hpp"
#include "support/l_shape_mesh.hpp"

#include "pde2d/fem/assembler.hpp"
#include "pde2d/solve/fixed_step.hpp"
#include "pde2d/solve/implicit_euler.hpp"
#include "pde2d/solve/implicit_midpoint.hpp"
#include "pde2d/solve/integrator.hpp"
#include "pde2d/solve/multifrontal_lu.hpp"
#include "pde2d/solve/simplicial_ldlt.hpp"
#include "pde2d/solve/sparse_lu.hpp"
#include "pde2d/solve/static_solver.hpp"
#include "pde2d/solve/supernodal_cholesky.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

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

// max |x - y| / max |y|.
double relative_difference(std::span<const double> x,
                           std::span<const double> y) {
  check(x.size() == y.size(), "the same number of values");
  double difference = 0.0, scale = 0.0;
  for (std::size_t i = 0; i < x.size(); ++i) {
    difference = std::max(difference, std::abs(x[i] - y[i]));
    scale = std::max(scale, std::abs(y[i]));
  }
  return difference / scale;
}

std::shared_ptr<const pde2d::mesh::Mesh> l_shape() {
  static const auto mesh = pde2d::test::refined_l_shape(0.04);
  return mesh;
}

const std::vector<std::string> sides = {"bottom", "right", "inner", "top",
                                        "left"};

// a (u_xx + u_yy) + b_x u_x + c u + m1 u_t = f on the L-shape, u = 0 on
// the whole boundary (or u_n = 0 with `neumann`). a = -1 gives a positive
// definite matrix, a = 1 a negative definite one; b_x not 0 makes it
// non-symmetric; c below minus the first eigenvalue of -Laplace (about
// 9.6) makes it indefinite.
std::shared_ptr<const MeshedProblem>
problem(const char *a, const char *b_x = "0", const char *c = "0",
        const char *m1 = "0", bool neumann = false) {
  Problem p{.equation = {.a_xx = e(a),
                         .a_yy = e(a),
                         .b_x = e(b_x),
                         .c = e(c),
                         .m1 = e(m1),
                         .f = e("1 + sin(t)*x")},
            .initial = {.u = e("x*y")}};
  for (const auto &side : sides)
    p.boundary_conditions.emplace(
        side, neumann ? BoundaryCondition{.beta_n = e("1"), .g = e("0")}
                      : BoundaryCondition{.alpha = e("1"), .g = e("0")});
  return std::make_shared<const MeshedProblem>(l_shape(), std::move(p));
}

// The static system of `meshed`: its matrix and right-hand side.
struct System {
  std::shared_ptr<const CsrMatrix> matrix;
  std::vector<double> right_hand_side;
};

System system_of(const std::shared_ptr<const MeshedProblem> &meshed) {
  const auto assembled = Assembler().assemble(meshed, 0.0, nullptr, 0);
  return {
      std::make_shared<const CsrMatrix>(assembled.stiffness().free),
      {assembled.right_hand_side().begin(), assembled.right_hand_side().end()}};
}

LinearSolveResult solve_with(const LinearSolver &solver, const System &s) {
  return solver.prepare(s.matrix)->solve(s.right_hand_side);
}

// `solver` agrees with `reference` on `s`, with a residual below 1e-10
// (both reach about 1e-12 on these systems: rounding times the condition
// number).
void agrees(const LinearSolver &solver, const LinearSolver &reference,
            const System &s) {
  const auto x = solve_with(solver, s);
  const auto y = solve_with(reference, s);
  check(x.residual < 1e-10,
        solver.name() + ": residual " + std::format("{}", x.residual));
  check(relative_difference(x.values, y.values) < 1e-10,
        solver.name() + " agrees with " + reference.name());
}

// 1, 2 and 4 threads: answers within a tight tolerance of each other,
// each count reported; 1 thread twice gives the same bits.
template <class Solver, class Options> void thread_counts(const System &s) {
  const auto serial = solve_with(Solver(Options{}), s);
  const auto again = solve_with(Solver(Options{.threads = 1}), s);
  check(same_bits(serial.values, again.values) &&
            std::bit_cast<std::uint64_t>(serial.residual) ==
                std::bit_cast<std::uint64_t>(again.residual),
        "1 thread twice: the same bits");
  check(serial.threads == 1, "1 thread reported");
  for (const std::size_t threads : {2, 4}) {
    const Solver solver(Options{.threads = threads});
    const auto parallel = solve_with(solver, s);
    check(parallel.threads == threads,
          solver.name() + ": " + std::to_string(threads) + " threads reported");
    check(relative_difference(parallel.values, serial.values) < 1e-12,
          solver.name() + ": " + std::to_string(threads) +
              " threads agree with 1");
    check(parallel.residual < 1e-10,
          solver.name() + ": residual " + std::format("{}", parallel.residual));
  }
}

IntegrationResult
integrate_with(std::shared_ptr<const LinearSolver> solver,
               const std::shared_ptr<const MeshedProblem> &p) {
  return Integrator(std::make_shared<ImplicitMidpoint>(),
                    std::make_shared<FixedStep>(FixedStepOptions{.dt = 0.05}),
                    std::move(solver))
      .integrate(p, 0.0, 0.2);
}

} // namespace

int main() {
  return pde2d::test::run({
      {"Poisson on the L-shape: both agree with the Eigen solvers",
       [] {
         const auto s = system_of(problem("-1"));
         check(s.matrix->rows() > 1000, "a real system");
         agrees(SupernodalCholesky(), SimplicialLdlt(), s);
         agrees(MultifrontalLu(), SparseLu(), s);
       }},
      {"convection: MultifrontalLu agrees with SparseLu",
       [] {
         const auto s = system_of(problem("-1", "5"));
         check(!is_symmetric(*s.matrix), "a non-symmetric matrix");
         agrees(MultifrontalLu(), SparseLu(), s);
       }},
      {"an indefinite symmetric matrix: the LU solves it, Cholesky refuses",
       [] {
         const auto s = system_of(problem("-1", "0", "-40"));
         agrees(MultifrontalLu(), SparseLu(), s);
         expect_throws<std::runtime_error>(
             [&] { (void)SupernodalCholesky().prepare(s.matrix); },
             "supernodal_cholesky: the matrix is not positive definite");
         // Negative definite: the equation written with a = 1.
         expect_throws<std::runtime_error>(
             [&] {
               (void)SupernodalCholesky().prepare(
                   system_of(problem("1")).matrix);
             },
             "supernodal_cholesky: the matrix is not positive definite (the "
             "factorization stopped at column 0 of");
       }},
      {"time steps: both agree with the Eigen solvers",
       [] {
         // Heat equation, symmetric step matrix M1 / dt + A / 2.
         const auto heat = problem("-1", "0", "0", "1");
         const auto ldlt =
             integrate_with(std::make_shared<SimplicialLdlt>(), heat);
         const auto cholesky =
             integrate_with(std::make_shared<SupernodalCholesky>(
                                SupernodalCholeskyOptions{.threads = 2}),
                            heat);
         check(relative_difference(cholesky.final.values, ldlt.final.values) <
                   1e-10,
               "Cholesky agrees with LDL^T");
         check(cholesky.report.metrics.at("linear_threads") == 2,
               "the integrator reports the solver's threads");
         // With convection: a non-symmetric step matrix.
         const auto convection = problem("-1", "5", "0", "1");
         const auto lu =
             integrate_with(std::make_shared<SparseLu>(), convection);
         const auto multifrontal =
             integrate_with(std::make_shared<MultifrontalLu>(), convection);
         check(relative_difference(multifrontal.final.values, lu.final.values) <
                   1e-10,
               "MultifrontalLu agrees with SparseLu");
       }},
      {"Cholesky is refused a non-symmetric matrix by its requirement",
       [] {
         const auto convection = problem("-1", "5");
         expect_throws<std::invalid_argument>(
             [&] {
               (void)StaticSolver(std::make_shared<SupernodalCholesky>())
                   .solve(convection);
             },
             "the linear solver supernodal_cholesky requires a symmetric "
             "matrix");
         expect_throws<std::invalid_argument>(
             [&] {
               (void)integrate_with(std::make_shared<SupernodalCholesky>(),
                                    problem("-1", "5", "0", "1"));
             },
             "the linear solver supernodal_cholesky requires a symmetric "
             "matrix");
       }},
      {"a pure Neumann problem: both refuse the singular matrix",
       [] {
         const auto neumann = problem("-1", "0", "0", "0", true);
         expect_throws<std::runtime_error>(
             [&] {
               (void)StaticSolver(std::make_shared<MultifrontalLu>())
                   .solve(neumann);
             },
             "multifrontal_lu: the matrix is singular");
         expect_throws<std::runtime_error>(
             [&] {
               (void)StaticSolver(std::make_shared<SupernodalCholesky>())
                   .solve(neumann);
             },
             "supernodal_cholesky: the matrix is");
       }},
      {"StaticSolver reports the solver's threads",
       [] {
         const auto solved =
             StaticSolver(std::make_shared<MultifrontalLu>(
                              MultifrontalLuOptions{.threads = 4}))
                 .solve(problem("-1"));
         check(solved.reports.at(1).metrics.at("linear_threads") == 4,
               "4 threads");
       }},
      {"SupernodalCholesky: 1, 2 and 4 threads",
       [] {
         thread_counts<SupernodalCholesky, SupernodalCholeskyOptions>(
             system_of(problem("-1")));
       }},
      {"MultifrontalLu: 1, 2 and 4 threads",
       [] {
         thread_counts<MultifrontalLu, MultifrontalLuOptions>(
             system_of(problem("-1", "5")));
       }},
      {"0 threads resolve to the processors",
       [] {
         const auto s = system_of(problem("-1"));
         const auto cholesky =
             solve_with(SupernodalCholesky({.threads = 0}), s);
         const auto lu = solve_with(MultifrontalLu({.threads = 0}), s);
         const auto processors =
             static_cast<std::size_t>(std::thread::hardware_concurrency());
         check(cholesky.threads >= 1 && lu.threads >= 1 &&
                   (processors == 0 || (cholesky.threads <= processors &&
                                        lu.threads <= processors)),
               "a count of processors reported");
       }},
      {"solves at once in two threads: each as if alone",
       [] {
         const auto spd = system_of(problem("-1"));
         const auto general = system_of(problem("-1", "5"));
         const auto cholesky_alone = solve_with(SupernodalCholesky(), spd);
         const auto lu_alone = solve_with(MultifrontalLu(), general);
         // One with 1 thread, one with 2, at the same time: the lock keeps
         // each one's BLAS count for its whole run.
         LinearSolveResult cholesky, lu;
         std::thread first([&] {
           for (int i = 0; i < 3; ++i)
             cholesky = solve_with(SupernodalCholesky(), spd);
         });
         std::thread second([&] {
           for (int i = 0; i < 3; ++i)
             lu = solve_with(MultifrontalLu({.threads = 2}), general);
         });
         first.join();
         second.join();
         check(same_bits(cholesky.values, cholesky_alone.values),
               "the 1-thread Cholesky gives the same bits");
         check(relative_difference(lu.values, lu_alone.values) < 1e-12,
               "the LU agrees");
       }},
  });
}
