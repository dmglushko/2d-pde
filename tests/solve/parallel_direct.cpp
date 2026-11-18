// The parallel direct solvers (SupernodalCholesky, MultifrontalLu) on
// assembled problems (docs/decisions.md, "Parallel direct solvers come from
// SuiteSparse"): their solutions agree with the Eigen direct solvers', they
// refuse what they cannot handle, the Cholesky solves a negative definite
// matrix by factoring -A and warns ("The Cholesky factors -A when A is
// negative definite, with a warning"), and the thread count changes the
// answer at most in the last digits, never with 1 thread. At most 4
// threads.

#include "solve_support.hpp"
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
#include "pde2d/solve/time_method.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <format>
#include <iostream>
#include <limits>
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
using pde2d::test::max_nodal_error;

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

// The start of SupernodalCholesky's warning when it factored -A.
constexpr const char *negative_warning =
    "supernodal_cholesky: the matrix is negative definite (every diagonal "
    "entry is negative";

std::shared_ptr<const pde2d::mesh::Mesh> l_shape() {
  static const auto mesh = pde2d::test::refined_l_shape(0.04);
  return mesh;
}

const std::vector<std::string> sides = {"bottom", "right", "inner", "top",
                                        "left"};

// a (u_xx + u_yy) + b_x u_x + c u + m1 u_t = f on the L-shape, u = 0 on
// the whole boundary (or u_n = 0 with `neumann`), f = 1 + sin(t) x (its
// negative with `negate_f`). a = -1 gives a positive definite matrix, a = 1
// a negative definite one; b_x not 0 makes it non-symmetric; c below minus
// the first eigenvalue of -Laplace (about 9.6) makes it indefinite (for
// a = 1: c above it).
std::shared_ptr<const MeshedProblem>
problem(const char *a, const char *b_x = "0", const char *c = "0",
        const char *m1 = "0", bool neumann = false, bool negate_f = false) {
  Problem p{.equation = {.a_xx = e(a),
                         .a_yy = e(a),
                         .b_x = e(b_x),
                         .c = e(c),
                         .m1 = e(m1),
                         .f = e(negate_f ? "-(1 + sin(t)*x)" : "1 + sin(t)*x")},
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

// -1e6 u_xx - u_yy = 0 on the unit square cut into 128 x 128 cells, with
// Robin conditions u + u_n = g on the horizontal sides and u_n = 0 on the
// vertical ones; u = y is exact, also for P1 (review finding: the relative
// residual of an accurate answer was 1.25e-7, and the Cholesky refused
// it).
std::shared_ptr<const MeshedProblem> contrast_problem() {
  Problem p{.equation = {.a_xx = e("-1000000"), .a_yy = e("-1")}};
  // Bottom: u = 0, u_n = -u_y = -1. Top: u = 1, u_n = 1.
  p.boundary_conditions.emplace(
      "bottom",
      BoundaryCondition{.alpha = e("1"), .beta_n = e("1"), .g = e("-1")});
  p.boundary_conditions.emplace(
      "top", BoundaryCondition{.alpha = e("1"), .beta_n = e("1"), .g = e("2")});
  for (const char *side : {"left", "right"})
    p.boundary_conditions.emplace(
        side, BoundaryCondition{.beta_n = e("1"), .g = e("0")});
  return std::make_shared<const MeshedProblem>(pde2d::test::unit_square(128),
                                               std::move(p));
}

// -u_xx - u_yy = 1 on [0, 1] x [0, 1/128], cut into 2048 x 16 square cells
// (34833 unknowns), with the Robin condition 2e-9 u + u_n = 0 on the left
// side and u_n = 0 on the others (review finding: well-posed, but its
// smallest pivot was below n machine epsilons times the largest, and both
// solvers refused it). The solution is u = 5e8 + x - x^2 / 2; P1 on this
// mesh gives it exactly at the nodes, as the 1D problem does.
std::shared_ptr<const MeshedProblem> weak_robin_problem() {
  Problem p{.equation = {.a_xx = e("-1"), .a_yy = e("-1"), .f = e("1")}};
  p.boundary_conditions.emplace(
      "left",
      BoundaryCondition{.alpha = e("2e-9"), .beta_n = e("1"), .g = e("0")});
  for (const char *side : {"bottom", "right", "top"})
    p.boundary_conditions.emplace(
        side, BoundaryCondition{.beta_n = e("1"), .g = e("0")});
  return std::make_shared<const MeshedProblem>(
      pde2d::test::rectangle(2048, 16, 1.0, 1.0 / 128.0), std::move(p));
}

void print(const std::string &what, const LinearSolveResult &r) {
  std::cout << std::format("{}: residual {:.3g}, backward error {:.3g}, pivot "
                           "ratio {:.3g}\n",
                           what, r.residual, r.backward_error, r.pivot_ratio);
}

// The mean of `v`.
double mean(std::span<const double> v) {
  double sum = 0.0;
  for (const double x : v)
    sum += x;
  return sum / static_cast<double>(v.size());
}

IntegrationResult
integrate_with(std::shared_ptr<const LinearSolver> solver,
               const std::shared_ptr<const MeshedProblem> &p) {
  return Integrator(std::make_shared<ImplicitMidpoint>(),
                    std::make_shared<FixedStep>(FixedStepOptions{.dt = 0.05}),
                    std::move(solver))
      .integrate(p, 0.0, 0.2);
}

// Implicit Euler for time order 1, (B / dt + A) U1 = b(t1) + B U0 / dt
// with the same arithmetic as ImplicitEuler, written here so that a test
// chooses what start() and the steps do with the linear solver (review
// findings on the integrator's log of warnings). With `start_matrix`,
// start() prepares it, solves for a right-hand side of ones and keeps the
// prepared solver and that result; with `concurrent`, every step solves
// its right-hand side twice, from two threads at once.
class TestEuler final : public TimeMethod {
public:
  struct Options {
    std::shared_ptr<const CsrMatrix> start_matrix = nullptr;
    bool concurrent = false;
  };
  explicit TestEuler(Options options) : options_(std::move(options)) {}

  std::string name() const override { return "test_euler"; }
  TimeMethodCapabilities capabilities() const override {
    return {.time_order_1 = true};
  }
  double evaluation_fraction() const override { return 1.0; }
  StepMatrixWeights step_matrix_weights(double dt) const override {
    return {.damping = 1.0 / dt, .stiffness = 1.0};
  }
  StartNeeds start_needs(int) const override {
    return options_.start_matrix ? StartNeeds::static_part
                                 : StartNeeds::nothing;
  }
  void start(const AssembledProblem &, const LinearSolver &linear_solver,
             TimeState &) const override {
    kept = linear_solver.prepare(options_.start_matrix);
    start_result = kept->solve(ones());
  }
  StepSolveInfo advance(const AssembledProblem &problem, double dt,
                        const PreparedLinearSolver &solver,
                        TimeState &state) const override {
    std::vector<double> rhs(problem.right_hand_side().begin(),
                            problem.right_hand_side().end());
    const auto damped = problem.damping().free.multiply(state.values);
    for (std::size_t i = 0; i < rhs.size(); ++i)
      rhs[i] += (1.0 / dt) * damped[i];
    LinearSolveResult solved, other_solved;
    if (options_.concurrent) {
      std::exception_ptr failure;
      std::thread other([&] {
        try {
          other_solved = solver.solve(rhs);
        } catch (...) {
          failure = std::current_exception();
        }
      });
      try {
        solved = solver.solve(rhs);
      } catch (...) {
        other.join();
        throw;
      }
      other.join();
      if (failure)
        std::rethrow_exception(failure);
      check(same_bits(solved.values, other_solved.values),
            "both threads solve to the same bits");
    } else {
      solved = solver.solve(rhs);
    }
    state.values = std::move(solved.values);
    return {.linear_iterations = solved.iterations,
            .residual = solved.residual,
            .linear_threads = solved.threads};
  }

  std::vector<double> ones() const {
    return std::vector<double>(options_.start_matrix->rows(), 1.0);
  }

  // Set by start(): test state, so a const method writes it.
  mutable std::shared_ptr<const PreparedLinearSolver> kept;
  mutable LinearSolveResult start_result;

private:
  Options options_;
};

IntegrationResult
integrate_test_euler(std::shared_ptr<const TimeMethod> method,
                     const std::shared_ptr<const MeshedProblem> &p, double t1) {
  return Integrator(std::move(method),
                    std::make_shared<FixedStep>(FixedStepOptions{.dt = 0.01}),
                    std::make_shared<SupernodalCholesky>())
      .integrate(p, 0.0, t1);
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
             "supernodal_cholesky: the matrix is not positive definite (the "
             "factorization stopped at column");
       }},
      {"Cholesky refuses diagonals of both signs, a zero one, and an "
       "indefinite matrix with a negative diagonal",
       [] {
         // Diagonal entries of both signs: factored as it is, refused at
         // its negative pivot.
         expect_throws<std::runtime_error>(
             [] {
               (void)SupernodalCholesky().prepare(pde2d::test::sparse(
                   {{-2.0, 1.0, 0.0}, {1.0, -3.0, 1.0}, {0.0, 1.0, 4.0}}));
             },
             "supernodal_cholesky: the matrix is not positive definite (the "
             "factorization stopped at column");
         // A zero diagonal entry (missing from the pattern), the others
         // negative: also factored as it is.
         expect_throws<std::runtime_error>(
             [] {
               (void)SupernodalCholesky().prepare(
                   pde2d::test::sparse({{-2.0, 1.0}, {1.0, 0.0}}));
             },
             "supernodal_cholesky: the matrix is not positive definite (the "
             "factorization stopped at column");
         // Every diagonal entry negative, but indefinite (u_xx + u_yy + 40 u
         // = f: c above the first eigenvalue of -Laplace): -A is factored
         // and refused.
         const auto s = system_of(problem("1", "0", "40"));
         bool negative = true;
         for (std::size_t i = 0; i < s.matrix->rows(); ++i)
           negative = negative && s.matrix->entry(i, i) < 0.0;
         check(negative, "every diagonal entry is negative");
         expect_throws<std::runtime_error>(
             [&] { (void)SupernodalCholesky().prepare(s.matrix); },
             "supernodal_cholesky: the matrix is not positive definite, and "
             "not negative definite although its diagonal is negative (the "
             "factorization of -A stopped at column");
       }},
      {"a negative definite matrix: Cholesky factors -A, with a warning",
       [] {
         // u_xx + u_yy = -(1 + sin(t) x) is -u_xx - u_yy = 1 + sin(t) x
         // written with the opposite sign: a = 1 gives a negative definite
         // matrix.
         const auto negative = problem("1", "0", "0", "0", false, true);
         const auto positive = problem("-1");
         const auto cholesky =
             StaticSolver(std::make_shared<SupernodalCholesky>(
                              SupernodalCholeskyOptions{.threads = 2}))
                 .solve(negative);
         const auto ldlt =
             StaticSolver(std::make_shared<SimplicialLdlt>()).solve(positive);
         check(relative_difference(cholesky.solution.values,
                                   ldlt.solution.values) < 1e-12,
               "the same answer as simplicial_ldlt on the positive form");
         const auto &report = cholesky.reports.at(1);
         check(report.stage == "static_solver", "the static solver's report");
         check(report.messages.size() == 1 &&
                   report.messages[0].find(negative_warning) == 0,
               "one warning in the static solver's report");
         check(report.metrics.at("backward_error") < 1e-14,
               "the backward error, measured with A and b as given");
         check(ldlt.reports.at(1).messages.empty(),
               "no message from simplicial_ldlt");
         // The positive form: no message.
         const auto plain = StaticSolver(std::make_shared<SupernodalCholesky>())
                                .solve(positive);
         check(plain.reports.at(1).messages.empty(),
               "no message for a positive definite matrix");
         check(relative_difference(plain.solution.values,
                                   cholesky.solution.values) < 1e-12,
               "the same answer from both forms");
         // Every solve of the prepared solver carries the warning.
         const auto s = system_of(negative);
         const auto prepared = SupernodalCholesky().prepare(s.matrix);
         for (int i = 0; i < 2; ++i) {
           const auto result = prepared->solve(s.right_hand_side);
           check(result.messages.size() == 1 &&
                     result.messages[0].find(negative_warning) == 0,
                 "a warning on every solve");
           check(result.residual < 1e-10 && result.backward_error < 1e-14,
                 "an accurate answer");
           check(result.pivot_ratio > 0.0, "a pivot ratio");
         }
       }},
      {"a negative definite step matrix: the warning once in the integrator's "
       "report",
       [] {
         // The heat equation u_t - u_xx - u_yy = f written as -u_t + u_xx +
         // u_yy = -f: the midpoint step matrix -M1 / dt - K / 2 is negative
         // definite. dt = 0.05 to t1 = 0.22: four steps of 0.05, one of
         // 0.02, so the solver is prepared twice and solves five times.
         const auto negative = problem("1", "0", "0", "-1", false, true);
         const auto positive = problem("-1", "0", "0", "1");
         const auto run = [](std::shared_ptr<const LinearSolver> solver,
                             const std::shared_ptr<const MeshedProblem> &p) {
           return Integrator(
                      std::make_shared<ImplicitMidpoint>(),
                      std::make_shared<FixedStep>(FixedStepOptions{.dt = 0.05}),
                      std::move(solver))
               .integrate(p, 0.0, 0.22);
         };
         const auto cholesky =
             run(std::make_shared<SupernodalCholesky>(), negative);
         const auto ldlt = run(std::make_shared<SimplicialLdlt>(), positive);
         check(cholesky.report.metrics.at("steps") == 5 &&
                   cholesky.report.metrics.at("prepares") == 2,
               "five steps, two prepares");
         check(relative_difference(cholesky.final.values, ldlt.final.values) <
                   1e-12,
               "the same answer as simplicial_ldlt on the positive form");
         check(cholesky.report.messages.size() == 1 &&
                   cholesky.report.messages[0].find(negative_warning) == 0,
               "the warning once in the integrator's report");
         check(ldlt.report.messages.empty(), "no message from simplicial_ldlt");
         check(run(std::make_shared<SupernodalCholesky>(), positive)
                   .report.messages.empty(),
               "no message for a positive definite step matrix");
         // Time order 0 with ImplicitEuler: its start() solves with A at t0,
         // then each step again; the warning still once.
         const auto static_negative = problem("1", "0", "0", "0", false, true);
         const auto euler = Integrator(std::make_shared<ImplicitEuler>(),
                                       std::make_shared<FixedStep>(
                                           FixedStepOptions{.dt = 0.1}),
                                       std::make_shared<SupernodalCholesky>())
                                .integrate(static_negative, 0.0, 0.3);
         check(euler.report.messages.size() == 1 &&
                   euler.report.messages[0].find(negative_warning) == 0,
               "time order 0: the warning once");
       }},
      {"the integrator's log: concurrent solves of one prepared solver",
       [] {
         // A negative definite step matrix: every solve warns. Each step
         // solves twice at once with the same prepared solver (review
         // finding: the log was not guarded and a probe crashed). The
         // answer is ImplicitEuler's, bit for bit.
         const auto negative = problem("1", "0", "0", "-1", false, true);
         const auto concurrent =
             integrate_test_euler(std::make_shared<TestEuler>(
                                      TestEuler::Options{.concurrent = true}),
                                  negative, 0.2);
         check(concurrent.report.metrics.at("steps") == 20, "20 steps");
         check(concurrent.report.messages.size() == 1 &&
                   concurrent.report.messages[0].find(negative_warning) == 0,
               "the warning once");
         const auto euler = Integrator(std::make_shared<ImplicitEuler>(),
                                       std::make_shared<FixedStep>(
                                           FixedStepOptions{.dt = 0.01}),
                                       std::make_shared<SupernodalCholesky>())
                                .integrate(negative, 0.0, 0.2);
         check(same_bits(concurrent.final.values, euler.final.values),
               "the same bits as ImplicitEuler");
       }},
      {"the integrator's log: a warning from start() alone",
       [] {
         // The heat equation in its positive form: the step matrix is
         // positive definite, so the steps give no warning; start() solves
         // with a negative definite matrix, which does (review finding:
         // the earlier test got the same warning from start() and from
         // every step).
         const auto positive = problem("-1", "0", "0", "1");
         const auto negative_matrix =
             system_of(problem("1", "0", "0", "0", false, true)).matrix;
         const auto method = std::make_shared<TestEuler>(
             TestEuler::Options{.start_matrix = negative_matrix});
         const auto result = integrate_test_euler(method, positive, 0.05);
         check(result.report.messages.size() == 1 &&
                   result.report.messages[0].find(negative_warning) == 0,
               "the warning of start()");
         check(integrate_test_euler(
                   std::make_shared<TestEuler>(TestEuler::Options{}), positive,
                   0.05)
                   .report.messages.empty(),
               "no warning from the steps");
       }},
      {"the integrator's log: a prepared solver used after integrate()",
       [] {
         // The method keeps the prepared solver of its start() and solves
         // with it once the Integrator and its run are gone (review
         // finding: the log was a local of integrate(), and AddressSanitizer
         // showed a stack-use-after-return). The solve works and carries
         // its warning; the log it adds to is no report's any more.
         const auto method = std::make_shared<TestEuler>(TestEuler::Options{
             .start_matrix =
                 system_of(problem("1", "0", "0", "0", false, true)).matrix});
         (void)integrate_test_euler(method, problem("-1", "0", "0", "1"), 0.05);
         check(method->kept != nullptr, "start() kept its prepared solver");
         const auto later = method->kept->solve(method->ones());
         check(same_bits(later.values, method->start_result.values),
               "the same answer as in start()");
         check(later.messages.size() == 1 &&
                   later.messages[0].find(negative_warning) == 0,
               "its warning");
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
      {"a coefficient contrast of 1e6: the direct solvers accept u = y",
       [] {
         // The three that check their backward error, and SparseLu (which
         // checks nothing) as a fourth measurement. All four reach a
         // backward error of about 2e-16 and nodal errors of 2e-7 to 1e-6
         // (measured): the condition number of this matrix is about 5e9,
         // so no backward stable solver in double precision does better
         // than about 1e-6. The relative residual of the same answers is
         // about 1e-7, which the old check refused.
         const auto meshed = contrast_problem();
         const auto u = [](double, double y) { return y; };
         for (const auto &solver : std::vector<std::shared_ptr<LinearSolver>>{
                  std::make_shared<SupernodalCholesky>(),
                  std::make_shared<MultifrontalLu>(),
                  std::make_shared<SimplicialLdlt>(),
                  std::make_shared<SparseLu>()}) {
           const auto solved = StaticSolver(solver).solve(meshed);
           const auto &metrics = solved.reports.at(1).metrics;
           const double error =
               max_nodal_error(*meshed->mesh(), solved.solution.values, u);
           std::cout << std::format(
               "contrast, {}: residual {:.3g}, backward error {:.3g}, pivot "
               "ratio {:.3g}, nodal error {:.3g}\n",
               solver->name(), metrics.at("residual"),
               metrics.at("backward_error"), metrics.at("pivot_ratio"), error);
           check(metrics.at("residual") > 1e-8,
                 solver->name() + ": a relative residual above the tolerance");
           check(metrics.at("backward_error") < 1e-14,
                 solver->name() + ": a backward error of rounding");
           check(error < 5e-6,
                 solver->name() + ": nodal error " + std::format("{}", error));
         }
       }},
      {"a very ill-conditioned well-posed problem: both accept it",
       [] {
         // Accepted, with a backward error of rounding and a pivot ratio
         // below n machine epsilons (about 4e-12; the old rule refused
         // that). But the answer is only as good as the conditioning
         // allows: |x| is 5e8, so the rounding of A x (about 1e-7 per row)
         // is amplified by 1 / (second eigenvalue, about 2e-6) in the
         // varying part, and more in the constant, which only the 2e-9
         // Robin term fixes. Measured: Cholesky and LU differ from LDL^T
         // by 13% and 4% (relative to 5e8), from the exact solution by 3%
         // and 11%, and, the constant removed, from LDL^T by 0.04 and
         // 0.01. So the checks are loose: the right magnitude.
         const auto meshed = weak_robin_problem();
         const auto s = system_of(meshed);
         check(s.matrix->rows() == 34833, "34833 unknowns");
         const auto ldlt = solve_with(SimplicialLdlt(), s);
         print("weak Robin, simplicial_ldlt", ldlt);
         const auto &nodes = meshed->mesh()->nodes;
         for (const auto &result : {solve_with(SupernodalCholesky(), s),
                                    solve_with(MultifrontalLu(), s)}) {
           print("weak Robin", result);
           check(result.pivot_ratio > 0.0 &&
                     result.pivot_ratio <
                         34833 * std::numeric_limits<double>::epsilon(),
                 "a pivot ratio below the rule that used to refuse it");
           check(result.backward_error < 1e-14, "backward error of rounding");
           const double difference =
               relative_difference(result.values, ldlt.values);
           double error = 0.0;
           for (std::size_t i = 0; i < nodes.size(); ++i) {
             const double x = nodes[i].x;
             error = std::max(
                 error, std::abs(result.values[i] - (5e8 + x - x * x / 2)));
           }
           std::cout << std::format("  relative difference to LDL^T {:.3g}, "
                                    "relative error {:.3g}\n",
                                    difference, error / 5e8);
           check(difference < 0.5, "the magnitude of simplicial_ldlt's");
           check(error / 5e8 < 0.5, "the magnitude of the exact solution");
         }
       }},
      {"a pure Neumann problem with a compatible load: one of its solutions",
       [] {
         // -u_xx - u_yy = x - 1/2 (mean zero) with u_n = 0 on the unit
         // square: the solutions are x^2 / 4 - x^3 / 6 + C. The load sums
         // to zero up to rounding (the rule integrates a linear f exactly),
         // so the singular system is compatible. Each solver returns one
         // of its solutions: the residual is rounding and, the constant
         // removed, the values are the P1 solution (within its
         // discretization error of the exact one); the pivot ratio says
         // the matrix is singular.
         Problem p{
             .equation = {.a_xx = e("-1"), .a_yy = e("-1"), .f = e("x - 1/2")}};
         for (const char *side : {"bottom", "right", "top", "left"})
           p.boundary_conditions.emplace(
               side, BoundaryCondition{.beta_n = e("1"), .g = e("0")});
         const auto meshed = std::make_shared<const MeshedProblem>(
             pde2d::test::unit_square(64), std::move(p));
         const auto s = system_of(meshed);
         // Every node is free, in node order.
         check(s.matrix->rows() == 4225, "4225 unknowns");
         std::vector<double> exact;
         for (const auto &node : meshed->mesh()->nodes)
           exact.push_back(node.x * node.x / 4 - node.x * node.x * node.x / 6);
         const double exact_mean = mean(exact);
         for (const auto &result : {solve_with(SupernodalCholesky(), s),
                                    solve_with(MultifrontalLu(), s)}) {
           print("pure Neumann", result);
           // n = 4225: below the old rule's n machine epsilons (9.4e-13).
           check(result.pivot_ratio > 0.0 &&
                     result.pivot_ratio <
                         4225 * std::numeric_limits<double>::epsilon(),
                 "a tiny pivot ratio");
           check(result.residual < 1e-10, "a solution: rounding residual");
           check(result.backward_error < 1e-14, "backward error of rounding");
           const double shift = mean(result.values);
           double error = 0.0;
           for (std::size_t i = 0; i < exact.size(); ++i)
             error = std::max(error, std::abs((result.values[i] - shift) -
                                              (exact[i] - exact_mean)));
           std::cout << std::format("  error up to the constant {:.3g}\n",
                                    error);
           check(error < 1e-4, "the solution up to its constant");
         }
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
