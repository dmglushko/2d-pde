#include "solve_support.hpp"

#include "pde2d/solve/bicgstab.hpp"
#include "pde2d/solve/conjugate_gradient.hpp"
#include "pde2d/solve/multifrontal_lu.hpp"
#include "pde2d/solve/simplicial_ldlt.hpp"
#include "pde2d/solve/sparse_lu.hpp"
#include "pde2d/solve/supernodal_cholesky.hpp"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// Small systems with known solutions: b is computed by hand from a chosen
// x, so x is the oracle.

using namespace pde2d::solve;
using pde2d::test::check;
using pde2d::test::Dense;
using pde2d::test::expect_throws;
using pde2d::test::near;
using pde2d::test::sparse;

namespace {

void expect_solution(const LinearSolveResult &result,
                     const std::vector<double> &x, double tolerance) {
  check(result.values.size() == x.size(), "one value per row");
  for (std::size_t i = 0; i < x.size(); ++i)
    near(result.values[i], x[i], tolerance, "solution entry");
}

// Symmetric positive definite: x = (1, 2, 3).
const Dense spd = {{4, -1, 0}, {-1, 4, -1}, {0, -1, 4}};
const std::vector<double> spd_x = {1, 2, 3};
const std::vector<double> spd_b = {2, 4, 10};

// Not symmetric: x1 = (1, -1, 2, 0.5) and x2 = (0, 1, 0, -1).
const Dense nonsymmetric = {
    {2, 1, 0, 0}, {0, 3, 1, 0}, {1, 0, 4, 1}, {0, 2, 0, 5}};
const std::vector<double> x1 = {1, -1, 2, 0.5};
const std::vector<double> b1 = {1, -1, 9.5, 0.5};
const std::vector<double> x2 = {0, 1, 0, -1};
const std::vector<double> b2 = {1, 3, -1, -3};

void solves_known_systems(const LinearSolver &solver) {
  const auto a = solver.prepare(sparse(spd));
  const auto r = a->solve(spd_b);
  expect_solution(r, spd_x, 1e-12);
  check(r.residual < 1e-12, "small residual");

  // One prepared solver, several right-hand sides, in any order.
  const auto n = solver.prepare(sparse(nonsymmetric));
  const auto first = n->solve(b1);
  expect_solution(first, x1, 1e-12);
  expect_solution(n->solve(b2), x2, 1e-12);
  const auto again = n->solve(b1);
  check(again.values == first.values, "a solve does not change the solver");

  // A zero right-hand side gives zero.
  expect_solution(n->solve(std::vector<double>(4, 0.0)), {0, 0, 0, 0}, 0.0);
}

// For the solvers that require a symmetric matrix: a symmetric positive
// definite one, several right-hand sides.
void solves_symmetric_systems(const LinearSolver &solver) {
  // x = (1, -2, 0.5, 3) for the 1D Laplacian with a shift.
  const Dense laplacian = {
      {3, -1, 0, 0}, {-1, 3, -1, 0}, {0, -1, 3, -1}, {0, 0, -1, 3}};
  const std::vector<double> x = {1, -2, 0.5, 3};
  const std::vector<double> b = {5, -7.5, 0.5, 8.5};
  const auto prepared = solver.prepare(sparse(laplacian));
  const auto first = prepared->solve(b);
  expect_solution(first, x, 1e-12);
  check(first.residual < 1e-12, "small residual");
  expect_solution(solver.prepare(sparse(spd))->solve(spd_b), spd_x, 1e-12);
  check(prepared->solve(b).values == first.values,
        "a solve does not change the solver");
  expect_solution(prepared->solve(std::vector<double>(4, 0.0)), {0, 0, 0, 0},
                  0.0);
}

// Symmetric but indefinite, with a zero diagonal: x = (0, 1) solves it,
// but CG breaks down at once (p^T A p = 0 for p = b) and LDL^T without
// pivoting meets a zero pivot. Neither may return an answer.
const Dense indefinite = {{0, 1}, {1, 0}};
const std::vector<double> indefinite_b = {1, 0};

// |b - A x| / (|A| |x| + |b|) in the infinity norm, computed here in
// long double from the dense matrix.
double backward_error(const Dense &a, const std::vector<double> &b,
                      const std::vector<double> &x) {
  long double r = 0, norm_a = 0, norm_x = 0, norm_b = 0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    long double product = 0, row_sum = 0;
    for (std::size_t j = 0; j < a[i].size(); ++j) {
      product += static_cast<long double>(a[i][j]) * x[j];
      row_sum += std::abs(static_cast<long double>(a[i][j]));
    }
    r = std::max(r, std::abs(b[i] - product));
    norm_a = std::max(norm_a, row_sum);
    norm_x = std::max(norm_x, std::abs(static_cast<long double>(x[i])));
    norm_b = std::max(norm_b, std::abs(static_cast<long double>(b[i])));
  }
  return static_cast<double>(r / (norm_a * norm_x + norm_b));
}

// `dense` with an explicitly stored zero at (row, column).
std::shared_ptr<const pde2d::fem::CsrMatrix>
with_explicit_zero(const Dense &dense, std::size_t row, std::size_t column) {
  auto pattern = std::make_shared<pde2d::fem::SparsityPattern>();
  pattern->rows = pattern->columns = dense.size();
  pattern->row_offsets.push_back(0);
  pde2d::fem::CsrMatrix matrix;
  for (std::size_t i = 0; i < dense.size(); ++i) {
    for (std::size_t j = 0; j < dense[i].size(); ++j)
      if (dense[i][j] != 0.0 || (i == row && j == column)) {
        pattern->column_indices.push_back(static_cast<std::ptrdiff_t>(j));
        matrix.values.push_back(dense[i][j]);
      }
    pattern->row_offsets.push_back(
        static_cast<std::ptrdiff_t>(pattern->column_indices.size()));
  }
  matrix.pattern = std::move(pattern);
  return std::make_shared<const pde2d::fem::CsrMatrix>(std::move(matrix));
}

const SparseLu sparse_lu;
const SimplicialLdlt simplicial_ldlt;
const SupernodalCholesky supernodal_cholesky;
const MultifrontalLu multifrontal_lu;
const ConjugateGradient conjugate_gradient({.tolerance = 1e-14});
const BiCgStab bicgstab({.tolerance = 1e-14});

void rejects_bad_input(const LinearSolver &solver) {
  expect_throws<std::invalid_argument>([&] { (void)solver.prepare(nullptr); },
                                       "null");
  expect_throws<std::invalid_argument>(
      [&] { (void)solver.prepare(sparse({{1, 2}}, 2)); }, "not square");
  const auto prepared = solver.prepare(sparse(spd));
  expect_throws<std::invalid_argument>(
      [&] { (void)prepared->solve(std::vector<double>{1, 2}); },
      "2 values for 3 rows");
  // An empty system has an empty solution.
  const auto empty = solver.prepare(sparse(Dense{}, 0));
  check(empty->solve(std::vector<double>{}).values.empty(), "empty solution");
}

} // namespace

int main() {
  return pde2d::test::run({
      {"SparseLu solves symmetric and non-symmetric systems",
       [] {
         solves_known_systems(SparseLu());
         const auto r = SparseLu().prepare(sparse(spd))->solve(spd_b);
         check(r.iterations == 0, "a direct solver has no iterations");
       }},
      {"BiCgStab solves symmetric and non-symmetric systems",
       [] {
         solves_known_systems(BiCgStab({.tolerance = 1e-14}));
         const auto r = BiCgStab().prepare(sparse(nonsymmetric))->solve(b1);
         check(r.iterations >= 1, "it iterates");
         check(r.residual <= 1e-10, "it reaches its tolerance");
       }},
      {"SparseLu throws for a singular matrix",
       [] {
         expect_throws<std::runtime_error>(
             [] { (void)SparseLu().prepare(sparse({{1, 2}, {2, 4}})); },
             "sparse_lu: the matrix is singular");
         // A row without entries.
         expect_throws<std::runtime_error>(
             [] { (void)SparseLu().prepare(sparse({{1, 0}, {0, 0}})); },
             "singular");
       }},
      {"BiCgStab throws when it does not converge",
       [] {
         // A non-symmetric 30 x 30 system that needs more than 2
         // iterations.
         Dense a(30, std::vector<double>(30, 0.0));
         for (std::size_t i = 0; i < 30; ++i) {
           a[i][i] = 2.0;
           if (i + 1 < 30)
             a[i][i + 1] = -1.5;
           if (i > 0)
             a[i][i - 1] = -0.4;
         }
         const auto prepared =
             BiCgStab({.tolerance = 1e-14, .max_iterations = 2})
                 .prepare(sparse(a));
         expect_throws<std::runtime_error>(
             [&] { (void)prepared->solve(std::vector<double>(30, 1.0)); },
             "bicgstab: did not converge in 2 iterations");
       }},
      {"BiCgStab validates its options",
       [] {
         for (const double tolerance :
              {0.0, -1.0, 1.0, std::numeric_limits<double>::quiet_NaN()})
           expect_throws<std::invalid_argument>(
               [&] { BiCgStab({.tolerance = tolerance}); }, "tolerance");
         expect_throws<std::invalid_argument>(
             [] { BiCgStab({.max_iterations = 0}); }, "max_iterations");
       }},
      {"every solver rejects bad input",
       [] {
         rejects_bad_input(SparseLu());
         rejects_bad_input(BiCgStab());
         rejects_bad_input(ConjugateGradient());
         rejects_bad_input(SimplicialLdlt());
         rejects_bad_input(SupernodalCholesky());
         rejects_bad_input(MultifrontalLu());
       }},
      {"requirements are declared",
       [] {
         check(!SparseLu().requirements().symmetric_matrix &&
                   !BiCgStab().requirements().symmetric_matrix,
               "LU and BiCGSTAB take any matrix");
         check(ConjugateGradient().requirements().symmetric_matrix &&
                   SimplicialLdlt().requirements().symmetric_matrix,
               "CG and LDL^T need a symmetric one");
         check(SupernodalCholesky().requirements().symmetric_matrix &&
                   !MultifrontalLu().requirements().symmetric_matrix,
               "Cholesky needs a symmetric matrix, the multifrontal LU not");
       }},
      {"ConjugateGradient solves symmetric positive definite systems",
       [] {
         solves_symmetric_systems(ConjugateGradient({.tolerance = 1e-14}));
         const auto r = ConjugateGradient().prepare(sparse(spd))->solve(spd_b);
         check(r.iterations >= 1, "it iterates");
         check(r.residual <= 1e-10, "it reaches its tolerance");
       }},
      {"SimplicialLdlt solves symmetric systems",
       [] {
         solves_symmetric_systems(SimplicialLdlt());
         const auto r = SimplicialLdlt().prepare(sparse(spd))->solve(spd_b);
         check(r.iterations == 0, "a direct solver has no iterations");
         // Indefinite, but with nonzero pivots: LDL^T handles it.
         const auto i = SimplicialLdlt()
                            .prepare(sparse({{2, 1}, {1, -3}}))
                            ->solve(std::vector<double>{3, -2});
         expect_solution(i, {1, 1}, 1e-14);
       }},
      {"CG and LDL^T throw on a matrix they cannot handle",
       [] {
         expect_throws<std::runtime_error>(
             [] {
               (void)ConjugateGradient()
                   .prepare(sparse(indefinite))
                   ->solve(indefinite_b);
             },
             "conjugate_gradient: did not converge");
         expect_throws<std::runtime_error>(
             [] { (void)SimplicialLdlt().prepare(sparse(indefinite)); },
             "simplicial_ldlt: the factorization failed");
         expect_throws<std::runtime_error>(
             [] { (void)SimplicialLdlt().prepare(sparse({{1, 2}, {2, 4}})); },
             "simplicial_ldlt: the factorization failed");
         // The 1D Laplacian of size 30 needs more than 2 iterations.
         Dense a(30, std::vector<double>(30, 0.0));
         for (std::size_t i = 0; i < 30; ++i) {
           a[i][i] = 2.0;
           if (i + 1 < 30)
             a[i][i + 1] = a[i + 1][i] = -1.0;
         }
         const auto prepared =
             ConjugateGradient({.tolerance = 1e-14, .max_iterations = 2})
                 .prepare(sparse(a));
         expect_throws<std::runtime_error>(
             [&] { (void)prepared->solve(std::vector<double>(30, 1.0)); },
             "conjugate_gradient: did not converge in 2 iterations");
       }},
      {"ConjugateGradient validates its options",
       [] {
         for (const double tolerance :
              {0.0, -1.0, 1.0, std::numeric_limits<double>::quiet_NaN()})
           expect_throws<std::invalid_argument>(
               [&] { ConjugateGradient({.tolerance = tolerance}); },
               "conjugate_gradient: tolerance");
         expect_throws<std::invalid_argument>(
             [] { ConjugateGradient({.max_iterations = 0}); },
             "max_iterations");
       }},
      {"MultifrontalLu solves symmetric and non-symmetric systems",
       [] {
         solves_known_systems(MultifrontalLu());
         const auto r = MultifrontalLu().prepare(sparse(spd))->solve(spd_b);
         check(r.iterations == 0, "a direct solver has no iterations");
         check(r.threads == 1, "1 thread by default");
         // It pivots: a zero diagonal is no problem.
         expect_solution(
             MultifrontalLu().prepare(sparse(indefinite))->solve(indefinite_b),
             {0, 1}, 0.0);
       }},
      {"MultifrontalLu throws for an exact zero pivot",
       [] {
         expect_throws<std::runtime_error>(
             [] { (void)MultifrontalLu().prepare(sparse({{1, 2}, {2, 4}})); },
             "multifrontal_lu: the matrix is singular (a pivot is exactly "
             "zero)");
         // An explicitly stored zero as the only entry of column 0: a
         // singleton whose pivot is zero.
         expect_throws<std::runtime_error>(
             [] {
               (void)MultifrontalLu().prepare(
                   with_explicit_zero({{0, 1}, {0, 1}}, 0, 0));
             },
             "multifrontal_lu: the matrix is singular (a pivot is exactly "
             "zero)");
         // A row without entries.
         expect_throws<std::runtime_error>(
             [] { (void)MultifrontalLu().prepare(sparse({{1, 0}, {0, 0}})); },
             "multifrontal_lu: the matrix is singular (a pivot is exactly "
             "zero): row 1 of 2 has no entry that is not zero");
         // A row whose only entry is an explicitly stored zero, on the
         // diagonal: a singleton whose row scale factor is zero too (ParU
         // alone accepted it, with a pivot 0 / 0; review finding). The
         // other row has a negative entry, so the scale is |a_00| = 1.
         expect_throws<std::runtime_error>(
             [] {
               (void)MultifrontalLu().prepare(
                   with_explicit_zero({{-1, 0}, {0, 0}}, 1, 1));
             },
             "multifrontal_lu: the matrix is singular (a pivot is exactly "
             "zero): row 1 of 2 has no entry that is not zero");
         // A zero row among larger ones, stored zeros only.
         expect_throws<std::runtime_error>(
             [] {
               (void)MultifrontalLu().prepare(
                   with_explicit_zero({{2, 1, 0}, {0, 0, 0}, {0, 1, 3}}, 1, 0));
             },
             "row 1 of 3 has no entry that is not zero");
       }},
      {"SupernodalCholesky solves symmetric positive definite systems",
       [] {
         solves_symmetric_systems(SupernodalCholesky());
         const auto r = SupernodalCholesky().prepare(sparse(spd))->solve(spd_b);
         check(r.iterations == 0, "a direct solver has no iterations");
         check(r.threads == 1, "1 thread by default");
       }},
      {"SupernodalCholesky throws on a matrix that is not positive definite",
       [] {
         // Indefinite with a zero diagonal, indefinite with nonzero pivots
         // (which LDL^T handles), singular.
         for (const Dense &a :
              {indefinite, Dense{{2, 1}, {1, -3}}, Dense{{1, 2}, {2, 4}}})
           expect_throws<std::runtime_error>(
               [&] { (void)SupernodalCholesky().prepare(sparse(a)); },
               "supernodal_cholesky: the matrix is not positive definite (the "
               "factorization stopped at column");
         // Positive semidefinite (singular), with an exact zero pivot.
         expect_throws<std::runtime_error>(
             [] {
               (void)SupernodalCholesky().prepare(
                   sparse({{1, -1, 0}, {-1, 2, -1}, {0, -1, 1}}));
             },
             "at a pivot that is zero or negative); for an indefinite matrix "
             "use simplicial_ldlt or a LU solver");
         // A negative diagonal: -A is factored. Negative semidefinite
         // (singular) and indefinite (-A has a negative pivot): refused.
         for (const Dense &a : {Dense{{-1, 1, 0}, {1, -2, 1}, {0, 1, -1}},
                                Dense{{-1, 2}, {2, -1}}})
           expect_throws<std::runtime_error>(
               [&] { (void)SupernodalCholesky().prepare(sparse(a)); },
               "supernodal_cholesky: the matrix is not positive definite, and "
               "not negative definite although its diagonal is negative (the "
               "factorization of -A stopped at column");
       }},
      {"SupernodalCholesky solves a negative definite system, with a warning",
       [] {
         // -A is factored: the same x as the Eigen LU on A, a warning on
         // every solve, the residual measured with A and b.
         const Dense a = {{-4, 1, 0}, {1, -4, 1}, {0, 1, -4}};
         const std::vector<double> b = {1, -2, 3};
         const auto prepared = SupernodalCholesky().prepare(sparse(a));
         const auto lu = SparseLu().prepare(sparse(a))->solve(b);
         for (int i = 0; i < 2; ++i) {
           const auto r = prepared->solve(b);
           for (std::size_t k = 0; k < 3; ++k)
             near(r.values[k], lu.values[k], 1e-14, "the same x");
           check(r.residual < 1e-14, "the residual with A and b");
           check(r.messages.size() == 1 &&
                     r.messages[0].starts_with(
                         "supernodal_cholesky: the matrix is negative "
                         "definite") &&
                     r.messages[0].find("-A was factored") != std::string::npos,
                 "the warning on every solve");
         }
         // No message for the positive definite form, nor from the other
         // solvers.
         Dense minus_a = a;
         for (auto &row : minus_a)
           for (double &value : row)
             value = -value;
         check(SupernodalCholesky()
                   .prepare(sparse(minus_a))
                   ->solve(b)
                   .messages.empty(),
               "no message for a positive definite matrix");
         check(lu.messages.empty(), "no message from sparse_lu");
       }},
      {"every solver reports its backward error",
       [] {
         const auto check_backward_error = [](const LinearSolver &solver,
                                              const Dense &a,
                                              const std::vector<double> &b) {
           // An accurate answer: rounding, both as reported and as
           // computed here (they differ, since r is itself rounding).
           const auto r = solver.prepare(sparse(a))->solve(b);
           check(r.backward_error >= 0.0 && r.backward_error < 1e-14 &&
                     backward_error(a, b, r.values) < 1e-14,
                 solver.name() + ": rounding");
         };
         for (const LinearSolver *solver :
              std::initializer_list<const LinearSolver *>{
                  &sparse_lu, &simplicial_ldlt, &supernodal_cholesky,
                  &multifrontal_lu, &conjugate_gradient, &bicgstab})
           check_backward_error(*solver, spd, spd_b);
         // A loose tolerance: an iterative answer with a visible backward
         // error (about 0.05), measured on the scaled system it solved.
         Dense a(30, std::vector<double>(30, 0.0));
         for (std::size_t i = 0; i < 30; ++i) {
           a[i][i] = 2.0;
           if (i + 1 < 30)
             a[i][i + 1] = -1.5;
           if (i > 0)
             a[i][i - 1] = -0.4;
         }
         const std::vector<double> b(30, 1e-3);
         const auto r =
             BiCgStab({.tolerance = 0.5}).prepare(sparse(a))->solve(b);
         const double expected = backward_error(a, b, r.values);
         check(expected > 1e-12, "a visible backward error");
         near(r.backward_error, expected, 1e-6 * expected,
              "bicgstab: the infinity-norm formula");
         // Above the tolerance: refused. LDL^T without pivoting on
         // [[1e-12, 1], [1, 1]] (pivots 1e-12 and 1 - 1e12) loses about
         // 1e-16 * 1e12 of accuracy: a backward error of about 2.79e-5
         // (measured).
         {
           const Dense tiny_pivot = {{1e-12, 1}, {1, 1}};
           const std::vector<double> tiny_b = {1.234567, 0.7654321};
           const auto tiny = SimplicialLdlt({.residual_tolerance = 1e-3})
                                 .prepare(sparse(tiny_pivot))
                                 ->solve(tiny_b);
           near(tiny.backward_error, 2.79e-5, 0.01e-5,
                "simplicial_ldlt: the backward error of a tiny pivot");
           near(tiny.backward_error,
                backward_error(tiny_pivot, tiny_b, tiny.values),
                1e-3 * tiny.backward_error, "as computed here");
           expect_throws<std::runtime_error>(
               [&] {
                 (void)SimplicialLdlt()
                     .prepare(sparse(tiny_pivot))
                     ->solve(tiny_b);
               },
               "simplicial_ldlt: the backward error 2.79");
           expect_throws<std::runtime_error>(
               [&] {
                 (void)SimplicialLdlt()
                     .prepare(sparse(tiny_pivot))
                     ->solve(tiny_b);
               },
               "is above the tolerance 1e-08 (residual_tolerance");
         }
         // The Cholesky of a positive definite matrix and the LU with
         // threshold pivoting are backward stable: no small matrix gives
         // them a large backward error. Their check is shown with a
         // tolerance below the rounding of an accurate answer instead.
         {
           const Dense small = {{4.1, 1.3, 0}, {1.3, 3.7, 0.9}, {0, 0.9, 2.3}};
           const std::vector<double> small_b = {0.123, 0.789, 0.456};
           const auto cholesky =
               supernodal_cholesky.prepare(sparse(small))->solve(small_b);
           const auto lu =
               multifrontal_lu.prepare(sparse(small))->solve(small_b);
           check(cholesky.backward_error > 1e-300 && lu.backward_error > 1e-300,
                 "rounding: a backward error above 1e-300");
           expect_throws<std::runtime_error>(
               [&] {
                 (void)SupernodalCholesky({.residual_tolerance = 1e-300})
                     .prepare(sparse(small))
                     ->solve(small_b);
               },
               "supernodal_cholesky: the backward error");
           expect_throws<std::runtime_error>(
               [&] {
                 (void)MultifrontalLu({.residual_tolerance = 1e-300})
                     .prepare(sparse(small))
                     ->solve(small_b);
               },
               "multifrontal_lu: the backward error");
           expect_throws<std::runtime_error>(
               [&] {
                 (void)MultifrontalLu({.residual_tolerance = 1e-300})
                     .prepare(sparse(small))
                     ->solve(small_b);
               },
               "is above the tolerance 1e-300 (residual_tolerance");
         }
         // b = 0: x = 0, nothing to measure.
         check(multifrontal_lu.prepare(sparse(spd))
                       ->solve(std::vector<double>(3, 0.0))
                       .backward_error == 0.0,
               "0 for b = 0");
       }},
      {"LinearSolveResult keeps its positional order",
       [] {
  // The members that existed before backward_error, pivot_ratio and
  // messages keep their order: a positional initialization written
  // then means the same (review finding).
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
         const LinearSolveResult r{{1.0, 2.0}, 3, 0.5, 4};
#pragma GCC diagnostic pop
         check(r.values == std::vector<double>{1.0, 2.0} && r.iterations == 3 &&
                   r.residual == 0.5 && r.threads == 4,
               "values, iterations, residual, threads");
         check(r.backward_error == 0.0 && r.pivot_ratio == 0.0 &&
                   r.messages.empty(),
               "the new members keep their defaults");
       }},
      {"the pivot ratio: measured by the factorizations that have pivots",
       [] {
         // Diagonal: the pivots are 2 and 8 in any order.
         const Dense diagonal = {{2, 0}, {0, 8}};
         const std::vector<double> b = {1, 1};
         for (const LinearSolver *solver :
              {static_cast<const LinearSolver *>(&simplicial_ldlt),
               static_cast<const LinearSolver *>(&supernodal_cholesky)})
           near(solver->prepare(sparse(diagonal))->solve(b).pivot_ratio, 0.25,
                1e-15, solver->name());
         // ParU's pivots are those of the matrix with each row divided by
         // its largest entry. Only singletons: every pivot is 1.
         near(multifrontal_lu.prepare(sparse(diagonal))->solve(b).pivot_ratio,
              1.0, 1e-15, "multifrontal_lu, singletons only");
         // Column 0 is a singleton with pivot 1 / 3; the rest, [[2, 1],
         // [1, 2]] scaled, is a front with pivots 1 and 3/4. Without the
         // singleton the ratio would be 3/4.
         near(
             multifrontal_lu.prepare(sparse({{1, 3, 0}, {0, 2, 1}, {0, 1, 2}}))
                 ->solve(std::vector<double>{1, 1, 1})
                 .pivot_ratio,
             1.0 / 3.0, 1e-15, "multifrontal_lu, a singleton and a front");
         // Not measured.
         for (const LinearSolver *solver :
              {static_cast<const LinearSolver *>(&sparse_lu),
               static_cast<const LinearSolver *>(&conjugate_gradient),
               static_cast<const LinearSolver *>(&bicgstab)})
           check(solver->prepare(sparse(spd))->solve(spd_b).pivot_ratio == 0.0,
                 solver->name() + ": 0, not measured");
       }},
      {"the parallel direct solvers validate their options",
       [] {
         for (const double tolerance :
              {0.0, -1.0, 1.0, std::numeric_limits<double>::quiet_NaN()}) {
           expect_throws<std::invalid_argument>(
               [&] { SupernodalCholesky({.residual_tolerance = tolerance}); },
               "supernodal_cholesky: residual_tolerance must be in (0, 1)");
           expect_throws<std::invalid_argument>(
               [&] { MultifrontalLu({.residual_tolerance = tolerance}); },
               "multifrontal_lu: residual_tolerance must be in (0, 1)");
         }
         const std::size_t too_many = 1025;
         const std::size_t negative = std::numeric_limits<std::size_t>::max();
         for (const std::size_t threads : {too_many, negative}) {
           expect_throws<std::invalid_argument>(
               [&] { SupernodalCholesky({.threads = threads}); },
               "supernodal_cholesky: threads must be at most 1024");
           expect_throws<std::invalid_argument>(
               [&] { MultifrontalLu({.threads = threads}); },
               "multifrontal_lu: threads must be at most 1024");
         }
         check(SupernodalCholesky({.threads = 1024}).options().threads ==
                       1024 &&
                   MultifrontalLu({.threads = 0}).options().threads == 0,
               "1024 and 0 are accepted");
       }},
  });
}
