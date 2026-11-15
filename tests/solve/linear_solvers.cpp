#include "solve_support.hpp"

#include "pde2d/solve/bicgstab.hpp"
#include "pde2d/solve/conjugate_gradient.hpp"
#include "pde2d/solve/simplicial_ldlt.hpp"
#include "pde2d/solve/sparse_lu.hpp"

#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
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
       }},
      {"requirements are declared",
       [] {
         check(!SparseLu().requirements().symmetric_matrix &&
                   !BiCgStab().requirements().symmetric_matrix,
               "LU and BiCGSTAB take any matrix");
         check(ConjugateGradient().requirements().symmetric_matrix &&
                   SimplicialLdlt().requirements().symmetric_matrix,
               "CG and LDL^T need a symmetric one");
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
  });
}
