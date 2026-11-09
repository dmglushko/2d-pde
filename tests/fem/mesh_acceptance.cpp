// The mesh-report invariant is enforced by type: FEM entry points that take
// a MeshResult refuse one that missed its size target, unless the caller
// wraps it in accept_unsatisfied(). Oracles: the TriangleMesh overloads
// (which never check) give the same values bit for bit, and the record of
// an accepted result keeps the unsatisfied report.
#include "discretization/geometry.hpp"
#include "discretization/triangulate.hpp"
#include "fem/evolution.hpp"
#include "fem/mesh_acceptance.hpp"
#include "fem/solver.hpp"
#include "support/check.hpp"
#include <string>
using namespace pde2d::fem;
using namespace pde2d::discretization;
using pde2d_test::check;
using pde2d_test::expect_throws;

namespace {
bool identical(const Vector &a, const Vector &b) {
  return a.size() == b.size() && (a.array() == b.array()).all();
}
Problem poisson() {
  Problem p;
  p.equation.source = Expr(1);
  p.assign(dirichlet(Expr(0)), {"bottom", "right", "top", "left"});
  return p;
}
// A Fixed boundary sampled coarser than the interior target: the request is
// impossible and the report says so.
MeshResult unsatisfied_square() {
  RectangleGeometry square(1, 1);
  MeshOptions o;
  o.boundary.max_edge_length = .5;
  o.refinement.max_edge_length = .2;
  auto result = generate_mesh(square, o);
  check(!result.report.size_satisfied, "the request must be impossible");
  return result;
}

void unsatisfied_results_are_refused() {
  const auto result = unsatisfied_square();
  const auto p = poisson();
  const auto plan = plan_from_user(AssemblyOptions{}, SolverOptions{});
  const std::string fragment = "misses its size target (" +
                               result.report.message +
                               "); check report.size_satisfied or pass "
                               "accept_unsatisfied(result)";
  expect_throws<std::invalid_argument>(
      [&] { (void)solve_stationary(result, p, plan); }, fragment);
  expect_throws<std::invalid_argument>(
      [&] { (void)solve_stationary(result, p); }, fragment);
  expect_throws<std::invalid_argument>(
      [&] { PreparedStationary s(result, p, plan); }, fragment);
  TimeOptions time;
  time.end = .1;
  time.step = .05;
  Problem heat = p;
  heat.equation.m1 = Expr(1);
  expect_throws<std::invalid_argument>(
      [&] { (void)evolve(result, heat, {Expr(0)}, time); }, fragment);

  // Accepted explicitly: the same values as the unchecked mesh overload.
  const auto accepted = solve_stationary(accept_unsatisfied(result), p, plan);
  const auto plain = solve_stationary(result.mesh, p, plan);
  check(identical(accepted.values, plain.values), "same values");
  check(accepted.record.mesh.provenance &&
            !accepted.record.mesh.provenance->report.size_satisfied,
        "the record keeps the unsatisfied report");
  check(!plain.record.mesh.provenance, "a TriangleMesh has no provenance");
  PreparedStationary prepared(accept_unsatisfied(result), p);
  check(identical(prepared.solve().values, plain.values));
  const auto evolved =
      evolve(accept_unsatisfied(result), heat, {Expr(0)}, time);
  const auto evolved_plain = evolve(result.mesh, heat, {Expr(0)}, time);
  check(identical(evolved.frames.back().values,
                  evolved_plain.frames.back().values),
        "accepted evolution");

  // A satisfied result needs no acceptance.
  MeshOptions fine;
  fine.boundary.max_edge_length = .2;
  fine.refinement.max_edge_length = .2;
  const auto good = generate_mesh(RectangleGeometry(1, 1), fine);
  check(good.report.size_satisfied, good.report.message);
  check(identical(solve_stationary(good, p, plan).values,
                  solve_stationary(good.mesh, p, plan).values));
}
} // namespace

int main() {
  return pde2d_test::run(
      {{"unsatisfied_results_are_refused", unsatisfied_results_are_refused}});
}
