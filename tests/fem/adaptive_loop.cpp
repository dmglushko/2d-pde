// The solve -> estimate -> mark -> refine -> transfer/re-solve loop on a
// stationary problem with a curved boundary, composed from public seams
// only. The transferred solution seeds the next (conjugate gradient) solve
// as its initial guess. Oracles: the exact H1-seminorm error of a
// manufactured solution must decrease in every iteration, and the seeded
// solve must agree with an unseeded solve within the solver tolerance. No
// performance claim is made beyond reporting the iteration counts.
#include "discretization/geometry.hpp"
#include "discretization/triangulate.hpp"
#include "fem/error_norms.hpp"
#include "fem/estimation.hpp"
#include "fem/solver.hpp"
#include "fem/transfer.hpp"
#include "support/check.hpp"
#include "support/check_eigen.hpp"
#include <cmath>
#include <iostream>
using namespace pde2d::fem;
using namespace pde2d::discretization;
using pde2d_test::check;

namespace {
void adaptive_loop_reduces_error() {
  // A sharp peak inside the unit disk: -Laplace(u) = f, u = exact on the
  // circle.
  const Expr exact = exp(-60 * ((x - .4) * (x - .4) + (y + .3) * (y + .3)));
  Problem p;
  p.equation.source = -(diff(diff(exact, x), x) + diff(diff(exact, y), y));
  p.assign(dirichlet(exact), {"boundary"});

  MeshOptions options;
  options.boundary_treatment = BoundaryTreatment::RefineOnCurves;
  options.boundary.max_curve_deviation = .02;
  options.refinement.max_edge_length = .3;
  auto mesh = generate_mesh(CircleGeometry(1.0), options);
  check(mesh.report.size_satisfied, mesh.report.message);

  SolverOptions cg;
  cg.method = LinearSolver::ConjugateGradient;
  const auto plan = plan_from_user(AssemblyOptions{8}, cg);
  auto u = solve_stationary(mesh.mesh, p, plan).values;
  double error = error_norms(mesh.mesh, u, exact).h1_seminorm;
  const double first_error = error;
  for (int iteration = 0; iteration < 8; ++iteration) {
    auto estimate = estimate_error(mesh.mesh, p, u);
    auto marked = mark_bulk(estimate.indicators, .6);
    auto refined = refine_mesh(mesh, marked, mesh.options);
    check(refined.report.size_satisfied, refined.report.message);
    check(refined.report.marked->bisected_edges > 0);
    // The previous solution moved to the new mesh; boundary nodes on the
    // circle lie outside the old chords by at most the curve deviation.
    TransferOptions transfer;
    transfer.max_outside_distance = options.boundary.max_curve_deviation;
    auto guess = transfer_field(mesh.mesh, u, refined.mesh, transfer);
    check(guess.values.allFinite());
    // One preparation serves the seeded and the unseeded solve.
    PreparedStationary stationary(refined.mesh, p, plan);
    const auto seeded = stationary.solve(0, guess.values);
    const auto cold = stationary.solve(0);
    check(seeded.report.initial_guess == InitialGuess::Used,
          "the transferred field must seed the solve");
    check(stationary.statistics().solver.preparations == 1 &&
              stationary.statistics().solver_reuses == 1,
          "the second solve on one mesh must reuse the preparation");
    pde2d_test::near_vector(seeded.values, cold.values, 1e-7);
    auto next = seeded.values;
    const double next_error =
        error_norms(refined.mesh, next, exact).h1_seminorm;
    std::cout << "iteration " << iteration << ": nodes "
              << mesh.mesh.nodes.size() << " -> " << refined.mesh.nodes.size()
              << ", marked " << marked.size() << ", estimate "
              << estimate.global << ", H1 error " << error << " -> "
              << next_error << ", change from transferred "
              << (next - guess.values).lpNorm<Eigen::Infinity>()
              << ", CG iterations seeded " << seeded.report.iterations
              << " / unseeded " << cold.report.iterations << '\n';
    check(next_error < error, "adaptive iteration did not reduce the error");
    mesh = std::move(refined);
    u = std::move(next);
    error = next_error;
  }
  check(error < .5 * first_error,
        "eight adaptive iterations gained too little");
  for (auto [a, b] : mesh.mesh.boundary_edges) {
    (void)b;
    auto q = mesh.mesh.nodes[a];
    check(std::abs(std::hypot(q.x, q.y) - 1) < 1e-12);
  }
}
} // namespace

int main() {
  return pde2d_test::run(
      {{"adaptive_loop_reduces_error", adaptive_loop_reduces_error}});
}
