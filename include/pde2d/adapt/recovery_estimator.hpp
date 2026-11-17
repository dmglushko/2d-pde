#pragma once

#include "pde2d/adapt/error_estimator.hpp"

namespace pde2d::adapt {

// The norm in which a RecoveryEstimator measures G - grad u_h.
enum class RecoveryNorm {
  // The energy norm of A: integral of e . |A| e. The equation's A may be
  // negative definite (-Laplace u = f is written with a_xx = a_yy = -1),
  // so its sign is taken away: |e . A e|, which is e . |A| e when A is
  // definite or semidefinite at the point. Where A is indefinite
  // (a_xx a_yy - a_xy^2 / 4 < 0) there is no energy norm, and the
  // estimator throws std::invalid_argument; when A is zero as written,
  // too.
  energy,
  // The L2 norm of the gradient difference: the classical
  // Zienkiewicz-Zhu estimator, independent of A.
  gradient,
};

struct RecoveryEstimatorOptions {
  RecoveryNorm norm = RecoveryNorm::energy;
  // The exact degree of the triangle quadrature rule
  // (fem/quadrature.hpp), in [0, fem::max_triangle_degree]. The default 3
  // is exact when A is a polynomial of degree at most 1 (G - grad u_h is
  // linear, so the integrand is a cubic).
  int triangle_degree = 3;
};

// The recovery (Zienkiewicz-Zhu) error estimator. The recovered gradient G
// is continuous and piecewise linear: at each node, the average of the
// constant gradients grad u_h of the triangles around it, weighted by
// their areas; between nodes, linear. Then
//
//   eta_T^2 = integral over T of (G - grad u_h) . W (G - grad u_h),
//
// W = |A| or the identity (RecoveryNorm). On structured meshes the
// averaged gradient is superconvergent at interior nodes, so the total
// tends to the true error in the same norm (effectivity -> 1).
// Quadrature: the triangle rule of quadrature.hpp of the exact degree
// RecoveryEstimatorOptions::triangle_degree (default 3: 6 points, exact
// when A is a polynomial of degree at most 1).
// No derivatives are needed (given ones are ignored); boundary conditions
// play no part. Report metrics (besides the shared ones): none.
class RecoveryEstimator final : public ErrorEstimator {
public:
  // Throws std::invalid_argument for a norm that is not one of the
  // RecoveryNorm values, or a triangle_degree that quadrature.hpp has no
  // rule for (below 0 or above fem::max_triangle_degree).
  explicit RecoveryEstimator(RecoveryEstimatorOptions options = {});
  const RecoveryEstimatorOptions &options() const { return options_; }
  std::string name() const override { return "recovery_estimator"; }

private:
  std::vector<double> indicators(const fem::MeshedProblem &problem,
                                 const solve::Solution &solution,
                                 const TimeDerivatives &derivatives,
                                 mesh::StageReport &report) const override;
  RecoveryEstimatorOptions options_;
};

} // namespace pde2d::adapt
