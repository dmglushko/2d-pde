#pragma once

#include "pde2d/adapt/error_estimator.hpp"

namespace pde2d::adapt {

// The exact degree of the edge quadrature rule (fem/quadrature.hpp) the
// Kelly estimator integrates with; see ResidualEstimatorOptions.
struct KellyEstimatorOptions {
  // For the jumps ||J_E||^2 and the boundary residuals ||R_E||^2, in
  // [0, fem::max_edge_degree]. The default 6 is exact for coefficients
  // and data of degree at most 1 with beta_hat_n constant.
  int edge_degree = 6;
};

// The Kelly error estimator: the jump terms of the residual estimator
// (residual_estimator.hpp) without the element residual,
//
//   eta_T^2 = sum over the interior edges E of T of  1/2 h_E ||J_E||_E^2
//           + sum over the natural and dynamic boundary edges E of T of
//                                                      h_E ||R_E||_E^2,
//
// with the same J_E, R_E, h_E and edge quadrature (of the degree
// KellyEstimatorOptions::edge_degree). It ignores the time terms:
// R_E is taken without gamma u_t, and no derivatives are needed (given
// ones are ignored). Report metrics (besides the shared ones): jump_part,
// boundary_part.
class KellyEstimator final : public ErrorEstimator {
public:
  // Throws std::invalid_argument for a degree that quadrature.hpp has no
  // rule for (below 0 or above fem::max_edge_degree).
  explicit KellyEstimator(KellyEstimatorOptions options = {});
  const KellyEstimatorOptions &options() const { return options_; }
  std::string name() const override { return "kelly_estimator"; }

private:
  std::vector<double> indicators(const fem::MeshedProblem &problem,
                                 const solve::Solution &solution,
                                 const TimeDerivatives &derivatives,
                                 mesh::StageReport &report) const override;
  KellyEstimatorOptions options_;
};

} // namespace pde2d::adapt
