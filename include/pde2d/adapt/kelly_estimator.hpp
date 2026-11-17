#pragma once

#include "pde2d/adapt/error_estimator.hpp"

namespace pde2d::adapt {

// The Kelly error estimator: the jump terms of the residual estimator
// (residual_estimator.hpp) without the element residual,
//
//   eta_T^2 = sum over the interior edges E of T of  1/2 h_E ||J_E||_E^2
//           + sum over the natural and dynamic boundary edges E of T of
//                                                      h_E ||R_E||_E^2,
//
// with the same J_E, R_E, h_E and quadrature. It ignores the time terms:
// R_E is taken without gamma u_t, and no derivatives are needed (given
// ones are ignored). Report metrics (besides the shared ones): jump_part,
// boundary_part.
class KellyEstimator final : public ErrorEstimator {
public:
  std::string name() const override { return "kelly_estimator"; }

private:
  std::vector<double> indicators(const fem::MeshedProblem &problem,
                                 const solve::Solution &solution,
                                 const TimeDerivatives &derivatives,
                                 mesh::StageReport &report) const override;
};

} // namespace pde2d::adapt
