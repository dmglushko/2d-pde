#pragma once

#include "pde2d/adapt/error_estimator.hpp"

namespace pde2d::adapt {

// The residual error estimator. For the equation
//   A : grad^2 u + b . grad u + c u + m1 u_t + m2 u_tt = f
// in the weak form of assembly (A : grad^2 u integrated by parts into
// -grad v . A grad u - (div A) . grad u v plus the flux (A grad u) . n on
// the boundary), with u_h piecewise linear:
//
//   eta_T^2 = h_T^2 ||r||_T^2
//           + sum over the interior edges E of T of  1/2 h_E ||J_E||_E^2
//           + sum over the natural and dynamic boundary edges E of T of
//                                                      h_E ||R_E||_E^2
//
// - h_T is the diameter of T (its longest edge), h_E the length of E.
// - The element residual is r = f - b . grad u_h - c u_h - m1 u_t
//   - m2 u_tt. The second-order part drops out exactly: grad u_h is
//   constant on T, so div(A grad u_h) = (div A) . grad u_h, which cancels
//   the -(div A) . grad u_h of the weak form.
// - The jump J_E = n . A (grad u_h|T - grad u_h|T') across an interior
//   edge, n the normal pointing out of T: the jump of the flux
//   (A grad u_h) . n, with A evaluated on the edge.
// - The boundary residual of a natural or dynamic edge is the flux the
//   condition prescribes minus the flux of u_h, which is the residual of
//   the condition as written scaled to a flux:
//     R_E = kappa_n / beta_n_hat * (g - alpha u_h - beta_n u_h,n
//           - beta_tau u_h,tau - beta_x u_h,x - beta_y u_h,y - gamma u_t),
//   with kappa_n = n . A n and beta_n_hat = beta_n + (beta_x, beta_y) . n
//   as in assembly (zero when kappa_n is; beta_n_hat = 0 throws where A
//   is not zero). Dirichlet edges contribute nothing.
//
// The time terms are evaluated from the given derivatives (P1 functions):
// u_t is required when m1 is not zero as written or a boundary region is
// dynamic (and A is not zero), u_tt when m2 is not zero as written;
// missing ones throw std::invalid_argument. A static solution passes
// zeros.
//
// Quadrature (quadrature.hpp), chosen so that each integral is exact when
// the coefficients and the data are polynomials of degree at most 1 and
// beta_hat_n is constant (then r is at most quadratic, J_E linear and R_E
// cubic, kappa_n / beta_hat_n times a quadratic): degree 4 on triangles
// (6 points) for ||r||^2, degree 2 on edges (2 points) for ||J||^2 and
// degree 6 on edges (4 points) for ||R||^2. A beta_hat_n that varies
// makes R_E rational, and other data are integrated approximately; that
// only changes the data oscillation part. The degrees are fixed, not
// options.
//
// Report metrics (besides the shared ones): element_part, jump_part,
// boundary_part: the square roots of the sums of each kind of term
// (element_part^2 + jump_part^2 + boundary_part^2 = total^2).
class ResidualEstimator final : public ErrorEstimator {
public:
  std::string name() const override { return "residual_estimator"; }

private:
  std::vector<double> indicators(const fem::MeshedProblem &problem,
                                 const solve::Solution &solution,
                                 const TimeDerivatives &derivatives,
                                 mesh::StageReport &report) const override;
};

} // namespace pde2d::adapt
