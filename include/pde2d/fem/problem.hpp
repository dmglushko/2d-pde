#pragma once

#include "pde2d/fem/expression.hpp"

#include <map>
#include <optional>
#include <string>

namespace pde2d::fem {

// a_xx u_xx + a_xy u_xy + a_yy u_yy + b_x u_x + b_y u_y + c u
//   + m1 u_t + m2 u_tt = f
// Every coefficient may depend on x, y and t. An unset coefficient is zero.
struct Equation {
  Expression a_xx{}, a_xy{}, a_yy{};
  Expression b_x{}, b_y{};
  Expression c{};
  Expression m1{}, m2{};
  Expression f{};
};

// alpha u + beta_n u_n + beta_tau u_tau + beta_x u_x + beta_y u_y
//   + gamma u_t = g
// on one boundary region. n is the outward normal and tau the tangent
// (n turned counterclockwise by 90 degrees, so the domain is on its left),
// both of the straight mesh edge. `priority` decides between two Dirichlet
// conditions at a corner: the higher one wins, equal ones are averaged.
struct BoundaryCondition {
  Expression alpha{};
  Expression beta_n{}, beta_tau{}, beta_x{}, beta_y{};
  Expression gamma{};
  Expression g{};
  int priority = 0;
};

// u and u_t at the start. Which ones are required follows from
// time_order(): none for 0, u for 1, both for 2. They may depend on x, y
// and parameters, and also on t: MeshedProblem evaluates them at the
// start time it is given (initial_values(t0)). One that is not required,
// or one that depends on t, is only a warning in MeshedProblem's report.
struct InitialConditions {
  std::optional<Expression> u{};
  std::optional<Expression> u_t{};
};

// The whole problem: the equation, the values of its named parameters, one
// boundary condition per boundary region name, and the initial conditions.
// It knows nothing about a mesh; MeshedProblem combines the two and checks
// that they fit.
struct Problem {
  Equation equation{};
  Parameters parameters{};
  std::map<std::string, BoundaryCondition> boundary_conditions{};
  InitialConditions initial{};
};

// The structural class of a boundary condition, read off which of its
// coefficients are zero as written:
// - dirichlet: only alpha (and g): u = g / alpha;
// - natural: a spatial derivative with a normal part, gamma zero (Neumann,
//   Robin, and oblique when beta_tau, beta_x or beta_y is set);
// - dynamic: like natural, plus gamma u_t.
enum class ConditionKind { dirichlet, natural, dynamic };

// Classifies one condition. Throws std::invalid_argument for the forms
// that are not supported: no term at all on the left-hand side, a boundary
// ODE (gamma not zero, no spatial derivative) and a tangential-only
// condition (beta_tau without beta_n, beta_x or beta_y, so b.n = 0).
ConditionKind classify(const BoundaryCondition &condition);

// True when the derivative is not purely along the normal: beta_tau,
// beta_x or beta_y is not zero as written.
bool is_oblique(const BoundaryCondition &condition);

// Summaries, computed from the coefficients' facts every time (never
// stored):
// - time_order: 2 if m2 is not zero, else 1 if m1 or the gamma of any
//   boundary condition is not zero, else 0;
// - lhs_depends_on_t: a coefficient of the left-hand side (the equation's
//   a_*, b_*, c, m1, m2, or a condition's alpha, beta_*, gamma) depends on
//   t;
// - data_depends_on_t: f or the g of any boundary condition depends on t,
//   as written. This describes the input only, not the right-hand side of
//   the assembled system: whether that changes in time is a fact of
//   assembly (it also depends on alpha, beta, the lifting term and the
//   left-hand side coefficients).
int time_order(const Problem &problem);
bool lhs_depends_on_t(const Problem &problem);
bool data_depends_on_t(const Problem &problem);

} // namespace pde2d::fem
