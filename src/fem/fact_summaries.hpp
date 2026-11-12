#pragma once

// Private: the summaries of problem.hpp, written once for both forms of a
// problem. `equation` has the members of Equation and `conditions` is a
// range of objects with the members of BoundaryCondition; each member has
// facts() (Expression in a Problem, CompiledExpression in a
// MeshedProblem).

namespace pde2d::fem::detail {

template <class Equation, class Conditions>
int time_order_of(const Equation &equation, const Conditions &conditions) {
  if (!equation.m2.facts().zero)
    return 2;
  if (!equation.m1.facts().zero)
    return 1;
  for (const auto &condition : conditions)
    if (!condition.gamma.facts().zero)
      return 1;
  return 0;
}

template <class Equation, class Conditions>
bool coefficients_depend_on_t_of(const Equation &equation,
                                 const Conditions &conditions) {
  const auto t = [](const auto &coefficient) {
    return coefficient.facts().depends_on_t;
  };
  if (t(equation.a_xx) || t(equation.a_xy) || t(equation.a_yy) ||
      t(equation.b_x) || t(equation.b_y) || t(equation.c) || t(equation.m1) ||
      t(equation.m2))
    return true;
  for (const auto &condition : conditions)
    if (t(condition.alpha) || t(condition.beta_n) || t(condition.beta_tau) ||
        t(condition.beta_x) || t(condition.beta_y) || t(condition.gamma))
      return true;
  return false;
}

template <class Equation, class Conditions>
bool data_depends_on_t_of(const Equation &equation,
                          const Conditions &conditions) {
  if (equation.f.facts().depends_on_t)
    return true;
  for (const auto &condition : conditions)
    if (condition.g.facts().depends_on_t)
      return true;
  return false;
}

} // namespace pde2d::fem::detail
