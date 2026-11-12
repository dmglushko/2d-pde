#include "pde2d/fem/problem.hpp"

#include "fact_summaries.hpp"

#include <ranges>
#include <stdexcept>

namespace pde2d::fem {

ConditionKind classify(const BoundaryCondition &condition) {
  const auto set = [](const Expression &coefficient) {
    return !coefficient.facts().zero;
  };
  const bool normal_part =
      set(condition.beta_n) || set(condition.beta_x) || set(condition.beta_y);
  const bool spatial_derivative = normal_part || set(condition.beta_tau);
  if (!spatial_derivative && !set(condition.gamma)) {
    if (!set(condition.alpha))
      throw std::invalid_argument(
          "the boundary condition has no term on its left-hand side (alpha, "
          "beta_n, beta_tau, beta_x, beta_y and gamma are all zero)");
    return ConditionKind::dirichlet;
  }
  if (!spatial_derivative)
    throw std::invalid_argument(
        "the boundary condition is a boundary ODE (gamma is not zero but "
        "there is no spatial derivative), which is not supported");
  if (!normal_part)
    throw std::invalid_argument(
        "the boundary condition has only a tangential derivative (beta_tau "
        "without beta_n, beta_x or beta_y), which is not supported");
  return set(condition.gamma) ? ConditionKind::dynamic : ConditionKind::natural;
}

bool is_oblique(const BoundaryCondition &condition) {
  return !condition.beta_tau.facts().zero || !condition.beta_x.facts().zero ||
         !condition.beta_y.facts().zero;
}

int time_order(const Problem &problem) {
  return detail::time_order_of(problem.equation,
                               std::views::values(problem.boundary_conditions));
}

bool lhs_depends_on_t(const Problem &problem) {
  return detail::lhs_depends_on_t_of(
      problem.equation, std::views::values(problem.boundary_conditions));
}

bool data_depends_on_t(const Problem &problem) {
  return detail::data_depends_on_t_of(
      problem.equation, std::views::values(problem.boundary_conditions));
}

} // namespace pde2d::fem
