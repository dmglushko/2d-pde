#include "derivative.hpp"

#include "symengine_expression.hpp"

#include <algorithm>
#include <utility>

namespace pde2d::fem::detail {

namespace {

using SymEngine::Basic;
using SymEngine::RCP;

// SymEngine's differentiation with two rules of our own. SymEngine leaves
// d/du abs(u) as an unevaluated Derivative, which the allow-list rejects;
// here it is sign(u) du. Piecewise is differentiated branch by branch as
// in SymEngine, and a result whose branches are all equal is that value
// (the last branch is unconditional, so one branch always applies).
class Differentiator
    : public SymEngine::BaseVisitor<Differentiator, SymEngine::DiffVisitor> {
public:
  explicit Differentiator(const RCP<const SymEngine::Symbol> &variable)
      : BaseVisitor(variable, true) {}

  using DiffVisitor::bvisit;

  void bvisit(const SymEngine::Pow &self) {
    using namespace SymEngine;
    // On every branch where 0^e is defined it is constant. In particular,
    // do not introduce log(0) in a guarded Piecewise branch.
    if (eq(*self.get_base(), *zero)) {
      result_ = zero;
      return;
    }
    const auto de = apply(self.get_exp());
    if (eq(*de, *zero)) {
      // Keep the chain rule free of division by a possibly zero base.
      result_ = mul(
          mul(self.get_exp(), pow(self.get_base(), sub(self.get_exp(), one))),
          apply(self.get_base()));
    } else {
      DiffVisitor::bvisit(self);
    }
  }

  void bvisit(const SymEngine::Abs &self) {
    using namespace SymEngine;
    const RCP<const Basic> &u = self.get_arg();
    const RCP<const Basic> du = apply(u);
    const RCP<const Basic> sign = piecewise({{integer(1), Lt(integer(0), u)},
                                             {integer(-1), Lt(u, integer(0))},
                                             {integer(0), boolTrue}});
    result_ = mul(sign, du);
  }

  void bvisit(const SymEngine::Piecewise &self) {
    SymEngine::PiecewiseVec branches;
    for (const auto &[value, condition] : self.get_vec())
      branches.emplace_back(apply(value), condition);
    const bool all_equal =
        std::ranges::all_of(branches, [&](const auto &branch) {
          return SymEngine::eq(*branch.first, *branches.front().first);
        });
    result_ = all_equal ? branches.front().first
                        : SymEngine::piecewise(std::move(branches));
  }
};

const char *name_of(Variable variable) {
  switch (variable) {
  case Variable::x:
    return "x";
  case Variable::y:
    return "y";
  case Variable::t:
    return "t";
  }
  return "t";
}

} // namespace

Expression derivative(const Expression &expression, Variable variable) {
  Differentiator differentiator(SymEngine::symbol(name_of(variable)));
  return expression_from_form(
      differentiator.apply(ExpressionAccess::impl(expression).form));
}

Expression time_derivative_of_quotient(const Expression &numerator,
                                       const Expression &denominator) {
  const auto quotient = expression_from_form(
      SymEngine::div(ExpressionAccess::impl(numerator).form,
                     ExpressionAccess::impl(denominator).form));
  return derivative(quotient, Variable::t);
}

std::array<Expression, 2> divergence_of_a(const Expression &a_xx,
                                          const Expression &a_xy,
                                          const Expression &a_yy) {
  const auto form = [](const Expression &expression) {
    return ExpressionAccess::impl(expression).form;
  };
  const auto half =
      SymEngine::div(SymEngine::integer(1), SymEngine::integer(2));
  const auto d_xy_dx = form(derivative(a_xy, Variable::x));
  const auto d_xy_dy = form(derivative(a_xy, Variable::y));
  return {
      expression_from_form(SymEngine::add(form(derivative(a_xx, Variable::x)),
                                          SymEngine::mul(half, d_xy_dy))),
      expression_from_form(SymEngine::add(
          SymEngine::mul(half, d_xy_dx), form(derivative(a_yy, Variable::y))))};
}

} // namespace pde2d::fem::detail
