#pragma once

#include "pde2d/fem/expression.hpp"

#include <memory>

namespace pde2d::fem {

// An Expression turned into a fast numeric function of (x, y, t), with its
// parameters replaced by their values. Built once, then called many times
// (for example at every quadrature point).
//
// The constructor throws std::invalid_argument when the expression uses a
// name that is neither x, y, t nor a parameter in `parameters`, or when
// `parameters` itself is invalid (see Parameters). A call does not check
// its result: 1/x at x = 0 gives an infinity, as in C++.
//
// The facts are those of the expression as written (expression.hpp): they
// do not change when parameters get values.
class CompiledExpression {
public:
  // Zero, like Expression().
  CompiledExpression();
  CompiledExpression(const Expression &expression,
                     const Parameters &parameters);

  double operator()(double x, double y, double t) const;
  const ExpressionFacts &facts() const;

private:
  struct Impl;
  std::shared_ptr<const Impl> impl_;
};

} // namespace pde2d::fem
