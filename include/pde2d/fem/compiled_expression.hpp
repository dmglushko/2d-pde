#pragma once

#include "pde2d/fem/expression.hpp"

#include <memory>

namespace pde2d::fem {

// An Expression turned into a fast numeric function of (x, y, t), with its
// parameters replaced by their values. Built once, then called many times
// (for example at every quadrature point).
//
// The constructor throws std::invalid_argument when the expression uses a
// name that is neither x, y, t nor a parameter in `parameters`, when
// `parameters` itself is invalid (see Parameters), or when the values make
// a part of the expression complex or not finite (log(k) with k = -1, 1/k
// with k = 0). A call does not check its result: 1/x at x = 0 gives an
// infinity, as in C++.
//
// Calls may be made from several threads at once: a call only reads the
// prepared function, and every construct of the allow-list (expression.hpp)
// evaluates without shared state. Building an Expression or a
// CompiledExpression, and dropping the last copy of one, is symbolic work
// that belongs in one thread (SymEngine's reference counts are not
// thread-safe in this build).
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
