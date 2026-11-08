#pragma once

#include <symengine/expression.h>
#include <symengine/functions.h>
#include <symengine/lambda_double.h>
#include <symengine/symengine_casts.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>

namespace pde2d::fem {
using Expr = SymEngine::Expression;
inline const Expr x{SymEngine::symbol("x")}, y{SymEngine::symbol("y")},
    t{SymEngine::symbol("t")}, nx{SymEngine::symbol("nx")},
    ny{SymEngine::symbol("ny")};

// Symbols available by context. Evaluator binds all five; expressions that
// are not evaluated on a boundary edge (interior coefficients, initial
// conditions, interpolated fields, exact solutions) must not use normals.
inline const std::array<Expr, 3> field_symbols{x, y, t};
inline const std::array<Expr, 5> boundary_symbols{x, y, t, nx, ny};

// Throws std::invalid_argument when `expression` uses one of x, y, t, nx, ny
// that is not in `allowed`: "<what> cannot depend on boundary normals (nx)"
// or "<what> cannot depend on t". Other free symbols are left to Evaluator
// ("Unbound symbol in FEM expression").
inline void require_symbols(const Expr &expression,
                            std::span<const Expr> allowed,
                            const std::string &what) {
  const auto contains = [](std::span<const Expr> symbols,
                           const SymEngine::RCP<const SymEngine::Basic> &s) {
    return std::any_of(symbols.begin(), symbols.end(), [&](const Expr &e) {
      return SymEngine::eq(*s, *e.get_basic());
    });
  };
  for (const auto &symbol : SymEngine::free_symbols(*expression.get_basic())) {
    if (!contains(boundary_symbols, symbol) || contains(allowed, symbol))
      continue;
    const bool normal = SymEngine::eq(*symbol, *nx.get_basic()) ||
                        SymEngine::eq(*symbol, *ny.get_basic());
    throw std::invalid_argument(
        what + " cannot depend on " +
        (normal ? "boundary normals (" + symbol->__str__() + ")"
                : symbol->__str__()));
  }
}

struct EvaluationPoint {
  double x = 0, y = 0, t = 0, nx = 0, ny = 0;
};

inline Expr diff(const Expr &expression, const Expr &variable) {
  if (!SymEngine::is_a<SymEngine::Symbol>(*variable.get_basic()))
    throw std::invalid_argument("Differentiation variable must be a symbol");
  return expression.diff(variable.get_basic());
}
inline bool is_zero(const Expr &expression) {
  const Expr expanded = SymEngine::expand(expression);
  // SymEngine distinguishes the trees RealDouble(0.0) and Integer(0).
  // Geometry normals introduce doubles, so recognize numeric zero by value.
  return SymEngine::is_a_Number(*expanded.get_basic()) &&
         SymEngine::down_cast<const SymEngine::Number &>(*expanded.get_basic())
             .is_zero();
}
// True when `symbol` occurs freely in `expression` (e.g. time dependence).
inline bool depends_on(const Expr &expression, const Expr &symbol) {
  for (const auto &s : SymEngine::free_symbols(*expression.get_basic()))
    if (SymEngine::eq(*s, *symbol.get_basic()))
      return true;
  return false;
}

// Numeric overloads preserve ordinary floating-point arithmetic in generic
// code.
using std::cos;
using std::exp;
using std::log;
using std::pow;
using std::sin;
using std::sqrt;
using SymEngine::pow;
inline Expr sin(const Expr &a) { return SymEngine::sin(a.get_basic()); }
inline Expr cos(const Expr &a) { return SymEngine::cos(a.get_basic()); }
inline Expr exp(const Expr &a) { return SymEngine::exp(a.get_basic()); }
inline Expr sqrt(const Expr &a) { return SymEngine::sqrt(a.get_basic()); }
inline Expr log(const Expr &a) { return SymEngine::log(a.get_basic()); }

class Evaluator {
  Expr expression_;
  // With CSE disabled, calls only read the prepared functions. Shared ownership
  // makes evaluators cheap to copy while retaining the original symbolic tree.
  std::shared_ptr<SymEngine::LambdaRealDoubleVisitor> prepared_;

public:
  explicit Evaluator(const Expr &expression)
      : expression_(expression),
        prepared_(std::make_shared<SymEngine::LambdaRealDoubleVisitor>()) {
    SymEngine::vec_basic variables{x.get_basic(), y.get_basic(), t.get_basic(),
                                   nx.get_basic(), ny.get_basic()};
    for (const auto &symbol :
         SymEngine::free_symbols(*expression.get_basic())) {
      bool bound = false;
      for (const auto &variable : variables)
        bound = bound || SymEngine::eq(*symbol, *variable);
      if (!bound)
        throw std::invalid_argument("Unbound symbol in FEM expression: " +
                                    symbol->__str__());
    }
    prepared_->init(variables, *expression.get_basic());
  }
  const Expr &expression() const noexcept { return expression_; }
  double operator()(EvaluationPoint point) const {
    const std::array<double, 5> values{point.x, point.y, point.t, point.nx,
                                       point.ny};
    for (double value : values)
      if (!std::isfinite(value))
        throw std::invalid_argument("Nonfinite FEM evaluation point");
    double result;
    prepared_->call(&result, values.data());
    if (!std::isfinite(result))
      throw std::domain_error("Nonfinite FEM expression result");
    return result;
  }
};
inline double evaluate(const Expr &expression, EvaluationPoint point) {
  return Evaluator(expression)(point);
}
} // namespace pde2d::fem
