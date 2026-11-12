#pragma once

// Private: what an Expression holds. The only file of the library that
// includes SymEngine headers; it is included only from .cpp files in
// src/fem/.
//
// SymEngine here is built without thread-safe reference counting, so its
// objects must not be copied or destroyed from several threads at once.
// The library does all symbolic work (parsing, substitution,
// differentiation, compiling) in one thread. Calling a compiled function
// only reads it: the evaluator's functions for the constructs on the
// allow-list (expression.hpp) keep no state and call only standard math
// functions that keep none either (tests/fem/expression_threads.cpp).

#include "pde2d/fem/expression.hpp"

// SymEngine includes <ciso646>, which GCC 15 warns about in C++20 and
// later; the warning is about SymEngine, not our code.
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcpp"
#endif
#include <symengine/add.h>
#include <symengine/basic.h>
#include <symengine/complex.h>
#include <symengine/complex_double.h>
#include <symengine/constants.h>
#include <symengine/eval_double.h>
#include <symengine/functions.h>
#include <symengine/infinity.h>
#include <symengine/integer.h>
#include <symengine/lambda_double.h>
#include <symengine/logic.h>
#include <symengine/mul.h>
#include <symengine/nan.h>
#include <symengine/number.h>
#include <symengine/parser.h>
#include <symengine/pow.h>
#include <symengine/rational.h>
#include <symengine/real_double.h>
#include <symengine/subs.h>
#include <symengine/symbol.h>
#include <symengine/visitor.h>
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace pde2d::fem {

struct Expression::Impl {
  std::string text;
  // The parsed form, as SymEngine builds it from the text.
  SymEngine::RCP<const SymEngine::Basic> form;
  ExpressionFacts facts;
  // Free symbols other than x, y and t, sorted by name.
  std::vector<std::string> parameter_names;
};

namespace detail {

struct ExpressionAccess {
  using Impl = Expression::Impl;
  static const Impl &impl(const Expression &expression) {
    return *expression.impl_;
  }
  static Expression make(std::shared_ptr<const Impl> impl) {
    Expression expression;
    expression.impl_ = std::move(impl);
    return expression;
  }
};

// Why `form` is not accepted as an expression, as a phrase that follows
// "The expression 'text' " (for example "uses 'Xor', which is not among
// the supported functions and operators"), or nothing when it is: every
// construct is on the allow-list (expression.hpp), every Piecewise ends
// with an unconditional branch, conditions appear only as Piecewise
// conditions and every number is finite as a double. When several things
// are wrong, the first of these is reported: unknown functions,
// unsupported constructs, complex numbers, infinite or undefined values,
// numbers too large, conditions used as numbers, Piecewise without a
// final branch; names are sorted.
std::optional<std::string> unsupported(const SymEngine::Basic &form);

// An Expression made from a form the library built (for example a
// derivative), checked like one built from text. Its text is SymEngine's
// printed form. Throws std::invalid_argument like Expression(text).
Expression expression_from_form(SymEngine::RCP<const SymEngine::Basic> form);

// d/dt (numerator / denominator), differentiated symbolically. Throws
// std::invalid_argument when the derivative is not an accepted expression
// (SymEngine cannot differentiate abs, for example, and leaves a
// Derivative).
Expression time_derivative_of_quotient(const Expression &numerator,
                                       const Expression &denominator);

// Throws std::invalid_argument when a parameter name is not a plain symbol
// other than x, y and t, or a value is not finite.
void check_parameters(const Parameters &parameters);

// True when both expressions have the same form as written (expression.hpp).
bool same_as_written(const Expression &a, const Expression &b);

} // namespace detail
} // namespace pde2d::fem
