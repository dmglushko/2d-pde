#pragma once

// Private: what an Expression holds. The only file of the library that
// includes SymEngine headers; it is included only from .cpp files in
// src/fem/.
//
// SymEngine here is built without thread-safe reference counting, so its
// objects must not be copied or destroyed from several threads at once.
// The library does all symbolic work (parsing, substitution, compiling)
// in one thread; calling a compiled function only reads it.

#include "pde2d/fem/expression.hpp"

// SymEngine includes <ciso646>, which GCC 15 warns about in C++20 and
// later; the warning is about SymEngine, not our code.
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcpp"
#endif
#include <symengine/basic.h>
#include <symengine/infinity.h>
#include <symengine/integer.h>
#include <symengine/lambda_double.h>
#include <symengine/nan.h>
#include <symengine/number.h>
#include <symengine/parser.h>
#include <symengine/real_double.h>
#include <symengine/subs.h>
#include <symengine/symbol.h>
#include <symengine/visitor.h>
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

#include <string>
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
};

// Throws std::invalid_argument when a parameter name is not a plain symbol
// other than x, y and t, or a value is not finite.
void check_parameters(const Parameters &parameters);

// True when both expressions have the same form as written (expression.hpp).
bool same_as_written(const Expression &a, const Expression &b);

} // namespace detail
} // namespace pde2d::fem
