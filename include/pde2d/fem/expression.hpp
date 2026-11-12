#pragma once

#include <map>
#include <memory>
#include <string>
#include <string_view>

namespace pde2d::fem {

// What can be read off an expression as written, without simplifying it.
// "As written" is the form SymEngine gives the text when it parses it: that
// form already combines like terms (so "x - x" and "0*x" are zero), but
// nothing beyond it is tried ("sin(x)^2 + cos(x)^2 - 1" is not zero and
// depends on x). Named parameters count as unknown constants: "k*x" is not
// zero, whatever value k gets later.
struct ExpressionFacts {
  bool zero = false;
  bool depends_on_x = false;
  bool depends_on_y = false;
  bool depends_on_t = false;
};

// Values for the named parameters of expressions, by name, for example
// {{"k", 1000.0}}. A name must be a plain symbol other than x, y and t (not
// a constant such as pi or E) and every value must be finite.
using Parameters = std::map<std::string, double>;

namespace detail {
struct ExpressionAccess;
}

// A scalar expression in the variables x, y and t and in named parameters,
// built from text: Expression("1 + k*x*y"). The text is parsed when the
// expression is built. Only these constructs are accepted (the
// allow-list; each one is tested against the standard library):
// - numbers, finite as doubles; the variables x, y, t; parameter names;
//   the constants pi and E;
// - +, -, *, / and powers (^ or **);
// - sin, cos, tan, asin, acos, atan, atan2(y, x), sinh, cosh, tanh, exp,
//   log (or ln; log(u, b) is log(u)/log(b)), sqrt and abs;
// - Piecewise((value, condition), ..., (value, True)), whose last branch
//   must be unconditional. A condition compares numbers with <, <=, >,
//   >=, == (or Eq) and != (or Ne), and combines comparisons with And (&),
//   Or (|) and Not (~). A condition is not a number: it may appear only
//   as the condition of a Piecewise branch.
// Anything else throws std::invalid_argument when the expression is built,
// naming what is wrong: a parse error, an unknown function, a function or
// constant not on the list (loggamma, Xor, max, EulerGamma, ...), a complex,
// infinite or undefined value, a number too large for a double, a condition
// used as a number, a Piecewise without a final unconditional branch.
// Expression() is zero. Copies are cheap and share the parsed form, which
// never changes.
//
// SymEngine does the parsing, but it is used only inside the library's .cpp
// files: this header does not depend on it.
class Expression {
public:
  Expression();
  explicit Expression(std::string_view text);

  // The text the expression was built from ("0" for Expression()).
  const std::string &text() const;
  const ExpressionFacts &facts() const;

private:
  struct Impl;
  std::shared_ptr<const Impl> impl_;
  friend struct detail::ExpressionAccess;
};

} // namespace pde2d::fem
