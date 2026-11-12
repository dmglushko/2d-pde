#include "pde2d/fem/expression.hpp"

#include "symengine_expression.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <exception>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace pde2d::fem {

namespace {

using SymEngine::Basic;
using SymEngine::RCP;

std::string in_quotes(const std::string &text) { return "'" + text + "'"; }

// Free symbol names of `form`, sorted (SymEngine's own sets are ordered by
// hash, which is not an order a user would recognize).
std::vector<std::string> free_symbol_names(const Basic &form) {
  std::vector<std::string> names;
  for (const auto &symbol : SymEngine::free_symbols(form))
    names.push_back(symbol->__str__());
  std::ranges::sort(names);
  return names;
}

// What the allow-list check found, each kind as a sorted set of names so
// that the message does not depend on SymEngine's hash order.
struct Findings {
  std::set<std::string> unknown_functions;
  std::set<std::string> unsupported;
  std::set<std::string> complex_numbers;
  bool infinite_or_undefined = false;
  bool number_not_finite = false;
  std::set<std::string> conditions_as_numbers;
  std::set<std::string> open_piecewise;
};

// The name a user wrote for a construct: "loggamma" for loggamma(x), "Xor"
// for Xor(a, b), the whole text for a constant such as EulerGamma.
std::string construct_name(const Basic &node) {
  const std::string text = node.__str__();
  const auto paren = text.find('(');
  return paren == std::string::npos || paren == 0 ? text
                                                  : text.substr(0, paren);
}

void walk_condition(const Basic &node, Findings &found);

// A node in a place where a number is expected. Each kind the evaluator
// (SymEngine's LambdaRealDoubleVisitor) handles is walked through the
// parts the evaluator reads: Add and Mul through their coefficient and
// terms, so that every number it turns into a double is seen.
void walk_number(const Basic &node, Findings &found) {
  using namespace SymEngine;
  if (is_a<Integer>(node) || is_a<Rational>(node) || is_a<RealDouble>(node)) {
    if (!std::isfinite(eval_double(node)))
      found.number_not_finite = true;
  } else if (is_a<Symbol>(node)) {
  } else if (is_a<Constant>(node)) {
    if (!eq(node, *pi) && !eq(node, *E))
      found.unsupported.insert(construct_name(node));
  } else if (is_a<Add>(node)) {
    const auto &add = down_cast<const Add &>(node);
    walk_number(*add.get_coef(), found);
    for (const auto &[term, coefficient] : add.get_dict()) {
      walk_number(*term, found);
      walk_number(*coefficient, found);
    }
  } else if (is_a<Mul>(node)) {
    const auto &mul = down_cast<const Mul &>(node);
    walk_number(*mul.get_coef(), found);
    for (const auto &[base, exponent] : mul.get_dict()) {
      walk_number(*base, found);
      walk_number(*exponent, found);
    }
  } else if (is_a<Pow>(node) || is_a<Sin>(node) || is_a<Cos>(node) ||
             is_a<Tan>(node) || is_a<ASin>(node) || is_a<ACos>(node) ||
             is_a<ATan>(node) || is_a<ATan2>(node) || is_a<Sinh>(node) ||
             is_a<Cosh>(node) || is_a<Tanh>(node) || is_a<Log>(node) ||
             is_a<Abs>(node)) {
    // exp(u) is Pow(E, u) and sqrt(u) is Pow(u, 1/2).
    for (const auto &argument : node.get_args())
      walk_number(*argument, found);
  } else if (is_a<Piecewise>(node)) {
    const auto &branches = down_cast<const Piecewise &>(node).get_vec();
    // SymEngine's evaluator assumes the last condition is True: without it
    // it reads past the branches where no condition holds.
    if (branches.empty() || !eq(*branches.back().second, *boolTrue))
      found.open_piecewise.insert(node.__str__());
    for (const auto &[value, condition] : branches) {
      walk_number(*value, found);
      walk_condition(*condition, found);
    }
  } else if (is_a<Infty>(node) || is_a<NaN>(node)) {
    found.infinite_or_undefined = true;
  } else if (is_a<Complex>(node) || is_a<ComplexDouble>(node)) {
    found.complex_numbers.insert(node.__str__());
  } else if (is_a_Boolean(node)) {
    found.conditions_as_numbers.insert(node.__str__());
  } else if (is_a<FunctionSymbol>(node)) {
    found.unknown_functions.insert(
        down_cast<const FunctionSymbol &>(node).get_name());
  } else {
    found.unsupported.insert(construct_name(node));
  }
}

// A node in the place of a Piecewise condition.
void walk_condition(const Basic &node, Findings &found) {
  using namespace SymEngine;
  if (is_a<Equality>(node) || is_a<Unequality>(node) || is_a<LessThan>(node) ||
      is_a<StrictLessThan>(node)) {
    // x > y and x >= y are parsed as y < x and y <= x.
    for (const auto &argument : node.get_args())
      walk_number(*argument, found);
  } else if (is_a<And>(node) || is_a<Or>(node) || is_a<Not>(node)) {
    for (const auto &argument : node.get_args())
      walk_condition(*argument, found);
  } else if (is_a<BooleanAtom>(node)) {
  } else {
    found.unsupported.insert(construct_name(node));
  }
}

// 'a', 'a' and 'b', 'a', 'b' and 'c'.
std::string quoted_list(const std::set<std::string> &names) {
  std::string result;
  std::size_t k = 0;
  for (const auto &name : names) {
    if (k > 0)
      result += k + 1 == names.size() ? " and " : ", ";
    result += in_quotes(name);
    ++k;
  }
  return result;
}

ExpressionFacts facts_of(const Basic &form,
                         const std::vector<std::string> &names) {
  const auto uses = [&](const char *name) {
    return std::ranges::binary_search(names, std::string(name));
  };
  return {.zero =
              SymEngine::is_a_Number(form) &&
              SymEngine::down_cast<const SymEngine::Number &>(form).is_zero(),
          .depends_on_x = uses("x"),
          .depends_on_y = uses("y"),
          .depends_on_t = uses("t")};
}

using Impl = detail::ExpressionAccess::Impl;

std::shared_ptr<const Impl> make_impl(std::string text, RCP<const Basic> form) {
  if (const auto reason = detail::unsupported(*form))
    throw std::invalid_argument("The expression " + in_quotes(text) + " " +
                                *reason);
  auto names = free_symbol_names(*form);
  auto facts = facts_of(*form, names);
  std::erase_if(names, [](const std::string &name) {
    return name == "x" || name == "y" || name == "t";
  });
  return std::make_shared<const Impl>(
      Impl{.text = std::move(text),
           .form = std::move(form),
           .facts = facts,
           .parameter_names = std::move(names)});
}

} // namespace

namespace detail {

std::optional<std::string> unsupported(const Basic &form) {
  Findings found;
  walk_number(form, found);
  if (!found.unknown_functions.empty())
    return "uses the unknown function " +
           in_quotes(*found.unknown_functions.begin());
  if (!found.unsupported.empty())
    return "uses " + quoted_list(found.unsupported) + ", which " +
           (found.unsupported.size() == 1 ? "is" : "are") +
           " not among the supported functions and operators";
  if (!found.complex_numbers.empty())
    return "has no real numeric value (it contains the complex number " +
           in_quotes(*found.complex_numbers.begin()) + ")";
  if (found.infinite_or_undefined)
    return "contains an infinite or undefined value";
  if (found.number_not_finite)
    return "contains a number that is not finite in double precision";
  if (!found.conditions_as_numbers.empty())
    return "uses the condition " +
           in_quotes(*found.conditions_as_numbers.begin()) +
           " as a number; a condition may appear only in a Piecewise "
           "branch, as in Piecewise((1, x < 0), (0, True))";
  if (!found.open_piecewise.empty())
    return "has a Piecewise without a final unconditional branch (value, "
           "True): " +
           *found.open_piecewise.begin();
  return std::nullopt;
}

Expression expression_from_form(RCP<const Basic> form) {
  auto text = form->__str__();
  return ExpressionAccess::make(make_impl(std::move(text), std::move(form)));
}

Expression time_derivative_of_quotient(const Expression &numerator,
                                       const Expression &denominator) {
  const auto quotient =
      SymEngine::div(ExpressionAccess::impl(numerator).form,
                     ExpressionAccess::impl(denominator).form);
  return expression_from_form(quotient->diff(SymEngine::symbol("t")));
}

bool same_as_written(const Expression &a, const Expression &b) {
  return SymEngine::eq(*ExpressionAccess::impl(a).form,
                       *ExpressionAccess::impl(b).form);
}

} // namespace detail

Expression::Expression() : impl_(make_impl("0", SymEngine::integer(0))) {}

Expression::Expression(std::string_view text) {
  std::string owned(text);
  RCP<const Basic> form;
  try {
    form = SymEngine::parse(owned);
  } catch (const std::exception &e) {
    throw std::invalid_argument("Cannot parse the expression " +
                                in_quotes(owned) + ": " + e.what());
  }
  impl_ = make_impl(std::move(owned), std::move(form));
}

const std::string &Expression::text() const { return impl_->text; }

const ExpressionFacts &Expression::facts() const { return impl_->facts; }

} // namespace pde2d::fem
