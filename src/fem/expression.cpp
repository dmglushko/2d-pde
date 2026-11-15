#include "pde2d/fem/expression.hpp"

#include "symengine_expression.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <exception>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace pde2d::fem {

namespace {

using SymEngine::Basic;
using SymEngine::RCP;

// Delay floating-literal evaluation until the power-aware substitution
// pass. Otherwise the parser already turns (-2)^2.0 into a complex value.
RCP<const Basic> parse_powers(const std::string &text,
                              std::optional<std::string> &reason) {
  std::string prefix = "pde2d_literal_";
  while (text.find(prefix) != std::string::npos)
    prefix += '_';
  std::string masked;
  SymEngine::map_basic_basic values;
  for (std::size_t i = 0; i < text.size();) {
    const auto start = i;
    if (std::isalpha(static_cast<unsigned char>(text[i])) || text[i] == '_') {
      while (
          i < text.size() &&
          (std::isalnum(static_cast<unsigned char>(text[i])) || text[i] == '_'))
        ++i;
    } else if (std::isdigit(static_cast<unsigned char>(text[i])) ||
               (text[i] == '.' && i + 1 < text.size() &&
                std::isdigit(static_cast<unsigned char>(text[i + 1])))) {
      while (i < text.size() &&
             std::isdigit(static_cast<unsigned char>(text[i])))
        ++i;
      if (i < text.size() && text[i] == '.') {
        ++i;
        while (i < text.size() &&
               std::isdigit(static_cast<unsigned char>(text[i])))
          ++i;
      }
      if (i < text.size() && (text[i] == 'e' || text[i] == 'E')) {
        ++i;
        if (i < text.size() && (text[i] == '+' || text[i] == '-'))
          ++i;
        while (i < text.size() &&
               std::isdigit(static_cast<unsigned char>(text[i])))
          ++i;
      }
      const auto token = text.substr(start, i - start);
      if (token.find_first_of(".eE") != std::string::npos) {
        // Let the original parser diagnose malformed numeric tokens.
        const auto number = SymEngine::parse(token);
        const auto name = prefix + std::to_string(values.size());
        values[SymEngine::symbol(name)] = number;
        masked += name;
        continue;
      }
    } else {
      ++i;
    }
    masked.append(text, start, i - start);
  }
  const auto form = SymEngine::parse(masked);
  if (!values.empty())
    reason = detail::unsupported_substitution(form, values);
  return detail::substitute_powers(form, values);
}

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
             is_a<ATan>(node) || is_a<Sinh>(node) || is_a<Cosh>(node) ||
             is_a<Tanh>(node) || is_a<Log>(node) || is_a<Abs>(node)) {
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

// The first constant part of `node` (a subexpression without x, y, t or
// parameters, other than a plain number) whose value is not a finite real
// double, such as exp(1000) or asin(2). Plain numbers are checked by
// walk_number; conditions are walked through their arguments.
std::optional<std::string> bad_constant_part(const Basic &node) {
  using namespace SymEngine;
  if (is_a_Number(node) || is_a<Symbol>(node) || is_a<Constant>(node))
    return std::nullopt;
  if (!is_a_Boolean(node) && free_symbols(node).empty()) {
    double value = std::nan("");
    try {
      value = eval_double(node);
    } catch (const std::exception &) {
    }
    if (!std::isfinite(value))
      return node.__str__();
    // A finite outer value can hide an overflowing descendant, such as
    // tanh(exp(1000)). Every constant part must be finite in its own right.
  }
  for (const auto &argument : node.get_args())
    if (auto bad = bad_constant_part(*argument))
      return bad;
  return std::nullopt;
}

// SymEngine's parser simplifies atan2 with repeated or opposite arguments
// to a constant without regard to their sign (atan2(x, x) becomes pi/4,
// although it is -3pi/4 for x < 0), so atan2 is refused by name, before
// parsing.
bool mentions_atan2(std::string_view text) {
  constexpr std::string_view name = "atan2";
  const auto is_name_char = [](char c) {
    return std::isalnum(static_cast<unsigned char>(c)) != 0 || c == '_';
  };
  for (auto at = text.find(name); at != std::string_view::npos;
       at = text.find(name, at + 1)) {
    if (at > 0 && is_name_char(text[at - 1]))
      continue;
    auto next = at + name.size();
    if (next < text.size() && is_name_char(text[next]))
      continue;
    while (next < text.size() &&
           std::isspace(static_cast<unsigned char>(text[next])) != 0)
      ++next;
    if (next < text.size() && text[next] == '(')
      return true;
  }
  return false;
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

std::optional<std::string>
unsupported_substitution(const SymEngine::RCP<const SymEngine::Basic> &form,
                         const SymEngine::map_basic_basic &values) {
  // Conditions are valid inside Piecewise, but not as standalone numbers.
  if (!SymEngine::is_a_Boolean(*form))
    if (auto reason = unsupported(*substitute_powers(form, values)))
      return reason;
  for (const auto &argument : form->get_args())
    if (auto reason = unsupported_substitution(argument, values))
      return reason;
  return std::nullopt;
}

SymEngine::RCP<const SymEngine::Basic>
substitute_powers(const SymEngine::RCP<const SymEngine::Basic> &form,
                  const SymEngine::map_basic_basic &values) {
  using namespace SymEngine;
  if (const auto it = values.find(form); it != values.end())
    return it->second;
  if (is_a<Pow>(*form)) {
    const auto &power = down_cast<const Pow &>(*form);
    const auto base = substitute_powers(power.get_base(), values);
    auto exponent = substitute_powers(power.get_exp(), values);
    if (is_a<RealDouble>(*exponent)) {
      const double value = down_cast<const RealDouble &>(*exponent).as_double();
      if (std::isfinite(value) && std::trunc(value) == value) {
        integer_class integral;
        mp_set_d(integral, value);
        exponent = integer(std::move(integral));
      }
    }
    return pow(base, exponent);
  }
  map_basic_basic children;
  for (const auto &argument : form->get_args()) {
    const auto replacement = substitute_powers(argument, values);
    if (!eq(*argument, *replacement))
      children[argument] = replacement;
  }
  return children.empty() ? form : xreplace(form, children);
}

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
  if (const auto bad = bad_constant_part(form))
    return "has the constant part " + in_quotes(*bad) +
           ", whose value is not a finite real number in double precision";
  return std::nullopt;
}

Expression expression_from_form(RCP<const Basic> form) {
  auto text = form->__str__();
  return ExpressionAccess::make(make_impl(std::move(text), std::move(form)));
}

bool same_as_written(const Expression &a, const Expression &b) {
  return SymEngine::eq(*ExpressionAccess::impl(a).form,
                       *ExpressionAccess::impl(b).form);
}

bool difference_is_zero(const Expression &a, const Expression &b) {
  const auto difference = SymEngine::sub(ExpressionAccess::impl(a).form,
                                         ExpressionAccess::impl(b).form);
  // sub already combines like terms; expand only when that was not enough.
  if (SymEngine::eq(*difference, *SymEngine::zero))
    return true;
  return SymEngine::eq(*SymEngine::expand(difference), *SymEngine::zero);
}

} // namespace detail

Expression::Expression() : impl_(make_impl("0", SymEngine::integer(0))) {}

Expression::Expression(std::string_view text) {
  std::string owned(text);
  if (mentions_atan2(owned))
    throw std::invalid_argument(
        "The expression " + in_quotes(owned) +
        " uses 'atan2', which is not among the supported functions and "
        "operators (SymEngine simplifies atan2(x, x) to pi/4 whatever the "
        "sign of x)");
  RCP<const Basic> form;
  std::optional<std::string> reason;
  try {
    form = parse_powers(owned, reason);
  } catch (const std::exception &e) {
    throw std::invalid_argument("Cannot parse the expression " +
                                in_quotes(owned) + ": " + e.what());
  }
  if (reason)
    throw std::invalid_argument("The expression " + in_quotes(owned) + " " +
                                *reason);
  impl_ = make_impl(std::move(owned), std::move(form));
}

const std::string &Expression::text() const { return impl_->text; }

const ExpressionFacts &Expression::facts() const { return impl_->facts; }

} // namespace pde2d::fem
