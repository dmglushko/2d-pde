#include "pde2d/fem/compiled_expression.hpp"

#include "symengine_expression.hpp"

#include <cmath>
#include <exception>
#include <stdexcept>
#include <string>

namespace pde2d::fem {

struct CompiledExpression::Impl {
  ExpressionFacts facts;
  // SymEngine's call() is not const, but without common subexpression
  // elimination (which is off here) it only reads the prepared functions.
  // The functions prepared for the constructs on the allow-list keep no
  // state either, so concurrent calls are safe
  // (tests/fem/expression_threads.cpp).
  mutable SymEngine::LambdaRealDoubleVisitor visitor;
};

namespace detail {

void check_parameters(const Parameters &parameters) {
  for (const auto &[name, value] : parameters) {
    if (name == "x" || name == "y" || name == "t")
      throw std::invalid_argument("Parameter '" + name +
                                  "' has the name of a variable (x, y, t)");
    // A usable name is one the parser reads back as that very symbol: this
    // rejects "2k", "a+b" and constants such as pi or E.
    bool plain = false;
    try {
      const auto form = SymEngine::parse(name);
      plain =
          SymEngine::is_a<SymEngine::Symbol>(*form) &&
          SymEngine::down_cast<const SymEngine::Symbol &>(*form).get_name() ==
              name;
    } catch (const std::exception &) {
      plain = false;
    }
    if (!plain)
      throw std::invalid_argument(
          "Parameter '" + name +
          "' is not a plain symbol name (letters, digits and _, starting "
          "with a letter, and not a constant such as pi or E)");
    if (!std::isfinite(value))
      throw std::invalid_argument("Parameter '" + name + "' is not finite");
  }
}

} // namespace detail

CompiledExpression::CompiledExpression()
    : CompiledExpression(Expression(), {}) {}

CompiledExpression::CompiledExpression(const Expression &expression,
                                       const Parameters &parameters) {
  detail::check_parameters(parameters);
  const auto &source = detail::ExpressionAccess::impl(expression);
  SymEngine::map_basic_basic values;
  for (const auto &name : source.parameter_names) {
    const auto found = parameters.find(name);
    if (found == parameters.end())
      throw std::invalid_argument("The expression '" + source.text +
                                  "' uses '" + name +
                                  "', which is not a parameter (the "
                                  "variables are x, y and t)");
    values[SymEngine::symbol(name)] = SymEngine::real_double(found->second);
  }
  auto impl = std::make_shared<Impl>();
  impl->facts = source.facts;
  // Substituting values evaluates the parts that become constant, which
  // may give a complex number (log(k) with k = -1) or one that is not
  // finite (1/k with k = 0): checked again, like the parsed form.
  const auto form = detail::substitute_powers(source.form, values);
  if (const auto reason = detail::unsupported_substitution(source.form, values))
    throw std::invalid_argument("The expression '" + source.text +
                                "' with the given parameters " + *reason);
  try {
    impl->visitor.init({SymEngine::symbol("x"), SymEngine::symbol("y"),
                        SymEngine::symbol("t")},
                       *form);
  } catch (const std::exception &e) {
    // Not expected: every construct the check accepts can be evaluated.
    throw std::invalid_argument("The expression '" + source.text +
                                "' cannot be evaluated with the given "
                                "parameters (" +
                                e.what() + ")");
  }
  impl_ = std::move(impl);
}

double CompiledExpression::operator()(double x, double y, double t) const {
  const double inputs[3] = {x, y, t};
  double result = 0.0;
  impl_->visitor.call(&result, inputs);
  return result;
}

const ExpressionFacts &CompiledExpression::facts() const {
  return impl_->facts;
}

} // namespace pde2d::fem
