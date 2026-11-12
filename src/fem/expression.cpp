#include "pde2d/fem/expression.hpp"

#include "symengine_expression.hpp"

#include <algorithm>
#include <exception>
#include <stdexcept>
#include <string>
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

// Rejects what parses but has no real numeric value: unknown functions,
// infinities, complex numbers. Compiling the form once with every free
// symbol as an input is what tells whether it can be evaluated.
void check_evaluable(const std::string &text, const Basic &form,
                     const std::vector<std::string> &names) {
  const auto functions = SymEngine::function_symbols(form);
  if (!functions.empty()) {
    std::vector<std::string> function_names;
    for (const auto &function : functions)
      function_names.push_back(
          SymEngine::down_cast<const SymEngine::FunctionSymbol &>(*function)
              .get_name());
    std::ranges::sort(function_names);
    throw std::invalid_argument("The expression " + in_quotes(text) +
                                " uses the unknown function " +
                                in_quotes(function_names.front()));
  }
  if (!SymEngine::atoms<SymEngine::Infty, SymEngine::NaN>(form).empty())
    throw std::invalid_argument("The expression " + in_quotes(text) +
                                " contains an infinite or undefined value");
  SymEngine::vec_basic inputs;
  for (const auto &name : names)
    inputs.push_back(SymEngine::symbol(name));
  try {
    SymEngine::LambdaRealDoubleVisitor visitor;
    visitor.init(inputs, form);
  } catch (const std::exception &e) {
    throw std::invalid_argument("The expression " + in_quotes(text) +
                                " has no real numeric value (" + e.what() +
                                ")");
  }
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
  auto names = free_symbol_names(*form);
  check_evaluable(text, *form, names);
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

namespace detail {

bool same_as_written(const Expression &a, const Expression &b) {
  return SymEngine::eq(*ExpressionAccess::impl(a).form,
                       *ExpressionAccess::impl(b).form);
}

} // namespace detail
} // namespace pde2d::fem
