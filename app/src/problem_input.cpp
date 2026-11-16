#include "problem_input.hpp"

#include "app_error.hpp"

#include "pde2d/fem/compiled_expression.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <format>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace pde2d::app {

namespace {

using EquationMember = fem::Expression fem::Equation::*;
using ConditionMember = fem::Expression fem::BoundaryCondition::*;
using InitialMember = std::optional<fem::Expression> fem::InitialConditions::*;

// A coefficient: its key, the term it multiplies (for the form) and where
// it goes in the library's struct.
template <class Member> struct Field {
  std::string_view name;
  std::string_view term;
  Member member;
};

constexpr std::array<Field<EquationMember>, 9> equation_fields{{
    {"a_xx", "u_xx", &fem::Equation::a_xx},
    {"a_xy", "u_xy", &fem::Equation::a_xy},
    {"a_yy", "u_yy", &fem::Equation::a_yy},
    {"b_x", "u_x", &fem::Equation::b_x},
    {"b_y", "u_y", &fem::Equation::b_y},
    {"c", "u", &fem::Equation::c},
    {"m1", "u_t", &fem::Equation::m1},
    {"m2", "u_tt", &fem::Equation::m2},
    {"f", "", &fem::Equation::f},
}};

constexpr std::array<Field<ConditionMember>, 7> condition_fields{{
    {"alpha", "u", &fem::BoundaryCondition::alpha},
    {"beta_n", "u_n", &fem::BoundaryCondition::beta_n},
    {"beta_tau", "u_tau", &fem::BoundaryCondition::beta_tau},
    {"beta_x", "u_x", &fem::BoundaryCondition::beta_x},
    {"beta_y", "u_y", &fem::BoundaryCondition::beta_y},
    {"gamma", "u_t", &fem::BoundaryCondition::gamma},
    {"g", "", &fem::BoundaryCondition::g},
}};

constexpr std::array<Field<InitialMember>, 2> initial_fields{{
    {"u", "u", &fem::InitialConditions::u},
    {"u_t", "u_t", &fem::InitialConditions::u_t},
}};

template <class Member, std::size_t N>
std::vector<std::string_view>
field_names(const std::array<Field<Member>, N> &fields) {
  std::vector<std::string_view> names;
  for (const Field<Member> &field : fields)
    names.push_back(field.name);
  return names;
}

// Where errors go: thrown at once (building), or collected per path
// (checking, so the page can mark every field that is wrong).
class Errors {
public:
  explicit Errors(Json *collected) : collected_(collected) {}

  // Runs `step`; an std::invalid_argument it throws is collected at
  // `path` (or at the PathError's own path, which the JSON readers give),
  // or thrown again as a PathError. Returns false after an error.
  bool attempt(const std::string &path, const std::function<void()> &step) {
    try {
      step();
      return true;
    } catch (const PathError &error) {
      fail(error.path().empty() ? path : error.path(), error.detail());
    } catch (const std::invalid_argument &error) {
      fail(path, error.what());
    }
    return false;
  }

  // Whether an error was collected at `path`.
  bool failed_at(const std::string &path) const {
    if (collected_ != nullptr)
      for (const Json &item : *collected_)
        if (item["path"] == path)
          return true;
    return false;
  }

private:
  void fail(const std::string &path, const std::string &detail) {
    if (collected_ == nullptr)
      throw PathError(path, detail);
    Json item = Json::object();
    item["path"] = path;
    item["message"] = detail;
    collected_->push_back(std::move(item));
  }

  Json *collected_;
};

// The text of a coefficient, or nothing when it is missing, null or empty.
// A number is accepted too (a hand-written case may say "c": 0).
std::optional<std::string> coefficient_text(const Json &object,
                                            std::string_view key,
                                            const std::string &where) {
  const Json &value = member(object, key);
  if (value.is_null())
    return std::nullopt;
  if (value.is_number())
    return std::format("{}", require_number(value, where));
  std::string text = require_string(value, where);
  const auto first = text.find_first_not_of(" \t\r\n");
  if (first == std::string::npos)
    return std::nullopt;
  const auto last = text.find_last_not_of(" \t\r\n");
  return text.substr(first, last - first + 1);
}

// Reads the coefficients of one object into `target`.
template <class Target, class Member, std::size_t N, class Set>
void read_fields(const Json &object, const std::string &where,
                 const std::array<Field<Member>, N> &fields, Target &target,
                 Errors &errors, Set set) {
  for (const Field<Member> &field : fields) {
    const std::string path = where + "." + std::string(field.name);
    errors.attempt(path, [&] {
      if (const auto text = coefficient_text(object, field.name, path))
        set(target, field.member, fem::Expression(*text));
    });
  }
}

fem::Problem read_problem(const Json &section, Errors &errors) {
  fem::Problem problem;
  const Json empty = Json::object();
  const Json &input = section.is_null() ? empty : section;
  if (!errors.attempt("problem", [&] {
        require_object(input, "problem");
        reject_unknown_keys(input, "problem", problem_section_keys);
      }))
    return problem;

  const auto set_expression = [](auto &target, auto member,
                                 fem::Expression expression) {
    target.*member = std::move(expression);
  };

  const Json &equation = member(input, "equation");
  if (errors.attempt("problem.equation", [&] {
        if (equation.is_null())
          return;
        require_object(equation, "problem.equation");
        const auto names = field_names(equation_fields);
        reject_unknown_keys(equation, "problem.equation", names);
      }))
    read_fields(equation, "problem.equation", equation_fields, problem.equation,
                errors, set_expression);

  errors.attempt("problem.parameters", [&] {
    problem.parameters = read_parameters(member(input, "parameters"));
  });

  const Json &conditions = member(input, "boundary_conditions");
  if (errors.attempt("problem.boundary_conditions",
                     [&] {
                       if (!conditions.is_null())
                         require_object(conditions,
                                        "problem.boundary_conditions");
                     }) &&
      conditions.is_object())
    for (const auto &item : conditions.items()) {
      const std::string where = "problem.boundary_conditions." + item.key();
      fem::BoundaryCondition condition;
      const Json &object = item.value();
      if (!errors.attempt(where, [&] {
            require_object(object, where);
            auto names = field_names(condition_fields);
            names.insert(names.end(), std::begin(condition_extra_keys),
                         std::end(condition_extra_keys));
            reject_unknown_keys(object, where, names);
          }))
        continue;
      read_fields(object, where, condition_fields, condition, errors,
                  set_expression);
      errors.attempt(where + ".priority", [&] {
        const Json &priority = member(object, "priority");
        if (priority.is_null())
          return;
        const double value = require_number(priority, where + ".priority");
        // Checked before the conversion: converting a double outside int's
        // range is undefined behaviour.
        if (value != std::floor(value) ||
            value < std::numeric_limits<int>::min() ||
            value > std::numeric_limits<int>::max())
          throw PathError(where + ".priority",
                          "expected a whole number that fits an int");
        condition.priority = static_cast<int>(value);
      });
      problem.boundary_conditions.emplace(item.key(), std::move(condition));
    }

  const Json &initial = member(input, "initial");
  if (errors.attempt("problem.initial", [&] {
        if (initial.is_null())
          return;
        require_object(initial, "problem.initial");
        const auto names = field_names(initial_fields);
        reject_unknown_keys(initial, "problem.initial", names);
      }))
    read_fields(initial, "problem.initial", initial_fields, problem.initial,
                errors, set_expression);

  return problem;
}

Json facts_json(const fem::ExpressionFacts &facts) {
  Json out = Json::object();
  out["zero"] = facts.zero;
  out["depends_on_x"] = facts.depends_on_x;
  out["depends_on_y"] = facts.depends_on_y;
  out["depends_on_t"] = facts.depends_on_t;
  return out;
}

std::string_view kind_name(fem::ConditionKind kind) {
  switch (kind) {
  case fem::ConditionKind::dirichlet:
    return "dirichlet";
  case fem::ConditionKind::natural:
    return "natural";
  case fem::ConditionKind::dynamic:
    return "dynamic";
  }
  throw std::logic_error("unknown condition kind");
}

template <class Member, std::size_t N>
Json fields_json(const std::array<Field<Member>, N> &fields) {
  Json out = Json::array();
  for (const Field<Member> &field : fields) {
    Json item = Json::object();
    item["name"] = field.name;
    item["term"] = field.term;
    out.push_back(std::move(item));
  }
  return out;
}

Json texts(std::initializer_list<std::pair<std::string_view, std::string_view>>
               values) {
  Json out = Json::object();
  for (const auto &[key, value] : values)
    out[std::string(key)] = value;
  return out;
}

Json named_template(std::string_view label, std::string_view description,
                    Json values) {
  Json out = Json::object();
  out["label"] = label;
  out["description"] = description;
  out["values"] = std::move(values);
  return out;
}

} // namespace

fem::Parameters read_parameters(const Json &parameters) {
  fem::Parameters out;
  if (parameters.is_null())
    return out;
  require_object(parameters, "problem.parameters");
  for (const auto &item : parameters.items())
    out[item.key()] = require_number(item.value(), "problem.parameters." +
                                                       std::string(item.key()));
  return out;
}

fem::Problem build_problem(const Json &problem_section) {
  Errors errors(nullptr);
  return read_problem(problem_section, errors);
}

Json check_problem(const Json &problem_section) {
  Json error_list = Json::array();
  Errors errors(&error_list);
  const fem::Problem problem = read_problem(problem_section, errors);

  // Compiling with the parameters finds names that are neither x, y, t
  // nor a parameter. Invalid parameters make every compile fail, so they
  // are checked once on their own first.
  const bool parameters_valid = errors.attempt("problem.parameters", [&] {
    [[maybe_unused]] const fem::CompiledExpression zero(fem::Expression(),
                                                        problem.parameters);
  });
  Json facts = Json::object();
  const auto inspect = [&](const std::string &path,
                           const fem::Expression &expression) {
    // A text that did not parse has no facts: the zero expression left in
    // its place is not what was written.
    if (errors.failed_at(path))
      return;
    facts[path] = facts_json(expression.facts());
    if (parameters_valid && !expression.facts().zero)
      errors.attempt(path, [&] {
        [[maybe_unused]] const fem::CompiledExpression compiled(
            expression, problem.parameters);
      });
  };
  for (const auto &field : equation_fields)
    inspect("problem.equation." + std::string(field.name),
            problem.equation.*field.member);
  Json conditions = Json::object();
  for (const auto &[region, condition] : problem.boundary_conditions) {
    const std::string where = "problem.boundary_conditions." + region;
    for (const auto &field : condition_fields)
      inspect(where + "." + std::string(field.name), condition.*field.member);
    errors.attempt(where, [&] {
      Json item = Json::object();
      item["kind"] = kind_name(fem::classify(condition));
      item["oblique"] = fem::is_oblique(condition);
      conditions[region] = std::move(item);
    });
  }
  for (const auto &field : initial_fields)
    if (const auto &expression = problem.initial.*field.member)
      inspect("problem.initial." + std::string(field.name), *expression);

  Json out = Json::object();
  out["ok"] = error_list.empty();
  out["errors"] = std::move(error_list);
  out["facts"] = std::move(facts);
  out["conditions"] = std::move(conditions);
  // The summaries describe the input as written; they need no compiling.
  const int order = fem::time_order(problem);
  out["time_order"] = order;
  out["coefficients_depend_on_t"] = fem::coefficients_depend_on_t(problem);
  out["data_depends_on_t"] = fem::data_depends_on_t(problem);
  Json required = Json::array();
  if (order >= 1)
    required.push_back("u");
  if (order >= 2)
    required.push_back("u_t");
  out["required_initial"] = std::move(required);
  return out;
}

Json default_problem_section() {
  Json out = Json::object();
  out["equation"] = texts({{"a_xx", "-1"}, {"a_yy", "-1"}, {"f", "1"}});
  out["parameters"] = Json::object();
  out["boundary_conditions"] = Json::object();
  out["initial"] = Json::object();
  return out;
}

Json default_condition() { return texts({{"alpha", "1"}, {"g", "0"}}); }

Json problem_catalog_json() {
  Json out = Json::object();
  out["equation_form"] = "a_xx u_xx + a_xy u_xy + a_yy u_yy + b_x u_x + "
                         "b_y u_y + c u + m1 u_t + m2 u_tt = f";
  out["condition_form"] = "alpha u + beta_n u_n + beta_tau u_tau + "
                          "beta_x u_x + beta_y u_y + gamma u_t = g";
  out["condition_note"] =
      "n is the outward normal and tau the tangent (n turned "
      "counterclockwise by 90 degrees), both of the straight mesh edge. "
      "At a corner between two Dirichlet regions the higher priority "
      "wins; equal priorities are averaged.";
  out["equation_fields"] = fields_json(equation_fields);
  out["condition_fields"] = fields_json(condition_fields);
  out["initial_fields"] = fields_json(initial_fields);
  out["initial_note"] =
      "Which initial conditions are needed follows from the time order: "
      "none for 0, u for 1, u and u_t for 2.";
  out["expression_help"] =
      "Variables x, y, t and named parameters; numbers, pi, E; + - * / ^; "
      "sin cos tan asin acos atan sinh cosh tanh exp log (or ln) sqrt abs; "
      "Piecewise((value, condition), ..., (value, True)) with < <= > >= == "
      "!= and And (&), Or (|), Not (~). An empty field is zero.";
  out["default_condition"] = default_condition();

  Json equations = Json::array();
  equations.push_back(
      named_template("Poisson", "-(u_xx + u_yy) = f",
                     texts({{"a_xx", "-1"}, {"a_yy", "-1"}, {"f", "1"}})));
  equations.push_back(named_template(
      "Heat", "u_t - k (u_xx + u_yy) = f",
      texts({{"a_xx", "-k"}, {"a_yy", "-k"}, {"m1", "1"}, {"f", "0"}})));
  equations.push_back(named_template(
      "Wave", "u_tt - c2 (u_xx + u_yy) = f",
      texts({{"a_xx", "-c2"}, {"a_yy", "-c2"}, {"m2", "1"}, {"f", "0"}})));
  equations.push_back(named_template("Convection-diffusion",
                                     "-k (u_xx + u_yy) + v_x u_x + v_y u_y = f",
                                     texts({{"a_xx", "-k"},
                                            {"a_yy", "-k"},
                                            {"b_x", "v_x"},
                                            {"b_y", "v_y"},
                                            {"f", "1"}})));
  equations.push_back(named_template(
      "Reaction-diffusion", "-(u_xx + u_yy) + s u = f",
      texts({{"a_xx", "-1"}, {"a_yy", "-1"}, {"c", "s"}, {"f", "1"}})));
  out["equation_templates"] = std::move(equations);

  Json conditions = Json::array();
  conditions.push_back(
      named_template("Dirichlet", "u = g", default_condition()));
  conditions.push_back(named_template("Neumann", "u_n = g",
                                      texts({{"beta_n", "1"}, {"g", "0"}})));
  conditions.push_back(
      named_template("Robin", "alpha u + u_n = g",
                     texts({{"alpha", "1"}, {"beta_n", "1"}, {"g", "0"}})));
  conditions.push_back(
      named_template("Dynamic", "u_n + u_t = g",
                     texts({{"beta_n", "1"}, {"gamma", "1"}, {"g", "0"}})));
  out["condition_templates"] = std::move(conditions);
  return out;
}

} // namespace pde2d::app
