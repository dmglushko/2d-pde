#include "presets.hpp"

#include "catalog.hpp"

#include <cmath>
#include <functional>
#include <initializer_list>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// Sign conventions (docs/app.md, "The catalog"): the equation is
// a_xx u_xx + a_xy u_xy + a_yy u_yy + b_x u_x + b_y u_y + c u + m1 u_t +
// m2 u_tt = f, so -k (u_xx + u_yy) is a_xx = a_yy = -k; a condition is
// alpha u + beta_n u_n + ... = g with n the outward normal, so cooling
// -k u_n = h (u - u_out) is written u_n = -(h/k)(u - u_out) with k = 1:
// alpha = h, beta_n = 1, g = h u_out.
namespace pde2d::app {

namespace {

Json texts(std::initializer_list<std::pair<const char *, std::string>> values) {
  Json out = Json::object();
  for (const auto &[key, value] : values)
    out[key] = value;
  return out;
}

Json numbers(std::initializer_list<std::pair<const char *, double>> values) {
  Json out = Json::object();
  for (const auto &[key, value] : values)
    out[key] = value;
  return out;
}

// `digits` significant digits, so the values in the form read well.
double round_to(double value, int digits) {
  if (value == 0.0 || !std::isfinite(value))
    return value;
  const double scale =
      std::pow(10.0, digits - 1 - std::floor(std::log10(std::abs(value))));
  return std::round(value * scale) / scale;
}

// What a preset fills in.
struct PresetProblem {
  Json equation;
  Json parameters = Json::object();
  // Every region gets `condition`; the first region gets
  // `first_condition` instead when it is set.
  Json condition;
  Json first_condition = nullptr;
  Json initial = Json::object();
  Json solve;
};

struct Preset {
  std::string id;
  std::string label;
  std::string description;
  std::function<PresetProblem(const DomainFacts &)> make;
};

Json dirichlet(std::string g) {
  return texts({{"alpha", "1"}, {"g", std::move(g)}});
}

Json static_solve(std::string_view linear_solver) {
  Json out = default_solve_section();
  out["mode"] = "static";
  out["linear_solver"] = solve_component_choice("linear_solver", linear_solver);
  out["static"] = numbers({{"t", 0.0}});
  return out;
}

// From 0 to t1 with `snapshots` kept times and `steps` steps between two
// of them, so dt divides every snapshot interval.
Json time_solve(std::string_view method, std::string_view linear_solver,
                double t1, int snapshots, int steps) {
  Json out = default_solve_section();
  out["mode"] = "time";
  out["linear_solver"] = solve_component_choice("linear_solver", linear_solver);
  Json time = Json::object();
  time["t0"] = 0.0;
  time["t1"] = t1;
  time["snapshots"] = snapshots;
  time["method"] = solve_component_choice("method", method);
  Json step = solve_component_choice("step", "fixed_step");
  step["options"]["dt"] = t1 / (snapshots * steps);
  time["step"] = std::move(step);
  out["time"] = std::move(time);
  return out;
}

// A smooth bump around the point well inside the domain: its centre and
// width as parameters x0, y0, w (half the distance to the boundary).
const std::string bump = "exp(-((x - x0)^2 + (y - y0)^2) / w^2)";

void add_bump(Json &parameters, const DomainFacts &facts) {
  parameters["x0"] = round_to(facts.inner.x, 3);
  parameters["y0"] = round_to(facts.inner.y, 3);
  parameters["w"] = round_to(facts.clearance / 2, 2);
}

double bump_width(const DomainFacts &facts) {
  return round_to(facts.clearance / 2, 2);
}

const std::vector<Preset> &presets() {
  static const std::vector<Preset> list = {
      {"poisson", "Poisson",
       "-(u_xx + u_yy) = 1 with u = 0 on the boundary; static",
       [](const DomainFacts &) {
         PresetProblem p;
         p.equation = texts({{"a_xx", "-1"}, {"a_yy", "-1"}, {"f", "1"}});
         p.condition = dirichlet("0");
         p.solve = static_solve("simplicial_ldlt");
         return p;
       }},
      {"laplace", "Laplace, u = x on the boundary",
       "u_xx + u_yy = 0 with u = x on the boundary: the solution is u = x; "
       "static",
       [](const DomainFacts &) {
         PresetProblem p;
         p.equation = texts({{"a_xx", "-1"}, {"a_yy", "-1"}});
         p.condition = dirichlet("x");
         p.solve = static_solve("simplicial_ldlt");
         return p;
       }},
      {"heat_bump", "Heat: a bump diffuses",
       "u_t = k (u_xx + u_yy) from a bump, u = 0 on the boundary; implicit "
       "midpoint",
       [](const DomainFacts &facts) {
         PresetProblem p;
         p.equation = texts({{"m1", "1"}, {"a_xx", "-k"}, {"a_yy", "-k"}});
         p.parameters = numbers({{"k", 1.0}});
         add_bump(p.parameters, facts);
         p.condition = dirichlet("0");
         p.initial = texts({{"u", bump}});
         const double w = bump_width(facts);
         p.solve = time_solve("implicit_midpoint", "simplicial_ldlt",
                              round_to(4 * w * w, 2), 50, 2);
         return p;
       }},
      {"heat_heated", "Heat: a boundary is heated",
       "u_t = k (u_xx + u_yy) from u = 0; the first region heats up, u = 1 "
       "- exp(-t/t_on), the others stay at u = 0; implicit Euler",
       [](const DomainFacts &facts) {
         PresetProblem p;
         const double t1 =
             round_to(facts.domain_size * facts.domain_size / 4, 2);
         p.equation = texts({{"m1", "1"}, {"a_xx", "-k"}, {"a_yy", "-k"}});
         p.parameters = numbers({{"k", 1.0}, {"t_on", round_to(t1 / 10, 2)}});
         p.condition = dirichlet("0");
         p.first_condition = dirichlet("1 - exp(-t/t_on)");
         p.first_condition["priority"] = 1;
         p.initial = texts({{"u", "0"}});
         p.solve = time_solve("implicit_euler", "simplicial_ldlt", t1, 50, 4);
         return p;
       }},
      {"heat_cooling", "Heat: cooling through the boundary",
       "u_t = k (u_xx + u_yy) from u = u0, with Robin cooling u_n = -h (u - "
       "u_out) on the boundary; implicit Euler",
       [](const DomainFacts &facts) {
         PresetProblem p;
         p.equation = texts({{"m1", "1"}, {"a_xx", "-k"}, {"a_yy", "-k"}});
         p.parameters =
             numbers({{"k", 1.0}, {"h", 2.0}, {"u0", 1.0}, {"u_out", 0.0}});
         p.condition =
             texts({{"alpha", "h"}, {"beta_n", "1"}, {"g", "h*u_out"}});
         p.initial = texts({{"u", "u0"}});
         const double r = facts.clearance;
         p.solve = time_solve("implicit_euler", "simplicial_ldlt",
                              round_to(2 * r * r + r, 2), 50, 4);
         return p;
       }},
      {"wave", "Wave: a membrane from a bump at rest",
       "u_tt = c2 (u_xx + u_yy) from a bump with u_t = 0, fixed edge u = 0; "
       "Newmark",
       [](const DomainFacts &facts) {
         PresetProblem p;
         p.equation = texts({{"m2", "1"}, {"a_xx", "-c2"}, {"a_yy", "-c2"}});
         p.parameters = numbers({{"c2", 1.0}});
         add_bump(p.parameters, facts);
         p.condition = dirichlet("0");
         p.initial = texts({{"u", bump}, {"u_t", "0"}});
         p.solve = time_solve("newmark", "simplicial_ldlt",
                              round_to(facts.domain_size, 2), 100, 4);
         return p;
       }},
      {"wave_damped", "Wave with damping",
       "u_tt + d u_t = c2 (u_xx + u_yy) from a bump at rest, u = 0; the "
       "motion dies out; Newmark",
       [](const DomainFacts &facts) {
         PresetProblem p;
         const double t1 = round_to(facts.domain_size, 2);
         p.equation = texts(
             {{"m2", "1"}, {"m1", "d"}, {"a_xx", "-c2"}, {"a_yy", "-c2"}});
         p.parameters = numbers({{"c2", 1.0}, {"d", round_to(4 / t1, 2)}});
         add_bump(p.parameters, facts);
         p.condition = dirichlet("0");
         p.initial = texts({{"u", bump}, {"u_t", "0"}});
         p.solve = time_solve("newmark", "simplicial_ldlt", t1, 100, 4);
         return p;
       }},
      {"convection_diffusion", "Convection-diffusion, steady",
       "-k (u_xx + u_yy) + v_x u_x + v_y u_y = 1 with u = 0: the flow (v_x, "
       "v_y) carries the solution downstream; static, sparse LU (the matrix "
       "is not symmetric)",
       [](const DomainFacts &facts) {
         PresetProblem p;
         p.equation = texts({{"a_xx", "-k"},
                             {"a_yy", "-k"},
                             {"b_x", "v_x"},
                             {"b_y", "v_y"},
                             {"f", "1"}});
         p.parameters = numbers({{"k", round_to(0.05 * facts.domain_size, 2)},
                                 {"v_x", 1.0},
                                 {"v_y", 0.5}});
         p.condition = dirichlet("0");
         p.solve = static_solve("sparse_lu");
         return p;
       }},
      {"reaction_diffusion", "Reaction-diffusion",
       "-k (u_xx + u_yy) + s u = a bump source, insulated boundary u_n = 0 "
       "(the reaction makes it well posed); static",
       [](const DomainFacts &facts) {
         PresetProblem p;
         p.equation =
             texts({{"a_xx", "-k"}, {"a_yy", "-k"}, {"c", "s"}, {"f", bump}});
         p.parameters = numbers(
             {{"k", 1.0},
              {"s",
               round_to(16 / (facts.domain_size * facts.domain_size), 2)}});
         add_bump(p.parameters, facts);
         p.condition = texts({{"beta_n", "1"}, {"g", "0"}});
         p.solve = static_solve("simplicial_ldlt");
         return p;
       }},
  };
  return list;
}

} // namespace

Json preset_catalog_json() {
  Json out = Json::array();
  for (const Preset &preset : presets()) {
    Json item = Json::object();
    item["id"] = preset.id;
    item["label"] = preset.label;
    item["description"] = preset.description;
    out.push_back(std::move(item));
  }
  return out;
}

Json apply_preset(const Json &case_json, std::string_view id,
                  const DomainFacts &facts) {
  for (const Preset &preset : presets()) {
    if (preset.id != id)
      continue;
    const PresetProblem p = preset.make(facts);
    Json conditions = Json::object();
    for (std::size_t i = 0; i < facts.regions.size(); ++i)
      conditions[facts.regions[i]] = i == 0 && !p.first_condition.is_null()
                                         ? p.first_condition
                                         : p.condition;
    Json problem = Json::object();
    problem["equation"] = p.equation;
    problem["parameters"] = p.parameters;
    problem["boundary_conditions"] = std::move(conditions);
    problem["initial"] = p.initial;
    Json out = case_json;
    out["problem"] = std::move(problem);
    out["solve"] = p.solve;
    return out;
  }
  std::string known;
  for (const Preset &preset : presets())
    known += (known.empty() ? "" : ", ") + preset.id;
  throw std::invalid_argument("preset: unknown preset '" + std::string(id) +
                              "' (" + known + ")");
}

} // namespace pde2d::app
