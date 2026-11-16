#pragma once

#include "options.hpp"

#include "pde2d/fem/problem.hpp"

#include <string_view>

// The "problem" section of a case, as text, and the fem::Problem built
// from it:
//
//   {"equation": {"a_xx": "-1", "a_yy": "-1", "f": "1"},
//    "parameters": {"k": 2.5},
//    "boundary_conditions": {
//      "outer": {"alpha": "1", "g": "0", "priority": 0}},
//    "initial": {"u": "0", "u_t": ""}}
//
// Every coefficient is the text of an Expression. A coefficient that is
// missing, null or empty is not set: zero for the equation and the
// boundary conditions, "not given" for the initial conditions.
namespace pde2d::app {

// The keys of the problem section, and what a boundary condition has
// besides its coefficients.
inline constexpr std::string_view problem_section_keys[] = {
    "equation", "parameters", "boundary_conditions", "initial"};
inline constexpr std::string_view condition_extra_keys[] = {"priority"};

// Throws std::invalid_argument naming the path in the case file
// ("problem.equation.a_xx: ...") for a wrong value type, an unknown key or
// a text that Expression refuses (with Expression's message).
fem::Problem build_problem(const Json &problem_section);

// The values of the named parameters ("problem.parameters").
fem::Parameters read_parameters(const Json &parameters);

// Checks the problem section without stopping at the first error, for the
// page: {"ok", "errors": [{"path", "message"}], "facts": {path: facts},
// "conditions": {region: {"kind", "oblique"}}, "time_order",
// "coefficients_depend_on_t", "data_depends_on_t", "required_initial"}.
// Every expression is also compiled with the parameters, which finds
// names that are neither x, y, t nor a parameter. It does not need a mesh,
// so it cannot check that the regions match one (MeshedProblem does).
Json check_problem(const Json &problem_section);

// The condition a new region gets: u = 0.
Json default_condition();

// The problem a new case starts with: -u_xx - u_yy = 1 and no boundary
// conditions (the page adds u = 0 for every region of the geometry).
Json default_problem_section();

// What the page builds the problem forms from: the form of the equation
// and of a boundary condition, their coefficients in order (name, the
// term each multiplies), the initial conditions, the allowed constructs
// of an expression, and templates for common equations and conditions.
Json problem_catalog_json();

} // namespace pde2d::app
