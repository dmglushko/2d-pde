#pragma once
// Deterministic fingerprints of FEM inputs for run records (record.hpp):
//
//   problem_fingerprint(problem)  every Equation field, then every boundary
//                                 assignment (regions and every condition
//                                 field) in assignment order
//   field_fingerprint(field)      an Expr or a nodal Vector (Field)
//   expression_text(e)            the canonical SymEngine string used above
//
// Expressions are hashed through their canonical SymEngine string, so
// equal expressions built the same way give equal fingerprints across runs
// and platforms; the text may change with the SymEngine version, which run
// records report next to the fingerprint. Doubles are hashed by bit
// pattern. The hash is pde2d::Fingerprint (provenance.hpp).
#include "fem/expression.hpp"
#include "fem/interpolation.hpp"
#include "fem/problem.hpp"
#include "provenance.hpp"
#include <string>
#include <variant>

namespace pde2d::fem {
inline std::string expression_text(const Expr &expression) {
  return expression.get_basic()->__str__();
}

namespace fingerprint_detail {
inline void add(pde2d::Fingerprint &f, const char *name, const Expr &e) {
  f.add(name).add(expression_text(e));
}
} // namespace fingerprint_detail

inline std::string problem_fingerprint(const Problem &problem) {
  using fingerprint_detail::add;
  pde2d::Fingerprint f;
  f.add("pde2d.problem.v1");
  const auto &eq = problem.equation;
  add(f, "a_xx", eq.a_xx);
  add(f, "a_xy", eq.a_xy);
  add(f, "a_yy", eq.a_yy);
  add(f, "b_x", eq.b_x);
  add(f, "b_y", eq.b_y);
  add(f, "c", eq.c);
  add(f, "source", eq.source);
  add(f, "m1", eq.m1);
  add(f, "m2", eq.m2);
  f.add(problem.boundaries.size());
  for (const auto &assignment : problem.boundaries) {
    f.add(assignment.regions.size());
    for (const auto &region : assignment.regions)
      f.add(region);
    const auto &bc = assignment.condition;
    add(f, "alpha", bc.alpha);
    add(f, "gamma", bc.gamma);
    add(f, "beta_flux", bc.beta_flux);
    add(f, "beta_normal", bc.beta_normal);
    add(f, "beta_x", bc.beta_x);
    add(f, "beta_y", bc.beta_y);
    add(f, "value", bc.value);
  }
  return f.hex();
}

// "expression" or "nodal".
inline const char *field_kind(const Field &field) {
  return std::holds_alternative<Expr>(field) ? "expression" : "nodal";
}
inline std::string field_fingerprint(const Field &field) {
  pde2d::Fingerprint f;
  f.add("pde2d.field.v1");
  if (const auto *expression = std::get_if<Expr>(&field))
    f.add("expression").add(expression_text(*expression));
  else {
    const auto &values = std::get<Vector>(field);
    f.add("nodal").add(static_cast<std::uint64_t>(values.size()));
    for (Eigen::Index k = 0; k < values.size(); ++k)
      f.add(values[k]);
  }
  return f.hex();
}
} // namespace pde2d::fem
