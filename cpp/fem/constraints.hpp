#pragma once
// Dirichlet lifting of an assembled linear system, independent of the
// element type and of the solver:
//
//   auto [a, b] = constrain(K, F, system.dirichlet);
//
// Constrained rows become identity rows with the prescribed value on the
// right-hand side; their columns are moved to the right-hand side of the
// other rows. solver_detail::validate(a, b) is the shared shape and
// finiteness check of a linear system (also used by PreparedLinearSolver).
#include "fem/system.hpp"
#include <cmath>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>

namespace pde2d::fem {
namespace solver_detail {
inline void validate(const SparseMatrix &a, const Vector &b) {
  if (a.rows() != a.cols() || a.rows() != b.size() || a.rows() == 0)
    throw std::invalid_argument(
        "Linear system must be nonempty and square with matching RHS");
  if (!b.allFinite())
    throw std::invalid_argument("Nonfinite linear RHS");
  // An uncompressed Eigen matrix may have unused slots between columns.
  for (int column = 0; column < a.outerSize(); ++column)
    for (SparseMatrix::InnerIterator entry(a, column); entry; ++entry)
      if (!std::isfinite(entry.value()))
        throw std::invalid_argument("Nonfinite linear matrix");
}
} // namespace solver_detail

inline std::pair<SparseMatrix, Vector>
constrain(const SparseMatrix &a, const Vector &b,
          const std::vector<DirichletValue> &constraints) {
  solver_detail::validate(a, b);
  std::map<Eigen::Index, double> prescribed;
  for (const auto &d : constraints) {
    if (d.node < 0 || d.node >= a.rows() || !std::isfinite(d.value))
      throw std::invalid_argument("Invalid Dirichlet constraint");
    auto [it, inserted] = prescribed.emplace(d.node, d.value);
    if (!inserted && it->second != d.value)
      throw std::invalid_argument("Conflicting Dirichlet constraints");
  }
  Vector rhs = b;
  std::vector<Eigen::Triplet<double>> entries;
  for (int k = 0; k < a.outerSize(); ++k)
    for (SparseMatrix::InnerIterator it(a, k); it; ++it) {
      if (prescribed.contains(it.row()))
        continue;
      const auto p = prescribed.find(it.col());
      if (p != prescribed.end())
        rhs[it.row()] -= it.value() * p->second;
      else
        entries.emplace_back(it.row(), it.col(), it.value());
    }
  for (const auto &[i, g] : prescribed) {
    entries.emplace_back(i, i, 1);
    rhs[i] = g;
  }
  SparseMatrix result(a.rows(), a.cols());
  result.setFromTriplets(entries.begin(), entries.end());
  if (!rhs.allFinite())
    throw std::overflow_error("Nonfinite Dirichlet lifting");
  return {std::move(result), std::move(rhs)};
}

} // namespace pde2d::fem
