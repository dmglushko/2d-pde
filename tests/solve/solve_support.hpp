#pragma once

// Helpers for the solver tests: a structured mesh of the unit square,
// sparse matrices from dense ones, and error norms. None of them uses the
// solver code.

#include "support/check.hpp"

#include "pde2d/fem/csr_matrix.hpp"
#include "pde2d/mesh/mesh.hpp"
#include "pde2d/solve/linear_solver.hpp"
#include "pde2d/solve/sparse_lu.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

namespace pde2d::test {

using Dense = std::vector<std::vector<double>>;

// The sparse form of a dense matrix: a slot for every entry that is not
// zero.
inline std::shared_ptr<const fem::CsrMatrix> sparse(const Dense &dense,
                                                    std::size_t columns) {
  auto pattern = std::make_shared<fem::SparsityPattern>();
  pattern->rows = dense.size();
  pattern->columns = columns;
  pattern->row_offsets.push_back(0);
  fem::CsrMatrix matrix;
  for (const auto &row : dense) {
    for (std::size_t j = 0; j < row.size(); ++j)
      if (row[j] != 0.0) {
        pattern->column_indices.push_back(static_cast<std::ptrdiff_t>(j));
        matrix.values.push_back(row[j]);
      }
    pattern->row_offsets.push_back(
        static_cast<std::ptrdiff_t>(pattern->column_indices.size()));
  }
  matrix.pattern = std::move(pattern);
  return std::make_shared<const fem::CsrMatrix>(std::move(matrix));
}

inline std::shared_ptr<const fem::CsrMatrix> sparse(const Dense &dense) {
  return sparse(dense, dense.size());
}

// The unit square cut into n x n cells, each split by its diagonal from
// (i, j) to (i + 1, j + 1). Node (i, j) is j (n + 1) + i, at (i / n, j / n).
// The sides are the regions "bottom" (y = 0), "right" (x = 1), "top"
// (y = 1) and "left" (x = 0).
inline std::shared_ptr<const mesh::Mesh> unit_square(std::size_t n) {
  mesh::Mesh m;
  const auto id = [n](std::size_t i, std::size_t j) { return j * (n + 1) + i; };
  for (std::size_t j = 0; j <= n; ++j)
    for (std::size_t i = 0; i <= n; ++i)
      m.nodes.push_back({static_cast<double>(i) / static_cast<double>(n),
                         static_cast<double>(j) / static_cast<double>(n)});
  for (std::size_t j = 0; j < n; ++j)
    for (std::size_t i = 0; i < n; ++i) {
      m.triangles.push_back({{id(i, j), id(i + 1, j), id(i + 1, j + 1)}});
      m.triangles.push_back({{id(i, j), id(i + 1, j + 1), id(i, j + 1)}});
    }
  m.region_names = {"bottom", "right", "top", "left"};
  // Counterclockwise around the square, the domain on the left.
  for (std::size_t i = 0; i < n; ++i)
    m.boundary_edges.push_back(
        {.nodes = {id(i, 0), id(i + 1, 0)}, .region = 0, .link = {}});
  for (std::size_t j = 0; j < n; ++j)
    m.boundary_edges.push_back(
        {.nodes = {id(n, j), id(n, j + 1)}, .region = 1, .link = {}});
  for (std::size_t i = n; i > 0; --i)
    m.boundary_edges.push_back(
        {.nodes = {id(i, n), id(i - 1, n)}, .region = 2, .link = {}});
  for (std::size_t j = n; j > 0; --j)
    m.boundary_edges.push_back(
        {.nodes = {id(0, j), id(0, j - 1)}, .region = 3, .link = {}});
  return std::make_shared<const mesh::Mesh>(std::move(m));
}

// The largest difference between nodal values and a function at the nodes.
inline double
max_nodal_error(const mesh::Mesh &m, const std::vector<double> &values,
                const std::function<double(double, double)> &exact) {
  double error = 0.0;
  for (std::size_t i = 0; i < m.nodes.size(); ++i)
    error = std::max(error,
                     std::abs(values[i] - exact(m.nodes[i].x, m.nodes[i].y)));
  return error;
}

// The L2 error of the piecewise linear function with these nodal values,
// with the edge-midpoint rule on each triangle (exact for degree 2).
inline double l2_error(const mesh::Mesh &m, const std::vector<double> &values,
                       const std::function<double(double, double)> &exact) {
  double sum = 0.0;
  for (const auto &triangle : m.triangles) {
    const auto &p = m.nodes[triangle.nodes[0]], &q = m.nodes[triangle.nodes[1]],
               &r = m.nodes[triangle.nodes[2]];
    const double area =
        ((q.x - p.x) * (r.y - p.y) - (r.x - p.x) * (q.y - p.y)) / 2;
    for (std::size_t k = 0; k < 3; ++k) {
      const std::size_t a = triangle.nodes[k], b = triangle.nodes[(k + 1) % 3];
      const auto &u = m.nodes[a], &v = m.nodes[b];
      const double d =
          (values[a] + values[b]) / 2 - exact((u.x + v.x) / 2, (u.y + v.y) / 2);
      sum += area / 3 * d * d;
    }
  }
  return std::sqrt(sum);
}

// A linear solver that counts how often it is prepared, through the
// public interface only; it solves with SparseLu.
class CountingSolver final : public solve::LinearSolver {
public:
  std::string name() const override { return "counting"; }
  solve::LinearSolverRequirements requirements() const override { return {}; }
  std::unique_ptr<const solve::PreparedLinearSolver>
  prepare(std::shared_ptr<const fem::CsrMatrix> matrix) const override {
    ++*count_;
    return solve::SparseLu().prepare(std::move(matrix));
  }
  std::size_t count() const { return *count_; }

private:
  // Shared so that prepare() can stay const.
  std::shared_ptr<std::size_t> count_ = std::make_shared<std::size_t>(0);
};

} // namespace pde2d::test
