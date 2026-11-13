#pragma once

// Helpers for the assembly tests: small meshes, dense copies of sparse
// matrices and a dense solver. None of them uses the assembly code.

#include "support/check.hpp"

#include "pde2d/fem/csr_matrix.hpp"
#include "pde2d/geometry/curve_geometry.hpp"
#include "pde2d/mesh/cdt.hpp"
#include "pde2d/mesh/chord_length_discretizer.hpp"
#include "pde2d/mesh/mesh.hpp"
#include "pde2d/mesh/mesher.hpp"
#include "pde2d/mesh/ruppert.hpp"

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace pde2d::test {

using Dense = std::vector<std::vector<double>>;

// Every entry of a sparse matrix, read one by one through entry().
inline Dense dense(const fem::CsrMatrix &matrix) {
  Dense result(matrix.rows(), std::vector<double>(matrix.columns(), 0.0));
  for (std::size_t i = 0; i < matrix.rows(); ++i)
    for (std::size_t j = 0; j < matrix.columns(); ++j)
      result[i][j] = matrix.entry(i, j);
  return result;
}

inline std::vector<double> times(const Dense &matrix,
                                 const std::vector<double> &vector) {
  std::vector<double> result(matrix.size(), 0.0);
  for (std::size_t i = 0; i < matrix.size(); ++i)
    for (std::size_t j = 0; j < vector.size(); ++j)
      result[i] += matrix[i][j] * vector[j];
  return result;
}

// Solves matrix x = rhs by Gaussian elimination with partial pivoting.
inline std::vector<double> solve(Dense matrix, std::vector<double> rhs) {
  const std::size_t n = rhs.size();
  for (std::size_t k = 0; k < n; ++k) {
    std::size_t pivot = k;
    for (std::size_t i = k + 1; i < n; ++i)
      if (std::abs(matrix[i][k]) > std::abs(matrix[pivot][k]))
        pivot = i;
    if (matrix[pivot][k] == 0.0)
      throw std::runtime_error("singular matrix");
    std::swap(matrix[k], matrix[pivot]);
    std::swap(rhs[k], rhs[pivot]);
    for (std::size_t i = k + 1; i < n; ++i) {
      const double factor = matrix[i][k] / matrix[k][k];
      if (factor == 0.0)
        continue;
      for (std::size_t j = k; j < n; ++j)
        matrix[i][j] -= factor * matrix[k][j];
      rhs[i] -= factor * rhs[k];
    }
  }
  std::vector<double> x(n, 0.0);
  for (std::size_t k = n; k-- > 0;) {
    double sum = rhs[k];
    for (std::size_t j = k + 1; j < n; ++j)
      sum -= matrix[k][j] * x[j];
    x[k] = sum / matrix[k][k];
  }
  return x;
}

// The triangle (0, 0), (1, 0), (0, 1) with its edges in the regions
// "bottom" (0 -> 1), "diagonal" (1 -> 2) and "left" (2 -> 0).
inline std::shared_ptr<const mesh::Mesh> reference_triangle() {
  mesh::Mesh m;
  m.nodes = {{0, 0}, {1, 0}, {0, 1}};
  m.triangles = {{{0, 1, 2}}};
  m.region_names = {"bottom", "diagonal", "left"};
  m.boundary_edges = {{.nodes = {0, 1}, .region = 0, .link = {}},
                      {.nodes = {1, 2}, .region = 1, .link = {}},
                      {.nodes = {2, 0}, .region = 2, .link = {}}};
  return std::make_shared<const mesh::Mesh>(std::move(m));
}

// The rectangle [0, width] x [0, height] meshed by the Mesher (Ruppert)
// with edges at most `h`; its sides are the regions "bottom", "right",
// "top" and "left".
inline std::shared_ptr<const mesh::Mesh>
rectangle_mesh(double width, double height, double h) {
  using geometry::Line;
  using geometry::Point;
  const Point a{0, 0}, b{width, 0}, c{width, height}, d{0, height};
  geometry::Loop loop;
  loop.curves = {{Line{a, b}, "bottom"},
                 {Line{b, c}, "right"},
                 {Line{c, d}, "top"},
                 {Line{d, a}, "left"}};
  const geometry::CurveGeometry geometry({loop});
  auto result = mesh::Mesher(std::make_shared<mesh::ChordLengthDiscretizer>(),
                             std::make_shared<mesh::Cdt>(),
                             std::make_shared<mesh::Ruppert>())
                    .run(geometry, {.max_edge_length = {.absolute = h}});
  check(result.target_met, "the mesh meets its edge length");
  return std::make_shared<const mesh::Mesh>(std::move(result.mesh));
}

inline double triangle_area(const mesh::Mesh &m, const mesh::Triangle &t) {
  const auto &p = m.nodes[t.nodes[0]], &q = m.nodes[t.nodes[1]],
             &r = m.nodes[t.nodes[2]];
  return ((q.x - p.x) * (r.y - p.y) - (r.x - p.x) * (q.y - p.y)) / 2;
}

} // namespace pde2d::test
