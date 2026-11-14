#pragma once

// Eigen views of our own arrays, without copying. Only the solver .cpp
// files include this header: Eigen never appears in a public header.

#include "pde2d/fem/csr_matrix.hpp"

#include <Eigen/SparseCore>

#include <cstddef>
#include <span>

namespace pde2d::solve::detail {

// The index type of our patterns is signed (std::ptrdiff_t), as Eigen
// requires.
using RowMajorMatrix =
    Eigen::SparseMatrix<double, Eigen::RowMajor, std::ptrdiff_t>;
using MatrixView = Eigen::Map<const RowMajorMatrix>;
using VectorView = Eigen::Map<const Eigen::VectorXd>;

// A CsrMatrix is exactly Eigen's compressed row-major format: row offsets
// (outer), column indices (inner) and values.
inline MatrixView view(const fem::CsrMatrix &matrix) {
  const auto &pattern = *matrix.pattern;
  return MatrixView(static_cast<std::ptrdiff_t>(pattern.rows),
                    static_cast<std::ptrdiff_t>(pattern.columns),
                    static_cast<std::ptrdiff_t>(pattern.nonzeros()),
                    pattern.row_offsets.data(), pattern.column_indices.data(),
                    matrix.values.data());
}

inline VectorView view(std::span<const double> vector) {
  return VectorView(vector.data(), static_cast<std::ptrdiff_t>(vector.size()));
}

} // namespace pde2d::solve::detail
