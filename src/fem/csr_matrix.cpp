#include "pde2d/fem/csr_matrix.hpp"

#include <algorithm>
#include <format>
#include <stdexcept>

namespace pde2d::fem {

std::optional<std::size_t> SparsityPattern::find(std::size_t row,
                                                 std::size_t column) const {
  if (row >= rows || column >= columns)
    throw std::invalid_argument(
        std::format("Entry ({}, {}) is outside the {} x {} matrix", row, column,
                    rows, columns));
  const auto begin = column_indices.begin() + row_offsets[row];
  const auto end = column_indices.begin() + row_offsets[row + 1];
  const auto found = std::lower_bound(begin, end, static_cast<Index>(column));
  if (found == end || *found != static_cast<Index>(column))
    return std::nullopt;
  return static_cast<std::size_t>(found - column_indices.begin());
}

double CsrMatrix::entry(std::size_t row, std::size_t column) const {
  if (!pattern)
    throw std::invalid_argument(
        std::format("Entry ({}, {}) is outside the empty matrix", row, column));
  const auto slot = pattern->find(row, column);
  return slot ? values[*slot] : 0.0;
}

std::vector<double> CsrMatrix::multiply(std::span<const double> vector) const {
  if (vector.size() != columns())
    throw std::invalid_argument(
        std::format("Cannot multiply a {} x {} matrix by a vector of size {}",
                    rows(), columns(), vector.size()));
  std::vector<double> product(rows(), 0.0);
  for (std::size_t row = 0; row < rows(); ++row) {
    double sum = 0.0;
    for (auto slot = pattern->row_offsets[row];
         slot < pattern->row_offsets[row + 1]; ++slot) {
      const auto s = static_cast<std::size_t>(slot);
      sum += values[s] *
             vector[static_cast<std::size_t>(pattern->column_indices[s])];
    }
    product[row] = sum;
  }
  return product;
}

} // namespace pde2d::fem
