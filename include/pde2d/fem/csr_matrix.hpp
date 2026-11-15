#pragma once

#include <cstddef>
#include <memory>
#include <optional>
#include <span>
#include <vector>

namespace pde2d::fem {

// Which entries of a sparse matrix may be nonzero, in compressed sparse row
// (CSR) form: the entries of row i are slots row_offsets[i] to
// row_offsets[i + 1] - 1, and slot s is in column column_indices[s].
// Within a row the columns are sorted and unique. The index type is signed
// so that a solver can view the arrays with Eigen::Map (whose index type
// must be signed) without copying them.
struct SparsityPattern {
  using Index = std::ptrdiff_t;

  std::size_t rows = 0;
  std::size_t columns = 0;
  std::vector<Index> row_offsets{};    // rows + 1 entries, starting at 0
  std::vector<Index> column_indices{}; // one per slot

  std::size_t nonzeros() const { return column_indices.size(); }
  // The slot of entry (row, column), found by binary search in the row, or
  // nothing when the pattern has no such entry. Throws
  // std::invalid_argument for a row or column out of range.
  std::optional<std::size_t> find(std::size_t row, std::size_t column) const;
};

// A sparse matrix: a pattern and one value per slot. Matrices of the same
// kind share one pattern object, so only their values differ.
struct CsrMatrix {
  std::shared_ptr<const SparsityPattern> pattern{};
  std::vector<double> values{};

  std::size_t rows() const { return pattern ? pattern->rows : 0; }
  std::size_t columns() const { return pattern ? pattern->columns : 0; }
  // Entry (row, column): its value, or 0 when the pattern does not hold
  // it. Throws std::invalid_argument for a row or column out of range.
  double entry(std::size_t row, std::size_t column) const;
  // The product of this matrix and `vector` (one value per column), one
  // value per row; each row is summed in slot order. Throws
  // std::invalid_argument when the size of `vector` is not columns().
  std::vector<double> multiply(std::span<const double> vector) const;
};

// Facts read from a matrix's values (docs/decisions.md, "Two kinds of
// facts per matrix"): they hold for these numbers only. A matrix without
// a pattern counts as 0 x 0. Both throw std::invalid_argument when the
// number of values is not the number of slots of the pattern.

// Whether every stored value is zero (-0.0 counts as zero; a NaN does not).
// A matrix with no slots has only zero entries.
[[nodiscard]] bool has_only_zero_entries(const CsrMatrix &matrix);

// Whether the matrix is square and |a_ij - a_ji| <= relative_tolerance *
// max |a| for every entry, where an entry missing from the pattern counts
// as 0. A matrix whose values are all zero is symmetric; one with a value
// that is not finite is not. The default tolerance allows the rounding of
// matrices computed in full, like the assembled ones, whose (i, j) and
// (j, i) can differ in the last bits. Throws std::invalid_argument for a
// tolerance that is negative or not finite.
[[nodiscard]] bool is_symmetric(const CsrMatrix &matrix,
                                double relative_tolerance = 1e-12);

} // namespace pde2d::fem
