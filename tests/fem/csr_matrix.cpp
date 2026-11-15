#include "pde2d/fem/csr_matrix.hpp"
#include "support/check.hpp"

#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

using namespace pde2d::fem;
using pde2d::test::check;
using pde2d::test::expect_throws;

namespace {

// [[1, 0, 2],
//  [0, 0, 0],
//  [3, 4, 0]] with an explicit zero at (2, 2) in the pattern.
CsrMatrix example() {
  auto pattern = std::make_shared<SparsityPattern>();
  pattern->rows = 3;
  pattern->columns = 3;
  pattern->row_offsets = {0, 2, 2, 5};
  pattern->column_indices = {0, 2, 0, 1, 2};
  return {.pattern = pattern, .values = {1, 2, 3, 4, 0}};
}

// A square matrix with one slot per listed (row, column, value), rows in
// order and columns sorted within a row.
struct Entry {
  std::size_t row, column;
  double value;
};
CsrMatrix from_entries(std::size_t rows, std::size_t columns,
                       const std::vector<Entry> &entries) {
  auto pattern = std::make_shared<SparsityPattern>();
  pattern->rows = rows;
  pattern->columns = columns;
  pattern->row_offsets.assign(rows + 1, 0);
  CsrMatrix m;
  for (const auto &entry : entries) {
    pattern->column_indices.push_back(
        static_cast<SparsityPattern::Index>(entry.column));
    m.values.push_back(entry.value);
    ++pattern->row_offsets[entry.row + 1];
  }
  for (std::size_t i = 0; i < rows; ++i)
    pattern->row_offsets[i + 1] += pattern->row_offsets[i];
  m.pattern = pattern;
  return m;
}

} // namespace

int main() {
  return pde2d::test::run({
      {"entries are read through the pattern",
       [] {
         const CsrMatrix m = example();
         check(m.rows() == 3 && m.columns() == 3, "size");
         check(m.pattern->nonzeros() == 5, "slots");
         check(m.entry(0, 0) == 1 && m.entry(0, 2) == 2 && m.entry(2, 1) == 4,
               "stored entries");
         check(m.entry(0, 1) == 0 && m.entry(1, 1) == 0 && m.entry(2, 2) == 0,
               "entries outside the pattern or zero");
         check(m.pattern->find(2, 1) == 3u, "slot of (2, 1)");
         check(!m.pattern->find(1, 0), "empty row");
         expect_throws<std::invalid_argument>([&] { (void)m.entry(3, 0); },
                                              "outside the 3 x 3 matrix");
         expect_throws<std::invalid_argument>(
             [&] { (void)CsrMatrix{}.entry(0, 0); }, "empty matrix");
       }},
      {"multiply",
       [] {
         const CsrMatrix m = example();
         const std::vector<double> x{1, 10, 100};
         check(m.multiply(x) == std::vector<double>{201, 0, 43}, "product");
         expect_throws<std::invalid_argument>(
             [&] { (void)m.multiply(std::vector<double>{1, 2}); },
             "Cannot multiply a 3 x 3 matrix by a vector of size 2");
         check(CsrMatrix{}.multiply({}).empty(), "empty matrix");
       }},
      {"has_only_zero_entries",
       [] {
         check(!has_only_zero_entries(example()), "the example has values");
         CsrMatrix zeros = example();
         zeros.values = {0.0, -0.0, 0.0, 0.0, 0.0};
         check(has_only_zero_entries(zeros), "zeros and a negative zero");
         zeros.values[3] = 1e-300;
         check(!has_only_zero_entries(zeros), "a tiny value is not zero");
         zeros.values[3] = std::nan("");
         check(!has_only_zero_entries(zeros), "a NaN is not zero");
         check(has_only_zero_entries(CsrMatrix{}), "no pattern: 0 x 0");
         check(has_only_zero_entries(from_entries(2, 2, {})), "no slots");
         CsrMatrix broken = example();
         broken.values.pop_back();
         expect_throws<std::invalid_argument>(
             [&] { (void)has_only_zero_entries(broken); },
             "has_only_zero_entries: the matrix has 4 values for 5 slots");
       }},
      {"is_symmetric",
       [] {
         // [[2, -1, 0], [-1, 2, -1], [0, -1, 2]] with every slot stored.
         const CsrMatrix tridiagonal = from_entries(3, 3,
                                                    {{0, 0, 2},
                                                     {0, 1, -1},
                                                     {1, 0, -1},
                                                     {1, 1, 2},
                                                     {1, 2, -1},
                                                     {2, 1, -1},
                                                     {2, 2, 2}});
         check(is_symmetric(tridiagonal), "a symmetric matrix");
         check(!is_symmetric(example()), "the example is not");
         // A slot whose mirror is missing from the pattern: the mirror
         // counts as 0, from either side.
         check(!is_symmetric(from_entries(2, 2, {{0, 1, 0.5}})),
               "(0, 1) = 0.5, (1, 0) missing");
         check(!is_symmetric(from_entries(2, 2, {{1, 0, 0.5}})),
               "(1, 0) = 0.5, (0, 1) missing");
         check(is_symmetric(from_entries(2, 2, {{0, 0, 1}, {0, 1, 0.0}})),
               "an explicit zero whose mirror is missing");
         // The tolerance is relative to the largest |a| (here 4).
         const auto nearly = [](double difference) {
           return from_entries(
               2, 2, {{0, 0, 4}, {0, 1, 1 + difference}, {1, 0, 1}, {1, 1, 3}});
         };
         check(is_symmetric(nearly(3e-12)), "3e-12 <= 1e-12 * 4");
         check(!is_symmetric(nearly(5e-12)), "5e-12 > 1e-12 * 4");
         check(is_symmetric(nearly(5e-12), 1e-11), "a looser tolerance");
         check(is_symmetric(nearly(0.0), 0.0), "exactly, with tolerance 0");
         check(!is_symmetric(nearly(1e-15), 0.0), "not exactly");
         // Not square, empty, all zero, not finite.
         check(!is_symmetric(from_entries(2, 3, {{0, 0, 1}})), "2 x 3");
         check(is_symmetric(CsrMatrix{}), "no pattern: 0 x 0");
         check(is_symmetric(from_entries(2, 2, {{0, 1, 0.0}, {1, 1, -0.0}})),
               "all zero");
         CsrMatrix with_nan = tridiagonal;
         with_nan.values[3] = std::nan("");
         check(!is_symmetric(with_nan), "a NaN on the diagonal");
         CsrMatrix with_inf = tridiagonal;
         with_inf.values[1] = with_inf.values[2] = INFINITY;
         check(!is_symmetric(with_inf), "an infinite pair");
         // Bad input.
         expect_throws<std::invalid_argument>(
             [&] { (void)is_symmetric(tridiagonal, -1e-12); },
             "is_symmetric: the relative tolerance -1e-12 must be finite and "
             "not negative");
         expect_throws<std::invalid_argument>(
             [&] { (void)is_symmetric(tridiagonal, std::nan("")); },
             "must be finite");
         CsrMatrix broken = tridiagonal;
         broken.values.push_back(0.0);
         expect_throws<std::invalid_argument>(
             [&] { (void)is_symmetric(broken); },
             "is_symmetric: the matrix has 8 values for 7 slots");
       }},
  });
}
