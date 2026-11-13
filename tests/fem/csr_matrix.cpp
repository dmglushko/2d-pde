#include "pde2d/fem/csr_matrix.hpp"
#include "support/check.hpp"

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
  });
}
