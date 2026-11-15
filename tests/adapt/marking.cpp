#include "support/check.hpp"

#include "pde2d/adapt/dorfler_marking.hpp"
#include "pde2d/adapt/equidistribution_marking.hpp"
#include "pde2d/adapt/fixed_fraction_marking.hpp"
#include "pde2d/adapt/maximum_marking.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

using namespace pde2d::adapt;
using namespace pde2d::test;
using pde2d::mesh::Mesh;

namespace {

using Marked = std::vector<std::size_t>;

// Marking only looks at the number of triangles.
Mesh mesh_with(std::size_t triangles) {
  Mesh m;
  m.triangles.resize(triangles);
  return m;
}

Marked mark(const MarkingStrategy &s, const std::vector<double> &eta) {
  return s.mark(mesh_with(eta.size()), eta);
}

// Bad indicators are refused the same way by every strategy (the checks
// are in MarkingStrategy::mark).
void check_bad_indicators(const MarkingStrategy &s) {
  const double nan = std::numeric_limits<double>::quiet_NaN(),
               inf = std::numeric_limits<double>::infinity();
  expect_throws<std::invalid_argument>(
      [&] { (void)s.mark(mesh_with(3), std::vector<double>{1, 2}); },
      "2 indicators for 3 triangles");
  expect_throws<std::invalid_argument>(
      [&] { (void)mark(s, {1, -0.5, 2}); },
      "indicator of triangle 1 must be finite and non-negative");
  expect_throws<std::invalid_argument>([&] { (void)mark(s, {1, 2, nan}); },
                                       "indicator of triangle 2");
  expect_throws<std::invalid_argument>([&] { (void)mark(s, {inf, 2, 1}); },
                                       "indicator of triangle 0");
  // All zeros (and no triangles) mark nothing.
  check(mark(s, {0, 0, 0}).empty(), "all zeros mark nothing");
  check(mark(s, {}).empty(), "no triangles mark nothing");
}

void maximum() {
  const std::vector<double> eta{1, 4, 2, 4, 0};
  check(mark(MaximumMarking({.theta = 0.5}), eta) == Marked{1, 2, 3},
        "theta 0.5: at least 2");
  check(mark(MaximumMarking({.theta = 1}), eta) == Marked{1, 3},
        "theta 1: both largest (a tie)");
  check(mark(MaximumMarking({.theta = 0}), eta) == Marked{0, 1, 2, 3},
        "theta 0: every positive indicator");
  check(mark(MaximumMarking({.theta = 0.25}), eta) == Marked{0, 1, 2, 3},
        "theta 0.25: exactly at the bound counts");
  check(mark(MaximumMarking(), eta) == Marked{1, 2, 3}, "default theta 0.5");
  check(mark(MaximumMarking({.theta = 0.5}), {0, 0, 3}) == Marked{2},
        "one positive indicator");
  check_bad_indicators(MaximumMarking());
  for (double theta : {-0.1, 1.1, std::nan("")})
    expect_throws<std::invalid_argument>(
        [&] { MaximumMarking m({.theta = theta}); },
        "MaximumMarkingOptions::theta must be in [0, 1]");
}

void dorfler() {
  // Squares 1, 9, 4, 9, 0: total 23. Order: 1, 3 (a tie, by index), 2, 0.
  const std::vector<double> eta{1, 3, 2, 3, 0};
  check(mark(DorflerMarking({.theta = 0.3}), eta) == Marked{1},
        "theta 0.3 (6.9): one of the tied pair, the lower index");
  check(mark(DorflerMarking({.theta = 0.5}), eta) == Marked{1, 3},
        "theta 0.5 (11.5): both 3s");
  check(mark(DorflerMarking({.theta = 0.78}), eta) == Marked{1, 3},
        "theta 0.78 (17.94 < 18): still both 3s");
  check(mark(DorflerMarking({.theta = 0.79}), eta) == Marked{1, 2, 3},
        "theta 0.79 (18.17): the 2 as well");
  check(mark(DorflerMarking({.theta = 1}), eta) == Marked{0, 1, 2, 3},
        "theta 1: every positive indicator, no zero");
  check(mark(DorflerMarking({.theta = 0}), eta).empty(),
        "theta 0: the empty set holds 0");
  // A tie that decides the set: four equal indicators, half of the total.
  check(mark(DorflerMarking({.theta = 0.5}), {2, 2, 2, 2}) == Marked{0, 1},
        "equal indicators are taken by index");
  // Squares of large indicators would overflow without scaling.
  check(mark(DorflerMarking({.theta = 0.5}), {1e200, 1e300, 1e300}) ==
            Marked{1},
        "large indicators");
  check_bad_indicators(DorflerMarking());
  for (double theta : {-1e-9, 1.5})
    expect_throws<std::invalid_argument>(
        [&] { DorflerMarking m({.theta = theta}); },
        "DorflerMarkingOptions::theta must be in [0, 1]");
}

void fixed_fraction() {
  const std::vector<double> eta{1, 3, 2, 3, 0};
  check(mark(FixedFractionMarking({.fraction = 0.4}), eta) == Marked{1, 3},
        "0.4 of 5 = 2 triangles");
  check(mark(FixedFractionMarking({.fraction = 0.5}), eta) == Marked{1, 2, 3},
        "0.5 of 5 = 2.5, rounded to 3");
  check(mark(FixedFractionMarking({.fraction = 0.1}), eta) == Marked{1},
        "0.1 of 5 = 0.5, rounded to 1; the tie by index");
  check(mark(FixedFractionMarking({.fraction = 0.09}), eta).empty(),
        "0.09 of 5 = 0.45, rounded to 0");
  check(mark(FixedFractionMarking({.fraction = 1}), eta) == Marked{0, 1, 2, 3},
        "fraction 1: every triangle but the zero one");
  check(mark(FixedFractionMarking({.fraction = 0}), eta).empty(), "fraction 0");
  // 0.07 * 100 is 7.000000000000001 in doubles: rounding, not ceil.
  std::vector<double> hundred(100);
  for (std::size_t t = 0; t < hundred.size(); ++t)
    hundred[t] = static_cast<double>(t + 1);
  Marked top7{93, 94, 95, 96, 97, 98, 99};
  check(mark(FixedFractionMarking({.fraction = 0.07}), hundred) == top7,
        "0.07 of 100 = 7 triangles, the largest");
  check(mark(FixedFractionMarking(), hundred).size() == 30,
        "default fraction 0.3");
  check_bad_indicators(FixedFractionMarking());
  expect_throws<std::invalid_argument>(
      [] { FixedFractionMarking m({.fraction = 2}); },
      "FixedFractionMarkingOptions::fraction must be in [0, 1]");
}

void equidistribution() {
  // N = 4, tol = 2: the bound is tol / sqrt(N) = 1.
  const std::vector<double> eta{0.5, 1, 2, 0.99};
  check(mark(EquidistributionMarking({.tolerance = 2}), eta) == Marked{1, 2},
        "at or above 1");
  check(mark(EquidistributionMarking({.tolerance = 100}), eta).empty(),
        "a large tolerance marks nothing");
  check(mark(EquidistributionMarking({.tolerance = 1e-9}), {0, 1, 0, 2}) ==
            Marked{1, 3},
        "a tiny tolerance marks every positive indicator");
  check_bad_indicators(EquidistributionMarking({.tolerance = 1}));
  for (double tol :
       {0.0, -1.0, std::numeric_limits<double>::infinity(), std::nan("")})
    expect_throws<std::invalid_argument>(
        [&] { EquidistributionMarking m({.tolerance = tol}); },
        "EquidistributionMarkingOptions::tolerance must be set, finite and "
        "positive");
  expect_throws<std::invalid_argument>([] { EquidistributionMarking m({}); },
                                       "must be set");
}

void names() {
  check(MaximumMarking().name() == "maximum", "maximum");
  check(DorflerMarking().name() == "dorfler", "dorfler");
  check(FixedFractionMarking().name() == "fixed_fraction", "fixed fraction");
  check(EquidistributionMarking({.tolerance = 1}).name() == "equidistribution",
        "equidistribution");
}

} // namespace

int main() {
  return run({{"maximum", maximum},
              {"dorfler", dorfler},
              {"fixed_fraction", fixed_fraction},
              {"equidistribution", equidistribution},
              {"names", names}});
}
