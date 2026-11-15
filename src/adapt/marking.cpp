// The marking strategies: the shared checks in MarkingStrategy::mark and
// the four strategies' select().

#include "pde2d/adapt/dorfler_marking.hpp"
#include "pde2d/adapt/equidistribution_marking.hpp"
#include "pde2d/adapt/fixed_fraction_marking.hpp"
#include "pde2d/adapt/marking.hpp"
#include "pde2d/adapt/maximum_marking.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace pde2d::adapt {

namespace {

// theta and fraction options: a number in [0, 1]. NaN fails the test too.
void check_unit_interval(double value, const std::string &what) {
  if (!(value >= 0.0 && value <= 1.0))
    throw std::invalid_argument(what + " must be in [0, 1], got " +
                                std::to_string(value));
}

// Triangle indices ordered by decreasing indicator, equal indicators by
// increasing index: the order in which Dörfler and fixed fraction marking
// take triangles. A stable sort of 0, 1, ..., N-1 keeps equal indicators
// in index order.
std::vector<std::size_t> largest_first(std::span<const double> indicators) {
  std::vector<std::size_t> order(indicators.size());
  for (std::size_t t = 0; t < order.size(); ++t)
    order[t] = t;
  std::stable_sort(order.begin(), order.end(),
                   [&](std::size_t a, std::size_t b) {
                     return indicators[a] > indicators[b];
                   });
  return order;
}

} // namespace

std::vector<std::size_t>
MarkingStrategy::mark(const mesh::Mesh &mesh,
                      std::span<const double> indicators) const {
  if (indicators.size() != mesh.triangles.size())
    throw std::invalid_argument(
        name() + " marking: " + std::to_string(indicators.size()) +
        " indicators for " + std::to_string(mesh.triangles.size()) +
        " triangles (one per triangle is needed)");
  bool any_positive = false;
  for (std::size_t t = 0; t < indicators.size(); ++t) {
    if (!std::isfinite(indicators[t]) || indicators[t] < 0.0)
      throw std::invalid_argument(
          name() + " marking: the indicator of triangle " + std::to_string(t) +
          " must be finite and non-negative, got " +
          std::to_string(indicators[t]));
    any_positive = any_positive || indicators[t] > 0.0;
  }
  if (!any_positive)
    return {};
  std::vector<std::size_t> picked = select(indicators);
  // The shared rules: a zero indicator is never marked; the result is
  // sorted and without repetitions.
  std::erase_if(picked, [&](std::size_t t) { return !(indicators[t] > 0.0); });
  std::sort(picked.begin(), picked.end());
  picked.erase(std::unique(picked.begin(), picked.end()), picked.end());
  return picked;
}

// --- maximum ----------------------------------------------------------------

MaximumMarking::MaximumMarking(MaximumMarkingOptions options)
    : options_(options) {
  check_unit_interval(options_.theta, "MaximumMarkingOptions::theta");
}

std::vector<std::size_t>
MaximumMarking::select(std::span<const double> indicators) const {
  const double largest =
      *std::max_element(indicators.begin(), indicators.end());
  const double bound = options_.theta * largest;
  std::vector<std::size_t> picked;
  for (std::size_t t = 0; t < indicators.size(); ++t)
    if (indicators[t] >= bound)
      picked.push_back(t);
  return picked;
}

// --- Dörfler ----------------------------------------------------------------

DorflerMarking::DorflerMarking(DorflerMarkingOptions options)
    : options_(options) {
  check_unit_interval(options_.theta, "DorflerMarkingOptions::theta");
}

std::vector<std::size_t>
DorflerMarking::select(std::span<const double> indicators) const {
  const std::vector<std::size_t> order = largest_first(indicators);
  // Squares of the indicators divided by the largest one (positive here),
  // so that large finite indicators cannot overflow when squared.
  const double largest = indicators[order.front()];
  auto square = [&](std::size_t t) {
    const double r = indicators[t] / largest;
    return r * r;
  };
  // The total in the same order as the partial sums below, so that the
  // partial sum over every triangle is bit for bit the total.
  double total = 0.0;
  for (auto t : order)
    total += square(t);
  const double bound = options_.theta * total;
  std::vector<std::size_t> picked;
  double sum = 0.0;
  for (auto t : order) {
    if (sum >= bound)
      break;
    picked.push_back(t);
    sum += square(t);
  }
  return picked;
}

// --- fixed fraction ---------------------------------------------------------

FixedFractionMarking::FixedFractionMarking(FixedFractionMarkingOptions options)
    : options_(options) {
  check_unit_interval(options_.fraction,
                      "FixedFractionMarkingOptions::fraction");
}

std::vector<std::size_t>
FixedFractionMarking::select(std::span<const double> indicators) const {
  std::vector<std::size_t> order = largest_first(indicators);
  const auto count = static_cast<std::size_t>(
      std::round(options_.fraction * static_cast<double>(indicators.size())));
  order.resize(std::min(count, order.size()));
  return order;
}

// --- equidistribution -------------------------------------------------------

EquidistributionMarking::EquidistributionMarking(
    EquidistributionMarkingOptions options)
    : options_(options) {
  if (!(std::isfinite(options_.tolerance) && options_.tolerance > 0.0))
    throw std::invalid_argument(
        "EquidistributionMarkingOptions::tolerance must be set, finite and "
        "positive, got " +
        std::to_string(options_.tolerance));
}

std::vector<std::size_t>
EquidistributionMarking::select(std::span<const double> indicators) const {
  const double bound =
      options_.tolerance / std::sqrt(static_cast<double>(indicators.size()));
  std::vector<std::size_t> picked;
  for (std::size_t t = 0; t < indicators.size(); ++t)
    if (indicators[t] >= bound)
      picked.push_back(t);
  return picked;
}

} // namespace pde2d::adapt
