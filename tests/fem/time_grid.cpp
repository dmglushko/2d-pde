#include "fem/time_grid.hpp"
#include "support/check.hpp"
#include <cmath>
#include <limits>
#include <tuple>
#include <vector>

using pde2d::fem::TimeGrid;
using pde2d::fem::TimeStep;
using pde2d_test::check;
using pde2d_test::expect_throws;
using pde2d_test::near;

namespace {
std::vector<TimeStep> steps(double start, double end, double step) {
  TimeGrid grid(start, end, step);
  std::vector<TimeStep> out;
  while (const auto s = grid.next())
    out.push_back(*s);
  return out;
}

void whole_steps_use_nominal_h() {
  const auto s = steps(0, 1, .1);
  check(s.size() == 10, "Expected ten steps");
  for (std::size_t i = 0; i < s.size(); ++i) {
    check(s[i].index == i + 1, "Step index");
    check(s[i].h == .1, "Full steps must use exactly the nominal step");
  }
  check(s.back().time == 1, "Final stamp must be exactly end");
}

void stamps_do_not_accumulate_rounding() {
  // 0.1 * 3 rounds above 0.3: snap to end instead of adding a tiny step.
  const auto s = steps(0, .3, .1);
  check(s.size() == 3 && s.back().time == .3, "Spurious final step");
  // Far from the origin the rounding allowance must not merge steps.
  const auto far = steps(1e6, 1e6 + 1, .1);
  check(far.size() == 10 && far.back().time == 1e6 + 1, "Large origin");
}

void only_last_step_is_shortened() {
  const auto s = steps(0, 1, .3);
  check(s.size() == 4, "Expected three full steps and one remainder");
  for (int i = 0; i < 3; ++i)
    check(s[i].h == .3, "Full step changed");
  near(s[3].h, .1, 1e-14);
  check(s[3].time == 1, "Shortened step must end exactly at end");
}

void zero_duration_has_no_steps() {
  TimeGrid grid(2, 2, .5);
  check(!grid.next(), "Zero duration produced a step");
  check(grid.time() == 2, "Start stamp");
}

void invalid_intervals_are_rejected() {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (auto [a, b, h] :
       {std::tuple{0., 1., 0.}, std::tuple{0., 1., -1.}, std::tuple{1., 0., .1},
        std::tuple{nan, 1., .1}, std::tuple{0., 1., nan}})
    expect_throws<std::invalid_argument>([&] { TimeGrid(a, b, h); },
                                         "Invalid time interval");
  // A step below the representable spacing at the origin cannot advance.
  TimeGrid stuck(1e16, 1e16 + 4, 1e-3);
  expect_throws<std::invalid_argument>([&] { stuck.next(); }, "cannot advance");
}
} // namespace

int main() {
  return pde2d_test::run(
      {{"whole_steps_use_nominal_h", whole_steps_use_nominal_h},
       {"stamps_do_not_accumulate_rounding", stamps_do_not_accumulate_rounding},
       {"only_last_step_is_shortened", only_last_step_is_shortened},
       {"zero_duration_has_no_steps", zero_duration_has_no_steps},
       {"invalid_intervals_are_rejected", invalid_intervals_are_rejected}});
}
