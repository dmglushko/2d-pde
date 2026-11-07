#pragma once
// Time stamps and step sizes of a fixed-step evolution on [start, end].
//
// Stamps are computed from the origin (start + k*step via fma) so no rounding
// accumulates. The final stamp is exactly `end`; when the interval is not a
// whole number of steps, only that last step is shortened. A remainder within
// rounding of a full step is snapped to `end` instead of producing a spurious
// tiny step.
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <stdexcept>

namespace pde2d::fem {
struct TimeStep {
  std::size_t index; // 1 for the first step
  double time;       // stamp at the end of the step
  double h;          // step size used by the integrator
};

class TimeGrid {
  double start_, end_, step_, time_;
  std::size_t index_ = 1;

public:
  TimeGrid(double start, double end, double step)
      : start_(start), end_(end), step_(step), time_(start) {
    if (!std::isfinite(start) || !std::isfinite(end) || !std::isfinite(step) ||
        step <= 0 || end < start || !std::isfinite(end - start))
      throw std::invalid_argument("Invalid time interval or step");
  }
  double start() const noexcept { return start_; }
  double end() const noexcept { return end_; }
  double step() const noexcept { return step_; }
  // Stamp reached so far (start before the first call to next()).
  double time() const noexcept { return time_; }

  // The next step, or nothing once `end` has been reached.
  std::optional<TimeStep> next() {
    if (!(time_ < end_))
      return std::nullopt;
    const double candidate =
        std::fma(static_cast<double>(index_), step_, start_);
    const double duration = end_ - start_;
    const double elapsed = static_cast<double>(index_) * step_;
    // A large absolute time origin must not turn the rounding allowance into
    // several valid steps. Scale by duration and cap below a requested step.
    const double snap =
        std::min(step_ / 8, 8 * std::numeric_limits<double>::epsilon() *
                                std::max(duration, std::abs(elapsed)));
    const double next =
        candidate >= end_ || duration - elapsed <= snap ? end_ : candidate;
    // Time stamps are rounded separately from the numerical step. Full steps
    // use exactly the nominal h; only a genuinely shortened endpoint changes
    // it.
    const double remainder = duration - static_cast<double>(index_ - 1) * step_;
    const double h =
        next == end_ && std::abs(remainder - step_) > snap ? remainder : step_;
    if (!(next > time_) || !(h > 0) || !std::isfinite(h))
      throw std::invalid_argument(
          "Time step cannot advance representable time");
    time_ = next;
    return TimeStep{index_++, next, h};
  }
};
} // namespace pde2d::fem
