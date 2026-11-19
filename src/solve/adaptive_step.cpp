#include "pde2d/solve/adaptive_step.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <stdexcept>
#include <string>

namespace pde2d::solve {

namespace {

constexpr const char *prefix = "adaptive_step";

void require(bool condition, const std::string &what) {
  if (!condition)
    throw std::invalid_argument(std::format("{}: {}", prefix, what));
}

} // namespace

AdaptiveStep::AdaptiveStep(AdaptiveStepOptions options) : options_(options) {
  const AdaptiveStepOptions &o = options_;
  require(std::isfinite(o.atol) && o.atol > 0.0,
          std::format("atol must be finite and positive, got {}", o.atol));
  require(std::isfinite(o.rtol) && o.rtol >= 0.0,
          std::format("rtol must be finite and at least 0, got {}", o.rtol));
  require(o.safety > 0.0 && o.safety <= 1.0,
          std::format("safety must be in (0, 1], got {}", o.safety));
  require(o.min_factor > 0.0 && o.min_factor < 1.0,
          std::format("min_factor must be in (0, 1), got {}", o.min_factor));
  require(std::isfinite(o.max_factor) && o.max_factor > 1.0,
          std::format("max_factor must be finite and above 1, got {}",
                      o.max_factor));
  require(o.min_keep_factor >= o.min_factor && o.min_keep_factor <= 1.0,
          std::format("min_keep_factor must be in [min_factor, 1], got {}",
                      o.min_keep_factor));
  require(o.max_keep_factor >= 1.0 && o.max_keep_factor <= o.max_factor,
          std::format("max_keep_factor must be in [1, max_factor], got {}",
                      o.max_keep_factor));
  require(
      std::isfinite(o.min_dt) && o.min_dt >= 0.0,
      std::format("min_dt must be finite and at least 0, got {}", o.min_dt));
  require(o.max_dt > 0.0 && !std::isnan(o.max_dt),
          std::format("max_dt must be positive, got {}", o.max_dt));
  require(o.min_dt <= o.max_dt,
          std::format("min_dt {} is above max_dt {}", o.min_dt, o.max_dt));
  require(std::isfinite(o.initial_dt) && o.initial_dt > 0.0,
          std::format("initial_dt must be finite and positive, got {}",
                      o.initial_dt));
  require(o.initial_dt >= o.min_dt && o.initial_dt <= o.max_dt,
          std::format("initial_dt {} is not within [min_dt, max_dt] = [{}, "
                      "{}]",
                      o.initial_dt, o.min_dt, o.max_dt));
}

double AdaptiveStep::error_norm(std::span<const double> before,
                                std::span<const double> after,
                                std::span<const double> error) const {
  if (before.size() != error.size() || after.size() != error.size())
    throw std::invalid_argument(std::format(
        "{}: the error has {} entries for {} values before and {} after",
        prefix, error.size(), before.size(), after.size()));
  if (error.empty())
    return 0.0;
  double sum = 0.0;
  for (std::size_t i = 0; i < error.size(); ++i) {
    const double scale =
        options_.atol +
        options_.rtol * std::max(std::abs(before[i]), std::abs(after[i]));
    const double scaled = error[i] / scale;
    sum += scaled * scaled;
  }
  return std::sqrt(sum / static_cast<double>(error.size()));
}

StepDecision AdaptiveStep::judge(const TakenStep &step) const {
  const AdaptiveStepOptions &o = options_;
  // A step the method cannot estimate yet keeps dt.
  if (!step.local_error)
    return {.accept = true, .next_dt = step.dt, .error_norm = std::nullopt};
  const double err =
      error_norm(step.values_before, step.values_after, *step.local_error);
  // The elementary controller: the local error behaves like
  // length^(p + 1), so the length that would give err = 1 is
  // length (1 / err)^(1 / (p + 1)), times the safety factor (Hairer,
  // Norsett and Wanner, Solving ODEs I, section II.4).
  const double exponent = 1.0 / static_cast<double>(step.order + 1);
  if (!(err <= 1.0)) {
    // Rejected (also for an error that is not a number): the same step
    // again, shorter than the rejected one by at least the safety factor
    // and at most min_factor, but not below min_dt. A rejected step that
    // was not longer than min_dt leaves no shorter try.
    if (step.length <= o.min_dt)
      throw std::runtime_error(std::format(
          "{}: the step from t = {} of length {} was rejected (error norm "
          "{}), and it is not longer than min_dt = {}, so no shorter step "
          "may be tried",
          prefix, step.t, step.length, err, o.min_dt));
    const double factor =
        std::isfinite(err)
            ? std::max(o.min_factor, o.safety * std::pow(1.0 / err, exponent))
            : o.min_factor;
    const double next = std::max(step.length * factor, o.min_dt);
    return {.accept = false, .next_dt = next, .error_norm = err};
  }
  // Accepted. The proposal from the step's own length and error, limited
  // relative to dt (not to a landing's shorter length), then to the dt
  // bounds; a small change keeps dt, so the step matrix need not be
  // prepared again.
  const double proposal =
      err > 0.0 ? step.length * o.safety * std::pow(1.0 / err, exponent)
                : step.dt * o.max_factor;
  const double next = std::clamp(
      std::clamp(proposal, o.min_factor * step.dt, o.max_factor * step.dt),
      o.min_dt, o.max_dt);
  const bool keep = next >= o.min_keep_factor * step.dt &&
                    next <= o.max_keep_factor * step.dt;
  return {.accept = true, .next_dt = keep ? step.dt : next, .error_norm = err};
}

} // namespace pde2d::solve
