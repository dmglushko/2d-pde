#pragma once
// Step sequence of an evolution, as a strategy.
//
// evolve() asks a StepController for each next step and tells it the outcome
// of every accepted step. The default, UniformStepController, is the fixed
// TimeGrid (time_grid.hpp); a user or future adaptive controller plugs in via
// TimeOptions::step_controller without editing evolution.hpp.
//
// Contract checked by evolve(): steps have consecutive indices starting at 1,
// strictly increasing finite stamps no later than TimeOptions::end, a finite
// positive h, and the last step ends exactly at TimeOptions::end. `h` is what
// the integrator uses; it may differ from the stamp difference only by
// rounding (TimeGrid computes stamps from the origin), within the bound of
// step_size_consistent() below.
//
// Trial, accept, reject: every step is first a trial. judge() sees the
// trial state and may reject it; the driver then discards it (the
// integrator's reject(), no frame, no observer call, counted in
// EvolutionTimings::rejected_steps) and calls next() again, which proposes
// the retry: the same index from the same previous stamp, typically with a
// smaller h. An accepted trial is committed and reported to accepted().
// Rejection needs a transactional integrator (TimeIntegrator::transactional).
// The driver caps retries: more than TimeOptions::max_consecutive_rejections
// rejected trials in a row (default default_max_consecutive_rejections)
// throw std::runtime_error ("rejected N consecutive trial steps").
#include "fem/time_grid.hpp"
#include "fem/time_integration.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <locale>
#include <memory>
#include <optional>
#include <sstream>
#include <string>

namespace pde2d::fem {
// Default bound on consecutive rejected trials of one step. Halving h 100
// times shrinks it by 2^-100, far below any useful step, so a controller
// that hits it is not converging.
inline constexpr std::size_t default_max_consecutive_rejections = 100;

// The rounding bound between a step's h and its stamp difference:
//
//   |h - (time - previous)| <= 32 eps (max(|time|, |previous|, end - start) +
//   h)
//
// with eps the double machine epsilon. It covers stamps computed from the
// origin and TimeGrid's snapping of the final step (at most 8 eps times the
// interval length); a controller that adds h to the previous stamp is well
// inside it. evolve() rejects steps outside it with std::invalid_argument.
inline bool step_size_consistent(const TimeStep &step, double previous,
                                 double start, double end) {
  const double scale = std::max({std::abs(step.time), std::abs(previous),
                                 std::abs(end - start)}) +
                       step.h;
  return std::abs(step.h - (step.time - previous)) <=
         32 * std::numeric_limits<double>::epsilon() * scale;
}

// A controller's judgement of one trial step.
enum class StepVerdict { Accept, Reject };

// What the driver shows the controller for one step: the state before it
// and the trial (or accepted) state after it, after Dirichlet values were
// imposed.
struct StepOutcome {
  const TimeStep &step;
  const IntegratorState &previous;
  const IntegratorState &current;
};

// Controllers may keep state, so use one instance per evolution.
class StepController {
public:
  virtual ~StepController() = default;
  // Name used in decision records and reports.
  virtual std::string name() const = 0;
  // The next step, or nothing once the evolution has reached its end.
  virtual std::optional<TimeStep> next() = 0;
  // Judge one trial. Reject to discard it; the next call to next() then
  // proposes the retry. The default accepts every trial.
  virtual StepVerdict judge(const StepOutcome &) { return StepVerdict::Accept; }
  // Called once per accepted step, before the next call to next().
  virtual void accepted(const StepOutcome &) {}
};

// Fixed step on [start, end] with TimeGrid's stamps and last-step snapping.
class UniformStepController final : public StepController {
  TimeGrid grid_;

public:
  UniformStepController(double start, double end, double step)
      : grid_(start, end, step) {}
  // The name of a uniform controller with this step, without building it.
  static std::string describe(double step) {
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << "Uniform(step=" << step << ')';
    return out.str();
  }
  std::string name() const override { return describe(grid_.step()); }
  std::optional<TimeStep> next() override { return grid_.next(); }
};

// Creates the step controller of one evolution.
using StepControllerFactory = std::function<std::unique_ptr<StepController>()>;
} // namespace pde2d::fem
