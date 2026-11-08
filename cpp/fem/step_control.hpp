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
// the integrator uses; it may differ from the stamp difference by rounding
// (TimeGrid computes stamps from the origin).
//
// Steps are not rejected or retried yet: accepted() only observes. A
// controller that needs to reject steps will need integrators that can
// restore their internal state, which is a separate change.
#include "fem/time_grid.hpp"
#include "fem/time_integration.hpp"
#include <functional>
#include <locale>
#include <memory>
#include <optional>
#include <sstream>
#include <string>

namespace pde2d::fem {
// What the driver reports after accepting one step: the state before and
// after it (after Dirichlet values were imposed).
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
  // Called once per accepted step, before the next call to next().
  virtual void accepted(const StepOutcome &) {}
};

// Fixed step on [start, end] with TimeGrid's stamps and last-step snapping.
class UniformStepController final : public StepController {
  TimeGrid grid_;

public:
  UniformStepController(double start, double end, double step)
      : grid_(start, end, step) {}
  std::string name() const override {
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << "Uniform(step=" << grid_.step() << ')';
    return out.str();
  }
  std::optional<TimeStep> next() override { return grid_.next(); }
};

// Creates the step controller of one evolution.
using StepControllerFactory = std::function<std::unique_ptr<StepController>()>;
} // namespace pde2d::fem
