#pragma once

#include <cstddef>
#include <optional>
#include <span>
#include <string>

namespace pde2d::solve {

// What the Integrator tells an error-controlling StepController about a
// step it just took (StepController::judge).
struct TakenStep {
  // The time at the start of the step.
  double t = 0.0;
  // The controller's step for it: next_step() for the first step of a run,
  // else the previous decision's next_dt.
  double dt = 0.0;
  // The step's length as taken: dt, or less when the step was shortened to
  // land on a snapshot time or t1.
  double length = 0.0;
  // The time method's order p (ErrorEstimateDeclaration::order): its local
  // error behaves like length^(p + 1).
  int order = 1;
  // The values at the start and at the end of the step, in the free
  // unknowns.
  std::span<const double> values_before{};
  std::span<const double> values_after{};
  // The method's estimate of the step's local error in the values, one
  // entry per free unknown (TimeMethod::local_error); none when the method
  // cannot estimate it yet (it reads earlier steps the run does not have).
  std::optional<std::span<const double>> local_error{};
};

// The controller's answer about a step.
struct StepDecision {
  // false: the Integrator takes the step again from the same state, with
  // next_dt, which must be smaller than the step's length.
  bool accept = true;
  // The step to take next (after a rejection: the same step again); finite
  // and positive.
  double next_dt = 0.0;
  // The error norm the decision was made with (StepRecord::error_norm);
  // none when the step had no estimate.
  std::optional<double> error_norm{};
};

// Decides the length of the next time step. The Integrator may shorten a
// step so that it lands exactly on a snapshot time or on the end time.
//
// Two kinds (docs/decisions.md, "Adaptive time steps are another
// StepController"), told apart by controls_error(), never by the type:
// - a controller that does not control the error (FixedStep) is asked
//   next_step(t) before every step; judge() is never called and every
//   step is accepted;
// - an error-controlling one (AdaptiveStep) is asked next_step(t0) for the
//   first step of a run (unless the run continues a stretch of steps that
//   carries its next dt, StepAnchor::next_dt), and judge() after every
//   step, with the method's local error estimate; it may reject the step.
//   The Integrator then refuses a time method that declares no error
//   estimate (TimeMethod::error_estimate) before the first step.
// A controller is const and may be shared between Integrators and runs:
// what a run remembers is kept by the Integrator.
class StepController {
public:
  virtual ~StepController() = default;
  virtual std::string name() const = 0;
  // The step to take from time t; finite and positive.
  virtual double next_step(double t) const = 0;
  // Whether the controller judges every step by the time method's local
  // error estimate (and may reject it). The default is false.
  virtual bool controls_error() const;
  // The decision about a step just taken; called only when
  // controls_error() holds. The default accepts it and proposes
  // next_step(step.t + step.length).
  virtual StepDecision judge(const TakenStep &step) const;
};

} // namespace pde2d::solve
