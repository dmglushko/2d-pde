#pragma once

#include "pde2d/solve/step_controller.hpp"

#include <limits>
#include <span>

namespace pde2d::solve {

// The options of AdaptiveStep (docs/decisions.md, "The error-controlled
// step"). Set with designated initializers:
// AdaptiveStep({.initial_dt = 1e-3, .atol = 1e-8, .rtol = 1e-4}).
struct AdaptiveStepOptions {
  // The first step of a run that does not continue a stretch of steps
  // (StepAnchor::next_dt), and of the steps the method cannot estimate
  // yet (ErrorEstimateDeclaration::previous_steps). Required: finite,
  // positive, within [min_dt, max_dt].
  double initial_dt = 0.0;
  // The tolerances of the error norm: entry i of the method's local error
  // is divided by atol + rtol max(|u_i before|, |u_i after|), and the norm
  // is the root mean square of these; a step is accepted when it is at
  // most 1. atol finite and positive (so the scale is never 0), rtol
  // finite and at least 0.
  double atol = 1e-6;
  double rtol = 1e-3;
  // The safety factor of the proposal dt safety (1 / err)^(1 / (p + 1));
  // in (0, 1].
  double safety = 0.9;
  // The limits of one change of dt: the proposal is at least min_factor
  // and at most max_factor times dt. min_factor in (0, 1), max_factor
  // above 1 and finite.
  double min_factor = 0.2;
  double max_factor = 5.0;
  // The band in which an accepted step keeps dt: a proposal from
  // min_keep_factor dt to max_keep_factor dt gives dt again, since a new
  // dt makes the linear solver prepare again. min_keep_factor in
  // [min_factor, 1], max_keep_factor in [1, max_factor].
  double min_keep_factor = 0.9;
  double max_keep_factor = 1.5;
  // The bounds of dt. A proposal is clamped to them, also the shorter try
  // after a rejection (at least min_dt); a rejected step that was not
  // longer than min_dt leaves no shorter try and throws
  // std::runtime_error. min_dt
  // finite and at least 0 (0: only the Integrator's own limit, a step too
  // small to change t), max_dt positive (infinity: no bound).
  double min_dt = 0.0;
  double max_dt = std::numeric_limits<double>::infinity();
};

// The error-controlled step (docs/decisions.md, "Adaptive time steps are
// another StepController", "Adaptive time steps: each method estimates its
// own error", "The error-controlled step"). After every step the time
// method estimates the step's local error (TimeMethod::local_error, from
// states it already has, no extra solve); the controller measures it in
// the norm of the options and
//
// - rejects the step when the norm is above 1: the Integrator takes it
//   again from the same state with dt safety (1 / err)^(1 / (p + 1)), at
//   least min_factor times its length (always shorter than the rejected
//   step) and at least min_dt; when the rejected step was not longer
//   than min_dt, no shorter try is left and it throws
//   std::runtime_error;
// - accepts it otherwise and proposes dt safety (1 / err)^(1 / (p + 1))
//   for the next step (p the method's order), limited to [min_factor dt,
//   max_factor dt] and to [min_dt, max_dt]; a proposal within
//   [min_keep_factor dt, max_keep_factor dt] keeps dt.
//
// dt is the controller's step, not the shortened length of a step that
// landed on a snapshot time or t1: a landing step's proposal is computed
// from its own length and error, but limited relative to dt, so a short
// landing does not shrink the next step. A step the method cannot
// estimate yet (the first steps of a run, for a method that reads earlier
// steps) is accepted and keeps dt.
//
// The Integrator refuses, before the first step, a time method that does
// not declare an error estimate at the problem's time order
// (TimeMethod::error_estimate), with std::invalid_argument naming the
// method. The controller is stateless (const, shareable): what it
// remembers between steps is in the Integrator's run and, across runs, in
// StepAnchor (next_dt, previous_steps).
class AdaptiveStep final : public StepController {
public:
  // Throws std::invalid_argument for options outside the ranges above.
  explicit AdaptiveStep(AdaptiveStepOptions options);
  const AdaptiveStepOptions &options() const { return options_; }

  std::string name() const override { return "adaptive_step"; }
  // The first step: initial_dt.
  double next_step(double) const override { return options_.initial_dt; }
  bool controls_error() const override { return true; }
  StepDecision judge(const TakenStep &step) const override;

  // The error norm of the options: the root mean square of error_i /
  // (atol + rtol max(|before_i|, |after_i|)); 0 for no entries. The three
  // have the same size (std::invalid_argument otherwise).
  double error_norm(std::span<const double> before,
                    std::span<const double> after,
                    std::span<const double> error) const;

private:
  AdaptiveStepOptions options_;
};

} // namespace pde2d::solve
