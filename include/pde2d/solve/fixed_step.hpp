#pragma once

#include "pde2d/solve/step_controller.hpp"

namespace pde2d::solve {

struct FixedStepOptions {
  // The step length; finite and positive.
  double dt = 0.0;
};

// Always the same step: FixedStep({.dt = 0.01}).
class FixedStep final : public StepController {
public:
  // Throws std::invalid_argument for a dt that is not finite and positive.
  explicit FixedStep(FixedStepOptions options);
  const FixedStepOptions &options() const { return options_; }

  std::string name() const override { return "fixed_step"; }
  double next_step(double) const override { return options_.dt; }

private:
  FixedStepOptions options_;
};

} // namespace pde2d::solve
