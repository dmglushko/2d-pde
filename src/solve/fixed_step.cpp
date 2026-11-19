#include "pde2d/solve/fixed_step.hpp"

#include <cmath>
#include <stdexcept>

namespace pde2d::solve {

bool StepController::controls_error() const { return false; }

StepDecision StepController::judge(const TakenStep &step) const {
  return {.accept = true,
          .next_dt = next_step(step.t + step.length),
          .error_norm = std::nullopt};
}

FixedStep::FixedStep(FixedStepOptions options) : options_(options) {
  if (!std::isfinite(options_.dt) || !(options_.dt > 0.0))
    throw std::invalid_argument("fixed_step: dt must be finite and positive");
}

} // namespace pde2d::solve
