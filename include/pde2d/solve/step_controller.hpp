#pragma once

#include <string>

namespace pde2d::solve {

// Decides the length of the next time step. The Integrator may shorten a
// step so that it lands exactly on a snapshot time or on the end time.
// This slice has only FixedStep; an error-controlled controller will need
// more from the step (an error estimate) and may extend this interface.
class StepController {
public:
  virtual ~StepController() = default;
  virtual std::string name() const = 0;
  // The step to take from time t; finite and positive.
  virtual double next_step(double t) const = 0;
};

} // namespace pde2d::solve
