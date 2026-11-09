#pragma once
// Settings of P1 assembly (assembly.hpp), apart from the assembler so plans
// and run records (plan.hpp, record.hpp) can hold them without the symbolic
// preparation and the boundary edge forms.
#include "execution.hpp"

namespace pde2d::fem {
struct AssemblyOptions {
  int quadrature_degree = 2;
  double boundary_tolerance = 1e-10;
  pde2d::ExecutionOptions execution{};
};
} // namespace pde2d::fem
