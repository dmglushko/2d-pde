#pragma once
// TimeMethod::BackwardEuler: (M1 + h K) u1 = M1 u0 + h F1.
#include "fem/detail/time_schemes/interface.hpp"
#include <utility>

namespace pde2d::fem {
class BackwardEulerIntegrator;
namespace time_scheme_detail {
struct BackwardEulerScheme : UnitBase<BackwardEulerScheme> {
  using Integrator = BackwardEulerIntegrator;
  static constexpr TimeMethod method = TimeMethod::BackwardEuler;
  static constexpr const char *name = "BackwardEuler";
  static constexpr int order = 1;
};
} // namespace time_scheme_detail

class BackwardEulerIntegrator final
    : public time_scheme_detail::BuiltinScheme<
          time_scheme_detail::BackwardEulerScheme,
          time_scheme_detail::FirstOrder> {
public:
  IntegratorState advance(const AssembledSystem &, const AssembledSystem &cur,
                          double h, const IntegratorState &s,
                          const IntegratorServices &services) override {
    SparseMatrix a = cur.first_mass + h * cur.stiffness;
    Vector rhs = cur.first_mass * s.u + h * cur.load;
    Vector u = services.solve(a, rhs, cur.dirichlet, true);
    Vector v = (u - s.u) / h;
    return {std::move(u), std::move(v)};
  }
};
} // namespace pde2d::fem
