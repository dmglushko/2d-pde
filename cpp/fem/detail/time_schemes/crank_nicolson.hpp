#pragma once
// TimeMethod::CrankNicolson. Endpoint trapezoidal residual: M1 at each
// endpoint multiplies the common difference quotient, not a midpoint
// evaluation:
//   (M1avg + h/2 K1) u1 = M1avg u0 - h/2 K0 u0 + h/2 (F0 + F1).
#include "fem/detail/time_schemes/interface.hpp"
#include <utility>

namespace pde2d::fem {
class CrankNicolsonIntegrator;
namespace time_scheme_detail {
struct CrankNicolsonScheme : UnitBase<CrankNicolsonScheme> {
  using Integrator = CrankNicolsonIntegrator;
  static constexpr TimeMethod method = TimeMethod::CrankNicolson;
  static constexpr const char *name = "CrankNicolson";
  static constexpr int order = 1;
};
} // namespace time_scheme_detail

class CrankNicolsonIntegrator final
    : public time_scheme_detail::BuiltinScheme<
          time_scheme_detail::CrankNicolsonScheme,
          time_scheme_detail::FirstOrder> {
public:
  IntegratorState advance(const AssembledSystem &old,
                          const AssembledSystem &cur, double h,
                          const IntegratorState &s,
                          const IntegratorServices &services) override {
    SparseMatrix average_mass = (old.first_mass + cur.first_mass) * .5;
    SparseMatrix a = average_mass + (h / 2) * cur.stiffness;
    Vector rhs = average_mass * s.u - (h / 2) * (old.stiffness * s.u) +
                 (h / 2) * (old.load + cur.load);
    Vector u = services.solve(a, rhs, cur.dirichlet, true);
    Vector v = (u - s.u) / h;
    return {std::move(u), std::move(v)};
  }
};
} // namespace pde2d::fem
