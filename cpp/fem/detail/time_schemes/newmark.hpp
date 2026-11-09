#pragma once
// TimeMethod::Newmark. Average acceleration (beta = 1/4, gamma = 1/2), solved
// for the new acceleration. Dirichlet displacement data become acceleration
// constraints.
#include "fem/detail/time_schemes/interface.hpp"
#include <utility>

namespace pde2d::fem {
class NewmarkIntegrator;
namespace time_scheme_detail {
struct NewmarkScheme : UnitBase<NewmarkScheme> {
  using Integrator = NewmarkIntegrator;
  static constexpr TimeMethod method = TimeMethod::Newmark;
  static constexpr const char *name = "Newmark";
  static constexpr int order = 2;
};
} // namespace time_scheme_detail

class NewmarkIntegrator final : public time_scheme_detail::BuiltinScheme<
                                    time_scheme_detail::NewmarkScheme> {
  Vector acceleration_;

public:
  void check(const AssembledSystem &s) const override {
    if (time_order(s) != 2)
      throw std::invalid_argument("Newmark requires nonzero second mass");
  }
  void start(const AssembledSystem &old, IntegratorState &s,
             const IntegratorServices &services) override {
    // Differentiate the prescribed value AFTER boundary reduction. General
    // input may become Dirichlet through cancellation only on a selected edge.
    auto velocity_data =
        services.start_dirichlet(DirichletDerivative::FirstDerivative);
    auto acceleration_data =
        services.start_dirichlet(DirichletDerivative::SecondDerivative);
    impose_compatible(s.v, velocity_data, services.boundary_tolerance,
                      "velocity");
    acceleration_ = services.solve(
        old.second_mass, old.load - old.stiffness * s.u - old.first_mass * s.v,
        acceleration_data, false);
  }
  IntegratorState advance(const AssembledSystem &, const AssembledSystem &cur,
                          double h, const IntegratorState &s,
                          const IntegratorServices &services) override {
    const Vector predicted_u = s.u + h * s.v + (h * h / 4) * acceleration_;
    const Vector predicted_v = s.v + (h / 2) * acceleration_;
    SparseMatrix a = cur.second_mass + (h / 2) * cur.first_mass +
                     (h * h / 4) * cur.stiffness;
    Vector rhs =
        cur.load - cur.stiffness * predicted_u - cur.first_mass * predicted_v;
    auto data = cur.dirichlet;
    for (auto &d : data)
      d.value = (d.value - predicted_u[d.node]) / (h * h / 4);
    Vector next_acceleration = services.solve(a, rhs, data, true);
    IntegratorState next{predicted_u + (h * h / 4) * next_acceleration,
                         predicted_v + (h / 2) * next_acceleration};
    acceleration_ = std::move(next_acceleration);
    return next;
  }
};
} // namespace pde2d::fem
