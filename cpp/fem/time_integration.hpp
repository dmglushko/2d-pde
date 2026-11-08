#pragma once
// One-step time integrators for the semi-discrete system
//
//   M2 u'' + M1 u' + K u = F(t),   u = g(t) on Dirichlet nodes,
//
// where AssembledSystem supplies K = stiffness, M1 = first_mass,
// M2 = second_mass and F = load at a time stamp. Integrators only form linear
// systems and delegate solving (with Dirichlet lifting and factorization
// reuse) to the driver through IntegratorServices, so the same integrator runs
// unchanged under evolve() and under a test's uncached reference driver.
//
// A user scheme needs no library change: implement TimeIntegrator and pass a
// factory in TimeOptions::integrator (evolution.hpp). A built-in scheme also
// gets a TimeMethod value, a case in make_time_integrator and a name in
// time_method_name.
#include "fem/system.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace pde2d::fem {
// Custom marks integrators supplied through TimeOptions::integrator.
enum class TimeMethod { BackwardEuler, CrankNicolson, Newmark, Custom };

inline const char *time_method_name(TimeMethod method) {
  switch (method) {
  case TimeMethod::BackwardEuler:
    return "BackwardEuler";
  case TimeMethod::CrankNicolson:
    return "CrankNicolson";
  case TimeMethod::Newmark:
    return "Newmark";
  case TimeMethod::Custom:
    return "Custom";
  }
  return "Unknown";
}

// Displacement and velocity at one time stamp.
struct IntegratorState {
  Vector u, v;
};

struct IntegratorServices {
  // Solve matrix*x = rhs subject to x[node] = value for every constraint.
  // time_step=false marks auxiliary solves (e.g. Newmark start-up), which are
  // excluded from per-step solver reports.
  std::function<Vector(const SparseMatrix &, const Vector &,
                       const std::vector<DirichletValue> &, bool time_step)>
      solve;
  // Dirichlet data of time-derivative order 1 or 2 at the start time.
  std::function<std::vector<DirichletValue>(int order)> start_dirichlet;
  double boundary_tolerance = 1e-10;
};

// Check that constrained entries of v agree with the prescribed data (relative
// to max(1,|v|,|g|)) and then overwrite them exactly.
inline void impose_compatible(Vector &v,
                              const std::vector<DirichletValue> &data,
                              double tolerance, const char *description) {
  for (const auto &d : data) {
    if (std::abs(v[d.node] - d.value) >
        tolerance * std::max({1., std::abs(v[d.node]), std::abs(d.value)}))
      throw std::invalid_argument(
          std::string("Incompatible initial prescribed ") + description);
    v[d.node] = d.value;
  }
}

// Integrators may keep internal state (Newmark keeps the acceleration), so
// use one instance per evolution.
class TimeIntegrator {
public:
  virtual ~TimeIntegrator() = default;
  // Built-in schemes return their TimeMethod; user schemes may keep Custom.
  virtual TimeMethod method() const { return TimeMethod::Custom; }
  // Name used in reports and EvolutionMetadata::method_name.
  virtual std::string name() const { return time_method_name(method()); }
  // 1: needs u at the start; 2: also needs u_t.
  virtual int order() const = 0;
  // Reject systems this scheme cannot integrate.
  virtual void check(const AssembledSystem &system) const = 0;
  // Called once with the start-time system, after the initial displacement
  // was made compatible with its Dirichlet data.
  virtual void start(const AssembledSystem &, IntegratorState &,
                     const IntegratorServices &) {}
  // Advance from `previous` (stamp t) to `current` (stamp t+h). The driver
  // re-imposes Dirichlet displacement on the returned u afterwards.
  virtual IntegratorState advance(const AssembledSystem &previous,
                                  const AssembledSystem &current, double h,
                                  const IntegratorState &state,
                                  const IntegratorServices &services) = 0;
};

namespace time_integration_detail {
class FirstOrder : public TimeIntegrator {
public:
  int order() const override { return 1; }
  void check(const AssembledSystem &s) const override {
    if (s.second_mass.norm() != 0)
      throw std::invalid_argument(
          "First-order integrator does not support second mass");
    if (s.first_mass.norm() == 0)
      throw std::invalid_argument("First-order integrator requires nonzero "
                                  "first mass (stationary problem)");
  }
};
} // namespace time_integration_detail

// (M1 + h K) u1 = M1 u0 + h F1.
class BackwardEulerIntegrator final
    : public time_integration_detail::FirstOrder {
public:
  TimeMethod method() const override { return TimeMethod::BackwardEuler; }
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

// Endpoint trapezoidal residual: M1 at each endpoint multiplies the common
// difference quotient, not a midpoint evaluation:
// (M1avg + h/2 K1) u1 = M1avg u0 - h/2 K0 u0 + h/2 (F0 + F1).
class CrankNicolsonIntegrator final
    : public time_integration_detail::FirstOrder {
public:
  TimeMethod method() const override { return TimeMethod::CrankNicolson; }
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

// Average-acceleration Newmark (beta = 1/4, gamma = 1/2), solved for the new
// acceleration. Dirichlet displacement data become acceleration constraints.
class NewmarkIntegrator final : public TimeIntegrator {
  Vector acceleration_;

public:
  TimeMethod method() const override { return TimeMethod::Newmark; }
  int order() const override { return 2; }
  void check(const AssembledSystem &s) const override {
    if (s.second_mass.norm() == 0)
      throw std::invalid_argument("Newmark requires nonzero second mass");
  }
  void start(const AssembledSystem &old, IntegratorState &s,
             const IntegratorServices &services) override {
    // Differentiate the prescribed value AFTER boundary reduction. General
    // input may become Dirichlet through cancellation only on a selected edge.
    auto velocity_data = services.start_dirichlet(1);
    auto acceleration_data = services.start_dirichlet(2);
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

inline std::unique_ptr<TimeIntegrator> make_time_integrator(TimeMethod method) {
  switch (method) {
  case TimeMethod::BackwardEuler:
    return std::make_unique<BackwardEulerIntegrator>();
  case TimeMethod::CrankNicolson:
    return std::make_unique<CrankNicolsonIntegrator>();
  case TimeMethod::Newmark:
    return std::make_unique<NewmarkIntegrator>();
  case TimeMethod::Custom:
    throw std::invalid_argument(
        "TimeMethod::Custom requires TimeOptions::integrator");
  }
  throw std::invalid_argument("Unsupported time method");
}
} // namespace pde2d::fem
