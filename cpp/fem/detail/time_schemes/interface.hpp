#pragma once
// Private (time_scheme_detail) interface of the built-in time schemes, one
// unit per TimeMethod, mirroring the boundary edge-form units
// (detail/edge_forms/) and the interior-algorithm units of meshing. A unit
// gathers everything the library knows about its scheme:
//
//   method        its TimeMethod value (the selector in TimePlan)
//   name          report name, decision choice and descriptor name (stable:
//                 run records and tests use it)
//   order         1 if the scheme needs only u at the start, 2 if it also
//                 needs u_t (InitialConditions::velocity)
//   Integrator    the public TimeIntegrator class of the scheme
//   make()        a fresh integrator (one per evolution)
//   descriptor()  the StrategyDescriptor run records show for it
//
// schemes.hpp lists the units; with_time_method() there is the only dispatch
// on TimeMethod and TimeSchemeUnit checks at compile time that every unit is
// complete. The integrator classes take method(), name(), order() and
// descriptor() from their unit through BuiltinScheme, so the unit is the
// one place that states them.
#include "fem/time_integrator.hpp"
#include "provenance.hpp"
#include <concepts>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

namespace pde2d::fem::time_scheme_detail {
template <class Unit>
concept TimeSchemeUnit =
    requires {
      { Unit::method } -> std::convertible_to<TimeMethod>;
      { Unit::name } -> std::convertible_to<const char *>;
      { Unit::order } -> std::convertible_to<int>;
      { Unit::make() } -> std::same_as<std::unique_ptr<TimeIntegrator>>;
      { Unit::descriptor() } -> std::same_as<pde2d::StrategyDescriptor>;
    } && std::derived_from<typename Unit::Integrator, TimeIntegrator> &&
    (Unit::order == 1 || Unit::order == 2);

// The parts of a unit every scheme shares: its descriptor is its name
// (built-in schemes have no parameters) and make() creates its Integrator.
template <class Self> struct UnitBase {
  static pde2d::StrategyDescriptor descriptor() { return {Self::name}; }
  static std::unique_ptr<TimeIntegrator> make() {
    return std::make_unique<typename Self::Integrator>();
  }
};

// The TimeIntegrator overrides that come from the unit.
template <class Unit, class Base = TimeIntegrator>
class BuiltinScheme : public Base {
public:
  TimeMethod method() const override { return Unit::method; }
  std::string name() const override { return Unit::name; }
  std::optional<pde2d::StrategyDescriptor> descriptor() const override {
    return Unit::descriptor();
  }
  int order() const override { return Unit::order; }
};

// One-step first-order schemes: they keep no state between steps and
// integrate systems with a nonzero first mass and no second mass.
class FirstOrder : public TimeIntegrator {
public:
  bool transactional() const override { return true; }
  void check(const AssembledSystem &s) const override {
    const int system_order = time_order(s);
    if (system_order == 2)
      throw std::invalid_argument(
          "First-order integrator does not support second mass");
    if (system_order == 0)
      throw std::invalid_argument("First-order integrator requires nonzero "
                                  "first mass (stationary problem)");
  }
};
} // namespace pde2d::fem::time_scheme_detail
