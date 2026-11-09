#pragma once
// Time integration: the TimeIntegrator interface (time_integrator.hpp) and
// the built-in schemes, one unit per TimeMethod (detail/time_schemes/):
//
//   BackwardEuler   first order, BackwardEulerIntegrator
//   CrankNicolson   first order, CrankNicolsonIntegrator
//   Newmark         second order (needs u_t), NewmarkIntegrator
//
// Each unit states the scheme's name, order and factory once;
// time_method_name, time_method_order and make_time_integrator below read
// them through the single dispatch
// (time_scheme_detail::with_time_method). A user scheme needs no library
// change: implement TimeIntegrator and pass a factory in
// TimeOptions::integrator (evolution.hpp). EXTENDING.md has the recipe for a
// built-in one.
#include "fem/detail/time_schemes/schemes.hpp"
#include "fem/time_integrator.hpp"
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

namespace pde2d::fem {
// Report name of a built-in scheme; "Custom" for TimeMethod::Custom and
// "Unknown" for a value that names nothing.
inline const char *time_method_name(TimeMethod method) {
  return time_scheme_detail::with_time_method(
      method, []<class Unit>(Unit) { return Unit::name; },
      [method] { return method == TimeMethod::Custom ? "Custom" : "Unknown"; });
}
// 1 or 2 for a built-in scheme (2: it needs an initial velocity); nullopt
// for Custom, whose order only the integrator knows.
inline std::optional<int> time_method_order(TimeMethod method) {
  return time_scheme_detail::with_time_method(
      method, []<class Unit>(Unit) { return std::optional<int>(Unit::order); },
      [] { return std::optional<int>(); });
}

inline std::unique_ptr<TimeIntegrator> make_time_integrator(TimeMethod method) {
  return time_scheme_detail::with_time_method(
      method, []<class Unit>(Unit) { return Unit::make(); },
      [method]() -> std::unique_ptr<TimeIntegrator> {
        if (method == TimeMethod::Custom)
          throw std::invalid_argument(
              "TimeMethod::Custom requires TimeOptions::integrator");
        throw std::invalid_argument("Unsupported time method");
      });
}
} // namespace pde2d::fem
