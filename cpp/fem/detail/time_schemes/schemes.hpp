#pragma once
// The list of built-in time-scheme units (interface.hpp) and the only
// dispatch on TimeMethod. A new built-in scheme is a TimeMethod value before
// Custom, its unit header and one entry in Units; the static_asserts below
// fail until all three agree.
#include "fem/detail/time_schemes/backward_euler.hpp"
#include "fem/detail/time_schemes/crank_nicolson.hpp"
#include "fem/detail/time_schemes/interface.hpp"
#include "fem/detail/time_schemes/newmark.hpp"
#include <cstddef>
#include <optional>
#include <tuple>
#include <utility>

namespace pde2d::fem::time_scheme_detail {
// In TimeMethod order.
using Units =
    std::tuple<BackwardEulerScheme, CrankNicolsonScheme, NewmarkScheme>;

template <std::size_t... I>
consteval bool units_complete(std::index_sequence<I...>) {
  return (TimeSchemeUnit<std::tuple_element_t<I, Units>> && ...) &&
         ((static_cast<std::size_t>(std::tuple_element_t<I, Units>::method) ==
           I) &&
          ...) &&
         static_cast<std::size_t>(TimeMethod::Custom) == sizeof...(I);
}
static_assert(
    units_complete(std::make_index_sequence<std::tuple_size_v<Units>>{}),
    "Every TimeMethod before Custom needs one complete unit in "
    "Units, in TimeMethod order");

// Calls f(Unit{}) for the unit of `method` and returns its result; `none()`
// for TimeMethod::Custom or a value that names no unit.
template <class F, class None>
decltype(auto) with_time_method(TimeMethod method, F &&f, None &&none) {
  return [&]<std::size_t... I>(std::index_sequence<I...>) -> decltype(auto) {
    using Result = decltype(f(std::tuple_element_t<0, Units>{}));
    std::optional<Result> result;
    ((static_cast<std::size_t>(method) == I
          ? (void)result.emplace(f(std::tuple_element_t<I, Units>{}))
          : (void)0),
     ...);
    return result ? std::move(*result) : Result(none());
  }(std::make_index_sequence<std::tuple_size_v<Units>>{});
}
} // namespace pde2d::fem::time_scheme_detail
