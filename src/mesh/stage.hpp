#pragma once

// Private to the mesh library.

#include "pde2d/mesh/report.hpp"

#include <chrono>
#include <string>
#include <utility>

namespace pde2d::mesh::detail {

// Runs f(report) and records the stage name and wall time.
template <class F>
auto timed_stage(StageReport &report, std::string name, F &&f) {
  report.stage = std::move(name);
  auto start = std::chrono::steady_clock::now();
  if constexpr (std::is_void_v<decltype(f(report))>) {
    f(report);
    report.time = std::chrono::steady_clock::now() - start;
  } else {
    auto result = f(report);
    report.time = std::chrono::steady_clock::now() - start;
    return result;
  }
}

} // namespace pde2d::mesh::detail
