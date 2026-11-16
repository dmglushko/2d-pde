#pragma once

#include <string_view>

// Which source this app was built from (exports' provenance): the git
// commit and whether the tree had uncommitted changes when it was built,
// each "unknown" outside git. Written at build time
// (app/cmake/build_info.cmake).
namespace pde2d::app {

struct BuildInfo {
  std::string_view commit; // 40 hex digits, or "unknown"
  std::string_view dirty;  // "true", "false" or "unknown"
};

BuildInfo build_info();

} // namespace pde2d::app
