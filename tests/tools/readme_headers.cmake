# Documentation check: every public header of cpp/discretization and cpp/fem
# (any directory but detail/, which is private; a header in a public
# subdirectory is named with its relative path, e.g. `sub/nested.hpp`) must
# be named, in backticks, in its module README (`cpp/<module>/README.md`, the
# header map); the top-level core headers (cpp/*.hpp) must be named in
# cpp/README.md. Fails with the list of missing headers.
#
#   cmake -DPDE2D_ROOT=<repository> -P tests/tools/readme_headers.cmake
if(NOT PDE2D_ROOT)
  message(FATAL_ERROR "readme_headers.cmake needs -DPDE2D_ROOT=<repository>")
endif()
set(missing "")
set(checked 0)
foreach(module IN ITEMS discretization fem)
  set(readme_path "${PDE2D_ROOT}/cpp/${module}/README.md")
  if(NOT EXISTS "${readme_path}")
    message(FATAL_ERROR "Missing module README: cpp/${module}/README.md")
  endif()
  file(READ "${readme_path}" readme)
  file(GLOB_RECURSE headers RELATIVE "${PDE2D_ROOT}/cpp/${module}"
       "${PDE2D_ROOT}/cpp/${module}/*.hpp")
  list(FILTER headers EXCLUDE REGEX "(^|/)detail/")
  list(SORT headers)
  foreach(header IN LISTS headers)
    math(EXPR checked "${checked} + 1")
    string(FIND "${readme}" "`${header}`" position)
    if(position EQUAL -1)
      list(APPEND missing "cpp/${module}/${header}")
    endif()
  endforeach()
endforeach()
# Core headers at the top of cpp/ (not recursive): named in cpp/README.md.
set(core_readme "${PDE2D_ROOT}/cpp/README.md")
if(NOT EXISTS "${core_readme}")
  message(FATAL_ERROR "Missing core README: cpp/README.md")
endif()
file(READ "${core_readme}" readme)
file(GLOB headers RELATIVE "${PDE2D_ROOT}/cpp" "${PDE2D_ROOT}/cpp/*.hpp")
list(SORT headers)
foreach(header IN LISTS headers)
  math(EXPR checked "${checked} + 1")
  string(FIND "${readme}" "`${header}`" position)
  if(position EQUAL -1)
    list(APPEND missing "cpp/${header}")
  endif()
endforeach()
if(checked EQUAL 0)
  message(FATAL_ERROR "No public headers found under ${PDE2D_ROOT}/cpp")
endif()
if(missing)
  list(JOIN missing "\n  " text)
  message(FATAL_ERROR
    "Public headers missing from their module README header map:\n  ${text}")
endif()
message(STATUS "All ${checked} public headers are in their module README")
