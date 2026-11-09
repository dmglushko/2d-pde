# Documentation check: every public header of cpp/discretization and cpp/fem
# (top level; detail/ is private) must be named, in backticks, in its
# module README (`cpp/<module>/README.md`, the header map). Fails with the
# list of missing headers.
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
  file(GLOB headers RELATIVE "${PDE2D_ROOT}/cpp/${module}"
       "${PDE2D_ROOT}/cpp/${module}/*.hpp")
  list(SORT headers)
  foreach(header IN LISTS headers)
    math(EXPR checked "${checked} + 1")
    string(FIND "${readme}" "`${header}`" position)
    if(position EQUAL -1)
      list(APPEND missing "cpp/${module}/${header}")
    endif()
  endforeach()
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
