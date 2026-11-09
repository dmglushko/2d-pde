# Every public header compiles on its own: one generated translation unit per
# header (included twice, so a missing include guard fails too). The units are
# only parsed and type-checked (-fsyntax-only through syntax_only.sh), never
# compiled to code, so the check costs a fraction of a normal build and a
# failure fails the build itself. Headers are discovered by globbing on
# purpose: a new public header is covered without registration, also in a
# public subdirectory. Private detail/ headers are not checked. FEM headers
# need PDE2D_BUILD_FEM, svg_geometry.hpp needs LibXml2.
set(pde2d_header_globs "${PROJECT_SOURCE_DIR}/cpp/*.hpp"
  "${PROJECT_SOURCE_DIR}/cpp/discretization/*.hpp")
if(PDE2D_BUILD_FEM)
  list(APPEND pde2d_header_globs "${PROJECT_SOURCE_DIR}/cpp/fem/*.hpp"
    "${PROJECT_SOURCE_DIR}/cpp/fem/*/*.hpp")
endif()
set(pde2d_header_sources)
set(pde2d_header_count 0)
foreach(pattern IN LISTS pde2d_header_globs)
  file(GLOB headers CONFIGURE_DEPENDS "${pattern}")
  list(FILTER headers EXCLUDE REGEX "/detail/")
  list(SORT headers)
  foreach(header IN LISTS headers)
    file(RELATIVE_PATH relative "${PROJECT_SOURCE_DIR}/cpp" "${header}")
    if(relative STREQUAL "discretization/svg_geometry.hpp" AND NOT LibXml2_FOUND)
      message(STATUS "UNAVAILABLE: standalone check of ${relative} (LibXml2)")
      continue()
    endif()
    string(MAKE_C_IDENTIFIER "${relative}" id)
    set(source "${CMAKE_CURRENT_BINARY_DIR}/standalone/${id}.cpp")
    file(CONFIGURE OUTPUT "${source}"
      CONTENT "#include \"${relative}\"\n#include \"${relative}\"\n")
    list(APPEND pde2d_header_sources "${source}")
    math(EXPR pde2d_header_count "${pde2d_header_count} + 1")
  endforeach()
endforeach()
# An object library nothing links: building it is the check. The launcher
# needs a GCC/Clang-style driver.
add_library(header_standalone OBJECT ${pde2d_header_sources})
set_target_properties(header_standalone PROPERTIES CXX_COMPILER_LAUNCHER
  "sh;${CMAKE_CURRENT_SOURCE_DIR}/syntax_only.sh")
target_link_libraries(header_standalone PRIVATE pde2d_execution Threads::Threads)
if(PDE2D_BUILD_FEM)
  target_link_libraries(header_standalone PRIVATE pde2d_fem)
endif()
if(LibXml2_FOUND)
  target_link_libraries(header_standalone PRIVATE LibXml2::LibXml2)
endif()
# The build already did the work; the test reports what it covered.
pde2d_test(header_standalone "${CMAKE_COMMAND}" -E echo
  "PASS standalone_headers (${pde2d_header_count} headers, syntax only)")
