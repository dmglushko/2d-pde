# Every public header compiles on its own: one generated translation unit per
# header (included twice, so a missing include guard fails too), all linked
# into one executable, so a non-inline definition in a header fails at link
# time. Headers are discovered by globbing on purpose: a new public header is
# covered without registration, also in a public subdirectory such as
# cpp/fem/reference/. Private detail/ headers are not checked.
# FEM headers need PDE2D_BUILD_FEM, svg_geometry.hpp needs LibXml2.
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
set(main "${CMAKE_CURRENT_BINARY_DIR}/standalone/main.cpp")
file(CONFIGURE OUTPUT "${main}" CONTENT
  "#include <cstdio>\nint main() {\n  std::printf(\"PASS standalone_headers (${pde2d_header_count} headers)\\n\");\n  return 0;\n}\n")
add_executable(header_standalone ${pde2d_header_sources} "${main}")
target_link_libraries(header_standalone PRIVATE pde2d_execution Threads::Threads)
if(PDE2D_BUILD_FEM)
  target_link_libraries(header_standalone PRIVATE pde2d_fem)
endif()
if(LibXml2_FOUND)
  target_link_libraries(header_standalone PRIVATE LibXml2::LibXml2)
endif()
pde2d_test(header_standalone $<TARGET_FILE:header_standalone>)
