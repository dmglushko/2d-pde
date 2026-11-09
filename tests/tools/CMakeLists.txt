# Documentation check (CMake only, always runs): every public header is in
# its module README. The negative case runs the same script on a fixture
# tree whose README omits a header and must fail.
pde2d_test(docs_header_map "${CMAKE_COMMAND}"
  "-DPDE2D_ROOT=${PROJECT_SOURCE_DIR}"
  -P "${CMAKE_CURRENT_SOURCE_DIR}/readme_headers.cmake")
set(header_map_fixture "${CMAKE_BINARY_DIR}/fixtures/docs_header_map_detects/tree")
foreach(module IN ITEMS discretization fem)
  file(MAKE_DIRECTORY "${header_map_fixture}/cpp/${module}/detail")
  file(WRITE "${header_map_fixture}/cpp/${module}/listed.hpp" "#pragma once\n")
  file(WRITE "${header_map_fixture}/cpp/${module}/detail/private.hpp" "#pragma once\n")
endforeach()
file(WRITE "${header_map_fixture}/cpp/fem/unlisted.hpp" "#pragma once\n")
file(WRITE "${header_map_fixture}/cpp/discretization/README.md" "| `listed.hpp` | |\n")
file(WRITE "${header_map_fixture}/cpp/fem/README.md" "| `listed.hpp` | |\n")
pde2d_test(docs_header_map_detects "${CMAKE_COMMAND}"
  "-DPDE2D_ROOT=${header_map_fixture}"
  -P "${CMAKE_CURRENT_SOURCE_DIR}/readme_headers.cmake")
set_tests_properties(docs_header_map_detects PROPERTIES
  PASS_REGULAR_EXPRESSION "header map:.*cpp/fem/unlisted.hpp")

# JavaScript tests for the viewer/diagnostic tooling. Tests that live next to
# their tools (tools/*/test*.cjs, tools/test_showcase.cjs) are registered by the
# top-level CMakeLists because they depend on example exports.
if(NOT PDE2D_NODE)
  return()
endif()
pde2d_test(viewer_diagnostics "${CMAKE_COMMAND}" -E chdir "${PROJECT_SOURCE_DIR}"
  "${PDE2D_NODE}" "${CMAKE_CURRENT_SOURCE_DIR}/viewer_diagnostics.cjs")
# Version 5 files (meshes with holes) written by tests/discretization/holes.cpp.
pde2d_test(mesh_holes_viewer "${PDE2D_NODE}"
  "${CMAKE_CURRENT_SOURCE_DIR}/mesh_holes_viewer.cjs"
  "${CMAKE_BINARY_DIR}/fixtures/mesh_holes")
set_tests_properties(mesh_holes_viewer PROPERTIES FIXTURES_REQUIRED mesh_holes_export)
pde2d_test(long_waves_diagnostics_regressions "${PDE2D_NODE}"
  "${CMAKE_CURRENT_SOURCE_DIR}/long_waves_diagnostics_regressions.cjs")
if(PDE2D_BUILD_FEM AND EXISTS "${PROJECT_SOURCE_DIR}/examples/fem/long_waves.cpp")
  pde2d_test(fem_wave_benchmark_cli "${PDE2D_NODE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/long_waves_benchmark_cli.cjs"
    $<TARGET_FILE:fem_long_waves>
    "${CMAKE_BINARY_DIR}/fixtures/fem_wave_benchmark_cli"
    "${PDE2D_ENABLE_OPENMP}")
endif()
