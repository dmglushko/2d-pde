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
