pde2d_add_test(NAME io_svg SOURCE svg.cpp LIBS pde2d_io)
pde2d_add_test(NAME io_pde2d SOURCE pde2d_mesh.cpp LIBS pde2d_io)
pde2d_add_test(NAME io_gmsh SOURCE gmsh.cpp LIBS pde2d_io)
# The old .pde2d files in examples/mesh_viewer.
target_compile_definitions(io_pde2d PRIVATE PDE2D_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
# The hand-written .msh files.
target_compile_definitions(io_gmsh
  PRIVATE PDE2D_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")

# The mesh viewer's own tests (unchanged from the previous code) and a check
# that the viewer reads what the example program writes. They need Node.js.
find_program(PDE2D_NODE node)
if(NOT PDE2D_NODE)
  message(STATUS "Node.js not found: the mesh viewer tests are skipped")
  return()
endif()
add_test(NAME viewer_test_mesh
  COMMAND ${PDE2D_NODE} ${PROJECT_SOURCE_DIR}/tools/mesh_viewer/test_mesh.cjs)
if(TARGET produce_mesh)
  set(viewer_dir ${CMAKE_BINARY_DIR}/fixtures/viewer)
  file(MAKE_DIRECTORY ${viewer_dir})
  add_test(NAME example_produce_mesh
    COMMAND produce_mesh ${viewer_dir}/plate.pde2d ${viewer_dir}/plate.svg)
  set_tests_properties(example_produce_mesh PROPERTIES
    FIXTURES_SETUP viewer_meshes)
  add_test(NAME viewer_reads_export
    COMMAND ${PDE2D_NODE} ${CMAKE_CURRENT_SOURCE_DIR}/viewer_reads_export.cjs
            ${viewer_dir}/plate.pde2d)
  set_tests_properties(viewer_reads_export PROPERTIES
    FIXTURES_REQUIRED viewer_meshes)
endif()
