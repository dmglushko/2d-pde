pde2d_add_test(NAME io_svg SOURCE svg.cpp LIBS pde2d_io)
pde2d_add_test(NAME io_pde2d SOURCE pde2d_mesh.cpp LIBS pde2d_io)
pde2d_add_test(NAME io_gmsh SOURCE gmsh.cpp LIBS pde2d_io)
# The old .pde2d files in examples/mesh_viewer.
target_compile_definitions(io_pde2d PRIVATE PDE2D_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
# The hand-written .msh files.
target_compile_definitions(io_gmsh
  PRIVATE PDE2D_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")
