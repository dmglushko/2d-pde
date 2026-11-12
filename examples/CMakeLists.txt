# Meshes a plate with a hole and writes it as .pde2d for the mesh viewer
# (tools/mesh_viewer). Usage: produce_mesh OUTPUT.pde2d [OUTPUT.svg]
add_executable(produce_mesh mesh_viewer/produce_mesh.cpp)
target_link_libraries(produce_mesh PRIVATE pde2d_io pde2d_warnings)
