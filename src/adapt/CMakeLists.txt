add_library(pde2d_adapt
  marking.cpp)
add_library(pde2d::adapt ALIAS pde2d_adapt)
target_include_directories(pde2d_adapt PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_compile_features(pde2d_adapt PUBLIC cxx_std_23)
# Marking works on meshes only: the module depends on mesh (and through it
# geometry), never on fem or solve, and nothing below depends on it. The
# refinement of marked triangles is in the mesh module (Remesher::refine).
target_link_libraries(pde2d_adapt PUBLIC pde2d_mesh PRIVATE pde2d_warnings)
