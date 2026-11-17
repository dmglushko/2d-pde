add_library(pde2d_adapt
  adaptive_static_solver.cpp
  error_estimator.cpp
  estimator_geometry.cpp
  marking.cpp
  mesh_refinement.cpp
  recovery_estimator.cpp
  residual_estimators.cpp)
add_library(pde2d::adapt ALIAS pde2d_adapt)
target_include_directories(pde2d_adapt PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_compile_features(pde2d_adapt PUBLIC cxx_std_23)
# Everything adaptive: estimators, marking, adaptations and the adaptive
# loop. adapt depends on solve and fem (and through them on mesh and
# geometry); nothing in the library depends on it. The refinement of marked
# triangles is in the mesh module (Remesher::refine), shared with meshing.
target_link_libraries(pde2d_adapt PUBLIC pde2d_solve pde2d_fem pde2d_mesh
                                  PRIVATE pde2d_warnings)
