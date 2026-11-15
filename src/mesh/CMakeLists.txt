add_library(pde2d_mesh
  bisection_refiners.cpp
  boundary_polygon.cpp
  boundary_split_policy.cpp
  boundary_split_steps.cpp
  cdt.cpp
  chord_length_discretizer.cpp
  components.cpp
  curve_chain.cpp
  delaunay.cpp
  delaunay_flips.cpp
  domain_size.cpp
  ear_clipping.cpp
  edge_bisection.cpp
  edge_recovery.cpp
  laplacian_smoothing.cpp
  loop_chains.cpp
  mesh.cpp
  mesh_properties.cpp
  mesh_validity.cpp
  mesher.cpp
  remesher.cpp
  ruppert.cpp
  working_mesh.cpp)
add_library(pde2d::mesh ALIAS pde2d_mesh)
target_include_directories(pde2d_mesh PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_compile_features(pde2d_mesh PUBLIC cxx_std_23)
# The mesh module depends on geometry, never the other way round.
target_link_libraries(pde2d_mesh PUBLIC pde2d_geometry PRIVATE pde2d_warnings)
