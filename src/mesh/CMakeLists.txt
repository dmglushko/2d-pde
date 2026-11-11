add_library(pde2d_mesh
  boundary_polygon.cpp
  cdt.cpp
  chord_length_discretizer.cpp
  components.cpp
  delaunay.cpp
  delaunay_flips.cpp
  ear_clipping.cpp
  laplacian_smoothing.cpp
  longest_edge_bisection.cpp
  mesh.cpp
  mesher.cpp
  remesher.cpp
  ruppert.cpp
  working_mesh.cpp)
add_library(pde2d::mesh ALIAS pde2d_mesh)
target_include_directories(pde2d_mesh PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_compile_features(pde2d_mesh PUBLIC cxx_std_23)
# The mesh module depends on geometry, never the other way round.
target_link_libraries(pde2d_mesh PUBLIC pde2d_geometry PRIVATE pde2d_warnings)
