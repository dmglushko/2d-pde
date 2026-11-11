add_library(pde2d_geometry
  curve.cpp
  curve_geometry.cpp
  polygon.cpp
  predicates.cpp
  shapes.cpp)
add_library(pde2d::geometry ALIAS pde2d_geometry)
target_include_directories(pde2d_geometry PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_compile_features(pde2d_geometry PUBLIC cxx_std_23)
target_link_libraries(pde2d_geometry PRIVATE Boost::headers pde2d_warnings)
