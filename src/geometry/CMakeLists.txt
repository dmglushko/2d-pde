add_library(pde2d_geometry
  curve.cpp
  curve_geometry.cpp
  polygon.cpp
  predicates.cpp
  shapes.cpp)
add_library(pde2d::geometry ALIAS pde2d_geometry)
target_include_directories(pde2d_geometry PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_compile_features(pde2d_geometry PUBLIC cxx_std_23)
target_link_libraries(pde2d_geometry PRIVATE pde2d_warnings)
# The adaptive predicates rely on every floating-point operation being
# rounded on its own (error-free transformations). -ffast-math would let
# the compiler simplify them away, and contracting a * b + c into a fused
# multiply-add changes their rounding, so both are switched off for this
# file whatever the global flags say. predicates.cpp also refuses to
# compile if __FAST_MATH__ is defined or FLT_EVAL_METHOD is not 0.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  set_source_files_properties(predicates.cpp PROPERTIES
    COMPILE_OPTIONS "-fno-fast-math;-ffp-contract=off")
endif()
