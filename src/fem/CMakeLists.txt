add_library(pde2d_fem
  compiled_expression.cpp
  derivative.cpp
  expression.cpp
  meshed_problem.cpp
  problem.cpp)
add_library(pde2d::fem ALIAS pde2d_fem)
target_include_directories(pde2d_fem PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_compile_features(pde2d_fem PUBLIC cxx_std_23)
# fem depends on mesh (and through it on geometry), never the other way.
# SymEngine is private: only .cpp files in src/fem/ include it (through
# symengine_expression.hpp), never a public header.
target_link_libraries(pde2d_fem PUBLIC pde2d_mesh PRIVATE symengine
                                                          pde2d_warnings)
