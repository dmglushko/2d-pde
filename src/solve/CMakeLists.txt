add_library(pde2d_solve
  bicgstab.cpp
  fixed_step.cpp
  integrator.cpp
  solution.cpp
  sparse_lu.cpp
  static_solver.cpp
  time_methods.cpp)
add_library(pde2d::solve ALIAS pde2d_solve)
target_include_directories(pde2d_solve PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_compile_features(pde2d_solve PUBLIC cxx_std_23)
# solve depends on fem (and through it on mesh and geometry), never the
# other way. Eigen is private: only .cpp files in src/solve/ include it
# (through eigen_view.hpp), never a public header.
target_link_libraries(pde2d_solve PUBLIC pde2d_fem PRIVATE Eigen3::Eigen
                                                           pde2d_warnings)
