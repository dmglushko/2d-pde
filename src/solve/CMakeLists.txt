add_library(pde2d_solve
  bicgstab.cpp
  conjugate_gradient.cpp
  factorization_threads.cpp
  fixed_step.cpp
  integrator.cpp
  multifrontal_lu.cpp
  simplicial_ldlt.cpp
  solution.cpp
  sparse_lu.cpp
  static_solver.cpp
  supernodal_cholesky.cpp
  time_methods.cpp)
add_library(pde2d::solve ALIAS pde2d_solve)
target_include_directories(pde2d_solve PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_compile_features(pde2d_solve PUBLIC cxx_std_23)
# solve depends on fem (and through it on mesh and geometry), never the
# other way. Eigen is private: only .cpp files in src/solve/ include it
# (through eigen_view.hpp), never a public header. With OpenMP Eigen runs
# its row-major sparse matrix-vector product in parallel; every Eigen
# computation here runs inside detail::EigenThreads (eigen_threads.hpp),
# which sets how many threads it may use. OpenMP is a PUBLIC requirement
# (no header of ours includes <omp.h>): a program that links pde2d_solve
# and includes Eigen itself must compile Eigen's inline functions with the
# same OpenMP setting, or the linker may keep its serial copy of
# Eigen::nbThreads() and the solvers silently run on one thread (a review
# finding, seen in Debug builds).
#
# SuiteSparse (CHOLMOD for SupernodalCholesky, ParU for MultifrontalLu) is
# private too: only supernodal_cholesky.cpp and multifrontal_lu.cpp (and
# the private suitesparse.hpp) include its headers, which declare C
# functions and non-inline C++ functions, so a program that includes them
# itself cannot end up with a differently compiled copy of one of ours.
# OpenBLAS is linked directly for its thread control
# (factorization_threads.cpp); SuiteSparse reaches it through the system
# BLAS.
target_link_libraries(
  pde2d_solve
  PUBLIC pde2d_fem OpenMP::OpenMP_CXX
  PRIVATE Eigen3::Eigen SuiteSparse::CHOLMOD SuiteSparse::ParU
          ${PDE2D_OPENBLAS_LIBRARY} pde2d_warnings)
