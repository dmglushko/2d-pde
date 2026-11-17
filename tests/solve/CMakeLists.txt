pde2d_add_test(NAME solve_linear_solvers SOURCE linear_solvers.cpp
               LIBS pde2d_solve)
pde2d_add_test(NAME solve_static_solver SOURCE static_solver.cpp
               LIBS pde2d_solve)
pde2d_add_test(NAME solve_time_integration SOURCE time_integration.cpp
               LIBS pde2d_solve)
pde2d_add_test(NAME solve_review_regressions SOURCE review_regressions.cpp
               LIBS pde2d_solve)
pde2d_add_test(NAME solve_review_slice2 SOURCE review_slice2.cpp
               LIBS pde2d_solve)
pde2d_add_test(NAME solve_review_start_needs SOURCE review_start_needs.cpp
               LIBS pde2d_solve)
# Uses at most 4 threads.
pde2d_add_test(NAME solve_threads SOURCE threads.cpp LIBS pde2d_solve)
# The parallel direct solvers; at most 4 threads.
pde2d_add_test(NAME solve_parallel_direct SOURCE parallel_direct.cpp
               LIBS pde2d_solve)
# A consumer that includes Eigen itself, at -O0 (review finding).
pde2d_add_test(NAME solve_eigen_consumer SOURCE eigen_consumer.cpp
               LIBS pde2d_solve Eigen3::Eigen)
set_source_files_properties(eigen_consumer.cpp PROPERTIES COMPILE_OPTIONS -O0)
