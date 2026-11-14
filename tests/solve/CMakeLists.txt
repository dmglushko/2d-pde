pde2d_add_test(NAME solve_linear_solvers SOURCE linear_solvers.cpp
               LIBS pde2d_solve)
pde2d_add_test(NAME solve_static_solver SOURCE static_solver.cpp
               LIBS pde2d_solve)
pde2d_add_test(NAME solve_time_integration SOURCE time_integration.cpp
               LIBS pde2d_solve)
