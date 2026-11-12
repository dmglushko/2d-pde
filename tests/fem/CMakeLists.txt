find_package(Threads REQUIRED)

pde2d_add_test(NAME fem_expression SOURCE expression.cpp LIBS pde2d_fem)
pde2d_add_test(NAME fem_expression_threads SOURCE expression_threads.cpp
               LIBS pde2d_fem Threads::Threads)
pde2d_add_test(NAME fem_problem SOURCE problem.cpp LIBS pde2d_fem)
pde2d_add_test(NAME fem_meshed_problem SOURCE meshed_problem.cpp
               LIBS pde2d_fem)
