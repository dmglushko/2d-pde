find_package(Threads REQUIRED)

pde2d_add_test(NAME fem_expression SOURCE expression.cpp LIBS pde2d_fem)
pde2d_add_test(NAME fem_expression_threads SOURCE expression_threads.cpp
               LIBS pde2d_fem Threads::Threads)
pde2d_add_test(NAME fem_problem SOURCE problem.cpp LIBS pde2d_fem)
pde2d_add_test(NAME fem_meshed_problem SOURCE meshed_problem.cpp
               LIBS pde2d_fem)
# The private derivative function (src/fem/derivative.hpp), which has no
# public caller in x and y yet.
pde2d_add_test(NAME fem_derivative SOURCE derivative.cpp LIBS pde2d_fem)
target_include_directories(fem_derivative PRIVATE
                           ${PROJECT_SOURCE_DIR}/src/fem)
