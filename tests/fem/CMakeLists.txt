if(NOT PDE2D_BUILD_FEM)
  message(STATUS "UNAVAILABLE: FEM tests (PDE2D_BUILD_FEM=OFF)")
  return()
endif()
foreach(name IN ITEMS
    assembly boundary_classification convergence dirichlet_time_derivatives error_diagnostics expression
    moving_convergence parallel_assembly prepared_assembly quadrature reporting
    reusable_evolution solver time_grid transient)
  pde2d_add_cpp_test(NAME fem_${name} SOURCE ${name}.cpp FEM)
endforeach()
