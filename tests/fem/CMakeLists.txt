if(NOT PDE2D_BUILD_FEM)
  message(STATUS "UNAVAILABLE: FEM tests (PDE2D_BUILD_FEM=OFF)")
  return()
endif()
foreach(name IN ITEMS
    annulus assembly boundary_binding boundary_classification convergence
    custom_integrator dirichlet_time_derivatives error_diagnostics expression
    moving_convergence nodal_state oblique_conditions p1_kernels
    parallel_assembly prepared_assembly prepared_stationary quadrature
    reference_problems reporting reusable_evolution run_records solver
    solver_backends symbol_contexts time_grid transient)
  pde2d_add_cpp_test(NAME fem_${name} SOURCE ${name}.cpp FEM)
endforeach()
