if(NOT PDE2D_BUILD_FEM)
  message(STATUS "UNAVAILABLE: FEM tests (PDE2D_BUILD_FEM=OFF)")
  return()
endif()
foreach(name IN ITEMS
    adaptive_loop analysis assembly boundary_classification convergence custom_integrator dirichlet_time_derivatives
    error_diagnostics estimation expression moving_convergence nodal_state oblique_conditions p1_kernels parallel_assembly
    prepared_assembly prepared_stationary quadrature reporting reusable_evolution solver solver_backends
    step_control step_rejection symbol_contexts time_grid transfer transient)
  pde2d_add_cpp_test(NAME fem_${name} SOURCE ${name}.cpp FEM)
endforeach()
