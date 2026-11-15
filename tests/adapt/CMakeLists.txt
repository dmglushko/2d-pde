pde2d_add_test(NAME adapt_marking SOURCE marking.cpp LIBS pde2d_adapt)
pde2d_add_test(NAME adapt_refinement SOURCE refinement.cpp LIBS pde2d_adapt)
pde2d_add_test(NAME adapt_curved SOURCE curved.cpp LIBS pde2d_adapt)
pde2d_add_test(NAME adapt_repeated SOURCE repeated.cpp LIBS pde2d_adapt)
