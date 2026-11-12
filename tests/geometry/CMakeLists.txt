pde2d_add_test(NAME geometry_predicates SOURCE predicates.cpp
               LIBS pde2d_geometry)
pde2d_add_test(NAME geometry_predicates_oracle SOURCE predicates_oracle.cpp
               LIBS pde2d_geometry Boost::headers)
pde2d_add_test(NAME geometry_curve SOURCE curve.cpp LIBS pde2d_geometry)
pde2d_add_test(NAME geometry_polygon SOURCE polygon.cpp LIBS pde2d_geometry)
pde2d_add_test(NAME geometry_curve_geometry SOURCE curve_geometry.cpp
               LIBS pde2d_geometry)
pde2d_add_test(NAME geometry_regions SOURCE regions.cpp LIBS pde2d_geometry)
pde2d_add_test(NAME geometry_shapes SOURCE shapes.cpp LIBS pde2d_geometry)
