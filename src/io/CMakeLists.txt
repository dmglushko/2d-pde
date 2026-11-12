# libxml2 parses SVG; it is used only in src/io/svg_import.cpp, never in a
# public header.
find_package(LibXml2 REQUIRED)

add_library(pde2d_io
  gmsh.cpp
  pde2d_mesh.cpp
  svg_export.cpp
  svg_import.cpp
  svg_path.cpp)
add_library(pde2d::io ALIAS pde2d_io)
target_include_directories(pde2d_io PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_compile_features(pde2d_io PUBLIC cxx_std_23)
# io depends on mesh (and through it on geometry), never the other way.
target_link_libraries(pde2d_io PUBLIC pde2d_mesh PRIVATE LibXml2::LibXml2
                                                         pde2d_warnings)
