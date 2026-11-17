# pde2d_add_test(NAME <name> SOURCE <file> LIBS <targets...>)
# Builds one test program and registers it with CTest.
function(pde2d_add_test)
  cmake_parse_arguments(ARG "" "NAME;SOURCE" "LIBS" ${ARGN})
  add_executable(${ARG_NAME} ${ARG_SOURCE})
  target_include_directories(${ARG_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/tests)
  target_link_libraries(${ARG_NAME} PRIVATE ${ARG_LIBS} pde2d_warnings)
  add_test(NAME ${ARG_NAME} COMMAND ${ARG_NAME})
endfunction()

add_subdirectory(geometry)
add_subdirectory(mesh)
add_subdirectory(io)
add_subdirectory(fem)
add_subdirectory(solve)
add_subdirectory(adapt)

# Every public header must compile on its own. One generated source per
# header is compiled (not linked) into an object library.
file(GLOB_RECURSE PDE2D_PUBLIC_HEADERS CONFIGURE_DEPENDS
     ${PROJECT_SOURCE_DIR}/include/pde2d/*.hpp)
set(PDE2D_HEADER_SOURCES "")
foreach(header IN LISTS PDE2D_PUBLIC_HEADERS)
  file(RELATIVE_PATH relative ${PROJECT_SOURCE_DIR}/include ${header})
  string(MAKE_C_IDENTIFIER ${relative} stem)
  set(source ${CMAKE_CURRENT_BINARY_DIR}/header_check/${stem}.cpp)
  file(CONFIGURE OUTPUT ${source} CONTENT "#include \"${relative}\"\n")
  list(APPEND PDE2D_HEADER_SOURCES ${source})
endforeach()
add_library(header_standalone OBJECT ${PDE2D_HEADER_SOURCES})
target_link_libraries(header_standalone PRIVATE pde2d_io pde2d_solve
                                                pde2d_adapt pde2d_warnings)
