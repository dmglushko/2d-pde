# Explicit test registration. Each test declares what it needs; nothing is
# discovered by globbing or inferred from file names.
#
#   pde2d_add_cpp_test(NAME <ctest-and-target-name> SOURCE <file>
#                      [FEM] [LIBXML2] [FIXTURE_ARG]
#                      [FIXTURE_FILES <repo-relative file>...])
#
#   FEM            links pde2d_fem (Eigen/SymEngine); skipped when PDE2D_BUILD_FEM=OFF.
#   LIBXML2        links LibXml2; skipped with an UNAVAILABLE note when missing.
#   FIXTURE_ARG    passes the test's private fixture directory as argv[1].
#   FIXTURE_FILES  copies repository files into the fixture directory, keeping
#                  their repo-relative paths.
# Every test runs in ${CMAKE_BINARY_DIR}/fixtures/<NAME> (see pde2d_test).
function(pde2d_add_cpp_test)
  cmake_parse_arguments(T "FEM;LIBXML2;FIXTURE_ARG" "NAME;SOURCE" "FIXTURE_FILES" ${ARGN})
  if(NOT T_NAME OR NOT T_SOURCE)
    message(FATAL_ERROR "pde2d_add_cpp_test requires NAME and SOURCE")
  endif()
  if(T_FEM AND NOT PDE2D_BUILD_FEM)
    return()
  endif()
  if(T_LIBXML2 AND NOT LibXml2_FOUND)
    message(STATUS "UNAVAILABLE: ${T_NAME} (LibXml2)")
    return()
  endif()
  add_executable(${T_NAME} "${T_SOURCE}")
  target_link_libraries(${T_NAME} PRIVATE pde2d_execution Threads::Threads)
  target_include_directories(${T_NAME} PRIVATE "${PROJECT_SOURCE_DIR}/tests")
  if(T_FEM)
    target_link_libraries(${T_NAME} PRIVATE pde2d_fem)
  endif()
  if(T_LIBXML2)
    target_link_libraries(${T_NAME} PRIVATE LibXml2::LibXml2)
  endif()
  set(fixture_dir "${CMAKE_BINARY_DIR}/fixtures/${T_NAME}")
  file(MAKE_DIRECTORY "${fixture_dir}")
  foreach(file IN LISTS T_FIXTURE_FILES)
    get_filename_component(relative_dir "${file}" DIRECTORY)
    file(MAKE_DIRECTORY "${fixture_dir}/${relative_dir}")
    configure_file("${PROJECT_SOURCE_DIR}/${file}" "${fixture_dir}/${file}" COPYONLY)
  endforeach()
  if(T_FIXTURE_ARG)
    pde2d_test(${T_NAME} $<TARGET_FILE:${T_NAME}> "${fixture_dir}")
  else()
    pde2d_test(${T_NAME} $<TARGET_FILE:${T_NAME}>)
  endif()
endfunction()

add_subdirectory(core)
add_subdirectory(headers)
add_subdirectory(discretization)
add_subdirectory(fem)
add_subdirectory(tools)
