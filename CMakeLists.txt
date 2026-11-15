cmake_minimum_required(VERSION 3.25)
project(pde2d VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

option(PDE2D_BUILD_TESTS "Build the tests" ON)
option(PDE2D_BUILD_EXAMPLES "Build the example programs" ON)
option(PDE2D_BUILD_APP "Build the graphical interface (app/)" ON)
option(PDE2D_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)

# Warning flags shared by every target of this project.
add_library(pde2d_warnings INTERFACE)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(pde2d_warnings INTERFACE -Wall -Wextra -Wpedantic
                                                  -Wshadow)
  if(PDE2D_WARNINGS_AS_ERRORS)
    target_compile_options(pde2d_warnings INTERFACE -Werror)
  endif()
endif()

add_subdirectory(src/geometry)
add_subdirectory(src/mesh)
add_subdirectory(src/io)

# SymEngine parses and compiles expressions in pde2d_fem, only in its .cpp
# files. Point CMake at an installation with -DSymEngine_DIR=<prefix>/lib/
# cmake/symengine or CMAKE_PREFIX_PATH. Its config file still declares
# compatibility with CMake 2.8, which CMake 4 refuses without a policy floor.
if(CMAKE_VERSION VERSION_GREATER_EQUAL 4.0 AND NOT DEFINED
                                               CMAKE_POLICY_VERSION_MINIMUM)
  set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
endif()
find_package(SymEngine 0.14 CONFIG REQUIRED)
add_subdirectory(src/fem)

# Eigen 3 (header-only) solves the linear systems in pde2d_solve, only in its
# .cpp files.
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
add_subdirectory(src/solve)

# The whole library as one target: link pde2d::pde2d to use every module.
# The modules stay separate libraries (one per module, dependencies one
# way); this target only bundles them.
add_library(pde2d INTERFACE)
add_library(pde2d::pde2d ALIAS pde2d)
target_link_libraries(pde2d INTERFACE pde2d_geometry pde2d_mesh pde2d_io
                                      pde2d_fem pde2d_solve)

if(PDE2D_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

if(PDE2D_BUILD_TESTS)
  # Boost.Multiprecision (header-only) is a test dependency only: the
  # predicate test uses an arbitrary-precision version of the predicates as
  # an oracle for the adaptive ones.
  find_package(Boost 1.74 REQUIRED CONFIG)
  enable_testing()
  add_subdirectory(tests)
endif()

# The graphical interface: a separate program on top of the library
# (app/README.md, docs/app.md). The library never depends on it.
if(PDE2D_BUILD_APP)
  add_subdirectory(app)
endif()
