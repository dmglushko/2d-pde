cmake_minimum_required(VERSION 3.25)
project(pde2d VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

option(PDE2D_BUILD_TESTS "Build the tests" ON)
option(PDE2D_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)

# Boost.Multiprecision (header-only) is used by the exact predicates, and
# only from src/geometry/predicates.cpp.
find_package(Boost 1.74 REQUIRED CONFIG)

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

if(PDE2D_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
