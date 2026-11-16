# The graphical interface of pde2d (docs/app.md): a local web server over
# the library and the page it serves. It uses only the library's public
# headers, through pde2d::pde2d. Its own dependencies stay here:
# nlohmann/json (installed) and cpp-httplib (one header, vendored in
# third_party/, used only in src/server.cpp).
find_package(nlohmann_json 3.10 CONFIG REQUIRED)
find_package(Threads REQUIRED)

# Everything but HTTP: the case format, the catalog of components and the
# stages. Tests link it directly.
# Which commit the app is built from, for the exports' provenance: written
# at every build (cmake/build_info.cmake), rewritten only when it changes,
# "unknown" outside git.
set(PDE2D_APP_BUILD_INFO "${CMAKE_CURRENT_BINARY_DIR}/build_info.cpp")
set(PDE2D_APP_BUILD_INFO_COMMAND
  "${CMAKE_COMMAND}" -DSOURCE_DIR=${PROJECT_SOURCE_DIR}
  -DOUTPUT=${PDE2D_APP_BUILD_INFO}
  -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/build_info.cmake")
execute_process(COMMAND ${PDE2D_APP_BUILD_INFO_COMMAND})
add_custom_target(pde2d_app_build_info
  COMMAND ${PDE2D_APP_BUILD_INFO_COMMAND}
  BYPRODUCTS "${PDE2D_APP_BUILD_INFO}"
  COMMENT "Recording the app's git commit"
  VERBATIM)

add_library(pde2d_app_core STATIC
  src/case_format.cpp
  src/case_schema.cpp
  src/case_set.cpp
  src/domain_input.cpp
  src/drawing.cpp
  src/exports.cpp
  src/field_eval.cpp
  src/locate.cpp
  src/mesh_catalog.cpp
  src/options.cpp
  src/presets.cpp
  src/problem_input.cpp
  src/runs.cpp
  src/session.cpp
  src/sha256.cpp
  src/solve_catalog.cpp
  src/sweep.cpp
  src/stage_cache.cpp
  src/views.cpp
  src/zip.cpp
  "${PDE2D_APP_BUILD_INFO}")
add_dependencies(pde2d_app_core pde2d_app_build_info)
target_include_directories(pde2d_app_core PUBLIC src)
target_compile_definitions(pde2d_app_core
  PRIVATE PDE2D_VERSION="${PROJECT_VERSION}")
target_link_libraries(pde2d_app_core
  PUBLIC pde2d::pde2d nlohmann_json::nlohmann_json PRIVATE pde2d_warnings)

add_library(pde2d_app_server STATIC src/server.cpp)
target_include_directories(pde2d_app_server SYSTEM
  PRIVATE third_party/cpp-httplib)
target_link_libraries(pde2d_app_server
  PUBLIC pde2d_app_core Threads::Threads PRIVATE pde2d_warnings)

add_executable(pde2d_app src/main.cpp)
# The page files are served from the source tree, so editing them needs
# no rebuild; --web DIR serves another copy.
target_compile_definitions(pde2d_app
  PRIVATE PDE2D_APP_WEB_DIR="${CMAKE_CURRENT_SOURCE_DIR}/web"
          PDE2D_APP_EXAMPLES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/examples")
target_link_libraries(pde2d_app PRIVATE pde2d_app_server pde2d_warnings)

if(PDE2D_BUILD_TESTS)
  add_subdirectory(tests)
endif()
