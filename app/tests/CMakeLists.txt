# Tests of the app. They use tests/support/check.hpp like the library's
# tests; the app's own headers are private and tested directly.
function(pde2d_app_add_test name source)
  add_executable(${name} ${source})
  target_include_directories(${name} PRIVATE ${PROJECT_SOURCE_DIR}/tests)
  target_link_libraries(${name} PRIVATE ${ARGN} pde2d_warnings)
  add_test(NAME ${name} COMMAND ${name})
endfunction()

pde2d_app_add_test(app_session session.cpp pde2d_app_core)
target_compile_definitions(app_session
  PRIVATE PDE2D_APP_EXAMPLES_DIR="${PROJECT_SOURCE_DIR}/app/examples")

pde2d_app_add_test(app_solve solve.cpp pde2d_app_core)
target_compile_definitions(app_solve
  PRIVATE PDE2D_APP_EXAMPLES_DIR="${PROJECT_SOURCE_DIR}/app/examples")

pde2d_app_add_test(app_runs runs.cpp pde2d_app_core)

pde2d_app_add_test(app_server server.cpp pde2d_app_server)
target_include_directories(app_server SYSTEM
  PRIVATE ${PROJECT_SOURCE_DIR}/app/third_party/cpp-httplib)
target_compile_definitions(app_server
  PRIVATE PDE2D_APP_WEB_DIR="${PROJECT_SOURCE_DIR}/app/web"
          PDE2D_APP_EXAMPLES_DIR="${PROJECT_SOURCE_DIR}/app/examples")

# The command line on every example case.
file(GLOB example_cases CONFIGURE_DEPENDS
     ${PROJECT_SOURCE_DIR}/app/examples/*.json)
foreach(example IN LISTS example_cases)
  get_filename_component(stem ${example} NAME_WE)
  add_test(NAME app_run_${stem}
    COMMAND pde2d_app --run ${example}
            --output ${CMAKE_CURRENT_BINARY_DIR}/run_${stem})
endforeach()

# The page's pure modules (no browser), when Node.js is installed.
find_program(PDE2D_NODE node)
if(PDE2D_NODE)
  add_test(NAME app_web
    COMMAND ${PDE2D_NODE} ${CMAKE_CURRENT_SOURCE_DIR}/web.test.mjs)
  add_test(NAME app_web_actions
    COMMAND ${PDE2D_NODE} ${CMAKE_CURRENT_SOURCE_DIR}/web_actions.test.mjs)
endif()
