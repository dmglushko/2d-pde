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

pde2d_app_add_test(app_compare compare.cpp pde2d_app_core)

pde2d_app_add_test(app_exports exports.cpp pde2d_app_core)

pde2d_app_add_test(app_server server.cpp pde2d_app_server)
target_include_directories(app_server SYSTEM
  PRIVATE ${PROJECT_SOURCE_DIR}/app/third_party/cpp-httplib)
target_compile_definitions(app_server
  PRIVATE PDE2D_APP_WEB_DIR="${PROJECT_SOURCE_DIR}/app/web"
          PDE2D_APP_EXAMPLES_DIR="${PROJECT_SOURCE_DIR}/app/examples")

# The command line on every example case, and its figures against the
# bounds in example_checks.json (app_check_<example>, after the run).
add_executable(app_example_check example_check.cpp)
target_link_libraries(app_example_check
  PRIVATE nlohmann_json::nlohmann_json pde2d_warnings)
file(GLOB example_cases CONFIGURE_DEPENDS
     ${PROJECT_SOURCE_DIR}/app/examples/*.json)
foreach(example IN LISTS example_cases)
  get_filename_component(stem ${example} NAME_WE)
  add_test(NAME app_run_${stem}
    COMMAND pde2d_app --run ${example}
            --output ${CMAKE_CURRENT_BINARY_DIR}/run_${stem})
  set_tests_properties(app_run_${stem} PROPERTIES
    FIXTURES_SETUP app_example_${stem})
  add_test(NAME app_check_${stem}
    COMMAND app_example_check ${CMAKE_CURRENT_SOURCE_DIR}/example_checks.json
            ${stem} ${CMAKE_CURRENT_BINARY_DIR}/run_${stem}/summary.json)
  set_tests_properties(app_check_${stem} PROPERTIES
    FIXTURES_REQUIRED app_example_${stem})
endforeach()

# --sweep: a convergence study of the L-shaped corner prints a row per run.
add_test(NAME app_run_sweep
  COMMAND pde2d_app --run ${PROJECT_SOURCE_DIR}/app/examples/l_shape_corner.json
          --sweep mesh.spec.max_edge_length.relative=0.08,0.04
          --output ${CMAKE_CURRENT_BINARY_DIR}/run_sweep)
set_tests_properties(app_run_sweep PROPERTIES
  PASS_REGULAR_EXPRESSION "rate_l2")

# --set on the command line: a finer mesh of an example runs, and a path
# that does not exist fails naming it.
add_test(NAME app_run_set
  COMMAND pde2d_app --run ${PROJECT_SOURCE_DIR}/app/examples/poisson_rectangle.json
          --set mesh.spec.max_edge_length.relative=0.08
          --set mesh.refiner.options.min_angle_degrees=28)
add_test(NAME app_run_set_missing
  COMMAND pde2d_app --run ${PROJECT_SOURCE_DIR}/app/examples/poisson_rectangle.json
          --set mesh.nothing.here=1)
set_tests_properties(app_run_set_missing PROPERTIES
  PASS_REGULAR_EXPRESSION "mesh.nothing.here: 'nothing' does not exist in mesh")

# The page's pure modules (no browser), when Node.js is installed.
find_program(PDE2D_NODE node)
if(PDE2D_NODE)
  add_test(NAME app_web
    COMMAND ${PDE2D_NODE} ${CMAKE_CURRENT_SOURCE_DIR}/web.test.mjs)
  add_test(NAME app_web_actions
    COMMAND ${PDE2D_NODE} ${CMAKE_CURRENT_SOURCE_DIR}/web_actions.test.mjs)
endif()

# The page in a real browser (headless Chrome), when one is found: the
# PDE2D_CHROME environment variable, else google-chrome or chromium on
# PATH, else the Windows Chrome seen from WSL. Needs python3 for the log.
set(PDE2D_CHROME_PATH "$ENV{PDE2D_CHROME}")
if(NOT PDE2D_CHROME_PATH)
  find_program(PDE2D_CHROME_PROGRAM
    NAMES google-chrome google-chrome-stable chromium chromium-browser)
  if(PDE2D_CHROME_PROGRAM)
    set(PDE2D_CHROME_PATH "${PDE2D_CHROME_PROGRAM}")
  elseif(EXISTS "/mnt/c/Program Files/Google/Chrome/Application/chrome.exe")
    set(PDE2D_CHROME_PATH
        "/mnt/c/Program Files/Google/Chrome/Application/chrome.exe")
  endif()
endif()
find_program(PDE2D_PYTHON3 python3)
find_program(PDE2D_BASH bash)
if(PDE2D_CHROME_PATH AND PDE2D_PYTHON3 AND PDE2D_BASH)
  add_test(NAME app_browser
    COMMAND ${PDE2D_BASH} ${CMAKE_CURRENT_SOURCE_DIR}/browser/run.sh
            --app $<TARGET_FILE:pde2d_app>)
  set_tests_properties(app_browser PROPERTIES
    ENVIRONMENT "PDE2D_CHROME=${PDE2D_CHROME_PATH}"
    TIMEOUT 900
    RUN_SERIAL TRUE)
endif()
