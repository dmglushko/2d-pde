# Writes OUTPUT (a C++ file) with the git commit of SOURCE_DIR and whether
# its tree has uncommitted changes, or "unknown" outside git. Run at every
# build (app/CMakeLists.txt); the file is rewritten only when its text
# changes, so an unchanged tree rebuilds nothing. Never fails the build.
set(commit "unknown")
set(dirty "unknown")
find_package(Git QUIET)
if(GIT_FOUND)
  execute_process(COMMAND "${GIT_EXECUTABLE}" rev-parse HEAD
    WORKING_DIRECTORY "${SOURCE_DIR}"
    OUTPUT_VARIABLE head RESULT_VARIABLE head_result
    OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
  if(head_result EQUAL 0 AND head MATCHES "^[0-9a-f]+$")
    set(commit "${head}")
    execute_process(COMMAND "${GIT_EXECUTABLE}" status --porcelain
        --untracked-files=no
      WORKING_DIRECTORY "${SOURCE_DIR}"
      OUTPUT_VARIABLE changes RESULT_VARIABLE status_result ERROR_QUIET)
    if(status_result EQUAL 0)
      if(changes STREQUAL "")
        set(dirty "false")
      else()
        set(dirty "true")
      endif()
    endif()
  endif()
endif()
set(text "// Written by app/cmake/build_info.cmake at build time.
#include \"build_info.hpp\"

namespace pde2d::app {

BuildInfo build_info() { return {.commit = \"${commit}\", .dirty = \"${dirty}\"}; }

} // namespace pde2d::app
")
if(EXISTS "${OUTPUT}")
  file(READ "${OUTPUT}" old)
else()
  set(old "")
endif()
if(NOT old STREQUAL text)
  file(WRITE "${OUTPUT}" "${text}")
endif()
