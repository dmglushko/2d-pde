#pragma once

#include "options.hpp"

#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <string_view>

// The app's work, independent of HTTP: each call takes a case (the whole
// setup as JSON, see docs/app.md) and returns JSON for the page. The
// server and the command line (--run) both use it.
//
// A Session keeps the results of the last call of each stage, keyed by the
// sections they were built from, so meshing again after a change to the
// problem reuses the mesh. Calls are serialized by one mutex: the library's
// symbolic work must not run in two threads at once (SymEngine's
// reference counts are not thread-safe in this build), and a local app has
// one user.
namespace pde2d::app {

// A stage failed. `kind` is "input" for std::invalid_argument (the
// library refused the input or the settings), "algorithm" for
// std::runtime_error (an algorithm could not finish) and "internal" for
// anything else.
class StageError : public std::runtime_error {
public:
  StageError(std::string stage, std::string kind, const std::string &message)
      : std::runtime_error(message), stage_(std::move(stage)),
        kind_(std::move(kind)) {}
  const std::string &stage() const { return stage_; }
  const std::string &kind() const { return kind_; }

private:
  std::string stage_;
  std::string kind_;
};

struct ExportedFile {
  std::string name;
  std::string content_type;
  std::string content;
};

class Session {
public:
  Session();
  ~Session();
  Session(const Session &) = delete;
  Session &operator=(const Session &) = delete;

  // Everything the page builds its forms from (docs/app.md, "The
  // catalog").
  static Json catalog();
  // A complete case to start from.
  static Json default_case();

  // Each throws StageError. Every call reads only the sections it needs.
  Json geometry(const Json &case_json);
  Json mesh(const Json &case_json);
  Json check_problem(const Json &case_json);
  Json assemble(const Json &case_json);
  // request: {"case": ..., "field": {"kind": ..., ...}}; see docs/app.md.
  Json field(const Json &request);
  // kind: "mesh.pde2d", "mesh.svg" or "geometry.svg".
  ExportedFile export_file(const Json &case_json, std::string_view kind);

  // Runs every stage the case allows, for the command line: geometry,
  // mesh, problem check, assembly. Stops at the first failing stage and
  // reports it under "error". "ok" is true when every stage ran.
  Json run(const Json &case_json);

private:
  struct State;
  std::mutex mutex_;
  std::unique_ptr<State> state_;
};

// Throws std::invalid_argument for a case that is not an object, has a
// key that is not a section, or names another format or a newer version.
void check_case(const Json &case_json);

} // namespace pde2d::app
