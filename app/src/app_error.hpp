#pragma once

#include "options.hpp"

#include <stdexcept>
#include <string>
#include <utility>

// The app's errors (docs/app.md, "Errors").
namespace pde2d::app {

// Input refused at a place in the case: `path` is that place
// ("mesh.refiner.options.min_angle_degrees", "geometry.region_edits[1]"),
// `detail` what is wrong there. what() is "path: detail", so a message
// stays readable where only the text is shown (the command line). It is an
// std::invalid_argument, so code that only knows the library's error
// policy treats it as refused input.
class PathError : public std::invalid_argument {
public:
  PathError(std::string path, std::string detail)
      : std::invalid_argument(path.empty() ? detail : path + ": " + detail),
        path_(std::move(path)), detail_(std::move(detail)) {}
  const std::string &path() const { return path_; }
  const std::string &detail() const { return detail_; }

private:
  std::string path_;
  std::string detail_;
};

// Runs `step`; an std::invalid_argument it throws (the library's, which
// has no path) becomes a PathError at `path`. A PathError passes through
// unchanged: it already names its place, which is inside `path`.
template <class Step> auto at_path(const std::string &path, Step step) {
  try {
    return step();
  } catch (const PathError &) {
    throw;
  } catch (const std::invalid_argument &error) {
    throw PathError(path, error.what());
  }
}

// A stage failed. `kind` is "input" for std::invalid_argument (the
// library refused the input or the settings), "algorithm" for
// std::runtime_error (an algorithm could not finish) and "internal" for
// anything else. `path` is the place in the case the error is about, or
// empty when the error names none (most of the library's own messages).
class StageError : public std::runtime_error {
public:
  StageError(std::string stage, std::string kind, const std::string &message,
             std::string path = {})
      : std::runtime_error(message), stage_(std::move(stage)),
        kind_(std::move(kind)), path_(std::move(path)) {}
  const std::string &stage() const { return stage_; }
  const std::string &kind() const { return kind_; }
  const std::string &path() const { return path_; }

private:
  std::string stage_;
  std::string kind_;
  std::string path_;
};

// {"stage", "kind", "message", "path"} (path null when there is none).
Json error_json(const StageError &error);
// The inverse of error_json.
StageError error_from_json(const Json &error);

// Runs one stage and turns its exception into a StageError.
template <class Step> auto staged(std::string_view stage, Step step) {
  try {
    return step();
  } catch (const StageError &) {
    throw;
  } catch (const PathError &error) {
    throw StageError(std::string(stage), "input", error.what(), error.path());
  } catch (const std::invalid_argument &error) {
    throw StageError(std::string(stage), "input", error.what());
  } catch (const std::runtime_error &error) {
    throw StageError(std::string(stage), "algorithm", error.what());
  } catch (const std::exception &error) {
    throw StageError(std::string(stage), "internal", error.what());
  }
}

} // namespace pde2d::app
