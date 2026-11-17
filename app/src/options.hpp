#pragma once

#include "pde2d/adapt/adaptive_static_solver.hpp"
#include "pde2d/mesh/mesh_spec.hpp"

#include <nlohmann/json.hpp>

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

// Options as the app sees them: a described list (what the page builds its
// forms from) and a reader that turns a JSON object into typed values.
//
// The app never checks an option's range: the library component that gets
// the value validates it in its constructor, and its message is shown.
// The reader checks only what JSON adds: the type of each value and that
// every key is a known option.
namespace pde2d::app {

// Keys keep the order they were written in, so a saved case reads like the
// form it came from.
using Json = nlohmann::ordered_json;

enum class OptionKind {
  number,       // a finite double
  integer,      // a whole number
  choice,       // one of OptionSpec::choices
  length_bound, // {"absolute": number or null, "relative": number or null}
  tolerance,    // the same object; relative to the first error estimate
};

struct OptionSpec {
  std::string name;        // the JSON key
  std::string label;       // for the form
  std::string description; // one or two sentences, for the form
  OptionKind kind = OptionKind::number;
  Json default_value;               // what a missing key means
  std::vector<std::string> choices; // for OptionKind::choice
  std::string unit;                 // shown after the field, may be empty
};

// "number", "integer", "choice", "length_bound" or "tolerance".
std::string_view option_kind_name(OptionKind kind);

// What the page receives for one option.
Json to_json(const OptionSpec &spec);
Json to_json(std::span<const OptionSpec> specs);

// {"absolute": ..., "relative": ...} with null for a value that is not set.
Json to_json(const mesh::LengthBound &bound);

// The options object with every default filled in.
Json defaults(std::span<const OptionSpec> specs);

// Reads the object `options` against `specs`. `where` is the path of the
// object in the case file ("mesh.refiner.options"); every error names it.
// The constructor throws std::invalid_argument when `options` is not an
// object (null counts as an empty one) or has a key that no spec names.
// A getter throws std::invalid_argument for a value of the wrong type and
// std::logic_error for a name that is not in the specs or has another
// kind (a mistake in the catalog, not in the input).
class OptionReader {
public:
  OptionReader(const Json &options, std::string where,
               std::span<const OptionSpec> specs);

  double number(std::string_view name) const;
  // A whole number; 3.0 counts as 3. Throws std::invalid_argument for a
  // fraction or a value outside long long.
  long long integer(std::string_view name) const;
  // A whole number that fits an int (std::invalid_argument otherwise).
  int small_integer(std::string_view name) const;
  // A whole number that is not negative.
  std::size_t count(std::string_view name) const;
  std::string choice(std::string_view name) const;
  mesh::LengthBound length_bound(std::string_view name) const;
  adapt::ErrorTolerance tolerance(std::string_view name) const;
  // Reads every option by its kind (and a choice against its choices),
  // throwing what the getters throw; for a part of a case that is checked
  // but not built.
  void check_kinds() const;

private:
  const OptionSpec &spec(std::string_view name, OptionKind kind) const;
  // The value written for the option, or its default.
  const Json &value(const OptionSpec &spec) const;
  std::string path(std::string_view name) const;

  Json options_;
  std::string where_;
  std::span<const OptionSpec> specs_;
};

// The option a component's (or shape's) constructor message is about, as
// a path suffix (".min_angle_degrees", or ".max_curve_deviation.absolute"
// for a part of a length bound), or "" when it names none. An option is
// named when its name stands as a word: at the start, or after "::", ": "
// or a space, and followed by the end, a space, ':' or '.' (the library
// writes "RuppertOptions::min_angle_degrees must be ...", "fixed_step: dt
// must be ..."). The first option named in the message wins.
std::string named_option(std::string_view message,
                         std::span<const OptionSpec> specs);

// Small JSON readers shared by the app, with errors that name `where`.
// All throw PathError (app_error.hpp), an std::invalid_argument.
const Json &require_object(const Json &value, std::string_view where);
// The member `key` of the object `object`, or null when it is missing.
const Json &member(const Json &object, std::string_view key);
double require_number(const Json &value, std::string_view where);
std::string require_string(const Json &value, std::string_view where);
// Rejects a key of `object` that is not in `known`.
void reject_unknown_keys(const Json &object, std::string_view where,
                         std::span<const std::string_view> known);

// A run's id from a request {"job": n, ...} with only the keys `known`.
std::uint64_t request_job(const Json &request,
                          std::span<const std::string_view> known);
// A whole number from 0 at `key` of a request.
std::size_t request_count(const Json &request, std::string_view key);

} // namespace pde2d::app
