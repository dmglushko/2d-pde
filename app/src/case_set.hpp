#pragma once

#include "options.hpp"

#include <string_view>

// --set PATH=VALUE of the command line: one value of a case replaced
// before it runs (docs/app.md, "The command line").
namespace pde2d::app {

// Sets the value at `path` (dotted keys with [i] indices:
// "mesh.spec.max_edge_length.relative", "mesh.improvers[0].options.use")
// to `value`, read as JSON or else taken as a string. Every step before
// the last must exist; the last may be a new key of an object (the case
// reader refuses one it does not know). Throws PathError naming the path
// for a step that does not exist or a malformed path.
void set_case_value(Json &case_json, std::string_view path,
                    std::string_view value);

// The same from "PATH=VALUE".
void apply_set_argument(Json &case_json, std::string_view argument);

} // namespace pde2d::app
