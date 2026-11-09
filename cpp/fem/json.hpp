#pragma once
// The JSON document of the FEM records (record_json.hpp, adaptivity_json.hpp,
// reference/compare.hpp). It lives in the core header cpp/json.hpp (namespace
// pde2d), shared with the mesh run records of discretization; these
// declarations keep the pde2d::fem names.
#include "../json.hpp"

namespace pde2d::fem {
using pde2d::json_bool;
using pde2d::json_number;
using pde2d::json_string;
using pde2d::json_text;
using pde2d::JsonValue;
using pde2d::parse_json;
using pde2d::write_json;
} // namespace pde2d::fem
