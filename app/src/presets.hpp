#pragma once

#include "domain_input.hpp"
#include "options.hpp"

#include <string_view>

// Problem presets: complete problems (equation, parameters, a boundary
// condition for every region, initial conditions and matching solve
// settings) for whatever geometry the case has. Lengths and times are
// scaled to the domain (its size and a point well inside it, see
// DomainFacts), so a preset runs on any shape. The page offers them by
// id; all physics stays here (docs/app.md, "Presets").
namespace pde2d::app {

// [{"id", "label", "description"}], in menu order.
Json preset_catalog_json();

// The case with its "problem" and "solve" sections replaced by the preset
// `id` for a domain with `facts`; everything else is kept. Throws
// std::invalid_argument for an unknown id.
Json apply_preset(const Json &case_json, std::string_view id,
                  const DomainFacts &facts);

} // namespace pde2d::app
