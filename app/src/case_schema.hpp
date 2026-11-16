#pragma once

#include "options.hpp"

// A JSON Schema (draft 2020-12) of the case, for editors and scripts that
// write cases (GET /api/case-schema). It is generated, not written by
// hand: the components and their options come from the catalogs (with
// their kinds, choices and defaults), the shapes from the shape catalog,
// the coefficients from the problem catalog, and each section's keys from
// the key lists its reader checks against (domain_input.hpp,
// problem_input.hpp, stage_cache.hpp, catalog.hpp, case_format.hpp). A
// key list with a key the generator does not describe is a
// std::logic_error, so a reader cannot gain a key the schema lacks.
//
// The schema says what the readers accept as structure and value types;
// ranges are the library's to check (they are not in the schema), so a
// case that validates may still be refused with the library's message.
namespace pde2d::app {

Json case_schema();

} // namespace pde2d::app
