#pragma once

#include "options.hpp"

#include <optional>
#include <span>
#include <string_view>

// The envelope of a case: its format name, its version and its sections
// (docs/app.md, "The case"), and reading a case written by an older app.
namespace pde2d::app {

inline constexpr std::string_view case_format = "pde2d-app-case";
// The version this app writes. Raise it when a change of the case format
// needs older cases changed to be read, and add the migration from the
// previous version to case_migrations().
inline constexpr int case_version = 1;

// The keys a case may have.
inline constexpr std::string_view case_keys[] = {
    "format",   "version", "geometry", "mesh", "problem",
    "assembly", "solve",   "view",     "notes"};

// Turns a case of version N into one of version N + 1 (and sets its
// "version" to N + 1).
using CaseMigration = Json (*)(Json);

// The migrations of this app: element i takes version i + 1 to i + 2, so
// there are case_version - 1 of them (none yet).
std::span<const CaseMigration> case_migrations();

// The case's version: 1 to `current` (a case without one is `current`).
// Throws std::invalid_argument (a PathError at "version") for a version
// that is not a whole number from 1, or one newer than `current` (the
// case was written by a newer app).
int case_version_of(const Json &case_json, int current = case_version);

// The case brought to `current` with `migrations` (element i takes
// version i + 1 to i + 2), or nothing when it already is (no copy). Throws
// like case_version_of, and std::logic_error when the chain is shorter
// than needed.
std::optional<Json>
migrate_case(const Json &case_json, int current = case_version,
             std::span<const CaseMigration> migrations = case_migrations());

// Throws std::invalid_argument for a case that is not an object, names
// another format, has a version this app cannot read, or (after its
// migration) has a key that is not a section.
void check_case(const Json &case_json);

// A case as the session reads it: checked and brought to the current
// version. Holds a migrated copy only when the case was older.
class CaseInput {
public:
  // Throws like check_case.
  explicit CaseInput(const Json &case_json);
  const Json &json() const { return migrated_ ? *migrated_ : *original_; }

private:
  const Json *original_;
  std::optional<Json> migrated_;
};

} // namespace pde2d::app
