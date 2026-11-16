#include "case_format.hpp"

#include "app_error.hpp"

#include <cmath>
#include <format>
#include <stdexcept>

namespace pde2d::app {

namespace {

// No migration yet: version 1 is the only one. The first one will be
//   Json migrate_v1_to_v2(Json case_json) { ...; return case_json; }
// listed here as {migrate_v1_to_v2}.
constexpr std::span<const CaseMigration> migrations_of_this_app{};

void check_format(const Json &case_json) {
  require_object(case_json, "case");
  if (const Json &format = member(case_json, "format"); !format.is_null()) {
    if (require_string(format, "format") != case_format)
      throw PathError("format", "this is not a pde2d app case");
  }
}

} // namespace

std::span<const CaseMigration> case_migrations() {
  static_assert(case_version >= 1);
  if (migrations_of_this_app.size() !=
      static_cast<std::size_t>(case_version - 1))
    throw std::logic_error("case_migrations: one migration per version");
  return migrations_of_this_app;
}

int case_version_of(const Json &case_json, int current) {
  const Json &version = member(case_json, "version");
  if (version.is_null())
    return current;
  const double number = require_number(version, "version");
  if (number != std::floor(number) || number < 1)
    throw PathError("version", "expected a whole number from 1");
  if (number > current)
    throw PathError(
        "version",
        std::format("the case was written by a newer version of the app "
                    "(case version {}; this app reads versions 1 to {}); "
                    "open it with a newer pde2d_app",
                    number, current));
  return static_cast<int>(number);
}

std::optional<Json> migrate_case(const Json &case_json, int current,
                                 std::span<const CaseMigration> migrations) {
  check_format(case_json);
  int version = case_version_of(case_json, current);
  if (version == current)
    return std::nullopt;
  if (migrations.size() < static_cast<std::size_t>(current - 1))
    throw std::logic_error("migrate_case: a migration is missing");
  Json out = case_json;
  for (; version < current; ++version) {
    out = migrations[static_cast<std::size_t>(version - 1)](std::move(out));
    out["version"] = version + 1;
  }
  return out;
}

void check_case(const Json &case_json) { (void)CaseInput(case_json); }

CaseInput::CaseInput(const Json &case_json)
    : original_(&case_json), migrated_(migrate_case(case_json)) {
  reject_unknown_keys(json(), "case", case_keys);
}

} // namespace pde2d::app
