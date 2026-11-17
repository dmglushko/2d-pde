#include "options.hpp"

#include "app_error.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>

namespace pde2d::app {

namespace {

const Json null_json = nullptr;

} // namespace

std::string_view option_kind_name(OptionKind kind) {
  switch (kind) {
  case OptionKind::number:
    return "number";
  case OptionKind::integer:
    return "integer";
  case OptionKind::choice:
    return "choice";
  case OptionKind::length_bound:
    return "length_bound";
  case OptionKind::tolerance:
    return "tolerance";
  }
  throw std::logic_error("unknown option kind");
}

namespace {

std::string_view kind_name(OptionKind kind) { return option_kind_name(kind); }

[[noreturn]] void fail(std::string_view where, std::string_view what) {
  throw PathError(std::string(where), std::string(what));
}

std::optional<double> optional_length(const Json &bound, std::string_view key,
                                      const std::string &where) {
  const Json &value = member(bound, key);
  if (value.is_null())
    return std::nullopt;
  return require_number(value, where + "." + std::string(key));
}

} // namespace

Json to_json(const OptionSpec &spec) {
  Json out = Json::object();
  out["name"] = spec.name;
  out["label"] = spec.label;
  out["description"] = spec.description;
  out["kind"] = kind_name(spec.kind);
  out["default"] = spec.default_value;
  if (spec.kind == OptionKind::choice)
    out["choices"] = spec.choices;
  if (!spec.unit.empty())
    out["unit"] = spec.unit;
  return out;
}

Json to_json(std::span<const OptionSpec> specs) {
  Json out = Json::array();
  for (const OptionSpec &spec : specs)
    out.push_back(to_json(spec));
  return out;
}

Json to_json(const mesh::LengthBound &bound) {
  Json out = Json::object();
  out["absolute"] = bound.absolute ? Json(*bound.absolute) : Json(nullptr);
  out["relative"] = bound.relative ? Json(*bound.relative) : Json(nullptr);
  return out;
}

Json defaults(std::span<const OptionSpec> specs) {
  Json out = Json::object();
  for (const OptionSpec &spec : specs)
    out[spec.name] = spec.default_value;
  return out;
}

OptionReader::OptionReader(const Json &options, std::string where,
                           std::span<const OptionSpec> specs)
    : options_(options.is_null() ? Json::object() : options),
      where_(std::move(where)), specs_(specs) {
  require_object(options_, where_);
  for (const auto &item : options_.items()) {
    const bool known =
        std::ranges::any_of(specs_, [&](const OptionSpec &option_spec) {
          return option_spec.name == item.key();
        });
    if (!known)
      fail(where_, "unknown option '" + item.key() + "'");
  }
}

const OptionSpec &OptionReader::spec(std::string_view name,
                                     OptionKind kind) const {
  for (const OptionSpec &candidate : specs_)
    if (candidate.name == name) {
      if (candidate.kind != kind)
        throw std::logic_error("option '" + std::string(name) + "' is not a " +
                               std::string(kind_name(kind)));
      return candidate;
    }
  throw std::logic_error("option '" + std::string(name) +
                         "' is not in the catalog");
}

const Json &OptionReader::value(const OptionSpec &option_spec) const {
  const auto found = options_.find(option_spec.name);
  if (found == options_.end() || found->is_null())
    return option_spec.default_value;
  return *found;
}

std::string OptionReader::path(std::string_view name) const {
  return where_ + "." + std::string(name);
}

double OptionReader::number(std::string_view name) const {
  return require_number(value(spec(name, OptionKind::number)), path(name));
}

long long OptionReader::integer(std::string_view name) const {
  const Json &json = value(spec(name, OptionKind::integer));
  if (json.is_number_unsigned() &&
      json.get<unsigned long long>() >
          static_cast<unsigned long long>(
              std::numeric_limits<long long>::max()))
    fail(path(name), "is out of range");
  if (json.is_number_integer())
    return json.get<long long>();
  // A form may send 3.0 for 3.
  if (json.is_number_float()) {
    const double number = json.get<double>();
    if (std::isfinite(number) && number == std::floor(number) &&
        std::abs(number) < 9.0e15)
      return static_cast<long long>(number);
  }
  fail(path(name), "expected a whole number");
}

int OptionReader::small_integer(std::string_view name) const {
  const long long number = integer(name);
  if (number < std::numeric_limits<int>::min() ||
      number > std::numeric_limits<int>::max())
    fail(path(name), "is out of range");
  return static_cast<int>(number);
}

std::size_t OptionReader::count(std::string_view name) const {
  const long long number = integer(name);
  if (number < 0)
    fail(path(name), "must not be negative");
  return static_cast<std::size_t>(number);
}

std::string OptionReader::choice(std::string_view name) const {
  const OptionSpec &option_spec = spec(name, OptionKind::choice);
  const std::string text = require_string(value(option_spec), path(name));
  if (std::ranges::find(option_spec.choices, text) ==
      option_spec.choices.end()) {
    std::string list;
    for (const std::string &candidate : option_spec.choices)
      list += (list.empty() ? "" : ", ") + candidate;
    fail(path(name), "'" + text + "' is not one of: " + list);
  }
  return text;
}

mesh::LengthBound OptionReader::length_bound(std::string_view name) const {
  const Json &json = value(spec(name, OptionKind::length_bound));
  const std::string where = path(name);
  if (json.is_null())
    return {};
  require_object(json, where);
  static constexpr std::string_view known[] = {"absolute", "relative"};
  reject_unknown_keys(json, where, known);
  return {.absolute = optional_length(json, "absolute", where),
          .relative = optional_length(json, "relative", where)};
}

adapt::ErrorTolerance OptionReader::tolerance(std::string_view name) const {
  const Json &json = value(spec(name, OptionKind::tolerance));
  const std::string where = path(name);
  if (json.is_null())
    return {};
  require_object(json, where);
  static constexpr std::string_view known[] = {"absolute", "relative"};
  reject_unknown_keys(json, where, known);
  return {.absolute = optional_length(json, "absolute", where),
          .relative = optional_length(json, "relative", where)};
}

void OptionReader::check_kinds() const {
  for (const OptionSpec &option : specs_)
    switch (option.kind) {
    case OptionKind::number:
      (void)number(option.name);
      break;
    case OptionKind::integer:
      (void)integer(option.name);
      break;
    case OptionKind::choice:
      (void)choice(option.name);
      break;
    case OptionKind::length_bound:
      (void)length_bound(option.name);
      break;
    case OptionKind::tolerance:
      (void)tolerance(option.name);
      break;
    }
}

const Json &require_object(const Json &value, std::string_view where) {
  if (!value.is_object())
    fail(where, "expected an object");
  return value;
}

const Json &member(const Json &object, std::string_view key) {
  if (!object.is_object())
    return null_json;
  const auto found = object.find(key);
  return found == object.end() ? null_json : *found;
}

double require_number(const Json &value, std::string_view where) {
  if (!value.is_number())
    fail(where, "expected a number");
  const double number = value.get<double>();
  if (!std::isfinite(number))
    fail(where, "expected a finite number");
  return number;
}

std::string require_string(const Json &value, std::string_view where) {
  if (!value.is_string())
    fail(where, "expected text");
  return value.get<std::string>();
}

void reject_unknown_keys(const Json &object, std::string_view where,
                         std::span<const std::string_view> known) {
  for (const auto &item : object.items())
    if (std::ranges::find(known, std::string_view(item.key())) == known.end())
      fail(where, "unknown key '" + item.key() + "'");
}

std::string named_option(std::string_view message,
                         std::span<const OptionSpec> specs) {
  const auto is_word = [](char c) {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
           (c >= '0' && c <= '9') || c == '_';
  };
  std::size_t best = std::string_view::npos;
  std::string suffix;
  for (const OptionSpec &spec : specs) {
    for (std::size_t at = message.find(spec.name);
         at != std::string_view::npos && at < best;
         at = message.find(spec.name, at + 1)) {
      const std::size_t end = at + spec.name.size();
      const bool starts = at == 0 || !is_word(message[at - 1]);
      const bool ends = end == message.size() || !is_word(message[end]);
      if (!starts || !ends)
        continue;
      best = at;
      suffix = "." + spec.name;
      if (spec.kind == OptionKind::length_bound ||
          spec.kind == OptionKind::tolerance)
        for (const std::string_view part : {".absolute", ".relative"})
          if (message.substr(end).starts_with(part))
            suffix += part;
      break;
    }
  }
  return suffix;
}

std::uint64_t request_job(const Json &request,
                          std::span<const std::string_view> known) {
  require_object(request, "request");
  reject_unknown_keys(request, "request", known);
  const double job = require_number(member(request, "job"), "job");
  if (job != std::floor(job) || job < 1 || job > 9e15)
    fail("job", "expected a run's id");
  return static_cast<std::uint64_t>(job);
}

std::size_t request_count(const Json &request, std::string_view key) {
  const double value = require_number(member(request, key), key);
  if (value != std::floor(value) || value < 0 || value > 1e9)
    fail(key, "expected a whole number from 0");
  return static_cast<std::size_t>(value);
}

Json error_json(const StageError &error) {
  Json out = Json::object();
  out["stage"] = error.stage();
  out["kind"] = error.kind();
  out["message"] = error.what();
  out["path"] = error.path().empty() ? Json(nullptr) : Json(error.path());
  return out;
}

StageError error_from_json(const Json &error) {
  const Json &path = member(error, "path");
  return StageError(error["stage"].get<std::string>(),
                    error["kind"].get<std::string>(),
                    error["message"].get<std::string>(),
                    path.is_string() ? path.get<std::string>() : "");
}

} // namespace pde2d::app
