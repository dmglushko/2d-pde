#include "case_set.hpp"

#include "app_error.hpp"

#include <charconv>
#include <string>
#include <variant>
#include <vector>

namespace pde2d::app {

namespace {

using Step = std::variant<std::string, std::size_t>;

std::vector<Step> parse_path(std::string_view path) {
  std::vector<Step> steps;
  std::size_t at = 0;
  const auto bad = [&](std::string_view why) {
    throw PathError(std::string(path), std::string(why));
  };
  if (path.empty())
    bad("an empty path");
  while (at < path.size()) {
    if (path[at] == '[') {
      const std::size_t end = path.find(']', at);
      if (end == std::string_view::npos)
        bad("a '[' without ']'");
      std::size_t index = 0;
      const auto digits = path.substr(at + 1, end - at - 1);
      const auto [ptr, error] =
          std::from_chars(digits.data(), digits.data() + digits.size(), index);
      if (digits.empty() || error != std::errc() ||
          ptr != digits.data() + digits.size())
        bad("expected a whole number between [ and ]");
      steps.emplace_back(index);
      at = end + 1;
      if (at < path.size() && path[at] == '.')
        ++at;
      continue;
    }
    std::size_t end = path.find_first_of(".[", at);
    if (end == std::string_view::npos)
      end = path.size();
    if (end == at)
      bad("an empty key");
    steps.emplace_back(std::string(path.substr(at, end - at)));
    at = end;
    if (at < path.size() && path[at] == '.') {
      ++at;
      if (at == path.size())
        bad("ends with '.'");
    }
  }
  return steps;
}

std::string spelled(const std::vector<Step> &steps, std::size_t count) {
  std::string out;
  for (std::size_t i = 0; i < count; ++i) {
    if (const auto *key = std::get_if<std::string>(&steps[i]))
      out += (out.empty() ? "" : ".") + *key;
    else
      out += "[" + std::to_string(std::get<std::size_t>(steps[i])) + "]";
  }
  return out.empty() ? "the case" : out;
}

} // namespace

void set_case_value(Json &case_json, std::string_view path,
                    std::string_view value) {
  const std::vector<Step> steps = parse_path(path);
  Json *node = &case_json;
  for (std::size_t i = 0; i < steps.size(); ++i) {
    const bool last = i + 1 == steps.size();
    const std::string parent = spelled(steps, i);
    if (const auto *key = std::get_if<std::string>(&steps[i])) {
      if (!node->is_object())
        throw PathError(std::string(path),
                        parent + " is not an object, so it has no key '" +
                            *key + "'");
      if (!node->contains(*key) && !last)
        throw PathError(std::string(path),
                        "'" + *key + "' does not exist in " + parent);
      node = &(*node)[*key];
    } else {
      const std::size_t index = std::get<std::size_t>(steps[i]);
      if (!node->is_array())
        throw PathError(std::string(path), parent + " is not a list");
      if (index >= node->size())
        throw PathError(std::string(path),
                        parent + " has " + std::to_string(node->size()) +
                            " items, so there is no [" +
                            std::to_string(index) + "]");
      node = &(*node)[index];
    }
  }
  try {
    *node = Json::parse(value);
  } catch (const Json::parse_error &) {
    *node = std::string(value);
  }
}

void apply_set_argument(Json &case_json, std::string_view argument) {
  const std::size_t equals = argument.find('=');
  if (equals == std::string_view::npos || equals == 0)
    throw std::invalid_argument("--set needs PATH=VALUE, got '" +
                                std::string(argument) + "'");
  set_case_value(case_json, argument.substr(0, equals),
                 argument.substr(equals + 1));
}

} // namespace pde2d::app
