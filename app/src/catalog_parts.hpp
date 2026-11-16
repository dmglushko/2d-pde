#pragma once

#include "app_error.hpp"
#include "catalog.hpp"

#include <memory>
#include <string>
#include <string_view>
#include <vector>

// What the mesh and solve catalogs share (mesh_catalog.cpp,
// solve_catalog.cpp): finding, building and describing a component.
namespace pde2d::app::detail {

template <class Interface>
const Component<Interface> *
find_component(const std::vector<Component<Interface>> &components,
               const std::string &type) {
  for (const Component<Interface> &component : components)
    if (component.type == type)
      return &component;
  return nullptr;
}

// Builds the component chosen by `choice` ({"type": ..., "options": ...}).
template <class Interface>
std::shared_ptr<const Interface>
build_component(const std::vector<Component<Interface>> &components,
                const Json &choice, const std::string &where) {
  require_object(choice, where);
  static constexpr std::string_view known[] = {"type", "options"};
  reject_unknown_keys(choice, where, known);
  const std::string type =
      require_string(member(choice, "type"), where + ".type");
  const Component<Interface> *component = find_component(components, type);
  if (component == nullptr) {
    std::string list;
    for (const Component<Interface> &candidate : components)
      list += (list.empty() ? "" : ", ") + candidate.type;
    throw PathError(where + ".type", "unknown component '" + type +
                                         "' (available: " + list + ")");
  }
  const std::string options_path = where + ".options";
  const OptionReader options(member(choice, "options"), options_path,
                             component->options);
  try {
    return component->make(options);
  } catch (const PathError &) {
    throw; // the reader's own errors already carry the path
  } catch (const std::invalid_argument &error) {
    const std::string_view message = error.what();
    throw PathError(options_path + named_option(message, component->options),
                    std::string(message));
  }
}

template <class Interface>
Json components_json(const std::vector<Component<Interface>> &components) {
  Json out = Json::array();
  for (const Component<Interface> &component : components) {
    Json item = Json::object();
    item["type"] = component.type;
    item["label"] = component.label;
    item["description"] = component.description;
    item["options"] = to_json(component.options);
    out.push_back(std::move(item));
  }
  return out;
}

inline Json slot_json(std::string_view id, std::string_view label,
                      std::string_view description,
                      std::string_view cardinality, Json components) {
  Json out = Json::object();
  out["id"] = id;
  out["label"] = label;
  out["description"] = description;
  out["cardinality"] = cardinality;
  out["components"] = std::move(components);
  return out;
}

template <class Interface>
Json default_choice(const Component<Interface> &component) {
  Json out = Json::object();
  out["type"] = component.type;
  out["options"] = defaults(component.options);
  return out;
}

// Checks a component choice without building it: an object with "type"
// and "options", a type the catalog has, and options with known names and
// values of the right kinds (OptionReader::check_kinds). A value only the
// component would refuse passes: nothing is built.
template <class Interface>
void check_component(const std::vector<Component<Interface>> &components,
                     const Json &choice, const std::string &where) {
  require_object(choice, where);
  static constexpr std::string_view known[] = {"type", "options"};
  reject_unknown_keys(choice, where, known);
  const std::string type =
      require_string(member(choice, "type"), where + ".type");
  const Component<Interface> *component = find_component(components, type);
  if (component == nullptr) {
    std::string list;
    for (const Component<Interface> &candidate : components)
      list += (list.empty() ? "" : ", ") + candidate.type;
    throw PathError(where + ".type", "unknown component '" + type +
                                         "' (available: " + list + ")");
  }
  OptionReader(member(choice, "options"), where + ".options",
               component->options)
      .check_kinds();
}

} // namespace pde2d::app::detail
