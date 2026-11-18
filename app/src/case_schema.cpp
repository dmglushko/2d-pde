#include "case_schema.hpp"

#include "case_format.hpp"
#include "catalog.hpp"
#include "domain_input.hpp"
#include "drawing.hpp"
#include "problem_input.hpp"
#include "stage_cache.hpp"

#include <map>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace pde2d::app {

namespace {

Json types(std::initializer_list<const char *> names) {
  Json out = Json::array();
  for (const char *name : names)
    out.push_back(name);
  return out;
}

Json typed(std::initializer_list<const char *> names) {
  Json out = Json::object();
  out["type"] = types(names);
  return out;
}

// An object with exactly `keys`, each described in `described`; throws
// std::logic_error for a key without a description (a reader's key list
// changed and this generator did not).
Json object_schema(std::span<const std::string_view> keys,
                   const std::map<std::string, Json> &described,
                   std::initializer_list<const char *> required = {},
                   bool nullable = false) {
  Json properties = Json::object();
  for (const std::string_view key : keys) {
    const auto found = described.find(std::string(key));
    if (found == described.end())
      throw std::logic_error("case_schema: no description of the key '" +
                             std::string(key) + "'");
    properties[std::string(key)] = found->second;
  }
  Json out = Json::object();
  out["type"] = nullable ? types({"object", "null"}) : types({"object"});
  out["properties"] = std::move(properties);
  out["additionalProperties"] = false;
  if (required.size() > 0)
    out["required"] = types(required);
  return out;
}

Json option_schema(const OptionSpec &spec) {
  Json out;
  switch (spec.kind) {
  case OptionKind::number:
    out = typed({"number", "null"});
    break;
  case OptionKind::integer:
    out = typed({"integer", "null"});
    break;
  case OptionKind::choice: {
    out = Json::object();
    Json values = Json::array();
    for (const std::string &choice : spec.choices)
      values.push_back(choice);
    values.push_back(nullptr);
    out["enum"] = std::move(values);
    break;
  }
  case OptionKind::length_bound:
  case OptionKind::tolerance: {
    static constexpr std::string_view keys[] = {"absolute", "relative"};
    out = object_schema(keys,
                        {{"absolute", typed({"number", "null"})},
                         {"relative", typed({"number", "null"})}},
                        {}, true);
    break;
  }
  }
  out["title"] = spec.label;
  out["description"] = spec.description;
  out["default"] = spec.default_value;
  // Read back by tests and tools: which catalog kind this option is.
  out["x-pde2d-kind"] = option_kind_name(spec.kind);
  return out;
}

// The options object of a component or of settings.
Json options_schema(std::span<const OptionSpec> specs) {
  Json properties = Json::object();
  for (const OptionSpec &spec : specs)
    properties[spec.name] = option_schema(spec);
  Json out = typed({"object", "null"});
  out["properties"] = std::move(properties);
  out["additionalProperties"] = false;
  return out;
}

// {"type": one of the components, "options": its options}.
template <class Interface>
Json choice_schema(const std::vector<Component<Interface>> &components,
                   bool nullable = false) {
  Json alternatives = Json::array();
  static constexpr std::string_view keys[] = {"type", "options"};
  for (const Component<Interface> &component : components) {
    Json type = Json::object();
    type["const"] = component.type;
    Json item = object_schema(
        keys, {{"type", type}, {"options", options_schema(component.options)}},
        {"type"});
    item["title"] = component.label;
    item["description"] = component.description;
    alternatives.push_back(std::move(item));
  }
  if (nullable)
    alternatives.push_back(typed({"null"}));
  Json out = Json::object();
  out["oneOf"] = std::move(alternatives);
  return out;
}

Json constant(std::string_view value) {
  Json out = Json::object();
  out["const"] = value;
  return out;
}

// For curves (shapes, SVG) the edits split at a fraction; for a mesh file
// at a node.
Json region_edits_schema(bool with_split) {
  const Json text = typed({"string"});
  Json alternatives = Json::array();
  alternatives.push_back(
      object_schema(rename_edit_keys,
                    {{"op", constant("rename")}, {"from", text}, {"to", text}},
                    {"op", "from", "to"}));
  alternatives.push_back(object_schema(
      merge_edit_keys,
      {{"op", constant("merge")}, {"source", text}, {"target", text}},
      {"op", "source", "target"}));
  if (with_split)
    alternatives.push_back(
        object_schema(split_edit_keys,
                      {{"op", constant("split")},
                       {"region", text},
                       {"fraction", typed({"number"})},
                       {"first", text},
                       {"second", text}},
                      {"op", "region", "fraction", "first", "second"}));
  else {
    Json node = Json::object();
    node["type"] = "integer";
    node["minimum"] = 0;
    alternatives.push_back(
        object_schema(split_at_node_edit_keys,
                      {{"op", constant("split_at_node")},
                       {"region", text},
                       {"node", node},
                       {"first", text},
                       {"second", text}},
                      {"op", "region", "node", "first", "second"}));
  }
  Json items = Json::object();
  items["oneOf"] = std::move(alternatives);
  Json out = typed({"array", "null"});
  out["items"] = std::move(items);
  return out;
}

Json point_schema() {
  Json out = Json::object();
  out["type"] = "array";
  out["items"] = typed({"number"});
  out["minItems"] = 2;
  out["maxItems"] = 2;
  return out;
}

// The drawing source (drawing.hpp): loops of vertices and edges, each edge
// one of the kinds with its own keys.
Json drawing_schema() {
  const Json text = typed({"string"});
  const Json number = typed({"number"});
  const Json point = point_schema();
  Json edges = Json::array();
  for (const std::string_view kind : drawing_edge_kinds) {
    if (kind == "line")
      edges.push_back(object_schema(
          line_edge_keys, {{"kind", constant(kind)}, {"region", text}},
          {"kind", "region"}));
    else if (kind == "quadratic")
      edges.push_back(object_schema(
          quadratic_edge_keys,
          {{"kind", constant(kind)}, {"region", text}, {"control", point}},
          {"kind", "region", "control"}));
    else if (kind == "cubic")
      edges.push_back(
          object_schema(cubic_edge_keys,
                        {{"kind", constant(kind)},
                         {"region", text},
                         {"control1", point},
                         {"control2", point}},
                        {"kind", "region", "control1", "control2"}));
    else if (kind == "arc")
      edges.push_back(
          object_schema(arc_edge_keys,
                        {{"kind", constant(kind)},
                         {"region", text},
                         {"center", point},
                         {"radius_x", number},
                         {"radius_y", number},
                         {"rotation", typed({"number", "null"})},
                         {"start_angle", number},
                         {"sweep_angle", number}},
                        {"kind", "region", "center", "radius_x", "radius_y",
                         "start_angle", "sweep_angle"}));
    else
      throw std::logic_error("case_schema: the edge kind '" +
                             std::string(kind) + "' is not described");
  }
  Json edge = Json::object();
  edge["oneOf"] = std::move(edges);
  Json vertex_list = Json::object();
  vertex_list["type"] = "array";
  vertex_list["items"] = point;
  vertex_list["minItems"] = 1;
  Json edge_list = Json::object();
  edge_list["type"] = "array";
  edge_list["items"] = std::move(edge);
  edge_list["minItems"] = 1;
  Json loop = object_schema(
      drawing_loop_keys,
      {{"vertices", std::move(vertex_list)}, {"edges", std::move(edge_list)}},
      {"vertices", "edges"});
  Json loops = Json::object();
  loops["type"] = "array";
  loops["items"] = std::move(loop);
  loops["minItems"] = 1;
  Json out = object_schema(
      drawing_section_keys,
      {{"source", constant("drawing")}, {"loops", std::move(loops)}},
      {"source", "loops"});
  out["title"] = "Drawing";
  return out;
}

Json geometry_schema() {
  const Json text = typed({"string"});
  const Json name = typed({"string", "null"});
  Json alternatives = Json::array();
  for (const Json &shape : shape_catalog_json()) {
    Json options = Json::object();
    options["type"] = types({"object", "null"});
    Json properties = Json::object();
    for (const Json &option : shape["options"]) {
      // The shape catalog gives each option as the page sees it.
      Json item = Json::object();
      const std::string kind = option["kind"].get<std::string>();
      item["type"] = kind == "integer" ? types({"integer", "null"})
                                       : types({"number", "null"});
      item["title"] = option["label"];
      item["default"] = option["default"];
      item["x-pde2d-kind"] = kind;
      properties[option["name"].get<std::string>()] = std::move(item);
    }
    options["properties"] = std::move(properties);
    options["additionalProperties"] = false;
    Json item =
        object_schema(shape_section_keys,
                      {{"source", constant("shape")},
                       {"shape", constant(shape["type"].get<std::string>())},
                       {"options", std::move(options)},
                       {"region_edits", region_edits_schema(true)}},
                      {"source", "shape"});
    item["title"] = shape["label"];
    alternatives.push_back(std::move(item));
  }
  alternatives.push_back(
      object_schema(svg_section_keys,
                    {{"source", constant("svg")},
                     {"name", name},
                     {"svg", text},
                     {"region_edits", region_edits_schema(true)}},
                    {"source", "svg"}));
  Json formats = Json::object();
  formats["enum"] = Json::array();
  for (const std::string_view format : mesh_file_formats)
    formats["enum"].push_back(format);
  alternatives.push_back(
      object_schema(mesh_file_section_keys,
                    {{"source", constant("mesh_file")},
                     {"format", formats},
                     {"name", name},
                     {"text", text},
                     {"region_edits", region_edits_schema(false)}},
                    {"source", "format", "text"}));
  alternatives.push_back(drawing_schema());
  if (alternatives.size() != shape_catalog_json().size() + 3 ||
      std::size(geometry_sources) != 4)
    throw std::logic_error("case_schema: a geometry source is not described");
  Json out = Json::object();
  out["oneOf"] = std::move(alternatives);
  return out;
}

Json mesh_schema() {
  const MeshCatalog &catalog = mesh_catalog();
  Json improvers = typed({"array", "null"});
  improvers["items"] = choice_schema(catalog.improvers);
  return object_schema(mesh_section_keys,
                       {{"spec", options_schema(catalog.spec)},
                        {"discretizer", choice_schema(catalog.discretizers)},
                        {"triangulator", choice_schema(catalog.triangulators)},
                        {"refiner", choice_schema(catalog.refiners, true)},
                        {"improvers", std::move(improvers)}},
                       {"discretizer", "triangulator"});
}

// An object of coefficients (expression texts; a number is read as its
// text) with the names of the problem catalog's `fields`.
Json coefficients_schema(const Json &fields,
                         std::span<const std::string_view> extra = {},
                         const Json &extra_schema = nullptr) {
  std::vector<std::string_view> keys;
  std::map<std::string, Json> described;
  for (const Json &field : fields) {
    const std::string &name = field["name"].get_ref<const std::string &>();
    Json item = typed({"string", "number", "null"});
    item["description"] = "multiplies " + field["term"].get<std::string>();
    described.emplace(name, std::move(item));
  }
  for (const Json &field : fields)
    keys.push_back(field["name"].get_ref<const std::string &>());
  for (const std::string_view key : extra) {
    keys.push_back(key);
    described.emplace(std::string(key), extra_schema);
  }
  return object_schema(keys, described, {}, true);
}

// The coefficients, or the line instead (line_key).
Json with_line(Json schema, std::string_view what) {
  Json line = typed({"string", "null"});
  line["description"] = "the " + std::string(what) +
                        " as one line, given instead of the coefficients "
                        "(fem::parse_equation, fem::parse_condition)";
  schema["properties"][std::string(line_key)] = std::move(line);
  return schema;
}

Json problem_schema() {
  const Json catalog = problem_catalog_json();
  Json parameters = typed({"object", "null"});
  parameters["additionalProperties"] = typed({"number"});
  Json conditions = typed({"object", "null"});
  conditions["additionalProperties"] = with_line(
      coefficients_schema(catalog["condition_fields"], condition_extra_keys,
                          typed({"integer", "null"})),
      "condition");
  return object_schema(
      problem_section_keys,
      {{"equation",
        with_line(coefficients_schema(catalog["equation_fields"]), "equation")},
       {"parameters", std::move(parameters)},
       {"boundary_conditions", std::move(conditions)},
       {"initial", coefficients_schema(catalog["initial_fields"])}},
      {}, true);
}

Json solve_schema() {
  const SolveCatalog &catalog = solve_catalog();
  Json modes = Json::object();
  modes["enum"] = Json::array();
  for (const std::string_view mode : solve_modes)
    modes["enum"].push_back(mode);
  // The time part: its settings and its two slots in one object.
  Json time = options_schema(catalog.time_settings);
  time["properties"]["method"] = choice_schema(catalog.time_methods);
  time["properties"]["step"] = choice_schema(catalog.step_controllers);
  // The adaptive part: its settings and three slots (each may be missing:
  // the catalog's default).
  Json adaptive = options_schema(catalog.adaptive_settings);
  adaptive["properties"]["estimator"] = choice_schema(catalog.estimators, true);
  adaptive["properties"]["marking"] = choice_schema(catalog.markings, true);
  adaptive["properties"]["refiner"] =
      choice_schema(mesh_catalog().refiners, true);
  Json out =
      object_schema(solve_section_keys,
                    {{"mode", std::move(modes)},
                     {"linear_solver", choice_schema(catalog.linear_solvers)},
                     {"assembler", options_schema(catalog.assembler_settings)},
                     {"static", options_schema(catalog.static_settings)},
                     {"time", std::move(time)},
                     {"adaptive", std::move(adaptive)}},
                    {"mode", "linear_solver"}, true);
  // In time, the time part needs its method and step.
  Json condition = Json::object();
  condition["properties"]["mode"] = constant("time");
  condition["required"] = types({"mode"});
  Json then = Json::object();
  then["required"] = types({"time"});
  then["properties"]["time"]["type"] = "object";
  then["properties"]["time"]["required"] = types({"method", "step"});
  Json rule = Json::object();
  rule["if"] = std::move(condition);
  rule["then"] = std::move(then);
  out["allOf"] = Json::array({std::move(rule)});
  return out;
}

} // namespace

Json case_schema() {
  Json version = typed({"integer", "null"});
  version["minimum"] = 1;
  version["maximum"] = case_version;
  Json format = Json::object();
  format["enum"] = Json::array({case_format, nullptr});
  Json assembly =
      object_schema(assembly_section_keys,
                    {{"t", typed({"number", "null"})},
                     {"max_time_order", typed({"integer", "null"})}},
                    {}, true);
  Json view = Json::object();
  view["description"] = "Free for the page and for people.";
  Json notes = Json::object();
  notes["description"] = "Free text.";
  Json out = object_schema(case_keys,
                           {{"format", std::move(format)},
                            {"version", std::move(version)},
                            {"geometry", geometry_schema()},
                            {"mesh", mesh_schema()},
                            {"problem", problem_schema()},
                            {"assembly", std::move(assembly)},
                            {"solve", solve_schema()},
                            {"view", std::move(view)},
                            {"notes", std::move(notes)}},
                           {"geometry", "mesh"});
  Json schema = Json::object();
  schema["$schema"] = "https://json-schema.org/draft/2020-12/schema";
  schema["title"] = "pde2d app case";
  schema["description"] =
      "A case of pde2d_app, version " + std::to_string(case_version) +
      ". Generated from the app's catalog and readers; ranges are checked "
      "by the library, not here.";
  for (const auto &[key, value] : out.items())
    schema[key] = value;
  return schema;
}

} // namespace pde2d::app
