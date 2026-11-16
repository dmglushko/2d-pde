#include "catalog.hpp"

#include "app_error.hpp"
#include "catalog_parts.hpp"

#include "pde2d/mesh/cdt.hpp"
#include "pde2d/mesh/chord_length_discretizer.hpp"
#include "pde2d/mesh/delaunay_flips.hpp"
#include "pde2d/mesh/ear_clipping.hpp"
#include "pde2d/mesh/laplacian_smoothing.hpp"
#include "pde2d/mesh/longest_edge_bisection.hpp"
#include "pde2d/mesh/ruppert.hpp"

#include <array>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace pde2d::app {

using detail::build_component;
using detail::components_json;
using detail::default_choice;
using detail::slot_json;

namespace {

// --- ImproverUse as a choice -----------------------------------------------

constexpr std::array<std::pair<mesh::ImproverUse, std::string_view>, 3>
    improver_uses{{{mesh::ImproverUse::local_and_final, "local_and_final"},
                   {mesh::ImproverUse::local_only, "local_only"},
                   {mesh::ImproverUse::final_only, "final_only"}}};

std::string use_name(mesh::ImproverUse use) {
  for (const auto &[value, name] : improver_uses)
    if (value == use)
      return std::string(name);
  throw std::logic_error("unknown ImproverUse");
}

mesh::ImproverUse use_from_name(const std::string &name) {
  for (const auto &[value, text] : improver_uses)
    if (text == name)
      return value;
  throw std::logic_error("unknown ImproverUse name");
}

OptionSpec use_option(mesh::ImproverUse default_use) {
  std::vector<std::string> choices;
  for (const auto &[value, name] : improver_uses)
    choices.emplace_back(name);
  return {.name = "use",
          .label = "Use",
          .description =
              "Locally, inside the refiner after each change; as a final "
              "pass over the whole mesh; or both. A use that cannot be "
              "honoured is a warning in the pipeline report.",
          .kind = OptionKind::choice,
          .default_value = use_name(default_use),
          .choices = std::move(choices),
          .unit = {}};
}

// --- the catalog ------------------------------------------------------------

MeshCatalog make_mesh_catalog() {
  MeshCatalog catalog;

  catalog.spec = {
      {.name = "max_edge_length",
       .label = "Maximum edge length",
       .description =
           "Every edge of the final mesh should be at most this long. Give "
           "an absolute length, a fraction of the domain size (the "
           "diagonal of the domain's bounding box), or both; with both, "
           "the smaller one applies. At least one is needed.",
       .kind = OptionKind::length_bound,
       .default_value = to_json(mesh::LengthBound{.relative = 0.05}),
       .choices = {},
       .unit = {}}};

  {
    const mesh::ChordLengthOptions defaults{};
    catalog.discretizers.push_back(
        {.type = "chord_length",
         .label = "Chord length",
         .description =
             "Joins the curves of each loop into chains and splits every "
             "chain into pieces of equal arc length, each at most the "
             "maximum edge length.",
         .options =
             {{.name = "max_curve_deviation",
               .label = "Maximum curve deviation",
               .description =
                   "Upper bound on the distance between a curve and its "
                   "chords. Empty: only the edge length bounds the chords.",
               .kind = OptionKind::length_bound,
               .default_value = to_json(defaults.max_curve_deviation),
               .choices = {},
               .unit = {}},
              {.name = "corner_angle_degrees",
               .label = "Corner angle",
               .description =
                   "Consecutive curves are sampled as one path unless the "
                   "region changes or the boundary turns by more than this "
                   "angle, in [0, 180]. A corner is always a mesh node.",
               .kind = OptionKind::number,
               .default_value = defaults.corner_angle_degrees,
               .choices = {},
               .unit = "degrees"}},
         .make = [](const OptionReader &options)
             -> std::shared_ptr<const mesh::BoundaryDiscretizer> {
           return std::make_shared<mesh::ChordLengthDiscretizer>(
               mesh::ChordLengthOptions{
                   .max_curve_deviation =
                       options.length_bound("max_curve_deviation"),
                   .corner_angle_degrees =
                       options.number("corner_angle_degrees")});
         }});
  }

  catalog.triangulators.push_back(
      {.type = "cdt",
       .label = "Constrained Delaunay",
       .description =
           "Inserts the boundary points one by one, recovers the boundary "
           "edges by flips and removes the triangles outside the domain. "
           "Adds no points.",
       .options = {},
       .make = [](const OptionReader &)
           -> std::shared_ptr<const mesh::InitialTriangulator> {
         return std::make_shared<mesh::Cdt>();
       }});
  catalog.triangulators.push_back(
      {.type = "ear_clipping",
       .label = "Ear clipping",
       .description =
           "Bridges the holes into the outer loop and cuts off ears, "
           "best-shaped ear first. Adds no points; the result is generally "
           "not Delaunay.",
       .options = {},
       .make = [](const OptionReader &)
           -> std::shared_ptr<const mesh::InitialTriangulator> {
         return std::make_shared<mesh::EarClipping>();
       }});

  {
    const mesh::RuppertOptions defaults{};
    catalog.refiners.push_back(
        {.type = "ruppert",
         .label = "Ruppert",
         .description =
             "Delaunay refinement: splits encroached or long boundary "
             "edges and inserts circumcenters of triangles that are too "
             "long or have too small an angle. Keeps the mesh Delaunay.",
         .options = {{.name = "min_angle_degrees",
                      .label = "Minimum angle",
                      .description =
                          "Smallest angle the refined triangles may have, in "
                          "(0, 34]. Termination is proven up to about 20.7 and "
                          "usual in practice up to about 33.",
                      .kind = OptionKind::number,
                      .default_value = defaults.min_angle_degrees,
                      .choices = {},
                      .unit = "degrees"},
                     {.name = "max_points",
                      .label = "Maximum points",
                      .description =
                          "Refinement stops with an error after inserting this "
                          "many points.",
                      .kind = OptionKind::integer,
                      .default_value = defaults.max_points,
                      .choices = {},
                      .unit = {}}},
         .make = [](const OptionReader &options)
             -> std::shared_ptr<const mesh::Refiner> {
           return std::make_shared<mesh::Ruppert>(mesh::RuppertOptions{
               .min_angle_degrees = options.number("min_angle_degrees"),
               .max_points = options.count("max_points")});
         }});
  }
  {
    const mesh::LongestEdgeBisectionOptions defaults{};
    catalog.refiners.push_back(
        {.type = "longest_edge_bisection",
         .label = "Longest-edge bisection",
         .description =
             "Rivara's bisection: while a triangle has an edge that is too "
             "long, splits the longest edge its propagation path ends at. "
             "Accepts local improvers that flip edges or move nodes.",
         .options = {{.name = "max_points",
                      .label = "Maximum points",
                      .description =
                          "Refinement stops with an error after inserting "
                          "this many points.",
                      .kind = OptionKind::integer,
                      .default_value = defaults.max_points,
                      .choices = {},
                      .unit = {}}},
         .make = [](const OptionReader &options)
             -> std::shared_ptr<const mesh::Refiner> {
           return std::make_shared<mesh::LongestEdgeBisection>(
               mesh::LongestEdgeBisectionOptions{
                   .max_points = options.count("max_points")});
         }});
  }

  {
    const mesh::DelaunayFlipsOptions defaults{};
    catalog.improvers.push_back(
        {.type = "delaunay_flips",
         .label = "Delaunay flips",
         .description =
             "Flips edges until every interior edge is locally Delaunay. "
             "Boundary edges are never flipped.",
         .options = {use_option(defaults.use)},
         .make = [](const OptionReader &options)
             -> std::shared_ptr<const mesh::Improver> {
           return std::make_shared<mesh::DelaunayFlips>(
               mesh::DelaunayFlipsOptions{
                   .use = use_from_name(options.choice("use"))});
         }});
  }
  {
    const mesh::LaplacianSmoothingOptions defaults{};
    catalog.improvers.push_back(
        {.type = "laplacian_smoothing",
         .label = "Laplacian smoothing",
         .description =
             "Moves each interior node to the average of its neighbours. A "
             "move that would invert a triangle is skipped; boundary nodes "
             "never move. It has only a final pass.",
         .options = {{.name = "iterations",
                      .label = "Iterations",
                      .description = "Number of passes over all interior "
                                     "nodes, at least 1.",
                      .kind = OptionKind::integer,
                      .default_value = defaults.iterations,
                      .choices = {},
                      .unit = {}},
                     use_option(defaults.use)},
         .make = [](const OptionReader &options)
             -> std::shared_ptr<const mesh::Improver> {
           return std::make_shared<mesh::LaplacianSmoothing>(
               mesh::LaplacianSmoothingOptions{
                   .iterations = options.small_integer("iterations"),
                   .use = use_from_name(options.choice("use"))});
         }});
  }

  return catalog;
}

} // namespace

const MeshCatalog &mesh_catalog() {
  static const MeshCatalog catalog = make_mesh_catalog();
  return catalog;
}

MeshSetup build_mesh_setup(const Json &mesh_section) {
  const MeshCatalog &catalog = mesh_catalog();
  require_object(mesh_section, "mesh");
  reject_unknown_keys(mesh_section, "mesh", mesh_section_keys);

  const OptionReader spec_options(member(mesh_section, "spec"), "mesh.spec",
                                  catalog.spec);
  const mesh::MeshSpec spec{.max_edge_length =
                                spec_options.length_bound("max_edge_length")};
  try {
    mesh::validate(spec);
  } catch (const std::invalid_argument &error) {
    throw PathError("mesh.spec" + named_option(error.what(), catalog.spec),
                    error.what());
  }

  auto discretizer =
      build_component(catalog.discretizers, member(mesh_section, "discretizer"),
                      "mesh.discretizer");
  auto triangulator = build_component(catalog.triangulators,
                                      member(mesh_section, "triangulator"),
                                      "mesh.triangulator");
  std::shared_ptr<const mesh::Refiner> refiner;
  if (const Json &choice = member(mesh_section, "refiner"); !choice.is_null())
    refiner = build_component(catalog.refiners, choice, "mesh.refiner");

  std::vector<std::shared_ptr<const mesh::Improver>> improvers;
  if (const Json &list = member(mesh_section, "improvers"); !list.is_null()) {
    if (!list.is_array())
      throw PathError("mesh.improvers", "expected a list");
    for (std::size_t i = 0; i < list.size(); ++i)
      improvers.push_back(
          build_component(catalog.improvers, list[i],
                          "mesh.improvers[" + std::to_string(i) + "]"));
  }

  return {.discretizer = std::move(discretizer),
          .triangulator = std::move(triangulator),
          .refiner = std::move(refiner),
          .improvers = std::move(improvers),
          .spec = spec};
}

mesh::Mesher MeshSetup::mesher() const {
  return mesh::Mesher(discretizer, triangulator, refiner, improvers);
}

mesh::Remesher MeshSetup::remesher() const {
  return mesh::Remesher(refiner, improvers);
}

Json default_mesh_section() {
  const MeshCatalog &catalog = mesh_catalog();
  Json out = Json::object();
  out["spec"] = defaults(catalog.spec);
  out["discretizer"] = default_choice(catalog.discretizers.at(0));
  out["triangulator"] = default_choice(catalog.triangulators.at(0));
  out["refiner"] = default_choice(catalog.refiners.at(0));
  out["improvers"] = Json::array();
  return out;
}

Json mesh_catalog_json() {
  const MeshCatalog &catalog = mesh_catalog();
  Json settings = Json::object();
  settings["key"] = "spec";
  settings["label"] = "Target";
  settings["options"] = to_json(catalog.spec);

  Json slots = Json::array();
  slots.push_back(slot_json("discretizer", "Boundary discretizer",
                            "Replaces each loop of exact curves by straight "
                            "edges.",
                            "one", components_json(catalog.discretizers)));
  slots.push_back(slot_json("triangulator", "Initial triangulator",
                            "Triangulates the boundary polygon without "
                            "adding points.",
                            "one", components_json(catalog.triangulators)));
  slots.push_back(slot_json("refiner", "Refiner",
                            "Adds nodes until the target is met. Without "
                            "one, the mesh has only the boundary points and "
                            "usually misses its target.",
                            "optional", components_json(catalog.refiners)));
  slots.push_back(slot_json("improvers", "Improvers",
                            "Quality passes, in this order: inside the "
                            "refiner, after it, or both.",
                            "list", components_json(catalog.improvers)));

  Json out = Json::object();
  out["id"] = "mesh";
  out["label"] = "Mesh";
  out["available"] = true;
  out["settings"] = std::move(settings);
  out["slots"] = std::move(slots);
  return out;
}

} // namespace pde2d::app
