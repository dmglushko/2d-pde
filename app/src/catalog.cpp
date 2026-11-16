#include "catalog.hpp"

#include "app_error.hpp"

#include "pde2d/mesh/cdt.hpp"
#include "pde2d/mesh/chord_length_discretizer.hpp"
#include "pde2d/mesh/delaunay_flips.hpp"
#include "pde2d/mesh/ear_clipping.hpp"
#include "pde2d/mesh/laplacian_smoothing.hpp"
#include "pde2d/mesh/longest_edge_bisection.hpp"
#include "pde2d/mesh/ruppert.hpp"
#include "pde2d/solve/bicgstab.hpp"
#include "pde2d/solve/conjugate_gradient.hpp"
#include "pde2d/solve/fixed_step.hpp"
#include "pde2d/solve/implicit_euler.hpp"
#include "pde2d/solve/implicit_midpoint.hpp"
#include "pde2d/solve/newmark.hpp"
#include "pde2d/solve/simplicial_ldlt.hpp"
#include "pde2d/solve/sparse_lu.hpp"

#include <array>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace pde2d::app {

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

// --- building ---------------------------------------------------------------

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

Json slot_json(std::string_view id, std::string_view label,
               std::string_view description, std::string_view cardinality,
               Json components) {
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

} // namespace

const MeshCatalog &mesh_catalog() {
  static const MeshCatalog catalog = make_mesh_catalog();
  return catalog;
}

MeshSetup build_mesh_setup(const Json &mesh_section) {
  const MeshCatalog &catalog = mesh_catalog();
  require_object(mesh_section, "mesh");
  static constexpr std::string_view known[] = {
      "spec", "discretizer", "triangulator", "refiner", "improvers"};
  reject_unknown_keys(mesh_section, "mesh", known);

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

// --- the solve catalog
// --------------------------------------------------------

namespace {

OptionSpec plain_option(std::string name, std::string label,
                        std::string description, OptionKind kind,
                        Json default_value, std::string unit = {}) {
  return {.name = std::move(name),
          .label = std::move(label),
          .description = std::move(description),
          .kind = kind,
          .default_value = std::move(default_value),
          .choices = {},
          .unit = std::move(unit)};
}

// The options of an iterative solver (tolerance and max_iterations).
std::vector<OptionSpec> iterative_options(double tolerance,
                                          std::size_t max_iterations) {
  return {plain_option("tolerance", "Tolerance",
                       "The solve stops when the relative residual "
                       "|b - A x| / |b| is at most this; in (0, 1).",
                       OptionKind::number, tolerance),
          plain_option("max_iterations", "Maximum iterations",
                       "The solve fails when it has not converged after this "
                       "many iterations; at least 1.",
                       OptionKind::integer, max_iterations)};
}

SolveCatalog make_solve_catalog() {
  SolveCatalog catalog;
  catalog.static_settings = {
      plain_option("t", "Time t",
                   "The time the coefficients and boundary data are taken "
                   "at. The time terms of the problem (u_t, u_tt) are "
                   "dropped: A U = b.",
                   OptionKind::number, 0.0)};
  catalog.time_settings = {
      plain_option("t0", "Start time t0", "The initial conditions hold at t0.",
                   OptionKind::number, 0.0),
      plain_option("t1", "End time t1", "Integrates from t0 to t1 > t0.",
                   OptionKind::number, 1.0),
      plain_option("snapshots", "Snapshots",
                   "How many equal intervals of [t0, t1] to keep the "
                   "solution at: the solution is kept at t0, t1 and the "
                   "times between (snapshots + 1 in all, at most 1000). "
                   "Steps are shortened to land on them.",
                   OptionKind::integer, 50)};

  catalog.linear_solvers.push_back(
      {.type = "sparse_lu",
       .label = "Sparse LU (direct)",
       .description =
           "An LU factorization with a fill-reducing ordering (Eigen's "
           "SparseLU). Works for any square matrix that is not singular; no "
           "options.",
       .options = {},
       .make = [](const OptionReader &)
           -> std::shared_ptr<const solve::LinearSolver> {
         return std::make_shared<solve::SparseLu>();
       }});
  {
    const solve::SimplicialLdltOptions defaults{};
    catalog.linear_solvers.push_back(
        {.type = "simplicial_ldlt",
         .label = "Simplicial LDLT (direct, symmetric)",
         .description =
             "An LDL^T factorization without pivoting (Eigen's "
             "SimplicialLDLT). Needs a symmetric matrix; every result's "
             "residual is checked.",
         .options = {plain_option(
             "residual_tolerance", "Residual tolerance",
             "A solve whose relative residual is above this fails (a tiny "
             "pivot of an indefinite matrix); in (0, 1).",
             OptionKind::number, defaults.residual_tolerance)},
         .make = [](const OptionReader &options)
             -> std::shared_ptr<const solve::LinearSolver> {
           return std::make_shared<solve::SimplicialLdlt>(
               solve::SimplicialLdltOptions{
                   .residual_tolerance = options.number("residual_tolerance")});
         }});
  }
  {
    const solve::ConjugateGradientOptions defaults{};
    catalog.linear_solvers.push_back(
        {.type = "conjugate_gradient",
         .label = "Conjugate gradient (iterative, symmetric)",
         .description =
             "Conjugate gradients with a diagonal preconditioner. Needs a "
             "symmetric matrix and converges only for a positive definite "
             "one; otherwise it fails rather than return a wrong answer.",
         .options =
             iterative_options(defaults.tolerance, defaults.max_iterations),
         .make = [](const OptionReader &options)
             -> std::shared_ptr<const solve::LinearSolver> {
           return std::make_shared<solve::ConjugateGradient>(
               solve::ConjugateGradientOptions{
                   .tolerance = options.number("tolerance"),
                   .max_iterations = options.count("max_iterations")});
         }});
  }
  {
    const solve::BiCgStabOptions defaults{};
    catalog.linear_solvers.push_back(
        {.type = "bicgstab",
         .label = "BiCGSTAB (iterative)",
         .description =
             "BiCGSTAB with a diagonal preconditioner, for any square "
             "matrix (no symmetry needed).",
         .options =
             iterative_options(defaults.tolerance, defaults.max_iterations),
         .make = [](const OptionReader &options)
             -> std::shared_ptr<const solve::LinearSolver> {
           return std::make_shared<solve::BiCgStab>(solve::BiCgStabOptions{
               .tolerance = options.number("tolerance"),
               .max_iterations = options.count("max_iterations")});
         }});
  }

  catalog.time_methods.push_back(
      {.type = "implicit_euler",
       .label = "Implicit Euler",
       .description =
           "Backward Euler, first order, assembled at the end of each step. "
           "Damps; for time order 0 it is a static solve at each step time.",
       .options = {},
       .make = [](const OptionReader &)
           -> std::shared_ptr<const solve::TimeMethod> {
         return std::make_shared<solve::ImplicitEuler>();
       }});
  catalog.time_methods.push_back(
      {.type = "implicit_midpoint",
       .label = "Implicit midpoint",
       .description =
           "Second order, assembled at the middle of each step. Does not "
           "damp: where M2 vanishes, errors in u_t can stay as an "
           "oscillation.",
       .options = {},
       .make = [](const OptionReader &)
           -> std::shared_ptr<const solve::TimeMethod> {
         return std::make_shared<solve::ImplicitMidpoint>();
       }});
  catalog.time_methods.push_back(
      {.type = "newmark",
       .label = "Newmark (average acceleration)",
       .description =
           "Second order for u_tt problems, no numerical damping. Starts "
           "from the acceleration at t0, so M2 must not be singular there.",
       .options = {},
       .make = [](const OptionReader &)
           -> std::shared_ptr<const solve::TimeMethod> {
         return std::make_shared<solve::Newmark>();
       }});

  {
    const solve::FixedStepOptions defaults{};
    catalog.step_controllers.push_back(
        {.type = "fixed_step",
         .label = "Fixed step",
         .description = "The same step dt every time (shortened to land on "
                        "the snapshot times and t1).",
         .options = {plain_option("dt", "Step dt",
                                  "The step length; finite and positive.",
                                  OptionKind::number, defaults.dt)},
         .make = [](const OptionReader &options)
             -> std::shared_ptr<const solve::StepController> {
           return std::make_shared<solve::FixedStep>(
               solve::FixedStepOptions{.dt = options.number("dt")});
         }});
  }
  return catalog;
}

// What a component declares, built with its default options.
template <class Interface, class Traits>
Json components_with_traits(const std::vector<Component<Interface>> &list,
                            Traits traits) {
  Json out = components_json(list);
  for (std::size_t i = 0; i < list.size(); ++i) {
    const OptionReader defaults_reader(Json(nullptr), "", list[i].options);
    out[i]["traits"] = traits(*list[i].make(defaults_reader));
  }
  return out;
}

} // namespace

const SolveCatalog &solve_catalog() {
  static const SolveCatalog catalog = make_solve_catalog();
  return catalog;
}

SolveSetup build_solve_setup(const Json &solve_section) {
  const SolveCatalog &catalog = solve_catalog();
  const Json section =
      solve_section.is_null() ? default_solve_section() : solve_section;
  require_object(section, "solve");
  static constexpr std::string_view known[] = {"mode", "linear_solver",
                                               "static", "time"};
  reject_unknown_keys(section, "solve", known);

  SolveSetup setup;
  const std::string mode =
      require_string(member(section, "mode"), "solve.mode");
  if (mode == "static")
    setup.mode = SolveMode::static_solve;
  else if (mode == "time")
    setup.mode = SolveMode::time_integration;
  else
    throw PathError("solve.mode",
                    "unknown mode '" + mode + "' (static or time)");
  setup.linear_solver =
      build_component(catalog.linear_solvers, member(section, "linear_solver"),
                      "solve.linear_solver");

  for (const std::string_view part : {"static", "time"}) {
    const Json &value = member(section, part);
    if (!value.is_null())
      require_object(value, "solve." + std::string(part));
  }

  if (setup.mode == SolveMode::static_solve) {
    const Json &part = member(section, "static");
    const OptionReader settings(part, "solve.static", catalog.static_settings);
    setup.t = settings.number("t");
    return setup;
  }

  const Json &part = member(section, "time");
  // The settings and the two component slots share the object.
  Json settings_json = part.is_null() ? Json::object() : part;
  const Json method = member(settings_json, "method");
  const Json step = member(settings_json, "step");
  settings_json.erase("method");
  settings_json.erase("step");
  const OptionReader settings(settings_json, "solve.time",
                              catalog.time_settings);
  setup.t0 = settings.number("t0");
  setup.t1 = settings.number("t1");
  setup.snapshots = settings.count("snapshots");
  if (setup.snapshots < 1 || setup.snapshots > max_snapshots)
    throw PathError("solve.time.snapshots",
                    "between 1 and " + std::to_string(max_snapshots));
  setup.method =
      build_component(catalog.time_methods, method, "solve.time.method");
  setup.step_controller =
      build_component(catalog.step_controllers, step, "solve.time.step");
  return setup;
}

Json default_solve_section() {
  const SolveCatalog &catalog = solve_catalog();
  Json time = defaults(catalog.time_settings);
  time["method"] = default_choice(catalog.time_methods.at(0));
  Json step = default_choice(catalog.step_controllers.at(0));
  // FixedStepOptions has no usable default (dt = 0 is refused); a new
  // case starts with 100 steps over the default interval.
  step["options"]["dt"] =
      (time["t1"].get<double>() - time["t0"].get<double>()) / 100.0;
  time["step"] = std::move(step);

  Json out = Json::object();
  out["mode"] = "static";
  out["linear_solver"] = default_choice(catalog.linear_solvers.at(0));
  out["static"] = defaults(catalog.static_settings);
  out["time"] = std::move(time);
  return out;
}

Json solve_component_choice(std::string_view slot, std::string_view type) {
  const SolveCatalog &catalog = solve_catalog();
  const auto find = [&](const auto &components) {
    for (const auto &component : components)
      if (component.type == type)
        return default_choice(component);
    throw std::logic_error("no " + std::string(slot) + " " + std::string(type) +
                           " in the solve catalog");
  };
  if (slot == "linear_solver")
    return find(catalog.linear_solvers);
  if (slot == "method")
    return find(catalog.time_methods);
  if (slot == "step")
    return find(catalog.step_controllers);
  throw std::logic_error("no solve slot " + std::string(slot));
}

Json solve_catalog_json() {
  const SolveCatalog &catalog = solve_catalog();
  const auto settings = [](std::string_view key, std::string_view label,
                           const std::vector<OptionSpec> &options) {
    Json out = Json::object();
    out["key"] = key;
    out["label"] = label;
    out["options"] = to_json(options);
    return out;
  };

  Json linear_solvers = components_with_traits(
      catalog.linear_solvers, [](const solve::LinearSolver &solver) {
        Json traits = Json::object();
        traits["symmetric_matrix"] = solver.requirements().symmetric_matrix;
        return traits;
      });
  Json methods = components_with_traits(
      catalog.time_methods, [](const solve::TimeMethod &method) {
        Json orders = Json::array();
        for (int order = 0; order <= 2; ++order)
          if (method.capabilities().supports(order))
            orders.push_back(order);
        Json traits = Json::object();
        traits["time_orders"] = std::move(orders);
        return traits;
      });

  Json static_mode = Json::object();
  static_mode["id"] = "static";
  static_mode["label"] = "Static";
  static_mode["description"] =
      "Solves A U = b once, at the time t, without the time terms.";
  static_mode["settings"] =
      settings("static", "Static solve", catalog.static_settings);
  static_mode["slots"] = Json::array();

  Json time_slots = Json::array();
  time_slots.push_back(slot_json("method", "Time method",
                                 "How a step from t to t + dt is taken.", "one",
                                 std::move(methods)));
  time_slots.push_back(slot_json("step", "Step control",
                                 "The length of each step.", "one",
                                 components_json(catalog.step_controllers)));
  Json time_mode = Json::object();
  time_mode["id"] = "time";
  time_mode["label"] = "In time";
  time_mode["description"] =
      "Integrates M2 U'' + B U' + A U = b from t0 to t1, starting from the "
      "initial conditions.";
  time_mode["settings"] =
      settings("time", "Time integration", catalog.time_settings);
  time_mode["slots"] = std::move(time_slots);

  Json modes = Json::array();
  modes.push_back(std::move(static_mode));
  modes.push_back(std::move(time_mode));

  Json out = Json::object();
  out["id"] = "solve";
  out["label"] = "Solve";
  out["available"] = true;
  out["linear_solver"] =
      slot_json("linear_solver", "Linear solver",
                "Solves each linear system: the static one, or the step "
                "matrix of every step.",
                "one", std::move(linear_solvers));
  out["modes"] = std::move(modes);
  out["max_snapshots"] = max_snapshots;
  return out;
}

} // namespace pde2d::app
