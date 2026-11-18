#include "catalog.hpp"

#include "app_error.hpp"
#include "catalog_parts.hpp"

#include "pde2d/adapt/dorfler_marking.hpp"
#include "pde2d/adapt/equidistribution_marking.hpp"
#include "pde2d/adapt/fixed_fraction_marking.hpp"
#include "pde2d/adapt/kelly_estimator.hpp"
#include "pde2d/adapt/maximum_marking.hpp"
#include "pde2d/adapt/recovery_estimator.hpp"
#include "pde2d/adapt/residual_estimator.hpp"
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

using detail::build_component;
using detail::check_component;
using detail::components_json;
using detail::default_choice;
using detail::slot_json;

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
             "backward error is checked.",
         .options = {plain_option(
             "residual_tolerance", "Residual tolerance",
             "A solve whose backward error |b - A x| / (|A| |x| + |b|) is "
             "above this fails (a tiny pivot of an indefinite matrix); in "
             "(0, 1).",
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

  catalog.adaptive_settings = {
      plain_option("t", "Time t",
                   "The time the coefficients and boundary data are taken "
                   "at; each cycle is a static solve (A U = b) at t.",
                   OptionKind::number, 0.0),
      plain_option("tolerance", "Tolerance",
                   "Stops when the estimated error is at most the smaller of "
                   "the absolute value and the relative one times the first "
                   "cycle's estimate. Empty: no tolerance (the budgets stop "
                   "the loop).",
                   OptionKind::tolerance,
                   Json{{"absolute", nullptr}, {"relative", 0.1}}),
      plain_option("max_cycles", "Maximum cycles",
                   "Stops after solving this many meshes (the first "
                   "included); at least 1.",
                   OptionKind::integer, 10),
      plain_option("max_nodes", "Maximum nodes",
                   "Stops before solving a refined mesh with more nodes "
                   "than this; at least the starting mesh's nodes.",
                   OptionKind::integer, 50000)};

  const auto degree_option = [](std::string name, std::string label,
                                std::string what, int value) {
    return plain_option(std::move(name), std::move(label),
                        "The exact degree of the quadrature rule for " +
                            std::move(what) +
                            ". The default is exact for coefficients and "
                            "data of degree at most 1.",
                        OptionKind::integer, value);
  };
  {
    const adapt::ResidualEstimatorOptions defaults{};
    catalog.estimators.push_back(
        {.type = "residual_estimator",
         .label = "Residual",
         .description =
             "The explicit residual estimator: h_T² ‖f − b·∇u − c u‖² on "
             "each triangle, plus the jumps of the flux A∇u across interior "
             "edges and the residual of natural conditions on boundary "
             "edges. A reliable upper bound up to a constant.",
         .options = {degree_option("triangle_degree", "Triangle degree",
                                   "the element residual",
                                   defaults.triangle_degree),
                     degree_option("edge_degree", "Edge degree",
                                   "the jumps and the boundary residual",
                                   defaults.edge_degree)},
         .make = [](const OptionReader &options)
             -> std::shared_ptr<const adapt::ErrorEstimator> {
           return std::make_shared<adapt::ResidualEstimator>(
               adapt::ResidualEstimatorOptions{
                   .triangle_degree = options.small_integer("triangle_degree"),
                   .edge_degree = options.small_integer("edge_degree")});
         }});
  }
  {
    const adapt::KellyEstimatorOptions defaults{};
    catalog.estimators.push_back(
        {.type = "kelly_estimator",
         .label = "Kelly (flux jumps)",
         .description =
             "Only the jumps of the flux A∇u across interior edges and the "
             "residual of natural conditions: no element residual, so it "
             "misses errors from the load inside triangles.",
         .options = {degree_option("edge_degree", "Edge degree",
                                   "the jumps and the boundary residual",
                                   defaults.edge_degree)},
         .make = [](const OptionReader &options)
             -> std::shared_ptr<const adapt::ErrorEstimator> {
           return std::make_shared<adapt::KellyEstimator>(
               adapt::KellyEstimatorOptions{
                   .edge_degree = options.small_integer("edge_degree")});
         }});
  }
  {
    const adapt::RecoveryEstimatorOptions defaults{};
    catalog.estimators.push_back(
        {.type = "recovery_estimator",
         .label = "Gradient recovery (ZZ)",
         .description =
             "Zienkiewicz–Zhu: the difference between ∇u and its recovered "
             "gradient (the area-weighted average at the nodes, linear "
             "between them). Close to the true energy error for smooth "
             "solutions.",
         .options = {{.name = "norm",
                      .label = "Norm",
                      .description =
                          "energy: weighted by |A| (needs A semidefinite); "
                          "gradient: the plain gradient difference.",
                      .kind = OptionKind::choice,
                      .default_value =
                          defaults.norm == adapt::RecoveryNorm::energy
                              ? "energy"
                              : "gradient",
                      .choices = {"energy", "gradient"},
                      .unit = {}},
                     degree_option("triangle_degree", "Triangle degree",
                                   "the gradient difference",
                                   defaults.triangle_degree)},
         .make = [](const OptionReader &options)
             -> std::shared_ptr<const adapt::ErrorEstimator> {
           return std::make_shared<adapt::RecoveryEstimator>(
               adapt::RecoveryEstimatorOptions{
                   .norm = options.choice("norm") == "energy"
                               ? adapt::RecoveryNorm::energy
                               : adapt::RecoveryNorm::gradient,
                   .triangle_degree =
                       options.small_integer("triangle_degree")});
         }});
  }

  {
    const adapt::DorflerMarkingOptions defaults{};
    catalog.markings.push_back(
        {.type = "dorfler",
         .label = "Dörfler (bulk)",
         .description =
             "Marks the fewest triangles, largest indicators first, whose "
             "squared indicators make up the share θ of the total.",
         .options = {plain_option("theta", "θ",
                                  "The share of Σ η² to mark; in [0, 1].",
                                  OptionKind::number, defaults.theta)},
         .make = [](const OptionReader &options)
             -> std::shared_ptr<const adapt::MarkingStrategy> {
           return std::make_shared<adapt::DorflerMarking>(
               adapt::DorflerMarkingOptions{.theta = options.number("theta")});
         }});
  }
  {
    const adapt::MaximumMarkingOptions defaults{};
    catalog.markings.push_back(
        {.type = "maximum",
         .label = "Maximum",
         .description = "Marks every triangle whose indicator is at least θ "
                        "times the largest.",
         .options = {plain_option("theta", "θ",
                                  "The fraction of the largest indicator; in "
                                  "[0, 1].",
                                  OptionKind::number, defaults.theta)},
         .make = [](const OptionReader &options)
             -> std::shared_ptr<const adapt::MarkingStrategy> {
           return std::make_shared<adapt::MaximumMarking>(
               adapt::MaximumMarkingOptions{.theta = options.number("theta")});
         }});
  }
  {
    const adapt::FixedFractionMarkingOptions defaults{};
    catalog.markings.push_back(
        {.type = "fixed_fraction",
         .label = "Fixed fraction",
         .description = "Marks the given fraction of the triangles, those "
                        "with the largest indicators.",
         .options = {plain_option("fraction", "Fraction",
                                  "The share of the triangles to mark; in "
                                  "[0, 1].",
                                  OptionKind::number, defaults.fraction)},
         .make = [](const OptionReader &options)
             -> std::shared_ptr<const adapt::MarkingStrategy> {
           return std::make_shared<adapt::FixedFractionMarking>(
               adapt::FixedFractionMarkingOptions{
                   .fraction = options.number("fraction")});
         }});
  }
  {
    const adapt::EquidistributionMarkingOptions defaults{.tolerance = 0.0};
    catalog.markings.push_back(
        {.type = "equidistribution",
         .label = "Equidistribution",
         .description = "Marks every triangle whose indicator is at least "
                        "tol / √N (N triangles): aims at an equal error per "
                        "triangle with total tol.",
         .options = {plain_option("tolerance", "Tolerance",
                                  "The total error aimed at; required, "
                                  "positive.",
                                  OptionKind::number, defaults.tolerance)},
         .make = [](const OptionReader &options)
             -> std::shared_ptr<const adapt::MarkingStrategy> {
           return std::make_shared<adapt::EquidistributionMarking>(
               adapt::EquidistributionMarkingOptions{
                   .tolerance = options.number("tolerance")});
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

// The "time" part of the solve section split into its settings and its two
// component slots (they share the object).
struct TimePart {
  Json settings;
  Json method;
  Json step;
};
TimePart split_time_part(const Json &part) {
  TimePart out{.settings = part.is_null() ? Json::object() : part,
               .method = nullptr,
               .step = nullptr};
  out.method = member(out.settings, "method");
  out.step = member(out.settings, "step");
  out.settings.erase("method");
  out.settings.erase("step");
  return out;
}

// The "adaptive" part: its settings and its three component slots.
struct AdaptivePart {
  Json settings;
  Json estimator;
  Json marking;
  Json refiner;
};
AdaptivePart split_adaptive_part(const Json &part) {
  AdaptivePart out{.settings = part.is_null() ? Json::object() : part,
                   .estimator = nullptr,
                   .marking = nullptr,
                   .refiner = nullptr};
  out.estimator = member(out.settings, "estimator");
  out.marking = member(out.settings, "marking");
  out.refiner = member(out.settings, "refiner");
  for (const char *key : {"estimator", "marking", "refiner"})
    out.settings.erase(key);
  return out;
}

// A slot of the adaptive part: a missing choice is the catalog's first
// component with its defaults.
template <class Interface>
Json choice_or_default(const Json &choice,
                       const std::vector<Component<Interface>> &components,
                       std::size_t index = 0) {
  return choice.is_null() ? default_choice(components.at(index)) : choice;
}

// The mesh catalog's newest vertex bisection: the default refiner of the
// adaptive mode.
std::size_t default_adaptive_refiner() {
  const auto &refiners = mesh_catalog().refiners;
  for (std::size_t i = 0; i < refiners.size(); ++i)
    if (refiners[i].type == "newest_vertex_bisection")
      return i;
  throw std::logic_error("no newest_vertex_bisection in the mesh catalog");
}

// Checks the adaptive part without building it (another mode is chosen).
void check_adaptive_part(const AdaptivePart &part) {
  const SolveCatalog &catalog = solve_catalog();
  OptionReader(part.settings, "solve.adaptive", catalog.adaptive_settings)
      .check_kinds();
  if (!part.estimator.is_null())
    check_component(catalog.estimators, part.estimator,
                    "solve.adaptive.estimator");
  if (!part.marking.is_null())
    check_component(catalog.markings, part.marking, "solve.adaptive.marking");
  if (!part.refiner.is_null())
    check_component(mesh_catalog().refiners, part.refiner,
                    "solve.adaptive.refiner");
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
  reject_unknown_keys(section, "solve", solve_section_keys);

  SolveSetup setup;
  const std::string mode =
      require_string(member(section, "mode"), "solve.mode");
  if (mode == "static")
    setup.mode = SolveMode::static_solve;
  else if (mode == "time")
    setup.mode = SolveMode::time_integration;
  else if (mode == "adaptive")
    setup.mode = SolveMode::adaptive;
  else
    throw PathError("solve.mode",
                    "unknown mode '" + mode + "' (static, time or adaptive)");
  setup.linear_solver =
      build_component(catalog.linear_solvers, member(section, "linear_solver"),
                      "solve.linear_solver");

  for (const std::string_view part : {"static", "time", "adaptive"}) {
    const Json &value = member(section, part);
    if (!value.is_null())
      require_object(value, "solve." + std::string(part));
  }

  // The part of the other mode is kept as written, but checked for
  // unknown keys and value types (nothing of it is built, so a value only
  // the library would refuse passes).
  const TimePart time = split_time_part(member(section, "time"));
  const AdaptivePart adaptive =
      split_adaptive_part(member(section, "adaptive"));
  const auto check_time_part = [&] {
    OptionReader(time.settings, "solve.time", catalog.time_settings)
        .check_kinds();
    if (!time.method.is_null())
      check_component(catalog.time_methods, time.method, "solve.time.method");
    if (!time.step.is_null())
      check_component(catalog.step_controllers, time.step, "solve.time.step");
  };
  if (setup.mode == SolveMode::adaptive) {
    OptionReader(member(section, "static"), "solve.static",
                 catalog.static_settings)
        .check_kinds();
    check_time_part();
    const OptionReader settings(adaptive.settings, "solve.adaptive",
                                catalog.adaptive_settings);
    setup.t = settings.number("t");
    setup.target = {.tolerance = settings.tolerance("tolerance"),
                    .max_cycles = settings.count("max_cycles"),
                    .max_nodes = settings.count("max_nodes")};
    setup.estimator = build_component(
        catalog.estimators,
        choice_or_default(adaptive.estimator, catalog.estimators),
        "solve.adaptive.estimator");
    setup.marking = build_component(
        catalog.markings, choice_or_default(adaptive.marking, catalog.markings),
        "solve.adaptive.marking");
    setup.refiner = build_component(
        mesh_catalog().refiners,
        choice_or_default(adaptive.refiner, mesh_catalog().refiners,
                          default_adaptive_refiner()),
        "solve.adaptive.refiner");
    return setup;
  }
  check_adaptive_part(adaptive);
  if (setup.mode == SolveMode::static_solve) {
    const OptionReader settings(member(section, "static"), "solve.static",
                                catalog.static_settings);
    setup.t = settings.number("t");
    check_time_part();
    return setup;
  }
  OptionReader(member(section, "static"), "solve.static",
               catalog.static_settings)
      .check_kinds();

  const Json &method = time.method;
  const Json &step = time.step;
  const OptionReader settings(time.settings, "solve.time",
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
  Json adaptive = defaults(catalog.adaptive_settings);
  adaptive["estimator"] = default_choice(catalog.estimators.at(0));
  adaptive["marking"] = default_choice(catalog.markings.at(0));
  adaptive["refiner"] =
      default_choice(mesh_catalog().refiners.at(default_adaptive_refiner()));
  out["adaptive"] = std::move(adaptive);
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
  if (slot == "estimator")
    return find(catalog.estimators);
  if (slot == "marking")
    return find(catalog.markings);
  if (slot == "refiner")
    return find(mesh_catalog().refiners);
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

  Json adaptive_slots = Json::array();
  adaptive_slots.push_back(
      slot_json("estimator", "Error estimator",
                "Gives each triangle an error indicator η_T from the solution.",
                "one", components_json(catalog.estimators)));
  adaptive_slots.push_back(slot_json("marking", "Marking",
                                     "Chooses the triangles to refine.", "one",
                                     components_json(catalog.markings)));
  adaptive_slots.push_back(slot_json(
      "refiner", "Refiner",
      "Refines the marked triangles (and their neighbours, so the mesh "
      "stays conforming). The same refiners as the Mesh step.",
      "one", components_json(mesh_catalog().refiners)));
  Json adaptive_mode = Json::object();
  adaptive_mode["id"] = "adaptive";
  adaptive_mode["label"] = "Adaptive";
  adaptive_mode["description"] =
      "Solves A U = b at t, estimates the error per triangle, refines the "
      "marked triangles and solves again, until the estimate meets the "
      "tolerance or a budget is reached. The picture shows the last mesh.";
  adaptive_mode["settings"] =
      settings("adaptive", "Adaptive loop", catalog.adaptive_settings);
  adaptive_mode["slots"] = std::move(adaptive_slots);

  Json modes = Json::array();
  modes.push_back(std::move(static_mode));
  modes.push_back(std::move(time_mode));
  modes.push_back(std::move(adaptive_mode));

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
