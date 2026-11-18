#pragma once

#include "options.hpp"

#include "pde2d/adapt/adaptive_integrator.hpp"
#include "pde2d/adapt/adaptive_static_solver.hpp"
#include "pde2d/adapt/error_estimator.hpp"
#include "pde2d/adapt/marking.hpp"
#include "pde2d/fem/assembler.hpp"
#include "pde2d/mesh/components.hpp"
#include "pde2d/mesh/mesh_spec.hpp"
#include "pde2d/mesh/mesher.hpp"
#include "pde2d/mesh/remesher.hpp"
#include "pde2d/solve/linear_solver.hpp"
#include "pde2d/solve/step_controller.hpp"
#include "pde2d/solve/time_method.hpp"

#include <functional>
#include <memory>
#include <string_view>
#include <string>
#include <vector>

// The catalog: every library component the app offers, with its options
// and a function that builds it. It is the one place that knows component
// names. The page builds its forms from the catalog's JSON, so a new
// component needs an entry here and nothing in the page.
//
// To add a component: write one entry in catalog.cpp. Its `type` must be
// the component's name(), and every default must be read from the
// library's options struct (tests/session.cpp and tests/solve.cpp check
// both).
namespace pde2d::app {

template <class Interface> struct Component {
  std::string type;        // the key in a case file; equals name()
  std::string label;       // for the form
  std::string description; // for the form
  std::vector<OptionSpec> options;
  std::function<std::shared_ptr<const Interface>(const OptionReader &)> make;
};

// The "mesh" section of a case:
//
//   {"spec": {"max_edge_length": {"absolute": null, "relative": 0.05}},
//    "discretizer": {"type": "chord_length", "options": {...}},
//    "triangulator": {"type": "cdt", "options": {}},
//    "refiner": {"type": "ruppert", "options": {...}} or null,
//    "improvers": [{"type": "delaunay_flips", "options": {...}}, ...]}
struct MeshCatalog {
  std::vector<OptionSpec> spec; // the fields of mesh::MeshSpec
  std::vector<Component<mesh::BoundaryDiscretizer>> discretizers;
  std::vector<Component<mesh::InitialTriangulator>> triangulators;
  std::vector<Component<mesh::Refiner>> refiners;
  std::vector<Component<mesh::Improver>> improvers;
};

const MeshCatalog &mesh_catalog();

// The keys of the mesh section.
inline constexpr std::string_view mesh_section_keys[] = {
    "spec", "discretizer", "triangulator", "refiner", "improvers"};

// The components and the target built from the "mesh" section of a case.
// A geometry is meshed by a Mesher made of all of them; an imported mesh
// by a Remesher made of the refiner and the improvers.
struct MeshSetup {
  std::shared_ptr<const mesh::BoundaryDiscretizer> discretizer;
  std::shared_ptr<const mesh::InitialTriangulator> triangulator;
  std::shared_ptr<const mesh::Refiner> refiner; // may be null
  std::vector<std::shared_ptr<const mesh::Improver>> improvers;
  mesh::MeshSpec spec;

  mesh::Mesher mesher() const;
  mesh::Remesher remesher() const;
};

// Throws std::invalid_argument, naming the path in the case file
// ("mesh.refiner.type: ..."), for a missing or unknown component, a wrong
// value type or an unknown key, and whatever the library's constructors
// throw for a value they refuse (with the path in front: the option's own
// path when the message names one of the component's options, as in
// "mesh.refiner.options.min_angle_degrees: RuppertOptions::...").
MeshSetup build_mesh_setup(const Json &mesh_section);

// The mesh section a new case starts with.
Json default_mesh_section();

// The mesh section as the page receives it: its settings (the MeshSpec)
// and its slots (discretizer, triangulator, refiner, improvers), each
// with the components that fit it. See docs/app.md, "The catalog".
Json mesh_catalog_json();

// The "solve" section of a case:
//
//   {"mode": "static", "time" or "adaptive",
//    "linear_solver": {"type": "sparse_lu", "options": {}},
//    "static": {"t": 0},
//    "time": {"t0": 0, "t1": 1, "snapshots": 50,
//             "method": {"type": "implicit_euler", "options": {}},
//             "step": {"type": "fixed_step", "options": {"dt": 0.01}}},
//    "adaptive": {"t": 0, "tolerance": {"absolute": null, "relative": 0.1},
//                 "max_cycles": 10, "max_nodes": 50000,
//                 "estimator": {"type": "residual_estimator", ...},
//                 "marking": {"type": "dorfler", ...},
//                 "refiner": {"type": "newest_vertex_bisection", ...}}}
//
// The adaptive mode is the library's AdaptiveStaticSolver with one
// MeshRefinement (estimator, marking, a Remesher of the refiner). Its
// refiners are the mesh catalog's.
//
// Only the part of the chosen mode is read into the library's objects; the
// other is kept as written (an object or null), so switching modes back
// and forth keeps both. It is still checked for unknown keys and value
// types, with paths.
struct SolveCatalog {
  std::vector<OptionSpec> static_settings; // t
  std::vector<OptionSpec> time_settings;   // t0, t1, snapshots
  // t, tolerance, max_cycles, max_nodes
  std::vector<OptionSpec> adaptive_settings;
  // every, tolerance, max_nodes, max_refinements, damping_steps
  std::vector<OptionSpec> adaptive_time_settings;
  // threads (fem::AssemblerOptions), for every mode
  std::vector<OptionSpec> assembler_settings;
  std::vector<Component<solve::LinearSolver>> linear_solvers;
  std::vector<Component<solve::TimeMethod>> time_methods;
  std::vector<Component<solve::StepController>> step_controllers;
  std::vector<Component<adapt::ErrorEstimator>> estimators;
  std::vector<Component<adapt::MarkingStrategy>> markings;
};

const SolveCatalog &solve_catalog();

// The keys of the solve section; "time" also holds "method" and "step"
// beside its settings, "adaptive" "estimator", "marking" and "refiner".
// "adaptive_time" "estimator", "marking", "refiner" and "damping_method"
// (the mode "adaptive_time" also reads "time").
// "assembler" (may be missing: its defaults) holds the assembly's options.
inline constexpr std::string_view solve_section_keys[] = {
    "mode", "linear_solver", "assembler",    "static",
    "time", "adaptive",      "adaptive_time"};
inline constexpr std::string_view solve_modes[] = {"static", "time", "adaptive",
                                                   "adaptive_time"};

enum class SolveMode {
  static_solve,
  time_integration,
  adaptive,
  adaptive_time
};

struct SolveSetup {
  SolveMode mode = SolveMode::static_solve;
  std::shared_ptr<const solve::LinearSolver> linear_solver;
  fem::AssemblerOptions assembler{};
  double t = 0.0; // static and adaptive
  // Time integration:
  std::shared_ptr<const solve::TimeMethod> method;
  std::shared_ptr<const solve::StepController> step_controller;
  double t0 = 0.0;
  double t1 = 0.0;
  // Solutions are kept at t0 + k (t1 - t0) / snapshots, k = 0..snapshots.
  std::size_t snapshots = 0;
  // Adaptive (at t), and adaptive in time (with the time integration's):
  std::shared_ptr<const adapt::ErrorEstimator> estimator;
  std::shared_ptr<const adapt::MarkingStrategy> marking;
  std::shared_ptr<const mesh::Refiner> refiner;
  adapt::AdaptiveTarget target;
  // Adaptive in time:
  adapt::AdaptiveTimeTarget time_target;
  adapt::AdaptiveIntegratorOptions time_options;
};

// The most snapshots a run keeps (each one is a value per mesh node sent
// to the page).
inline constexpr std::size_t max_snapshots = 1000;

// Throws std::invalid_argument, naming the path ("solve.time.step.options.
// dt: ..."), like build_mesh_setup. A null section is the default one.
SolveSetup build_solve_setup(const Json &solve_section);

// The solve section a new case starts with.
Json default_solve_section();

// {"type", "options"} with the catalog's defaults, for the component
// `type` of a solve slot: "linear_solver", "method", "step", "estimator",
// "marking" or "refiner". Throws
// std::logic_error for a type the catalog does not have.
Json solve_component_choice(std::string_view slot, std::string_view type);

// The solve section as the page receives it: the linear solver slot, and
// per mode its settings and slots. Linear solvers carry their declared
// requirements and time methods their supported time orders ("traits").
Json solve_catalog_json();

} // namespace pde2d::app
