#pragma once

#include "options.hpp"

#include "pde2d/mesh/components.hpp"
#include "pde2d/mesh/mesh_spec.hpp"
#include "pde2d/mesh/mesher.hpp"
#include "pde2d/mesh/remesher.hpp"

#include <functional>
#include <memory>
#include <string>
#include <vector>

// The catalog: every library component the app offers, with its options
// and a function that builds it. It is the one place that knows component
// names. The page builds its forms from the catalog's JSON, so a new
// component needs an entry here and nothing in the page.
//
// To add a component: write one entry in catalog.cpp. Its `type` must be
// the component's name(), and every default must be read from the
// library's options struct (tests/catalog.cpp checks both).
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
// throw for a value they refuse (with the path in front).
MeshSetup build_mesh_setup(const Json &mesh_section);

// The mesh section a new case starts with.
Json default_mesh_section();

// The mesh section as the page receives it: its settings (the MeshSpec)
// and its slots (discretizer, triangulator, refiner, improvers), each
// with the components that fit it. See docs/app.md, "The catalog".
Json mesh_catalog_json();

} // namespace pde2d::app
