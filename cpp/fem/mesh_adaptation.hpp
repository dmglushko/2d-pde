#pragma once
// How an adaptive loop changes the mesh after marking: the MeshAdaptation
// strategy (AdaptOptions::adaptation, adaptivity.hpp). The default,
// MarkedBisection, is discretization::refine_mesh(result, marked, options):
// marked longest-edge bisection. A strategy returns a whole MeshResult made
// by the meshing entry points, so its report and run record describe the
// new mesh; apply_mesh_adaptation checks that before the loop uses it.
//
//   class RefineWithNeighbours final : public MeshAdaptation {
//     std::string name() const override { return "with neighbours"; }
//     discretization::MeshResult
//     apply(const discretization::MeshResult &mesh,
//           const std::vector<std::size_t> &marked,
//           const discretization::MeshOptions &options) const override {
//       return discretization::refine_mesh(mesh, grow(mesh.mesh, marked),
//                                          options);
//     }
//   };
//   options.adaptation = std::make_shared<RefineWithNeighbours>();
//
// The library has no coarsening or node movement; a strategy that does
// either plugs in here.
#include "discretization/mesh_fingerprint.hpp"
#include "discretization/mesh_types.hpp"
#include "discretization/mesh_validation.hpp"
#include "discretization/triangulate.hpp"
#include "provenance.hpp"
#include <cstddef>
#include <format>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace pde2d::fem {
class MeshAdaptation {
public:
  virtual ~MeshAdaptation() = default;
  // Name used in records and in the Refinement decision.
  virtual std::string name() const = 0;
  // Stable description (name + parameters); without one the record says
  // the strategy is not replayable.
  virtual std::optional<pde2d::StrategyDescriptor> descriptor() const {
    return std::nullopt;
  }
  // The new mesh of `mesh` for the marked elements (ascending, in range;
  // mark_elements checked them), meshed with `options` (the requested
  // refinement options or a mesh planner's). The result's report says
  // whether it met its size target; its record must describe its mesh.
  virtual discretization::MeshResult
  apply(const discretization::MeshResult &mesh,
        const std::vector<std::size_t> &marked,
        const discretization::MeshOptions &options) const = 0;
};

// Marked longest-edge bisection (discretization::refine_mesh with marked
// elements), the default strategy.
class MarkedBisection final : public MeshAdaptation {
public:
  std::string name() const override { return "marked longest-edge bisection"; }
  std::optional<pde2d::StrategyDescriptor> descriptor() const override {
    return pde2d::StrategyDescriptor{"MarkedBisection"};
  }
  discretization::MeshResult
  apply(const discretization::MeshResult &mesh,
        const std::vector<std::size_t> &marked,
        const discretization::MeshOptions &options) const override {
    return discretization::refine_mesh(mesh, marked, options);
  }
};

// The choice text of a Refinement decision: "<name> (<algorithm>,
// <boundary treatment>)".
inline std::string
mesh_adaptation_choice(const MeshAdaptation &adaptation,
                       const discretization::MeshOptions &o) {
  return std::format(
      "{} ({}, {})", adaptation.name(),
      discretization::interior_algorithm_name(o.algorithm),
      discretization::boundary_treatment_name(o.boundary_treatment));
}

// Run a strategy and check its result: a meshing record that describes the
// returned mesh (operation set, fingerprint of this mesh) and, for a
// strategy other than MarkedBisection, a valid mesh
// (discretization::validate_mesh_geometry). Throws std::invalid_argument
// ("Mesh adaptation <name> returned an invalid mesh: <reason>").
inline discretization::MeshResult
apply_mesh_adaptation(const MeshAdaptation &adaptation,
                      const discretization::MeshResult &mesh,
                      const std::vector<std::size_t> &marked,
                      const discretization::MeshOptions &options) {
  auto result = adaptation.apply(mesh, marked, options);
  const auto fail = [&](const std::string &reason) {
    throw std::invalid_argument("Mesh adaptation " + adaptation.name() +
                                " returned an invalid mesh: " + reason);
  };
  if (result.record.operation == discretization::MeshOperation::None)
    fail("its record names no meshing operation");
  if (result.record.fingerprint !=
      discretization::mesh_fingerprint(result.mesh))
    fail("its record describes another mesh");
  if (dynamic_cast<const MarkedBisection *>(&adaptation) == nullptr) {
    try {
      discretization::validate_mesh_geometry(result.mesh);
    } catch (const std::invalid_argument &error) {
      fail(error.what());
    }
  }
  return result;
}
} // namespace pde2d::fem
