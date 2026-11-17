#pragma once

#include "pde2d/adapt/error_estimator.hpp"
#include "pde2d/fem/meshed_problem.hpp"
#include "pde2d/mesh/mesh.hpp"
#include "pde2d/mesh/mesh_properties.hpp"
#include "pde2d/mesh/report.hpp"
#include "pde2d/solve/solution.hpp"

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace pde2d::adapt {

// A solved problem: what an adaptation looks at. The solution is on the
// problem's mesh; the derivatives as for ErrorEstimator::estimate (zeros
// for a static solution).
struct SolvedState {
  const fem::MeshedProblem &problem;
  const solve::Solution &solution;
  TimeDerivatives derivatives{};
};

// What an adaptation declares about itself; drivers decide from this,
// never from its type.
struct AdaptationCapabilities {
  // estimate() gives an error estimate (a driver's tolerance can use it).
  bool estimates_error = false;
  // adapt() may return a new mesh.
  bool changes_mesh = false;
};

// What one adaptation did.
struct AdaptationResult {
  // The new mesh, or null when the adaptation changed nothing.
  std::shared_ptr<const mesh::Mesh> mesh{};
  // With a new mesh from refinement: its properties (measured once by the
  // Remesher) and, when kept, the parents (old triangle of each new
  // triangle, old node or old edge of each new node).
  std::optional<mesh::MeshProperties> properties{};
  std::optional<mesh::Parents> parents{};
  // The estimate it acted on, if it has one.
  std::optional<ErrorEstimate> estimate{};
  // In stage order.
  std::vector<mesh::StageReport> reports{};
};

// One way of improving a discretization after a solve: mesh refinement
// now; later coarsening, moving nodes or revising the solver. It acts in
// two steps, so that a driver can test its target between them:
// estimate() looks at the solved state, adapt() acts on it.
class Adaptation {
public:
  virtual ~Adaptation() = default;
  virtual std::string name() const = 0;
  virtual AdaptationCapabilities capabilities() const = 0;

  // The adaptation's own error estimate of the state, or nothing when it
  // has none (capabilities().estimates_error is false).
  [[nodiscard]] virtual std::optional<ErrorEstimate>
  estimate(const SolvedState &state) const = 0;

  // Acts on the state. `estimate` is what estimate() returned for this
  // same state, or null: the adaptation then estimates itself, so one
  // call does everything.
  [[nodiscard]] virtual AdaptationResult
  adapt(const SolvedState &state, const ErrorEstimate *estimate) const = 0;
};

} // namespace pde2d::adapt
