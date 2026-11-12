#include "pde2d/mesh/components.hpp"

#include <stdexcept>

namespace pde2d::mesh {

namespace {

// Why the refiner does not accept this improver locally, or empty if it
// does.
std::string refusal(const Refiner &refiner, const ImproverEffects &fx) {
  auto caps = refiner.capabilities();
  if (fx.makes_delaunay && caps.maintains_delaunay)
    return refiner.name() + " already maintains Delaunay";
  if (fx.flips_edges && !caps.tolerates_flips)
    return refiner.name() + " does not tolerate edge flips";
  if (fx.moves_nodes && !caps.tolerates_node_moves)
    return refiner.name() + " does not tolerate node moves";
  return {};
}

} // namespace

PipelinePlan
plan_pipeline(const Refiner *refiner,
              const std::vector<std::shared_ptr<const Improver>> &improvers) {
  PipelinePlan plan;
  for (std::size_t i = 0; i < improvers.size(); ++i) {
    const auto &improver = improvers[i];
    if (!improver)
      throw std::invalid_argument("Improver list contains a null pointer");
    ImproverUse use = improver->use();
    bool wants_local = use != ImproverUse::final_only;
    bool wants_final = use != ImproverUse::local_only;
    // Why each wanted role cannot be honoured (empty: it can).
    std::string local_reason, final_reason;
    if (wants_local) {
      if (!improver->as_local())
        local_reason = "it has no local version";
      else if (!refiner)
        local_reason = "there is no refiner to run it locally";
      else
        local_reason = refusal(*refiner, improver->effects());
    }
    if (wants_final && !improver->as_final())
      final_reason = "it has no final version";
    bool runs_local = wants_local && local_reason.empty();
    bool runs_final = wants_final && final_reason.empty();
    if (runs_local)
      plan.local_improvers.push_back(i);
    if (runs_final)
      plan.final_passes.push_back(i);
    if (wants_local && !runs_local)
      plan.warnings.push_back(improver->name() + ": " + local_reason +
                              (runs_final ? ", so it runs only as a final pass"
                                          : ", so it does not run at all"));
    if (wants_final && !runs_final)
      plan.warnings.push_back(improver->name() + ": " + final_reason +
                              (runs_local ? ", so it runs only locally"
                                          : ", so it does not run at all"));
  }

  // Final passes that would make an already Delaunay mesh Delaunay.
  bool delaunay = false;
  std::string source;
  if (refiner && refiner->capabilities().maintains_delaunay) {
    delaunay = true;
    source = refiner->name();
  }
  for (auto i : plan.final_passes) {
    auto fx = improvers[i]->effects();
    if (fx.makes_delaunay && delaunay)
      plan.warnings.push_back(improvers[i]->name() +
                              ": the mesh is already Delaunay after " + source +
                              ", so this final pass changes nothing");
    if (fx.makes_delaunay) {
      delaunay = true;
      source = improvers[i]->name();
    } else if (fx.flips_edges || fx.moves_nodes) {
      delaunay = false;
    }
  }
  return plan;
}

} // namespace pde2d::mesh
