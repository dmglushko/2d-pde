#include "pde2d/mesh/components.hpp"

#include <stdexcept>

namespace pde2d::mesh {

namespace {

// Why the refiner does not accept this local improver, or empty if it
// does.
std::string refusal(const Refiner &refiner, const LocalImprover &local) {
  auto caps = refiner.capabilities();
  auto fx = local.effects();
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
    if (wants_local) {
      std::string reason;
      const LocalImprover *local = improver->as_local_improver();
      if (!local)
        reason = "it has no local version";
      else if (!refiner)
        reason = "there is no refiner to run it locally";
      else
        reason = refusal(*refiner, *local);
      if (reason.empty())
        plan.local_improvers.push_back(i);
      else
        plan.warnings.push_back(improver->name() + ": " + reason +
                                (wants_final
                                     ? ", so it runs only as a final pass"
                                     : ", so it does not run at all"));
    }
    if (wants_final)
      plan.final_passes.push_back(i);
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
