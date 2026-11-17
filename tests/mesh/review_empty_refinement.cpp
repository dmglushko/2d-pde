// From the review of the shared refiners: Remesher::refine with nothing
// marked used to run the final passes anyway (a Delaunay pass flipped an
// edge on a mesh nobody asked to change) while the plan said the pass
// changes nothing. Now the refiner finds nothing to do and the final
// passes do not run: the mesh comes back as it is.
#include "support/check.hpp"
#include "support/mesh_checks.hpp"

#include "pde2d/mesh/delaunay_flips.hpp"
#include "pde2d/mesh/remesher.hpp"
#include "pde2d/mesh/ruppert.hpp"

#include <memory>
#include <string>

using namespace pde2d::mesh;
using namespace pde2d::test;

namespace {
void empty_refinement_changes_nothing() {
  // A quadrilateral with the non-Delaunay diagonal: a Delaunay pass would
  // flip it.
  Mesh m;
  m.nodes = {{0, 0}, {2, 0}, {2, 1}, {0, 2}};
  m.triangles = {{{0, 1, 3}}, {{1, 2, 3}}};
  m.region_names = {"wall"};
  m.boundary_edges = {{{0, 1}, 0, std::nullopt},
                      {{1, 2}, 0, std::nullopt},
                      {{2, 3}, 0, std::nullopt},
                      {{3, 0}, 0, std::nullopt}};
  check_valid_mesh(m);
  check(!is_delaunay(m), "the quadrilateral needs a diagonal flip");
  const Remesher remesher(std::make_shared<Ruppert>(),
                          {std::make_shared<DelaunayFlips>()});
  const auto result = remesher.refine(m, {});
  check(result.mesh.nodes == m.nodes, "nodes unchanged");
  check(result.mesh.triangles.size() == m.triangles.size(),
        "triangle count unchanged");
  for (std::size_t t = 0; t < m.triangles.size(); ++t)
    check(result.mesh.triangles[t].nodes == m.triangles[t].nodes,
          "triangle unchanged (no flip)");
  check(result.reports.size() == 2,
        "the pipeline and the refiner report, no final pass");
  bool said = false;
  for (const auto &message : result.reports.front().messages)
    said = said ||
           message.find("the final passes did not run") != std::string::npos;
  check(said, "the pipeline report says the final passes did not run");
  check(result.properties.triangles == 2, "properties of the returned mesh");
}
} // namespace

int main() {
  return run({{"refine with nothing marked changes nothing",
               empty_refinement_changes_nothing}});
}
