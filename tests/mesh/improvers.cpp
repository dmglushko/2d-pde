#include "support/check.hpp"
#include "support/mesh_checks.hpp"
#include "support/shapes.hpp"

#include "pde2d/mesh/chord_length_discretizer.hpp"
#include "pde2d/mesh/delaunay_flips.hpp"
#include "pde2d/mesh/ear_clipping.hpp"
#include "pde2d/mesh/laplacian_smoothing.hpp"
#include "pde2d/mesh/longest_edge_bisection.hpp"

using namespace pde2d::mesh;
using namespace pde2d::test;

namespace {

WorkingMesh refined(const CurveGeometry &g, double h) {
  StageReport r;
  auto polygon = ChordLengthDiscretizer().run(g, {h}, r);
  auto w = EarClipping().run(polygon, r);
  LongestEdgeBisection().run(w, {h}, {}, r);
  return w;
}

} // namespace

int main() {
  return run({
      {"delaunay_flips_whole_mesh",
       [] {
         auto w = refined(l_shape(), 0.2);
         check(!is_delaunay(w.to_mesh()), "bisection output is not Delaunay");
         StageReport r;
         DelaunayFlips().run(w, {0.2}, r);
         Mesh m = w.to_mesh();
         check_valid_mesh(m);
         check(is_delaunay(m), "Delaunay after the pass");
         check(r.metrics.at("flips") > 0, "flips counted");
         near(mesh_area(m), 3, 1e-12, "area unchanged");
       }},
      {"laplacian_smoothing",
       [] {
         auto w = refined(annulus(1, 0.4), 0.15);
         Mesh before = w.to_mesh();
         StageReport r;
         LaplacianSmoothing({.iterations = 5}).run(w, {0.15}, r);
         Mesh after = w.to_mesh();
         check_valid_mesh(after);
         for (auto v : boundary_nodes(before))
           check(before.nodes[v] == after.nodes[v], "boundary nodes fixed");
         check(r.metrics.at("moves") > 0, "nodes moved");
         near(mesh_area(after), mesh_area(before), 1e-12, "area unchanged");
         expect_throws<std::invalid_argument>(
             [] { LaplacianSmoothing({.iterations = 0}); }, "at least 1");
       }},
      {"effects_declared",
       [] {
         auto f = DelaunayFlips().effects();
         check(f.flips_edges && f.makes_delaunay && !f.moves_nodes, "flips");
         auto s = LaplacianSmoothing().effects();
         check(s.moves_nodes && !s.flips_edges && !s.makes_delaunay, "smooth");
       }},
  });
}
