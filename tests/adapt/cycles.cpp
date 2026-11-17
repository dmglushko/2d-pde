// Marking and refinement together, as an adaptive loop does it: a marking
// strategy picks triangles from indicators, Remesher::refine refines them.
// The indicator is shaped like the error of a corner singularity.

#include "mesh/refinement_checks.hpp"
#include "support/check.hpp"
#include "support/mesh_checks.hpp"
#include "support/shapes.hpp"

#include "pde2d/adapt/dorfler_marking.hpp"
#include "pde2d/adapt/maximum_marking.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

using namespace pde2d::test;

namespace {

// sqrt(area) / distance to `corner`: like the error of a corner
// singularity.
std::vector<double> peaked(const Mesh &m, Point corner) {
  std::vector<double> eta;
  for (std::size_t t = 0; t < m.triangles.size(); ++t) {
    auto c = corners(m, t);
    Point centroid{(c[0].x + c[1].x + c[2].x) / 3,
                   (c[0].y + c[1].y + c[2].y) / 3};
    eta.push_back(std::sqrt(triangle_area(c[0], c[1], c[2])) /
                  distance(centroid, corner));
  }
  return eta;
}

// Twelve Dörfler cycles on the L-shape towards its re-entrant corner,
// starting directly from the Mesher's mesh (CDT; no call to
// set_longest_edges_as_refinement_edges: the Mesher delivers the order):
// every result passes the oracles, the shapes stay in at most four
// similarity classes per initial triangle (newest vertex, red-green-blue)
// and the smallest angle does not drop after the first cycles.
void dorfler_l_shape() {
  const pde2d::adapt::DorflerMarking dorfler({.theta = 0.3});
  for (const auto &refiner : bisection_refiners()) {
    Mesh mesh = initial_mesh(l_shape(), 0.5);
    const std::size_t initial = mesh.triangles.size();
    auto classes = shape_classes(mesh);
    std::vector<double> min_angles{min_angle_degrees(mesh)};
    for (int cycle = 1; cycle <= 12; ++cycle) {
      const Marked marked = dorfler.mark(mesh, peaked(mesh, {1, 1}));
      auto r = refine_with(refiner, mesh, marked);
      check_refinement(mesh, r, marked,
                       {.marked_children = marked_children(*refiner)});
      mesh = std::move(r.mesh);
      classes = shape_classes(mesh, classes);
      min_angles.push_back(min_angle_degrees(mesh));
      std::cout << "INFO " << refiner->name() << " Dorfler L-shape cycle "
                << cycle << ": " << mesh.triangles.size()
                << " triangles, min angle " << min_angles.back() << '\n';
    }
    if (refiner->name() != "longest_edge_bisection")
      check(classes.size() <= 4 * initial,
            refiner->name() + ": at most four similarity classes per initial "
                              "triangle");
    const double early =
        *std::min_element(min_angles.begin(), min_angles.begin() + 4);
    for (std::size_t k = 4; k < min_angles.size(); ++k)
      check(min_angles[k] >= early - 1e-9,
            refiner->name() + ": the minimum angle does not drop");
  }
}

// The maximum strategy's marks refine exactly the marked triangles (and
// the closure): the marked ones are refined, an unmarked triangle far from
// them is not.
void maximum_marks_are_refined() {
  Mesh mesh = initial_mesh(CurveGeometry({square_loop({0, 0}, 1, "s")}), 0.3);
  const pde2d::adapt::MaximumMarking maximum({.theta = 0.9});
  const Marked marked = maximum.mark(mesh, peaked(mesh, {0, 0}));
  check(!marked.empty(), "something is marked");
  for (const auto &refiner : bisection_refiners()) {
    auto r = refine_with(refiner, mesh, marked);
    check_refinement(mesh, r, marked,
                     {.marked_children = marked_children(*refiner)});
  }
}

} // namespace

int main() {
  return run({{"dorfler_l_shape", dorfler_l_shape},
              {"maximum_marks_are_refined", maximum_marks_are_refined}});
}
