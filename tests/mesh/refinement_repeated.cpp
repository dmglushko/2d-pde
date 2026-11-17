// Repeated refinement of marked triangles on straight domains: the mesh
// stays conforming and shape regular. Newest vertex bisection makes at
// most four similarity classes per initial triangle; red-green-blue
// refinement, whose red children are similar to their parent, makes no
// others. Longest-edge bisection (whose bisection edges are the longest
// edges at each refinement, not inherited) is checked for its smallest
// angle only.

#include "mesh/refinement_checks.hpp"
#include "support/check.hpp"
#include "support/mesh_checks.hpp"
#include "support/shapes.hpp"

#include <algorithm>
#include <functional>
#include <memory>
#include <vector>

using namespace pde2d::mesh;
using namespace pde2d::test;

namespace {

// Which triangles a cycle marks (the marking strategies are in
// pde2d_adapt; these are the plain cases, written out).
using Marking = std::function<Marked(const Mesh &)>;

// A cheap conformity check for big meshes (check_valid_mesh is brute
// force): counterclockwise triangles, every directed edge used once, and
// the edges without a twin are exactly the boundary edges.
void check_conforming(const Mesh &m) {
  std::vector<std::pair<std::size_t, std::size_t>> directed;
  directed.reserve(3 * m.triangles.size());
  for (const auto &t : m.triangles) {
    check(triangle_area(m.nodes[t.nodes[0]], m.nodes[t.nodes[1]],
                        m.nodes[t.nodes[2]]) > 0,
          "counterclockwise");
    for (int i = 0; i < 3; ++i)
      directed.push_back({t.nodes[i], t.nodes[(i + 1) % 3]});
  }
  std::sort(directed.begin(), directed.end());
  check(std::adjacent_find(directed.begin(), directed.end()) == directed.end(),
        "no directed edge twice");
  std::vector<std::pair<std::size_t, std::size_t>> open;
  for (auto [a, b] : directed)
    if (!std::binary_search(directed.begin(), directed.end(), std::pair{b, a}))
      open.push_back({a, b});
  std::vector<std::pair<std::size_t, std::size_t>> boundary;
  for (const auto &e : m.boundary_edges)
    boundary.push_back({e.nodes[0], e.nodes[1]});
  std::sort(boundary.begin(), boundary.end());
  check(open == boundary, "edges used once are the boundary edges");
}

// Refines `count` times with the marks of `marking`, checking each
// result, and checks the shapes: at most four similarity classes per
// initial triangle over all cycles (newest vertex and red-green-blue),
// and the smallest angle never below the smallest of the first three
// cycles.
void cycles(const std::string &label, Mesh m, int count, const Marking &marking,
            std::size_t full_check_up_to = 1'000'000) {
  for (const auto &refiner : bisection_refiners()) {
    Mesh mesh = m;
    const std::size_t initial = mesh.triangles.size();
    const double area = mesh_area(mesh);
    auto classes = shape_classes(mesh);
    std::vector<double> min_angles{min_angle_degrees(mesh)};
    for (int cycle = 1; cycle <= count; ++cycle) {
      const Marked marked = marking(mesh);
      check(!marked.empty(), "something is marked");
      auto r = refine_with(refiner, mesh, marked);
      if (r.mesh.triangles.size() <= full_check_up_to) {
        check_refinement(mesh, r, marked,
                         {.marked_children = marked_children(*refiner)});
      } else {
        check_conforming(r.mesh);
        near(mesh_area(r.mesh), area, 1e-10 * area, "area preserved");
      }
      near(refiner_report(r).metrics.at("incompatible_splits"), 0, 0,
           "refinement edges stay compatible");
      mesh = std::move(r.mesh);
      classes = shape_classes(mesh, classes);
      min_angles.push_back(min_angle_degrees(mesh));
      std::cout << "INFO " << refiner->name() << ' ' << label << " cycle "
                << cycle << ": " << mesh.triangles.size()
                << " triangles, min angle " << min_angles.back()
                << ", shape classes so far " << classes.size() << '\n';
    }
    if (refiner->name() != "longest_edge_bisection")
      check(classes.size() <= 4 * initial,
            refiner->name() + " " + label +
                ": at most four similarity classes per initial triangle");
    const double early =
        *std::min_element(min_angles.begin(), min_angles.begin() + 4);
    for (std::size_t k = 4; k < min_angles.size(); ++k)
      check(min_angles[k] >= early - 1e-9,
            refiner->name() + " " + label +
                ": the minimum angle does not drop after the first cycles");
  }
}

Marking everything() {
  return [](const Mesh &m) {
    Marked all;
    for (std::size_t t = 0; t < m.triangles.size(); ++t)
      all.push_back(t);
    return all;
  };
}

// The triangles touching node position `corner`.
Marking touching(Point corner) {
  return [corner](const Mesh &m) {
    Marked marked;
    for (std::size_t t = 0; t < m.triangles.size(); ++t)
      for (auto v : m.triangles[t].nodes)
        if (m.nodes[v] == corner) {
          marked.push_back(t);
          break;
        }
    return marked;
  };
}

// One scalene triangle, refined uniformly ten times (4^10 triangles).
void uniform_triangle() {
  Mesh m = initial_mesh(
      CurveGeometry({polygon_loop({{0, 0}, {3, 0}, {1, 2}}, "edge")}), 10);
  check(m.triangles.size() == 1, "one triangle");
  cycles("uniform triangle", m, 10, everything(), 20'000);
}

void uniform_l_shape() {
  cycles("uniform L-shape", initial_mesh(l_shape(), 0.8), 6, everything(),
         20'000);
}

// Corner-focused: the triangles at the re-entrant corner, 15 times.
void corner_l_shape() {
  cycles("corner L-shape", initial_mesh(l_shape(), 0.5), 15, touching({1, 1}));
}

void corner_square() {
  cycles("corner square",
         initial_mesh(CurveGeometry({square_loop({0, 0}, 1, "square")}), 0.3),
         15, touching({0, 0}));
}

} // namespace

int main() {
  return run({{"uniform_triangle", uniform_triangle},
              {"uniform_l_shape", uniform_l_shape},
              {"corner_l_shape", corner_l_shape},
              {"corner_square", corner_square}});
}
