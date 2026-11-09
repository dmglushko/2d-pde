// Spatially varying refinement targets (RefinementOptions::size_field).
// Oracles: the documented edge rule length <= min(h(a), h(mid), h(b)),
// element counts of the graded halves, and bit-for-bit equality of a
// constant field with the scalar max_edge_length path.
#include "discretization/geometry.hpp"
#include "discretization/mesh_io.hpp"
#include "discretization/triangulate.hpp"
#include "support/check.hpp"
#include <cmath>
#include <iostream>
#include <string>
using namespace pde2d::discretization;
using pde2d_test::check;
using pde2d_test::expect_throws;

namespace {
double edge_length(Point2 a, Point2 b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}
// The rule as documented in mesh_types.hpp, written out independently.
double edge_target(const SizeField &h, Point2 a, Point2 b) {
  Point2 mid{(a.x + b.x) / 2, (a.y + b.y) / 2};
  return std::min({h(a), h(mid), h(b)});
}
RectangleGeometry unit_square() {
  RectangleGeometry g(1, 1);
  g.setCenter(.5, .5);
  return g;
}
bool same_mesh(const TriangleMesh &a, const TriangleMesh &b) {
  if (a.nodes.size() != b.nodes.size() || a.elements != b.elements ||
      a.boundary_edges != b.boundary_edges ||
      a.boundary_tags != b.boundary_tags)
    return false;
  for (std::size_t i = 0; i < a.nodes.size(); ++i)
    if (a.nodes[i].x != b.nodes[i].x || a.nodes[i].y != b.nodes[i].y)
      return false;
  return true;
}

void graded_rectangle() {
  const SizeField h = [](Point2 p) { return 0.05 + 0.3 * p.x; };
  for (auto treatment : {BoundaryTreatment::SplitPolygonEdges,
                         BoundaryTreatment::RefineOnCurves}) {
    MeshOptions o;
    o.boundary_treatment = treatment;
    o.refinement.size_field = h;
    auto r = generate_mesh(unit_square(), o);
    check(r.report.size_satisfied, r.report.message);
    check(r.report.delaunay_verified && is_constrained_delaunay(r.mesh));
    validate_mesh_geometry(r.mesh);
    for (auto t : r.mesh.elements)
      for (int e = 0; e < 3; ++e) {
        auto a = r.mesh.nodes[t[e]], b = r.mesh.nodes[t[(e + 1) % 3]];
        check(edge_length(a, b) <= edge_target(h, a, b) * (1 + 1e-12),
              "edge exceeds its local size target");
      }
    std::size_t fine = 0, coarse = 0;
    for (auto t : r.mesh.elements) {
      double cx = 0;
      for (auto v : t)
        cx += r.mesh.nodes[v].x / 3;
      (cx < .5 ? fine : coarse) += 1;
    }
    std::cout << "graded: x<0.5 " << fine << " elements, x>0.5 " << coarse
              << '\n';
    // h ranges over [0.05, 0.2] on the left half and [0.2, 0.35] on the
    // right: element density scales like 1/h^2, so the left half needs
    // several times as many elements.
    check(fine > 4 * coarse, "size field did not grade the mesh");
    // A field never refines less than the constant target at its maximum.
    o.refinement.size_field = {};
    o.refinement.max_edge_length = 0.35;
    auto uniform = generate_mesh(unit_square(), o);
    check(r.mesh.elements.size() > 2 * uniform.mesh.elements.size());
  }
}

void constant_field_matches_scalar() {
  for (auto treatment :
       {BoundaryTreatment::SplitPolygonEdges, BoundaryTreatment::RefineOnCurves,
        BoundaryTreatment::Fixed}) {
    for (double target : {0.1, 0.07}) {
      MeshOptions scalar;
      scalar.boundary_treatment = treatment;
      scalar.boundary.max_edge_length = 0.05;
      scalar.refinement.max_edge_length = target;
      MeshOptions field = scalar;
      field.refinement.max_edge_length = 0;
      field.refinement.size_field = [target](Point2) { return target; };
      const CircleGeometry circle(1.0);
      const RectangleWithSemicircleGeometry capped;
      for (const CurveGeometry *g :
           {static_cast<const CurveGeometry *>(&circle),
            static_cast<const CurveGeometry *>(&capped)}) {
        auto a = generate_mesh(*g, scalar), b = generate_mesh(*g, field);
        check(same_mesh(a.mesh, b.mesh), "constant field differs from scalar");
        check(a.report.refinement_steps == b.report.refinement_steps);
        check(a.report.size_satisfied == b.report.size_satisfied);
        check(a.report.message == b.report.message);
      }
    }
  }
}

void combined_targets_and_errors() {
  // Both targets: the smaller one applies everywhere.
  MeshOptions o;
  o.boundary_treatment = BoundaryTreatment::SplitPolygonEdges;
  o.refinement.size_field = [](Point2 p) { return 0.05 + 0.3 * p.x; };
  o.refinement.max_edge_length = 0.15;
  auto r = generate_mesh(unit_square(), o);
  check(r.report.size_satisfied, r.report.message);
  for (auto t : r.mesh.elements)
    for (int e = 0; e < 3; ++e) {
      auto a = r.mesh.nodes[t[e]], b = r.mesh.nodes[t[(e + 1) % 3]];
      check(edge_length(a, b) <=
            std::min(0.15, edge_target(o.refinement.size_field, a, b)) *
                (1 + 1e-12));
    }
  // Fixed boundary edges that exceed the field cannot be refined.
  MeshOptions fixed;
  fixed.refinement.size_field = [](Point2) { return 0.2; };
  auto conflict = generate_mesh(unit_square(), fixed);
  check(!conflict.report.size_satisfied);
  check(conflict.report.message.find("local size target") != std::string::npos,
        conflict.report.message);
  // Invalid field values and incompatible algorithms throw.
  MeshOptions bad;
  bad.boundary_treatment = BoundaryTreatment::SplitPolygonEdges;
  bad.refinement.size_field = [](Point2 p) { return p.x - 0.5; };
  expect_throws<std::invalid_argument>(
      [&] { (void)generate_mesh(unit_square(), bad); },
      "Size field must return a finite positive length");
  bad.refinement.size_field = [](Point2) { return std::nan(""); };
  expect_throws<std::invalid_argument>(
      [&] { (void)generate_mesh(unit_square(), bad); }, "finite positive");
  MeshOptions ear;
  ear.algorithm = InteriorAlgorithm::EarClipping;
  ear.refinement.size_field = [](Point2) { return 0.2; };
  expect_throws<std::invalid_argument>(
      [&] { (void)generate_mesh(unit_square(), ear); },
      "Ear clipping has no interior");
  // A step budget still reports an unmet target honestly.
  o.refinement.max_steps = 10;
  auto partial = generate_mesh(unit_square(), o);
  check(!partial.report.size_satisfied &&
        partial.report.message == "Refinement step budget exhausted");
}

void field_is_not_serialized() {
  MeshOptions o;
  o.boundary_treatment = BoundaryTreatment::SplitPolygonEdges;
  o.refinement.size_field = [](Point2 p) { return 0.05 + 0.3 * p.x; };
  auto r = generate_mesh(unit_square(), o);
  // Each test runs in its own working directory.
  const std::string path = "size_field.pde2d";
  export_mesh(r, path);
  auto back = import_mesh_result(path);
  check(!back.options.refinement.size_field,
        "a callable cannot be restored from a file");
  check(back.options.refinement.max_edge_length == 0);
  check(same_mesh(back.mesh, r.mesh));
}
} // namespace

int main() {
  return pde2d_test::run(
      {{"graded_rectangle", graded_rectangle},
       {"constant_field_matches_scalar", constant_field_matches_scalar},
       {"combined_targets_and_errors", combined_targets_and_errors},
       {"field_is_not_serialized", field_is_not_serialized}});
}
