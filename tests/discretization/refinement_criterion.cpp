// User refinement criteria (RefinementOptions::criterion, size_criterion.hpp)
// and the interior algorithm capabilities. Oracles: a user criterion built
// from the public constant/size-field builders reproduces the built-in
// option paths bit for bit; an anisotropic criterion is checked against its
// own rule written out independently on every edge of the result; error
// messages are the documented ones.
#include "discretization/geometry.hpp"
#include "discretization/mesh_io.hpp"
#include "discretization/size_criterion.hpp"
#include "discretization/triangulate.hpp"
#include "support/check.hpp"
#include <cmath>
#include <filesystem>
#include <string>
using namespace pde2d::discretization;
using pde2d_test::check;
using pde2d_test::expect_throws;

namespace {
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
template <class F> bool every_edge(const TriangleMesh &mesh, F &&ok) {
  for (const auto &t : mesh.elements)
    for (int e = 0; e < 3; ++e)
      if (!ok(mesh.nodes[t[e]], mesh.nodes[t[(e + 1) % 3]]))
        return false;
  return true;
}
// Edges fail when their x extent exceeds hx: long edges along y are fine.
RefinementCriterion x_extent(double hx) {
  RefinementCriterion c;
  c.size = [hx](Point2 a, Point2 b) -> std::optional<SizePriority> {
    const double dx = std::abs(b.x - a.x);
    if (dx > hx)
      return SizePriority{0, dx / hx, std::hypot(b.x - a.x, b.y - a.y)};
    return std::nullopt;
  };
  c.fixed_conflict = "Fixed boundary edge is too wide";
  c.descriptor = pde2d::StrategyDescriptor{"XExtent", {{"hx", "0.1"}}};
  return c;
}

void capabilities_are_declared() {
  const auto ear =
      interior_algorithm_capabilities(InteriorAlgorithm::EarClipping);
  check(ear == InteriorAlgorithmCapabilities{}, "ear clipping only seeds");
  const auto cdt =
      interior_algorithm_capabilities(InteriorAlgorithm::ConstrainedDelaunay);
  check(cdt.supports_size_target && cdt.supports_size_field &&
            cdt.supports_size_criterion && cdt.supports_refinement &&
            cdt.supports_marked_refinement && cdt.constrained_delaunay &&
            !cdt.requires_boundary_context,
        "constrained Delaunay");
  expect_throws<std::invalid_argument>(
      [] {
        (void)interior_algorithm_capabilities(
            static_cast<InteriorAlgorithm>(7));
      },
      "Unknown interior algorithm");
  // The generation-specific errors are the capabilities' messages.
  MeshOptions ear_options;
  ear_options.algorithm = InteriorAlgorithm::EarClipping;
  ear_options.refinement.max_edge_length = .2;
  expect_throws<std::invalid_argument>(
      [&] { (void)generate_mesh(unit_square(), ear_options); },
      "Ear clipping has no interior refinement; use constrained Delaunay or "
      "refine_mesh with subdivision");
  ear_options.refinement.max_edge_length = 0;
  ear_options.refinement.criterion = x_extent(.1);
  expect_throws<std::invalid_argument>(
      [&] { (void)generate_mesh(unit_square(), ear_options); },
      "Ear clipping has no interior refinement");
  ear_options.refinement.criterion.reset();
  const auto seed = generate_mesh(unit_square(), ear_options);
  check(!seed.report.delaunay_verified && seed.report.refinement_steps == 0);
  expect_throws<std::invalid_argument>(
      [&] { (void)refine_mesh(seed.mesh, ear_options); },
      "refine_mesh uses constrained Delaunay; use subdivide_mesh for "
      "subdivision");
  expect_throws<std::invalid_argument>(
      [&] { (void)refine_mesh(seed, {0}, ear_options); },
      "refine_mesh uses constrained Delaunay");
}

void builders_reproduce_the_option_paths() {
  for (auto treatment : {BoundaryTreatment::SplitPolygonEdges,
                         BoundaryTreatment::RefineOnCurves}) {
    MeshOptions scalar;
    scalar.boundary_treatment = treatment;
    scalar.refinement.max_edge_length = .1;
    MeshOptions user = scalar;
    user.refinement.max_edge_length = 0;
    user.refinement.criterion =
        RefinementCriterion{constant_size_criterion(.1)};
    const auto a = generate_mesh(CircleGeometry(1.0), scalar);
    const auto b = generate_mesh(CircleGeometry(1.0), user);
    check(a.report.size_satisfied && b.report.size_satisfied);
    check(same_mesh(a.mesh, b.mesh) &&
              a.report.refinement_steps == b.report.refinement_steps,
          "a constant user criterion is the scalar target bit for bit");

    const SizeField h = [](Point2 p) { return .04 + .2 * std::abs(p.x); };
    MeshOptions field = scalar;
    field.refinement.max_edge_length = .15;
    field.refinement.size_field = h;
    MeshOptions user_field = scalar;
    user_field.refinement.max_edge_length = 0;
    user_field.refinement.criterion =
        RefinementCriterion{size_field_criterion(h, .15)};
    const auto c = generate_mesh(CircleGeometry(1.0), field);
    const auto d = generate_mesh(CircleGeometry(1.0), user_field);
    check(same_mesh(c.mesh, d.mesh), "the size-field builder is the field");
    // Refinement of an existing result follows the same path.
    MeshOptions finer = scalar;
    finer.refinement.max_edge_length = .07;
    MeshOptions finer_user = user;
    finer_user.refinement.criterion =
        RefinementCriterion{constant_size_criterion(.07)};
    check(
        same_mesh(refine_mesh(a, finer).mesh, refine_mesh(b, finer_user).mesh),
        "refine_mesh with a user criterion");
  }
}

void anisotropic_criterion() {
  MeshOptions o;
  o.boundary_treatment = BoundaryTreatment::SplitPolygonEdges;
  o.refinement.criterion = x_extent(.1);
  std::size_t splits = 0;
  o.refinement.criterion->split_hook = [&](Point2, Point2) { ++splits; };
  const auto r = generate_mesh(unit_square(), o);
  check(r.report.size_satisfied, r.report.message);
  check(r.report.delaunay_verified && is_constrained_delaunay(r.mesh));
  check(splits == r.report.refinement_steps && splits > 0,
        "the split hook sees every split");
  check(every_edge(r.mesh, [](Point2 a,
                              Point2 b) { return std::abs(b.x - a.x) <= .1; }),
        "every edge meets the x-extent rule");
  check(!every_edge(r.mesh, [](Point2 a,
                               Point2 b) { return std::abs(b.y - a.y) <= .1; }),
        "the criterion is anisotropic: y extents stay larger");
  check(r.options.refinement.criterion &&
            r.options.refinement.criterion->descriptor->name == "XExtent",
        "the result keeps the criterion and its descriptor");

  // strictest_of combines a user rule with a built-in target.
  MeshOptions both = o;
  both.refinement.criterion->size =
      strictest_of(x_extent(.1).size, constant_size_criterion(.25));
  both.refinement.criterion->split_hook = {};
  const auto s = generate_mesh(unit_square(), both);
  check(s.report.size_satisfied, s.report.message);
  check(every_edge(s.mesh,
                   [](Point2 a, Point2 b) {
                     return std::abs(b.x - a.x) <= .1 &&
                            std::hypot(b.x - a.x, b.y - a.y) <=
                                .25 * (1 + 1e-12);
                   }),
        "both rules hold");

  // Marked refinement enforces the criterion afterwards.
  MeshOptions coarse;
  coarse.boundary_treatment = BoundaryTreatment::SplitPolygonEdges;
  coarse.refinement.max_edge_length = .3;
  const auto base = generate_mesh(unit_square(), coarse);
  MeshOptions marked_options = o;
  marked_options.refinement.criterion->split_hook = {};
  const auto marked = refine_mesh(base, {0, 1}, marked_options);
  check(marked.report.size_satisfied && marked.report.marked &&
            marked.report.marked->bisected_edges > 0,
        marked.report.message);
  check(
      every_edge(marked.mesh,
                 [](Point2 a, Point2 b) { return std::abs(b.x - a.x) <= .1; }),
      "marked refinement then meets the criterion");

  // A Fixed boundary that fails the criterion reports its message.
  MeshOptions fixed;
  fixed.boundary.max_edge_length = .5;
  fixed.refinement.criterion = x_extent(.1);
  const auto conflict = generate_mesh(unit_square(), fixed);
  check(!conflict.report.size_satisfied &&
            conflict.report.message == "Fixed boundary edge is too wide",
        conflict.report.message);

  // Request-time: files do not store the criterion.
  const auto path = std::filesystem::temp_directory_path() /
                    "pde2d_refinement_criterion.pde2d";
  export_mesh(r, path.string());
  const auto restored = import_mesh_result(path.string());
  std::filesystem::remove(path);
  check(!restored.options.refinement.criterion, "not serialized");
  check(same_mesh(restored.mesh, r.mesh));
}

void invalid_requests() {
  MeshOptions o;
  o.refinement.criterion = x_extent(.1);
  o.refinement.max_edge_length = .2;
  expect_throws<std::invalid_argument>(
      [&] { (void)generate_mesh(unit_square(), o); },
      "A refinement criterion replaces max_edge_length and size_field");
  o.refinement.max_edge_length = 0;
  o.refinement.size_field = [](Point2) { return .2; };
  expect_throws<std::invalid_argument>(
      [&] { (void)generate_mesh(unit_square(), o); }, "replaces");
  o.refinement.size_field = {};
  o.refinement.criterion = RefinementCriterion{};
  expect_throws<std::invalid_argument>(
      [&] { (void)generate_mesh(unit_square(), o); },
      "Refinement criterion has no size test");
}
} // namespace

int main() {
  return pde2d_test::run(
      {{"capabilities_are_declared", capabilities_are_declared},
       {"builders_reproduce_the_option_paths",
        builders_reproduce_the_option_paths},
       {"anisotropic_criterion", anisotropic_criterion},
       {"invalid_requests", invalid_requests}});
}
