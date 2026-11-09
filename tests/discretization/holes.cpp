// Multiply connected domains: geometry with holes, nesting validation,
// meshing (ear clipping with bridges, CDT, curve refinement), mesh
// validation, point location and the version 5 file format. Oracles: polygon
// areas by the shoelace formula, exact circle radii, Euler characteristics,
// the independent CDT verifier and byte-identical re-export.
#include "discretization/geometry.hpp"
#include "discretization/mesh_io.hpp"
#include "discretization/point_location.hpp"
#include "discretization/triangulate.hpp"
#include "support/cdt.hpp"
#include "support/check.hpp"
#include "support/geometry.hpp"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <numbers>
#include <set>
#include <sstream>
#include <string>

using namespace pde2d::discretization;
using pde2d_test::check;
using pde2d_test::expect_throws;
using pde2d_test::near;

namespace {
std::string fixture_dir;

double shoelace(const std::vector<Point2> &p) {
  double twice = 0;
  for (std::size_t i = 0; i < p.size(); ++i) {
    auto a = p[i], b = p[(i + 1) % p.size()];
    twice += a.x * b.y - a.y * b.x;
  }
  return std::abs(twice) / 2;
}
double polygon_area(const Boundary &b) {
  double area = shoelace(b.points);
  for (const auto &h : b.holes)
    area -= shoelace(h.points);
  return area;
}
double mesh_area(const TriangleMesh &m) {
  double area = 0;
  for (auto t : m.elements) {
    auto a = m.nodes[t[0]], b = m.nodes[t[1]], c = m.nodes[t[2]];
    area += ((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y)) / 2;
  }
  return area;
}
std::size_t edge_count(const TriangleMesh &m) {
  std::set<std::pair<std::size_t, std::size_t>> edges;
  for (auto t : m.elements)
    for (int e = 0; e < 3; ++e)
      edges.insert(std::minmax(t[e], t[(e + 1) % 3]));
  return edges.size();
}
// V - E + F of a connected planar domain with `loops` boundary loops.
void check_topology(const TriangleMesh &m, std::size_t loops) {
  validate_mesh_geometry(m);
  check(boundary_loops(m).size() == loops, "Boundary loop count");
  check(static_cast<long>(m.nodes.size()) - static_cast<long>(edge_count(m)) +
                static_cast<long>(m.elements.size()) ==
            2 - static_cast<long>(loops),
        "Euler characteristic 2 - loops");
}
std::vector<CurveSegment> square(double x0, double y0, double s,
                                 bool clockwise = false) {
  std::vector<Point2> p{{x0, y0}, {x0 + s, y0}, {x0 + s, y0 + s}, {x0, y0 + s}};
  if (clockwise)
    std::reverse(p.begin(), p.end());
  std::vector<CurveSegment> lines;
  for (std::size_t i = 0; i < 4; ++i)
    lines.push_back(LineCurve{p[i], p[(i + 1) % 4]});
  return lines;
}
CurveSegment circle(Point2 c, double r) {
  return EllipticalArc{c, r, r, 0, 0, 2 * std::numbers::pi};
}
// Square [0,3]^2 with the hole [1,2]^2, triangulated by hand; boundary edges
// of the hole listed first when `hole_first`.
TriangleMesh framed_square(bool hole_first = false) {
  TriangleMesh m;
  m.nodes = {{0, 0}, {3, 0}, {3, 3}, {0, 3}, {1, 1}, {2, 1}, {2, 2}, {1, 2}};
  m.elements = {{0, 1, 5}, {0, 5, 4}, {1, 2, 6}, {1, 6, 5},
                {2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7}};
  std::vector<std::pair<std::size_t, std::size_t>> outer{
      {0, 1}, {1, 2}, {2, 3}, {3, 0}},
      inner{{4, 5}, {5, 6}, {6, 7}, {7, 4}};
  m.boundary_edges = hole_first ? inner : outer;
  m.boundary_edges.insert(m.boundary_edges.end(),
                          (hole_first ? outer : inner).begin(),
                          (hole_first ? outer : inner).end());
  for (std::size_t i = 0; i < 8; ++i)
    m.boundary_tags.push_back((i < 4) == !hole_first ? "wall" : "hole");
  return m;
}

void geometry_loops_and_regions() {
  // A region may span loops: "wall" is on the outer loop and on hole 2.
  CurveGeometry g(
      {{"wall", square(0, 0, 10)}},
      {{{"a", square(1, 1, 2)}},
       {{"wall", {LineCurve{{5, 5}, {7, 5}}}},
        {"b", {LineCurve{{7, 5}, {7, 7}}, LineCurve{{7, 7}, {5, 7}}}},
        {"wall", {LineCurve{{5, 7}, {5, 5}}}}}});
  check(g.loopCount() == 3, "Outer loop plus two holes");
  check(g.curves().size() == 12);
  for (std::size_t i = 0; i < 12; ++i)
    check(g.curves()[i].loop == (i < 4 ? 0 : i < 8 ? 1 : 2), "Loop of curve");
  // "wall": the whole outer loop, and on hole 2 one piece joined across that
  // loop's stored seam (last curve followed by first).
  auto wall = g.region("wall");
  check(!wall.connected() && wall.pieces.size() == 2 && wall.curves.size() == 6,
        "Region pieces per loop");
  check(wall.pieces[0].size() == 4 && wall.pieces[1].size() == 2);
  check(wall.pieces[1][0] == g.curves()[11].id &&
            wall.pieces[1][1] == g.curves()[8].id,
        "Seam piece joined within its own loop");
  check(g.region("a").connected() && g.region("b").connected());
  // Region length is summed over loops: 40 + 4 on hole 2.
  near(g.regionLength("wall"), 44, 1e-12);
  // Splitting a hole region by length keeps its loop.
  // 3/8 of the hole's length lies inside its second side.
  auto parts = g.splitRegion("a", 0.375, "a1", "a2");
  check(parts.size() == 5 && g.curves().size() == 13);
  for (auto id : parts)
    check(g.curve(id).loop == 1, "Split pieces stay on their loop");
  check(g.loopCount() == 3);
  // Positional runs never continue from one loop into the next.
  check(g.curveIdAt(0, 0) == g.curves()[0].id);
  expect_throws<std::invalid_argument>(
      [] { CurveGeometry({{"o", square(0, 0, 1)}}, {{}}); },
      "holes require regions");

  AnnulusGeometry annulus(2, 1);
  check(annulus.loopCount() == 2 && annulus.curves().size() == 2);
  check(annulus.curves()[1].loop == 1 && annulus.curves()[1].region == "inner");
  annulus.renameRegion("inner", "cylinder");
  annulus.setRadii(3, 0.5); // edits replay on the resized shape
  check(annulus.region("cylinder").curves.size() == 1);
  near(annulus.regionLength("cylinder"), std::numbers::pi, 1e-12);
  expect_throws<std::invalid_argument>([&] { annulus.setInnerRadius(3); },
                                       "smaller than the outer");
  check(annulus.innerRadius() == 0.5, "Failed setter leaves the shape");
}

void polygon_nesting_validation() {
  auto mesh = [](CurveGeometry g) {
    MeshOptions o;
    o.boundary.max_edge_length = 0.5;
    return generate_mesh(g, o);
  };
  expect_throws<std::invalid_argument>(
      [&] {
        mesh(CurveGeometry({{"o", square(0, 0, 2)}},
                           {{{"h", square(3, 3, 1)}}}));
      },
      "Hole is not inside the outer boundary");
  expect_throws<std::invalid_argument>(
      [&] {
        mesh(CurveGeometry({{"o", square(0, 0, 2)}},
                           {{{"h", square(1, 1, 2)}}}));
      },
      "Boundary loops intersect or touch");
  // Touching at one vertex is rejected as well.
  expect_throws<std::invalid_argument>(
      [&] {
        mesh(CurveGeometry({{"o", square(0, 0, 2)}},
                           {{{"h", square(0, 0, 1)}}}));
      },
      "Boundary loops intersect or touch");
  expect_throws<std::invalid_argument>(
      [&] {
        mesh(CurveGeometry(
            {{"o", square(0, 0, 10)}},
            {{{"h", square(1, 1, 6)}}, {{"g", square(2, 2, 1)}}}));
      },
      "Holes must not be nested");
  expect_throws<std::invalid_argument>(
      [&] {
        mesh(CurveGeometry(
            {{"o", square(0, 0, 10)}},
            {{{"h", square(1, 1, 3)}}, {{"g", square(2, 2, 3)}}}));
      },
      "Boundary loops intersect or touch");
  // A self-intersecting hole fails the per-loop check.
  Boundary bow{{{0, 0}, {10, 0}, {10, 10}, {0, 10}},
               {},
               {{{{1, 1}, {3, 3}, {3, 1}, {1, 3}}, {}}}};
  expect_throws<std::invalid_argument>([&] { (void)generate_mesh(bow); },
                                       "Boundary self-intersects");
  // Hole tags must match their own loop.
  Boundary tags{{{0, 0}, {10, 0}, {10, 10}, {0, 10}},
                {},
                {{{{1, 1}, {3, 1}, {3, 3}}, {"x"}}}};
  expect_throws<std::invalid_argument>([&] { (void)generate_mesh(tags); },
                                       "Boundary tag count mismatch");
}

void meshing_with_holes() {
  // Either orientation of each loop meshes the same domain; boundary edges
  // keep the given direction.
  for (bool outer_cw : {false, true})
    for (bool hole_cw : {false, true}) {
      CurveGeometry g(
          {{"o", square(0, 0, 4, outer_cw)}},
          {{{"h", square(1, 1, 1, hole_cw)}}, {{"k", {circle({3, 3}, 0.4)}}}});
      for (auto alg : {InteriorAlgorithm::EarClipping,
                       InteriorAlgorithm::ConstrainedDelaunay}) {
        MeshOptions o;
        o.algorithm = alg;
        o.boundary.max_edge_length = 0.5;
        if (alg == InteriorAlgorithm::ConstrainedDelaunay)
          o.refinement.max_edge_length = 0.5;
        auto d = discretize_boundary(g, o.boundary);
        check(d.boundary.holes.size() == 2 &&
                  d.context->snapshot_loop_sizes.size() == 3,
              "Sampled loops");
        auto r = generate_mesh(d, o);
        check(r.report.size_satisfied, r.report.message);
        check_topology(r.mesh, 3);
        near(mesh_area(r.mesh), polygon_area(d.boundary), 1e-12);
        std::set<std::string> tags(r.mesh.boundary_tags.begin(),
                                   r.mesh.boundary_tags.end());
        check(tags == std::set<std::string>{"o", "h", "k"}, "Tags per loop");
        // The first boundary edge of every loop keeps the input direction.
        auto first = r.mesh.boundary_edges.front();
        check(first.first == 0 && first.second == 1);
        if (alg == InteriorAlgorithm::ConstrainedDelaunay) {
          check(r.report.delaunay_verified && is_constrained_delaunay(r.mesh));
          pde2d_test::independent_cdt(r.mesh);
        }
      }
    }
  // Many holes in a grid: collinear rightmost vertices and nearby bridges.
  std::vector<std::vector<CurveRegion>> holes;
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 3; ++j)
      holes.push_back({{"h", square(1 + 2.0 * i, 1 + 2.0 * j, 1)}});
  holes.push_back({{"c", {circle({4.5, 7.5}, 0.3)}}});
  CurveGeometry grid({{"o", square(0, 0, 9)}}, holes);
  MeshOptions ear;
  ear.algorithm = InteriorAlgorithm::EarClipping;
  ear.boundary.max_edge_length = 0.5;
  auto clipped = generate_mesh(grid, ear);
  check_topology(clipped.mesh, 14);
  auto sampled = discretize_boundary(grid, ear.boundary).boundary;
  near(mesh_area(clipped.mesh), polygon_area(sampled), 1e-12);
  MeshOptions cdt;
  cdt.boundary.max_edge_length = 0.5;
  cdt.refinement.max_edge_length = 0.3;
  cdt.boundary_treatment = BoundaryTreatment::SplitPolygonEdges;
  auto refined = generate_mesh(grid, cdt);
  check(refined.report.size_satisfied, refined.report.message);
  check_topology(refined.mesh, 14);
  near(mesh_area(refined.mesh), polygon_area(sampled), 1e-12);
  // A raw Boundary with holes (no analytical context), one hole untagged.
  Boundary raw{
      {{0, 0}, {6, 0}, {6, 4}, {0, 4}},
      {"s", "s", "s", "s"},
      {{{{1, 1}, {2, 1}, {2, 2}}, {}}, {{{4, 1}, {5, 3}, {4, 3}}, {}}}};
  auto r = generate_mesh(raw);
  check_topology(r.mesh, 3);
  check(r.mesh.boundary_tags.size() == 10 && r.mesh.boundary_tags[4].empty(),
        "Untagged hole edges are empty tags");
}

void curve_refinement_on_holes() {
  const double R = 1, rin = 0.35;
  AnnulusGeometry g(R, rin);
  g.setCenter(0.25, -0.5);
  MeshOptions o;
  o.boundary.max_edge_length = 0.4;
  o.boundary_treatment = BoundaryTreatment::RefineOnCurves;
  o.refinement.max_edge_length = 0.12;
  auto r = generate_mesh(g, o);
  check(r.report.size_satisfied, r.report.message);
  check_topology(r.mesh, 2);
  auto on_circles = [&](const MeshResult &m) {
    auto loops = boundary_loops(m.mesh);
    check(loops.size() == 2);
    for (std::size_t k = 0; k < 2; ++k)
      for (auto e : loops[k]) {
        auto p = m.mesh.nodes[m.mesh.boundary_edges[e].first];
        near(std::hypot(p.x - 0.25, p.y + 0.5), k == 0 ? R : rin, 1e-12);
        check(m.mesh.boundary_tags[e] == (k == 0 ? "outer" : "inner"));
      }
    for (const auto &c : m.boundary_context->curves)
      check(c.loop == (c.region == "outer" ? 0u : 1u), "Context curve loop");
  };
  on_circles(r);
  // The polygon area converges to the annulus area.
  const double exact = std::numbers::pi * (R * R - rin * rin);
  check(std::abs(mesh_area(r.mesh) - exact) < 0.01 * exact);
  // Size field graded towards the hole keeps new vertices on the curves.
  auto graded = o;
  graded.refinement.max_edge_length = 0;
  graded.refinement.size_field = [](Point2 p) {
    return 0.03 + 0.2 * (std::hypot(p.x - 0.25, p.y + 0.5) - 0.35);
  };
  auto fine = refine_mesh(r, graded);
  check(fine.report.size_satisfied, fine.report.message);
  on_circles(fine);
  check(fine.mesh.nodes.size() > r.mesh.nodes.size());
  // Marked refinement of the triangles touching the hole.
  std::set<std::size_t> hole_nodes;
  for (auto e : boundary_loops(r.mesh)[1])
    hole_nodes.insert(r.mesh.boundary_edges[e].first);
  std::vector<std::size_t> marked;
  for (std::size_t i = 0; i < r.mesh.elements.size(); ++i)
    for (auto v : r.mesh.elements[i])
      if (hole_nodes.contains(v)) {
        marked.push_back(i);
        break;
      }
  auto marked_options = o;
  marked_options.refinement.max_edge_length = 0;
  auto bisected = refine_mesh(r, marked, marked_options);
  check(bisected.report.size_satisfied, bisected.report.message);
  check(bisected.report.marked->bisected_edges > 0);
  check(bisected.mesh.nodes.size() > r.mesh.nodes.size());
  on_circles(bisected);
}

void mesh_validation_with_holes() {
  check_topology(framed_square(), 2);
  auto loops = boundary_loops(framed_square(true));
  check(loops.size() == 2 && loops[0].front() == 4 && loops[1].front() == 0,
        "Outer loop first even when its edges are listed last");
  check(loops[0] == std::vector<std::size_t>{4, 5, 6, 7});
  // Two separate triangles: two boundary loops but not one domain.
  TriangleMesh two{{{0, 0}, {1, 0}, {0, 1}, {5, 5}, {6, 5}, {5, 6}},
                   {{0, 1, 2}, {3, 4, 5}},
                   {{0, 1}, {1, 2}, {2, 0}, {3, 4}, {4, 5}, {5, 3}}};
  expect_throws<std::invalid_argument>([&] { validate_mesh_geometry(two); },
                                       "Disconnected mesh");
  // Two triangles sharing a vertex: the boundary is pinched.
  TriangleMesh pinch{{{0, 0}, {1, 0}, {0, 1}, {-1, 0}, {0, -1}},
                     {{0, 1, 2}, {0, 3, 4}},
                     {{0, 1}, {1, 2}, {2, 0}, {0, 3}, {3, 4}, {4, 0}}};
  expect_throws<std::invalid_argument>([&] { validate_mesh_geometry(pinch); },
                                       "Boundary is not a closed loop");
  // A hole edge missing from the boundary records.
  auto missing = framed_square();
  missing.boundary_edges.pop_back();
  missing.boundary_tags.pop_back();
  expect_throws<std::invalid_argument>(
      [&] { validate_mesh_geometry(missing); });
  // A triangle filling part of the hole overlaps nothing but leaves the
  // records wrong.
  auto filled = framed_square();
  filled.elements.push_back({4, 5, 6});
  expect_throws<std::invalid_argument>([&] { validate_mesh_geometry(filled); },
                                       "incorrect boundary records");
  // CDT refinement of the hand-made mesh treats hole edges as constraints.
  MeshOptions o;
  o.refinement.max_edge_length = 0.4;
  o.boundary_treatment = BoundaryTreatment::SplitPolygonEdges;
  auto refined = refine_mesh(framed_square(), o);
  check(refined.report.size_satisfied, refined.report.message);
  check_topology(refined.mesh, 2);
  near(mesh_area(refined.mesh), 8, 1e-12);
}

void point_location_on_annulus() {
  auto mesh = framed_square();
  PointLocator locator(mesh);
  check(!locator.locate({1.5, 1.5}), "The hole is outside the mesh");
  check(!locator.locate({1.5, 1.2}));
  auto inside = locator.locate({0.5, 1.5});
  check(inside && inside->inside, "Points in the band are located");
  auto on_hole_edge = locator.locate({1.5, 1});
  check(on_hole_edge && on_hole_edge->inside, "Hole edges belong to the mesh");
  PointLocator tolerant(mesh, {0.3});
  auto near_hole = tolerant.locate({1.5, 1.2});
  check(near_hole && !near_hole->inside, "Extrapolated from a hole edge");
  near(near_hole->distance, 0.2, 1e-15);
  // The owner of the hole edge (4, 5) is triangle (0, 5, 4).
  check(near_hole->element == 1);
  // Barycentric extrapolation reproduces affine fields.
  const auto &t = mesh.elements[near_hole->element];
  double value = 0;
  for (int k = 0; k < 3; ++k)
    value += near_hole->barycentric[k] *
             (2 * mesh.nodes[t[k]].x - mesh.nodes[t[k]].y);
  near(value, 2 * 1.5 - 1.2, 1e-14);
  check(!tolerant.locate({1.5, 1.5}), "The hole center is too far");
}

std::string read_file(const std::string &path) {
  std::ifstream in(path);
  std::stringstream s;
  s << in.rdbuf();
  return s.str();
}
void write_file(const std::string &path, const std::string &text) {
  std::ofstream(path) << text;
}

void version5_format() {
  namespace fs = std::filesystem;
  const auto dir = fs::path(fixture_dir);
  AnnulusGeometry g(1, 0.4);
  MeshOptions o;
  o.boundary.max_edge_length = 0.35;
  o.boundary_treatment = BoundaryTreatment::RefineOnCurves;
  o.refinement.max_edge_length = 0.2;
  auto r = generate_mesh(g, o);
  const auto path = (dir / "annulus_result.pde2d").string();
  export_mesh(r, path);
  const auto text = read_file(path);
  check(text.starts_with("PDE2D_MESH 5\n"), "Holes are written as version 5");
  check(text.find("BOUNDARY_LOOPS 2\n") != std::string::npos &&
            text.find("RESULT 1\n") != std::string::npos &&
            text.find("POLYGON_LOOPS 2\n") != std::string::npos,
        "Loop records");
  auto back = import_mesh_result(path);
  check(back.mesh.nodes.size() == r.mesh.nodes.size() &&
        back.mesh.elements == r.mesh.elements &&
        back.mesh.boundary_edges == r.mesh.boundary_edges &&
        back.mesh.boundary_tags == r.mesh.boundary_tags);
  for (std::size_t i = 0; i < r.mesh.nodes.size(); ++i)
    check(back.mesh.nodes[i].x == r.mesh.nodes[i].x &&
          back.mesh.nodes[i].y == r.mesh.nodes[i].y);
  const auto &c = *r.boundary_context, &d = *back.boundary_context;
  check(d.snapshot_loop_sizes == c.snapshot_loop_sizes &&
        d.protected_vertices == c.protected_vertices &&
        d.curves.size() == c.curves.size());
  for (std::size_t i = 0; i < c.curves.size(); ++i)
    check(d.curves[i].loop == c.curves[i].loop &&
              d.curves[i].id == c.curves[i].id &&
              d.curves[i].region == c.curves[i].region,
          "Source curve loops are derived on import");
  check(back.report.size_satisfied && back.report.delaunay_verified);
  const auto again = (dir / "annulus_again.pde2d").string();
  export_mesh(back, again);
  check(read_file(again) == text, "Re-export is byte-identical");
  // Resume curve refinement from the file.
  o.refinement.max_edge_length = 0.1;
  auto resumed = refine_mesh(back, o);
  check(resumed.report.size_satisfied, resumed.report.message);
  check_topology(resumed.mesh, 2);
  export_mesh(resumed, (dir / "annulus_refined.pde2d").string());

  // A raw mesh with holes: version 5 without a result; ungrouped edges are
  // regrouped with the outer loop first.
  const auto raw = (dir / "framed_raw.pde2d").string();
  export_mesh(framed_square(true), raw);
  const auto raw_text = read_file(raw);
  check(raw_text.starts_with("PDE2D_MESH 5\n") &&
        raw_text.ends_with("BOUNDARY_LOOPS 2\n4\n4\nRESULT 0\n"));
  auto raw_back = import_mesh_result(raw);
  check(raw_back.mesh.boundary_edges == framed_square().boundary_edges &&
            raw_back.mesh.boundary_tags == framed_square().boundary_tags,
        "Regrouped stably by loop");
  check(!raw_back.report.size_satisfied && !raw_back.boundary_context &&
        raw_back.report.message.find("not recorded") != std::string::npos);
  // A result with ungrouped edges and context is regrouped consistently.
  MeshResult shuffled = r;
  auto loops = boundary_loops(r.mesh);
  std::vector<std::size_t> order(loops[1].begin(), loops[1].end());
  order.insert(order.end(), loops[0].begin(), loops[0].end());
  auto permute = [&](auto values) {
    auto out = values;
    for (std::size_t i = 0; i < order.size(); ++i)
      out[i] = values[order[i]];
    return out;
  };
  shuffled.mesh.boundary_edges = permute(r.mesh.boundary_edges);
  shuffled.mesh.boundary_tags = permute(r.mesh.boundary_tags);
  shuffled.boundary_context->edge_sources =
      permute(r.boundary_context->edge_sources);
  const auto shuffled_path = (dir / "shuffled.pde2d").string();
  export_mesh(shuffled, shuffled_path);
  auto regrouped = import_mesh_result(shuffled_path);
  check(boundary_loops(regrouped.mesh).front().front() == 0 &&
            regrouped.mesh.boundary_tags.front() == "outer",
        "Outer loop is written first");

  // Single-outline meshes keep their historical versions.
  RectangleGeometry rect;
  auto plain = generate_mesh(rect);
  export_mesh(plain, (dir / "plain.pde2d").string());
  export_mesh(plain.mesh, (dir / "plain_raw.pde2d").string());
  check(
      read_file((dir / "plain.pde2d").string()).starts_with("PDE2D_MESH 4\n"));
  check(read_file((dir / "plain_raw.pde2d").string())
            .starts_with("PDE2D_MESH 2\n"));

  // Malformed version 5 files.
  auto rejects = [&](const std::string &body, const char *fragment) {
    const auto bad = (dir / "bad.pde2d").string();
    write_file(bad, body);
    expect_throws<std::exception>([&] { (void)import_mesh_result(bad); },
                                  fragment);
  };
  auto replace = [](std::string s, const std::string &a, const std::string &b) {
    auto at = s.find(a);
    check(at != std::string::npos, "Fixture edit applies");
    return s.replace(at, a.size(), b);
  };
  rejects(replace(raw_text, "PDE2D_MESH 5", "PDE2D_MESH 2")
              .substr(0, raw_text.find("BOUNDARY_LOOPS")),
          "requires PDE2D_MESH version 5");
  rejects(replace(raw_text, "4\n4\nRESULT", "5\n3\nRESULT"),
          "not grouped by their recorded loops");
  rejects(replace(raw_text, "4\n4\nRESULT", "4\n5\nRESULT"),
          "Invalid boundary loop sizes");
  rejects(
      replace(raw_text, "BOUNDARY_LOOPS 2\n4\n4\n", "BOUNDARY_LOOPS 1\n8\n"),
      "not grouped by their recorded loops");
  rejects(replace(raw_text, "RESULT 0", "RESULT 2"), "Invalid unsigned");
  rejects(replace(raw_text, "RESULT 0\n", ""), "Expected section RESULT");
  rejects(replace(text, "POLYGON_LOOPS 2\n", "POLYGON_LOOPS 1\n"),
          "polygon snapshot loop sizes");
  rejects(raw_text + "extra", "Unexpected content");
  rejects(replace(text, "PDE2D_MESH 5", "PDE2D_MESH 6"), "header");
}
} // namespace

int main(int argc, char **argv) {
  if (argc == 2)
    fixture_dir = argv[1];
  return pde2d_test::run(
      {{"geometry_loops_and_regions", geometry_loops_and_regions},
       {"polygon_nesting_validation", polygon_nesting_validation},
       {"meshing_with_holes", meshing_with_holes},
       {"curve_refinement_on_holes", curve_refinement_on_holes},
       {"mesh_validation_with_holes", mesh_validation_with_holes},
       {"point_location_on_annulus", point_location_on_annulus},
       {"version5_format", [] {
          check(!fixture_dir.empty(), "Supply private fixture directory");
          version5_format();
        }}});
}
