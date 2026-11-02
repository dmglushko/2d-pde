#include "discretization/geometry.hpp"
#include "discretization/triangulate.hpp"
#include "discretization/mesh_io.hpp"
#include <filesystem>
#include <iostream>
#include <memory>
#include <random>
using namespace discretization;
namespace {
void require(bool value, const std::string &message) {
  if (!value)
    throw std::runtime_error(message);
}
template <class F> void rejects(F f, const std::string &message) {
  bool threw = false;
  try {
    f();
  } catch (const std::exception &) {
    threw = true;
  }
  require(threw, message);
}
double area(const TriangleMesh &m) {
  double sum = 0;
  for (auto t : m.elements) {
    auto a = m.nodes[t[0]], b = m.nodes[t[1]], c = m.nodes[t[2]];
    sum += ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)) / 2;
  }
  return sum;
}
double polygon_area(const Boundary &b) {
  double sum = 0;
  auto a = b.points[0];
  for (std::size_t i = 1; i + 1 < b.points.size(); ++i) {
    auto p = b.points[i], q = b.points[i + 1];
    sum += ((p.x - a.x) * (q.y - a.y) - (p.y - a.y) * (q.x - a.x)) / 2;
  }
  return std::abs(sum);
}
void check_size(const MeshResult &r, double h) {
  require(r.report.size_satisfied, "Unmet size: " + r.report.message);
  for (auto t : r.mesh.elements)
    for (int i = 0; i < 3; ++i)
      require(mesh_detail::distance(r.mesh.nodes[t[i]],
                                    r.mesh.nodes[t[(i + 1) % 3]]) <=
                  h * (1 + 1e-10),
              "Final mesh exceeds edge target");
}
TriangleMesh square() {
  return {{{0, 0}, {1, 0}, {1, 1}, {0, 1}},
          {{0, 1, 2}, {0, 2, 3}},
          {{0, 1}, {1, 2}, {2, 3}, {3, 0}},
          {}};
}
bool has_point(const TriangleMesh &m, Point2 p) {
  return std::any_of(m.nodes.begin(), m.nodes.end(), [&](auto q) {
    return mesh_detail::distance(p, q) < 1e-11;
  });
}
void exact_predicates() {
  require(predicates::orient({0, 0}, {1, 1}, {2, 2}) == 0,
          "Collinearity failed");
  require(predicates::incircle({0, 0}, {1, 0}, {1, 1}, {0, 1}) == 0,
          "Cocircularity failed");
  require(predicates::incircle({0, 0}, {1, 0}, {1, 1}, {.5, .5}) > 0,
          "Incircle failed");
  double tiny = std::numeric_limits<double>::denorm_min();
  require(predicates::orient({0, 0}, {tiny, 0}, {0, tiny}) == 1,
          "Underflow predicate failed");
  require(predicates::orient({1e300, 1e300}, {-1e300, 1e300}, {0, -1e300}) > 0,
          "Overflow predicate failed");
  require(predicates::incircle({0, 0}, {tiny, 0}, {tiny, tiny}, {0, tiny}) == 0,
          "Underflow incircle failed");
  // Independent exact integer determinant for many exactly represented points.
  std::mt19937 random(17);
  std::uniform_int_distribution<int> d(-100, 100);
  for (int i = 0; i < 1000; ++i) {
    long long ax = d(random), ay = d(random), bx = d(random), by = d(random),
              cx = d(random), cy = d(random), dx = d(random), dy = d(random);
    long long x = ax - dx, y = ay - dy, u = bx - dx, v = by - dy, s = cx - dx,
              t = cy - dy;
    long long determinant = (x * x + y * y) * (u * t - v * s) -
                            (u * u + v * v) * (x * t - y * s) +
                            (s * s + t * t) * (x * v - y * u);
    require(predicates::incircle(
                {double(ax), double(ay)}, {double(bx), double(by)},
                {double(cx), double(cy)},
                {double(dx), double(dy)}) == (determinant > 0   ? 1
                                              : determinant < 0 ? -1
                                                                : 0),
            "Integer incircle mismatch");
  }
}
} // namespace
int main(int argc, char **argv) {
  try {
    require(argc == 2, "Usage: test TEMP_DIRECTORY");
    exact_predicates();
    for (double radius : {0.001, 0.01}) {
      auto small = generate_mesh(CircleGeometry(radius));
      require(small.mesh.nodes.size() >= 3, "Small circle failed to form a polygon");
      validate_mesh_geometry(small.mesh);
    }
    auto shifted_sources = discretize_boundary(CircleGeometry{});
    auto &spans = shifted_sources.context->edge_sources;
    std::rotate(spans.begin(), spans.begin() + 1, spans.end());
    for (auto algorithm : {InteriorAlgorithm::EarClipping,
                           InteriorAlgorithm::ConstrainedDelaunay}) {
      MeshOptions options;
      options.algorithm = algorithm;
      rejects([&] { generate_mesh(shifted_sources, options); },
              "Shifted source intervals accepted");
      auto missing = discretize_boundary(CircleGeometry{});
      missing.context->edge_sources.clear();
      rejects([&] { generate_mesh(missing, options); },
              "Missing source intervals accepted");
    }
    const auto path =
        (std::filesystem::path(argv[1]) / "roundtrip.pde2d").string();
    std::vector<std::unique_ptr<CurveGeometry>> shapes;
    shapes.push_back(std::make_unique<CircleGeometry>());
    shapes.push_back(std::make_unique<EllipseGeometry>());
    shapes.push_back(std::make_unique<RectangleGeometry>());
    shapes.push_back(std::make_unique<RectangleWithSemicircleGeometry>());
    shapes.push_back(std::make_unique<RectangleWithSemicircleGeometry>(
        2, 1, SemicircleEnd::NegativeX));
    shapes.push_back(std::make_unique<RegularPolygonGeometry>(5, 1));
    shapes.push_back(std::make_unique<PolygonGeometry>(
        std::vector<Point2>{{0, 0}, {2, 0}, {2, 1}, {1, 1}, {1, 2}, {0, 2}}));
    shapes.push_back(std::make_unique<PolygonGeometry>(
        std::vector<Point2>{{0, 1}, {2, 1}, {2, 0}, {1, 0}, {0, 0}}));
    int combinations = 0;
    for (const auto &geometry : shapes) {
      auto boundary = discretize_boundary(
          *geometry, {.max_edge_length = .4, .max_curve_deviation = .01});
      for (auto algorithm : {InteriorAlgorithm::EarClipping,
                             InteriorAlgorithm::ConstrainedDelaunay}) {
        MeshOptions options;
        options.algorithm = algorithm;
        auto result = generate_mesh(boundary, options);
        validate_mesh_geometry(result.mesh);
        require(result.mesh.elements.size() + 2 ==
                    boundary.boundary.points.size(),
                "Seed triangle count");
        require(std::abs(area(result.mesh) - polygon_area(boundary.boundary)) <
                    1e-10,
                "Seed area changed");
        if (algorithm == InteriorAlgorithm::ConstrainedDelaunay)
          require(is_constrained_delaunay(result.mesh), "Seed is not CDT");
        export_mesh(result.mesh, path);
        auto restored = import_mesh(path);
        require(restored.elements == result.mesh.elements &&
                    restored.boundary_tags == result.mesh.boundary_tags,
                "Raw mesh roundtrip");
        ++combinations;
      }
      for (auto treatment :
           {BoundaryTreatment::Fixed, BoundaryTreatment::SplitPolygonEdges,
            BoundaryTreatment::RefineOnCurves}) {
        MeshOptions options;
        options.boundary_treatment = treatment;
        options.refinement.max_edge_length = .45;
        auto result = generate_mesh(boundary, options);
        check_size(result, .45);
        validate_mesh_geometry(result.mesh);
        require(is_constrained_delaunay(result.mesh),
                "Refined mesh is not CDT");
        if (treatment != BoundaryTreatment::RefineOnCurves)
          require(std::abs(area(result.mesh) -
                           polygon_area(boundary.boundary)) < 1e-10,
                  "Polygon changed during refinement");
        if (treatment == BoundaryTreatment::Fixed)
          require(result.mesh.boundary_edges.size() ==
                      boundary.boundary.points.size(),
                  "Fixed boundary split");
        export_mesh(result, path);
        auto resumed = import_mesh_result(path);
        require(resumed.mesh.elements == result.mesh.elements &&
                    resumed.boundary_context.has_value() &&
                    resumed.report.delaunay_verified,
                "Full-result roundtrip lost context or report");
        auto next_options = options;
        next_options.refinement.max_edge_length = .42;
        auto continued = refine_mesh(resumed, next_options);
        check_size(continued, .42);
        ++combinations;
      }
    }
    std::cout << combinations
              << " shape/algorithm/treatment combinations passed\n";
    CircleGeometry circle;
    circle.splitRegion("boundary", .13, "inlet", "wall");
    auto junction = curvePoint(
        circle.region("wall").curves.empty()
            ? circle.curves()[0].segment
            : circle.curve(circle.region("wall").curves.front()).segment,
        0);
    MeshOptions options;
    options.boundary = {.max_edge_length = .9, .max_curve_deviation = .1};
    auto coarse = generate_mesh(circle, options);
    require(has_point(coarse.mesh, junction), "Region junction missing");
    options.boundary_treatment = BoundaryTreatment::SplitPolygonEdges;
    options.refinement.max_edge_length = .25;
    auto chord = refine_mesh(coarse, options);
    check_size(chord, .25);
    require(std::abs(area(chord.mesh) - area(coarse.mesh)) < 1e-10,
            "Chord policy changed area");
    require(has_point(chord.mesh, junction), "Chord refinement lost junction");
    options.boundary_treatment = BoundaryTreatment::RefineOnCurves;
    auto curved = refine_mesh(coarse, options);
    check_size(curved, .25);
    require(area(curved.mesh) > area(coarse.mesh),
            "Curve refinement did not improve circle approximation");
    for (auto e : curved.mesh.boundary_edges) {
      auto p = curved.mesh.nodes[e.first];
      require(std::abs(std::hypot(p.x, p.y) - 1) < 1e-12,
              "Curve boundary vertex off circle");
    }
    require(has_point(curved.mesh, junction), "Curve refinement lost junction");
    export_mesh(chord, path);
    auto chord_import = import_mesh_result(path);
    auto disabled_curve_options = options;
    disabled_curve_options.refinement.max_edge_length = 0;
    auto disabled_curve = refine_mesh(chord_import, disabled_curve_options);
    require(disabled_curve.mesh.elements == chord.mesh.elements,
            "Disabled refinement changed connectivity");
    export_mesh(disabled_curve, path);
    require(import_mesh_result(path).options.refinement.max_edge_length == 0,
            "Disabled curve transition failed roundtrip");
    auto switched = refine_mesh(chord_import, options);
    check_size(switched, .25);
    for (auto [a, b] : switched.mesh.boundary_edges) {
      (void)b;
      auto p = switched.mesh.nodes[a];
      require(std::abs(std::hypot(p.x, p.y) - 1) < 1e-12,
              "Switch to curves left chord vertices off curve");
    }
    auto reversed = coarse;
    for (auto &c : reversed.boundary_context->curves)
      c.segment = mesh_detail::subcurve(c.segment, 1, 0);
    for (auto &spans : reversed.boundary_context->edge_sources)
      for (auto &s : spans) {
        s.begin = 1 - s.begin;
        s.end = 1 - s.end;
      }
    export_mesh(reversed, path);
    auto reversed_refinement = refine_mesh(import_mesh_result(path), options);
    check_size(reversed_refinement, .25);
    auto malformed_context = coarse;
    malformed_context.boundary_context->edge_sources[0][0].end *= .5;
    rejects([&] { refine_mesh(malformed_context, options); },
            "Missing source interval accepted");
    rejects([&] { export_mesh(malformed_context, path); },
            "Invalid source context exported");
    auto snapshot = coarse;
    circle.setRadius(4);
    auto same = refine_mesh(snapshot, options);
    require(std::abs(area(same.mesh) - area(curved.mesh)) < 1e-10,
            "Snapshot depended on edited geometry");
    RectangleGeometry regions(2, 1);
    regions.splitRegion("bottom", .37, "a", "b");
    auto tagged = generate_mesh(regions);
    options = {};
    options.preserve_regions = false;
    options.boundary.max_edge_length = 0;
    auto untagged = generate_mesh(regions, options);
    require(untagged.mesh.boundary_tags.empty(),
            "Regions disabled retained tags");
    require(untagged.mesh.nodes.size() < tagged.mesh.nodes.size(),
            "Disabled region junction still mandatory");
    require(untagged.boundary_context->protected_vertices.size() == 4,
            "Geometry corners not retained");
    auto relocated = untagged;
    relocated.boundary_context->protected_vertices.clear();
    for (auto &c : relocated.boundary_context->curves) {
      auto &line = std::get<LineCurve>(c.segment);
      line.start.x += 1000;
      line.end.x += 1000;
    }
    rejects([&] { export_mesh(relocated, path); },
            "Mismatched curve context accepted");
    auto missing_corners = untagged;
    missing_corners.boundary_context->protected_vertices.clear();
    rejects([&] { export_mesh(missing_corners, path); },
            "Missing geometric corner anchors accepted");
    options.preserve_regions = true;
    rejects([&] { refine_mesh(untagged, options); },
            "Region preservation enabled without junctions");
    MeshOptions partial{.boundary = {.max_curve_deviation = .001}};
    require(partial.boundary.max_edge_length == 1,
            "Nested default initialization");
    options = {};
    options.boundary = {0, .005};
    auto deviation_only = generate_mesh(CircleGeometry{}, options);
    require(deviation_only.mesh.nodes.size() > 3, "Deviation-only sampling");
    export_mesh(deviation_only, path);
    require(import_mesh_result(path).options.boundary.max_edge_length == 0,
            "Disabled boundary control lost in export");
    options.boundary = {.2, 0};
    require(generate_mesh(CircleGeometry{}, options).mesh.nodes.size() > 3,
            "Length-only sampling");
    options.boundary = {0, 0};
    require(generate_mesh(RectangleGeometry{}, options).mesh.nodes.size() == 4,
            "Disabled line constraints");
    rejects([&] { generate_mesh(CircleGeometry{}, options); },
            "Both curve constraints disabled accepted");
    options = {};
    options.boundary.max_edge_length = -1;
    rejects([&] { generate_mesh(regions, options); },
            "Negative length accepted");
    options = {};
    options.boundary.max_curve_deviation =
        std::numeric_limits<double>::quiet_NaN();
    rejects([&] { generate_mesh(regions, options); }, "NaN accepted");
    options = {};
    options.algorithm = InteriorAlgorithm::EarClipping;
    options.refinement.max_edge_length = .3;
    rejects([&] { generate_mesh(regions, options); },
            "Ear clipping ignored refinement");
    options = {};
    options.refinement.max_edge_length = .2;
    auto conflict = refine_mesh(square(), options);
    require(!conflict.report.size_satisfied &&
                conflict.report.message.find("Fixed") != std::string::npos,
            "Fixed boundary conflict not reported");
    auto false_size = conflict;
    false_size.report.size_satisfied = true;
    rejects([&] { export_mesh(false_size, path); },
            "False size certificate accepted");
    MeshResult false_cdt;
    false_cdt.mesh = square();
    false_cdt.mesh.nodes = {{0, 0}, {2, 0}, {2, 2}, {0, 1}};
    false_cdt.report.delaunay_verified = true;
    rejects([&] { export_mesh(false_cdt, path); },
            "False CDT certificate accepted");
    options.boundary_treatment = BoundaryTreatment::SplitPolygonEdges;
    options.refinement.max_steps = 0;
    auto limited = refine_mesh(square(), options);
    require(!limited.report.size_satisfied, "Budget exhaustion not reported");
    options.refinement.max_steps = 10000;
    auto imported = refine_mesh(square(), options);
    check_size(imported, .2);
    auto subdivided = subdivide_mesh(square(), options);
    check_size(subdivided, .2);
    require(std::abs(area(subdivided.mesh) - 1) < 1e-12,
            "Subdivision changed area");
    options.boundary_treatment = BoundaryTreatment::RefineOnCurves;
    rejects([&] { refine_mesh(square(), options); },
            "Missing source curves accepted");
    // CW boundary, collinear samples and translated coordinates.
    for (bool clockwise : {false, true}) {
      Boundary b{{{0, 0}, {1, 0}, {2, 0}, {2, 1}, {0, 1}}};
      for (auto &p : b.points) {
        p.x += 1e9;
        p.y += 1e9;
      }
      if (clockwise)
        std::reverse(b.points.begin(), b.points.end());
      require(std::abs(area(generate_mesh(b).mesh) - 2) < 1e-12,
              "Translated polygon failed");
    }
    for (const Boundary &b :
         std::vector<Boundary>{{{{0, 0}, {2, 0}, {1, 0}, {1, 1}, {0, 1}}},
                               {{{0, 0}, {2, 2}, {0, 2}, {2, 0}}},
                               {{{0, 0}, {1, 0}, {2, 0}}}})
      rejects([&] { generate_mesh(b); }, "Invalid polygon accepted");
    auto bad = square();
    bad.nodes.push_back({2, 2});
    rejects([&] { refine_mesh(bad); }, "Unused node accepted");
    rejects([&] { export_mesh(bad, path); }, "Unused node exported");
    bad = square();
    bad.elements.push_back(bad.elements[0]);
    rejects([&] { refine_mesh(bad); }, "Duplicate triangle accepted");
    rejects([&] { export_mesh(bad, path); }, "Duplicate triangle exported");
    bad = square();
    bad.boundary_edges.pop_back();
    rejects([&] { refine_mesh(bad); }, "Open boundary accepted");
    rejects([&] { export_mesh(bad, path); }, "Open boundary exported");
    auto serializationResult = generate_mesh(RectangleGeometry{});
    serializationResult.mesh.boundary_tags.clear();
    rejects([&] { export_mesh(serializationResult, path); }, "Missing source tags exported");
    serializationResult = generate_mesh(RectangleGeometry{});
    serializationResult.boundary_context->curves.front().region = "invalid tag";
    serializationResult.options.preserve_regions = false;
    rejects([&] { export_mesh(serializationResult, path); }, "Invalid source name exported");
    bad = square();
    bad.boundary_tags.assign(bad.boundary_edges.size(), "-");
    rejects([&] { export_mesh(bad, path); }, "Reserved boundary tag exported");
    {
      std::ofstream f(path);
      f << "PDE2D_MESH 1 ORDER 1 NODES 3 0 0";
    }
    rejects([&] { import_mesh(path); }, "Truncated file accepted");
    export_mesh(square(), path);
    {
      std::ofstream f(path, std::ios::app);
      f << " EXTRA";
    }
    rejects([&] { import_mesh(path); }, "Trailing data accepted");
    if (std::filesystem::exists("/dev/full"))
      rejects([&] { export_mesh(square(), "/dev/full"); },
              "Write failure ignored");
    validate_mesh_geometry(
        import_mesh("examples/mesh_viewer/rectangle_p1.pde2d"));
    auto thin = generate_mesh(Boundary{{{0, 0}, {1, 0}, {1, 1e-15}}});
    export_mesh(thin, path);
    require(import_mesh_result(path).mesh.elements == thin.mesh.elements,
            "Thin valid triangle failed roundtrip");
    std::cout
        << "Defaults, disabled controls, curve snapshots, region junctions, "
           "robust predicates, refinement and rejection tests passed\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
