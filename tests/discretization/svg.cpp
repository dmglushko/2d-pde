#include "discretization/geometry.hpp"
#include "discretization/svg_geometry.hpp"
#include "discretization/triangulate.hpp"
#include "support/geometry.hpp"
#include "support/check.hpp"
#include <cmath>
#include <fstream>
#include <iostream>
using namespace pde2d::discretization;
using pde2d_test::check;
using pde2d_test::expect_throws;
void svg_import_export(const std::string &dir) {
  std::string file = std::string(dir) + "/test.svg";
  auto write = [&](const std::string &s) {
    std::ofstream f(file);
    f << s;
  };
  write("<svg><circle id='origin' cx='1' cy='1'/><path id='bottom' d='M0 0 L2 "
        "0'/><path id='top' d='M0 2 Q1 3 2 2'/><path id='right' d='M2 2 C3 2 3 "
        "0 2 0'/><path id='left' d='M0 0 A1 1 0 0 0 0 2'/></svg>");
  auto imported = importSvg(file, {2, SvgScaleMode::Factor, 16});
  auto b = discretize_boundary(imported).boundary;
  auto importedId = imported.curveIdAt("top", 0);
  imported.renameRegion("top", "wall");
  imported.moveCurve(importedId, "bottom");
  check(imported.curve(importedId).region == "bottom" &&
        imported.region("bottom").curves.size() == 2);
  auto children = imported.splitCurve(importedId, .5, "a", "b");
  check(children.size() == 2 && children[0] != importedId &&
        children[1] != importedId);
  check(b.points.size() > 4 && b.edge_tags.front() == "bottom" &&
        std::find(b.edge_tags.begin(), b.edge_tags.end(), "right") !=
            b.edge_tags.end());
  check(std::abs(b.points.front().x + 2) < 1e-9 &&
        std::abs(b.points.front().y + 2) < 1e-9);
  MeshOptions options;
  options.boundary_treatment = BoundaryTreatment::RefineOnCurves;
  options.refinement.max_edge_length = 1;
  auto result = generate_mesh(imported, options);
  check(result.report.size_satisfied && result.boundary_context.has_value());
  check(std::find(result.mesh.boundary_tags.begin(),
                  result.mesh.boundary_tags.end(),
                  "a") != result.mesh.boundary_tags.end());
  check(std::find(result.mesh.boundary_tags.begin(),
                  result.mesh.boundary_tags.end(),
                  "b") != result.mesh.boundary_tags.end());
  auto scaled =
      discretize_boundary(importSvg(file, {3, SvgScaleMode::Farthest, 16}))
          .boundary;
  double max = 0;
  for (auto p : scaled.points)
    max = std::max(max, std::hypot(p.x, p.y));
  check(std::abs(max - 3) < 1e-9);
  write("<svg><circle id='origin' cx='1' cy='1'/><path id='a' d='M0 0 l2 0 0 2 "
        "-2 0 z'/></svg>");
  auto near =
      discretize_boundary(importSvg(file, {2, SvgScaleMode::Nearest, 8}))
          .boundary;
  check(std::abs(near.points[0].x + 2) < 1e-9);
  for (const std::string bad :
       {"<svg><path id='x' d='M0 0 H2'/></svg>", "<svg><g/></svg>",
        "<svg><path id='x' d='M0 0 L1 0'/></svg>",
        "<svg><path id='x' transform='scale(2)' d='M0 0 L1 0'/></svg>"}) {
    write(bad);
    expect_throws<std::invalid_argument>([&] { (void)importSvg(file); });
  }

  // Compare analytical parameterizations, not just sampled mesh outlines.
  auto roundtrip = [&](const CurveGeometry &source) {
    exportSvg(source, file);
    auto copy = importSvg(file);
    std::size_t next = 0;
    for (const auto &record : source.curves()) {
      const auto *arc = std::get_if<EllipticalArc>(&record.segment);
      int pieces = arc ? static_cast<int>(std::ceil(std::abs(arc->sweep_angle) /
                                                    (std::numbers::pi / 2)))
                       : 1;
      for (int part = 0; part < pieces; ++part) {
        check(next < copy.curves().size());
        const auto &actual = copy.curves()[next++];
        check(actual.region == record.region &&
              actual.segment.index() == record.segment.index());
        for (double t : {0., .17, .5, .81, 1.}) {
          auto p = curvePoint(record.segment, (part + t) / pieces);
          double c = std::cos(source.rotation()),
                 s = std::sin(source.rotation());
          Point2 expected{source.center().x + c * p.x - s * p.y,
                          source.center().y + s * p.x + c * p.y};
          check(
              pde2d_test::points_near(expected, curvePoint(actual.segment, t)));
        }
      }
    }
    check(next == copy.curves().size());
    check(copy.center().x == 0 && copy.center().y == 0 && copy.rotation() == 0);
    return copy;
  };
  CurveGeometry mixed(
      {{"bottom", {LineCurve{{0, 0}, {2, 0}}}},
       {"right", {QuadraticBezier{{2, 0}, {3, 1}, {2, 2}}}},
       {"top", {CubicBezier{{2, 2}, {1.8, 3}, {.2, 3}, {0, 2}}}},
       {"left",
        {EllipticalArc{
            {0, 1}, 1, 1, 0, std::numbers::pi / 2, std::numbers::pi}}}});
  // The left arc runs from (0,2) through (-1,1) to (0,0).
  mixed.setCenter(7, -4);
  mixed.setRotation(.71);
  (void)roundtrip(mixed);
  std::ostringstream text;
  exportSvg(mixed, text);
  check(text.str().find("data-region") == std::string::npos);
  check(text.str().find("transform=") == std::string::npos);
  check(text.str().find(" Q ") != std::string::npos &&
        text.str().find(" C ") != std::string::npos);
  struct CommaDecimal : std::numpunct<char> {
    char do_decimal_point() const override { return ','; }
  };
  auto oldLocale = std::locale();
  std::locale::global(std::locale(oldLocale, new CommaDecimal));
  std::ostringstream localized;
  exportSvg(mixed, localized);
  std::locale::global(oldLocale);
  check(localized.str() == text.str());
  // Both sweep directions, non-circular ellipses, and full circles.
  for (double ry : {1., 2.})
    for (double sweep : {2 * std::numbers::pi, -2 * std::numbers::pi}) {
      CurveGeometry full(
          {{"origin", {EllipticalArc{{1, 2}, 2, ry, .37, .21, sweep}}}});
      full.setCenter(-3, 5);
      full.setRotation(-.63);
      auto copy = roundtrip(full);
      check(copy.curves().size() == 4 && copy.region("origin").connected());
      check(!discretize_boundary(copy).boundary.points.empty());
    }
  CurveGeometry largeRotation(
      {{"ellipse",
        {EllipticalArc{{0, 0}, 2, 1, 1e20, .3, 2 * std::numbers::pi}}}});
  largeRotation.setRotation(-1e20);
  (void)roundtrip(largeRotation);
  // Edits create disconnected membership, generated-ID collisions, names
  // invalid as XML IDs, and a run crossing the stored boundary seam.
  CurveGeometry edited({{"pde2d_run_1", {LineCurve{{0, 0}, {2, 0}}}},
                        {"right", {LineCurve{{2, 0}, {2, 2}}}},
                        {"top", {LineCurve{{2, 2}, {0, 2}}}},
                        {"left", {LineCurve{{0, 2}, {0, 0}}}}});
  edited.renameRegion("right", "9wall");
  edited.mergeRegionInto("left", "9wall");
  auto children2 =
      edited.splitCurve(edited.curveIdAt("top", 0), .4, "origin", "9wall");
  edited.moveCurve(children2[0], "pde2d_run_1");
  auto restored = roundtrip(edited);
  check(!restored.region("9wall").connected());
  restored.mergeRegionInto("9wall", "pde2d_run_1");
  (void)roundtrip(restored);
  // Explicit replacement, including a region whose pieces straddle the seam.
  edited.setRegions({{"same", {LineCurve{{0, 0}, {1, 0}}}},
                     {"other", {LineCurve{{1, 0}, {0, 1}}}},
                     {"same", {LineCurve{{0, 1}, {0, 0}}}}});
  check(roundtrip(edited).region("same").connected());
  CurveSegment closedBezier = CubicBezier{{0, 0}, {2, 0}, {0, 2}, {0, 0}};
  CurveGeometry closed({{"loop", {closedBezier}}});
  exportSvg(closed, file);
  auto closedCopy = importSvg(file);
  check(closedCopy.curves().size() == 2);
  for (std::size_t i = 0; i < 2; ++i)
    for (double t : {0., .3, .7, 1.})
      check(
          pde2d_test::points_near(curvePoint(closedCopy.curves()[i].segment, t),
                                  curvePoint(closedBezier, (i + t) / 2)));
  for (const std::string bad :
       {"<svg><path id='x' data-region='' d='M0 0 L1 0 L0 1 Z'/></svg>",
        "<svg><path id='x' data-region='a' d='M0 0 L1 0'/><path id='x' "
        "data-region='b' d='M1 0 L0 0'/></svg>",
        "<svg><path id='x' data-region='bad&amp;name' d='M0 0 L1 0 L0 1 "
        "Z'/></svg>"}) {
    write(bad);
    expect_throws<std::invalid_argument>([&] { (void)importSvg(file); });
  }
  auto invalidExport = [&](const CurveGeometry &g) {
    write("keep existing file");
    expect_throws<std::invalid_argument>([&] { exportSvg(g, file); });
    std::ifstream input(file);
    std::string saved;
    std::getline(input, saved);
    check(saved == "keep existing file");
  };
  invalidExport(CurveGeometry{});
  invalidExport(CurveGeometry({{"open", {LineCurve{{0, 0}, {1, 0}}}}}));
  invalidExport(CurveGeometry(
      {{"disconnected",
        {LineCurve{{0, 0}, {1, 0}}, LineCurve{{2, 0}, {0, 0}}}}}));
  invalidExport(CurveGeometry({{"zero", {LineCurve{{0, 0}, {0, 0}}}}}));
  auto huge = mixed;
  huge.setCenter(1e300, 1e300);
  invalidExport(huge);
  expect_throws<std::runtime_error>(
      [&] { exportSvg(mixed, std::string(dir)); });
  std::ostringstream broken;
  broken.setstate(std::ios::badbit);
  expect_throws<std::runtime_error>([&] { exportSvg(mixed, broken); });
  std::ostringstream untouched;
  try {
    exportSvg(CurveGeometry{}, untouched);
  } catch (const std::invalid_argument &) {
  }
  check(untouched.str().empty());
  // A real short write must also be reported, including stream exceptions.
  struct FailedBuffer : std::streambuf {
    std::streamsize xsputn(const char *, std::streamsize) override { return 0; }
  } buffer;
  std::ostream shortWrite(&buffer);
  expect_throws<std::runtime_error>([&] { exportSvg(mixed, shortWrite); });
  shortWrite.clear();
  shortWrite.exceptions(std::ios::badbit);
  expect_throws<std::ios_base::failure>([&] { exportSvg(mixed, shortWrite); });
}
// Several closed subpaths or paths: the loop of largest area is the outer
// loop, the others are holes in file order.
void svg_holes(const std::string &dir) {
  std::string file = std::string(dir) + "/holes.svg";
  auto write = [&](const std::string &s) {
    std::ofstream f(file);
    f << s;
  };
  // A hole first in the file, then the outer square as two subpaths of one
  // path that close into one loop, then a circular hole from two arcs.
  write("<svg><path id='hole' d='M4 4 L6 4 L6 6 L4 6 Z'/>"
        "<path id='wall' d='M0 0 L10 0 L10 10 M10 10 L0 10 L0 0'/>"
        "<path id='round' d='M8 2 A0.5 0.5 0 0 1 8 3 A0.5 0.5 0 0 1 8 "
        "2'/></svg>");
  auto g = importSvg(file);
  check(g.loopCount() == 3, "Outer loop plus two holes");
  check(g.curves().front().region == "wall" && g.curves().front().loop == 0,
        "Largest loop is the outer loop");
  std::size_t loop_of_hole = 0, loop_of_round = 0;
  for (const auto &c : g.curves()) {
    if (c.region == "hole")
      loop_of_hole = c.loop;
    if (c.region == "round")
      loop_of_round = c.loop;
  }
  check(loop_of_hole == 1 && loop_of_round == 2, "Holes keep file order");
  MeshOptions options;
  options.boundary.max_edge_length = 0.5;
  options.refinement.max_edge_length = 0.6;
  auto mesh = generate_mesh(g, options);
  check(mesh.report.size_satisfied, mesh.report.message);
  check(boundary_loops(mesh.mesh).size() == 3);
  // Export writes one closed chain per loop; import recovers the loops.
  AnnulusGeometry annulus(2, 0.75);
  annulus.setCenter(1, -1);
  exportSvg(annulus, file);
  auto copy = importSvg(file);
  check(copy.loopCount() == 2 && copy.region("inner").curves.size() == 4 &&
            copy.region("outer").curves.size() == 4,
        "Full circles export as four arcs per loop");
  for (const auto &c : copy.curves()) {
    auto p = curvePoint(c.segment, 0.3);
    double r = std::hypot(p.x - 1, p.y + 1);
    check(std::abs(r - (c.loop == 0 ? 2 : 0.75)) < 1e-9, "Radius per loop");
  }
  // A region spanning loops exports with generated IDs and data-region.
  CurveGeometry shared(
      {{"wall",
        {LineCurve{{0, 0}, {4, 0}}, LineCurve{{4, 0}, {4, 4}},
         LineCurve{{4, 4}, {0, 4}}, LineCurve{{0, 4}, {0, 0}}}}},
      {{{"wall",
         {LineCurve{{1, 1}, {2, 1}}, LineCurve{{2, 1}, {1, 2}},
          LineCurve{{1, 2}, {1, 1}}}}}});
  exportSvg(shared, file);
  auto reread = importSvg(file);
  check(reread.loopCount() == 2 && reread.region("wall").pieces.size() == 2);
  // An open loop next to a closed one is still rejected.
  for (const std::string bad :
       {"<svg><path id='a' d='M0 0 L4 0 L4 4 Z'/><path id='b' d='M1 1 L2 "
        "1'/></svg>",
        "<svg><path id='a' d='M0 0 L4 0 L4 4 Z M1 1'/></svg>",
        "<svg><path id='a' d='M0 0 L4 0 L4 4 Z L1 1'/></svg>"}) {
    write(bad);
    expect_throws<std::invalid_argument>([&] { (void)importSvg(file); });
  }
  // Nesting is validated when the boundary is sampled.
  write("<svg><path id='a' d='M0 0 L4 0 L4 4 L0 4 Z M5 5 L6 5 L6 6 Z'/></svg>");
  auto outside = importSvg(file);
  expect_throws<std::invalid_argument>(
      [&] { (void)discretize_boundary(outside); },
      "Hole is not inside the outer boundary");
}
int main(int argc, char **argv) {
  return pde2d_test::run(
      {{"svg_import_export",
        [&] {
          pde2d_test::check(argc == 2, "Usage: test TEMP_DIRECTORY");
          svg_import_export(argv[1]);
        }},
       {"svg_holes", [&] {
          pde2d_test::check(argc == 2, "Usage: test TEMP_DIRECTORY");
          svg_holes(argv[1]);
        }}});
}
