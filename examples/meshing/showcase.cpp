// Reproduce the geometric gallery; no external meshing dependencies.
#include "discretization/geometry.hpp"
#include "discretization/triangulate.hpp"
#include "discretization/mesh_io.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>

using namespace pde2d::discretization;

int main(int argc, char **argv) {
  try {
    const std::filesystem::path output = argc > 1 ? argv[1] : "docs/showcase";
    std::filesystem::create_directories(output);
    std::ofstream manifest(output / "manifest.tsv");
    manifest << "file\ttitle\tnodes\ttriangles\tsize_satisfied\tmessage\n";
    auto save = [&](const std::string &name, const std::string &title,
                    const CurveGeometry &geometry, const MeshOptions &options,
                    bool expect_size = true) {
      auto result = generate_mesh(geometry, options);
      if (result.mesh.nodes.size() >= 500)
        throw std::runtime_error(name + ": showcase exceeds vertex budget");
      if (result.report.size_satisfied != expect_size)
        throw std::runtime_error(
            name + ": unexpected size report: " + result.report.message);
      export_mesh(result, (output / (name + ".pde2d")).string());
      manifest << name << ".pde2d\t" << title << '\t'
               << result.mesh.nodes.size() << '\t'
               << result.mesh.elements.size() << '\t'
               << result.report.size_satisfied << '\t'
               << (result.report.message.empty() ? "-" : result.report.message)
               << '\n';
      std::cout << name << ": " << result.mesh.nodes.size() << " vertices\n";
    };
    std::vector<std::pair<std::string, std::unique_ptr<CurveGeometry>>> shapes;
    shapes.emplace_back("circle", std::make_unique<CircleGeometry>());
    shapes.emplace_back("ellipse", std::make_unique<EllipseGeometry>());
    shapes.emplace_back("rectangle", std::make_unique<RectangleGeometry>());
    shapes.emplace_back("capped_rectangle",
                        std::make_unique<RectangleWithSemicircleGeometry>());
    shapes.emplace_back("regular_polygon",
                        std::make_unique<RegularPolygonGeometry>());
    shapes.emplace_back(
        "concave_polygon",
        std::make_unique<PolygonGeometry>(std::vector<Point2>{
            {-1, -1}, {1, -1}, {1, 0}, {0, 0}, {0, 1}, {-1, 1}}));
    // Default boundary settings; no interior target. Compare the two
    // algorithms.
    for (const auto &[name, shape] : shapes) {
      MeshOptions options;
      options.algorithm = InteriorAlgorithm::EarClipping;
      save(name + "_ear", name + " | ear clipping", *shape, options);
      options.algorithm = InteriorAlgorithm::ConstrainedDelaunay;
      save(name + "_cdt", name + " | CDT", *shape, options);
    }
    CircleGeometry circle;
    MeshOptions fixed;
    fixed.boundary.max_edge_length = 0.25;
    fixed.boundary.max_curve_deviation = 0.005;
    fixed.refinement.max_edge_length = 0.4;
    save("circle_fixed_fine", "Fixed | fine initial boundary", circle, fixed);
    MeshOptions coarse;
    coarse.boundary.max_edge_length = 1.0;
    coarse.boundary.max_curve_deviation = 0.15;
    save("circle_coarse", "Coarse boundary | no interior target", circle,
         coarse);
    coarse.refinement.max_edge_length = 0.4;
    save("circle_fixed_conflict", "Fixed | size target unmet", circle, coarse,
         false);
    coarse.boundary_treatment = BoundaryTreatment::SplitPolygonEdges;
    save("circle_chords", "Chord splits | original polygon retained", circle,
         coarse);
    coarse.boundary_treatment = BoundaryTreatment::RefineOnCurves;
    save("circle_curves", "Curve splits | new points on circle", circle,
         coarse);
    RectangleGeometry regions;
    regions.splitRegion("bottom", {0.3, 0.7}, {"wall", "inlet", "wall"});
    MeshOptions tagged;
    tagged.boundary.max_edge_length =
        0.0; // Isolate protection of region junctions.
    save("regions_on", "Regions preserved | inlet endpoints retained", regions,
         tagged);
    tagged.preserve_regions = false;
    save("regions_off", "Regions disabled | redundant junctions removable",
         regions, tagged);
    if (!manifest)
      throw std::runtime_error("Could not write manifest");
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
