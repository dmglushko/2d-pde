#include "discretization/geometry.hpp"
#include "discretization/mesh_io.hpp"
#include <chrono>
#include <iostream>
#include <memory>
using namespace pde2d::discretization;
using Clock = std::chrono::steady_clock;
template <class F> double timed(F f) {
  auto t = Clock::now();
  f();
  return std::chrono::duration<double, std::milli>(Clock::now() - t).count();
}
int main(int argc, char **argv) {
  if (argc == 2) {
    auto m = import_mesh(argv[1]);
    mesh_detail::Topology t(m);
    std::vector<mesh_detail::Box> boxes;
    for (const auto &[e, own] : t.owners) {
      (void)own;
      boxes.push_back(
          mesh_detail::Box::segment(m.nodes[e.first], m.nodes[e.second]));
    }
    mesh_detail::BoxIndex index(boxes);
    std::size_t pairs = 0;
    for (std::size_t i = 0; i < boxes.size(); ++i)
      for (auto j : index.candidates(boxes[i]))
        if (j > i)
          ++pairs;
    std::cout << "nodes,triangles,edges,all_pairs,broadphase_pairs\n"
              << m.nodes.size() << ',' << m.elements.size() << ','
              << boxes.size() << ',' << boxes.size() * (boxes.size() - 1) / 2
              << ',' << pairs << '\n';
    return 0;
  }
  std::cout << "shape,h,repeat,policy,boundary,nodes,triangles,steps,sampling_"
               "ms,seed_ms,refine_ms,validation_ms,context_ms,io_ms\n";
  for (int shape = 0; shape < 4; ++shape)
    for (double h : {.3, .15, .075})
      for (int repeat = 0; repeat < 3; ++repeat) {
        std::unique_ptr<CurveGeometry> g;
        if (shape == 0)
          g = std::make_unique<RectangleGeometry>();
        if (shape == 1)
          g = std::make_unique<PolygonGeometry>(std::vector<Point2>{
              {-1, -1}, {1, -1}, {1, 0}, {0, 0}, {0, 1}, {-1, 1}});
        if (shape == 2)
          g = std::make_unique<CircleGeometry>();
        if (shape == 3) {
          auto p = std::make_unique<RectangleGeometry>();
          p->splitRegion("bottom", {.3, .7}, {"wall", "inlet", "wall"});
          g = std::move(p);
        }
        for (auto policy :
             {BoundaryTreatment::Fixed, BoundaryTreatment::SplitPolygonEdges,
              BoundaryTreatment::RefineOnCurves}) {
          MeshOptions o;
          o.boundary_treatment = policy;
          o.boundary.max_edge_length =
              policy == BoundaryTreatment::Fixed ? h : .5;
          o.boundary.max_curve_deviation = .01;
          o.refinement.max_edge_length = h;
          DiscretizedBoundary b;
          TriangleMesh seed;
          MeshResult r;
          double sampling =
              timed([&] { b = discretize_boundary(*g, o.boundary, true); });
          double seeding =
              timed([&] { seed = mesh_detail::seed_mesh(b.boundary); });
          double refining = timed([&] { r = refine_mesh(seed, o, b.context); });
          double validation = timed([&] { validate_mesh_geometry(r.mesh); });
          double context = timed([&] {
            validate_boundary_context(r.mesh, *r.boundary_context, true);
          });
          double io = timed([&] {
            export_mesh(r, "/tmp/pde2d-mesh-issue3/benchmark-roundtrip.pde2d");
            auto q = import_mesh_result(
                "/tmp/pde2d-mesh-issue3/benchmark-roundtrip.pde2d");
            (void)q;
          });
          std::cout << shape << ',' << h << ',' << repeat << ',' << int(policy)
                    << ',' << b.boundary.points.size() << ','
                    << r.mesh.nodes.size() << ',' << r.mesh.elements.size()
                    << ',' << r.report.refinement_steps << ',' << sampling
                    << ',' << seeding << ',' << refining << ',' << validation
                    << ',' << context << ',' << io << '\n'
                    << std::flush;
        }
      }
}
