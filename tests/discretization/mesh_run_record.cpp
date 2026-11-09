// Mesh run records (MeshResult::record, mesh_record.hpp,
// mesh_record_json.hpp). Oracles: values known from the call (operation,
// options, the fingerprints of the produced and the refined mesh computed
// separately, the library version), quality statistics recomputed here with
// a different formula (law of cosines instead of atan2) and the size
// guarantee of a satisfied report, work-counter relations that follow from
// the algorithms (every refinement step inserts one vertex, every curve
// split rebuilds, ear clipping does no work), two identical calls (records
// equal except the wall-clock time) and the JSON round trip.
#include "discretization/geometry.hpp"
#include "discretization/mesh_fingerprint.hpp"
#include "discretization/mesh_io.hpp"
#include "discretization/mesh_record.hpp"
#include "discretization/mesh_record_json.hpp"
#include "discretization/triangulate.hpp"
#include "support/check.hpp"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <numbers>
#include <sstream>
#include <string>

using namespace pde2d::discretization;
using pde2d_test::check;
using pde2d_test::expect_throws;

namespace {
MeshOptions refined(double h) {
  MeshOptions o;
  o.boundary.max_edge_length = h;
  o.refinement.max_edge_length = h;
  return o;
}
std::string json(const MeshRunRecord &r) {
  std::ostringstream out;
  write_json(out, r);
  return out.str();
}
// Equal except total_seconds.
bool same_record(MeshRunRecord a, MeshRunRecord b) {
  a.total_seconds = b.total_seconds = 0;
  return json(a) == json(b);
}

void generate_record() {
  const RectangleGeometry square(1, 1);
  const auto options = refined(.2);
  const auto r = generate_mesh(square, options);
  check(r.report.size_satisfied, r.report.message);
  const auto &rec = r.record;
  check(rec.operation == MeshOperation::Generate, "operation");
  check(rec.fingerprint == mesh_fingerprint(r.mesh), "fingerprint");
  check(rec.source_fingerprint.empty(), "generation has no source mesh");
  check(rec.library == pde2d::library_version(), "library version");
  check(rec.options.refinement.max_edge_length == .2 &&
            rec.options.boundary.max_edge_length == .2,
        "options of the call");
  // Every refinement step inserts one vertex into the seed polygon (the
  // Fixed boundary keeps the sampled vertices).
  const auto &w = rec.work;
  check(w.refinement_steps == r.report.refinement_steps &&
            w.refinement_steps > 0 && w.inserted_vertices == w.refinement_steps,
        "one vertex per refinement step");
  check(r.mesh.nodes.size() ==
            r.mesh.boundary_edges.size() + w.inserted_vertices,
        "nodes = boundary vertices + inserted vertices");
  check(w.boundary_splits == 0 && w.curved_rebuilds == 0,
        "a Fixed boundary is never split");
  check(w.edge_flips > 0, "Delaunay legalization flips edges");
  check(same_record(rec, generate_mesh(square, options).record),
        "identical calls: identical records except the time");
}

void quality_statistics() {
  const auto r = generate_mesh(CircleGeometry(1), refined(.25));
  const auto &q = r.record.quality;
  double min_angle = 180, max_angle = 0, max_length = 0;
  for (const auto &t : r.mesh.elements) {
    double len[3];
    for (int k = 0; k < 3; ++k) {
      const auto a = r.mesh.nodes[t[(k + 1) % 3]],
                 b = r.mesh.nodes[t[(k + 2) % 3]];
      len[k] = std::hypot(a.x - b.x, a.y - b.y); // opposite vertex k
      max_length = std::max(max_length, len[k]);
    }
    for (int k = 0; k < 3; ++k) {
      const double b = len[(k + 1) % 3], c = len[(k + 2) % 3];
      const double angle =
          std::acos(std::clamp((b * b + c * c - len[k] * len[k]) / (2 * b * c),
                               -1.0, 1.0)) *
          180 / std::numbers::pi;
      min_angle = std::min(min_angle, angle);
      max_angle = std::max(max_angle, angle);
    }
  }
  check(q.nodes == r.mesh.nodes.size() &&
            q.elements == r.mesh.elements.size() &&
            q.boundary_edges == r.mesh.boundary_edges.size(),
        "counts");
  check(std::abs(q.min_angle_degrees - min_angle) < 1e-9 &&
            std::abs(q.max_angle_degrees - max_angle) < 1e-9,
        "angles agree with the law of cosines");
  check(q.min_angle_degrees > 0 && q.min_angle_degrees <= 60 &&
            q.max_angle_degrees >= 60 && q.max_angle_degrees < 180,
        "angle bounds of a triangle mesh");
  check(std::abs(q.max_edge_length - max_length) < 1e-15, "longest edge");
  check(q.max_target_ratio && *q.max_target_ratio <= 1 + 1e-12 &&
            std::abs(*q.max_target_ratio - max_length / .25) < 1e-12,
        "a satisfied constant target bounds the length ratio");

  // A size field: the ratio uses the local target; satisfied means <= 1.
  auto graded = refined(.3);
  graded.refinement.max_edge_length = 0;
  graded.boundary_treatment = BoundaryTreatment::SplitPolygonEdges;
  graded.refinement.size_field = [](Point2 p) { return .05 + .2 * p.x * p.x; };
  graded.refinement.size_field_descriptor =
      pde2d::StrategyDescriptor{"Quadratic", {{"base", ".05"}}};
  const auto g = generate_mesh(RectangleGeometry(1, 1), graded);
  check(g.report.size_satisfied, g.report.message);
  check(g.record.quality.max_target_ratio &&
            *g.record.quality.max_target_ratio <= 1 + 1e-12 &&
            *g.record.quality.max_target_ratio > .5,
        "size field target ratio");
  const auto text = json(g.record);
  check(text.find("\"Quadratic\"") != std::string::npos,
        "the size field is recorded by its descriptor");

  // A user criterion has no target: no ratio, its descriptor is recorded.
  auto by_criterion = refined(.3);
  by_criterion.refinement.max_edge_length = 0;
  RefinementCriterion c;
  c.size = constant_size_criterion(.2);
  c.descriptor = pde2d::StrategyDescriptor{"Constant", {{"h", ".2"}}};
  by_criterion.refinement.criterion = c;
  const auto u = generate_mesh(RectangleGeometry(1, 1), by_criterion);
  check(!u.record.quality.max_target_ratio, "a criterion has no target");
  check(json(u.record).find("\"criterion_descriptor\": {") != std::string::npos,
        "criterion descriptor recorded");
}

void refinement_operations() {
  const auto coarse = generate_mesh(RectangleGeometry(1, 1), refined(.5));
  auto split_edges = refined(.25);
  split_edges.boundary_treatment = BoundaryTreatment::SplitPolygonEdges;
  const auto finer = refine_mesh(coarse, split_edges);
  check(finer.report.size_satisfied, finer.report.message);
  check(finer.record.operation == MeshOperation::Refine &&
            finer.record.source_fingerprint == mesh_fingerprint(coarse.mesh) &&
            finer.record.fingerprint == mesh_fingerprint(finer.mesh),
        "refine: source and result fingerprints");
  check(finer.record.work.refinement_steps == finer.report.refinement_steps &&
            finer.record.work.boundary_splits > 0,
        "refine: steps, chord splits");
  const auto marked = refine_mesh(coarse, {0, 1}, coarse.options);
  check(marked.record.operation == MeshOperation::MarkedRefine &&
            marked.record.source_fingerprint == mesh_fingerprint(coarse.mesh),
        "marked refinement");
  check(marked.record.work.inserted_vertices ==
            marked.mesh.nodes.size() - coarse.mesh.nodes.size(),
        "marked refinement inserts one vertex per bisection");
  const auto split = subdivide_mesh(coarse.mesh, split_edges);
  check(split.record.operation == MeshOperation::Subdivide &&
            split.record.work.refinement_steps > 0 &&
            split.record.work.edge_flips == 0,
        "subdivision does not legalize");

  // Ear clipping: the seed triangulation, no work.
  MeshOptions ear;
  ear.algorithm = InteriorAlgorithm::EarClipping;
  const auto e = generate_mesh(RectangleGeometry(1, 1), ear);
  check(e.record.work == MeshWork{}, "ear clipping does no refinement work");
  check(!e.record.quality.max_target_ratio, "no target");

  // Curves: every boundary split rebuilds the triangulation.
  auto curved = refined(.5);
  curved.boundary_treatment = BoundaryTreatment::RefineOnCurves;
  const auto disk = generate_mesh(CircleGeometry(1), curved);
  auto finer_curved = curved;
  finer_curved.refinement.max_edge_length = .2;
  const auto rebuilt = refine_mesh(disk, finer_curved);
  const auto &w = rebuilt.record.work;
  check(w.boundary_splits > 0 && w.curved_rebuilds >= w.boundary_splits,
        "every curve split rebuilds");
  check(w.reinserted_vertices > 0, "rebuilds reinsert interior vertices");
}

void import_and_json() {
  const auto r = generate_mesh(RectangleGeometry(1, 1), refined(.3));
  const auto path =
      (std::filesystem::temp_directory_path() / "pde2d_mesh_run_record.pde2d")
          .string();
  export_mesh(r, path);
  const auto back = import_mesh_result(path);
  std::filesystem::remove(path);
  check(back.record.operation == MeshOperation::Import &&
            back.record.fingerprint == r.record.fingerprint &&
            back.record.work == MeshWork{} &&
            back.record.quality == r.record.quality,
        "import: fingerprint and quality of the same mesh, no work");

  const auto text = json(r.record);
  std::istringstream in(text);
  const auto document = read_mesh_run_record_json(in);
  check(document.at("operation").text == "generate" &&
            document.at("fingerprint").text == r.record.fingerprint,
        "keys");
  check(document.at("work").at("edge_flips").number() ==
            static_cast<double>(r.record.work.edge_flips),
        "counters");
  check(document.at("quality").at("min_angle_degrees").number() ==
            r.record.quality.min_angle_degrees,
        "numbers round-trip exactly");
  check(pde2d::json_text(document) == text, "byte-identical re-serialization");
  std::istringstream other("{\"schema\": \"pde2d.run_record\"}");
  expect_throws<std::invalid_argument>(
      [&] { (void)read_mesh_run_record_json(other); }, "Not a mesh run record");
  auto newer = text;
  newer.replace(newer.find("\"schema_version\": 1"), 19,
                "\"schema_version\": 2");
  std::istringstream future(newer);
  expect_throws<std::invalid_argument>(
      [&] { (void)read_mesh_run_record_json(future); },
      "Unsupported mesh run record schema version");
}
} // namespace

int main() {
  return pde2d_test::run({{"generate_record", generate_record},
                          {"quality_statistics", quality_statistics},
                          {"refinement_operations", refinement_operations},
                          {"import_and_json", import_and_json}});
}
