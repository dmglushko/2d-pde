// mesh_fingerprint identifies a mesh exactly. Oracles: invariants (copies
// agree; any change of a coordinate bit, an index, an edge or a tag changes
// it) and a pinned value, which must be the same on every platform and
// build because the encoding is explicit (provenance.hpp).
#include "discretization/mesh_fingerprint.hpp"
#include "discretization/square_mesh.hpp"
#include "support/check.hpp"
#include <cmath>
#include <utility>

using namespace pde2d::discretization;
using pde2d_test::check;
namespace {
void identifies_the_arrays() {
  const auto mesh = square_mesh(2);
  const auto base = mesh_fingerprint(mesh);
  check(base.size() == 16, "16 hex digits");
  check(mesh_fingerprint(square_mesh(2)) == base, "Copies agree");
  check(mesh_fingerprint(square_mesh(3)) != base, "Other mesh");

  auto moved = mesh;
  moved.nodes[4].x = std::nextafter(moved.nodes[4].x, 2.0);
  check(mesh_fingerprint(moved) != base, "One ulp moves the fingerprint");
  auto renumbered = mesh;
  std::swap(renumbered.elements[0][0], renumbered.elements[0][1]);
  check(mesh_fingerprint(renumbered) != base, "Element orientation");
  auto edge = mesh;
  std::swap(edge.boundary_edges[0].first, edge.boundary_edges[0].second);
  check(mesh_fingerprint(edge) != base, "Boundary edge direction");
  auto tags = mesh;
  tags.boundary_tags[0] = "other";
  check(mesh_fingerprint(tags) != base, "Boundary tags");
  auto untagged = mesh;
  untagged.boundary_tags.clear();
  check(mesh_fingerprint(untagged) != base, "Missing tags");
}

void pinned_value() {
  TriangleMesh mesh;
  mesh.nodes = {{0, 0}, {1, 0}, {0, 1}};
  mesh.elements = {{0, 1, 2}};
  mesh.boundary_edges = {{0, 1}, {1, 2}, {2, 0}};
  mesh.boundary_tags = {"a", "b", "c"};
  // Pinned when the encoding was introduced; a change here changes every
  // stored run record, so it needs a new encoding tag ("pde2d.mesh.v2").
  check(mesh_fingerprint(mesh) == "ba0173754791adc3", "Encoding changed");
}
} // namespace
int main() {
  return pde2d_test::run({{"identifies_the_arrays", identifies_the_arrays},
                          {"pinned_value", pinned_value}});
}
