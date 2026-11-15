#pragma once

#include "pde2d/mesh/mesh.hpp"
#include "pde2d/mesh/report.hpp"

#include <array>
#include <cstddef>
#include <span>
#include <string>
#include <vector>

// Refinement: the third part of adaptive refinement (estimate -> mark ->
// refine). A refinement method takes a mesh and its marked triangles and
// returns a finer conforming mesh with the parent of every new triangle
// and node (the seam for transferring a solution later).
//
// THE REFINEMENT EDGE CONVENTION. Every triangle has a refinement edge:
// the edge opposite its vertex 0 (Triangle::nodes[0], the "newest
// vertex"). Both methods here read it from the vertex order and write it
// into the vertex order of the triangles they create, so it persists from
// one refinement to the next with no extra field in mesh::Mesh. The
// vertex order stays counterclockwise; only which vertex comes first
// carries the information. Code that rebuilds or edits a Mesh's triangles
// (flips, the Remesher, a Gmsh import that makes triangles
// counterclockwise) chooses vertex 0 freely, which keeps every mesh valid
// for refinement but loses the shape guarantee below; call
// set_longest_edges_as_refinement_edges() again after such a change.
//
// Shape guarantee (for triangles away from curved boundaries): the
// triangles created by repeated refinement, with either method, fall into
// at most four similarity classes per triangle of the mesh the
// refinement edges were set on, so their angles stay bounded below.
namespace pde2d::adapt {

// What a refinement did.
struct RefinementResult {
  // The refined mesh: valid and conforming, with the input's region names
  // and chains. Every input node keeps its index; new nodes follow.
  // Triangles are listed by parent: the children of input triangle 0
  // first (or triangle 0 itself if it was not refined), then those of
  // triangle 1, and so on. A boundary edge that was split is replaced, in
  // place, by its two halves (same region, the link's arc-length interval
  // halved), so loops stay in walking order.
  mesh::Mesh mesh;
  // For each triangle of `mesh`, the input triangle it came from (itself
  // if it was not refined). A child of a triangle with a curved boundary
  // edge may stick out of its parent; it still names that parent.
  std::vector<std::size_t> triangle_parents;
  // For each node of `mesh`: {v, v} for input node v, {a, b} (a < b) for a
  // new node that splits the input edge a-b. A new node is the midpoint of
  // a-b, except on a boundary edge with a curve link, where it is the
  // curve point at the middle of the link's arc-length interval. So
  // (u[a] + u[b]) / 2 interpolates a P1 function u to every node.
  std::vector<std::array<std::size_t, 2>> node_parents;
  // stage = the method's name(), time, and the metrics
  //   "marked"             distinct marked triangles,
  //   "refined_by_closure" triangles refined though not marked,
  //   "new_nodes"          nodes added,
  //   "new_triangles"      triangles added (output count minus input count).
  // A message says when nothing was marked.
  mesh::StageReport report;
};

// The common interface of the refinement methods. (Not mesh::Refiner,
// which refines towards a MeshSpec inside the Mesher.)
class RefinementMethod {
public:
  virtual ~RefinementMethod() = default;
  virtual std::string name() const = 0;

  // Refines every marked triangle (indices into mesh.triangles, any order,
  // repetitions allowed) and as many others as needed to keep the mesh
  // conforming. Nothing marked returns the mesh unchanged. Deterministic.
  // Throws std::invalid_argument for an invalid mesh (mesh::validate) or
  // a marked index out of range, and std::runtime_error if a curve point
  // of a linked boundary edge cannot be placed by a plain split of its
  // triangle (it would invert a triangle or overlap another part of the
  // mesh; the boundary split policy would have to re-triangulate, which
  // refinement does not do), naming the edge and the reason.
  [[nodiscard]] virtual RefinementResult
  refine(const mesh::Mesh &mesh, std::span<const std::size_t> marked) const = 0;
};

// Sets the refinement edge of every triangle to its longest edge: rotates
// each triangle's vertices (keeping them counterclockwise) so that the
// vertex opposite the longest edge comes first. Among equally long
// longest edges (exactly equal squared lengths) the one whose opposite
// vertex has the smallest node index is taken. Use it once on a mesh that
// did not come from a RefinementMethod (from the Mesher, a file), before
// the first refinement. Throws std::invalid_argument for a node index out
// of range.
void set_longest_edges_as_refinement_edges(mesh::Mesh &mesh);

} // namespace pde2d::adapt
