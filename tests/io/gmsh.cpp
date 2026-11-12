#include "support/check.hpp"
#include "support/mesh_checks.hpp"

#include "pde2d/io/gmsh.hpp"
#include "pde2d/io/pde2d_mesh.hpp"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

using namespace pde2d;
using namespace pde2d::test;
using mesh::Mesh;

namespace {

std::filesystem::path data(const char *name) {
  return std::filesystem::path(PDE2D_TEST_DATA_DIR) / name;
}

std::string file_text(const char *name) {
  std::ifstream in(data(name));
  std::stringstream s;
  s << in.rdbuf();
  return s.str();
}

Mesh from_text(const std::string &text) {
  std::istringstream in(text);
  return io::import_gmsh(in);
}

std::string replace(std::string s, const std::string &from,
                    const std::string &to) {
  auto at = s.find(from);
  check(at != std::string::npos, "test text contains '" + from + "'");
  s.replace(at, from.size(), to);
  return s;
}

// The unit square split into four triangles around its center (node tags
// 1-4 the corners, 5 the center); see data/square_v41.msh.
void check_square(const Mesh &m) {
  check_valid_mesh(m);
  check(m.nodes.size() == 5, "unused node 6 dropped");
  check(m.nodes[4] == Point{0.5, 0.5}, "file order kept");
  check(m.triangles.size() == 4, "four triangles");
  // The third triangle is clockwise in the file.
  check(m.triangles[2].nodes == std::array<std::size_t, 3>{4, 2, 3},
        "made counterclockwise");
  check(m.region_names ==
            std::vector<std::string>{"bottom", "sides", "physical_30"},
        "regions by physical tag; an unnamed group gets physical_<tag>");
  check(m.boundary_edges.size() == 4, "four boundary edges");
  std::vector<std::string> regions;
  for (const auto &e : m.boundary_edges)
    regions.push_back(m.region_names[e.region]);
  check(regions ==
            std::vector<std::string>{"bottom", "sides", "physical_30", "sides"},
        "each side's region");
  check(m.boundary_edges[0].nodes == std::array<std::size_t, 2>{0, 1},
        "domain on the left");
  check(m.chains.empty(), "no links");
  near(mesh_area(m), 1, 1e-15, "area");
}

} // namespace

int main() {
  return run({
      {"msh_4_1",
       [] { check_square(io::import_gmsh(data("square_v41.msh"))); }},
      {"msh_2_2",
       [] {
         Mesh v2 = io::import_gmsh(data("square_v22.msh"));
         check_square(v2);
         check(same_mesh(v2, io::import_gmsh(data("square_v41.msh"))),
               "both versions give the same mesh");
       }},
      {"hole",
       [] {
         Mesh m = io::import_gmsh(data("square_with_hole_v22.msh"));
         check_valid_mesh(m);
         check(m.region_names == std::vector<std::string>{"outer", "hole"},
               "regions");
         near(mesh_area(m), 8, 1e-15, "area 9 - 1");
         near(boundary_area(m), 8, 1e-15, "hole loop clockwise");
         // And on to the viewer's format: two loops.
         std::ostringstream out;
         io::export_pde2d(m, out);
         check(out.str().find("BOUNDARY_LOOPS 2") != std::string::npos,
               "version 5 with two loops");
       }},
      {"unsupported_files_are_rejected",
       [] {
         auto rejects = [](const std::string &text, const char *fragment) {
           expect_throws<std::invalid_argument>([&] { from_text(text); },
                                                fragment);
         };
         const std::string v2 = file_text("square_v22.msh");
         const std::string v4 = file_text("square_v41.msh");
         rejects(replace(v2, "2.2 0 8", "3.0 0 8"),
                 "MSH version 3.0 is not supported");
         rejects(replace(v2, "2.2 0 8", "2.2 1 8"), "binary .msh files");
         rejects(replace(v2, "10 2 2 100 1 4 1 5", "10 3 2 100 1 4 1 5 2"),
                 "quadrangles (type 3) are not supported");
         rejects(replace(v2, "10 2 2 100 1 4 1 5", "10 9 2 100 1 4 1 5 2 3 4"),
                 "second- or higher-order elements (type 9)");
         rejects(replace(v2, "10 2 2 100 1 4 1 5", "10 4 2 100 1 4 1 5 2"),
                 "3D elements (type 4)");
         rejects(replace(v2, "5 1 2 30 3 3 4", "5 1 2 0 3 3 4"),
                 "the boundary edge between nodes 3 and 4 is not covered by a "
                 "line element of a physical group of dimension 1");
         rejects(replace(v2, "4 0 1 0\n", "4 0 1 0.5\n"),
                 "node 4 is not in the plane z = 0");
         rejects(replace(v2, "1 10 \"bottom\"", "1 10 \"the bottom\""),
                 "physical group name 'the bottom' is not a valid region");
         rejects(replace(v2, "7 2 2 100 1 1 2 5", "7 2 2 100 1 1 2 9"),
                 "uses node 9, which is not defined");
         rejects(replace(v2, "7 2 2 100 1 1 2 5", "7 2 2 100 1 1 2 1"),
                 "is degenerate");
         rejects(replace(v4, "1 0 0 0 1 0 0 1 10 2 1 -2",
                         "1 0 0 0 1 0 0 2 10 20 2 1 -2"),
                 "belongs to several physical groups");
         rejects(replace(v4, "$Entities", "$PartitionedEntities"),
                 "partitioned meshes are not supported");
         rejects("$Nodes\n0\n$EndNodes\n",
                 "the file does not start with $MeshFormat");
         rejects("", "the file is empty");
         rejects(v2.substr(0, v2.find("$Elements")),
                 "needs both $Nodes and $Elements");
         rejects(v2.substr(0, v2.find("$EndElements")),
                 "the file ends inside a section");
         expect_throws<std::runtime_error>(
             [] { (void)io::import_gmsh(data("missing.msh")); }, "Cannot open");
         // Connected correctly, but the second triangle lies inside the
         // first.
         rejects("$MeshFormat\n2.2 0 8\n$EndMeshFormat\n$Nodes\n6\n"
                 "1 0 0 0\n2 2 0 0\n3 0 2 0\n4 0.2 0.2 0\n5 1 0.2 0\n"
                 "6 0.2 1 0\n$EndNodes\n$Elements\n8\n"
                 "1 1 2 7 1 1 2\n2 1 2 7 1 2 3\n3 1 2 7 1 3 1\n"
                 "4 1 2 7 1 4 5\n5 1 2 7 1 5 6\n6 1 2 7 1 6 4\n"
                 "7 2 2 9 1 1 2 3\n8 2 2 9 1 4 5 6\n$EndElements\n",
                 "invalid mesh: Node 3 lies in triangle 0");
         // Counts read from the file are never added to or multiplied
         // before they are compared with what the line really holds.
         rejects(replace(v2, "7 2 2 100 1 1 2 5", "7 2 18446744073709551615"),
                 "malformed element line");
         rejects(replace(v2, "7 2 2 100 1 1 2 5", "7 2 18446744073709551614"),
                 "malformed element line");
         rejects(replace(v2, "7 2 2 100 1 1 2 5", "7 2 99999999999999999999"),
                 "invalid tag count");
         rejects(replace(v4, "1 0 0 0 1 0 0 1 10 2 1 -2",
                         "1 0 0 0 1 0 0 18446744073709551615"),
                 "malformed entity line");
         rejects(replace(v4, "1 0 0 0 1 0 0 1 10 2 1 -2",
                         "1 0 0 0 1 0 0 18446744073709551609 10 2 1 -2"),
                 "malformed entity line");
         rejects(replace(v4, "1 0 0 0 1 0 0 1 10 2 1 -2",
                         "1 0 0 0 1 0 0 1 -9223372036854775808 2 1 -2"),
                 "invalid physical tag '-9223372036854775808'");
         rejects(
             replace(v4, "1 2 1 1\n4 2 3", "18446744073709551615 2 1 1\n4 2 3"),
             "invalid entity dimension");
         // A huge count with nothing behind it: no allocation of that size.
         rejects(replace(v4, "0 1 0 1\n1\n", "0 1 0 4611686018427387904\n1\n"),
                 "the node block needs more lines than the file has");
         rejects(replace(v4, "2 1 0 2\n", "18446744073709551614 1 1 2\n"),
                 "invalid entity dimension");
         rejects(replace(v4, "0 1 15 1\n", "0 1 15 4611686018427387904\n"),
                 "the element block needs more lines than the file has");
         rejects(
             replace(v2, "$Elements\n10\n", "$Elements\n4611686018427387904\n"),
             "$Elements needs more lines than the file has");
         rejects(replace(v2, "$Nodes\n6\n", "$Nodes\n4611686018427387904\n"),
                 "$Nodes needs more lines than the file has");
         // Unknown sections are skipped.
         Mesh m = from_text(replace(v2, "$Nodes",
                                    "$Comments\nanything $at all\n"
                                    "$EndComments\n$Nodes"));
         check(m.triangles.size() == 4, "comments skipped");
       }},
  });
}
