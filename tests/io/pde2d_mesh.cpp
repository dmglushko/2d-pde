#include "support/check.hpp"
#include "support/mesh_checks.hpp"
#include "support/shapes.hpp"

#include "pde2d/geometry/shapes.hpp"
#include "pde2d/io/pde2d_mesh.hpp"
#include "pde2d/mesh/cdt.hpp"
#include "pde2d/mesh/chord_length_discretizer.hpp"
#include "pde2d/mesh/mesher.hpp"
#include "pde2d/mesh/ruppert.hpp"

#include <filesystem>
#include <memory>
#include <sstream>
#include <string>

using namespace pde2d;
using namespace pde2d::test;
using mesh::Mesh;

namespace {

std::string to_text(const Mesh &m) {
  std::ostringstream out;
  io::export_pde2d(m, out);
  return out.str();
}

Mesh from_text(const std::string &text) {
  std::istringstream in(text);
  return io::import_pde2d(in);
}

Mesh mesh_of(const geometry::CurveGeometry &g, double h) {
  return mesh::Mesher(std::make_shared<mesh::ChordLengthDiscretizer>(),
                      std::make_shared<mesh::Cdt>(),
                      std::make_shared<mesh::Ruppert>())
      .run(g, {h})
      .mesh;
}

// The unit square in two triangles, with one node at x = 0.1 to show the
// 17 significant digits.
Mesh tiny() {
  Mesh m;
  m.nodes = {{0, 0}, {1, 0}, {1, 1}, {0.1, 1}};
  m.triangles = {{{0, 1, 2}}, {{0, 2, 3}}};
  m.boundary_edges = {
      {{0, 1}, 0, {}}, {{1, 2}, 1, {}}, {{2, 3}, 0, {}}, {{3, 0}, 2, {}}};
  m.region_names = {"wall", "outlet", "inlet"};
  return m;
}

const std::string tiny_text = "PDE2D_MESH 2\n"
                              "ORDER 1\n"
                              "NODES 4\n"
                              "0 0\n"
                              "1 0\n"
                              "1 1\n"
                              "0.10000000000000001 1\n"
                              "ELEMENTS 2\n"
                              "3 0 1 2\n"
                              "3 0 2 3\n"
                              "BOUNDARY_EDGES 4\n"
                              "0 1 wall\n"
                              "1 2 outlet\n"
                              "2 3 wall\n"
                              "3 0 inlet\n";

} // namespace

int main() {
  return run({
      {"exact_text_of_version_2",
       [] {
         check(to_text(tiny()) == tiny_text, "the old format, byte for byte");
         Mesh back = from_text(tiny_text);
         check(same_mesh(back, tiny()), "read back");
       }},
      {"one_loop_round_trip",
       [] {
         Mesh m = mesh_of(geometry::rectangle_with_semicircle(3, 1), 0.3);
         check(!m.chains.empty(), "the mesh has links");
         std::string text = to_text(m);
         check(text.rfind("PDE2D_MESH 2\nORDER 1\n", 0) == 0, "version 2");
         Mesh back = from_text(text);
         check(same_mesh(back, m), "same nodes (exactly), triangles, edges, "
                                   "regions");
         check(back.chains.empty(), "links are dropped");
         for (const auto &e : back.boundary_edges)
           check(!e.link, "no link");
         check(to_text(back) == text, "stable");
       }},
      {"holes_and_islands_are_version_5",
       [] {
         Mesh m = mesh_of(holes_and_island(), 0.5);
         std::string text = to_text(m);
         check(text.rfind("PDE2D_MESH 5\n", 0) == 0, "version 5");
         check(text.find("BOUNDARY_LOOPS 4\n") != std::string::npos,
               "four loops");
         check(text.size() >= 9 && text.substr(text.size() - 9) == "RESULT 0\n",
               "no result trailer");
         Mesh back = from_text(text);
         check(same_mesh(back, m), "same mesh (outer loop already first)");
         check_valid_mesh(back);
       }},
      {"outer_loop_written_first",
       [] {
         // A square with a square hole; the hole's edges come first.
         Mesh m = mesh_of(CurveGeometry({square_loop({1, 1}, 1, "hole"),
                                         square_loop({0, 0}, 3, "outer")}),
                          1.0);
         check(m.region_names[m.boundary_edges[0].region] == "hole",
               "hole first in the mesh");
         std::string text = to_text(m);
         auto edges = text.find("BOUNDARY_EDGES");
         auto first = text.find('\n', edges) + 1;
         auto line = text.substr(first, text.find('\n', first) - first);
         check(line.ends_with(" outer"), "outer loop first in the file");
         Mesh back = from_text(text);
         check(back.region_names == std::vector<std::string>{"outer", "hole"},
               "regions in order of appearance");
         check_valid_mesh(back);
         near(mesh_area(back), 8, 1e-12, "area");
       }},
      {"old_files_are_read",
       [] {
         // Written by the previous code: version 3 with a boundary
         // context, which is skipped.
         std::filesystem::path examples =
             std::filesystem::path(PDE2D_SOURCE_DIR) / "examples/mesh_viewer";
         Mesh old = io::import_pde2d(examples / "rectangle_native.pde2d");
         check_valid_mesh(old);
         check(old.nodes.size() == 112, "112 nodes");
         check(old.boundary_edges.size() == 48, "48 boundary edges");
         check(old.region_names.size() == 4, "four regions");
         near(mesh_area(old), 6, 1e-9, "a 3 x 2 rectangle");
         // Version 1 has no region tags.
         expect_throws<std::invalid_argument>(
             [&] { (void)io::import_pde2d(examples / "rectangle_p1.pde2d"); },
             "version 1 has no region tags");
       }},
      {"invalid_files_are_rejected",
       [] {
         auto rejects = [](const std::string &text, const char *fragment) {
           expect_throws<std::invalid_argument>([&] { from_text(text); },
                                                fragment);
         };
         auto replace = [](std::string s, const std::string &from,
                           const std::string &to) {
           s.replace(s.find(from), from.size(), to);
           return s;
         };
         rejects(replace(tiny_text, "3 0 inlet", "3 0 -"),
                 "has no region tag ('-')");
         rejects(replace(tiny_text, "3 0 inlet", "3 0 a/b"),
                 "invalid region tag 'a/b'");
         rejects(tiny_text + "extra", "unexpected content after the last");
         rejects(replace(tiny_text, "PDE2D_MESH 2", "PDE2D_MESH 6"),
                 "version 6 is not supported");
         rejects(replace(tiny_text, "ORDER 1", "ORDER 2"), "only ORDER 1");
         rejects(replace(tiny_text, "3 0 2 3", "4 0 2 3"),
                 "only three-node triangles");
         rejects(replace(tiny_text, "1 1\n0.1", "1 nan\n0.1"),
                 "invalid node y 'nan'");
         rejects(replace(tiny_text, "3 0 2 3", "3 0 2 7"),
                 "element node 7 is out of range");
         rejects(tiny_text.substr(0, tiny_text.size() - 10),
                 "the file ends before");
         rejects(replace(tiny_text, "3 0 2 3", "3 0 3 2"),
                 "invalid mesh: Triangle 1 is not counterclockwise");
         // Connected correctly, but one triangle lies inside the other.
         rejects("PDE2D_MESH 5\nORDER 1\nNODES 6\n0 0\n2 0\n0 2\n"
                 "0.2 0.2\n1 0.2\n0.2 1\nELEMENTS 2\n3 0 1 2\n3 3 4 5\n"
                 "BOUNDARY_EDGES 6\n0 1 a\n1 2 a\n2 0 a\n3 4 a\n4 5 a\n"
                 "5 3 a\nBOUNDARY_LOOPS 2\n3\n3\nRESULT 0\n",
                 "invalid mesh: Node 3 lies in triangle 0");
         // Several loops need version 5; version 5 groups must be loops.
         Mesh holed = mesh_of(geometry::annulus(1, 0.5), 0.5);
         std::string v5 = to_text(holed);
         std::string v2 = replace(v5.substr(0, v5.find("BOUNDARY_LOOPS")),
                                  "PDE2D_MESH 5", "PDE2D_MESH 2");
         rejects(v2, "several boundary loops, which needs version 5");
         auto loops = v5.find("BOUNDARY_LOOPS 2\n") + 17;
         auto second = v5.find('\n', loops) + 1;
         std::size_t a = std::stoul(v5.substr(loops)),
                     b = std::stoul(v5.substr(second));
         std::string shifted = v5.substr(0, loops) + std::to_string(a - 1) +
                               "\n" + std::to_string(b + 1) + "\n" +
                               v5.substr(v5.find("RESULT"));
         rejects(shifted, "boundary loop 0 is not one closed loop");
         rejects(replace(v5, "RESULT 0", "RESULT 1"), "the file ends before");
         // Large counts with nothing behind them: the reader stops at the
         // end of the file instead of allocating for the count first.
         rejects("PDE2D_MESH 2\nORDER 1\nNODES 50000000\n0 0\n",
                 "the file ends before node x");
         rejects(replace(tiny_text, "ELEMENTS 2", "ELEMENTS 50000000"),
                 "invalid element node count 'BOUNDARY_EDGES'");
         rejects("PDE2D_MESH 2\nORDER 1\nNODES 18446744073709551615\n",
                 "invalid node count");
       }},
      {"export_rejects_invalid_meshes",
       [] {
         Mesh m = tiny();
         m.boundary_edges.pop_back();
         std::ostringstream out;
         expect_throws<std::invalid_argument>(
             [&] { io::export_pde2d(m, out); },
             "has no neighbour but is not a boundary edge");
         Mesh huge = tiny();
         huge.nodes[2] = {1e200, 1e200};
         huge.nodes[1] = {1e200, 0};
         expect_throws<std::invalid_argument>(
             [&] { io::export_pde2d(huge, out); }, "numeric scale");
         expect_throws<std::runtime_error>(
             [&] {
               io::export_pde2d(tiny(),
                                std::filesystem::path("/no/such/dir/m.pde2d"));
             },
             "Cannot open '/no/such/dir/m.pde2d' for writing");
       }},
  });
}
