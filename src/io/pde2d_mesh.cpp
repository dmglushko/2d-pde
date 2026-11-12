#include "pde2d/io/pde2d_mesh.hpp"

#include "text.hpp"

#include "pde2d/geometry/curve_geometry.hpp"
#include "pde2d/mesh/working_mesh.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace pde2d::io {

using geometry::Point;

namespace {

// Counts are capped like the previous reader (and the viewer).
constexpr std::size_t max_count = 50'000'000;

// The mesh without its links, validated like a WorkingMesh.
void validate_for_export(const mesh::Mesh &m) {
  mesh::Mesh plain;
  plain.nodes = m.nodes;
  plain.triangles = m.triangles;
  plain.region_names = m.region_names;
  for (const auto &e : m.boundary_edges)
    plain.boundary_edges.push_back({e.nodes, e.region, std::nullopt});
  (void)mesh::WorkingMesh::from_mesh(plain);
  double xmin = INFINITY, xmax = -INFINITY, ymin = INFINITY, ymax = -INFINITY;
  for (Point p : m.nodes) {
    xmin = std::min(xmin, p.x);
    xmax = std::max(xmax, p.x);
    ymin = std::min(ymin, p.y);
    ymax = std::max(ymax, p.y);
  }
  double span = std::max(xmax - xmin, ymax - ymin);
  if (!std::isfinite(span) || !std::isfinite(span * span))
    throw std::invalid_argument(
        ".pde2d export: the coordinate range exceeds the format's numeric "
        "scale");
}

// The boundary loops as the previous writer grouped them: walk from each
// edge not yet visited, in index order; each loop's edges sorted by
// index; the loop through the lexicographically smallest boundary node
// moved to the front.
std::vector<std::vector<std::size_t>> loop_groups(const mesh::Mesh &m) {
  const auto &edges = m.boundary_edges;
  std::map<std::size_t, std::size_t> outgoing;
  for (std::size_t i = 0; i < edges.size(); ++i)
    outgoing[edges[i].nodes[0]] = i;
  std::vector<std::vector<std::size_t>> loops;
  std::vector<bool> visited(edges.size(), false);
  for (std::size_t start = 0; start < edges.size(); ++start) {
    if (visited[start])
      continue;
    auto &loop = loops.emplace_back();
    for (std::size_t e = start; !visited[e];
         e = outgoing.at(edges[e].nodes[1])) {
      visited[e] = true;
      loop.push_back(e);
    }
    std::sort(loop.begin(), loop.end());
  }
  if (loops.size() > 1) {
    auto less = [&](std::size_t u, std::size_t v) {
      Point p = m.nodes[u], q = m.nodes[v];
      return p.x < q.x || (p.x == q.x && p.y < q.y);
    };
    std::size_t least = edges[0].nodes[0], outer = 0;
    for (const auto &e : edges)
      if (less(e.nodes[0], least))
        least = e.nodes[0];
    for (std::size_t k = 0; k < loops.size(); ++k)
      for (auto e : loops[k])
        if (edges[e].nodes[0] == least)
          outer = k;
    std::rotate(loops.begin(),
                loops.begin() + static_cast<std::ptrdiff_t>(outer),
                loops.begin() + static_cast<std::ptrdiff_t>(outer) + 1);
  }
  return loops;
}

// Reads whitespace-separated tokens.
class Tokens {
public:
  explicit Tokens(std::istream &in) : in_(in) {}

  std::string next(const char *what) {
    std::string token;
    if (!(in_ >> token))
      fail(std::string("the file ends before ") + what);
    return token;
  }
  void expect(const std::string &label) {
    std::string token = next(label.c_str());
    if (token != label)
      fail("expected " + label + ", found '" + token + "'");
  }
  std::size_t count(const char *what, std::size_t maximum = max_count) {
    std::string token = next(what);
    std::size_t value = 0;
    if (!detail::parse_unsigned(token, value) || value > maximum)
      fail(std::string("invalid ") + what + " '" + token + "'");
    return value;
  }
  std::size_t index(const char *what, std::size_t size) {
    std::size_t value = count(what, std::numeric_limits<std::size_t>::max());
    if (value >= size)
      fail(std::string(what) + " " + std::to_string(value) +
           " is out of range");
    return value;
  }
  double real(const char *what) {
    std::string token = next(what);
    double value = 0;
    if (!detail::parse_double(token, value))
      fail(std::string("invalid ") + what + " '" + token + "'");
    return value;
  }
  bool at_end() {
    std::string token;
    return !(in_ >> token);
  }

  [[noreturn]] static void fail(const std::string &what) {
    throw std::invalid_argument(".pde2d import: " + what);
  }

private:
  std::istream &in_;
};

// Checks the structure of a version 3/4 (or 5 with RESULT 1) trailer and
// discards it.
void skip_result(Tokens &in, int version, std::size_t nodes,
                 std::size_t edges) {
  in.expect("OPTIONS");
  in.count("algorithm");
  in.count("boundary treatment");
  in.count("preserve_regions", 1);
  in.real("boundary.max_edge_length");
  in.real("boundary.max_curve_deviation");
  in.real("refinement.max_edge_length");
  in.count("refinement.max_steps", std::numeric_limits<int>::max());
  if (version >= 4)
    in.count("preserve_corners", 1);
  in.expect("REPORT");
  in.count("size_satisfied", 1);
  in.count("delaunay_verified", 1);
  in.count("refinement_steps", std::numeric_limits<std::size_t>::max());
  in.next("the report message");
  in.expect("CONTEXT");
  if (in.count("context flag", 1) == 1) {
    in.expect("PROTECTED_VERTICES");
    std::size_t n = in.count("protected vertex count", nodes);
    for (std::size_t i = 0; i < n; ++i)
      in.index("protected vertex", nodes);
    in.expect("POLYGON_SNAPSHOT");
    n = in.count("polygon snapshot count");
    for (std::size_t i = 0; i < 2 * n; ++i)
      in.real("polygon snapshot coordinate");
    if (version >= 5) {
      in.expect("POLYGON_LOOPS");
      std::size_t k = in.count("polygon loop count", n);
      for (std::size_t i = 0; i < k; ++i)
        in.count("polygon loop size", n);
    }
    in.expect("SOURCE_CURVES");
    std::size_t curves = in.count("source curve count");
    for (std::size_t i = 0; i < curves; ++i) {
      in.next("a source curve id");
      in.next("a source curve region");
      std::size_t kind = in.count("source curve kind", 3);
      static const std::size_t parameters[] = {4, 6, 8, 7};
      for (std::size_t j = 0; j < parameters[kind]; ++j)
        in.real("source curve parameter");
    }
    in.expect("EDGE_SOURCES");
    if (in.count("edge source count") != edges)
      Tokens::fail("EDGE_SOURCES must have one record per boundary edge");
    for (std::size_t e = 0; e < edges; ++e) {
      std::size_t spans = in.count("source span count");
      for (std::size_t s = 0; s < spans; ++s) {
        in.index("source curve index", curves);
        in.real("span begin");
        in.real("span end");
      }
    }
  }
  in.expect("END_RESULT");
}

} // namespace

void export_pde2d(const mesh::Mesh &m, std::ostream &out) {
  validate_for_export(m);
  auto loops = loop_groups(m);
  const int version = loops.size() > 1 ? 5 : 2;
  std::ostringstream s;
  s.imbue(std::locale::classic());
  s << std::setprecision(17);
  s << "PDE2D_MESH " << version << "\nORDER 1\n";
  s << "NODES " << m.nodes.size() << '\n';
  for (Point p : m.nodes)
    s << p.x << ' ' << p.y << '\n';
  s << "ELEMENTS " << m.triangles.size() << '\n';
  for (const auto &t : m.triangles)
    s << "3 " << t.nodes[0] << ' ' << t.nodes[1] << ' ' << t.nodes[2] << '\n';
  s << "BOUNDARY_EDGES " << m.boundary_edges.size() << '\n';
  for (const auto &loop : loops)
    for (auto e : loop) {
      const auto &edge = m.boundary_edges[e];
      s << edge.nodes[0] << ' ' << edge.nodes[1] << ' '
        << m.region_names[edge.region] << '\n';
    }
  if (version == 5) {
    s << "BOUNDARY_LOOPS " << loops.size() << '\n';
    for (const auto &loop : loops)
      s << loop.size() << '\n';
    s << "RESULT 0\n";
  }
  out << s.str();
  detail::check_written(out);
}

void export_pde2d(const mesh::Mesh &m, const std::filesystem::path &path) {
  detail::write_file(path, [&](std::ostream &out) { export_pde2d(m, out); });
}

mesh::Mesh import_pde2d(std::istream &stream) {
  stream.imbue(std::locale::classic());
  Tokens in(stream);
  in.expect("PDE2D_MESH");
  std::size_t version = in.count("version", 99);
  if (version < 1 || version > 5)
    Tokens::fail("version " + std::to_string(version) +
                 " is not supported (1 to 5 exist)");
  if (version == 1)
    Tokens::fail("version 1 has no region tags, but every boundary edge of a "
                 "Mesh needs a region");
  in.expect("ORDER");
  if (in.count("order") != 1)
    Tokens::fail("only ORDER 1 (three-node triangles) is supported");
  mesh::Mesh m;
  in.expect("NODES");
  // Records are appended as they are read, never allocated for the count
  // first: a file that claims millions of records but ends early stops at
  // its end instead of allocating for the claim.
  std::size_t count = in.count("node count");
  for (std::size_t i = 0; i < count; ++i) {
    double x = in.real("node x");
    m.nodes.push_back({x, in.real("node y")});
  }
  in.expect("ELEMENTS");
  count = in.count("element count");
  for (std::size_t i = 0; i < count; ++i) {
    if (in.count("element node count") != 3)
      Tokens::fail("only three-node triangles are supported");
    auto &t = m.triangles.emplace_back();
    for (auto &v : t.nodes)
      v = in.index("element node", m.nodes.size());
  }
  in.expect("BOUNDARY_EDGES");
  count = in.count("boundary edge count");
  std::map<std::string, std::size_t> region_id;
  for (std::size_t i = 0; i < count; ++i) {
    auto &e = m.boundary_edges.emplace_back();
    e.nodes[0] = in.index("boundary edge node", m.nodes.size());
    e.nodes[1] = in.index("boundary edge node", m.nodes.size());
    std::string tag = in.next("a region tag");
    if (tag == "-")
      Tokens::fail("a boundary edge has no region tag ('-'), but every "
                   "boundary edge of a Mesh needs a region");
    if (!geometry::is_valid_region_name(tag))
      Tokens::fail("invalid region tag '" + tag + "'");
    auto [it, added] = region_id.try_emplace(tag, m.region_names.size());
    if (added)
      m.region_names.push_back(tag);
    e.region = it->second;
  }
  std::vector<std::size_t> loop_sizes;
  bool result = version == 3 || version == 4;
  if (version == 5) {
    in.expect("BOUNDARY_LOOPS");
    loop_sizes.resize(in.count("boundary loop count", m.boundary_edges.size()));
    if (loop_sizes.empty())
      Tokens::fail("BOUNDARY_LOOPS needs at least one loop");
    std::size_t total = 0;
    for (auto &n : loop_sizes) {
      n = in.count("boundary loop size", m.boundary_edges.size());
      if (n < 3)
        Tokens::fail("a boundary loop needs at least three edges");
      total += n;
    }
    if (total != m.boundary_edges.size())
      Tokens::fail("the boundary loop sizes do not add up to the number of "
                   "boundary edges");
    in.expect("RESULT");
    result = in.count("result flag", 1) == 1;
  }
  if (result)
    skip_result(in, static_cast<int>(version), m.nodes.size(),
                m.boundary_edges.size());
  if (!in.at_end())
    Tokens::fail("unexpected content after the last section");

  mesh::WorkingMesh working = [&] {
    try {
      return mesh::WorkingMesh::from_mesh(m);
    } catch (const std::invalid_argument &e) {
      Tokens::fail(std::string("invalid mesh: ") + e.what());
    }
  }();
  // Groups of version 5: each one closed loop. WorkingMesh guarantees one
  // outgoing boundary edge per boundary node.
  std::map<std::size_t, std::size_t> outgoing;
  for (std::size_t i = 0; i < m.boundary_edges.size(); ++i)
    outgoing[m.boundary_edges[i].nodes[0]] = i;
  if (version < 5)
    loop_sizes = {m.boundary_edges.size()};
  std::size_t first = 0;
  for (std::size_t k = 0; k < loop_sizes.size(); ++k) {
    const std::size_t end = first + loop_sizes[k];
    std::size_t e = first, steps = 0;
    bool inside = true;
    do {
      e = outgoing.at(m.boundary_edges[e].nodes[1]);
      inside = e >= first && e < end;
      ++steps;
    } while (inside && e != first);
    if (!inside || steps != loop_sizes[k])
      Tokens::fail(version < 5 ? "the mesh has several boundary loops, which "
                                 "needs version 5"
                               : "boundary loop " + std::to_string(k) +
                                     " is not one closed loop");
    first = end;
  }
  return working.to_mesh();
}

mesh::Mesh import_pde2d(const std::filesystem::path &path) {
  auto in = detail::open_for_reading(path);
  return import_pde2d(in);
}

} // namespace pde2d::io
