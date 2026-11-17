#include "pde2d/io/gmsh.hpp"

#include "text.hpp"

#include "pde2d/geometry/curve_geometry.hpp"
#include "pde2d/geometry/predicates.hpp"
#include "pde2d/mesh/working_mesh.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace pde2d::io {

using geometry::Point;

namespace {

[[noreturn]] void fail(const std::string &what) {
  throw std::invalid_argument("Gmsh import: " + what);
}

// The file as lines of tokens; section markers ($Nodes, ...) are lines of
// their own.
class Lines {
public:
  explicit Lines(const std::string &text) {
    std::size_t begin = 0;
    while (begin <= text.size()) {
      std::size_t end = text.find('\n', begin);
      if (end == std::string::npos)
        end = text.size();
      std::string_view line(text.data() + begin, end - begin);
      if (!line.empty() && line.back() == '\r')
        line.remove_suffix(1);
      lines_.push_back(line);
      begin = end + 1;
    }
  }

  bool done() {
    while (at_ < lines_.size() && tokens(lines_[at_]).empty())
      ++at_;
    return at_ >= lines_.size();
  }
  std::size_t line_number() const { return at_; }
  // Lines left, empty ones included: an upper bound on the records that
  // can still follow, checked before a count from the file is trusted.
  std::size_t remaining() const { return lines_.size() - at_; }
  // Fails unless `count` records of `lines_each` lines can still follow.
  void need_lines(std::size_t count, std::size_t lines_each,
                  const std::string &what) const {
    if (count > remaining() / lines_each)
      fail(what + " needs more lines than the file has");
  }
  std::string_view raw() {
    if (done())
      fail("the file ends inside a section");
    return lines_[at_++];
  }
  // The next non-empty line, split into tokens.
  std::vector<std::string_view> next() { return tokens(raw()); }

  static std::vector<std::string_view> tokens(std::string_view line) {
    std::vector<std::string_view> result;
    std::size_t i = 0;
    while (i < line.size()) {
      while (i < line.size() && (line[i] == ' ' || line[i] == '\t'))
        ++i;
      std::size_t j = i;
      while (j < line.size() && line[j] != ' ' && line[j] != '\t')
        ++j;
      if (j > i)
        result.push_back(line.substr(i, j - i));
      i = j;
    }
    return result;
  }

private:
  std::vector<std::string_view> lines_;
  std::size_t at_ = 0;
};

std::size_t to_unsigned(std::string_view token, const char *what) {
  std::size_t value = 0;
  if (!detail::parse_unsigned(token, value))
    fail(std::string("invalid ") + what + " '" + std::string(token) + "'");
  return value;
}

// The whole token as an integer whose magnitude fits a long long, so that
// negating it is safe.
long long to_signed(std::string_view token, const char *what) {
  long long value = 0;
  auto [end, ec] =
      std::from_chars(token.data(), token.data() + token.size(), value);
  if (ec != std::errc{} || end != token.data() + token.size() ||
      value == std::numeric_limits<long long>::min())
    fail(std::string("invalid ") + what + " '" + std::string(token) + "'");
  return value;
}

// An entity dimension: 0 to 3.
int to_dimension(std::string_view token, const char *what) {
  std::size_t value = to_unsigned(token, what);
  if (value > 3)
    fail(std::string("invalid ") + what + " '" + std::string(token) + "'");
  return static_cast<int>(value);
}

double to_real(std::string_view token, const char *what) {
  double value = 0;
  if (!detail::parse_double(token, value))
    fail(std::string("invalid ") + what + " '" + std::string(token) + "'");
  return value;
}

// A line with exactly / at least `n` tokens.
std::vector<std::string_view> need(std::vector<std::string_view> tokens,
                                   std::size_t n, const char *what,
                                   bool exact = true) {
  if (exact ? tokens.size() != n : tokens.size() < n)
    fail(std::string("malformed ") + what + " line");
  return tokens;
}

void end_section(Lines &in, const std::string &name) {
  auto t = in.next();
  if (t.size() != 1 || t[0] != "$End" + name)
    fail("expected $End" + name);
}

void skip_section(Lines &in, const std::string &name) {
  while (true) {
    auto t = in.next();
    if (t.size() == 1 && t[0] == "$End" + name)
      return;
  }
}

// What an element type is, for the error messages.
std::string describe_type(long long type) {
  switch (type) {
  case 3:
  case 10:
  case 16:
    return "quadrangles (type " + std::to_string(type) + ")";
  case 8:
  case 9:
  case 20:
  case 21:
  case 22:
  case 23:
  case 24:
  case 25:
  case 26:
  case 27:
  case 28:
    return "second- or higher-order elements (type " + std::to_string(type) +
           "); only first-order (P1) meshes are supported";
  case 4:
  case 5:
  case 6:
  case 7:
  case 11:
  case 12:
  case 13:
  case 14:
  case 17:
  case 18:
  case 19:
  case 29:
  case 30:
  case 31:
    return "3D elements (type " + std::to_string(type) + ")";
  default:
    return "elements of type " + std::to_string(type);
  }
}

struct Element {
  long long type = 0;
  std::vector<std::size_t> nodes; // node tags
  std::size_t physical = 0;       // 0: none
  bool several_physicals = false;
};

struct File {
  std::map<std::pair<int, std::size_t>, std::string> physical_names;
  std::vector<std::pair<std::size_t, Point>> nodes; // tag, position
  std::vector<Element> elements;
};

void read_physical_names(Lines &in, File &f) {
  std::size_t n = to_unsigned(need(in.next(), 1, "$PhysicalNames")[0],
                              "physical name count");
  in.need_lines(n, 1, "$PhysicalNames");
  for (std::size_t i = 0; i < n; ++i) {
    std::string_view line = in.raw();
    auto t = need(Lines::tokens(line), 3, "$PhysicalNames", false);
    int dim = to_dimension(t[0], "physical dimension");
    std::size_t tag = to_unsigned(t[1], "physical tag");
    // The name is quoted and may contain spaces.
    auto open = line.find('"'), close = line.rfind('"');
    if (open == std::string_view::npos || close == open)
      fail("a physical name is not quoted");
    f.physical_names[{dim, tag}] =
        std::string(line.substr(open + 1, close - open - 1));
  }
  end_section(in, "PhysicalNames");
}

void element_type_check(long long type, std::size_t nodes) {
  std::size_t expected = type == 1 ? 2 : type == 2 ? 3 : 1;
  if (nodes != expected)
    fail("an element of type " + std::to_string(type) + " needs " +
         std::to_string(expected) + " nodes");
}

bool known_type(long long type) { return type == 1 || type == 2 || type == 15; }

// MSH 2.2: "id type ntags tags... nodes...".
void read_v2(Lines &in, File &f, const std::string &section) {
  if (section == "Nodes") {
    std::size_t n = to_unsigned(need(in.next(), 1, "$Nodes")[0], "node count");
    in.need_lines(n, 1, "$Nodes");
    for (std::size_t i = 0; i < n; ++i) {
      auto t = need(in.next(), 4, "node");
      if (to_real(t[3], "z coordinate") != 0)
        fail("node " + std::string(t[0]) + " is not in the plane z = 0");
      f.nodes.push_back(
          {to_unsigned(t[0], "node tag"),
           {to_real(t[1], "x coordinate"), to_real(t[2], "y coordinate")}});
    }
    end_section(in, "Nodes");
  } else if (section == "Elements") {
    std::size_t n =
        to_unsigned(need(in.next(), 1, "$Elements")[0], "element count");
    in.need_lines(n, 1, "$Elements");
    for (std::size_t i = 0; i < n; ++i) {
      auto t = need(in.next(), 3, "element", false);
      Element e;
      e.type = to_signed(t[1], "element type");
      if (!known_type(e.type))
        fail(describe_type(e.type) + " are not supported");
      // Compared with the tokens left, never added to: a huge count from
      // the file would wrap around.
      std::size_t tags = to_unsigned(t[2], "tag count");
      if (tags > t.size() - 3)
        fail("malformed element line");
      if (tags >= 1)
        e.physical = to_unsigned(t[3], "physical tag");
      for (std::size_t k = 3 + tags; k < t.size(); ++k)
        e.nodes.push_back(to_unsigned(t[k], "node tag"));
      element_type_check(e.type, e.nodes.size());
      f.elements.push_back(std::move(e));
    }
    end_section(in, "Elements");
  } else {
    skip_section(in, section);
  }
}

// MSH 4.1: entities carry the physical tags; nodes and elements come in
// blocks per entity.
void read_v4(Lines &in, File &f, const std::string &section,
             std::map<std::pair<int, long long>, std::vector<std::size_t>>
                 &entity_physicals) {
  if (section == "Entities") {
    auto counts = need(in.next(), 4, "$Entities");
    for (int dim = 0; dim < 4; ++dim) {
      std::size_t n =
          to_unsigned(counts[static_cast<std::size_t>(dim)], "entity count");
      in.need_lines(n, 1, "$Entities");
      for (std::size_t i = 0; i < n; ++i) {
        // Points: tag x y z; others: tag minx miny minz maxx maxy maxz.
        std::size_t skip = dim == 0 ? 4 : 7;
        auto t = need(in.next(), skip + 1, "entity", false);
        long long tag = to_signed(t[0], "entity tag");
        std::size_t np = to_unsigned(t[skip], "physical tag count");
        if (np > t.size() - (skip + 1))
          fail("malformed entity line");
        auto &list = entity_physicals[{dim, tag}];
        for (std::size_t k = 0; k < np; ++k) {
          long long p = to_signed(t[skip + 1 + k], "physical tag");
          list.push_back(static_cast<std::size_t>(p < 0 ? -p : p));
        }
      }
    }
    end_section(in, "Entities");
  } else if (section == "Nodes") {
    auto head = need(in.next(), 4, "$Nodes");
    std::size_t blocks = to_unsigned(head[0], "node block count");
    in.need_lines(blocks, 1, "$Nodes");
    for (std::size_t b = 0; b < blocks; ++b) {
      auto t = need(in.next(), 4, "node block");
      std::size_t dim =
          static_cast<std::size_t>(to_dimension(t[0], "entity dimension"));
      bool parametric = to_unsigned(t[2], "parametric flag") != 0;
      std::size_t n = to_unsigned(t[3], "node count");
      // A tag line and a coordinate line per node.
      in.need_lines(n, 2, "the node block");
      std::vector<std::size_t> tags(n);
      for (auto &tag : tags)
        tag = to_unsigned(need(in.next(), 1, "node tag")[0], "node tag");
      for (std::size_t i = 0; i < n; ++i) {
        auto c = need(in.next(), 3 + (parametric ? dim : 0), "node");
        if (to_real(c[2], "z coordinate") != 0)
          fail("node " + std::to_string(tags[i]) +
               " is not in the plane z = 0");
        f.nodes.push_back(
            {tags[i],
             {to_real(c[0], "x coordinate"), to_real(c[1], "y coordinate")}});
      }
    }
    end_section(in, "Nodes");
  } else if (section == "Elements") {
    auto head = need(in.next(), 4, "$Elements");
    std::size_t blocks = to_unsigned(head[0], "element block count");
    in.need_lines(blocks, 1, "$Elements");
    for (std::size_t b = 0; b < blocks; ++b) {
      auto t = need(in.next(), 4, "element block");
      int dim = to_dimension(t[0], "entity dimension");
      long long entity = to_signed(t[1], "entity tag");
      long long type = to_signed(t[2], "element type");
      std::size_t n = to_unsigned(t[3], "element count");
      in.need_lines(n, 1, "the element block");
      if (!known_type(type))
        fail(describe_type(type) + " are not supported");
      std::size_t physical = 0;
      bool several = false;
      if (auto it = entity_physicals.find({dim, entity});
          it != entity_physicals.end() && !it->second.empty()) {
        physical = it->second.front();
        several = it->second.size() > 1;
      }
      for (std::size_t i = 0; i < n; ++i) {
        auto e = need(in.next(), 2, "element", false);
        Element element{type, {}, physical, several};
        for (std::size_t k = 1; k < e.size(); ++k)
          element.nodes.push_back(to_unsigned(e[k], "node tag"));
        element_type_check(type, element.nodes.size());
        f.elements.push_back(std::move(element));
      }
    }
    end_section(in, "Elements");
  } else if (section == "PartitionedEntities") {
    fail("partitioned meshes are not supported");
  } else {
    skip_section(in, section);
  }
}

mesh::Mesh build_mesh(const File &f) {
  // Node tag -> position in the file.
  std::map<std::size_t, std::size_t> by_tag;
  for (std::size_t i = 0; i < f.nodes.size(); ++i)
    if (!by_tag.emplace(f.nodes[i].first, i).second)
      fail("node tag " + std::to_string(f.nodes[i].first) + " appears twice");
  auto node = [&](std::size_t tag) {
    auto it = by_tag.find(tag);
    if (it == by_tag.end())
      fail("an element uses node " + std::to_string(tag) +
           ", which is not defined");
    return it->second;
  };
  // Triangles, by file node index, made counterclockwise.
  std::vector<std::array<std::size_t, 3>> triangles;
  for (const auto &e : f.elements) {
    if (e.type != 2)
      continue;
    std::array<std::size_t, 3> t{node(e.nodes[0]), node(e.nodes[1]),
                                 node(e.nodes[2])};
    int o = geometry::orient(f.nodes[t[0]].second, f.nodes[t[1]].second,
                             f.nodes[t[2]].second);
    if (o == 0)
      fail("a triangle with nodes " + std::to_string(e.nodes[0]) + ", " +
           std::to_string(e.nodes[1]) + ", " + std::to_string(e.nodes[2]) +
           " is degenerate");
    if (o < 0)
      std::swap(t[1], t[2]);
    triangles.push_back(t);
  }
  if (triangles.empty())
    fail("the file has no triangles");
  // Physical line regions by undirected edge.
  std::map<std::pair<std::size_t, std::size_t>, std::size_t> line_region;
  for (const auto &e : f.elements) {
    if (e.type != 1 || e.physical == 0)
      continue;
    std::size_t a = node(e.nodes[0]), b = node(e.nodes[1]);
    auto key = std::minmax(a, b);
    if (e.several_physicals) {
      line_region[key] = 0; // marked: checked if it is a boundary edge
      continue;
    }
    auto [it, added] = line_region.try_emplace(key, e.physical);
    if (!added && it->second != e.physical)
      it->second = 0;
  }
  // Boundary edges: directed triangle edges whose reverse is not used.
  std::set<std::pair<std::size_t, std::size_t>> directed;
  for (const auto &t : triangles)
    for (std::size_t i = 0; i < 3; ++i)
      directed.insert({t[i], t[(i + 1) % 3]});
  struct Edge {
    std::size_t a, b, physical;
  };
  std::vector<Edge> boundary;
  for (const auto &t : triangles)
    for (std::size_t i = 0; i < 3; ++i) {
      std::size_t a = t[i], b = t[(i + 1) % 3];
      if (directed.count({b, a}))
        continue;
      auto it = line_region.find(std::minmax(a, b));
      std::string where = "the boundary edge between nodes " +
                          std::to_string(f.nodes[a].first) + " and " +
                          std::to_string(f.nodes[b].first);
      if (it == line_region.end())
        fail(where + " is not covered by a line element of a physical "
                     "group of dimension 1");
      if (it->second == 0)
        fail(where + " belongs to several physical groups");
      boundary.push_back({a, b, it->second});
    }
  // Regions in increasing physical tag order.
  std::set<std::size_t> tags;
  for (const auto &e : boundary)
    tags.insert(e.physical);
  std::map<std::size_t, std::size_t> region_of;
  mesh::Mesh m;
  for (auto tag : tags) {
    auto it = f.physical_names.find({1, tag});
    std::string name = it != f.physical_names.end()
                           ? it->second
                           : "physical_" + std::to_string(tag);
    if (!geometry::is_valid_region_name(name))
      fail("physical group name '" + name +
           "' is not a valid region name; use ASCII letters, digits, '_', "
           "'.' or '-'");
    region_of[tag] = m.region_names.size();
    m.region_names.push_back(name);
  }
  // Keep the nodes used by triangles, in file order.
  std::vector<std::size_t> index(f.nodes.size(), mesh::no_index);
  for (const auto &t : triangles)
    for (auto v : t)
      index[v] = 0;
  for (std::size_t i = 0; i < f.nodes.size(); ++i)
    if (index[i] == 0) {
      index[i] = m.nodes.size();
      m.nodes.push_back(f.nodes[i].second);
    }
  for (const auto &t : triangles)
    m.triangles.push_back({{index[t[0]], index[t[1]], index[t[2]]}});
  for (const auto &e : boundary)
    m.boundary_edges.push_back(
        {{index[e.a], index[e.b]}, region_of.at(e.physical), std::nullopt});
  mesh::Mesh result;
  try {
    result = mesh::WorkingMesh::from_mesh(m).to_mesh();
  } catch (const std::invalid_argument &e) {
    fail(std::string("invalid mesh: ") + e.what());
  }
  // Gmsh chose the vertex order; the mesh is delivered ready for adaptive
  // refinement, with each triangle's longest edge as its refinement edge.
  mesh::set_longest_edges_as_refinement_edges(result);
  return result;
}

} // namespace

mesh::Mesh import_gmsh(std::istream &stream) {
  const std::string text = detail::read_all(stream);
  Lines in(text);
  File f;
  int version = 0;
  std::map<std::pair<int, long long>, std::vector<std::size_t>> entities;
  bool nodes = false, elements = false;
  while (!in.done()) {
    auto t = in.next();
    if (t.size() != 1 || t[0].size() < 2 || t[0][0] != '$')
      fail("expected a section such as $Nodes");
    std::string section(t[0].substr(1));
    if (section == "MeshFormat") {
      auto h = need(in.next(), 3, "$MeshFormat");
      if (h[0] == "4.1")
        version = 4;
      else if (h[0] == "2.2")
        version = 2;
      else
        fail("MSH version " + std::string(h[0]) +
             " is not supported; save as version 4.1 or 2.2 (ASCII)");
      if (h[1] != "0")
        fail("binary .msh files are not supported; save as ASCII");
      end_section(in, "MeshFormat");
      continue;
    }
    if (version == 0)
      fail("the file does not start with $MeshFormat");
    if (section == "PhysicalNames") {
      read_physical_names(in, f);
      continue;
    }
    if (section == "Elements" && version == 4 && entities.empty())
      fail("$Elements comes before $Entities");
    nodes = nodes || section == "Nodes";
    elements = elements || section == "Elements";
    if (version == 4)
      read_v4(in, f, section, entities);
    else
      read_v2(in, f, section);
  }
  if (version == 0)
    fail("the file is empty");
  if (!nodes || !elements)
    fail("the file needs both $Nodes and $Elements");
  return build_mesh(f);
}

mesh::Mesh import_gmsh(const std::filesystem::path &path) {
  auto in = detail::open_for_reading(path);
  return import_gmsh(in);
}

} // namespace pde2d::io
