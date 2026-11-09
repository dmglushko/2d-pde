#pragma once

#include "boundary_context.hpp"
#include "curve_io.hpp"
#include "mesh_record.hpp"
#include "mesh_types.hpp"
#include "mesh_validation.hpp"
#include "triangulate.hpp"
#include <charconv>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace pde2d::discretization {
namespace detail {

inline void validate_mesh_format(const TriangleMesh &mesh) {
  for (const auto &tag : mesh.boundary_tags) {
    if (!boundary_detail::valid_tag(tag))
      throw std::runtime_error("Invalid boundary tag");
  }
  double xmin = mesh.nodes[0].x, xmax = xmin, ymin = mesh.nodes[0].y,
         ymax = ymin;
  for (const auto &p : mesh.nodes) {
    xmin = std::min(xmin, p.x);
    xmax = std::max(xmax, p.x);
    ymin = std::min(ymin, p.y);
    ymax = std::max(ymax, p.y);
  }
  const double span = std::max(xmax - xmin, ymax - ymin);
  if (!std::isfinite(span) || !std::isfinite(span * span))
    throw std::runtime_error("Mesh exceeds supported numeric scale");
}

inline std::size_t read_unsigned(std::istream &in,
                                 std::size_t maximum = 50000000) {
  std::string token;
  if (!(in >> token))
    throw std::runtime_error("Missing unsigned integer");
  std::size_t value = 0;
  auto parsed =
      std::from_chars(token.data(), token.data() + token.size(), value);
  if (parsed.ec != std::errc{} || parsed.ptr != token.data() + token.size() ||
      value > maximum)
    throw std::runtime_error("Invalid unsigned integer: " + token);
  return value;
}
inline void expect(std::istream &in, const std::string &label) {
  std::string actual;
  if (!(in >> actual) || actual != label)
    throw std::runtime_error("Expected section " + label);
}
inline std::size_t read_count(std::istream &in, const std::string &label) {
  expect(in, label);
  return read_unsigned(in);
}

// Mesh loops as consecutive groups: the permutation of boundary edges that
// lists each loop of boundary_loops() (outer loop first) in its original
// relative edge order, and each group's size. Identity for grouped meshes,
// which is what meshing produces.
struct LoopGrouping {
  std::vector<std::size_t> order, sizes;
  bool identity() const {
    for (std::size_t i = 0; i < order.size(); ++i)
      if (order[i] != i)
        return false;
    return true;
  }
};
inline LoopGrouping loop_grouping(const TriangleMesh &mesh) {
  LoopGrouping g;
  for (auto loop : boundary_loops(mesh)) {
    std::sort(loop.begin(), loop.end());
    g.order.insert(g.order.end(), loop.begin(), loop.end());
    g.sizes.push_back(loop.size());
  }
  return g;
}
template <class T>
std::vector<T> permuted(const std::vector<T> &values,
                        const std::vector<std::size_t> &order) {
  std::vector<T> out;
  out.reserve(values.size());
  for (auto i : order)
    out.push_back(values[i]);
  return out;
}
// Version 5 lists boundary edges loop by loop: regroup edges, tags and
// their source intervals together (vertex indices are unchanged).
inline MeshResult grouped_by_loop(const MeshResult &r, const LoopGrouping &g) {
  if (g.identity())
    return r;
  MeshResult out = r;
  out.mesh.boundary_edges = permuted(r.mesh.boundary_edges, g.order);
  if (!r.mesh.boundary_tags.empty())
    out.mesh.boundary_tags = permuted(r.mesh.boundary_tags, g.order);
  if (r.boundary_context)
    out.boundary_context->edge_sources =
        permuted(r.boundary_context->edge_sources, g.order);
  return out;
}
inline void write_loops(std::ostream &out, const LoopGrouping &g) {
  out << "BOUNDARY_LOOPS " << g.sizes.size() << '\n';
  for (auto n : g.sizes)
    out << n << '\n';
}
} // namespace detail

// Save a validated P1 triangular mesh in the project's
// versioned, zero-based-index text format.
namespace detail {
inline void write_mesh(std::ostream &out, const TriangleMesh &mesh,
                       int version) {
  out << "PDE2D_MESH " << version << "\nORDER 1\n";
  out << "NODES " << mesh.nodes.size() << '\n' << std::setprecision(17);
  for (const auto &p : mesh.nodes)
    out << p.x << ' ' << p.y << '\n';
  out << "ELEMENTS " << mesh.elements.size() << '\n';
  for (const auto &e : mesh.elements) {
    out << e.size();
    for (auto i : e)
      out << ' ' << i;
    out << '\n';
  }
  out << "BOUNDARY_EDGES " << mesh.boundary_edges.size() << '\n';
  for (std::size_t i = 0; i < mesh.boundary_edges.size(); ++i) {
    const auto &e = mesh.boundary_edges[i];
    const auto tag =
        mesh.boundary_tags.empty() ? std::string{} : mesh.boundary_tags[i];
    out << e.first << ' ' << e.second << ' ' << (tag.empty() ? "-" : tag)
        << '\n';
  }
}
} // namespace detail

namespace detail {
inline void validate_region(const std::string &tag) {
  if (!boundary_detail::valid_tag(tag))
    throw std::runtime_error("Invalid source curve region");
}
inline void validate_result(const MeshResult &r) {
  const auto &o = r.options;
  try {
    validateMeshOptionValues(o);
  } catch (const std::invalid_argument &) {
    throw std::runtime_error("Invalid mesh result options");
  }
  if (r.report.delaunay_verified && !mesh_detail::cdt_verified(r.mesh))
    throw std::runtime_error("Recorded CDT guarantee does not hold");
  if (r.report.size_satisfied && o.refinement.max_edge_length > 0) {
    for (auto t : r.mesh.elements)
      for (int e = 0; e < 3; ++e)
        if (mesh_detail::distance(r.mesh.nodes[t[e]],
                                  r.mesh.nodes[t[(e + 1) % 3]]) >
            o.refinement.max_edge_length * (1 + 1e-12))
          throw std::runtime_error("Recorded size guarantee does not hold");
  }
  if (!r.boundary_context)
    return;
  const auto &c = *r.boundary_context;
  validate_boundary_context(r.mesh, c, o.preserve_regions,
                            o.boundary.preserve_corners);
  for (const auto &curve : c.curves)
    validate_region(curve.region);
  // Unlike refinement, serialization cannot restore omitted region tags.
  if (o.preserve_regions && r.mesh.boundary_tags.empty() &&
      std::any_of(c.curves.begin(), c.curves.end(),
                  [](const auto &curve) { return !curve.region.empty(); }))
    throw std::runtime_error("Boundary region disagrees with source curve");
}
inline std::string encode_message(const std::string &message) {
  if (message.empty())
    return "-";
  std::string hex;
  constexpr char digits[] = "0123456789abcdef";
  for (unsigned char c : message) {
    hex += digits[c >> 4];
    hex += digits[c & 15];
  }
  return hex;
}
inline std::string decode_message(const std::string &hex) {
  if (hex == "-")
    return {};
  if (hex.empty() || hex.size() % 2)
    throw std::runtime_error("Invalid report message encoding");
  auto digit = [](char c) -> int {
    if (c >= '0' && c <= '9')
      return c - '0';
    if (c >= 'a' && c <= 'f')
      return c - 'a' + 10;
    return -1;
  };
  std::string message;
  for (std::size_t i = 0; i < hex.size(); i += 2) {
    int a = digit(hex[i]), b = digit(hex[i + 1]);
    if (a < 0 || b < 0)
      throw std::runtime_error("Invalid report message encoding");
    message += static_cast<char>(a * 16 + b);
  }
  return message;
}
inline void write_result(std::ostream &out, const MeshResult &r, int version) {
  const auto &o = r.options;
  out << "OPTIONS " << static_cast<int>(o.algorithm) << ' '
      << static_cast<int>(o.boundary_treatment) << ' ' << o.preserve_regions
      << ' ' << o.boundary.max_edge_length << ' '
      << o.boundary.max_curve_deviation << ' ' << o.refinement.max_edge_length
      << ' ' << o.refinement.max_steps << ' ' << o.boundary.preserve_corners
      << '\n';
  out << "REPORT " << r.report.size_satisfied << ' '
      << r.report.delaunay_verified << ' ' << r.report.refinement_steps << ' '
      << encode_message(r.report.message) << '\n';
  out << "CONTEXT " << bool(r.boundary_context) << '\n';
  if (r.boundary_context) {
    const auto &c = *r.boundary_context;
    out << "PROTECTED_VERTICES " << c.protected_vertices.size() << '\n';
    for (auto v : c.protected_vertices)
      out << v << '\n';
    out << "POLYGON_SNAPSHOT " << c.polygon_snapshot.size() << '\n';
    for (auto p : c.polygon_snapshot)
      out << p.x << ' ' << p.y << '\n';
    if (version >= 5) {
      out << "POLYGON_LOOPS "
          << std::max<std::size_t>(1, c.snapshot_loop_sizes.size()) << '\n';
      if (c.snapshot_loop_sizes.empty())
        out << c.polygon_snapshot.size() << '\n';
      for (auto n : c.snapshot_loop_sizes)
        out << n << '\n';
    }
    out << "SOURCE_CURVES " << c.curves.size() << '\n';
    for (const auto &curve : c.curves) {
      out << curve.id.value << ' '
          << (curve.region.empty() ? "-" : curve.region) << ' ';
      curve_io::write(out, curve.segment);
      out << '\n';
    }
    out << "EDGE_SOURCES " << c.edge_sources.size() << '\n';
    for (const auto &spans : c.edge_sources) {
      out << spans.size();
      for (const auto &s : spans)
        out << ' ' << s.curve << ' ' << s.begin << ' ' << s.end;
      out << '\n';
    }
  }
  out << "END_RESULT\n";
}
inline double read_real(std::istream &in) {
  double d = 0;
  if (!(in >> d) || !std::isfinite(d))
    throw std::runtime_error("Invalid finite real value");
  return d;
}
inline std::uint64_t read_curve_id(std::istream &in) {
  std::string token;
  if (!(in >> token))
    throw std::runtime_error("Missing curve ID");
  std::uint64_t id = 0;
  auto p = std::from_chars(token.data(), token.data() + token.size(), id);
  if (p.ec != std::errc{} || p.ptr != token.data() + token.size())
    throw std::runtime_error("Invalid curve ID");
  return id;
}
inline void read_result(std::istream &in, MeshResult &r, int version) {
  expect(in, "OPTIONS");
  auto &o = r.options;
  o.algorithm = static_cast<InteriorAlgorithm>(
      read_unsigned(in, std::numeric_limits<int>::max()));
  o.boundary_treatment = static_cast<BoundaryTreatment>(
      read_unsigned(in, std::numeric_limits<int>::max()));
  o.preserve_regions = read_unsigned(in, 1) != 0;
  o.boundary.max_edge_length = read_real(in);
  o.boundary.max_curve_deviation = read_real(in);
  o.refinement.max_edge_length = read_real(in);
  o.refinement.max_steps =
      static_cast<int>(read_unsigned(in, std::numeric_limits<int>::max()));
  o.boundary.preserve_corners = version >= 4 ? read_unsigned(in, 1) != 0 : true;
  expect(in, "REPORT");
  r.report.size_satisfied = read_unsigned(in, 1) != 0;
  r.report.delaunay_verified = read_unsigned(in, 1) != 0;
  r.report.refinement_steps =
      read_unsigned(in, std::numeric_limits<std::size_t>::max());
  std::string message;
  if (!(in >> message))
    throw std::runtime_error("Missing report message");
  r.report.message = decode_message(message);
  expect(in, "CONTEXT");
  if (read_unsigned(in, 1)) {
    r.boundary_context.emplace();
    auto &c = *r.boundary_context;
    auto n = read_count(in, "PROTECTED_VERTICES");
    if (n > r.mesh.nodes.size())
      throw std::runtime_error("Too many protected vertices");
    c.protected_vertices.resize(n);
    for (auto &v : c.protected_vertices)
      v = read_unsigned(in);
    n = read_count(in, "POLYGON_SNAPSHOT");
    c.polygon_snapshot.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
      double x = read_real(in), y = read_real(in);
      c.polygon_snapshot.push_back({x, y});
    }
    if (version >= 5) {
      auto loops = read_count(in, "POLYGON_LOOPS");
      if (loops == 0 || loops > c.polygon_snapshot.size())
        throw std::runtime_error("Invalid polygon snapshot loop count");
      std::vector<std::size_t> sizes(loops);
      for (auto &size : sizes)
        size = read_unsigned(in);
      if (loops > 1)
        c.snapshot_loop_sizes = std::move(sizes);
      else if (sizes[0] != c.polygon_snapshot.size())
        throw std::runtime_error("Invalid polygon snapshot loop sizes");
    }
    n = read_count(in, "SOURCE_CURVES");
    c.curves.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
      BoundaryCurve curve;
      curve.id.value = read_curve_id(in);
      if (!(in >> curve.region))
        throw std::runtime_error("Missing source region");
      if (curve.region == "-")
        curve.region.clear();
      auto kind = read_unsigned(in, std::numeric_limits<unsigned>::max());
      curve.segment = curve_io::read(in, static_cast<unsigned>(kind));
      c.curves.push_back(std::move(curve));
    }
    n = read_count(in, "EDGE_SOURCES");
    if (n != r.mesh.boundary_edges.size())
      throw std::runtime_error("Invalid edge source count");
    c.edge_sources.resize(n);
    std::size_t total = 0;
    for (auto &spans : c.edge_sources) {
      auto count = read_unsigned(in);
      if (count > 50000000 - total)
        throw std::runtime_error("Too many source spans");
      total += count;
      spans.resize(count);
      for (auto &s : spans) {
        s.curve = read_unsigned(in);
        s.begin = read_real(in);
        s.end = read_real(in);
      }
    }
  }
  expect(in, "END_RESULT");
}
} // namespace detail

// Meshes with one boundary loop are written as version 4 (MeshResult) or 2
// (TriangleMesh), byte for byte as before holes existed. Meshes with holes
// are written as version 5, which lists the boundary edges loop by loop
// (outer loop first); edges of an ungrouped mesh are regrouped stably.
inline void export_mesh(const MeshResult &result, const std::string &path) {
  validate_mesh_geometry(result.mesh);
  detail::validate_mesh_format(result.mesh);
  detail::validate_result(result);
  const auto grouping = detail::loop_grouping(result.mesh);
  const bool holes = grouping.sizes.size() > 1;
  std::ofstream out(path);
  if (!out)
    throw std::runtime_error("Cannot create mesh file: " + path);
  if (holes) {
    const auto grouped = detail::grouped_by_loop(result, grouping);
    detail::write_mesh(out, grouped.mesh, 5);
    detail::write_loops(out, grouping);
    out << "RESULT 1\n";
    detail::write_result(out, grouped, 5);
  } else {
    detail::write_mesh(out, result.mesh, 4);
    detail::write_result(out, result, 4);
  }
  out.close();
  if (!out)
    throw std::runtime_error("Failed while writing mesh file: " + path);
}

inline void export_mesh(const TriangleMesh &mesh, const std::string &path) {
  validate_mesh_geometry(mesh);
  detail::validate_mesh_format(mesh);
  const auto grouping = detail::loop_grouping(mesh);
  std::ofstream out(path);
  if (!out)
    throw std::runtime_error("Cannot create mesh file: " + path);
  if (grouping.sizes.size() > 1) {
    const auto grouped = detail::grouped_by_loop(
        MeshResult{mesh, std::nullopt, {}, {}}, grouping);
    detail::write_mesh(out, grouped.mesh, 5);
    detail::write_loops(out, grouping);
    out << "RESULT 0\n";
  } else
    detail::write_mesh(out, mesh, 2);
  out.close();
  if (!out)
    throw std::runtime_error("Failed while writing mesh file: " + path);
}

// Load a mesh saved by export_mesh. Rejects truncated, malformed, unsupported,
// or structurally inconsistent data before returning it to the caller.
// The result's record (MeshResult::record) says MeshOperation::Import, with
// the fingerprint, the stored options and the quality of the imported mesh;
// files store no work counters, so they are zero.
[[nodiscard]] inline MeshResult import_mesh_result(const std::string &path) {
  const auto started = mesh_record_detail::Clock::now();
  std::ifstream in(path);
  if (!in)
    throw std::runtime_error("Cannot open mesh file: " + path);
  std::string magic;
  int version = 0;
  if (!(in >> magic >> version) || magic != "PDE2D_MESH" || version < 1 ||
      version > 5)
    throw std::runtime_error("Unsupported or invalid PDE2D mesh-file header");
  MeshResult result;
  if (version < 3) {
    result.report.size_satisfied = false;
    result.report.message =
        "Legacy mesh: meshing settings and guarantees were not recorded";
  }
  auto &mesh = result.mesh;
  std::string label;
  int order = 0;
  // Keep the version-1 record for compatibility with existing P1 files.
  if (!(in >> label >> order) || label != "ORDER" || order != 1)
    throw std::runtime_error("Only P1 mesh files (ORDER 1) are supported");
  const auto node_count = detail::read_count(in, "NODES");
  mesh.nodes.resize(node_count);
  for (auto &p : mesh.nodes)
    if (!(in >> p.x >> p.y))
      throw std::runtime_error("Truncated or invalid node coordinates");
  const auto element_count = detail::read_count(in, "ELEMENTS");
  mesh.elements.resize(element_count);
  for (auto &e : mesh.elements) {
    std::size_t n = detail::read_unsigned(in);
    if (n != 3)
      throw std::runtime_error(
          "A P1 triangle must contain three vertex indices");
    for (auto &i : e)
      i = detail::read_unsigned(in);
  }
  const auto edge_count = detail::read_count(in, "BOUNDARY_EDGES");
  mesh.boundary_edges.resize(edge_count);
  if (version >= 2)
    mesh.boundary_tags.resize(edge_count);
  for (std::size_t i = 0; i < edge_count; ++i) {
    auto &e = mesh.boundary_edges[i];
    e.first = detail::read_unsigned(in);
    e.second = detail::read_unsigned(in);
    if (version >= 2) {
      if (!(in >> mesh.boundary_tags[i]))
        throw std::runtime_error("Missing boundary tag");
      if (mesh.boundary_tags[i] == "-")
        mesh.boundary_tags[i].clear();
    }
  }
  std::vector<std::size_t> loop_sizes;
  bool has_result = version >= 3;
  if (version >= 5) {
    const auto loops = detail::read_count(in, "BOUNDARY_LOOPS");
    if (loops == 0 || loops > edge_count)
      throw std::runtime_error("Invalid boundary loop count");
    loop_sizes.resize(loops);
    std::size_t total = 0;
    for (auto &n : loop_sizes) {
      n = detail::read_unsigned(in);
      if (n < 3 || n > edge_count - total)
        throw std::runtime_error("Invalid boundary loop sizes");
      total += n;
    }
    if (total != edge_count)
      throw std::runtime_error("Invalid boundary loop sizes");
    detail::expect(in, "RESULT");
    has_result = detail::read_unsigned(in, 1) != 0;
    if (!has_result) {
      result.report.size_satisfied = false;
      result.report.message =
          "Meshing settings and guarantees were not recorded";
    }
  }
  if (has_result)
    detail::read_result(in, result, version);
  std::string extra;
  if (in >> extra)
    throw std::runtime_error("Unexpected content after final mesh section");
  validate_mesh_geometry(mesh);
  detail::validate_mesh_format(mesh);
  if (version < 5 && boundary_loops(mesh).size() > 1)
    throw std::runtime_error("A mesh with holes requires PDE2D_MESH version 5");
  if (version >= 5) {
    // Each recorded group must be exactly one mesh boundary loop, outer first.
    const auto grouping = detail::loop_grouping(mesh);
    if (grouping.sizes != loop_sizes || !grouping.identity())
      throw std::runtime_error(
          "Boundary edges are not grouped by their recorded loops");
    if (result.boundary_context) {
      // Source curves belong to the loop whose edges they are sampled on.
      std::size_t offset = 0;
      auto &context = *result.boundary_context;
      for (std::size_t k = 0; k < loop_sizes.size(); ++k) {
        for (std::size_t e = offset; e < offset + loop_sizes[k]; ++e)
          for (auto span : context.edge_sources[e])
            if (span.curve < context.curves.size())
              context.curves[span.curve].loop = k;
        offset += loop_sizes[k];
      }
    }
  }
  if (has_result)
    detail::validate_result(result);
  const double seconds = mesh_record_detail::seconds_since(started);
  finish_mesh_run_record(result, MeshOperation::Import, {}, seconds);
  return result;
}

inline TriangleMesh import_mesh(const std::string &path) {
  return import_mesh_result(path).mesh;
}

} // namespace pde2d::discretization
