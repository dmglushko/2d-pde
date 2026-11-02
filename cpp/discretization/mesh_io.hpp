#pragma once

#include "boundary_context.hpp"
#include "curve_io.hpp"
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

namespace discretization {
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
  if (r.report.delaunay_verified && !is_constrained_delaunay(r.mesh))
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
  validate_boundary_context(r.mesh, c, o.preserve_regions);
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
inline void write_result(std::ostream &out, const MeshResult &r) {
  const auto &o = r.options;
  out << "OPTIONS " << static_cast<int>(o.algorithm) << ' '
      << static_cast<int>(o.boundary_treatment) << ' ' << o.preserve_regions
      << ' ' << o.boundary.max_edge_length << ' '
      << o.boundary.max_curve_deviation << ' ' << o.refinement.max_edge_length
      << ' ' << o.refinement.max_steps << '\n';
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
inline void read_result(std::istream &in, MeshResult &r) {
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

inline void export_mesh(const MeshResult &result, const std::string &path) {
  validate_mesh_geometry(result.mesh);
  detail::validate_mesh_format(result.mesh);
  detail::validate_result(result);
  std::ofstream out(path);
  if (!out)
    throw std::runtime_error("Cannot create mesh file: " + path);
  detail::write_mesh(out, result.mesh, 3);
  detail::write_result(out, result);
  out.close();
  if (!out)
    throw std::runtime_error("Failed while writing mesh file: " + path);
}

inline void export_mesh(const TriangleMesh &mesh, const std::string &path) {
  validate_mesh_geometry(mesh);
  detail::validate_mesh_format(mesh);
  std::ofstream out(path);
  if (!out)
    throw std::runtime_error("Cannot create mesh file: " + path);
  detail::write_mesh(out, mesh, 2);
  out.close();
  if (!out)
    throw std::runtime_error("Failed while writing mesh file: " + path);
}

// Load a mesh saved by export_mesh. Rejects truncated, malformed, unsupported,
// or structurally inconsistent data before returning it to the caller.
inline MeshResult import_mesh_result(const std::string &path) {
  std::ifstream in(path);
  if (!in)
    throw std::runtime_error("Cannot open mesh file: " + path);
  std::string magic;
  int version = 0;
  if (!(in >> magic >> version) || magic != "PDE2D_MESH" ||
      (version != 1 && version != 2 && version != 3))
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
  if (version == 3)
    detail::read_result(in, result);
  std::string extra;
  if (in >> extra)
    throw std::runtime_error("Unexpected content after final mesh section");
  validate_mesh_geometry(mesh);
  detail::validate_mesh_format(mesh);
  if (version == 3)
    detail::validate_result(result);
  return result;
}

inline TriangleMesh import_mesh(const std::string &path) {
  return import_mesh_result(path).mesh;
}

} // namespace discretization
