#pragma once

// Helpers of the app's tests: the mesh as the page receives it (the
// summary and the binary arrays), decoded the page's way.

#include "session.hpp"

#include <bit>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <string_view>

namespace pde2d::app::test_support {

inline std::uint32_t read_uint32(std::string_view bytes, std::size_t at) {
  std::uint32_t value = 0;
  for (int byte = 3; byte >= 0; --byte)
    value = (value << 8) | static_cast<unsigned char>(
                               bytes[at + static_cast<std::size_t>(byte)]);
  return value;
}

inline double read_float64(std::string_view bytes, std::size_t at) {
  std::uint64_t value = 0;
  for (int byte = 7; byte >= 0; --byte)
    value = (value << 8) | static_cast<unsigned char>(
                               bytes[at + static_cast<std::size_t>(byte)]);
  return std::bit_cast<double>(value);
}

inline float read_float32(std::string_view bytes, std::size_t at) {
  return std::bit_cast<float>(read_uint32(bytes, at));
}

// {"nodes": [x0, y0, ...], "triangles": [a0, b0, c0, ...],
// "boundary_edges": [a0, b0, region0, ...], "region_names"} from the
// summary and the arrays.
inline Json decode_mesh(const Json &summary, std::string_view bytes) {
  const Json &arrays = summary["arrays"];
  if (bytes.size() != arrays["bytes"].get<std::size_t>())
    throw std::runtime_error("the mesh arrays have the wrong size");
  Json out = Json::object();
  const auto part = [&](std::string_view name) -> const Json & {
    for (const Json &item : arrays["parts"])
      if (item["name"] == name)
        return item;
    throw std::runtime_error("no part " + std::string(name));
  };
  Json nodes = Json::array();
  {
    const Json &p = part("nodes");
    for (std::size_t i = 0; i < p["count"].get<std::size_t>(); ++i)
      nodes.push_back(
          read_float64(bytes, p["offset"].get<std::size_t>() + 8 * i));
  }
  Json triangles = Json::array();
  {
    const Json &p = part("triangles");
    for (std::size_t i = 0; i < p["count"].get<std::size_t>(); ++i)
      triangles.push_back(
          read_uint32(bytes, p["offset"].get<std::size_t>() + 4 * i));
  }
  Json edges = Json::array();
  {
    const Json &p = part("boundary_edges");
    const Json &r = part("boundary_regions");
    for (std::size_t e = 0; e < r["count"].get<std::size_t>(); ++e) {
      edges.push_back(
          read_uint32(bytes, p["offset"].get<std::size_t>() + 8 * e));
      edges.push_back(
          read_uint32(bytes, p["offset"].get<std::size_t>() + 8 * e + 4));
      edges.push_back(
          read_uint32(bytes, r["offset"].get<std::size_t>() + 4 * e));
    }
  }
  out["nodes"] = std::move(nodes);
  out["triangles"] = std::move(triangles);
  out["boundary_edges"] = std::move(edges);
  out["region_names"] = summary["region_names"];
  return out;
}

// The mesh of the case, as the page gets it.
inline Json mesh_view(Session &session, const Json &case_json) {
  const Json summary = session.mesh(case_json)["mesh"];
  Json request = Json::object();
  request["mesh"] = summary["id"];
  return decode_mesh(summary, session.mesh_arrays(request));
}

} // namespace pde2d::app::test_support
