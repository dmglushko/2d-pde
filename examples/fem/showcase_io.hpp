#pragma once
#include "fem/assembly.hpp"
#include <ostream>
#include <string>
namespace showcase_io {
inline void write_vector(std::ostream &out, const pde2d::fem::Vector &values) {
  out << '[';
  for (Eigen::Index i = 0; i < values.size(); ++i) {
    if (i)
      out << ',';
    out << values[i];
  }
  out << ']';
}
inline void quote(std::ostream &out, const std::string &text) {
  out << '"';
  for (char c : text) {
    if (c == '"' || c == '\\')
      out << '\\';
    if (c == '\n')
      out << "\\n";
    else
      out << c;
  }
  out << '"';
}
inline void write_mesh(std::ostream &out,
                       const pde2d::discretization::TriangleMesh &mesh) {
  out << "\"nodes\":[";
  for (std::size_t i = 0; i < mesh.nodes.size(); ++i) {
    if (i)
      out << ',';
    out << '[' << mesh.nodes[i].x << ',' << mesh.nodes[i].y << ']';
  }
  out << "],\"triangles\":[";
  for (std::size_t i = 0; i < mesh.elements.size(); ++i) {
    if (i)
      out << ',';
    const auto &tri = mesh.elements[i];
    out << '[' << tri[0] << ',' << tri[1] << ',' << tri[2] << ']';
  }
  out << ']';
}
} // namespace showcase_io
