#pragma once
// Private part of PreparedAssembly (assembly.hpp): serial scatter of local
// element and edge contributions into the global system. The fixed
// element-then-edge order also fixes Eigen's duplicate summation order, so
// serial and parallel evaluation give bit-identical matrices.
#include "fem/detail/assembly_edges.hpp"
#include "fem/detail/assembly_elements.hpp"
#include "fem/system.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <map>
#include <stdexcept>
#include <type_traits>
#include <variant>
#include <vector>

namespace pde2d::fem::assembly_detail {
inline AssembledSystem scatter(std::size_t node_count, double time,
                               const std::vector<PreparedElement> &elements,
                               const std::vector<LocalBlock> &blocks,
                               const std::vector<PreparedEdge> &edges,
                               const std::vector<EdgeResult> &boundary,
                               double boundary_tolerance) {
  constexpr std::size_t element_nodes = LocalBlock::nodes;
  constexpr std::size_t edge_nodes =
      std::tuple_size_v<decltype(PreparedEdge::nodes)>;
  const auto count = static_cast<Eigen::Index>(node_count);
  AssembledSystem result{SparseMatrix(count, count),
                         SparseMatrix(count, count),
                         SparseMatrix(count, count),
                         Vector::Zero(count),
                         {},
                         time};
  std::vector<Eigen::Triplet<double>> stiffness, first, second;
  stiffness.reserve(element_nodes * element_nodes * elements.size() +
                    edge_nodes * edge_nodes * edges.size());
  first.reserve(stiffness.capacity());
  second.reserve(element_nodes * element_nodes * elements.size());
  auto add = [](auto &v, std::size_t i, std::size_t j, double a) {
    if (checked(a) != 0)
      v.emplace_back(i, j, a);
  };
  for (std::size_t n = 0; n < elements.size(); ++n) {
    const auto &e = elements[n];
    const auto &a = blocks[n];
    const auto &b = e.constant;
    for (std::size_t i = 0; i < element_nodes; ++i) {
      result.load[e.nodes[i]] =
          checked(result.load[e.nodes[i]] + a.load[i] + b.load[i]);
      for (std::size_t j = 0; j < element_nodes; ++j) {
        const auto ij = LocalBlock::index(i, j);
        add(stiffness, e.nodes[i], e.nodes[j],
            a.stiffness[ij] + b.stiffness[ij]);
        add(first, e.nodes[i], e.nodes[j], a.first[ij] + b.first[ij]);
        add(second, e.nodes[i], e.nodes[j], a.second[ij] + b.second[ij]);
      }
    }
  }
  std::map<Eigen::Index, double> prescribed;
  for (std::size_t n = 0; n < edges.size(); ++n) {
    const auto &e = edges[n];
    const auto &a = boundary[n];
    std::visit(
        [&]<class Form>(const Form &) {
          for (std::size_t i = 0; i < edge_nodes; ++i) {
            if constexpr (std::is_same_v<Form, PreparedDirichletEdge>) {
              double g = a.values[i];
              auto [it, inserted] = prescribed.emplace(e.nodes[i], g);
              if (!inserted &&
                  std::abs(it->second - g) >
                      boundary_tolerance *
                          std::max({1., std::abs(it->second), std::abs(g)}))
                throw std::invalid_argument(
                    "Contradictory corner Dirichlet values");
            } else if constexpr (std::is_same_v<Form, PreparedNaturalEdge>) {
              result.load[e.nodes[i]] =
                  checked(result.load[e.nodes[i]] + a.block.load[i]);
              for (std::size_t j = 0; j < edge_nodes; ++j) {
                const auto ij = LocalBlock::index(i, j);
                add(stiffness, e.nodes[i], e.nodes[j], a.block.stiffness[ij]);
                add(first, e.nodes[i], e.nodes[j], a.block.first[ij]);
              }
            } else
              static_assert(classification_detail::unhandled_form<Form>,
                            "Scatter every prepared edge form");
          }
        },
        e.form);
  }
  for (const auto &[node, value] : prescribed)
    result.dirichlet.push_back({node, value});
  result.stiffness.setFromTriplets(stiffness.begin(), stiffness.end());
  result.first_mass.setFromTriplets(first.begin(), first.end());
  result.second_mass.setFromTriplets(second.begin(), second.end());
  for (const auto *matrix :
       {&result.stiffness, &result.first_mass, &result.second_mass})
    for (int k = 0; k < matrix->nonZeros(); ++k)
      checked(matrix->valuePtr()[k]);
  return result;
}
} // namespace pde2d::fem::assembly_detail
