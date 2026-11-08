#pragma once
// Independent constrained-Delaunay verifier for meshing tests: enumerate
// triangle pairs, derive each shared diagonal and test the opposing vertex
// directly with the robust predicates. It consults no mesher state.
#include "discretization/geometry_base.hpp"
#include "discretization/predicates.hpp"
#include "support/check.hpp"
#include <algorithm>
#include <cstddef>
#include <utility>
#include <vector>

namespace pde2d_test {
inline void independent_cdt(const pde2d::discretization::TriangleMesh &m) {
  namespace predicates = pde2d::discretization::predicates;
  for (std::size_t i = 0; i < m.elements.size(); ++i)
    for (std::size_t j = i + 1; j < m.elements.size(); ++j) {
      auto t = m.elements[i], u = m.elements[j];
      std::vector<std::size_t> shared;
      std::size_t c = 0, d = 0;
      for (auto a : t) {
        if (std::find(u.begin(), u.end(), a) != u.end())
          shared.push_back(a);
        else
          c = a;
      }
      if (shared.size() != 2)
        continue;
      for (auto a : u)
        if (a != shared[0] && a != shared[1])
          d = a;
      auto a = shared[0], b = shared[1];
      if (predicates::orient(m.nodes[c], m.nodes[d], m.nodes[a]) *
              predicates::orient(m.nodes[c], m.nodes[d], m.nodes[b]) >=
          0)
        continue;
      if (predicates::orient(m.nodes[a], m.nodes[b], m.nodes[c]) < 0)
        std::swap(a, b);
      check(predicates::incircle(m.nodes[a], m.nodes[b], m.nodes[c],
                                 m.nodes[d]) <= 0,
            "Independent CDT failure");
    }
}
} // namespace pde2d_test
