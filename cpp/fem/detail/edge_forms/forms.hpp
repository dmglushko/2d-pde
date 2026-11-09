#pragma once
// The list of boundary edge forms: one EdgeForm alternative per unit
// (interface.hpp). Adding a form means adding its unit header, including it
// here and appending its struct to EdgeForm; the static_assert below fails
// for an alternative without a complete unit.
#include "fem/detail/edge_forms/dirichlet.hpp"
#include "fem/detail/edge_forms/interface.hpp"
#include "fem/detail/edge_forms/natural.hpp"
#include "fem/detail/edge_forms/oblique.hpp"
#include <variant>

namespace pde2d::fem {
// One alternative per supported boundary form.
using EdgeForm = std::variant<DirichletEdge, NaturalEdge, ObliqueEdge>;
} // namespace pde2d::fem

namespace pde2d::fem::edge_form_detail {
template <class> struct PreparedForms;
template <class... Forms> struct PreparedForms<std::variant<Forms...>> {
  static_assert((EdgeFormUnit<Forms> && ...),
                "Every EdgeForm alternative needs its unit: a complete "
                "edge_form_detail::Unit<Form> (see edge_forms/interface.hpp)");
  using type = std::variant<typename Unit<Forms>::Prepared...>;
};
// Prepared counterpart of EdgeForm, alternative by alternative.
using PreparedEdgeForm = PreparedForms<EdgeForm>::type;

// The unit of a form or of its prepared data.
template <class Prepared> using unit_of = Unit<typename Prepared::Form>;
} // namespace pde2d::fem::edge_form_detail
