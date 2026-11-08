#pragma once
// Report category of a boundary region (analysis_types.hpp), kept apart so
// the boundary edge-form units (detail/edge_forms/) can name the category
// their form reports as without depending on the analysis types.
namespace pde2d::fem {
// Of all edges of one boundary region (EdgeForm per edge). Mixed: more than
// one category. Rejected: the condition cannot be classified on some edge
// (reason holds the message). New values go at the end.
enum class BoundaryForm { Dirichlet, Natural, Mixed, Rejected, Oblique };

inline const char *boundary_form_name(BoundaryForm form) {
  switch (form) {
  case BoundaryForm::Dirichlet:
    return "Dirichlet";
  case BoundaryForm::Natural:
    return "Natural";
  case BoundaryForm::Mixed:
    return "Mixed";
  case BoundaryForm::Rejected:
    return "Rejected";
  case BoundaryForm::Oblique:
    return "Oblique";
  }
  return "Invalid";
}
} // namespace pde2d::fem
