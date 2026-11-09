#pragma once
// Private (mesh_detail) part of triangulate.hpp: one unit per
// InteriorAlgorithm, mirroring the boundary policies of
// refinement_policy.hpp. A unit gathers everything its algorithm decides:
//
//   id                    the wire ID (mesh_types.hpp; its report name is
//                         interior_algorithm_name(id))
//   capabilities          InteriorAlgorithmCapabilities (mesh_types.hpp)
//   size_refinement_unsupported, refinement_unsupported
//                         the messages of requests the capabilities exclude
//                         (stable: tests match them)
//   generate(seed, opts)  the mesh of a seeded result (ear clipping of the
//                         sampled polygon, see seeded_result)
//   refine(result, opts)  only when capabilities.supports_refinement
//
// with_interior_algorithm() is the only dispatch on InteriorAlgorithm;
// InteriorAlgorithmUnit checks at compile time that every unit is complete.
// Compatibility errors come from the capabilities (require_generation,
// refine_with_algorithm), never from comparisons with enum values.
#include "../mesh_types.hpp"
#include "ear_clipping.hpp"
#include "refinement_loop.hpp"
#include <concepts>
#include <stdexcept>
#include <string>
#include <utility>

namespace pde2d::discretization::mesh_detail {
// The seed every unit starts from: the ear-clipped polygon with the boundary
// context and options of the request (tags cleared without regions).
inline MeshResult seeded_result(const DiscretizedBoundary &boundary,
                                const MeshOptions &options) {
  MeshResult result;
  result.mesh = seed_mesh(boundary.boundary);
  result.boundary_context = boundary.context;
  result.options = options;
  if (!options.preserve_regions)
    result.mesh.boundary_tags.clear();
  return result;
}

// InteriorAlgorithm::EarClipping: the seed triangulation itself, without
// interior vertices or legalization.
struct EarClippingAlgorithm {
  static constexpr InteriorAlgorithm id = InteriorAlgorithm::EarClipping;
  static constexpr InteriorAlgorithmCapabilities capabilities{};
  static constexpr const char *size_refinement_unsupported =
      "Ear clipping has no interior refinement; use constrained Delaunay or "
      "refine_mesh with subdivision";
  static constexpr const char *refinement_unsupported =
      "refine_mesh uses constrained Delaunay; use subdivide_mesh for "
      "subdivision";
  static MeshResult generate(MeshResult result, const MeshOptions &options) {
    validate_mesh_geometry(result.mesh);
    if (result.boundary_context)
      validate_boundary_context(result.mesh, *result.boundary_context,
                                options.preserve_regions,
                                options.boundary.preserve_corners);
    return result;
  }
};

// InteriorAlgorithm::ConstrainedDelaunay: legalize the seed under the
// constrained Delaunay criterion and refine it with midpoint insertion
// (refinement_loop.hpp). No minimum-angle guarantee.
struct ConstrainedDelaunayAlgorithm {
  static constexpr InteriorAlgorithm id =
      InteriorAlgorithm::ConstrainedDelaunay;
  static constexpr InteriorAlgorithmCapabilities capabilities{
      .supports_size_target = true,
      .supports_refinement = true,
      .constrained_delaunay = true,
      .requires_boundary_context = false};
  static constexpr const char *size_refinement_unsupported = "";
  static constexpr const char *refinement_unsupported = "";
  static MeshResult generate(MeshResult result, const MeshOptions &options) {
    return refine_result(std::move(result), options, false);
  }
  static MeshResult refine(MeshResult result, const MeshOptions &options) {
    return refine_result(std::move(result), options, false);
  }
};

template <class Unit>
concept InteriorAlgorithmUnit =
    requires(MeshResult r, const MeshOptions &o) {
      { Unit::id } -> std::convertible_to<InteriorAlgorithm>;
      {
        Unit::capabilities
      } -> std::convertible_to<InteriorAlgorithmCapabilities>;
      {
        Unit::size_refinement_unsupported
      } -> std::convertible_to<const char *>;
      { Unit::refinement_unsupported } -> std::convertible_to<const char *>;
      { Unit::generate(std::move(r), o) } -> std::same_as<MeshResult>;
    } && (!Unit::capabilities.supports_refinement ||
          requires(MeshResult r, const MeshOptions &o) {
            { Unit::refine(std::move(r), o) } -> std::same_as<MeshResult>;
          });

// The only dispatch: calls f(Unit{}) for the unit of `algorithm`.
template <class F>
decltype(auto) with_interior_algorithm(InteriorAlgorithm algorithm, F &&f) {
  static_assert(InteriorAlgorithmUnit<EarClippingAlgorithm> &&
                InteriorAlgorithmUnit<ConstrainedDelaunayAlgorithm>);
  switch (algorithm) {
  case InteriorAlgorithm::EarClipping:
    return f(EarClippingAlgorithm{});
  case InteriorAlgorithm::ConstrainedDelaunay:
    return f(ConstrainedDelaunayAlgorithm{});
  }
  throw std::invalid_argument("Unknown interior algorithm");
}

// A size refinement request the unit cannot generate throws its message.
template <class Unit> void require_generation(const MeshOptions &o) {
  if (o.refinement.max_edge_length > 0 &&
      !Unit::capabilities.supports_size_target)
    throw std::invalid_argument(Unit::size_refinement_unsupported);
}
inline void validate_options(const MeshOptions &o) {
  validateMeshOptionValues(o);
  with_interior_algorithm(
      o.algorithm, [&](auto unit) { require_generation<decltype(unit)>(o); });
}
inline MeshResult generate_with_algorithm(const DiscretizedBoundary &boundary,
                                          const MeshOptions &options) {
  validate_options(options);
  return with_interior_algorithm(
      options.algorithm, [&](auto unit) -> MeshResult {
        using Unit = decltype(unit);
        if (Unit::capabilities.requires_boundary_context && !boundary.context)
          throw std::invalid_argument(
              std::string(interior_algorithm_name(Unit::id)) +
              " requires analytical boundary context");
        return Unit::generate(seeded_result(boundary, options), options);
      });
}
// Refinement of an existing mesh by the unit of options.algorithm. The
// capability check comes before any other validation.
inline MeshResult refine_with_algorithm(MeshResult result,
                                        const MeshOptions &options) {
  return with_interior_algorithm(
      options.algorithm, [&](auto unit) -> MeshResult {
        using Unit = decltype(unit);
        if constexpr (Unit::capabilities.supports_refinement)
          return Unit::refine(std::move(result), options);
        else
          throw std::invalid_argument(Unit::refinement_unsupported);
      });
}
} // namespace pde2d::discretization::mesh_detail
