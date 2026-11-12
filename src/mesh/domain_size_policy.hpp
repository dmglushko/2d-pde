#pragma once

// Private to the mesh library.

#include "pde2d/geometry/curve.hpp"
#include "pde2d/geometry/point.hpp"

#include <vector>

namespace pde2d::mesh::detail {

// What "the size of a domain" means (see domain_size.hpp). Like the
// boundary split policy, this is a fixed choice of the library, not a user
// setting: domain_size_policy() returns the one in use, and changing it
// means changing that function.
class DomainSizePolicy {
public:
  virtual ~DomainSizePolicy() = default;
  // The size of a domain bounded by these exact curves (all of its
  // boundary). The list is not empty.
  virtual double of_curves(const std::vector<geometry::Curve> &curves) const = 0;
  // The size of a domain bounded by straight edges through these points
  // (all of its boundary nodes). The list is not empty.
  virtual double of_points(const std::vector<geometry::Point> &points) const = 0;
};

// The policy the library uses: BoundingBoxDiagonal in domain_size.cpp, the
// diagonal of the axis-aligned bounding box. It is easy to compute exactly
// for curves and within a factor sqrt(2) of the diameter, but unlike the
// diameter it changes when the domain is rotated.
const DomainSizePolicy &domain_size_policy();

} // namespace pde2d::mesh::detail
