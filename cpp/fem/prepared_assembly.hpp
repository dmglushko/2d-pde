#pragma once
#include "assembly.hpp"

namespace fem {
// Functional compatibility baseline for the prepared-assembly implementation.
// Own snapshots now; the preparation agent replaces repeated work with caches.
class PreparedAssembly {
  discretization::TriangleMesh mesh_;
  Problem problem_;
  AssemblyOptions options_;
public:
  PreparedAssembly(const discretization::TriangleMesh& mesh,const Problem& problem,
                   const AssemblyOptions& options={})
      :mesh_(mesh),problem_(problem),options_(options) {
    pde2d::validate_execution(options_.execution);
    discretization::validate_mesh_geometry(mesh_);
  }
  AssembledSystem assemble(double time=0,int dirichlet_derivative_order=0) const {
    if(dirichlet_derivative_order<0||dirichlet_derivative_order>2)
      throw std::invalid_argument("Dirichlet derivative order must be in [0,2]");
    return assembly_detail::assemble_impl(mesh_,problem_,time,options_,dirichlet_derivative_order);
  }
  const discretization::TriangleMesh& mesh() const {return mesh_;}
  const Problem& problem() const {return problem_;}
  const AssemblyOptions& options() const {return options_;}
};
} // namespace fem
