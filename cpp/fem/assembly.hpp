#pragma once

#include "problem.hpp"
#include "execution.hpp"
#include "quadrature.hpp"
#include "discretization/mesh_validation.hpp"
#include <Eigen/Sparse>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <vector>

namespace fem {
using SparseMatrix = Eigen::SparseMatrix<double>;
using Vector = Eigen::VectorXd;
struct AssemblyOptions {
  int quadrature_degree=2; double boundary_tolerance=1e-10;
  pde2d::ExecutionOptions execution{};
};
struct DirichletValue { Eigen::Index node; double value; };
struct AssembledSystem {
  SparseMatrix stiffness, first_mass, second_mass;
  Vector load;
  std::vector<DirichletValue> dirichlet;
  double time;
};

namespace assembly_detail {
inline bool zero(const Expr& e) { return fem::is_zero(e); }
inline double checked(double v) {
  if (!std::isfinite(v)) throw std::domain_error("Nonfinite FEM assembly contribution");
  return v;
}
inline Expr edge_expression(const Expr& e, double normal_x, double normal_y,
                            discretization::Point2 a, discretization::Point2 b) {
  // SymEngine substitution is simultaneous: x on the right is the new edge
  // coordinate s, not the original world coordinate. Restrict BEFORE deciding
  // equivalence or choosing a nonzero flux component for division.
  return SymEngine::expand(e.subs({
      {nx.get_basic(),Expr(normal_x).get_basic()},
      {ny.get_basic(),Expr(normal_y).get_basic()},
      {x.get_basic(),(Expr(a.x)+(b.x-a.x)*x).get_basic()},
      {y.get_basic(),(Expr(a.y)+(b.y-a.y)*x).get_basic()}}));
}
} // namespace assembly_detail

// Matrices are deliberately unconstrained, including for pure Neumann problems.
// Rows are test functions, columns are trial functions.
namespace assembly_detail {
inline AssembledSystem assemble_impl(const discretization::TriangleMesh& mesh,
    const Problem& problem, double time, const AssemblyOptions& options,
    int dirichlet_derivative_order=0) {
  using assembly_detail::checked;
  if (!std::isfinite(time) || !std::isfinite(options.boundary_tolerance) ||
      options.boundary_tolerance < 0)
    throw std::invalid_argument("Invalid FEM time or boundary tolerance");
  discretization::validate_mesh_geometry(mesh);
  const auto triangle_rule=triangle_quadrature(options.quadrature_degree);
  const auto edge_rule=edge_quadrature(options.quadrature_degree);
  if (mesh.nodes.size()>static_cast<std::size_t>(std::numeric_limits<int>::max()))
    throw std::invalid_argument("Mesh exceeds sparse matrix index capacity");
  const Eigen::Index count=static_cast<Eigen::Index>(mesh.nodes.size());
  AssembledSystem system{SparseMatrix(count,count),SparseMatrix(count,count),
      SparseMatrix(count,count),Vector::Zero(count),{},time};
  using Triplet=Eigen::Triplet<double>;
  std::vector<Triplet> stiffness, first_mass, second_mass;
  auto add=[](auto& entries, Eigen::Index i, Eigen::Index j, double value) {
    if (checked(value)!=0) entries.emplace_back(i,j,value);
  };
  const auto& eq=problem.equation;
  for (const auto* expression:{&eq.a_xx,&eq.a_xy,&eq.a_yy,&eq.b_x,&eq.b_y,
                               &eq.c,&eq.source,&eq.m1,&eq.m2})
    for (const auto& symbol:SymEngine::free_symbols(*expression->get_basic()))
      if (SymEngine::eq(*symbol,*nx.get_basic()) || SymEngine::eq(*symbol,*ny.get_basic()))
        throw std::invalid_argument("Interior coefficients cannot depend on boundary normals");
  const std::array<Evaluator,9> coefficients{
      Evaluator(eq.a_xx),Evaluator(eq.a_xy/2),Evaluator(eq.a_yy),
      Evaluator(eq.b_x-diff(eq.a_xx,x)-diff(eq.a_xy,y)/2),
      Evaluator(eq.b_y-diff(eq.a_yy,y)-diff(eq.a_xy,x)/2),
      Evaluator(eq.c),Evaluator(eq.source),Evaluator(eq.m1),Evaluator(eq.m2)};
  // Save the opposite vertex so normals do not depend on boundary edge storage.
  std::map<std::pair<std::size_t,std::size_t>,std::size_t> opposite;
  for (const auto& element:mesh.elements) {
    const auto a=mesh.nodes[element[0]], b=mesh.nodes[element[1]], c=mesh.nodes[element[2]];
    const double twice_area=checked((b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x));
    if (twice_area<=0) throw std::invalid_argument("Unrepresentable triangle area");
    const double area=twice_area/2;
    const std::array<double,3> gx{(b.y-c.y)/twice_area,(c.y-a.y)/twice_area,(a.y-b.y)/twice_area};
    const std::array<double,3> gy{(c.x-b.x)/twice_area,(a.x-c.x)/twice_area,(b.x-a.x)/twice_area};
    for (int k=0;k<3;++k)
      opposite[std::minmax(element[k],element[(k+1)%3])]=element[(k+2)%3];
    for (const auto& q:triangle_rule) {
      const auto& phi=q.barycentric;
      EvaluationPoint point{phi[0]*a.x+phi[1]*b.x+phi[2]*c.x,
          phi[0]*a.y+phi[1]*b.y+phi[2]*c.y,time};
      std::array<double,9> v;
      for (std::size_t k=0;k<v.size();++k) v[k]=coefficients[k](point);
      const double w=area*q.weight;
      for (int i=0;i<3;++i) {
        system.load[element[i]]=checked(system.load[element[i]]+w*v[6]*phi[i]);
        for (int j=0;j<3;++j) {
          add(stiffness,element[i],element[j],w*(-v[0]*gx[i]*gx[j]
              -v[1]*(gx[i]*gy[j]+gy[i]*gx[j])-v[2]*gy[i]*gy[j]
              +phi[i]*(v[3]*gx[j]+v[4]*gy[j])+v[5]*phi[i]*phi[j]));
          add(first_mass,element[i],element[j],w*v[7]*phi[i]*phi[j]);
          add(second_mass,element[i],element[j],w*v[8]*phi[i]*phi[j]);
        }
      }
    }
  }
  if (mesh.boundary_tags.size()!=mesh.boundary_edges.size())
    throw std::invalid_argument("Every boundary edge needs an explicit region tag and condition");
  std::set<std::string> present(mesh.boundary_tags.begin(),mesh.boundary_tags.end());
  if (present.contains("")) throw std::invalid_argument("Empty boundary region tag");
  std::map<std::string,const BoundaryCondition*> assigned;
  for (const auto& assignment:problem.boundaries) {
    if (assignment.regions.empty()) throw std::invalid_argument("Empty boundary assignment");
    for (const auto& region:assignment.regions) {
      if (!present.contains(region)) throw std::invalid_argument("Absent boundary region: "+region);
      if (!assigned.emplace(region,&assignment.condition).second)
        throw std::invalid_argument("Overlapping boundary assignments: "+region);
    }
  }
  std::map<Eigen::Index,double> prescribed;
  for (std::size_t edge=0;edge<mesh.boundary_edges.size();++edge) {
    const auto condition=assigned.find(mesh.boundary_tags[edge]);
    if (condition==assigned.end()) throw std::invalid_argument("Missing boundary assignment: "+mesh.boundary_tags[edge]);
    const auto& bc=*condition->second;
    const std::array<std::size_t,2> nodes{mesh.boundary_edges[edge].first,mesh.boundary_edges[edge].second};
    const auto a=mesh.nodes[nodes[0]], b=mesh.nodes[nodes[1]];
    const auto inside=mesh.nodes[opposite.at(std::minmax(nodes[0],nodes[1]))];
    const double dx=b.x-a.x, dy=b.y-a.y, length=checked(std::hypot(dx,dy));
    if (length==0) throw std::invalid_argument("Zero boundary edge length");
    double normal_x=dy/length, normal_y=-dx/length;
    if (normal_x*(inside.x-a.x)+normal_y*(inside.y-a.y)>0) {
      normal_x=-normal_x;normal_y=-normal_y;
    }
    const auto on_edge=[&](const Expr& e) { return assembly_detail::edge_expression(e,normal_x,normal_y,a,b); };
    const Expr alpha=on_edge(bc.alpha), gamma=on_edge(bc.gamma), value=on_edge(bc.value);
    const Expr flux_x=on_edge(-nx*eq.a_xx-ny*eq.a_xy/2);
    const Expr flux_y=on_edge(-nx*eq.a_xy/2-ny*eq.a_yy);
    const Expr extra_x=on_edge(bc.beta_normal*nx+bc.beta_x);
    const Expr extra_y=on_edge(bc.beta_normal*ny+bc.beta_y);
    Expr beta=on_edge(bc.beta_flux);
    if (!assembly_detail::zero(extra_x) || !assembly_detail::zero(extra_y)) {
      if (!assembly_detail::zero(extra_x*flux_y-extra_y*flux_x) ||
          (assembly_detail::zero(flux_x) && assembly_detail::zero(flux_y)))
        throw std::invalid_argument("Unsupported oblique/normal derivative boundary condition: derivative vector is not algebraically proportional to natural flux");
      // Project only AFTER proving exact proportionality. This quotient avoids
      // a removable 0/0 where one component vanishes but the flux vector does not.
      beta=beta+(extra_x*flux_x+extra_y*flux_y)/(flux_x*flux_x+flux_y*flux_y);
    }
    const Evaluator eval_alpha(alpha),eval_gamma(gamma),eval_value(value),eval_beta(beta);
    // Boundary expressions above now use x as their parameter on [0,1].
    auto point_at=[&](double s) { return EvaluationPoint{s,0,time,normal_x,normal_y}; };
    auto validate_divisor=[&](const Evaluator& evaluator,const char* name) {
      std::vector<double> samples{0,.5,1};
      for(const auto& q:edge_rule) samples.push_back(q.coordinate);
      std::sort(samples.begin(),samples.end());
      double previous=evaluator(point_at(samples.front()));
      for(const double s:samples) {
        const double current=evaluator(point_at(s));
        if(current==0 || std::signbit(current)!=std::signbit(previous))
          throw std::invalid_argument(std::string(name)+" vanishes or changes sign on boundary");
        previous=current;
      }
      // This detects sampled zeros and sign changes, not every possible root
      // of arbitrary symbolic functions. Nonvanishing data remain a precondition.
    };
    if (assembly_detail::zero(beta)) {
      if (!assembly_detail::zero(gamma)) throw std::invalid_argument("Derivative-free dynamic boundary conditions are unsupported");
      if (assembly_detail::zero(alpha)) throw std::invalid_argument("Boundary condition has no nonzero operator");
      validate_divisor(eval_alpha,"Dirichlet alpha");
      Expr prescribed_value=value/alpha;
      for(int order=0;order<dirichlet_derivative_order;++order)
        prescribed_value=diff(prescribed_value,t);
      const Evaluator eval_prescribed(prescribed_value);
      for (int k=0;k<2;++k) {
        const auto point=point_at(k);
        const double av=eval_alpha(point);
        if (av==0) throw std::invalid_argument("Dirichlet alpha vanishes on boundary");
        const double g=eval_prescribed(point);
        auto [it,inserted]=prescribed.emplace(nodes[k],g);
        if (!inserted && std::abs(it->second-g)>options.boundary_tolerance*std::max({1.,std::abs(it->second),std::abs(g)}))
          throw std::invalid_argument("Contradictory corner Dirichlet values");
      }
      // Also validate data inside each edge, even though P1 constraints use nodes.
      for (const auto& q:edge_rule) {
        const auto p=point_at(q.coordinate);
        if (eval_alpha(p)==0) throw std::invalid_argument("Dirichlet alpha vanishes on boundary");
        (void)eval_value(p);
      }
    } else {
      validate_divisor(eval_beta,"Natural flux multiplier");
      for (int endpoint=0;endpoint<2;++endpoint) {
        const auto p=point_at(endpoint);
        if (eval_beta(p)==0)
          throw std::invalid_argument("Natural flux multiplier vanishes on boundary");
        (void)eval_alpha(p); (void)eval_gamma(p); (void)eval_value(p);
      }
      for (const auto& q:edge_rule) {
        const auto point=point_at(q.coordinate);
        const double bv=eval_beta(point);
        if (bv==0) throw std::invalid_argument("Natural flux multiplier vanishes on boundary");
        const double av=checked(eval_alpha(point)/bv),gv=checked(eval_gamma(point)/bv),f=checked(eval_value(point)/bv);
        const double w=length*q.weight;
        const std::array<double,2> phi{1-q.coordinate,q.coordinate};
        for (int i=0;i<2;++i) {
          system.load[nodes[i]]=checked(system.load[nodes[i]]+w*f*phi[i]);
          for (int j=0;j<2;++j) {
            add(stiffness,nodes[i],nodes[j],w*av*phi[i]*phi[j]);
            add(first_mass,nodes[i],nodes[j],w*gv*phi[i]*phi[j]);
          }
        }
      }
    }
  }
  for (const auto& [node,value]:prescribed) system.dirichlet.push_back({node,value});
  system.stiffness.setFromTriplets(stiffness.begin(),stiffness.end());
  system.first_mass.setFromTriplets(first_mass.begin(),first_mass.end());
  system.second_mass.setFromTriplets(second_mass.begin(),second_mass.end());
  for (const auto* matrix:{&system.stiffness,&system.first_mass,&system.second_mass})
    for (int k=0;k<matrix->nonZeros();++k) checked(matrix->valuePtr()[k]);
  return system;
}
} // namespace assembly_detail

inline AssembledSystem assemble(const discretization::TriangleMesh& mesh,
    const Problem& problem, double time=0, const AssemblyOptions& options={}) {
  return assembly_detail::assemble_impl(mesh,problem,time,options);
}
} // namespace fem
