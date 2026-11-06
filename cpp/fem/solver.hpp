#pragma once
#include "assembly.hpp"
#include "timing.hpp"
#include <Eigen/SparseLU>
#include <Eigen/SparseQR>
#include <Eigen/SparseCholesky>
#include <Eigen/IterativeLinearSolvers>
#include <unsupported/Eigen/IterativeSolvers>
#include <algorithm>
#include <string>
#include <utility>

namespace fem {
enum class LinearSolver { SparseLU, ConjugateGradient, GMRES };
struct SolverOptions {
  LinearSolver method=LinearSolver::SparseLU;
  double relative_tolerance=1e-10, absolute_tolerance=1e-12;
  int max_iterations=2000, restart=40;
};
struct SolverReport {
  bool converged=false;
  int iterations=0;
  double residual_norm=std::numeric_limits<double>::infinity();
  double relative_residual=std::numeric_limits<double>::infinity();
  std::string message;
  SolverTimings timings;
};
struct LinearSolution { Vector values; SolverReport report; };
namespace solver_detail {
inline void validate(const SolverOptions& o) {
  if (!std::isfinite(o.relative_tolerance)||!std::isfinite(o.absolute_tolerance)||
      o.relative_tolerance<0||o.absolute_tolerance<0||
      (o.relative_tolerance==0&&o.absolute_tolerance==0)||o.max_iterations<=0||o.restart<=0)
    throw std::invalid_argument("Invalid linear solver options");
  switch(o.method) {
    case LinearSolver::SparseLU: case LinearSolver::ConjugateGradient: case LinearSolver::GMRES: break;
    default: throw std::invalid_argument("Unsupported linear solver");
  }
}
inline void validate(const SparseMatrix& a,const Vector& b) {
  if(a.rows()!=a.cols()||a.rows()!=b.size()||a.rows()==0)
    throw std::invalid_argument("Linear system must be nonempty and square with matching RHS");
  if(!b.allFinite()) throw std::invalid_argument("Nonfinite linear RHS");
  // An uncompressed Eigen matrix may have unused slots between columns.
  for(int column=0;column<a.outerSize();++column)
    for(SparseMatrix::InnerIterator entry(a,column);entry;++entry)
      if(!std::isfinite(entry.value())) throw std::invalid_argument("Nonfinite linear matrix");
}
}
inline std::pair<SparseMatrix,Vector> constrain(const SparseMatrix& a,const Vector& b,
    const std::vector<DirichletValue>& constraints) {
  solver_detail::validate(a,b);
  std::map<Eigen::Index,double> prescribed;
  for(const auto& d:constraints) {
    if(d.node<0||d.node>=a.rows()||!std::isfinite(d.value))
      throw std::invalid_argument("Invalid Dirichlet constraint");
    auto [it,inserted]=prescribed.emplace(d.node,d.value);
    if(!inserted&&it->second!=d.value) throw std::invalid_argument("Conflicting Dirichlet constraints");
  }
  Vector rhs=b;
  std::vector<Eigen::Triplet<double>> entries;
  for(int k=0;k<a.outerSize();++k) for(SparseMatrix::InnerIterator it(a,k);it;++it) {
    if(prescribed.contains(it.row())) continue;
    const auto p=prescribed.find(it.col());
    if(p!=prescribed.end()) rhs[it.row()]-=it.value()*p->second;
    else entries.emplace_back(it.row(),it.col(),it.value());
  }
  for(const auto& [i,g]:prescribed) { entries.emplace_back(i,i,1);rhs[i]=g; }
  SparseMatrix result(a.rows(),a.cols());result.setFromTriplets(entries.begin(),entries.end());
  if(!rhs.allFinite()) throw std::overflow_error("Nonfinite Dirichlet lifting");
  return {std::move(result),std::move(rhs)};
}
inline LinearSolution solve_linear(const SparseMatrix& input,const Vector& b,const SolverOptions& o={}) {
  const auto started=timing_detail::Clock::now();
  solver_detail::validate(o);solver_detail::validate(input,b);
  SparseMatrix a=input;a.makeCompressed();
  LinearSolution result{Vector::Zero(b.size()),{}};
  auto& r=result.report;
  auto finish=[&]() {r.timings.total_seconds=timing_detail::seconds(started);return std::move(result);};
  r.residual_norm=b.stableNorm();
  r.relative_residual=r.residual_norm>0?1.:0.;
  r.timings.validation_seconds=timing_detail::seconds(started);
  // A rank check also catches compatible singular systems with a zero RHS.
  Eigen::SparseQR<SparseMatrix,Eigen::COLAMDOrdering<int>> rank;
  auto phase=timing_detail::Clock::now();
  rank.compute(a);
  r.timings.rank_check_seconds=timing_detail::seconds(phase);
  if(rank.info()!=Eigen::Success||rank.rank()!=a.rows()) {
    r.message="Singular or numerically rank-deficient linear system";return finish();
  }
  const double bn=b.stableNorm();
  const double target=std::max(o.absolute_tolerance,o.relative_tolerance*bn);
  const double tolerance=bn>0?target/bn:o.relative_tolerance;
  if(o.method==LinearSolver::ConjugateGradient) {
    phase=timing_detail::Clock::now();
    const SparseMatrix skew=a-SparseMatrix(a.transpose());
    if(skew.norm()>1e-12*a.norm()) {
      r.timings.factorization_seconds=timing_detail::seconds(phase);
      r.message="ConjugateGradient requires a symmetric positive definite matrix";return finish();
    }
    Eigen::SimplicialLDLT<SparseMatrix> ldlt;ldlt.compute(a);
    if(ldlt.info()!=Eigen::Success||!ldlt.vectorD().allFinite()||ldlt.vectorD().minCoeff()<=0) {
      r.timings.factorization_seconds=timing_detail::seconds(phase);
      r.message="ConjugateGradient requires a positive definite matrix";return finish();
    }
    Eigen::ConjugateGradient<SparseMatrix,Eigen::Lower|Eigen::Upper> cg;
    cg.setMaxIterations(o.max_iterations);cg.setTolerance(tolerance);cg.compute(a);
    r.timings.factorization_seconds=timing_detail::seconds(phase);
    phase=timing_detail::Clock::now();
    result.values=cg.solve(b);r.iterations=static_cast<int>(cg.iterations());
    r.timings.solve_seconds=timing_detail::seconds(phase);
  } else if(o.method==LinearSolver::GMRES) {
    phase=timing_detail::Clock::now();
    Eigen::GMRES<SparseMatrix> gmres;
    gmres.setMaxIterations(o.max_iterations);gmres.set_restart(o.restart);gmres.setTolerance(tolerance);
    gmres.compute(a);r.timings.factorization_seconds=timing_detail::seconds(phase);
    phase=timing_detail::Clock::now();
    result.values=gmres.solve(b);r.iterations=static_cast<int>(gmres.iterations());
    r.timings.solve_seconds=timing_detail::seconds(phase);
  } else {
    phase=timing_detail::Clock::now();
    Eigen::SparseLU<SparseMatrix> lu;lu.compute(a);
    r.timings.factorization_seconds=timing_detail::seconds(phase);
    if(lu.info()!=Eigen::Success) {r.message="SparseLU factorization failed";return finish();}
    phase=timing_detail::Clock::now();
    result.values=lu.solve(b);r.iterations=1;
    r.timings.solve_seconds=timing_detail::seconds(phase);
    if(lu.info()!=Eigen::Success) {r.message="SparseLU solve failed";return finish();}
  }
  phase=timing_detail::Clock::now();
  r.residual_norm=(a*result.values-b).stableNorm();
  r.relative_residual=bn>0?r.residual_norm/bn:r.residual_norm;
  r.converged=result.values.allFinite()&&std::isfinite(r.residual_norm)&&r.residual_norm<=target;
  r.message=r.converged?"Converged":"Linear solver did not converge to requested true residual tolerance";
  r.timings.residual_seconds=timing_detail::seconds(phase);
  return finish();
}
inline LinearSolution solve_stationary(const discretization::TriangleMesh& mesh,const Problem& problem,
    const AssemblyOptions& assembly={},const SolverOptions& solver={}) {
  auto s=assemble(mesh,problem,0,assembly);
  auto [a,b]=constrain(s.stiffness,s.load,s.dirichlet);
  auto solution=solve_linear(a,b,solver);
  if(!solution.report.converged) throw std::runtime_error("Stationary solve: "+solution.report.message);
  return solution;
}
} // namespace fem
