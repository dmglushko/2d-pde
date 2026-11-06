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
// Owns its matrix snapshot and all matrix-dependent preparation. Calls to solve
// are sequential; each returned report contains only that RHS's work.
class PreparedLinearSolver {
  SparseMatrix matrix_;
  SolverOptions options_;
  SolverTimings preparation_;
  std::string failure_;
  Eigen::SparseLU<SparseMatrix> lu_;
  Eigen::ConjugateGradient<SparseMatrix,Eigen::Lower|Eigen::Upper> cg_;
  Eigen::GMRES<SparseMatrix> gmres_;
public:
  PreparedLinearSolver(const SparseMatrix& input,const SolverOptions& options={})
      :options_(options) {
    const auto started=timing_detail::Clock::now();
    matrix_=input;
    solver_detail::validate(options_);
    solver_detail::validate(matrix_,Vector::Zero(matrix_.rows()));
    matrix_.makeCompressed();
    preparation_.preparations=1;
    preparation_.validation_seconds=timing_detail::seconds(started);
    auto phase=timing_detail::Clock::now();
    Eigen::SparseQR<SparseMatrix,Eigen::COLAMDOrdering<int>> rank;
    rank.compute(matrix_);++preparation_.rank_checks;
    preparation_.rank_check_seconds=timing_detail::seconds(phase);
    if(rank.info()!=Eigen::Success||rank.rank()!=matrix_.rows())
      failure_="Singular or numerically rank-deficient linear system";
    else {
      phase=timing_detail::Clock::now();
      ++preparation_.factorizations;
      if(options_.method==LinearSolver::ConjugateGradient) {
        const SparseMatrix skew=matrix_-SparseMatrix(matrix_.transpose());
        if(skew.norm()>1e-12*matrix_.norm())
          failure_="ConjugateGradient requires a symmetric positive definite matrix";
        else {
          Eigen::SimplicialLDLT<SparseMatrix> ldlt;ldlt.compute(matrix_);
          if(ldlt.info()!=Eigen::Success||!ldlt.vectorD().allFinite()||ldlt.vectorD().minCoeff()<=0)
            failure_="ConjugateGradient requires a positive definite matrix";
          else {cg_.setMaxIterations(options_.max_iterations);cg_.compute(matrix_);}
        }
      } else if(options_.method==LinearSolver::GMRES) {
        gmres_.setMaxIterations(options_.max_iterations);gmres_.set_restart(options_.restart);
        gmres_.compute(matrix_);
      } else {
        lu_.compute(matrix_);
        if(lu_.info()!=Eigen::Success) failure_="SparseLU factorization failed";
      }
      preparation_.factorization_seconds=timing_detail::seconds(phase);
    }
    preparation_.total_seconds=timing_detail::seconds(started);
  }
  const SparseMatrix& matrix() const noexcept {return matrix_;}
  const SolverOptions& options() const noexcept {return options_;}
  const SolverTimings& preparation_timings() const noexcept {return preparation_;}
  // Compare actual coefficients (including explicit zeros), never identities or
  // merely sparsity. A conservative false miss is safe; an approximate hit isn't.
  bool matches(const SparseMatrix& a,const SolverOptions& o) const {
    if(o.method!=options_.method||o.relative_tolerance!=options_.relative_tolerance||
       o.absolute_tolerance!=options_.absolute_tolerance||o.max_iterations!=options_.max_iterations||
       o.restart!=options_.restart||a.rows()!=matrix_.rows()||a.cols()!=matrix_.cols()) return false;
    for(int col=0;col<a.outerSize();++col) {
      SparseMatrix::InnerIterator left(a,col),right(matrix_,col);
      for(;left&&right;++left,++right)
        if(left.row()!=right.row()||left.value()!=right.value()) return false;
      if(left||right) return false;
    }
    return true;
  }
  LinearSolution solve(const Vector& b) {
    const auto started=timing_detail::Clock::now();
    if(b.size()!=matrix_.rows()||!b.allFinite()) throw std::invalid_argument("Invalid linear RHS");
    LinearSolution result{Vector::Zero(b.size()),{}};auto& r=result.report;
    r.timings.rhs_solves=1;
    const double bn=b.stableNorm();
    r.residual_norm=bn;r.relative_residual=bn>0?1.:0.;
    r.timings.validation_seconds=timing_detail::seconds(started);
    auto finish=[&]() {r.timings.total_seconds=timing_detail::seconds(started);return std::move(result);};
    if(!failure_.empty()) {r.message=failure_;return finish();}
    const double target=std::max(options_.absolute_tolerance,options_.relative_tolerance*bn);
    const double tolerance=bn>0?target/bn:options_.relative_tolerance;
    auto phase=timing_detail::Clock::now();
    if(options_.method==LinearSolver::ConjugateGradient) {
      cg_.setTolerance(tolerance);result.values=cg_.solve(b);r.iterations=static_cast<int>(cg_.iterations());
    } else if(options_.method==LinearSolver::GMRES) {
      gmres_.setTolerance(tolerance);result.values=gmres_.solve(b);r.iterations=static_cast<int>(gmres_.iterations());
    } else {
      result.values=lu_.solve(b);r.iterations=1;
      if(lu_.info()!=Eigen::Success) {
        r.timings.solve_seconds=timing_detail::seconds(phase);
        r.message="SparseLU solve failed";return finish();
      }
    }
    r.timings.solve_seconds=timing_detail::seconds(phase);
    phase=timing_detail::Clock::now();
    r.residual_norm=(matrix_*result.values-b).stableNorm();
    r.relative_residual=bn>0?r.residual_norm/bn:r.residual_norm;
    r.converged=result.values.allFinite()&&std::isfinite(r.residual_norm)&&r.residual_norm<=target;
    r.message=r.converged?"Converged":"Linear solver did not converge to requested true residual tolerance";
    r.timings.residual_seconds=timing_detail::seconds(phase);
    return finish();
  }
};
inline LinearSolution solve_linear(const SparseMatrix& input,const Vector& b,const SolverOptions& o={}) {
  const auto started=timing_detail::Clock::now();
  solver_detail::validate(o);solver_detail::validate(input,b);
  PreparedLinearSolver prepared(input,o);
  auto result=prepared.solve(b);
  result.report.timings+=prepared.preparation_timings();
  result.report.timings.total_seconds=timing_detail::seconds(started);
  return result;
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
