#pragma once
#include "solver.hpp"

namespace fem {
enum class TimeMethod { BackwardEuler, CrankNicolson, Newmark };
struct TimeOptions {
  TimeMethod method=TimeMethod::BackwardEuler;
  double start=0,end=1,step=.01;
  AssemblyOptions assembly;
  SolverOptions solver;
};
struct Frame { double time; Vector values; Vector velocity; };
struct Evolution {
  std::vector<Frame> frames; std::vector<SolverReport> reports;
  EvolutionTimings timings;
};
namespace evolution_detail {
inline Vector initial(const discretization::TriangleMesh& mesh,const Expr& field,double time) {
  Evaluator evaluate_field(field);Vector v(mesh.nodes.size());
  for(std::size_t i=0;i<mesh.nodes.size();++i)
    v[i]=evaluate_field({mesh.nodes[i].x,mesh.nodes[i].y,time});
  return v;
}
inline void compatible(Vector& v,const std::vector<DirichletValue>& data,double tolerance,
    const char* description) {
  for(const auto& d:data) {
    if(std::abs(v[d.node]-d.value)>tolerance*std::max({1.,std::abs(v[d.node]),std::abs(d.value)}))
      throw std::invalid_argument(std::string("Incompatible initial prescribed ")+description);
    v[d.node]=d.value;
  }
}
inline Vector step(const SparseMatrix& a,const Vector& b,const std::vector<DirichletValue>& data,
    const SolverOptions& options,Evolution& result) {
  auto phase=timing_detail::Clock::now();
  auto [matrix,rhs]=constrain(a,b,data);
  result.timings.constraints_seconds+=timing_detail::seconds(phase);
  phase=timing_detail::Clock::now();
  auto solution=solve_linear(matrix,rhs,options);
  result.timings.solver_seconds+=timing_detail::seconds(phase);
  result.timings.solver+=solution.report.timings;
  ++result.timings.linear_solves;
  if(!solution.report.converged) throw std::runtime_error("Evolution solve: "+solution.report.message);
  result.reports.push_back(solution.report);
  return solution.values;
}
inline void check_method(const AssembledSystem& s,TimeMethod method) {
  if(method==TimeMethod::Newmark) {
    if(s.second_mass.norm()==0) throw std::invalid_argument("Newmark requires nonzero second mass");
  } else {
    if(s.second_mass.norm()!=0) throw std::invalid_argument("First-order integrator does not support second mass");
    if(s.first_mass.norm()==0) throw std::invalid_argument("First-order integrator requires nonzero first mass (stationary problem)");
  }
}
}
inline Evolution evolve(const discretization::TriangleMesh& mesh,const Problem& problem,
    const Expr& initial_value,const Expr& initial_velocity=Expr(0),const TimeOptions& options={}) {
  using namespace evolution_detail;
  const auto started=timing_detail::Clock::now();
  Evolution result;
  auto timed_assembly=[&](double time,int derivative_order=0) {
    const auto phase=timing_detail::Clock::now();
    auto system=assembly_detail::assemble_impl(mesh,problem,time,options.assembly,derivative_order);
    result.timings.assembly_seconds+=timing_detail::seconds(phase);
    ++result.timings.assembly_calls;
    return system;
  };
  if(!std::isfinite(options.start)||!std::isfinite(options.end)||!std::isfinite(options.step)||
      options.step<=0||options.end<options.start||!std::isfinite(options.end-options.start))
    throw std::invalid_argument("Invalid time interval or step");
  switch(options.method) {
    case TimeMethod::BackwardEuler: case TimeMethod::CrankNicolson: case TimeMethod::Newmark: break;
    default: throw std::invalid_argument("Unsupported time method");
  }
  solver_detail::validate(options.solver);
  auto old=timed_assembly(options.start);
  check_method(old,options.method);
  Vector u=initial(mesh,initial_value,options.start);
  Vector v=initial(mesh,initial_velocity,options.start);
  compatible(u,old.dirichlet,options.assembly.boundary_tolerance,"displacement");
  Vector acceleration;
  if(options.method==TimeMethod::Newmark) {
    // Differentiate the prescribed value AFTER boundary reduction. General
    // input may become Dirichlet through cancellation only on a selected edge.
    auto velocity_data=timed_assembly(options.start,1).dirichlet;
    auto acceleration_data=timed_assembly(options.start,2).dirichlet;
    compatible(v,velocity_data,options.assembly.boundary_tolerance,"velocity");
    // Initialization is a solve, but reports correspond to time steps only.
    acceleration=step(old.second_mass,old.load-old.stiffness*u-old.first_mass*v,
        acceleration_data,options.solver,result);
    result.reports.clear(); // Keep initialization in timings, not the per-step reports.
  }
  result.frames.push_back({options.start,u,v});
  double time=options.start;
  std::size_t step_index=1;
  while(time<options.end) {
    // Compute from the origin to avoid accumulating a spurious tiny final step.
    const double candidate=std::fma(static_cast<double>(step_index),options.step,options.start);
    const double duration=options.end-options.start;
    const double elapsed=static_cast<double>(step_index)*options.step;
    // A large absolute time origin must not turn the rounding allowance into
    // several valid steps. Scale by duration and cap below a requested step.
    const double snap=std::min(options.step/8,
        8*std::numeric_limits<double>::epsilon()*std::max(duration,std::abs(elapsed)));
    const double next=candidate>=options.end ||
        duration-elapsed<=snap
        ?options.end:candidate;
    const double h=next-time;
    if(!(h>0)||!std::isfinite(h)) throw std::invalid_argument("Time step cannot advance representable time");
    auto current=timed_assembly(next);
    check_method(current,options.method);
    Vector next_u,next_v;
    if(options.method==TimeMethod::Newmark) {
      const Vector predicted_u=u+h*v+(h*h/4)*acceleration;
      const Vector predicted_v=v+(h/2)*acceleration;
      SparseMatrix a=current.second_mass+(h/2)*current.first_mass+(h*h/4)*current.stiffness;
      Vector rhs=current.load-current.stiffness*predicted_u-current.first_mass*predicted_v;
      auto data=current.dirichlet;
      for(auto& d:data) d.value=(d.value-predicted_u[d.node])/(h*h/4);
      Vector next_acceleration=step(a,rhs,data,options.solver,result);
      next_u=predicted_u+(h*h/4)*next_acceleration;
      next_v=predicted_v+(h/2)*next_acceleration;
      acceleration=std::move(next_acceleration);
    } else {
      SparseMatrix a;
      Vector rhs;
      if(options.method==TimeMethod::BackwardEuler) {
        a=current.first_mass+h*current.stiffness;
        rhs=current.first_mass*u+h*current.load;
      } else {
        // Endpoint trapezoidal residual: M1 at each endpoint multiplies
        // the common difference quotient, not a midpoint evaluation.
        SparseMatrix average_mass=(old.first_mass+current.first_mass)*.5;
        a=average_mass+(h/2)*current.stiffness;
        rhs=average_mass*u-(h/2)*(old.stiffness*u)+(h/2)*(old.load+current.load);
      }
      next_u=step(a,rhs,current.dirichlet,options.solver,result);
      next_v=(next_u-u)/h;
    }
    // Avoid roundoff in the public boundary displacement.
    for(const auto& d:current.dirichlet) next_u[d.node]=d.value;
    if(!next_u.allFinite()||!next_v.allFinite()) throw std::runtime_error("Nonfinite evolution output");
    u=std::move(next_u);v=std::move(next_v);old=std::move(current);time=next;
    result.frames.push_back({time,u,v});
    ++step_index;
  }
  result.timings.total_seconds=timing_detail::seconds(started);
  result.timings.other_seconds=std::max(0.,result.timings.total_seconds-
      result.timings.assembly_seconds-result.timings.constraints_seconds-result.timings.solver_seconds);
  return result;
}
} // namespace fem
