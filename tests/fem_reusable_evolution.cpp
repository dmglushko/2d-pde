#include "fem/evolution.hpp"
#include "../examples/fem/square_mesh.hpp"
#include <iostream>
#include <thread>

using namespace fem;
namespace {
void check(bool yes,const char* text) {if(!yes) throw std::runtime_error(text);}
void near(const Vector& a,const Vector& b) {
  check((a-b).stableNorm()<2e-9*std::max(1.,b.stableNorm()),"Cached and uncached values differ");
}
// Deliberately reassemble and prepare each solve. This checks the complete
// recurrence, including endpoint loads, lifting and initial wave acceleration.
std::vector<Frame> uncached(const discretization::TriangleMesh& mesh,const Problem& p,
    const Expr& field,const Expr& velocity,const TimeOptions& o) {
  auto old=assemble(mesh,p,o.start,o.assembly);
  Vector u=evolution_detail::initial(mesh,field,o.start),v=evolution_detail::initial(mesh,velocity,o.start),acc;
  auto solve=[&](const SparseMatrix& a,const Vector& b,const std::vector<DirichletValue>& d) {
    auto [c,r]=constrain(a,b,d);auto s=solve_linear(c,r,o.solver);
    check(s.report.converged,"Uncached reference failed");return s.values;
  };
  if(o.method==TimeMethod::Newmark) {
    auto d=assembly_detail::assemble_impl(mesh,p,o.start,o.assembly,2).dirichlet;
    acc=solve(old.second_mass,old.load-old.stiffness*u-old.first_mass*v,d);
  }
  std::vector<Frame> out{{o.start,u,v}};
  for(std::size_t i=1;out.back().time<o.end;++i) {
    double elapsed=i*o.step;
    double stamp=std::min(o.end,std::fma(double(i),o.step,o.start));
    double h=elapsed>=o.end-o.start?o.end-o.start-(i-1)*o.step:o.step;
    auto cur=assemble(mesh,p,stamp,o.assembly);
    Vector un,vn;
    if(o.method==TimeMethod::Newmark) {
      Vector pu=u+h*v+(h*h/4)*acc,pv=v+(h/2)*acc;
      SparseMatrix a=cur.second_mass+(h/2)*cur.first_mass+(h*h/4)*cur.stiffness;
      auto d=cur.dirichlet;for(auto& x:d)x.value=(x.value-pu[x.node])/(h*h/4);
      acc=solve(a,cur.load-cur.stiffness*pu-cur.first_mass*pv,d);
      un=pu+(h*h/4)*acc;vn=pv+(h/2)*acc;
    } else {
      SparseMatrix a;Vector rhs;
      if(o.method==TimeMethod::BackwardEuler) {
        a=cur.first_mass+h*cur.stiffness;rhs=cur.first_mass*u+h*cur.load;
      } else {
        SparseMatrix m=(old.first_mass+cur.first_mass)*.5;
        a=m+(h/2)*cur.stiffness;rhs=m*u-(h/2)*(old.stiffness*u)+(h/2)*(old.load+cur.load);
      }
      un=solve(a,rhs,cur.dirichlet);vn=(un-u)/h;
    }
    for(auto d:cur.dirichlet)un[d.node]=d.value;
    u=un;v=vn;old=cur;out.push_back({stamp,u,v});
  }
  return out;
}
void repeated_rhs() {
  SparseMatrix a(3,3);std::vector<Eigen::Triplet<double>> entries{{0,0,4},{1,1,5},{2,2,6},{0,1,-1},{1,0,-1}};
  a.setFromTriplets(entries.begin(),entries.end());
  for(auto method:{LinearSolver::SparseLU,LinearSolver::ConjugateGradient,LinearSolver::GMRES}) {
    SolverOptions o;o.method=method;PreparedLinearSolver prepared(a,o);
    auto stats=prepared.preparation_timings();
    check(stats.preparations==1&&stats.rank_checks==1&&stats.factorizations==1,"Preparation counters");
    for(double scale:{1.,0.,-2.,1e-180}) {
      Vector rhs=Vector::Constant(3,scale);
      auto cached=prepared.solve(rhs),fresh=solve_linear(a,rhs,o);
      check(cached.report.converged==fresh.report.converged,"Cached convergence mismatch");
      near(cached.values,fresh.values);
      check(cached.report.timings.preparations==0&&cached.report.timings.rank_checks==0&&
            cached.report.timings.factorizations==0&&cached.report.timings.rhs_solves==1,"Repeated preparation");
    }
    SparseMatrix changed=a;changed.coeffRef(0,0)+=1;
    check(!prepared.matches(changed,o),"Value change reused factorization");
    check(prepared.matches(a,o),"Equal matrix missed cache");
    SolverOptions other=o;other.restart++;
    check(!prepared.matches(a,other),"Option change reused preparation");
    a.coeffRef(0,0)+=2;
    check(!prepared.matches(a,o),"Caller mutation changed snapshot");a.coeffRef(0,0)-=2;
    Vector rhs=Vector::Ones(3);near(prepared.solve(rhs).values,solve_linear(a,rhs,o).values);
  }
  SparseMatrix singular(3,3);PreparedLinearSolver bad(singular);
  check(!bad.solve(Vector::Zero(3)).report.converged,"Prepared singular zero RHS accepted");
}
void evolution() {
  auto mesh=square_mesh(3);const Expr phi=1+x+2*y;
  for(auto solver:{LinearSolver::SparseLU,LinearSolver::ConjugateGradient,LinearSolver::GMRES})
  for(auto method:{TimeMethod::BackwardEuler,TimeMethod::CrankNicolson,TimeMethod::Newmark})
  for(bool variable:{false,true}) {
    Problem p;p.equation.m1=1+(variable?t*t:Expr(0));
    p.equation.m2=method==TimeMethod::Newmark?Expr(1)+x/4:Expr(0);
    p.equation.c=1+x+(variable?sin(t):Expr(0));
    p.equation.source=phi*(1+sin(t)+t*t*t);
    p.assign(dirichlet(phi*(1+t+t*t)),{"bottom"});
    p.assign(dynamic_flux_robin(.3,1,1,(nx+2*ny)*(1+t+t*t)+phi*(1+t+t*t)+.3*phi*(1+2*t)),{"right","top","left"});
    TimeOptions o;o.method=method;o.solver.method=solver;o.step=.03;o.end=.1;
    auto e=evolve(mesh,p,phi,phi,o);auto ref=uncached(mesh,p,phi,phi,o);
    check(e.frames.size()==ref.size(),"Frame count mismatch");
    for(std::size_t i=0;i<ref.size();++i) {near(e.frames[i].values,ref[i].values);near(e.frames[i].velocity,ref[i].velocity);}
    const std::size_t prep=(variable?4:2)+(method==TimeMethod::Newmark?1:0);
    check(e.timings.solver.preparations==prep,"Effective matrix invalidation count");
    check(e.timings.time_steps==4&&e.reports.size()==4,"Step/report counts");
    check(e.timings.solver.rank_checks==prep&&e.timings.solver.factorizations==prep,"Rank/factor count");
    check(e.timings.solver_reuses+prep==e.timings.linear_solves,"Reuse accounting");
  }
  Problem p;p.equation.m1=1;p.equation.source=sin(t)+t*t*t;
  p.assign(flux_neumann(0),{"bottom","right","top","left"});
  TimeOptions o;o.end=.3;o.step=.01;
  auto full=evolve(mesh,p,0,0,o);
  check(full.timings.time_steps==30&&full.timings.solver.preparations==1&&full.timings.solver_reuses==29,
        "Timestamp ULPs or source variation prevented reuse");
  o.frame_stride=7;o.store_reports=false;std::size_t observed=0;
  const auto caller=std::this_thread::get_id();
  o.observer=[&](const Frame& f) {
    check(std::this_thread::get_id()==caller,"Observer moved threads");
    near(f.values,full.frames.at(observed).values);++observed;
  };
  auto bounded=evolve(mesh,p,0,0,o);
  check(observed==31&&bounded.frames.size()==6&&bounded.reports.empty(),"Observation/storage semantics");
  check(bounded.frames.front().time==0&&bounded.frames.back().time==.3,"Retention endpoints");
  observed=0;
  o.observer=[&](const Frame&){if(++observed==3)throw std::runtime_error("observer failure");};
  bool threw=false;try{evolve(mesh,p,0,0,o);}catch(const std::runtime_error& e){threw=std::string(e.what())=="observer failure";}
  check(threw,"Observer exception swallowed");
  o.observer={};
  p.equation.m2=1;o.method=TimeMethod::Newmark;
  auto wave=evolve(mesh,p,0,0,o);
  check(wave.timings.solver.preparations==2&&wave.timings.solver_reuses==29&&
        wave.timings.linear_solves==31&&wave.timings.time_steps==30,"Constant wave reuse count");
  check(wave.timings.solver.rhs_solves==31,"Initial solve omitted from timings");
  p.equation.m2=0;o.method=TimeMethod::BackwardEuler;
  o.end=0;check(evolve(mesh,p,0,0,o).frames.size()==1,"Zero duration");
  o.frame_stride=0;threw=false;try{evolve(mesh,p,0,0,o);}catch(const std::invalid_argument&){threw=true;}
  check(threw,"Zero stride accepted");
  if(pde2d::openmp_available()) {
    pde2d::ExecutionOptions parallel;parallel.backend=pde2d::ExecutionBackend::OpenMP;parallel.threads=2;parallel.min_parallel_size=0;
    near(evolution_detail::initial(mesh,exp(x+y),0,parallel),evolution_detail::initial(mesh,exp(x+y),0));
    o.frame_stride=1;o.end=.04;o.assembly.execution=parallel;
    observed=0;o.observer=[&](const Frame&){check(std::this_thread::get_id()==caller,"Parallel observer thread");++observed;};
    auto par=evolve(mesh,p,0,0,o);o.assembly.execution={};o.observer={};
    near(par.frames.back().values,evolve(mesh,p,0,0,o).frames.back().values);
    check(observed==5,"Parallel observer count");
    threw=false;try{evolution_detail::initial(mesh,log(x-2),0,parallel);}catch(const std::exception&){threw=true;}
    check(threw,"Parallel initial evaluation failure swallowed");
  }
}
}
int main() {try {repeated_rhs();evolution();std::cout<<"Reusable solver/evolution checks passed\n";}
catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
