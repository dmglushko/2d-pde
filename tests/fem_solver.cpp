#include "fem/evolution.hpp"
#include "../examples/fem/square_mesh.hpp"
#include <Eigen/Dense>
#include <iostream>

namespace {
using namespace fem;
void check(bool ok,const char* message) {if(!ok) throw std::runtime_error(message);}
void near(double a,double b,double tol=1e-9) {
  check(std::abs(a-b)<=tol*std::max({1.,std::abs(a),std::abs(b)}),"Numerical mismatch");
}
template<class F> void rejects(F f,const char* fragment="") {
  try {f();} catch(const std::exception& e) {
    if(std::string(e.what()).find(fragment)==std::string::npos)
      throw std::runtime_error(std::string("Unexpected diagnostic: ")+e.what());
    return;
  }
  throw std::runtime_error("Expected rejection");
}
SparseMatrix sparse(const Eigen::MatrixXd& a) {return a.sparseView();}
void linear() {
  Eigen::MatrixXd dense(3,3);dense<<4,-1,0,-1,4,-1,0,-1,3;
  SparseMatrix a=sparse(dense);Vector b(3);b<<1,2,3;
  for(auto method:{LinearSolver::SparseLU,LinearSolver::ConjugateGradient,LinearSolver::GMRES}) {
    SolverOptions o;o.method=method;
    auto s=solve_linear(a,b,o);check(s.report.converged,"Linear solve failed");
    near(s.report.residual_norm,(a*s.values-b).norm());
    near(s.report.relative_residual,s.report.residual_norm/b.norm());
  }
  auto [c,rhs]=constrain(a,b,{{0,2}});
  near((c-SparseMatrix(c.transpose())).norm(),0);
  near(rhs[1],4);near(c.coeff(1,0),0);near(c.coeff(0,0),1);
  auto s=solve_linear(c,rhs);near(s.values[0],2);
  near((a*s.values-b)[1],0);near((a*s.values-b)[2],0);
  SolverOptions cg;cg.method=LinearSolver::ConjugateGradient;
  dense(0,1)=2;
  check(!solve_linear(sparse(dense),b,cg).report.converged,"CG accepted nonsymmetry");
  dense.setIdentity();dense(0,0)=-1;
  check(!solve_linear(sparse(dense),b,cg).report.converged,"CG accepted indefiniteness");
  dense.setZero();check(!solve_linear(sparse(dense),Vector::Zero(3)).report.converged,"Singularity missed");
  dense<<4,-1,0,-1,4,-1,0,-1,3;
  cg.max_iterations=1;cg.relative_tolerance=1e-15;cg.absolute_tolerance=0;
  s=solve_linear(sparse(dense),b,cg);
  check(!s.report.converged&&s.report.residual_norm>0,"Iteration failure missed");
  near(s.report.residual_norm,(sparse(dense)*s.values-b).norm());
  cg.method=LinearSolver::GMRES;cg.restart=1;
  check(!solve_linear(sparse(dense),b,cg).report.converged,"GMRES failure missed");
  rejects([&]{constrain(a,b,{{0,1},{0,2}});},"Conflicting");
  SolverOptions bad;bad.relative_tolerance=std::numeric_limits<double>::quiet_NaN();
  rejects([&]{solve_linear(a,b,bad);},"options");
}
void stationary() {
  auto mesh=square_mesh(3);Problem p;
  p.assign(dirichlet(1+x+2*y),{"bottom","right","top","left"});
  auto s=solve_stationary(mesh,p);
  for(std::size_t i=0;i<mesh.nodes.size();++i) near(s.values[i],1+mesh.nodes[i].x+2*mesh.nodes[i].y);
  p.boundaries.clear();p.assign(flux_neumann(0),{"bottom","right","top","left"});
  rejects([&]{solve_stationary(mesh,p);},"Singular");
}
void heat() {
  // One free node gives an exact semidiscrete eigenmode, removing spatial
  // error from the temporal order test.
  auto mesh=square_mesh(2);Problem p;p.equation.m1=1;
  p.assign(dirichlet(0),{"bottom","right","top","left"});
  auto a=assemble(mesh,p);const double lambda=a.stiffness.coeff(4,4)/a.first_mass.coeff(4,4);
  const Expr pi=acos(-1.); // Numeric pi is sufficient for nodal initial data.
  const Expr initial=sin(pi*x)*sin(pi*y);
  auto error=[&](TimeMethod method,double h) {
    TimeOptions o;o.method=method;o.end=.1;o.step=h;
    auto e=evolve(mesh,p,initial,0,o);
    check(e.frames.front().time==0&&e.frames.back().time==o.end,"Endpoint missing");
    return std::abs(e.frames.back().values[4]-std::exp(-lambda*o.end));
  };
  check(error(TimeMethod::BackwardEuler,.01)/error(TimeMethod::BackwardEuler,.005)>1.8,"BE temporal order");
  check(error(TimeMethod::CrankNicolson,.01)/error(TimeMethod::CrankNicolson,.005)>3.8,"CN temporal order");
  TimeOptions o;o.step=.03;o.end=.1;
  auto e=evolve(mesh,p,initial,0,o);
  check(e.frames.size()==5&&e.frames.back().time==.1,"Short final step");
  near(e.frames[0].values[4],1);
  double amplification=1;
  for(std::size_t k=1;k<e.frames.size();++k)
    amplification/=1+lambda*(e.frames[k].time-e.frames[k-1].time);
  near(e.frames.back().values[4],amplification);
}
void damped_eigenmode() {
  auto mesh=square_mesh(2);Problem p;p.equation.m2=1;p.equation.m1=.7;
  p.assign(dirichlet(0),{"bottom","right","top","left"});
  auto a=assemble(mesh,p);const double lambda=a.stiffness.coeff(4,4)/a.second_mass.coeff(4,4);
  const double omega=std::sqrt(lambda-.35*.35), end=.3;
  const double exact=std::exp(-.35*end)*(std::cos(omega*end)+.35/omega*std::sin(omega*end));
  const Expr pi=std::acos(-1.), initial=sin(pi*x)*sin(pi*y);
  auto error=[&](double h) {
    TimeOptions o;o.method=TimeMethod::Newmark;o.step=h;o.end=end;
    auto e=evolve(mesh,p,initial,0,o);
    return std::abs(e.frames.back().values[4]-exact);
  };
  check(error(.02)/error(.01)>3.8,"Damped Newmark temporal order");
  // An explicit scalar recurrence distinguishes endpoint trapezoidal assembly
  // from evaluating nonlinear time coefficients only at the midpoint.
  p.equation.m2=0;p.equation.m1=1+t*t;p.equation.c=t*t;
  TimeOptions o;o.method=TimeMethod::CrankNicolson;o.step=.07;o.end=.2;
  auto e=evolve(mesh,p,initial,0,o);
  double expected=1;
  for(std::size_t k=1;k<e.frames.size();++k) {
    const double s=e.frames[k-1].time, q=e.frames[k].time,h=q-s;
    const double mass=1+(s*s+q*q)/2;
    expected*=(mass-h*(lambda+s*s)/2)/(mass+h*(lambda+q*q)/2);
  }
  near(e.frames.back().values[4],expected);
}
void moving_and_waves() {
  auto mesh=square_mesh(3);
  const Expr phi=1+x+2*y, shape=1+t+t*t;
  for(double damping:{0.,1.}) {
    Problem p;p.equation.m2=1;p.equation.m1=damping*(1+t);
    p.equation.c=1+t;
    p.equation.source=2*phi+p.equation.m1*phi*(1+2*t)+p.equation.c*phi*shape;
    // Time-dependent alpha checks differentiation of the ratio value/alpha.
    BoundaryCondition bc=dirichlet(0);bc.alpha=1+t;bc.value=(1+t)*phi*shape;
    p.assign(bc,{"bottom"});
    const Expr flux=(nx+2*ny)*shape;
    p.assign(dynamic_flux_robin(2,1,1,2*phi*(1+2*t)+phi*shape+flux),{"right","top","left"});
    TimeOptions o;o.method=TimeMethod::Newmark;o.end=.13;o.step=.04;o.assembly.quadrature_degree=6;
    auto e=evolve(mesh,p,phi,phi,o);
    for(const auto& frame:e.frames) for(std::size_t i=0;i<mesh.nodes.size();++i) {
      const double f=1+mesh.nodes[i].x+2*mesh.nodes[i].y;
      near(frame.values[i],f*(1+frame.time+frame.time*frame.time),2e-9);
      near(frame.velocity[i],f*(1+2*frame.time),2e-8);
    }
    rejects([&]{evolve(mesh,p,phi,0,o);},"velocity");
    rejects([&]{evolve(mesh,p,0,phi,o);},"displacement");
    o.method=TimeMethod::BackwardEuler;
    rejects([&]{evolve(mesh,p,phi,phi,o);},"second mass");
  }
  // Moving boundary lifting with a time-varying bulk first mass. The affine
  // solution is exact for both first-order schemes and exercises reassembly.
  for(auto method:{TimeMethod::BackwardEuler,TimeMethod::CrankNicolson}) {
    Problem p;p.equation.m1=1+t;p.equation.source=(1+t)*phi;
    p.assign(dirichlet(phi*(1+t)),{"bottom","right","top","left"});
    TimeOptions o;o.method=method;o.step=.03;o.end=.1;
    auto e=evolve(mesh,p,phi,0,o);
    for(const auto& frame:e.frames) for(std::size_t i=0;i<mesh.nodes.size();++i)
      near(frame.values[i],(1+mesh.nodes[i].x+2*mesh.nodes[i].y)*(1+frame.time));
  }
  // Boundary storage is a legitimate first mass even when bulk m1 is zero.
  auto tiny=square_mesh(1);Problem storage;
  storage.assign(dynamic_flux_robin(1,1,1,2+t),{"bottom","right","top","left"});
  TimeOptions o;o.end=.1;o.step=.03;
  auto e=evolve(tiny,storage,1,0,o);
  for(double value:e.frames.back().values) near(value,1.1);
}
void invalid_options() {
  auto mesh=square_mesh(2);Problem p;
  p.assign(dirichlet(0),{"bottom","right","top","left"});
  rejects([&]{evolve(mesh,p,0);},"first mass");
  p.equation.m1=1;TimeOptions o;o.method=TimeMethod::Newmark;
  rejects([&]{evolve(mesh,p,0,0,o);},"second mass");
  o.method=TimeMethod::BackwardEuler;o.step=std::numeric_limits<double>::infinity();
  rejects([&]{evolve(mesh,p,0,0,o);},"time interval");
  o.step=0;rejects([&]{evolve(mesh,p,0,0,o);});
  o.step=.1;o.end=-1;rejects([&]{evolve(mesh,p,0,0,o);});
  o.end=0;auto e=evolve(mesh,p,0,0,o);check(e.frames.size()==1,"Initial-only evolution");
}
}
int main() {
  try {linear();stationary();heat();damped_eigenmode();moving_and_waves();invalid_options();
    std::cout<<"FEM solver tests passed\n";
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
