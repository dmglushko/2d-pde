// Two 100-second waves on a 10 x 10 square, with wall-clock profiling.
#include "fem/evolution.hpp"
#include "square_mesh.hpp"
#include "error_norms.hpp"
#include "showcase_io.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <optional>

using namespace fem;
using namespace showcase_io;
using Clock=fem::timing_detail::Clock;

struct Case {
  std::string id,name,equation,boundary,description;
  Problem problem;
  Expr initial=0;
  std::optional<Expr> exact;
  Evolution evolution;
  double setup_seconds=0,diagnostics_seconds=0;
  double max_balance_error=0,mean_drift=0;
  std::vector<double> energy,mean,work,l2,h1;
};

void write_timings(std::ostream& out,const Case& c) {
  const auto& e=c.evolution.timings;const auto& s=e.solver;
  out<<"{\"setup_seconds\":"<<c.setup_seconds
     <<",\"integration_seconds\":"<<e.total_seconds
     <<",\"assembly_seconds\":"<<e.assembly_seconds
     <<",\"constraints_seconds\":"<<e.constraints_seconds
     <<",\"solver_seconds\":"<<e.solver_seconds
     <<",\"solver_validation_seconds\":"<<s.validation_seconds
     <<",\"rank_check_seconds\":"<<s.rank_check_seconds
     <<",\"factorization_seconds\":"<<s.factorization_seconds
     <<",\"linear_solve_seconds\":"<<s.solve_seconds
     <<",\"residual_seconds\":"<<s.residual_seconds
     <<",\"evolution_other_seconds\":"<<e.other_seconds
     <<",\"diagnostics_seconds\":"<<c.diagnostics_seconds
     <<",\"assembly_calls\":"<<e.assembly_calls
     <<",\"linear_solves\":"<<e.linear_solves<<'}';
}

int main(int argc,char** argv) {
  try {
    const auto started=Clock::now();
    const std::filesystem::path destination=argc>1?argv[1]:"build/long-waves.json";
    const int divisions=argc>2?std::stoi(argv[2]):20;
    const double step=argc>3?std::stod(argv[3]):.05;
    constexpr double length=10, end=100;
    if(destination.has_parent_path())std::filesystem::create_directories(destination.parent_path());
    auto phase=Clock::now();
    auto mesh=square_mesh(divisions);
    for(auto& node:mesh.nodes){node.x*=length;node.y*=length;}
    discretization::validate_mesh_geometry(mesh);
    const double mesh_seconds=timing_detail::seconds(phase);
    const double pi=std::numbers::pi;
    const Expr source_space=.15*exp(-((x-3)*(x-3)+(y-5)*(y-5))/1.2);
    const Expr drive=(1-exp(-t*t/4))*sin(.8*t);
    const Evaluator drive_value(drive);
    Case forced;
    forced.id="long-dirichlet";forced.name="100 s: driven Dirichlet wave";
    forced.equation="u_tt - u_xx - u_yy = 0.15 exp(-[(x-3)²+(y-5)²]/1.2) (1-exp(-t²/4)) sin(0.8t)";
    forced.boundary="u=0 on every edge; u(x,y,0)=0; u_t(x,y,0)=0";
    forced.description="10 × 10 square, wave speed 1, 100 simulated seconds. A smooth localized source near (3,5) drives a membrane with fixed edges. Energy is checked against accumulated source work. There is no closed-form reference for this driven case.";
    forced.problem.equation.m2=1;forced.problem.equation.source=source_space*drive;
    forced.problem.assign(dirichlet(0),{"bottom","right","top","left"});
    Case free;
    free.id="long-neumann";free.name="100 s: source-free Neumann wave";
    free.equation="u_tt - u_xx - u_yy = 0 (no source)";
    free.boundary="du/dn=0 on every edge; initial velocity=0; initial displacement=two cosine modes + 0.2";
    free.description="10 × 10 square with reflecting Neumann edges. Initial displacement launches the motion; the source is identically zero. The discrete energy and spatial mean should remain constant. An exact cosine-mode solution checks accumulated phase error over 100 seconds.";
    free.exact=.2+cos(pi*x/length)*cos(pi*y/length)*cos(pi*std::sqrt(2.)*t/length)
      +.35*cos(2*pi*x/length)*cos(pi*y/length)*cos(pi*std::sqrt(5.)*t/length);
    free.initial=free.exact->subs({{t.get_basic(),Expr(0).get_basic()}});
    free.problem.equation.m2=1;
    free.problem.assign(neumann(0),{"bottom","right","top","left"});
    if(!is_zero(free.problem.equation.source))throw std::runtime_error("Neumann showcase must have no source");
    std::vector<Case> cases;cases.push_back(std::move(forced));cases.push_back(std::move(free));
    for(auto& c:cases) {
      phase=Clock::now();
      // Constant matrices for energy and a spatial load for source-work checks.
      auto diagnostic_problem=c.problem;
      diagnostic_problem.equation.source=c.exact?Expr(0):source_space;
      const auto system=assemble(mesh,diagnostic_problem,0,AssemblyOptions{6});
      const Vector ones=Vector::Ones(mesh.nodes.size());
      const Vector weights=system.second_mass*ones;
      const double area=weights.sum();
      c.setup_seconds=timing_detail::seconds(phase);
      TimeOptions time;time.method=TimeMethod::Newmark;time.end=end;time.step=step;
      time.assembly.quadrature_degree=6;
      std::cout<<"Running "<<c.name<<" on "<<mesh.nodes.size()<<" nodes, dt="<<step<<" s"<<std::endl;
      c.evolution=evolve(mesh,c.problem,c.initial,0,time);
      phase=Clock::now();
      double cumulative_work=0;
      const auto& frames=c.evolution.frames;
      for(std::size_t k=0;k<frames.size();++k) {
        const auto& f=frames[k];
        const double energy=.5*(f.velocity.dot(system.second_mass*f.velocity)+f.values.dot(system.stiffness*f.values));
        c.energy.push_back(energy);c.mean.push_back(weights.dot(f.values)/area);
        if(k && !c.exact) {
          const auto& previous=frames[k-1];
          const double forcing=.5*(drive_value({0,0,previous.time})+drive_value({0,0,f.time}));
          cumulative_work+=(f.values-previous.values).dot(system.load)*forcing;
        }
        c.work.push_back(cumulative_work);
        c.max_balance_error=std::max(c.max_balance_error,std::abs(energy-c.energy.front()-cumulative_work));
        c.mean_drift=std::max(c.mean_drift,std::abs(c.mean.back()-c.mean.front()));
        if(k%5==0 || k+1==frames.size()) {
          if(c.exact){const auto error=error_norms(mesh,f.values,*c.exact,f.time);c.l2.push_back(error.l2);c.h1.push_back(error.h1_seminorm);}
        }
      }
      c.diagnostics_seconds=timing_detail::seconds(phase);
      const double energy_scale=std::max(1.,*std::max_element(c.energy.begin(),c.energy.end()));
      if(c.max_balance_error/energy_scale>1e-8)throw std::runtime_error("Discrete energy/work balance check failed");
      if(c.exact && c.mean_drift>1e-9)throw std::runtime_error("Source-free Neumann mean conservation failed");
      if(frames.back().time!=end)throw std::runtime_error("Simulation did not reach 100 seconds");
      if(frames.size()!=static_cast<std::size_t>(std::ceil(end/step))+1)
        throw std::runtime_error("Unexpected number of simulation time steps");
      std::cout<<"  "<<frames.size()-1<<" steps in "<<c.evolution.timings.total_seconds
               <<" wall seconds; max energy/work defect "<<c.max_balance_error
               <<"; mean drift "<<c.mean_drift<<std::endl;
      std::cout<<"  assembly "<<c.evolution.timings.assembly_seconds
               <<" s, constraints "<<c.evolution.timings.constraints_seconds
               <<" s, rank checks "<<c.evolution.timings.solver.rank_check_seconds
               <<" s, factorization "<<c.evolution.timings.solver.factorization_seconds
               <<" s, linear solves "<<c.evolution.timings.solver.solve_seconds
               <<" s, diagnostics "<<c.diagnostics_seconds<<" s"<<std::endl;
    }
    phase=Clock::now();
    std::ofstream out(destination);if(!out)throw std::runtime_error("Cannot open wave output");
    out<<std::setprecision(10)<<'{';write_mesh(out,mesh);
    out<<",\"previewTime\":20,\"time_step\":"<<step<<",\"simulated_seconds\":100,\"mesh_seconds\":"<<mesh_seconds<<",\"convergence\":[],\"examples\":[";
    for(std::size_t i=0;i<cases.size();++i) {
      if(i)out<<',';
      const auto& c=cases[i];
      out<<'{';
      for(const auto& [key,value]:std::vector<std::pair<std::string,std::string>>{{"id",c.id},{"name",c.name},{"equation",c.equation},{"boundary",c.boundary},{"description",c.description}}) {
        quote(out,key);out<<':';quote(out,value);out<<',';
      }
      double residual=0;for(const auto& r:c.evolution.reports)residual=std::max(residual,r.relative_residual);
      out<<"\"residual\":"<<residual<<",\"timings\":";write_timings(out,c);out<<",\"frames\":[";
      bool first=true;std::size_t saved=0;
      for(std::size_t k=0;k<c.evolution.frames.size();++k) {
        if(k%5!=0 && k+1!=c.evolution.frames.size())continue;
        if(!first)out<<',';
        first=false;
        const auto& frame=c.evolution.frames[k];
        out<<"{\"time\":"<<frame.time<<",\"energy\":"<<c.energy[k]<<",\"mean\":"<<c.mean[k]<<",\"work\":"<<c.work[k]<<",\"values\":";
        write_vector(out,frame.values);
        if(c.exact) {
          out<<",\"l2\":"<<c.l2[saved]<<",\"h1\":"<<c.h1[saved]<<",\"exact\":";
          Vector truth(mesh.nodes.size());Evaluator evaluator(*c.exact);
          for(std::size_t n=0;n<mesh.nodes.size();++n)truth[n]=evaluator({mesh.nodes[n].x,mesh.nodes[n].y,frame.time});
          write_vector(out,truth);
        }
        out<<'}';++saved;
      }
      out<<"]}";
    }
    out<<"]}\n";out.close();if(!out)throw std::runtime_error("Wave output write failed");
    const double export_seconds=timing_detail::seconds(phase);
    auto profile_path=destination;profile_path.replace_extension("timings.json");
    std::ofstream profile(profile_path);if(!profile)throw std::runtime_error("Cannot open timing output");
    profile<<std::setprecision(10)<<"{\"clock\":\"steady_clock wall seconds\",\"nodes\":"<<mesh.nodes.size()<<",\"triangles\":"<<mesh.elements.size()
      <<",\"side_length\":10,\"simulated_seconds\":100,\"time_step\":"<<step<<",\"saved_frame_stride\":5,\"mesh_seconds\":"<<mesh_seconds
      <<",\"data_export_seconds\":"<<export_seconds<<",\"execution_seconds\":"<<timing_detail::seconds(started)<<",\"cases\":[";
    for(std::size_t i=0;i<cases.size();++i){if(i)profile<<',';profile<<"{\"name\":";quote(profile,cases[i].name);profile<<",\"timings\":";write_timings(profile,cases[i]);profile<<",\"max_energy_work_defect\":"<<cases[i].max_balance_error<<",\"mean_drift\":"<<cases[i].mean_drift<<'}';}
    profile<<"]}\n";profile.close();if(!profile)throw std::runtime_error("Timing output write failed");
    std::cout<<"Mesh "<<mesh_seconds<<" s; data export "<<export_seconds<<" s; profile: "<<profile_path<<std::endl;
  } catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
