// Two 100-second waves on a 10 x 10 square, with wall-clock profiling.
#include "fem/report.hpp"
#include "square_mesh.hpp"
#include "error_norms.hpp"
#include "showcase_io.hpp"
#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <optional>
#include <thread>

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
  std::size_t observed_frames=0;
  double max_balance_error=0,mean_drift=0;
  std::vector<double> energy,mean,work,l2,h1;
};

struct WaveRun {
  struct Profile {
    std::string id;
    EvolutionTimings timings;
    double setup_seconds,diagnostics_seconds;
  };
  double execution_seconds=0,export_seconds=0;
  std::vector<Profile> cases;
  // Full retained fields and diagnostics, including velocity, for comparisons.
  std::vector<double> comparison;
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
     <<",\"observer_included_in_integration\":true"
     <<",\"integration_excluding_observer_seconds\":"<<std::max(0.,e.total_seconds-c.diagnostics_seconds)
     <<",\"observed_frames\":"<<c.observed_frames
     <<",\"retained_frames\":"<<c.evolution.frames.size()
     <<",\"retained_reports\":"<<c.evolution.reports.size()
     <<",\"time_steps\":"<<e.time_steps
     <<",\"solver_preparations\":"<<s.preparations
     <<",\"rank_checks\":"<<s.rank_checks
     <<",\"factorizations\":"<<s.factorizations
     <<",\"solver_reuses\":"<<e.solver_reuses
     <<",\"assembly_calls\":"<<e.assembly_calls
     <<",\"linear_solves\":"<<e.linear_solves<<'}';
}

WaveRun run_waves(const std::filesystem::path& destination,int divisions,
    double step,int threads,bool compare=false) {
    const auto started=Clock::now();
    if(threads<1)throw std::invalid_argument("Thread count must be positive");
    pde2d::ExecutionOptions execution;
    execution.threads=threads;
    if(threads>1)execution.backend=pde2d::ExecutionBackend::OpenMP;
    pde2d::validate_execution(execution);
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
      AssemblyOptions diagnostic_options{6};diagnostic_options.execution=execution;
      const auto system=assemble(mesh,diagnostic_problem,0,diagnostic_options);
      const Vector ones=Vector::Ones(mesh.nodes.size());
      const Vector weights=system.second_mass*ones;
      const double area=weights.sum();
      std::optional<PreparedErrorNorms> errors;
      if(c.exact)errors.emplace(mesh,*c.exact);
      c.setup_seconds=timing_detail::seconds(phase);
      TimeOptions time;time.method=TimeMethod::Newmark;time.end=end;time.step=step;
      time.assembly.quadrature_degree=6;time.frame_stride=5;
      time.assembly.execution=execution;
      double cumulative_work=0,initial_energy=0,initial_mean=0,max_energy=0,previous_time=0;
      Vector previous_values;
      time.observer=[&](const Frame& f){
        const double energy=.5*(f.velocity.dot(system.second_mass*f.velocity)+f.values.dot(system.stiffness*f.values));
        const double mean=weights.dot(f.values)/area;
        if(c.observed_frames==0){initial_energy=energy;initial_mean=mean;}
        else if(!c.exact){
          const double forcing=.5*(drive_value({0,0,previous_time})+drive_value({0,0,f.time}));
          cumulative_work+=(f.values-previous_values).dot(system.load)*forcing;
        }
        c.max_balance_error=std::max(c.max_balance_error,std::abs(energy-initial_energy-cumulative_work));
        c.mean_drift=std::max(c.mean_drift,std::abs(mean-initial_mean));
        max_energy=std::max(max_energy,energy);
        if(c.observed_frames%time.frame_stride==0 || f.time==end){
          c.energy.push_back(energy);c.mean.push_back(mean);c.work.push_back(cumulative_work);
          if(errors){const auto error=errors->evaluate(f.values,f.time,execution);c.l2.push_back(error.l2);c.h1.push_back(error.h1_seminorm);}
        }
        if(!c.exact)previous_values=f.values;
        previous_time=f.time;++c.observed_frames;
      };
      std::cout<<"Running "<<c.name<<" on "<<mesh.nodes.size()<<" nodes, dt="<<step<<" s"<<std::endl;
      c.evolution=evolve(mesh,c.problem,c.initial,0,time);
      c.diagnostics_seconds=c.evolution.timings.observer_seconds;
      const auto& frames=c.evolution.frames;
      const double energy_scale=std::max(1.,max_energy);
      if(c.max_balance_error/energy_scale>1e-8)throw std::runtime_error("Discrete energy/work balance check failed");
      if(c.exact && c.mean_drift>1e-9)throw std::runtime_error("Source-free Neumann mean conservation failed");
      if(frames.back().time!=end)throw std::runtime_error("Simulation did not reach 100 seconds");
      if(c.observed_frames!=c.evolution.timings.time_steps+1 || frames.size()!=c.energy.size() || frames.front().time!=0)
        throw std::runtime_error("Unexpected number of simulation time steps");
      std::cout<<"  "<<c.evolution.timings.time_steps<<" steps in "<<c.evolution.timings.total_seconds
               <<" wall seconds; max energy/work defect "<<c.max_balance_error
               <<"; mean drift "<<c.mean_drift<<std::endl;
      write_report(std::cout,c.evolution);
    }
    phase=Clock::now();
    std::ofstream out(destination);if(!out)throw std::runtime_error("Cannot open wave output");
    out<<std::setprecision(17)<<'{';write_mesh(out,mesh);
    out<<",\"execution_backend\":\""<<(threads==1?"serial":"openmp")<<"\",\"execution_threads\":"<<threads;
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
      std::optional<Evaluator> evaluator;if(c.exact)evaluator.emplace(*c.exact);
      bool first=true;std::size_t saved=0;
      for(std::size_t k=0;k<c.evolution.frames.size();++k) {
        if(!first)out<<',';
        first=false;
        const auto& frame=c.evolution.frames[k];
        out<<"{\"time\":"<<frame.time<<",\"energy\":"<<c.energy[k]<<",\"mean\":"<<c.mean[k]<<",\"work\":"<<c.work[k]<<",\"values\":";
        write_vector(out,frame.values);
        if(c.exact) {
          out<<",\"l2\":"<<c.l2[saved]<<",\"h1\":"<<c.h1[saved]<<",\"exact\":";
          Vector truth(mesh.nodes.size());
          for(std::size_t n=0;n<mesh.nodes.size();++n)truth[n]=(*evaluator)({mesh.nodes[n].x,mesh.nodes[n].y,frame.time});
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
    profile<<std::setprecision(17)<<"{\"clock\":\"steady_clock wall seconds\",\"nodes\":"<<mesh.nodes.size()<<",\"triangles\":"<<mesh.elements.size()
      <<",\"execution_backend\":\""<<(threads==1?"serial":"openmp")<<"\",\"execution_threads\":"<<threads
      <<",\"side_length\":10,\"simulated_seconds\":100,\"time_step\":"<<step<<",\"saved_frame_stride\":5,\"mesh_seconds\":"<<mesh_seconds
      <<",\"data_export_seconds\":"<<export_seconds<<",\"execution_seconds\":"<<timing_detail::seconds(started)<<",\"cases\":[";
    for(std::size_t i=0;i<cases.size();++i){if(i)profile<<',';profile<<"{\"name\":";quote(profile,cases[i].name);profile<<",\"timings\":";write_timings(profile,cases[i]);profile<<",\"max_energy_work_defect\":"<<cases[i].max_balance_error<<",\"mean_drift\":"<<cases[i].mean_drift<<'}';}
    profile<<"]}\n";profile.close();if(!profile)throw std::runtime_error("Timing output write failed");
    std::cout<<"Mesh "<<mesh_seconds<<" s; data export "<<export_seconds<<" s; profile: "<<profile_path<<std::endl;
    WaveRun result;result.export_seconds=export_seconds;
    result.execution_seconds=timing_detail::seconds(started);
    for(const auto& c:cases) {
      result.cases.push_back({c.id,c.evolution.timings,c.setup_seconds,c.diagnostics_seconds});
      if(compare)for(std::size_t i=0;i<c.evolution.frames.size();++i) {
        const auto& f=c.evolution.frames[i];
        result.comparison.push_back(f.time);
        for(double v:f.values)result.comparison.push_back(v);
        for(double v:f.velocity)result.comparison.push_back(v);
        for(double v:{c.energy[i],c.mean[i],c.work[i]})result.comparison.push_back(v);
        if(c.exact)for(double v:{c.l2[i],c.h1[i]})result.comparison.push_back(v);
      }
    }
    // Comparison copying is benchmark bookkeeping, outside measured execution.
    return result;
}

double median(std::vector<double> values) {
  std::sort(values.begin(),values.end());const auto n=values.size();
  return n%2?values[n/2]:(values[n/2-1]+values[n/2])/2;
}

void benchmark(const std::filesystem::path& directory,int repetitions,int divisions,double step) {
  if(!pde2d::openmp_available())throw std::invalid_argument(
      "--benchmark requires an OpenMP-enabled build (PDE2D_ENABLE_OPENMP=ON)");
  if(repetitions<1)throw std::invalid_argument("Benchmark repetitions must be positive");
  for(int threads:{1,2,4})pde2d::validate_execution(
      {threads==1?pde2d::ExecutionBackend::Serial:pde2d::ExecutionBackend::OpenMP,threads});
  std::filesystem::create_directories(directory);
  struct Measurement {int repetition,threads;WaveRun run;double difference;};
  std::vector<Measurement> measurements;
  std::vector<double> reference;
  const std::array<int,3> policies{1,2,4};
  // Rotate the order after the first serial reference to reduce ordering bias.
  for(int repetition=0;repetition<repetitions;++repetition)for(int slot=0;slot<3;++slot) {
    const int threads=policies[(slot+repetition)%3];
    const auto output=directory/((threads==1?"serial":"omp"+std::to_string(threads))+
        "-"+std::to_string(repetition+1)+".json");
    auto run=run_waves(output,divisions,step,threads,true);
    if(reference.empty())reference=run.comparison;
    if(reference.size()!=run.comparison.size())throw std::runtime_error("Benchmark workload/frame mismatch");
    double difference=0;
    for(std::size_t i=0;i<reference.size();++i) {
      if(!std::isfinite(reference[i])||!std::isfinite(run.comparison[i]))
        throw std::runtime_error("Nonfinite benchmark field or diagnostic");
      difference=std::max(difference,std::abs(reference[i]-run.comparison[i]));
    }
    if(difference>1e-8)throw std::runtime_error("Serial/OpenMP benchmark solutions differ");
    run.comparison.clear();run.comparison.shrink_to_fit();
    measurements.push_back({repetition+1,threads,std::move(run),difference});
  }
  std::ofstream out;out.exceptions(std::ios::failbit|std::ios::badbit);
  out.open(directory/"benchmark.json");out<<std::setprecision(17);
  out<<"{\"clock\":\"steady_clock wall seconds\",\"compiler\":";
#ifdef __VERSION__
  quote(out,__VERSION__);
#else
  quote(out,"unknown");
#endif
  out<<",\"hardware_threads\":"<<std::thread::hardware_concurrency()
     <<",\"repetitions\":"<<repetitions<<",\"subdivisions\":"<<divisions
     <<",\"side_length\":10,\"simulated_seconds\":100,\"time_step\":"<<step
     <<",\"quadrature_degree\":6,\"solver\":\"SparseLU\",\"solver_validation\":\"Checked\""
     <<",\"note\":\"Same executable and workload, serial backend versus requested OpenMP workers. Sequential runs, rotating order, no discarded warmup; machine load/frequency not controlled. Timings are nested: total includes observer diagnostics and export.\",\"runs\":[";
  bool first=true;
  for(const auto& m:measurements) {
    if(!first)out<<',';first=false;
    out<<"{\"repetition\":"<<m.repetition<<",\"threads\":"<<m.threads
       <<",\"backend\":\""<<(m.threads==1?"serial":"openmp")<<"\",\"execution_seconds\":"<<m.run.execution_seconds
       <<",\"export_seconds\":"<<m.run.export_seconds<<",\"max_difference\":"<<m.difference<<",\"cases\":[";
    for(std::size_t i=0;i<m.run.cases.size();++i) {
      if(i)out<<',';const auto& c=m.run.cases[i];
      out<<"{\"id\":";quote(out,c.id);
      out<<",\"setup_seconds\":"<<c.setup_seconds<<",\"integration_seconds\":"<<c.timings.total_seconds
         <<",\"integration_excluding_diagnostics_seconds\":"<<std::max(0.,c.timings.total_seconds-c.diagnostics_seconds)
         <<",\"diagnostics_seconds\":"<<c.diagnostics_seconds<<'}';
    }
    out<<"]}";
  }
  out<<"],\"summary\":[";first=true;
  for(int threads:policies) {
    std::vector<double> total;
    for(const auto& m:measurements)if(m.threads==threads)total.push_back(m.run.execution_seconds);
    if(!first)out<<',';first=false;
    out<<"{\"threads\":"<<threads<<",\"backend\":\""<<(threads==1?"serial":"openmp")
       <<"\",\"median_seconds\":"<<median(total)<<",\"min_seconds\":"<<*std::min_element(total.begin(),total.end())
       <<",\"max_seconds\":"<<*std::max_element(total.begin(),total.end())<<'}';
    std::cout<<(threads==1?"Serial":"OpenMP "+std::to_string(threads))
             <<": median "<<median(total)<<" s (range "<<*std::min_element(total.begin(),total.end())
             <<"–"<<*std::max_element(total.begin(),total.end())<<" s)\n";
  }
  out<<"]}\n";out.close();
  std::cout<<"Benchmark report: "<<directory/"benchmark.json"<<'\n';
}

int main(int argc,char** argv) {
  try {
    if(argc>1&&std::string(argv[1])=="--help") {
      std::cout<<"fem_long_waves [output.json [subdivisions [step [threads>=1]]]]\n"
          <<"fem_long_waves --benchmark [directory [repetitions [subdivisions [step]]]]\n"
          <<"Normal solves always return and print phase timings. --benchmark repeats the same workload\n"
          <<"with serial and 2/4 requested OpenMP workers (defaults: 3 repetitions, 20 subdivisions, dt=.05).\n";
      return 0;
    }
    if(argc>1&&std::string(argv[1])=="--benchmark") {
      if(argc>6)throw std::invalid_argument("Too many benchmark arguments; see --help");
      benchmark(argc>2?argv[2]:"build/wave-benchmark",argc>3?std::stoi(argv[3]):3,
          argc>4?std::stoi(argv[4]):20,argc>5?std::stod(argv[5]):.05);
    } else {
      if(argc>5)throw std::invalid_argument("Too many wave arguments; see --help");
      run_waves(argc>1?argv[1]:"build/long-waves.json",argc>2?std::stoi(argv[2]):20,
          argc>3?std::stod(argv[3]):.05,argc>4?std::stoi(argv[4]):1);
    }
  } catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
