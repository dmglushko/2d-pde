# Performance implementation coordination

Starting implementation snapshot follows audit of 695d785. Read the audit and
assigned GitHub issue bodies. Work in the T3-provided worktree, commit only your
owned files, and return SHA plus test/benchmark results. No force pushes, main
branch changes, PR creation, issue closure or extra subagents from children.
Only OpenAI models are authorized. Preserve public compatibility unless agreed
here. C++23; no unsafe fast-math. Existing package can be found with
`-DSymEngine_DIR=/tmp/pde2d-fem-build/_deps/symengine-build
-DPDE2D_FETCH_SYMENGINE=OFF`; do not rebuild/install dependencies globally.
Each child gets its own build directory; limit build jobs to 2 during concurrent
work and avoid asserting speedups from contended wall-clock samples.

## Ownership / scheduling

First wave:
- Assembly agent: #5 and assembly portion of #10. Own `cpp/fem/assembly.hpp`,
  `cpp/fem/prepared_assembly.hpp`, `cpp/fem/expression.hpp` if necessary,
  `cpp/fem/quadrature.hpp` if necessary, `tests/fem_prepared_assembly.cpp`,
  `tests/fem_parallel_assembly.cpp`, and its benchmark/docs under a unique name.
- Solver/evolution agent: #6. Own `cpp/fem/solver.hpp`, `cpp/fem/evolution.hpp`,
  `cpp/fem/timing.hpp`, new solver/evolution tests and its benchmark/docs.
- Mesh agent: #3, validation portion of #10. Own `triangulate.hpp`,
  `mesh_validation.hpp`, `boundary_context.hpp`, `mesh_io.hpp`, new mesh
  performance/robustness tests and its benchmark/docs. Preserve public APIs and
  add optional execution arguments only at the end with serial defaults.

Second wave once slots are free:
- Geometry agent: #4 and #8. Own geometry/length/sampling/SVG headers and focused
  concurrency, accuracy and performance tests. Coordinate robust validation
  helper use through the shared execution policy.
- Diagnostics/viewer agent: #7. Own diagnostic helpers, FEM examples and viewers
  (including generated gallery packaging), new diagnostic/viewer tests. Do not
  change FEM solver/assembly headers, geometry or build scripts.
- Build/reference agent: #9, #10 verification, #11. Own root CMake, test scripts,
  new analytical/reference tests and exporters, Python dependency files,
  `tools/plot_mesh.py` and plotter tests, documentation for running these.

Root coordinates common API scaffolding, integration, independent review,
overall measurements, issue bookkeeping and documentation. Do not overwrite
another owner's files. Necessary small integration fixes belong to root.

## Shared execution API

`cpp/execution.hpp`, namespace `pde2d`:
```
enum class ExecutionBackend { Serial, OpenMP };
struct ExecutionOptions {
  ExecutionBackend backend=ExecutionBackend::Serial;
  int threads=0; // zero uses runtime maximum; no negative values
  size_t min_parallel_size=64;
};
bool openmp_available();
void validate_execution(const ExecutionOptions&);
template<class F> void parallel_for(size_t count,const ExecutionOptions&,F&&);
```
Root supplies the helper. Default serial, optional `_OPENMP` backend. A selected
unavailable backend is an explicit error, even if work is small. Worker errors
are joined and rethrown, with the lowest failing work index chosen. No shared
sparse writes; numerical kernels must use independent output slots and stable
reduction. Do not create symbolic expressions or copy RCP-containing objects
inside workers. Keep setup outside parallel regions.
`AssemblyOptions` gains `pde2d::ExecutionOptions execution{}` after its existing
fields, preserving aggregate initializers. Geometry-only clients retain serial
headers without mandatory Eigen/SymEngine or OpenMP.

## Prepared assembly API (shared with solver agent)

Root supplies a compatibility adapter at this interface; assembly agent replaces
its internals with real preparation/cache. The adapter is scaffolding, not a
performance claim.
```
class PreparedAssembly {
 public:
  PreparedAssembly(const TriangleMesh&,const Problem&,const AssemblyOptions&={});
  AssembledSystem assemble(double time=0,int dirichlet_derivative_order=0) const;
  const TriangleMesh& mesh() const;
  const Problem& problem() const;
  const AssemblyOptions& options() const;
};
```
Own immutable input snapshots. `dirichlet_derivative_order` 0,1,2 returns values,
velocities, accelerations AFTER edge-local boundary reduction, as the current
implementation does. All current assembled-system fields stay compatible.
The convenience `fem::assemble` remains. Reuse constant operators/load and
physical/normal/region/quadrature data; update time-dependent fields correctly.
The result still owns its sparse matrices/vectors. Add optional introspection
counters/tests where useful without changing required semantics.

## Reusable solvers and evolution

Provide `PreparedLinearSolver` with owned constrained matrix and solver options,
repeated `solve(const Vector&)` returning `LinearSolution`, and preparation
metrics. The one-shot API stays. Strict rank/SPD diagnostics stay, but preparation
is reused. Detect actual constrained matrix/options changes, including initial
mass versus the effective wave matrix. Do not cache by address or sparsity alone.
Nominal fixed steps must reuse despite timestamp-rounding noise; keep correct
large-origin, underflow, final-step, nonconvergence and moving-boundary behavior.
Use PreparedAssembly above throughout evolution. Preparation and per-RHS solve
metrics must not double-count.

Append to `TimeOptions`:
```
size_t frame_stride=1; // positive: initial, every Nth step, final always retained
bool store_reports=true;
std::function<void(const Frame&)> observer{}; // initial and EVERY accepted step
```
Observer is synchronous on the caller thread; reference lives only during the
call. Exceptions propagate. This permits full-step energy/work checks without
storing every state. Defaults retain existing frames/reports. Add actual time
step/preparation counters to timings if useful. Diagnostics agent will adapt
examples using this contract, not infer step counts from retained frames.

## Verification and documentation

Tests must include nonconstant spatial/time coefficients, mixed terms, moving
Dirichlet and dynamic Robin including u_t. Parallel and serial tests must verify
answers and errors, not just return success. Analytical convergence and external
scikit-fem/SciPy comparisons are required; avoid Eigen-vs-Eigen circular checks.
Use the Python dependency-management skill for new third-party imports and
isolated requirements. Optional dependencies absent must be labeled skipped or
unavailable; execute the reference suite in this environment before final handoff.
Root will enable CMake OpenMP/geometry targets and merge all owners' additions.
State sanitizer/runtime failures honestly. Keep benchmarks separate from tests
and report baseline, work settings, actual timings and remaining fallbacks.
