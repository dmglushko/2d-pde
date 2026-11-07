# Build and independent verification

Geometry needs C++23, CMake and Threads, with no Eigen, SymEngine, Python or
network download. `tools/test.sh` configures an incremental Release build in
`build/geometry`. `tools/test_fem.sh` enables numerical dependencies in `build`.
Both accept additional CMake arguments. Set `PDE2D_BUILD_DIR`,
`PDE2D_BUILD_JOBS` (default 2), and `PDE2D_TEST_JOBS` (default 2).
CTest discovers all `tests/*.cpp` (`fem_*` only when FEM is enabled), and
`tools/*/test*.cjs`; each test has its own working/output directory. The legacy
relative mesh fixture is copied into its test directory. Showcase export is a
CTest fixture for its JS roundtrip and optional PNG tests. New geometry tests
are discovered on reconfiguration. Concurrency tests link `Threads::Threads`.

`PDE2D_ENABLE_OPENMP=ON` explicitly links the existing `pde2d_execution` target;
it defaults OFF. Compiling with OpenMP still leaves the runtime default serial.
`PDE2D_SANITIZERS=address,undefined` or `thread` optionally adds compiler/linker
instrumentation. Sanitizers and performance measurement are separate builds;
no correctness test has a wall-time threshold. SymEngine download is explicitly
opt-in with `PDE2D_FETCH_SYMENGINE=ON`. Missing LibXml2/node/reference support is
reported as UNAVAILABLE during configuration, never a passing full suite.

## Isolated reference environment

No project Python manager existed. This run used `.venv` with pip. Python 3.14
on this machine lacked ensurepip, so pip was bootstrapped into that venv using
`https://bootstrap.pypa.io/get-pip.py`; no global package changes were made.
Reproduce on a Python installation with venv support:

```sh
python3 -m venv .venv
.venv/bin/pip install -r tools/requirements-reference.txt
PDE2D_BUILD_DIR=build/reference tools/test_fem.sh \
  -DSymEngine_DIR=/tmp/pde2d-fem-build/_deps/symengine-build \
  -DPDE2D_FETCH_SYMENGINE=OFF -DPDE2D_ENABLE_REFERENCES=ON
```

All transitive runtime dependencies are frozen. The opt-in interpreter path is
`PDE2D_REFERENCE_PYTHON` (defaults to `.venv/bin/python`). Explicitly requesting
references without working imports fails configuration. The exporter lives in
`tools/reference`, is built only with FEM plus references, and is never
registered as a standalone test or linked into geometry-only builds.

The reference uses [scikit-fem's independent assembly](https://scikit-fem.readthedocs.io/en/latest/)
on the exact exported coordinates/connectivity, with a checked bijective
coordinate-to-DOF mapping. It assembles variable anisotropic diffusion,
nondivergence coefficient correction, reaction, convection, volume and boundary
loads, mixed essential/Neumann/Robin conditions and dynamic Robin first mass.
SciPy sparse solves compare the stationary solution. Independent mass,
stiffness and boundary forms drive heat and damped wave ODEs with nonzero
initial data and time-varying forcing. High-accuracy
[DOP853 and Radau](https://docs.scipy.org/doc/scipy/reference/generated/scipy.integrate.solve_ivp.html)
agreement checks the adaptive oracle; three fixed steps check BE first order,
CN and Newmark second order against that oracle. No exported C++ matrix is used
as the time-reference operator. JSON metrics are written in the CTest fixture.

Existing `fem_convergence` checks spatial convergence of Poisson and variable
anisotropic mixed-boundary problems. `fem_transient` checks dynamic Robin
heat/wave/damping convergence and moving quadratic data. New
`fem_moving_convergence` adds exponential moving Dirichlet plus dynamic Robin
data, forcing and damping, with temporal convergence rather than an exactly
reproduced time polynomial. `execution` checks invalid backend/thread policy,
small-work thresholds, deterministic values and lowest-index worker errors;
`fem_parallel_assembly` compares all assembled arrays exactly at 2/4 workers.

The plotter accepts topology prefixes v1–v4 and deliberately ignores metadata
trailers, including malformed metadata. It is not a restart-file validator.
Malformed/truncated/nonfinite/out-of-range topology must still fail. Loaded
meshes are shared for colors, individual images and gallery. The plot test
counts exactly one parse per fresh input and creates 19 PNGs plus a gallery
from current C++ v4 exports.

## Recorded run environment and limits

2026-11-06, Linux/WSL x86-64, AMD Ryzen 9 6900HX (8 cores/16 logical CPUs),
GCC 15.2.0, CMake 4.2.3, Node 24.22.1, Python 3.14.4, Eigen 3.4.0,
SymEngine 0.14.0 (existing `/tmp/pde2d-fem-build/_deps` installation).
Runtime Python versions are in the frozen requirements and result JSON.
Builds use Release `-O3 -DNDEBUG`, C++23, at most two compiler jobs;
CTest uses two jobs. Other agents may use this machine: timings are single-run
observations under uncontrolled contention, not speedup claims.

Before changing quadrature, a standalone binary was saved at
`build-verification/baseline/fem_quadrature`, built with
`g++ -std=c++23 -O3 -DNDEBUG -Icpp tests/fem_quadrature.cpp`, and run under
`/usr/bin/time -f '%e seconds'`: **18.85 s**, passed. Cached recurrence powers
retain the same loops over degrees 0–64, every edge exponent (2,145 checks),
every triangle monomial (47,905 checks), and all point/weight positivity and
normalization checks. Only powers are reused. No wall-time assertions exist.

The opt-in `PDE2D_BUILD_BENCHMARKS=ON` builds standalone geometry and, when FEM
is enabled, prepared-assembly benchmark executables; CTest never runs these.
For controlled benchmarking, run a selected executable separately after builds
and tests finish. This task reports no assembly/solver performance speedup.

Executed commands and complete CTest logs are preserved under
`build-verification/` in this worktree. It is a local build artifact directory,
not committed. The committed result JSON records independent error metrics.

After caching, `/usr/bin/time -f '%e seconds'
build-verification/serial/fem_quadrature` passed in **0.13 s**. These before/after
samples use the same compiler optimization and degree/positivity coverage, but
were not controlled repeated benchmark trials. The saved baseline binary can
be rerun independently.

Commands used for the numerical configurations:

```sh
cmake -S . -B build-verification/serial -DCMAKE_BUILD_TYPE=Release \
  -DPDE2D_BUILD_FEM=ON -DPDE2D_ENABLE_REFERENCES=ON \
  -DSymEngine_DIR=/tmp/pde2d-fem-build/_deps/symengine-build \
  -DPDE2D_FETCH_SYMENGINE=OFF
cmake --build build-verification/serial -j2
ctest --test-dir build-verification/serial --output-on-failure -j2
# Same configuration in build-verification/openmp, additionally:
# -DPDE2D_ENABLE_OPENMP=ON
OMP_NUM_THREADS=4 ctest --test-dir build-verification/openmp --output-on-failure -j2
```

OpenMP reference runs explicitly compare default serial results with 2- and
4-worker exports, including full stationary solutions and all nine evolution
results, for exact JSON numeric equality. Operation counts accompany each
fixed-step result. No numeric worker count is inferred from the environment
alone. The ordinary tests continue to use serial unless they explicitly select
another backend.

Observed analytical endpoints: variable anisotropic mixed-boundary L2 errors
at n=4,8,16 were 0.0060732, 0.00150608, 0.000375765. Moving exponential data
max errors at dt=0.1,0.05,0.025 were BE 0.0436669, 0.0225524, 0.0114707;
CN 0.000780144, 0.000194700, 0.0000486597; damped Newmark 0.00116695,
0.000293741, 0.0000736522. Existing dynamic Robin heat/wave/damping tests
also passed their temporal-rate assertions.

Full numerical CTest results: **24/24 serial**, **24/24 OpenMP-enabled**, both
with executed Python references and PNG generation. The latter checks explicit
2/4-worker kernels and full reference-export determinism. The largest assembly
entry discrepancy against scikit-fem was 3.552713678800501e-15, dynamic first
mass discrepancy 2.7755575615628914e-17, load discrepancy 5.551115123125783e-17,
and stationary nodal discrepancy 1.3322676295501878e-15. Finest-step max errors
against independent time integration: BE 0.007561503303624484, CN
0.00004613071378489764, Newmark 0.00014484707244766248; every refinement rate
assertion passed. Maximum DOP853/Radau difference was 4.140821019404939e-11.
See [machine-readable metrics](verification-results-2026-11-06.json) for all
steps, errors, versions and assembly/solve/reuse counts. For 20 time steps,
BE/CN each made 21 assembly calls, 20 linear solves and 19 solver reuses;
Newmark made 23 assembly calls, 21 solves and 19 reuses (initial acceleration
requires additional assembly and a mass solve).

Sanitizer configurations are provided but were not run in this task. SVG and
Node were available for both complete numerical runs. A separate configuration
with LibXml2 and Node deliberately unavailable reported those test groups as
UNAVAILABLE; references OFF likewise reports unavailable. Eigen and SymEngine
find-package disabling flags were unused in geometry mode, confirming neither
package was consulted. pip bootstrap version was 26.3. Optional dependencies
remain opt-in and no global Python/apt installation was performed.

Geometry script verification (clean and incremental):

```sh
PDE2D_BUILD_DIR=build-verification/geometry tools/test.sh \
  -DCMAKE_DISABLE_FIND_PACKAGE_Eigen3=TRUE \
  -DCMAKE_DISABLE_FIND_PACKAGE_SymEngine=TRUE -DPDE2D_BUILD_BENCHMARKS=ON
PDE2D_BUILD_DIR=build-verification/geometry tools/test.sh
```

Both runs passed **12/12**, including SVG, all old examples, viewer unit tests
and 19-export roundtrip. The optional geometry benchmark target compiled;
benchmarks were not executed for performance claims. The incremental build
compiled no unchanged sources. The final FEM script run uses the same serial
configuration described above. Initial implementation checks exposed a missing
private copy of the legacy relative fixture and an exporter call signature
error; both were fixed before these complete passing runs.

Final `tools/test_fem.sh` incremental run passed **24/24**. An explicit
`PDE2D_ENABLE_REFERENCES=ON` configuration with interpreter `/nonexistent`
failed configuration with the expected isolated-dependencies-unavailable
message; unavailable tools cannot silently count as successful references.
The committed metric artifact came from the OpenMP run with exact serial/2/4
export equality, and the final serial run reproduced its numerical errors.
