# Equations, boundary conditions, and solutions

The optional C++23 `fem` module connects the existing triangle meshes to symbolic
coefficient functions, P1 assembly, sparse linear solvers, and time evolution.
The geometry API remains usable without the numerical dependencies.

Open [the computed solution gallery](showcase.html) for two stationary problems,
heat diffusion, a vibrating membrane, a damped membrane, and a wave problem with
an explicit `u_t` term on its boundary. Select **Wave: vibrating membrane** and
press **Play**. The gallery includes exact-solution comparisons and a separate
mesh-refinement experiment. Frames are calculated in C++, not simulated in the
viewer.

The [100-second wave examples](long-waves.md) use a larger 10-by-10 square:
a localized source with fixed Dirichlet edges, and a source-free Neumann wave.
They include energy/work checks and a recorded execution-time breakdown.

## Build and reproduce

Prerequisites: C++23 compiler, CMake 3.20+, Boost headers, Eigen3 development
headers, and Node.js for packaging the standalone gallery. SymEngine is a
compiled dependency. CMake uses an installed SymEngine when available; otherwise
it downloads the hash-verified v0.14.0 release and builds it with Boost integers.
The first configure needs network access in the latter case.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
ctest --test-dir build --output-on-failure
./build/fem_showcase build/fem-showcase.json
node tools/render_solutions.mjs build/fem-showcase.json docs/fem/showcase.html
```

For an existing SymEngine installation, pass `-DSymEngine_DIR=/path/to/lib/cmake/symengine`.
Set `-DPDE2D_FETCH_SYMENGINE=OFF` to forbid dependency downloads. Link a CMake
consumer against the interface target `pde2d_fem`. The existing geometry and SVG
tests remain available through `./tools/test.sh`.

## A complete stationary problem

```cpp
#include "discretization/geometry.hpp"
#include "discretization/triangulate.hpp"
#include "fem/solver.hpp"

using namespace fem;
discretization::RectangleGeometry geometry(1,1);
geometry.setCenter(.5,.5);
discretization::MeshOptions meshing;
meshing.boundary.max_edge_length=.1;
meshing.boundary.preserve_corners=true;
meshing.refinement.max_edge_length=.15;
auto generated=discretization::generate_mesh(geometry,meshing);
if(!generated.report.size_satisfied)
    throw std::runtime_error(generated.report.message);

Problem problem;
problem.equation.source=1; // Default operator is -u_xx-u_yy.
auto grounded=dirichlet(0);
problem.assign(grounded,{"left","right"});
problem.assign(neumann(0),{"bottom"});
problem.assign(robin(2,1,0),{"top"}); // 2*u + du/dn = 0.
AssemblyOptions integration;
integration.quadrature_degree=6;
auto solution=solve_stationary(generated.mesh,problem,integration);
// solution.values[i] is the value at generated.mesh.nodes[i].
// solution.report contains the measured algebraic residual.
```

One condition can be assigned to multiple named regions. Conditions and
assignments live on the problem; geometry tags remain reusable independently.
Every mesh boundary edge must receive exactly one assignment. Missing regions,
unknown names, multiple assignments to an edge, and incompatible Dirichlet
values at shared corners are errors. There is no implicit zero-Neumann fallback.

## Equation convention and symbolic data

The scalar input equation is

\[
m_2u_{tt}+m_1u_t+a_{xx}u_{xx}+a_{xy}u_{xy}+a_{yy}u_{yy}
+b_xu_x+b_yu_y+cu=f.
\]

Set fields `m2`, `m1`, `a_xx`, `a_xy`, `a_yy`, `b_x`, `b_y`, `c`, and `source`
on `problem.equation`. The mixed coefficient multiplies `u_xy` once: there is
**no implicit factor of two**. Coefficients may depend on `x,y,t` and remain
independent of the unknown. This is a non-divergence-form input; e.g.
`a_xx=-a` represents `-a*u_xx`, not `-(a*u_x)_x`. Time coefficients likewise
multiply derivatives directly: `m1*u_t` does not mean `(m1*u)_t`.

`Expr` is an actual SymEngine expression. Use `x,y,t` and mathematical functions
to construct formulas in C++; use `diff(expr,x)` for a symbolic derivative.
Boundary expressions may additionally use `nx,ny`, the outward normal of the
straight mesh edge. `Evaluator` prepares an expression for repeated numerical
evaluation while retaining its symbolic tree. Unknown free symbols and
nonfinite evaluations are rejected. No user-provided opaque callback is needed.

Coefficients must be sufficiently smooth for the coefficient derivatives used
by the weak-form conversion. This module does not define interface laws for
discontinuous strong-form coefficients. Mixed space-time derivatives, nonlinear
terms, and coupled systems are outside the current implementation.

## Boundary representation

The full local input is

\[
\alpha u+\gamma u_t+\beta_q q_n+\beta_n\partial_nu
+\beta_xu_x+\beta_yu_y=g.
\]

These map to the fields `alpha`, `gamma`, `beta_flux`, `beta_normal`, `beta_x`,
`beta_y`, and `value` of `BoundaryCondition`. Keeping `beta_normal` explicit makes
normal derivative conditions convenient even though `du/dn=nx*u_x+ny*u_y`.

For this module's sign convention,

\[
A=\begin{pmatrix}a_{xx}&a_{xy}/2\\a_{xy}/2&a_{yy}\end{pmatrix},
\qquad q_n=-n^T A\nabla u.
\]

Thus `q_n=du/dn` for `-Delta u`; for `-k Delta u`, it is `k du/dn`.
This defines a signed mathematical conormal derivative; it is not automatically
a physical outward heat flux. In anisotropic equations it need not be parallel
to the normal derivative. The sign is explicit because the earlier design
discussion's integration-by-parts expression `F_n=n^T A grad(u)` is `-q_n`.

Convenient builders:

| Builder | Condition |
| --- | --- |
| `dirichlet(g)` | `u=g` |
| `neumann(g)` | `du/dn=g` |
| `robin(alpha,beta,g)` | `alpha*u+beta*du/dn=g` |
| `dynamic_robin(gamma,alpha,beta,g)` | `gamma*u_t+alpha*u+beta*du/dn=g` |
| `flux_neumann(g)` | `q_n=g` |
| `flux_robin(alpha,beta,g)` | `alpha*u+beta*q_n=g` |
| `dynamic_flux_robin(gamma,alpha,beta,g)` | `gamma*u_t+alpha*u+beta*q_n=g` |

The representation is broader than the supported weak boundary treatment.
Dirichlet and flux Robin, including dynamic flux Robin, are supported. Normal
Robin is supported when it can be converted to the assembled natural flux.
Unsupported oblique derivative conditions fail explicitly. General input does
not imply that every boundary problem is well posed or supported numerically.
Equivalence checks first restrict expressions to each straight edge. Divided
Robin multipliers must stay nonzero on that edge; sampled zeros and sign changes
are rejected. These checks cannot prove nonvanishing for arbitrary symbolic
functions, so that remains a requirement on the supplied data.

Boundary `gamma*u_t` adds a boundary integral to the first-derivative matrix.
It is present during both heat and wave evolution. It uses the trace of the
same nodal field as the interior; it does not create a separate boundary field.

## Assembly and quadrature

With a symmetric split of the mixed derivative, assembly produces

\[
M_2\ddot U+M_1\dot U+KU=F.
\]

The interior stiffness uses `-A` in its gradient terms, and the effective
first-derivative coefficients are

\[
b_x-\partial_x a_{xx}-\tfrac12\partial_y a_{xy},\qquad
b_y-\partial_y a_{yy}-\tfrac12\partial_x a_{xy}.
\]

SymEngine differentiates these coefficient expressions. A condition
`alpha*u+gamma*u_t+beta*q_n=g` adds `alpha/beta` to the boundary stiffness,
`gamma/beta` to boundary `M1`, and `g/beta` to the boundary load.
`assemble(mesh,problem,time,options)` returns unconstrained matrices and a list
of Dirichlet values. `constrain` eliminates both constrained rows and columns
and adjusts the load, preserving symmetry when the original matrix is symmetric.

`quadrature_degree=2` is the default: the triangle rule has three positive
weights and integrates total-degree-two polynomials exactly. Higher degrees
through 64 use positive Gauss rules with a triangle Duffy transform. Edge
integration uses positive Gauss–Legendre rules on the straight edges. The option
is polynomial exactness, not interpolation order; elements remain P1. Higher
quadrature cannot compensate for an under-resolved mesh or nonsmooth data.

## Linear solvers and time integration

`SolverOptions::method` selects `SparseLU` (default), `ConjugateGradient`, or
`GMRES`. Solver reports use the actual residual of the solved algebraic system.
An algebraic residual measures how accurately the discrete equations were
solved; it does not measure PDE approximation error. The gallery reports both
residuals and continuous exact-solution errors.

For time evolution, include `fem/evolution.hpp`:

```cpp
problem.equation.m2=1; // u_tt-Delta u=0: a vibrating membrane.
problem.equation.source=0;
problem.boundaries.clear();
problem.assign(dirichlet(0),{"bottom","right","top","left"});
TimeOptions time;
time.method=TimeMethod::Newmark;
time.step=.01;
time.end=2;
const double pi=std::numbers::pi;
auto motion=evolve(generated.mesh,problem,sin(pi*x)*sin(pi*y),Expr(0),time);
// motion.frames includes the initial frame and the exact final time.
```

Backward Euler and Crank–Nicolson handle first-order evolution. Newmark uses
average acceleration (`beta=1/4`, `gamma=1/2`) and supports both `m2*u_tt` and
`m1*u_t` simultaneously, including dynamic boundary contributions. Data are
reassembled at each required time; no constant-coefficient assumption is made.
A shorter final step reaches the requested end time.

Initial displacement must agree with strong Dirichlet data. Wave evolution also
requires compatible initial velocity. Moving Dirichlet boundaries require
consistent lifting; boundary values are enforced at each step. Expressions
used for moving prescribed values must be differentiable in time.

Pure Neumann stationary diffusion has a constant nullspace. This implementation
reports failure instead of choosing a normalization silently. Add an appropriate
Dirichlet condition or a coercive reaction/Robin term; automatic mean-zero gauges
are not implemented. Ordinary Galerkin P1 has no transport stabilization, and the
API does not certify ellipticity, well-posedness, or stability for arbitrary
coefficient choices. Time stepping currently repeats mesh validation, symbolic
preparation, and sparse factorization; caching and factor reuse remain future
performance work.
