# Implementation contract

C++23 module in namespace `fem`, using Eigen sparse matrices and SymEngine expressions.
Existing geometry headers/API stay independent of the new dependencies.

## Expressions (expression.hpp)
- `using Expr = SymEngine::Expression`; inline symbolic Expr `x,y,t,nx,ny`.
- `struct EvaluationPoint { double x=0,y=0,t=0,nx=0,ny=0; };`
- `Expr diff(const Expr&, const Expr& variable)`; `bool is_zero(const Expr&)` (algebraic expansion); `double evaluate(const Expr&, EvaluationPoint)`.
- `class Evaluator { public: explicit Evaluator(const Expr&); double operator()(EvaluationPoint) const; };` prepared repeated numerical evaluation; reject unbound symbols and nonfinite results.
- math helpers sin/cos/exp/sqrt/pow/log available as fem functions (wrappers if needed).

## Equation and BCs (problem.hpp, parent owns)
`struct Equation { Expr a_xx=-1,a_xy=0,a_yy=-1,b_x=0,b_y=0,c=0,source=0,m1=0,m2=0; };`
Convention: m2 u_tt + m1 u_t + a_xx u_xx + a_xy u_xy + a_yy u_yy + b_x u_x + b_y u_y + c u = source.
A=[[a_xx,a_xy/2],[a_xy/2,a_yy]]. D=-A. Natural flux q_n=n^T D grad u (so -Laplacian has q_n=normal derivative).
`struct BoundaryCondition { Expr alpha=0,gamma=0,beta_flux=0,beta_normal=0,beta_x=0,beta_y=0,value=0; };`
BC alpha*u + gamma*u_t + beta_flux*q_n + beta_normal*du_dn + beta_x*u_x + beta_y*u_y = value.
Builders `dirichlet(value)`, `neumann(value)` (normal derivative), `robin(alpha,beta,value)` (normal derivative), `dynamic_robin(gamma,alpha,beta,value)` (normal derivative), plus `flux_neumann(value)`, `flux_robin(alpha,beta,value)`, `dynamic_flux_robin(gamma,alpha,beta,value)`.
`struct BoundaryAssignment { vector<string> regions; BoundaryCondition condition; };`
`struct Problem { Equation equation; vector<BoundaryAssignment> boundaries; void assign(BoundaryCondition, initializer_list<string>); };`
No implicit zero Neumann. Missing tags, absent region names, overlapping edge assignments, contradictory corner Dirichlet values, nonfinite data, unsupported derivative conditions must error.
General oblique input retained; no promise to solve it. Normal derivative Robin supported when equivalent to flux (e.g. isotropic A); reject other cases with clear error. No silent projection of oblique derivatives. Gamma with nonzero flux supported as boundary contribution to m1. Derivative-free gamma != 0 can reject explicitly for now.

## Quadrature (quadrature.hpp)
`struct TriangleQuadraturePoint { array<double,3> barycentric; double weight; };` weights sum to 1 (physical area scaling in assembly).
`struct EdgeQuadraturePoint { double coordinate; double weight; };` [0,1], weights sum 1.
`vector<TriangleQuadraturePoint> triangle_quadrature(int degree=2);` positive-weight degree-two 3-point baseline, selectable higher degrees (at least 4,6,8; Duffy + Gauss valid).
`vector<EdgeQuadraturePoint> edge_quadrature(int degree=2);` degree refers to polynomial exactness, NOT points or FEM order. Reject invalid/excessive degrees explicitly.

## Assembly (assembly.hpp)
`using SparseMatrix = Eigen::SparseMatrix<double>; using Vector=Eigen::VectorXd;`
`struct AssemblyOptions { int quadrature_degree=2; double boundary_tolerance=1e-10; };`
`struct DirichletValue { Eigen::Index node; double value; };`
`struct AssembledSystem { SparseMatrix stiffness, first_mass, second_mass; Vector load; vector<DirichletValue> dirichlet; double time; };`
`AssembledSystem assemble(const discretization::TriangleMesh&, const Problem&, double time=0, const AssemblyOptions&={});`
Assemble UNCONSTRAINED matrices: M2 u_tt+M1 u_t+K u=F. Symmetric highest-order split with coefficient derivatives b_x - d_x a_xx - 0.5 d_y a_xy; b_y - d_y a_yy - 0.5 d_x a_xy. K highest part -A. BC alpha*u+gamma*u_t+beta*q=g contributes +alpha/beta to K, +gamma/beta to M1, +g/beta to F. Normals of straight mesh edges, outward derived independently of stored edge direction. P1 gradients physical. Validate mesh topology via existing validation once per assemble (can later cache).

## Solving/evolution (solver.hpp, evolution.hpp)
`enum class LinearSolver { SparseLU, ConjugateGradient, GMRES };`
`struct SolverOptions { LinearSolver method=LinearSolver::SparseLU; double relative_tolerance=1e-10; double absolute_tolerance=1e-12; int max_iterations=2000; int restart=40; };`
`struct SolverReport { bool converged; int iterations; double residual_norm; double relative_residual; string message; };`
`struct LinearSolution { Vector values; SolverReport report; };`
`LinearSolution solve_linear(const SparseMatrix&,const Vector&,const SolverOptions&={});`
`LinearSolution solve_stationary(mesh,problem,AssemblyOptions={},SolverOptions={});`
Dirichlet constraint helper available publicly: `pair<SparseMatrix,Vector> constrain(const SparseMatrix&,const Vector&,const vector<DirichletValue>&);` symmetric row/column elimination + RHS correction. Singular stationary pure Neumann detected/reported (don't silently pin). Use default LU for broad inputs; CG must validate symmetry and positive definiteness or require explicit documented assumption. Nonconvergence report returned by solve_linear, higher APIs throw on failed step (or report unmistakably, no success mislabel).
`enum class TimeMethod { BackwardEuler, CrankNicolson, Newmark };`
`struct TimeOptions { TimeMethod method=TimeMethod::BackwardEuler; double start=0,end=1,step=0.01; AssemblyOptions assembly; SolverOptions solver; };`
`struct Frame { double time; Vector values; Vector velocity; };`
`struct Evolution { vector<Frame> frames; vector<SolverReport> reports; };`
`Evolution evolve(mesh,problem,const Expr& initial_value,const Expr& initial_velocity=Expr(0),const TimeOptions&={});`
BE/CN for m2=0, Newmark average acceleration beta=1/4,gamma=1/2 for m2 !=0 (m1 damping supported too). Time-varying coefficients reassemble; CN trapezoidal residual at endpoints. Boundary gamma contributes M1. Time-varying strong Dirichlet exact at each step with correct lifting; Newmark initialize acceleration with boundary g_tt, velocity initial boundary g_t; validate initial displacement/velocity compatibility. Avoid zero-diagonal singular mass failure for stationary cases: reject wrong integrator explicitly. End time reached exactly with shorter last step. Initial frame always included. Validate options finite/positive, finite outputs. Newmark includes damping and boundary storage even when bulk m1=0.
