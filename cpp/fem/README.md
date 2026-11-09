# `cpp/fem` — scalar P1 finite elements

Header-only, optional module (CMake `PDE2D_BUILD_FEM=ON`; needs Eigen3 and
SymEngine). User-level documentation with equations, sign conventions and
examples is in [docs/fem/README.md](../../docs/fem/README.md). This page maps
the code.

## Pipeline

```
Problem (equation + boundary conditions)        problem.hpp
   │   symbolic coefficients (SymEngine Expr)    expression.hpp
   ├─ bind_boundary(mesh, problem) → regions, edge frames
   │                                             boundary_assignment.hpp
   ├─ analyze(problem[, mesh]) → ProblemProperties (each with Evidence)
   │                                             analysis.hpp, analysis_types.hpp
   ▼
SolvePlan {assembly, solver, time, decisions, mesh, analysis, planner}
   one binding table per plan kind (solve_plan_bindings, mesh_plan_bindings):
   setters (update), plan_from_user, validate_decisions derive from it
   analysis = make_analysis_record(problem[, mesh]) bound by fingerprints
                                                plan.hpp, mesh_plan.hpp
   SolvePlanner / MeshPlanner → plan (UserPlanner = identity), run_planner
                                                planner.hpp
   ▼
PreparedAssembly(mesh, problem)                 assembly.hpp
   ├─ bind_boundary (same binding as analysis)  boundary_assignment.hpp
   ├─ weak_form_coefficients(equation)          weak_form.hpp
   ├─ classify_edge_condition → EdgeForm        boundary_classification.hpp
   │    (DirichletEdge | NaturalEdge | ObliqueEdge; sampled divisor signs)
   │    one unit per form: prepare/evaluate/scatter/judge  detail/edge_forms/
   └─ P1 geometry + local kernels               p1_element.hpp, quadrature.hpp
   ▼
   ├─ interior elements, boundary edges, scatter detail/assembly_*.hpp
   ▼
AssembledSystem {K, M1, M2, F, Dirichlet}       system.hpp (element independent)
   ▼
constrain() + PreparedLinearSolver              constraints.hpp, prepared_linear_solver.hpp
   │   LinearSolverBackend (LU / CG / GMRES / user), optional initial guess
   │   contracts: linear_solver_types.hpp; Eigen backends: detail/eigen_backends.hpp
   │   (linear_solver.hpp includes all of them)
   ▼
stationary: PreparedStationary(mesh, problem, plan).solve(t[, guess])
            solve_stationary() = one-shot wrapper                solver.hpp
evolve(mesh, problem, InitialConditions, opts | plan + output)  evolution.hpp
   ├─ StepController (default: TimeGrid)        step_control.hpp, time_grid.hpp
   │    next() → trial; judge() accepts or rejects; rejected → retry
   ├─ TimeIntegrator (BE / CN / Newmark / user) time_integrator.hpp,
   │    one unit per built-in TimeMethod         detail/time_schemes/
   │    trial() / accept() / reject(); IntegratorServices: solve,
   │    start_dirichlet, assemble(t)
   └─ initial fields: Expr or nodal Vector      interpolation.hpp (Field)
   ▼
StationarySolution {values, report, record}     solver.hpp, record.hpp
Evolution {frames, reports, timings, record}
   RunRecord: plan, input fingerprints, mesh provenance, strategies, versions
   → write_report()                             report.hpp, timing.hpp, decision.hpp
   → write_json() / read_run_record_json() / diff_records()   record_json.hpp
L2 / H1 errors against exact solutions          error_norms.hpp

Adaptivity (stationary): solve → estimate → mark → refine → transfer → seed
   estimate_error → ErrorEstimate {indicators, global}   estimation.hpp
   MarkingStrategy (BulkMarking = mark_bulk, Dörfler)    marking.hpp
   discretization::refine_mesh(result, marked, options)  triangulate.hpp
   transfer_field(old mesh, values, new mesh)            transfer.hpp
   PreparedStationary(new mesh, …).solve(t, transferred) solver.hpp
   adapt(result, problem, plan, AdaptOptions) → AdaptiveHistory
       optional driver composing the above            adaptivity.hpp
       optional solve/mesh planners between iterations planner.hpp
       write_json(out, history)                       adaptivity_json.hpp
Transient restart: transfer_field(last frame) → InitialConditions{Vector}
Adaptivity (transient), windows of plan.time: evolve → estimate (last frame)
   → mark → refine → transfer_field + Dirichlet data → nodal restart → …
   adapt_evolution(result, problem, initial, plan, EvolutionAdaptOptions)
       → EvolutionAdaptiveHistory (per-window EvolutionWindow)  adaptivity.hpp
Reference problems with exact solutions                reference/problems.hpp
   compare_plans / compare_mesh_plans → PlanComparison  reference/compare.hpp
       (per plan: RunRecord, L2/H1 errors, work, timings; JSON)
MeshResult entry points refuse report.size_satisfied == false unless
   wrapped in accept_unsatisfied(result)          mesh_acceptance.hpp
```

One adaptive iteration composed by hand (see `tests/fem/adaptive_loop.cpp`,
where this is the oracle of the driver below):

```cpp
auto plan = plan_from_user(AssemblyOptions{}, cg); // an iterative solver
auto u = solve_stationary(result.mesh, problem, plan).values;
auto estimate = estimate_error(result.mesh, problem, u);
auto marked = mark_bulk(estimate.indicators, 0.6);
auto finer = discretization::refine_mesh(result, marked, result.options);
auto start = transfer_field(result.mesh, u, finer.mesh,
                            {.max_outside_distance = 0.02}); // curved domain
auto next = PreparedStationary(finer.mesh, problem, plan)
                .solve(0, start.values); // the transfer seeds the solve
```

The same loop as one call; the driver gives the same meshes and values bit
for bit and records every iteration:

```cpp
AdaptOptions adapt_options;
adapt_options.marker = std::make_shared<BulkMarking>(0.6); // default 0.5
adapt_options.max_iterations = 8;                         // refinements
adapt_options.transfer.max_outside_distance = 0.02;      // curved domain
auto history = adapt(result, problem, plan, adapt_options);
// history.mesh, history.solution, history.estimate: the final state
// history.iterations[k]: RunRecord, transfer, estimate summary, marking,
//                        refinement report, decisions; history.stop
write_json(std::cout, history);                           // adaptivity_json.hpp
```

`fem.hpp` includes everything. Parallel loops use `pde2d::parallel_for`
from `cpp/execution.hpp`; meshes are `discretization::TriangleMesh`.

## Headers

| Header | Responsibility | Depends on |
| --- | --- | --- |
| `evidence.hpp` | `Evidence` (Proven / Sampled / Unknown), `Property<T>` | — |
| `decision.hpp` | `DecisionSubject` and `decision_subject_table` (one row per subject: stable name, `DecisionOwner` Solve/Time/Mesh/Adapt; `decision_subject_name`, `decision_subject_owner`, `decision_subject_from_name`), `Decision` records (subject, choice, basis, evidence, source), `Rationale`, `find_decision`/`set_decision`; the binding row type `DecisionBinding<Options>` (subject, option, choice reader, recorded-by-default) with `bound_user_decisions`, `record_bound_decision`, `validate_bound_decisions`; `StrategyNames`, `PlannerRecord` | evidence, provenance |
| `expression.hpp` | `Expr`, symbols `x y t nx ny`, `require_symbols` by context, compiled `Evaluator` | SymEngine |
| `problem.hpp` | `Equation`, `BoundaryCondition` + factories, `Problem` | expression |
| `weak_terms.hpp` | `weak::Term` names, `WeakCoefficients<T>`, `WeakFlags` (no symbolic types) | — |
| `weak_form.hpp` | weak-form coefficients by `weak::Term`, `weak_time_dependence` | problem, weak_terms |
| `quadrature.hpp` | triangle and edge rules by exactness degree | — |
| `p1_element.hpp` | P1 gradients, edge frames/normals, local kernels | weak_form |
| `boundary_classification.hpp` | general BC → `EdgeForm` (`DirichletEdge`, `NaturalEdge` or `ObliqueEdge`), pure symbolic; `derivative_vectors`, `may_be_oblique`; form and divisor names from each form's unit | detail/edge_forms, p1_element, problem |
| `boundary_form.hpp` | `BoundaryForm` report category (Dirichlet, Natural, Mixed, Rejected, Oblique) and its name | — |
| `detail/edge_forms/interface.hpp` | `restrict_to_edge`, `edge_divisor_samples`, `divisor_keeps_strict_sign` (public); private: the unit interface `Unit<Form>` (`EdgeFormUnit` concept), `EdgeSite`, `EdgeResult`, `ScatterTarget`, `EdgeJudgement` | p1_element, quadrature, system, boundary_form |
| `detail/edge_forms/{dirichlet,natural,oblique}.hpp` | one unit per form: the public form struct and its name, divisors, prepared data, per-time evaluation, scatter and analysis judgement. `oblique.hpp` holds the oblique derivation | interface (oblique: natural) |
| `detail/edge_forms/forms.hpp` | `EdgeForm` variant; `static_assert` that every alternative has its unit; `PreparedEdgeForm` | the units |
| `boundary_assignment.hpp` | `bind_boundary`: validated region → condition binding (owning snapshot of the equation and one condition per assignment; edges refer to them by index) and the frame (length, outward normal) of every boundary edge; shared by assembly and analysis | p1_element, problem |
| `system.hpp` | `SparseMatrix`, `Vector`, `DirichletValue`, `DirichletDerivative`, `AssembledSystem`, `time_order`, `numerically_symmetric` | Eigen |
| `assembly_options.hpp` | `AssemblyOptions` (quadrature degree, boundary tolerance, execution), apart from the assembler so plans and records need no assembly | execution |
| `assembly.hpp` | `PreparedAssembly`, validation, parallel evaluation | all above, assembly_options, system |
| `detail/assembly_elements.hpp` | private: interior coefficients, quadrature points, element blocks | p1_element, weak_form |
| `detail/assembly_edges.hpp` | private: classification and preparation of the bound edges, per-time evaluation, all through the form units | boundary_assignment, boundary_classification, edge_forms |
| `detail/assembly_scatter.hpp` | private: deterministic serial scatter (edges through their units) | system, edge_forms |
| `linear_solver_types.hpp` | Contracts only: `LinearSolver`, `Preconditioner`, `SolverValidation`, `SolverOptions`, `InitialGuess`, `SolverReport`, `LinearSolution`, `BackendSolution`, `LinearSolverBackend` (`prepare`, `solve`, `solve_with_guess`), `LinearSolverBackendFactory` (`descriptor()`), `make_linear_solver_backend`, report names (`linear_solver_name`, `linear_solver_method_name`, `preconditioner_name`, `solver_validation_name`), the decision rows of the solver settings (`solver_option_bindings`) and `solver_decisions` derived from them. Included by plans and records | system, timing, decision, provenance |
| `constraints.hpp` | `constrain(A, b, dirichlet)`: Dirichlet lifting; the shared linear-system check | system |
| `prepared_linear_solver.hpp` | `PreparedLinearSolver` (validation, rank check, backend setup, true-residual check, reuse via `matches()`), `solve_linear` | linear_solver_types, constraints, detail/eigen_backends |
| `detail/eigen_backends.hpp` | private: built-in Eigen backends (SparseLU, CG, GMRES and their preconditioners), `with_builtin` (the only dispatch on `LinearSolver`), option validation, backend creation | linear_solver_types |
| `linear_solver.hpp` | Umbrella: includes `linear_solver_types.hpp`, `constraints.hpp` and `prepared_linear_solver.hpp` (include it for `solve_linear`) | the three |
| `mesh_plan.hpp` | `mesh_plan_bindings` (the binding table of a mesh plan, one row per Mesh-owned subject), `MeshPlan` (MeshOptions + decisions + `planner`; `update(subject, change, why)` and the setters `set_interior_algorithm`, `set_boundary_treatment`, `set_max_edge_length`, `set_size_field(field, descriptor)`, `set_refinement_criterion`), `mesh_plan_from_user`, `mesh_decisions` (a refinement criterion decision only when `RefinementOptions::criterion` is set), `mesh_decision_choice`, `size_field_choice`, `refinement_criterion_choice`, `mesh_options_difference`, `validate_decisions(MeshPlan[, used options])` | decision, discretization/mesh_types |
| `mesh_acceptance.hpp` | The mesh-report invariant by type: `AcceptedMesh`, `accept_unsatisfied(result)`, `require_size_satisfied` (MeshResult entry points refuse an unsatisfied report: "MeshResult misses its size target (...)") | discretization/mesh_types |
| `plan.hpp` | `TimePlan` (the one source of time settings), `SolvePlan` (assembly, solver, time, decisions, `mesh`, `analysis`, `planner`; `update(subject, change, why)` and the setters), `solve_plan_bindings` (time rows `time_plan_bindings`, the lifted `solver_option_bindings`, `assembly_plan_bindings`), `AnalysisRecord` (properties bound to problem and mesh fingerprints and sampling options; made only by `make_analysis_record`), `require_current_analysis`, `plan_from_user`, `time_decisions`, `validate_decisions` (stale/duplicate/misplaced decisions throw). No assembly, symbolic or edge-form headers | assembly_options, linear_solver_types, step_control, time_integration, mesh_plan, analysis_types |
| `planner.hpp` | `SolvePlanner` and `MeshPlanner` interfaces (`name`, `descriptor`, `plan(problem, properties, defaults, PlanningContext)`), the identity `UserPlanner`, `PlanningContext` (mesh, previous `AdaptiveIteration`, iteration), `run_planner` (validates and stamps the plan), `planner_choice` | plan, mesh_plan, analysis_types, fingerprint |
| `fingerprint.hpp` | `problem_fingerprint`, `field_fingerprint`, `field_kind`, `expression_text` (canonical SymEngine string) | problem, interpolation, provenance |
| `record.hpp` | `RunRecord` (plan, execution, workload, work with `solver_iterations`, `EvidenceSummary`, `VersionRecord`, `MeshRecord` with fingerprint and `MeshProvenance` (with the `discretization::MeshRunRecord` of the meshing call, `run`), problem fingerprint, stationary time, `InitialRecord`, `StationarySolveRecord` (initial-guess use and fingerprint, iterations, final residual), `StrategyRecord`s with `StrategyReplay`, including the plan's planners, `TimingRecord`), `make_run_record`, `find_strategy`, `summarize_evidence` | plan, execution, provenance, discretization/mesh_fingerprint |
| `json.hpp` | The FEM names (`pde2d::fem::JsonValue`, `json_number`, `json_string`, `json_bool`, `json_text`, `write_json(JsonValue)`, `parse_json`) of the core JSON document `cpp/json.hpp` (number tokens kept as text; classic locale, max_digits10), shared with discretization's mesh run records | cpp/json.hpp |
| `record_json.hpp` | `to_json(RunRecord)`, `write_json(std::ostream &, const RunRecord &)` (schema `pde2d.run_record`, version 1; mesh options and reports in discretization's encoding, the mesh run record under `mesh.provenance.run`), `read_run_record_json`, `diff_records` → `RecordDifference` (`RecordDiffOptions`: skips `timings` and `outcome` groups by default) | record, json, analysis_types, discretization/mesh_record_json |
| `solver.hpp` | `PreparedStationary` (prepared assembly + reused linear solver, `StationaryStatistics`; from a mesh, a checked `MeshResult` or an `AcceptedMesh`; validates the plan, its mesh plan and its analysis), `StationarySolution`, `solve_stationary` wrappers | assembly, constraints, prepared_linear_solver, mesh_acceptance, plan, record, fingerprint |
| `interpolation.hpp` | nodal interpolation of a field (rejects normals); `Field` (Expr or nodal Vector), `validate_field`, `nodal_values` | system, expression |
| `time_grid.hpp` | time stamps and step sizes | — |
| `time_integrator.hpp` | `TimeMethod`, `TimeIntegrator` interface (transactional `trial`/`accept`/`reject`, `descriptor()`), `IntegratorState`, `IntegratorServices`, `impose_compatible`, `TimeIntegratorFactory` | system, provenance |
| `detail/time_schemes/{backward_euler,crank_nicolson,newmark}.hpp` | one unit per built-in `TimeMethod`: `method`, `name`, `order`, `Integrator` (the public class, e.g. `NewmarkIntegrator`), `make()`, `descriptor()`; the classes take `method()`/`name()`/`order()`/`descriptor()` from their unit (`BuiltinScheme`) | time_schemes/interface |
| `detail/time_schemes/{interface,schemes}.hpp` | private: the `TimeSchemeUnit` concept, `BuiltinScheme`, `FirstOrder`; the unit list `Units` with a `static_assert` that every `TimeMethod` before `Custom` has its unit in order, and `with_time_method` (the only dispatch on `TimeMethod`) | time_integrator |
| `time_integration.hpp` | the built-in schemes and `time_method_name`, `time_method_order`, `time_method_descriptor`, `make_time_integrator`, all read from the units | time_integrator, detail/time_schemes |
| `step_control.hpp` | `StepController` interface (`judge` → `StepVerdict`, `descriptor()`), `UniformStepController` (TimeGrid), `step_size_consistent` rounding bound, `default_max_consecutive_rejections` | time_grid, time_integrator |
| `evolution.hpp` | `evolve()` driver (options or plan + `EvolutionOutput`; mesh or `MeshResult`), `TimeOptions` = `TimePlan` + `EvolutionOutput` + assembly/solver, trial/accept/reject loop, frames, observer, accounting, output/step checks, decisions, record | all above |
| `timing.hpp` | timing structs, `timing::Accumulate` | — |
| `analysis_types.hpp` | `AnalysisOptions`, `Sign`, `Definiteness`, `ProblemProperties`, `BoundaryRegionProperties` (incl. `oblique_edges`), `SystemProperties` (result types only: no symbolic or edge-form headers) | evidence, weak_terms, boundary_form |
| `classified_edge.hpp` | `ClassifiedEdge`, the edge input of region analysis (classified condition or rejection, edge frame) | boundary_classification, p1_element |
| `analysis.hpp` | `analyze()` → `ProblemProperties` / `SystemProperties` with evidence; `analyze_boundary_region`; `make_analysis_record(problem[, mesh], options)` (the only `AnalysisRecord` factory) and `refresh_analysis(plan, problem, mesh)` | analysis_types, classified_edge, boundary_assignment, plan, fingerprint, detail/analysis_* |
| `detail/analysis_interior.hpp` | private: constant signs, sampled signs and definiteness, interior properties | analysis_types, weak_form, quadrature |
| `detail/analysis_regions.hpp` | private: mesh-free region judgement, per-region accumulation of the units' judgements, Robin and tangential terms, nullspace | analysis_types, boundary_classification, edge_forms |
| `report.hpp` | text summaries of solves, evolutions, run records, decisions and analyses | evolution, solver, record, analysis |
| `error_norms.hpp` | prepared L2/H1 error norms (rejects normals) | system, quadrature |
| `estimation.hpp` | `ErrorEstimator` interface (with `descriptor()`), `GradientRecoveryEstimator` (ZZ, H1 seminorm), `estimate_error`, Dörfler `mark_bulk` | p1_element, problem, system |
| `marking.hpp` | `MarkingStrategy` interface (`name()`, `descriptor()`, `mark(estimate, mesh)`), `BulkMarking` (Dörfler via `mark_bulk`, default fraction 0.5), `mark_elements` (runs a strategy, checks ascending in-range indices) | estimation, provenance |
| `adaptivity.hpp` | Optional adaptive drivers. Stationary: `adapt(MeshResult | AcceptedMesh, problem, plan, AdaptOptions)` → `AdaptiveHistory` (`AdaptiveSettings`; per-iteration `AdaptiveIteration`: `RunRecord`, `TransferRecord`, `EstimateSummary`, `MarkingRecord`, requested refinement options, refinement `MeshReport` and mesh run record, planner/marking/refinement `Decision`s; `AdaptiveStop`; final mesh, solution and estimate); optional `AdaptOptions::solve_planner` and `mesh_planner`; per iteration it refreshes a mesh-bound analysis and updates `plan.mesh` explicitly. Transient: `adapt_evolution(MeshResult | AcceptedMesh, problem, initial, plan, EvolutionAdaptOptions)` → `EvolutionAdaptiveHistory` (`EvolutionAdaptSettings`, per-window `EvolutionWindow` with `RestartRecord`), `adaptive_window_times` | solver, evolution, assembly, analysis, planner, estimation, marking, transfer, mesh_acceptance, discretization/triangulate |
| `adaptivity_json.hpp` | `to_json(AdaptiveHistory)`, `write_json` (schema `pde2d.adaptive_history`, version 1; settings, each iteration embeds its run record), `read_adaptive_history_json`; `to_json(EvolutionAdaptiveHistory)` (schema `pde2d.evolution_adaptive_history`, version 1), `read_evolution_adaptive_history_json` | adaptivity, record_json |
| `reference/problems.hpp` | `ReferenceProblem` (name, `CurveGeometry`, `Problem`, optional `ReferenceTransient` with `InitialConditions` and window, exact solution, notes, suggested `MeshOptions`), the library (`reference_l_shape_corner`, `reference_boundary_layer`, `reference_variable_anisotropic`, `reference_oblique_square`, `reference_annulus_mixed`, `reference_heat_moving_peak`, `reference_wave_standing`), `reference_problems()`, `find_reference_problem` | discretization/geometry, problem, evolution |
| `reference/compare.hpp` | `compare_plans(reference, MeshPlan, {SolvePlan…})`, `compare_mesh_plans(reference, {MeshPlan…}, SolvePlan)` → `PlanComparison` (per `ComparisonEntry`: `RunRecord`, `ComparisonErrors` L2/H1, `ComparisonWork`, `ComparisonTimings`), `comparison_label`, `to_json`/`write_json` (schema `pde2d.plan_comparison`, version 1), `read_plan_comparison_json` | reference/problems, solver, evolution, analysis, error_norms, record_json, discretization/triangulate |
| `transfer.hpp` | `transfer_field`: P1 nodal field onto another mesh (point location + interpolation, documented outside rule) | discretization/point_location, system |

## Invariants

- **P1 only.** One scalar unknown per vertex; no element-order option.
  Higher order belongs in a separate module (see EXTENDING.md).
- **Preparation is symbolic, assembly is numeric.** Workers in parallel loops
  only call prepared `Evaluator`s; they never construct or copy `Expr`.
- **Determinism.** Scatter happens serially in element/edge order, so OpenMP
  and serial builds give bit-identical matrices.
- **Errors are exceptions** (`std::invalid_argument` for unsupported input,
  `std::domain_error` for non-finite values, `std::runtime_error` for
  solver failures). Nothing prints unless `write_report` is called.
- `*_detail` namespaces (and everything in `detail/`) are private. Tests and
  examples use public API only.
- **One judgement, one function.** Analysis and the solve share their
  checks: `bind_boundary` (region binding, edge frames),
  `weak_time_dependence` (caching), `divisor_keeps_strict_sign` on
  `edge_divisor_samples` (boundary divisors, Evidence::Sampled),
  `time_order` (integrator `check()`), `numerically_symmetric` (the CG
  backend). Do not re-implement one of them elsewhere.
- **Domains may have holes.** Nothing in the FEM module assumes one
  boundary loop: regions are bound per edge wherever they lie, and each
  edge's outward normal comes from its triangle, so on a hole it points into
  the hole (`nx`, `ny` in conditions). `tests/fem/annulus.cpp` solves a
  manufactured problem with Dirichlet data outside and a Robin condition on
  the hole at second order in L2.
- **Decisions are recorded and current.** Every choice that shapes a solve
  appears in `SolverReport::decisions` or `Evolution::record.plan.decisions`,
  copied from the `SolvePlan` that ran. Subjects are typed
  (`DecisionSubject`, one row each in `decision_subject_table` with a stable
  name and an owner). Each plan kind has one binding table
  (`solve_plan_bindings`, `mesh_plan_bindings`) that the decisions of the
  user's options, the setters (`update`) and validation all read; a
  `static_assert` fails for an owned subject without its row. Solves validate
  every decision against the option it explains (`validate_decisions`) and
  throw on a stale one or on a subject the plan does not own. Today all are
  user-selected (`plan_from_user`); nothing selects solvers automatically
  yet. A planner (`SolvePlanner`, `planner.hpp`) returns a `SolvePlan`.
- **Evidence is bound to its inputs.** `AnalysisRecord` carries the problem
  fingerprint, the mesh fingerprint (when it sampled a mesh) and its options,
  and only `make_analysis_record` makes one. `PreparedStationary`,
  `solve_stationary`, `evolve` and `adapt` throw "Stale analysis: ..." for
  evidence made for other inputs; `refresh_analysis` recomputes it, and
  `adapt` does so for every refined mesh.
- **Comparisons are plain solves.** `compare_plans` and
  `compare_mesh_plans` run the public meshing and solve calls on a reference
  problem and only measure them; an entry's values are those of the same
  solve run directly.
- **Results are reproducible.** `StationarySolution::record` and
  `Evolution::record` hold the plan, input fingerprints (mesh, problem,
  initial fields), mesh provenance, the stationary time, the strategies that
  ran (and whether they are replayable), library and dependency versions,
  execution policy, workload, work counters and evidence summary as
  structured fields; `write_json` serializes them (versioned schema) and
  `diff_records` compares two runs. Two runs of the same inputs differ only
  in `timings`.
- **One source per setting.** Time settings live in `TimePlan` only
  (`TimeOptions` derives from it and from `EvolutionOutput`); what ran is in
  `Evolution::record`.
- **One unit per built-in time scheme.** `TimeMethod` values before
  `Custom` each have a unit in `detail/time_schemes/` (name, order, factory,
  descriptor); `time_method_name`, `make_time_integrator` and the integrator
  classes read them through `with_time_method`, and `schemes.hpp` fails to
  compile for a value without its unit.
- **Contract headers stay light.** `plan.hpp`, `record.hpp`,
  `analysis_types.hpp` and `mesh_plan.hpp` include no assembly, symbolic or
  boundary-classification header. The `header_standalone` test compiles
  every public header in its own translation unit.
- **Bindings own their data.** `BoundaryBinding` snapshots the equation and
  conditions, so it outlives and ignores later changes to the `Problem`.
- **Steps are transactional.** Only accepted trials change integrator state,
  produce frames or reach the observer; the default controller accepts
  every trial, so the default path is unchanged. More than
  `TimeOptions::max_consecutive_rejections` rejections in a row throw.
- **One unit per boundary form.** Generic code (classification names,
  assembly, scatter, analysis) reaches a form only through
  `edge_form_detail::Unit<Form>`; `forms.hpp` fails to compile for an
  `EdgeForm` alternative without its unit.
- **Estimators report, they do not decide.** `estimate_error` returns
  indicators and a global value; marking (`MarkingStrategy`) and refinement
  are separate calls. The optional driver `adapt()` only composes them and
  records each marking and refinement as a `Decision`
  (`DecisionSubject::Marking`, `Refinement`) in its `AdaptiveIteration`;
  every component stays usable on its own.
- **Unsatisfied meshes are refused at the type boundary.** Entry points
  taking a `MeshResult` (`PreparedStationary`, `solve_stationary`, `evolve`,
  `adapt`) throw when `report.size_satisfied` is false; the caller opts in
  with `accept_unsatisfied(result)`. `TriangleMesh` overloads never check.
- **Normals only on boundaries.** `nx`, `ny` are bound in boundary
  expressions; interior coefficients, interpolated and initial fields and
  exact solutions are checked with `require_symbols(..., field_symbols, ...)`.

See [EXTENDING.md](EXTENDING.md) for adding terms, boundary forms, schemes,
linear solver backends and element types.
