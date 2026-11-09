# Concepts

What you need to know for a task, smallest first. The minimal path needs
five concepts; everything after it is optional and can be ignored until the
task asks for it. Module details: [cpp/discretization/README.md](../cpp/discretization/README.md),
[cpp/fem/README.md](../cpp/fem/README.md), [docs/fem/README.md](fem/README.md).

## The minimal path

```
CurveGeometry ──generate_mesh──▶ MeshResult ──┐
                                              ├─ solve_stationary ──▶ values
Problem (equation + conditions on regions) ───┘   evolve ──▶ frames
```

| Concept | What it is | Where |
| --- | --- | --- |
| `CurveGeometry` | the domain: analytic curves grouped in named regions (`RectangleGeometry`, `CircleGeometry`, `AnnulusGeometry`, ...) | `discretization/geometry.hpp` |
| `MeshOptions` → `MeshResult` | how fine to mesh, and the mesh with its `report`; check `report.size_satisfied` | `discretization/triangulate.hpp` |
| `Problem` | the equation (default −Δu = source) and one `BoundaryCondition` per region (`dirichlet`, `neumann`, `robin`, ...) written with `Expr` in `x`, `y`, `t` | `fem/problem.hpp` |
| `solve_stationary` | the solution at one time: `StationarySolution::values`, one per mesh node | `fem/solver.hpp` |
| `evolve` | time stepping with `TimeOptions` (method, start, end, step) from `InitialConditions`; `Evolution::frames` | `fem/evolution.hpp` |

[examples/fem/minimal.cpp](../examples/fem/minimal.cpp) is the whole path
in under 50 lines and a CTest smoke test. It constructs no plan, record or
decision: the solves make them from the options and attach them to the
result, where they wait until you need them. `error_norms` (L2, H1) checks
a result against an exact solution.

## Optional layers

Each layer builds on the ones above it and is opt-in.

**Settings as plans and decisions** (`fem/plan.hpp`, `fem/mesh_plan.hpp`,
`fem/decision.hpp`). A `SolvePlan` is the options of a solve plus one
`Decision` per setting saying why it has its value; a `MeshPlan` does the
same for `MeshOptions`. Every solve makes one from its options
(`plan_from_user`). You need plans to override a setting with its reason
(`plan.set_linear_solver(..., why)`), or to accept the output of a planner.

**Planners** (`fem/planner.hpp`). `SolvePlanner` and `MeshPlanner` are the
seams of a future automatic chooser; `UserPlanner` keeps the plan as given.
`plan_mesh(geometry, problem, planner, defaults)` plans before the first
mesh, `run_planner` checks a planner's plan. You need them only to write or
test a chooser.

**Analysis** (`fem/analysis.hpp`). `analyze(problem[, mesh])` reports
properties of a problem (time order, symmetry, ellipticity, boundary forms,
nullspace risk), each with its `Evidence`. A plan can carry the analysis it
relied on (`AnalysisRecord`, bound to that problem and mesh). You need it to
ask "is this problem symmetric/definite?" or to base a choice on that.

**Records** (`fem/record.hpp`, `fem/record_json.hpp`,
`discretization/mesh_record_json.hpp`). Every result carries a `RunRecord`
(plan, input fingerprints, mesh provenance, strategies, versions, work)
and every mesh a `MeshRunRecord`; `write_json` and `diff_records` compare
two runs. You need them to reproduce or compare runs, never to solve.

**Adaptivity** (`fem/adaptivity.hpp`). `adapt` (stationary) and
`adapt_evolution` (windows in time) repeat solve → estimate → mark → change
mesh → transfer. One `AdaptOptions` configures both (strategies:
`ErrorEstimator`, `MarkingStrategy`, `MeshAdaptation`, optional planners);
one `AdaptiveHistory` with one `AdaptiveStep` per solve or window reports
them. The components (`estimate_error`, `mark_bulk`, `refine_mesh`,
`transfer_field`) work on their own as well.

**Reference comparison** (`fem/reference/`). Problems with exact solutions
and `compare_plans`, `compare_mesh_plans`, `compare_planners` (always
against the identity baseline), which report error and work per entry. You
need them to show that a choice is better, not to solve.

## Naming

Inputs you configure end in `Options` (`MeshOptions`, `TimeOptions`,
`AdaptOptions`); settings with their decisions are `*Plan`; what happened is
a `*Record` (`RunRecord`, `MeshRunRecord`, `TransferRecord`,
`StrategyRecord`); a sequence of records is a `*History`. Strategies are
abstract classes with `name()` and `descriptor()` (`ErrorEstimator`,
`MarkingStrategy`, `MeshAdaptation`, `TimeIntegrator`, `StepController`,
`LinearSolverBackend`, the planners).
