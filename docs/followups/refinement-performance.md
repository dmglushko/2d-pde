# Mesh refinement performance follow-up

Status: deferred; this document is a plan, not an implementation.

## Problem

In cpp/discretization/triangulate.hpp, each refinement insertion reconstructs Topology, scans all edges for the longest candidate, and queues all edges for legalization. RefineOnCurves rebuilds the polygon triangulation and reinserts surviving interior vertices after boundary updates. Geometric validation in mesh_validation.hpp enumerates every pair of edges. These costs grow with the complete mesh even when an edit is local.

No speedup or practical size threshold has been measured. Establish a baseline before selecting an implementation.

## Proposed work

1. Benchmark representative convex, concave, curved, and tagged meshes at increasing resolutions. Measure initial sampling, triangulation, each refinement policy, validation, and memory separately.
2. Retain adjacency throughout refinement and maintain a priority queue of oversized edges, with safe invalidation after splits and flips.
3. Seed legalization from affected edges. Preserve an independent final CDT verification path.
4. Investigate local curve-boundary retriangulation or batched updates. Specify node renumbering/removal and mapping before changing behavior.
5. Replace all-pairs intersection candidates with a spatial broad phase, retaining robust predicates and full external-input validation.

## Required invariants

Preserve Fixed, SplitPolygonEdges, and RefineOnCurves semantics; P1 connectivity; source-interval coverage and correspondence; region tags and protected junctions; explicit size/budget reports; transactional public operations; and mesh import/export compatibility. Do not weaken validation or add minimum-angle claims.

## Acceptance criteria

- Full tools/test.sh passes, including SVG, viewer, context, and resumed-refinement tests.
- Sanitizer checks pass and adversarial/near-degenerate cases remain rejected or correctly supported.
- Benchmarks record compiler, hardware, settings, work sizes, repeated-run timings, and memory. Compare measured baseline and proposed implementation; do not invent savings.
- New queues/adjacency are tested for stale entries and split/flip updates.
- Document topology/identity changes from curve updates and any remaining global fallback.

Implementation and algorithm choices remain for a future session.
