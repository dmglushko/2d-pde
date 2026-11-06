# Mesh refinement and validation, issue #3

Baseline: `15bc00d5fe957b6fbc651dfa4841fb034ea1437d`, before owned-header edits.
Binaries and logs are retained at `/tmp/pde2d-mesh-issue3/{before,after}` and
in that directory on the implementation machine; raw CSV and GNU time reports
are checked in beside this file. Baseline executable was compiled and started
before changing headers. No timing thresholds are used by correctness tests.

GCC 15.2.0, C++23, `-O2 -Wall -Wextra -Werror`, no fast-math, serial execution,
AMD Ryzen 9 6900HX, WSL2/Linux, 16 logical CPUs. Other implementation work was
running concurrently: these are measured observations under contention, not
isolated-machine speedup guarantees. Builds used at most two compiler jobs.
GNU time peak RSS covers the entire benchmark process, not individual phases.
Sampling includes geometry sampling/simplification; seed includes polygon
validation/ear clipping; refinement includes entry/context validation, initial
legalization, insertion and final validation. I/O times export plus import and
include all their external-input checks. Times are milliseconds.

`benchmarks/mesh_issue3.cpp` runs rectangle, L-shaped concave polygon, circle,
and rectangle with a split bottom region. Each runs Fixed, SplitPolygonEdges,
and RefineOnCurves at h=0.3, 0.15, 0.075, three repeats each (108 rows).
Fixed starts with boundary sampling length h; other policies start at 0.5.
Curve deviation is 0.01; insertion budget 10000; regions retained; default
corner preservation false. Raw files record boundary count, final node and
triangle counts, accepted insertion counts and every phase separately.
All 108 before/after node/triangle/insertion counts agree. This comparison
checks work sizes, not bitwise mesh identity or a minimum-angle guarantee.

## Implementation and retained fallbacks

Straight refinement owns one persistent Topology. An oversized-edge heap orders
by descending double length then ascending undirected vertex key, matching the
old ordered-map scan's strict-greater tie rule. A monotonically increasing
per-update generation invalidates stale and recreated keys. Every triangle add,
including flips, updates affected edge generations/heap entries. Initial global
legalization remains even when h=0 or budget=0. Later splits legalize their
changed neighborhood. Pending legalization keys are deduplicated until popped;
a subsequent adjacency change can enqueue them again. Owner sets remain
allocated sets with explicit third-owner rejection; no bounded-owner truncation.

Polygon/mesh validation and ear blockers use a static median-partition AABB
index. Inclusive boxes retain endpoint contact, overlap and zero-width boxes;
robust orientation/intersection/on-segment predicates remain authoritative.
Candidates are sorted by original index so mesh validation retains pair order.
All topological, connectivity, nonmanifold and boundary-record checks remain.
Worst-case broad-phase candidate counts can still be quadratic for overlapping
boxes. Ear clipping still restarts its first-valid-ear search and maintains
inclusive collinear blockers; no cache assumes neighbor-only invalidation.

Provenance point/segment indexes retain the original tolerance tests, expanded
outward by one representable coordinate value. Both endpoints must still be
supported by the same snapshot segment. Source coverage, directed records,
region junction and corner checks are unchanged. External I/O validates geometry
once and then calls a read-only CDT verifier over edge opposites; it no longer
revalidates geometry or copies the complete mesh for that report check. Public
`is_constrained_delaunay` still validates its external input independently.

Curved boundary splits still perform a validated global polygon rebuild and
global legalization. Boundary vertices are ordered by the directed traversal;
protected vertices, tags and source intervals follow that order. Surviving old
interior vertices are reinserted in original order; removed ones retain the
existing per-rebuild removal-message behavior. Boundary-first renumbering,
retained interior ordering and index changes remain the existing contract.
Classification is now a single strict-inside traversal. Exact coordinate lookup
and conservative triangle-box rejection reduce reinsertion work, but triangle
point location still scans boxes linearly. Boundary-record vector insertion
and actual boundary-edge lookup still take linear work; interior edges bypass
that lookup. Topology mutation remains serial. No optional parallel validation
API was added: the reduced numerical candidate work did not justify introducing
thread scheduling here without a separate measured experiment.

## Commands

Run from repository root. The directory is intentionally persistent to retain
baseline binaries and artifacts. Execute compiles serially or with at most two
concurrent compiler processes.

```sh
mkdir -p /tmp/pde2d-mesh-issue3
g++ -std=c++23 -O2 -Wall -Wextra -Werror -Icpp benchmarks/mesh_issue3.cpp -o /tmp/pde2d-mesh-issue3/before
/usr/bin/time -v -o /tmp/pde2d-mesh-issue3/before-memory.txt /tmp/pde2d-mesh-issue3/before > /tmp/pde2d-mesh-issue3/before.csv
# The preceding two commands ran before editing the owned headers.
g++ -std=c++23 -O2 -Wall -Wextra -Werror -Icpp benchmarks/mesh_issue3.cpp -o /tmp/pde2d-mesh-issue3/after
/usr/bin/time -v -o /tmp/pde2d-mesh-issue3/after-memory.txt /tmp/pde2d-mesh-issue3/after > /tmp/pde2d-mesh-issue3/after.csv
/tmp/pde2d-mesh-issue3/after /tmp/pde2d-mesh-issue3/benchmark-roundtrip.pde2d > /tmp/pde2d-mesh-issue3/candidates.csv
bash tools/test.sh
g++ -std=c++23 -O2 -Wall -Wextra -Werror -Icpp tests/mesh_performance_regression.cpp -o /tmp/pde2d-mesh-issue3/regression
/tmp/pde2d-mesh-issue3/regression
g++ -std=c++23 -O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer -Icpp tests/mesh_performance_regression.cpp -o /tmp/pde2d-mesh-issue3/regression-asan
ASAN_OPTIONS=detect_leaks=1 /tmp/pde2d-mesh-issue3/regression-asan
g++ -std=c++23 -O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer -Icpp tests/discretization.cpp -o /tmp/pde2d-mesh-issue3/discretization-asan
ASAN_OPTIONS=detect_leaks=1 /tmp/pde2d-mesh-issue3/discretization-asan /tmp/pde2d-mesh-issue3
g++ -std=c++23 -O1 -g -fsanitize=thread -Icpp tests/mesh_performance_regression.cpp -o /tmp/pde2d-mesh-issue3/regression-tsan
/tmp/pde2d-mesh-issue3/regression-tsan
```

The focused test checks the flip-created length-1.0625 witness at target 1,
zero/sufficient/exhausted budgets, disabled sizing with initial legalization,
recreated generations, stale heap entries, ordered full-scan versus heap choices
after repeated splits/flips, equal-length ties, pending draining and re-enqueue,
a separate triangle-pair CDT oracle, randomized exhaustive AABB comparisons,
touching/degenerate boxes, collinear ears, deterministic repeated refinement,
fixed-boundary reports and third-owner rejection. Existing discretization/SVG/
context/viewer/showcase tests provide the broader provenance, precision,
rollback, malformed-input and resumed-refinement coverage. The new test is a
standalone target for the build owner to register; no build/tool files changed.

## Recorded results

Median of three repeats at h=0.075; phase cells are before / after ms.

| Shape | Policy | Nodes / triangles / insertions | Refinement | Validation | Context | Export + import |
|---|---|---|---|---|---|---|
| Rectangle | Fixed | 843 / 1588 / 747 | 414.582 / 14.166 | 21.111 / 3.823 | 2.199 / 0.157 | 84.699 / 10.156 |
| Rectangle | SplitPolygonEdges | 961 / 1824 / 949 | 533.412 / 12.252 | 23.184 / 4.490 | 0.082 / 0.073 | 114.544 / 11.971 |
| Rectangle | RefineOnCurves | 961 / 1824 / 949 | 876.047 / 419.186 | 35.344 / 4.483 | 2.228 / 0.154 | 138.040 / 13.148 |
| Concave | Fixed | 1279 / 2428 / 1151 | 920.851 / 22.258 | 48.043 / 6.035 | 2.645 / 0.197 | 203.991 / 16.395 |
| Concave | SplitPolygonEdges | 1385 / 2640 / 1369 | 1181.340 / 18.922 | 56.627 / 6.648 | 0.125 / 0.093 | 241.156 / 18.308 |
| Concave | RefineOnCurves | 1385 / 2640 / 1369 | 1841.850 / 929.303 | 57.514 / 6.948 | 2.552 / 0.196 | 236.984 / 18.459 |
| Circle | Fixed | 1450 / 2770 / 1322 | 1186.790 / 23.308 | 63.264 / 6.201 | 0.593 / 0.169 | 260.822 / 19.355 |
| Circle | SplitPolygonEdges | 1489 / 2848 / 1457 | 1291.270 / 20.901 | 67.806 / 6.671 | 0.186 / 0.122 | 273.802 / 18.757 |
| Circle | RefineOnCurves | 1503 / 2876 / 1471 | 2411.050 / 1740.030 | 72.039 / 7.071 | 0.578 / 0.173 | 290.564 / 19.220 |
| Tagged rectangle | Fixed | 844 / 1590 / 748 | 378.129 / 13.959 | 19.137 / 3.577 | 2.083 / 0.159 | 82.612 / 10.065 |
| Tagged rectangle | SplitPolygonEdges | 913 / 1728 / 899 | 477.934 / 11.563 | 22.809 / 4.364 | 0.105 / 0.076 | 92.284 / 12.595 |
| Tagged rectangle | RefineOnCurves | 913 / 1728 / 899 | 811.900 / 529.639 | 22.222 / 4.405 | 2.046 / 0.160 | 97.278 / 12.467 |

Process peak RSS before / after: 6248 / 8104 KiB. These
process-wide maxima include all cases and cannot attribute memory to a phase.

On the final tagged curved case (913 nodes, 1728 triangles, 2640 unique edges),
the previous validation loop enumerated 3,483,480 edge pairs. The AABB index
returns 14,027 distinct inclusive-overlap candidate pairs (see candidates.csv);
the original exact predicates then decide these pairs. This counter excludes
self-pairs and counts each unordered pair once.

Verification: full tools/test.sh PASS; focused optimized regression PASS;
ASan+UBSan focused regression and existing discretization suite PASS with leak
detection enabled; TSan focused regression PASS (runtime available). TSan here
executes serial topology/validation and is not evidence for concurrent mutation
support. Standalone include compilation PASS without Eigen, SymEngine or
OpenMP. git diff --check PASS.
