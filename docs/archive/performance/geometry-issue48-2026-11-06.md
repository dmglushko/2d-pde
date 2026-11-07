# Geometry query safety and preparation (#4, #8)

Base: `cb9c3e34c1bb7c1aaba7abbe7664afb9b812288a`. No mesh broadphase
owner headers, formats, build files or existing tests were changed.

## Concurrency and value semantics

Const queries, including simultaneous first length/region-length queries, and
copying a read-only source are supported. Each geometry owns a mutex and length
map. Cache reads, insertion and source-cache copying lock that mutex; an
integration exception releases the lock without inserting a result. Different
copies never alias a mutable cache. Move operations require exclusive source
access, as do all mutations and assignments to the destination. No topology or
geometry mutation is supported alongside queries on that same object.

All edits may invalidate references returned by `curves()` and `curve()`.
The mutex protects measurements, not references or mutation. Independent copies
may be edited while readers query the original. Local-space lengths survive
rename/rigid transforms; replacement and resize clear them, and splits retire
old measurements and install measured fragments. Transactional edits build all
indexes on the temporary before swapping; failed allocation/replay leaves the
original geometry intact. IDs/history and replay-generated fragments remain
geometry-local values.

ID, region and positional-run indexes are eager, owned values, rebuilt during edits. Region
lookup touches only its membership; `regionGroups()` returns a copy in original
first-occurrence order. Seam-spanning pieces start at the first name transition,
with the last-to-first piece joined in the original traversal order.

## Numerical and ordering safeguards

Length inversion retains direct measurement of the smaller portion, the existing
endpoint-relative residual/roundoff criterion and an enclosing parameter bracket.
Newton proposals must be finite and strictly inside the bracket. Every fourth
initial attempt bisects; after 16 attempts, up to 128 bisections retain the
original fallback budget. Zero derivatives therefore never authorize an unsafe
step. Integrals are not cached against publicly mutable curve fields.

Sampling computes each original source interval's bounds and endpoints once,
then carries those data alongside provenance. Rejected removals do not copy or
concatenate provenance vectors. Accepted removals retain the original scan,
erase and restart order; this global simplification fallback deliberately remains
to preserve exact greedy decisions. Interval sums retain their original addition
order. No merged-interval approximation replaces the conservative bound.

SVG uses the existing `mesh_detail::BoxIndex` for original endpoints, preserving
the first path anchor and testing every candidate with the unchanged `near()`.
Query boxes use the global endpoint norm (both operands of the relative tolerance)
and outward rounding. Both endpoints of one path remain separate candidates, so
a double match still throws. Consumed paths are ignored; remaining paths are
reversed only after ambiguity checks. Nonfinite computed endpoint norms use the
old exhaustive matching fallback. An overflowing-arc rejection test exercises
this path. Globally disparate coordinate scales may degrade candidate pruning;
no tolerance or ambiguity check is weakened.

## Reproduction and actual validation

Private artifacts: `/tmp/pde-geometry-48/`. Baseline binary `baseline` was built
and executed before header edits; its first result is `baseline-1.txt`. It remains
available with the subsequent repeated results and final `current` binary.
Compiler: g++ (Ubuntu 15.2.0-16ubuntu1) 15.2.0; CPU: AMD Ryzen 9 6900HX,
16 logical CPUs. C++23, `-O2 -Wall -Wextra -Werror`, no fast-math; single-threaded
benchmark, no OpenMP. Build artifacts use the private directory above.
Other implementation work was active on the machine, so wall-clock differences
are not evidence of speedup. These are descriptive process-CPU measurements;
small sampling differences are inconclusive and copy overhead increases.

Benchmark command (run three times per binary):

```sh
g++ -std=c++23 -O2 -Wall -Wextra -Werror -Icpp \
  $(pkg-config --cflags libxml-2.0) benchmarks/geometry_queries.cpp \
  $(pkg-config --libs libxml-2.0) -o /tmp/pde-geometry-48/current
/tmp/pde-geometry-48/current /tmp/pde-geometry-48/input.svg
```

Process CPU seconds, all three runs, same source/settings:

| Case | Baseline | Final |
| --- | --- | --- |
| Queries | 9.26861 / 9.48345 / 9.32231 | .008021 / .006303 / .010595 |
| Copy | .009165 / .009264 / .009180 | .012183 / .011535 / .015831 |
| Inversion | .069847 / .072820 / .078533 | .040419 / .044288 / .040301 |
| Sampling | .005546 / .005550 / .006153 | .007034 / .005780 / .006577 |
| SVG | .323144 / .325690 / .345817 | .023725 / .025383 / .029717 |

Work: 1,800 curves / 90 interleaved names, six passes of curve and region length
queries (10,800 of each); 40 warm-cache copies; 160 cubic/ellipse inversions;
20 ellipse samplings with edge length .04 and deviation .0003; five imports of
1,800 permuted, alternately reversed paths. Checksum `20112.5` matches every run.
Timings include allocation and normal validation; they are not timing assertions.

Executed validation:

- `bash tools/test.sh`: full existing suite passed, including SVG and 19 showcase
  meshes loading in the viewer.
- New tests compiled with `-std=c++23 -O2 -Wall -Wextra -Werror -Icpp`:
  `geometry_query_threads.cpp` (also `-pthread`),
  `geometry_sampling_regression.cpp`, `geometry_inversion_guard.cpp`,
  `geometry_svg_index.cpp` (libxml flags as above, private directory argument).
- Concurrency harness: eight workers across eight fresh geometries, cold ellipse
  integration and line-region misses, repeated hits, copy construction and
  assignment alongside readers, independent edits, move, retired IDs, seam
  ordering, resize replay and rollback. Passed.
- Sampling signatures recorded using saved baseline headers and binary, then
  compared byte-for-byte against final results: ellipse and multi-segment square,
  corners/region preservation on/off, tags, points and all source intervals.
- New inversion grid: cusp, stationary endpoint, cubic and ellipse, scales
  `1e-100`, `1`, `1e100`, fractions `1e-14` through `1-1e-14`; checks smaller
  portion residuals and monotone parameters. Public field edits change lengths.
- ASan/UBSan (`-O1 -g -fsanitize=address,undefined
  -fno-omit-frame-pointer`): concurrency, sampling, inversion grid, existing
  length tests, existing SVG tests and focused SVG tests passed.
- TSan (`-O1 -g -fsanitize=thread -pthread`): concurrency harness executed
  successfully, exit 0, no reports. This was an available runtime, not a skip.

For example, sanitizer invocations use the same includes/link flags and run the
result directly; the SVG test takes `/tmp/pde-geometry-48` as its argument.
Logs are `full-tests-final.log`, `tsan-final.log`, and `asan-*.log` in the private
artifact directory. New tests are intentionally unregistered; the build owner
will register them later.
