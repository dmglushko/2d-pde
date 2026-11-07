# Performance and verification

Start with the [post-review wave comparison](post-review-waves-2026-11-07.md),
its [interactive phase chart](post-review-waves-2026-11-07.html), and
[recorded runs and provenance](post-review-waves-2026-11-07.json).

The [native report API](../fem/README.md#native-profiling-and-report-output)
collects timings on every solve. The [wave guide](../fem/long-waves.md) explains
the one-option benchmark. Additional standalone experiments are documented in
[benchmarks/](../../benchmarks/README.md).

Integration advances displacement, velocity, and acceleration by solving the
discrete PDE. Diagnostics inspect those states: energy, supplied work, mean,
and errors against an exact solution. Exact-solution error integration can cost
more than a cached PDE time step. Diagnostics do not advance the PDE state.

The evolution wall-clock total includes synchronous diagnostic callbacks.
`observer_seconds` isolates callback cost; the wave profile's
`integration_excluding_observer_seconds` reports numerical stepping without
those callbacks. Solver subphase durations are nested inside solver time, so
they must not be added to the parent total again.

## Historical evidence

These reports describe their recorded snapshots, not the current API or new
performance promises. Raw measurements are retained to make comparisons
auditable.

- [November 6 audit and issue inventory](audit-2026-11-06.md)
- [Integrated implementation and verification](integration-2026-11-06.md)
- [Assembly preparation](assembly-2026-11-06.md)
- [Geometry query performance](geometry-issue48-2026-11-06.md)
- [Mesh refinement](mesh-issue3/README.md)
- [Solver reuse](solver6/README.md)
- [Diagnostics and viewers](issue-7-diagnostics-viewers.md)
- [Independent references and build verification](verification-build-2026-11-06.md)
- [Repeated historical wave measurements](waves-benchmark.html)
