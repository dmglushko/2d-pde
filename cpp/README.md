# `cpp/` — core headers

Headers at the top of `cpp/` are shared by both modules and depend on
nothing but the standard library (and OpenMP when enabled). The modules
have their own maps: [discretization](discretization/README.md) and
[fem](fem/README.md). `tests/tools/readme_headers.cmake` fails when a
top-level header is missing from this table.

| Header | Role | Main entry points |
| --- | --- | --- |
| `execution.hpp` | Execution policy and the deterministic parallel loop | `ExecutionOptions`, `ExecutionBackend`, `parallel_for`, `openmp_available` |
| `json.hpp` | The JSON document of every record: value tree, deterministic writer (classic locale, `max_digits10`), strict parser, leaf-by-leaf diff | `JsonValue`, `json_number`, `json_string`, `json_bool`, `json_text`, `write_json`, `parse_json`, `diff_records`, `RecordDifference`, `RecordDiffOptions` |
| `provenance.hpp` | Reproducibility building blocks | `Fingerprint` (portable FNV-1a 64), `LibraryVersion`, `library_version` |

`library_version()` reads `pde2d_version.hpp`, which CMake generates at
configure time from `cmake/pde2d_version.hpp.in` (project version and git
commit; CMake re-runs after a commit or checkout). Without it (header-only
use without CMake) the version is `"unknown"` and the SHA empty.

`diff_records(left, right, options)` flattens two documents to dotted leaf
paths (`options.solver.method`, `entries[1].errors.l2`) and returns the
leaves that differ, in the order of `left`, then the leaves only `right`
has (`"<absent>"` on the missing side). By default it skips every
`timings` object (wall clock, the only part that differs between identical
runs) and every `outcome` object (iterations and residuals of a solve:
reproducible, but consequences of the inputs); `RecordDiffOptions` turns
either back on. The modules overload it for their records
(`discretization::diff_records(MeshRunRecord, ...)`,
`fem::diff_records(RunRecord, ...)`).
