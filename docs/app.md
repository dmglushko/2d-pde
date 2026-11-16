# The app

How the graphical interface in `app/` is built, as it is now. How to use
it is in [app/README.md](../app/README.md); why it is like this, decision
by decision, is in [app-decisions.md](app-decisions.md).

**The app's design is delegated** (the owner, 2026-11-14). Agents make
the app's decisions themselves and record them in
[app-decisions.md](app-decisions.md). The library's rules still hold: the
app must not change the library's design, and anything the app needs from
the library is a library change, raised with the owner
([library-requests.md](library-requests.md)).

## Shape

```mermaid
flowchart LR
    subgraph Browser
        PAGE["app/web: HTML + JS modules"]
    end
    subgraph pde2d_app
        SERVER["server.cpp · cpp-httplib"]
        SESSION["Session (facade)"]
        CACHE["StageCache"]
        RUNS["RunManager: runs per client"]
        EVAL["field_eval: comparisons, probes"]
        EXPORTS["exports: table of kinds"]
        INPUT["catalog / domain_input / problem_input"]
    end
    LIB["pde2d::pde2d (geometry, mesh, io, fem, solve)"]

    PAGE -- "JSON and binary over HTTP" --> SERVER
    SERVER --> SESSION
    SESSION --> CACHE
    SESSION --> RUNS
    SESSION --> EVAL
    SESSION --> EXPORTS
    CACHE --> INPUT
    INPUT --> LIB
    CACHE --> LIB
    RUNS --> LIB
    EVAL --> LIB
```

- **One program, `pde2d_app`**, serves the page and an API (on 127.0.0.1
  by default), and runs a saved case without the browser (`--run`).
- **The page holds no physics.** It edits a *case* (JSON), sends it to the
  server and draws what comes back. Every check is the library's own: the
  app forwards the library's messages with the path of the field they
  concern. Numbers the user reads as results (errors against an exact
  solution, u(t) at probes) are computed on the server in double.
- **The page is generated from a catalog.** The server describes every
  component and its options (`GET /api/catalog`); the forms are built from
  that, so a new library component needs one catalog entry in C++ and no
  change to the page.
- **The library stays as it is.** The app uses only public headers,
  through the umbrella target `pde2d::pde2d`, which links every module.

## Files

| Path | Contents |
| --- | --- |
| `app/src/options.*` | `OptionSpec` (an option as the page sees it), `OptionReader` (reads a JSON object strictly against the specs), small JSON readers, `named_option` (the option a library message names), `request_job` / `request_count`, `error_json` |
| `app/src/app_error.hpp` | `PathError` (input refused at a path), `StageError` (a stage failed: stage, kind, message, path), `staged`, `at_path` |
| `app/src/catalog.*` | the mesh and solve catalogs: every component, its options with the library's defaults, a function that builds it; `build_mesh_setup`, `build_solve_setup` |
| `app/src/domain_input.*` | the `geometry` section: shapes, SVG, mesh files, region edits; the geometry view; `DomainFacts` |
| `app/src/problem_input.*` | the `problem` section: `build_problem`, `check_problem` (every error at once, with paths), the problem catalog and templates |
| `app/src/presets.*` | problem presets: a complete problem and its solve settings for any geometry, scaled by `DomainFacts` |
| `app/src/stage_cache.*` | `StageCache`: the last domain, mesh (with its id), `MeshedProblem` and assembly, keyed by the case sections; `assembly_settings`, `solve_key` |
| `app/src/runs.*` | `SessionOptions`, `RunState`, `Run` (a solve: static, or a time run in chunks), `make_run`, `advance`, `progress_json`, `RunManager` (each client's run and reference run, bounded) |
| `app/src/field_eval.*` | expressions at the nodes, `lumped_weights`, `difference_stats`, `Comparison` (against an exact solution or the reference run), `PointLocator` (a grid over the mesh), `probe_series` |
| `app/src/exports.*` | `ExportedFile` (content or a stream), `ExportKind`, `ExportNeeds`, the table of export kinds with their writers, provenance |
| `app/src/case_set.*` | `--set PATH=VALUE`: `set_case_value`, `apply_set_argument` |
| `app/src/views.*` | reports, mesh stats, sparsity as JSON; the mesh summary and its binary arrays |
| `app/src/binary.hpp` | little-endian float32, float64 and uint32 |
| `app/src/session.*` | `Session`: the facade the server and `--run` use (stages, runs, comparisons, probes, exports, `run`), the default case, `check_case` |
| `app/src/server.*`, `app/src/main.cpp` | HTTP (the only file that includes `httplib.h`: routes, client ids, token, Origin, the queue bound) and the command line |
| `app/web/` | `index.html`, `style.css` (one dark theme, every colour a token), `js/`: `pure/` (no DOM, tested in Node: `format.js` numbers, decimal parsing and ticks; `color.js` OKLab ramps, field scales, the locked range; `mesh_math.js` angles, point location, interpolation, isolines, \|∇u\| per triangle, line sampling, the binary mesh; `case_model.js` normalization, region edits, visible coefficients; `stages.js` the stage registry, keys, error paths; `run_values.js` the snapshot window; `prefs.js` settings, folds, probe slots, the picture's height; `tickets.js` request tickets; `history.js` undo steps), `api.js` (every call, the client id and token headers), `state.js` (the store, `editCase`, tickets, undo, invalid entries), `actions.js` (stages, runs, presets, exports), `snapshots.js`, `settings.js`, `dom.js` (forms from specs, `numberInput`, error marks, warning chips, info toggles, foldable sections), `view/` (`viewer.js` the picture's state and layer order, `layers.js` the drawing, `legend.js` the legend, its range lock and the saved picture, `interaction.js` pointer, pinch, wheel, keys, the readout, `tokens.js`), `timebar.js`, `probes.js` (probes, u(t), the line profile), `charts.js` (the SVG line chart), `compare.js` (comparisons with an exact solution and the reference run), `panels/*.js` (one per step), `main.js` |
| `app/examples/` | example cases (each runs as `app_run_*`) and an example SVG |
| `app/tests/` | `browser/` (the page in headless Chrome: `run.sh`, `harness.html`, `scenarios/*.js`), `session.cpp`, `solve.cpp` (the Solve step, with exact solutions), `runs.cpp` (runs in chunks, every preset), `compare.cpp` (comparisons in double, reference runs, probes, clients, the binary mesh), `exports.cpp` (exports never compute, provenance, VTK, error paths, `--set`), `server.cpp` (HTTP, token, Origin, 429, clients), `app_support.hpp` (the mesh decoded as the page decodes it), `web.test.mjs`, `web_actions.test.mjs` |
| `app/third_party/cpp-httplib/` | `httplib.h` 0.18.3 and its MIT license |

## The case

A case is the whole setup in one JSON object. Every section is read
strictly: a key the reader does not know is an error (a misspelt option
must not be ignored silently), and a missing value means its default.

```json
{
  "format": "pde2d-app-case",
  "version": 1,
  "notes": "free text",
  "geometry": {"source": "shape", "shape": "rectangle",
               "options": {"width": 2, "height": 1}, "region_edits": []},
  "mesh": {
    "spec": {"max_edge_length": {"absolute": null, "relative": 0.05}},
    "discretizer": {"type": "chord_length", "options": {}},
    "triangulator": {"type": "cdt", "options": {}},
    "refiner": {"type": "ruppert", "options": {"min_angle_degrees": 25}},
    "improvers": [{"type": "delaunay_flips", "options": {"use": "final_only"}}]
  },
  "problem": {
    "equation": {"a_xx": "-1", "a_yy": "-1", "f": "1"},
    "parameters": {"k": 0.1},
    "boundary_conditions": {"left": {"alpha": "1", "g": "0", "priority": 0}},
    "initial": {"u": "0"}
  },
  "assembly": {"t": 0, "max_time_order": 2},
  "solve": {
    "mode": "time",
    "linear_solver": {"type": "sparse_lu", "options": {}},
    "static": {"t": 0},
    "time": {"t0": 0, "t1": 1, "snapshots": 50,
             "method": {"type": "implicit_euler", "options": {}},
             "step": {"type": "fixed_step", "options": {"dt": 0.01}}}
  },
  "view": {"exact_solution": "x"}
}
```

- `geometry.source` is `shape` (`shape`, `options`), `svg` (`svg`: the
  document's text, `name`) or `mesh_file` (`format`: `gmsh` or `pde2d`,
  `text`, `name`). SVG's y axis points down; the library flips it, so the
  SVG point (x, y) is (x, −y) in the case. Shapes and SVGs take
  `region_edits`: `{"op": "rename", "from", "to"}`, `{"op": "merge",
  "source", "target"}`, `{"op": "split", "region", "fraction", "first",
  "second"}`, applied in order with `CurveGeometry`'s methods. A mesh file
  takes `rename` and `merge` (not `split`: it has no curves); the app
  applies them to the imported `mesh::Mesh` (a renamed region keeps its
  tag; a merge moves the source's boundary edges to the target's tag and
  drops the source's name, renumbering the tags above it) and checks the
  result with `mesh::validate`.
- A mesh file is not meshed again: the `Remesher` runs the refiner and
  the improvers on it (the discretizer and triangulator are not used).
- `mesh.refiner` may be `null` (no refiner). A length bound is
  `{"absolute": number or null, "relative": number or null}`.
- Problem coefficients are expression texts; an empty or missing one is
  not set. `priority` is a whole number.
- `assembly.max_time_order` (0, 1 or 2) is `Assembler::assemble`'s: which
  time terms are built.
- `solve.mode` is `static` (`StaticSolver` at `static.t`) or `time`
  (`Integrator` from `t0` to `t1` with `method` and `step`). The linear
  solver serves both. Only the chosen mode's part is read into the
  library's objects; the other is kept as written. A time run keeps the
  solution at `snapshots` + 1 equal times from t0 to t1 (at most 1000
  snapshots). A case written before the solvers has `"solve": null`, which
  means the default section (a static solve).
- `view` and `notes` are free for the page and for people. The page keeps
  `view.exact_solution` (the comparison's expression; `--run` compares
  with it too), `view.region_edit_undo` (one undo record per region edit)
  and `view.condition_archive` (conditions of regions that are gone)
  there.
- `version` is checked: a whole number from 1 to the app's version; a
  case from a newer app is refused, not misread.

## Clients

Every page instance makes a random client id when it loads (`api.js`,
`clientId`) and sends it on every request in `X-PDE2D-Client` (letters,
digits, `-`, `_`, at most 64). Runs, reference runs and their progress
belong to the client: two tabs do not replace each other's run, and one
tab cannot read or stop another's. A request without the header is the
client `local` (the command line, tests, `curl`). A reload is a new
client (it has no run to follow).

The session keeps at most `SessionOptions::max_clients` (8) clients; a
solve by a new client drops the one used least recently, with its runs
(a running one stops). Only a solve makes a client; asking for an unknown
client's run is an error, not a new client.

The stage cache (domain, mesh, problem, assembly) is shared by all
clients: it is keyed by the case's text, so a client gets the same answer
whoever built it; two clients that alternate between different meshes
rebuild them in turn.

## The API

All on `127.0.0.1:8765` by default. Every POST body is JSON and must be
sent as `application/json`. Stage calls take a case (the page sends only
the sections the stage reads: `requestCase`); run calls take `{"job",
...}`.

| Call | Body | Gives |
| --- | --- | --- |
| `GET /api/catalog` | | the catalog (below) and the default case |
| `GET /api/default-case` | | the case a new session starts with |
| `GET /api/examples`, `GET /api/examples/<name>.json` | | the example cases (name and notes), and each case |
| `POST /api/geometry` | case | loops (each curve sampled for drawing, with its region and kind), regions and lengths, bounding box, domain size; for a mesh file its boundary edges |
| `POST /api/mesh` | case | `mesh`: the mesh's summary (`id`, counts `nodes`, `triangles`, `boundary_edges`, `region_names`, `has_links`, and `arrays`: where its arrays are, see "Binary answers"); `stats`, `target_met`, `longest_edge`, `max_edge_length`, `domain_size`, every stage's `reports` |
| `POST /api/mesh/arrays` | `{"mesh": id}` | the mesh's arrays in binary. The session keeps the last 16 meshes it built and the meshes of every kept run |
| `POST /api/problem` | case | the check: `ok`, `errors` (`path`, `message`), `facts` per path (none for a text that did not parse), each condition's kind, time order, needed initial conditions |
| `POST /api/assemble` | case | the `MeshedProblem` report and Dirichlet nodes; the assembled operators' facts and sizes, a sparsity picture, the assembly report |
| `POST /api/field` | `{"case", "field"}` | values at the nodes (`null` where there is none) of `{"kind": "expression", "text"}`, `initial_u`, `initial_u_t`, `dirichlet` or `right_hand_side`, at `t` (default: `assembly.t`) |
| `POST /api/solve` | case | starts the client's solve of the case, or finds its current one with the same inputs (see "Runs"), and answers its progress (below) |
| `POST /api/solve/progress` | `{"job"}` | the same progress, now |
| `POST /api/solve/stop` | `{"job"}` | stops the run after the chunk that is running; its progress. A solve of the same inputs continues it |
| `POST /api/solve/values` | `{"job", "first", "count"}` | snapshots `first` to `first + count` (ready ones only) in binary |
| `POST /api/solve/reference` | `{"job"}` | keeps the client's run (it must be done) as its reference run: `{"reference": {"job", "mode", "times", "mesh_id"}}`. It stays when the run is replaced |
| `POST /api/solve/reference/clear` | `{}` | forgets the reference: `{"reference": null}` |
| `POST /api/solve/compare` | `{"job", "exact": text}` or `{"job", "reference": true}` | the comparison in double (see "Comparisons") |
| `POST /api/solve/compare/difference` | the same and `"snapshot": k` | u − other at snapshot k in binary (float32) |
| `POST /api/solve/probes` | `{"job", "points": [[x, y], ...]}` | u(t) at each point (at most 64) over the run's ready snapshots (see "Probes") |
| `POST /api/preset` | `{"case", "preset"}` | the case with its `problem` and `solve` sections replaced by the preset for the case's geometry (see "Presets") |
| `POST /api/export/<kind>` | case | a file (see "Exports"); `X-PDE2D-Case-Name` (URL-encoded) names the case in its provenance |

**The progress of a run** (`/api/solve`, `/progress`, `/stop`): `job`
(the run's id), `state` (`running`, `done`, `failed`, `stopped`), `mode`
(`static` or `time`), `mesh_id`, `times` of the snapshots ready, `min` and
`max` over their values (exact doubles, `null` before any value),
`time_order` (assembly's, `Assembler::time_order`), `problem_time_order`
(as written), `free_unknowns`, `reports` (`static_solver`'s, or one
`integrator` report combining the chunks', with `chunks`), and `values`:
`{"path": "/api/solve/values", "format": "float32le", "snapshots"
(ready), "nodes", "bytes"}`. A time run adds `t0`, `t1`, `t` (reached) and
`snapshots_total`; a failed one `error`. A static solve is `done` at once.

**Errors** are `{"error": {"stage", "kind", "message", "path"}}`.
`message` is readable on its own; when the error concerns a field it
starts with that field's path (`mesh.refiner.options.min_angle_degrees:
...`), and `path` holds the same path as a field (`null` when the error
names none, as most of the library's own algorithm messages). The page
marks the field `path` names (or its nearest ancestor that has a form
element); only an error without `path` falls back to the path at the
start of its text. Where the paths come from:

- The JSON readers (`require_*`, `OptionReader`) throw `PathError` at the
  value they read.
- A component's constructor (mesh and solve components) and a shape's
  function refuse their options with the library's message; when it names
  one of the options as a word (`RuppertOptions::min_angle_degrees must
  be ...`, `fixed_step: dt must be ...`, `Rectangle width must be ...`),
  the path is that option's (`named_option`), else the options object's.
- An SVG or mesh text the library refuses is at `geometry.svg` or
  `geometry.text`; a region edit at `geometry.region_edits[i]` (or its
  `.op`); a coefficient at `problem.equation.a_xx` and so on; the mesh
  spec at `mesh.spec.max_edge_length...`; `--set` at the path it was
  given; a comparison's expression at `exact`; a probe point at
  `points[i]`.

| Status | When | `kind` |
| --- | --- | --- |
| 400 | the body is not JSON | `input` |
| 403 | the Host is not this machine (loopback server), the token is missing or wrong, or the Origin is another site | `input` |
| 409 | a result that is not ready: a solution export before the client's run of the case is done, a reference that is not done | `not_ready` |
| 413 | a body over 64 MiB | |
| 415 | a POST that is not `application/json` | `input` |
| 422 | the library refused the input (`std::invalid_argument`) or an algorithm could not finish (`std::runtime_error`) | `input`, `algorithm` |
| 429 | more than `ServerOptions::max_waiting` (6) API requests are already being served or waiting for the session | `busy` |
| 500 | anything else | `internal` |

## Binary answers

All little-endian, as `application/octet-stream`.

- **Solution values** (`/api/solve/values`): float32, snapshot after
  snapshot, one value per mesh node (in node order).
- **Differences** (`/api/solve/compare/difference`): float32, one per
  node.
- **Mesh arrays** (`/api/mesh/arrays`): the parts listed in the summary's
  `arrays.parts` (`name`, `type`, `count`, `offset` in bytes), one after
  the other: `nodes` (float64 x0, y0, x1, y1, ...), `triangles` (uint32
  a0, b0, c0, ...; counterclockwise), `boundary_edges` (uint32 a0, b0, a1,
  b1, ...; the domain on the left) and `boundary_regions` (uint32, the
  region tag of each boundary edge, an index into `region_names`).
  `arrays.bytes` is the total. The page decodes them into a
  `Float64Array` of nodes, a `Uint32Array` of triangles and a
  `Uint32Array` of boundary edges as `[a, b, region, ...]` (`pure/mesh_math.js`,
  `decodeMesh`).

## Comparisons

`POST /api/solve/compare` compares the run `job` with an exact solution
(`"exact"`: an expression in x, y, t and the run's parameters, compiled
by the library's `CompiledExpression` under the session mutex and
evaluated at every node at each snapshot's time) or with the client's
reference run (`"reference": true`). The answer, all in double:

```json
{"job": 3, "against": {"exact": "x + t"}, "nodes": 564,
 "snapshots": [{"t": 0, "max_abs": 1e-16, "rms": 4e-17, "l2": 3e-17}, ...],
 "max_abs": 2e-15, "max_rms": 9e-16, "max_l2": 6e-16}
```

- `max_abs` is max over nodes of |u − other|; `rms` is the nodal root
  mean square sqrt(Σ d²/N); `l2` is the lumped L2 norm sqrt(Σ w_i d_i²)
  with w_i a third of the area of the triangles around node i (the L2
  norm of the P1 function d with nodal quadrature). A node where a value
  is not finite makes the snapshot's figures `null`. `max_*` are the
  largest over the snapshots (`null` when none could be compared).
- **A reference run** must be on the same mesh object as the run (node i
  is the same point in both): solving after a change of the geometry or
  mesh sections gives another mesh, and the comparison is refused with a
  message saying so. Each run snapshot is compared with the reference's
  snapshot at the same time (up to rounding); a static reference (one
  snapshot) stands for every time; a snapshot without a match is `null`.
- The figures cover the snapshots ready when the call is made.

## Probes

`POST /api/solve/probes` gives, for each point, `x`, `y`, `inside`,
`triangle` and `values` (u at the point at every ready snapshot, in
double, `null` where not finite; empty outside the mesh), and `times`.
The point's triangle is found with a uniform grid over the mesh's
bounding box (about one triangle per cell, at most 1024 cells a side;
each cell lists the triangles whose boxes overlap it) and barycentric
coordinates in double; on an edge the triangle with the largest smallest
weight wins, and a point outside every triangle by more than 1e−10 in
barycentric terms is outside. u is the P1 interpolant there.

## Exports

`POST /api/export/<kind>` with the case. One table (`exports.cpp`,
`export_kinds`) lists each kind with its file name, content type, what it
needs and its writer:

| Kind | Needs | Contents |
| --- | --- | --- |
| `mesh.pde2d` | the mesh | the old `.pde2d` format, exactly (no provenance: the format has no place for it) |
| `mesh.svg` | the mesh | an SVG picture of the mesh |
| `geometry.svg` | the domain | the curve geometry as SVG (refused for an imported mesh) |
| `stiffness.mtx`, `damping.mtx`, `mass.mtx` | the assembly at `assembly.t` | the free part of A, B or M2 in Matrix Market `coordinate real general`, 1-based; a zero operator has no entries |
| `rhs.mtx` | the assembly | b in Matrix Market `array` form |
| `solution.csv` | a done run | x, y and u at every kept time (`u(t=...)` columns), one row per node, full doubles |
| `solution.vtk` | a done run | VTK legacy ASCII `UNSTRUCTURED_GRID`: the nodes (z = 0), the triangles (`VTK_TRIANGLE`), and one `POINT_DATA` scalar array per kept snapshot, named `u(t=...)`; ParaView and VisIt open it |
| `provenance.json` | the case | `{"format": "pde2d-app-provenance", "version": 1, "app", "library", "case_name", "exported", "file", "case" (the full case as sent), "run"}`: the run is the client's done run of the case with its progress (reports included, without the values layout) and `started`, or `null` |

- **Solution exports never compute.** They take the client's current run
  when its inputs (the geometry, mesh, problem and solve sections, as
  text) are the case's and it is done; with no such run, or one that is
  running, stopped or failed, they answer 409 with a message. Mesh and
  system exports use the stage cache (built when missing).
- **Large files stream.** `solution.csv` and `solution.vtk` are written
  piece by piece as they are sent (`ExportedFile::stream`, an httplib
  chunked content provider). The stream holds only the mesh and the
  values (no expressions), so it runs after the session mutex is
  released; a done run's values do not change.
- **Provenance.** Every file but `.pde2d` says what produced it: the app
  and library versions (both the CMake project version, `PDE2D_VERSION`),
  the case's file name when known, the export time (UTC), the run (id,
  state, start time) for solution exports, and `settings`: the case
  sections the kind is built from (geometry with an SVG or mesh text
  replaced by its length; plus mesh; plus problem and assembly for the
  system; plus solve for a solution) as one line of JSON. As comments:
  `%` lines after the `%%MatrixMarket` banner, `#` lines at the top of the
  CSV (read it with `pandas.read_csv(path, comment="#")`, `numpy.loadtxt`
  (`#` is its default comment), R's `read.csv(comment.char = "#")`;
  spreadsheets show them as rows), an XML comment after the SVG's
  declaration. VTK legacy has no comments: its title line (at most 255
  characters) names the app, the library, the run, the time and the
  case; `provenance.json` has everything.

## The catalog

`GET /api/catalog` returns:

- `sections`: one per pipeline the page builds forms for. The mesh
  section has `settings` (the `MeshSpec` fields) and `slots`; a slot has
  an `id` (its key in the case), a `cardinality` (`one`, `optional` or
  `list`) and its `components`, each with `type`, `label`, `description`
  and `options`. An option has `name`, `label`, `description`, `kind`
  (`number`, `integer`, `choice`, `length_bound`), `default`, and
  `choices` or `unit` where they apply. The solve section has the
  `linear_solver` slot and `modes` (`static`, `time`), each with its
  `settings` and `slots` (`method` and `step` for `time`). Components of
  the solve section carry `traits`, read from the library's declarations
  of a default-built instance: a linear solver's `symmetric_matrix`
  requirement, a time method's supported `time_orders`.
- `shapes`: the ready-made shapes with their options.
- `problem`: the coefficients of the equation, of a condition and of the
  initial conditions in order, with the term each multiplies; help text;
  templates of common equations and conditions; `presets` (`id`,
  `label`, `description`).
- `format`, `version`, `library_version`, `default_case`.

## Adding a component

1. Add one entry to `make_mesh_catalog()` or `make_solve_catalog()` in
   `app/src/catalog.cpp`: its `type` (the component's `name()`), label,
   description, its options with defaults read from the library's options
   struct, and `make`, which builds the component from an `OptionReader`.
2. That is all for the page. `tests/session.cpp` and `tests/solve.cpp`
   check that every type equals `name()`; add its defaults to the
   defaults test.

Ranges are never checked in the app: the component's constructor
validates its options, and its message is shown at the field.

## The stage cache

`StageCache` keeps the last result of each stage with the key it was
built from: the domain (the `geometry` section's text), the mesh (plus
`mesh`), the `MeshedProblem` (plus `problem`) and the assembly (plus
`assembly.t` and `max_time_order`). Changing the problem does not remesh;
an assembly at another t of the same `MeshedProblem` passes the previous
one so the library reuses its constant parts. Each mesh it builds gets an
id (`MeshOutcome::id`); the last 16 stay findable by id for
`/api/mesh/arrays`.

## Runs

`Session::solve` makes a **run** (`Run`) for the client, keyed by
`solve_key` (the geometry, mesh, problem and solve sections' text): a
static solve is done when it is made; a time integration runs in chunks
that end on snapshot times. Each chunk is one `Integrator::integrate(meshed,
ta, tb, output, &last)` from the state the previous chunk ended with
(`IntegrationResult::final`, with its velocities for time order 2); the
library guarantees that a run that lands on a time and one that starts
there take the same steps.

- `solve` runs the first chunk (one snapshot interval) before it answers,
  so the library's refusals (a time order the method does not support, a
  solver's requirement) are thrown as before and land on their field.
  With `SessionOptions::background_runs` (the server) one thread then
  runs the next chunks of every running run, taking the clients in turn;
  without (the command line, tests) `solve` runs them all before it
  returns. Both use `advance`.
- Each chunk holds the session mutex only while it integrates; between
  chunks the runner releases it for a moment, so other requests are
  answered after at most one chunk.
- The next chunk's number of snapshot intervals is scaled so that it
  takes about `chunk_seconds` (0.4 s): between half and four times the
  last one, at least one interval.
- The combined `integrator` report adds up `steps`, `rejected_steps`,
  `linear_iterations`, `assemblies`, `prepares`, `snapshots`, takes the
  largest `max_linear_iterations`, `max_residual`, `max_dt`, the smallest
  `min_dt`, the first `t0` and the last `t1`, and adds `chunks`.
  `prepares` counts one more per chunk (each chunk prepares the linear
  solver once).
- A solve of the same inputs reuses the client's run: a running or done
  one is answered as it is, a stopped one continues, a failed one starts
  again. Other inputs replace it (the old one is dropped under the mutex:
  it holds expressions). The reference run is a second pointer that keeps
  a done run alive.
- **Exactness.** For implicit Euler and the implicit midpoint rule the
  state is U (and U' for time order 2), which `final` carries, so the
  snapshots of a chunked run are bit for bit those of one call
  (`tests/runs.cpp`). Newmark's state also has the acceleration, which
  `Solution` does not carry: each chunk's start computes it again from
  the equation, so the snapshots differ from one call by rounding (below
  1e-8 of the largest value in the test).
- What a chunk cannot do: show progress or stop inside one snapshot
  interval (library-requests.md, item 2).

## The page

**Following a run** (`actions.js`, `followRun`): the page polls the
progress every 250 ms, shows the newest snapshot unless the time bar was
moved during the run (or the setting is off), and colours with the range
so far (the summary's exact `min` and `max`, one scale for every
snapshot). **Stop** keeps what arrived; **Continue** solves again, which
continues the run on the server. A change of the case that changes the
solve's inputs stops the run.

**Snapshots on demand** (`snapshots.js`): a solve result's `values` has
one entry per ready snapshot, a `Float32Array` where loaded and `null`
where not. Showing snapshot k fetches the missing ones from k − 2 to
k + 8 in one range request; beyond `keptSnapshots(nodes)` (about 64 MB of
float32, from 8 to 1000 snapshots) the loaded ones farthest from k are
dropped (`pure/run_values.js`, `snapshotPlan`). Until the shown one arrives the
picture keeps the nearest loaded snapshot and says so in its label
("u at t = 0.2 (loading t = 0.3)"); the time bar says "loading…" for its
range.

**The time bar** under the picture (play or pause, a slider, the shown
time with that snapshot's min and max) is the one control of the shown
snapshot (`store.solveFrame`); during a run it also shows how far the run
got.

**Probes.** A click on the picture (or Enter with the picture focused)
pins a probe, up to four; each keeps its colour slot while others come
and go. The list under the picture gives x, y and the shown field's
value there (interpolated page-side from the shown values, `pure/mesh_math.js`:
`meshGrid`, `locatePoint`, `interpolateAt`); for a run in time a line
chart shows u(t) of every probe from `/api/solve/probes`, asked again when
the probes or the number of ready snapshots change.

**Comparisons** (`compare.js`, the Solve result). *With an exact
solution*: an expression (kept in the case as `view.exact_solution`) is
sent to `/api/solve/compare`; the section shows max |u − exact|, the RMS
over the nodes and the lumped L2 error at the shown snapshot, the largest
max over all, and for a run in time a chart of the three per snapshot.
Next to the figures it says that this is the nodal error in double, not
the solver's residual, and that a difference picture is float32. "Show
u − exact in the picture" (`store.view.difference`) fetches the shown
snapshot's difference from `/api/solve/compare/difference` and draws it
with one diverging scale from −max to max. A comparison belongs to its
solution and its expression: another solution drops it (a new done run is
compared again with the same text), another expression in the case
greys it out with "for the previous expression" until Compare. The
expression's error is marked on its input (`path` `exact`). *With the
reference run*: "Keep as reference" (a done, fresh run;
`/api/solve/reference`, kept in `store.reference`), "Compare with
reference" (max difference per snapshot, a chart for a run in time, "Show
u − reference in the picture"), "Forget reference"; a refusal (another
mesh, a run not done) shows the server's message in the section.

**Request tickets** (`state.js`, `requestTicket`; `pure/tickets.js`).
Every asynchronous answer is applied by one rule: a request takes a ticket
of its kind with the case it was made in (`store.caseId`) and the key of
its input; `live()` is true while no newer request of the kind was made in
the same case (the answer may be kept, as outdated), `holds()` while the
input key is also the same. Kinds and keys: each stage (its stage key;
the problem check needs `holds`), `open` (New, Open, an example, a file:
any newer replacement wins), `preset` (the revisions of the geometry,
problem and solve sections: an edit made meanwhile refuses the answer
with a toast), `compare-exact` and `compare-reference` (the solution),
`difference-*` (not exclusive: several snapshots side by side, for the
same comparison), `probes` (the run, snapshot count and points),
`reference`.

**Invalid entries.** Number fields are `numberInput` (`dom.js`): text
inputs with `inputmode="decimal"` and the page's own parsing
(`pure/format.js`, `parseNumberText`: a decimal point and an exponent,
whatever the locale; a decimal comma gets its own message), shown with
`numberText` (the shortest text that reads back as the same double).
Text that is not a number changes nothing in the case: the field says
"… The case still has 0.05.", and a field with a case path is recorded in
`store.invalid` (path → label, reason). While any is recorded, every step
head says "1 field is not a number: <label>" with a link that opens the
step and focuses the field, Mesh, Assemble, Solve, Show (a field) and Run
all are disabled with the reason (`guarded` buttons), `runWithPrerequisites`
and `runAll` refuse with a toast, and Save warns. Building the forms again
(a new case, an undo, a structural edit) clears the record, since the
fields then show the case's values.

**Undo and redo** (`state.js`, `pure/history.js`). `editCase` records, per
case section it changed, the section's JSON text before and after; a
step merges the following edits from the same input element (the element
whose `input` event is being handled, `setInputGroup`) until a field
loses the focus, so typing in a field is one step. At most 100 steps; a
new case clears them. Undo and redo put the sections back, rebuild the
forms and read the geometry again when it changed. Keys: Ctrl+Z, and
Ctrl+Shift+Z or Ctrl+Y, except in a text field typed in since it got the
focus, where the browser's own undo of the field wins; the conditions that
follow a new geometry (`record: false`) are not steps of their own.
**Restore previous case** (top bar, `store.previousCase`) brings back the
case the last New, Open, example or file replaced, until the next
replacement.

**The picture** (`view/`). The legend names the field shown (`u at t =
…`, `u − exact at t = …`, `Dirichlet values at t = …`, `|∇u| at t = …`),
the Colour menu's options name it too (`field: u`, `|∇| of u`), a field
with one value says "constant: <value>". **Lock range** fixes the colour
range per quantity (field or |∇|) in `store.view.locked` (`lockedScale`),
kept until **Unlock** and editable at its ends. **|∇|** colours each
triangle by the gradient of the shown P1 field (`gradientMagnitudes`,
display only). **Line profile**: with the tool on, a drag draws a line;
the shown field along it (`sampleLine`, 200 intervals, gaps outside the
mesh) is charted under the probes and follows the shown snapshot.
Interaction: one pointer pans (a press that does not move pins a probe),
two pinch and pan, the wheel zooms, +/−/fit buttons on the picture; at
phone width a handle under the picture sets its height (kept in this
browser). Edges are drawn fainter over a field's colours. Results whose
reports hold messages show a warning chip with the count and the first
message; it opens the reports. Solution exports are CSV, VTK and
provenance; a 409 or 429 answer is said in plain words
(`exportErrorText`).

**Errors** carry `path` (`ApiError.path`); `markError` uses it, and the
text only for an error without one (`pure/stages.js`, `errorPathOf`).

## Presets

`POST /api/preset` fills a whole problem for the case's geometry:
equation, parameters, a condition for every region (the heated preset
gives the first region its own), initial conditions and the solve section
(mode, linear solver, method, dt, t1, snapshots). Lengths and times are
scaled to the domain (`DomainFacts`): its size, and a point well inside it
with its distance to the boundary, found on a grid over the bounding box
(inside an odd number of loops, farthest from the sampled boundary) or
among the triangle centroids of a mesh file. Bumps are centred there with
a width of half that distance. The page names presets only by the ids the
catalog gives; every physical choice is in `presets.cpp`, which spells out
the sign conventions (−k Δu is `a_xx = a_yy = -k`; cooling is
`alpha = h, beta_n = 1, g = h u_out`). Each preset runs on the default
rectangle and on an annulus in `tests/runs.cpp`.

## Security

- **Loopback by default.** On 127.0.0.1 (or `localhost`, `::1`) API
  requests must name a loopback host in their Host header (against DNS
  rebinding).
- **A token beyond this machine.** With `--host` anything else, the
  server prints a warning and requires a token on every `/api/` request:
  `--token TOKEN`, or 32 random hex digits made at start. It prints the
  address with `?token=...`; the page reads the token from its own
  address and sends it in `X-PDE2D-Token` (a request may also carry
  `?token=`). The page's files themselves need no token. `--token` on
  loopback requires it too.
- **Origin.** A POST that carries an `Origin` header must name the same
  host and port as its Host header (the page's own); a POST must be
  `application/json`, which a cross-site form cannot send without a
  preflight this server never answers.
- **Bounded waiting.** At most `max_waiting` + 1 API requests are served
  or wait for the session at a time; one more is answered 429 at once
  instead of queueing behind a long mesh.
- **Bounded memory.** Bodies up to 64 MiB, at most 8 clients, 1000
  snapshots per run, 64 probe points per request.

## The command line

```sh
pde2d_app --run CASE.json [--set PATH=VALUE]... [--output DIR]
```

- Runs geometry, mesh, problem check, assembly and solve (to the end, in
  the same chunks as the page's runs) and prints a JSON summary; the exit
  code is 0 when every stage ran. With `view.exact_solution` in the case,
  the summary has `comparison`: `max_abs`, `max_rms`, `max_l2` over the
  snapshots against it, with `nodes` and `longest_edge` (for convergence
  studies).
- `--set PATH=VALUE` (repeatable, applied in order before running)
  replaces one value: PATH is dotted keys with `[i]` indices
  (`mesh.improvers[0].options.use`); VALUE is read as JSON, or else taken
  as a string (`--set solve.mode=time`). Every step before the last must
  exist; the last may be a new key of an object (the case reader refuses
  one it does not know). A step that does not exist is an error naming
  the path.
- `--output DIR` writes `mesh.pde2d`, `mesh.svg`, `solution.csv`,
  `solution.vtk`, `provenance.json` and the system (`stiffness.mtx`,
  `damping.mtx`, `mass.mtx`, `rhs.mtx`), with the case file's name in
  their provenance.
- `--default-case` prints the case a new session starts with.
