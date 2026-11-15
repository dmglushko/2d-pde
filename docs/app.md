# The app

How the graphical interface in `app/` is built. How to use it is in
[app/README.md](../app/README.md).

**The app's design is delegated** (the owner, 2026-11-14). Agents make
the app's decisions themselves and record them in the decision list
below. The library's rules still hold: the app must not change the
library's design, and anything the app needs from the library is a
library change, raised with the owner as usual (see
[decisions.md](decisions.md)).

## Shape

```mermaid
flowchart LR
    subgraph Browser
        PAGE["app/web: HTML + JS modules"]
    end
    subgraph pde2d_app
        SERVER["server.cpp · cpp-httplib"]
        SESSION["Session: stages + cache"]
        INPUT["catalog / domain_input / problem_input"]
        VIEWS["views: library results as JSON"]
    end
    LIB["pde2d::pde2d (geometry, mesh, io, fem, solve)"]

    PAGE -- "JSON over HTTP (127.0.0.1)" --> SERVER
    SERVER --> SESSION
    SESSION --> INPUT
    SESSION --> VIEWS
    INPUT --> LIB
    SESSION --> LIB
```

- **One program, `pde2d_app`**, serves the page and a JSON API on
  127.0.0.1, and runs a saved case without the browser (`--run`).
- **The page holds no physics.** It edits a *case* (JSON), sends it to the
  server and draws what comes back. Every check is the library's own:
  the app forwards the library's messages with the path of the field
  they concern.
- **The page is generated from a catalog.** The server describes every
  component and its options (`GET /api/catalog`); the forms are built from
  that, so a new library component needs one catalog entry in C++ and no
  change to the page.
- **The library stays as it is.** The app uses only public headers,
  through the umbrella target `pde2d::pde2d`, which links every module.

## Files

| Path | Contents |
| --- | --- |
| `app/src/options.*` | `OptionSpec` (an option as the page sees it), `OptionReader` (reads a JSON object strictly against the specs), small JSON readers |
| `app/src/catalog.*` | the mesh and solve catalogs: every component, its options with the library's defaults, a function that builds it; `build_mesh_setup`, `build_solve_setup` |
| `app/src/domain_input.*` | the `geometry` section: shapes, SVG, mesh files, region edits; the geometry view |
| `app/src/problem_input.*` | the `problem` section: `build_problem`, `check_problem` (every error at once, with paths), the problem catalog and templates |
| `app/src/views.*` | meshes, reports and sparsity as JSON |
| `app/src/session.*` | `Session`: the stages with their cache, `StageError`, the default case, exports, `run` for the command line |
| `app/src/server.*`, `app/src/main.cpp` | HTTP (the only file that includes `httplib.h`) and the command line |
| `app/web/` | `index.html`, `style.css`, `js/`: `core.js` (pure helpers, tested in Node), `api.js`, `state.js`, `actions.js`, `dom.js` (forms from specs), `view.js` (the picture), `panels/*.js` (one per step, and `components.js`: the cards of component slots), `main.js` |
| `app/examples/` | example cases and an example SVG |
| `app/tests/` | `session.cpp`, `solve.cpp` (the Solve step, with exact solutions), `server.cpp`, `web.test.mjs` (pure helpers), `web_actions.test.mjs` (overlapping requests, with a mocked fetch) |
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
  }
}
```

- `geometry.source` is `shape` (`shape`, `options`), `svg` (`svg`: the
  document's text, `name`) or `mesh_file` (`format`: `gmsh` or `pde2d`,
  `text`, `name`). Shapes and SVGs take `region_edits`: `{"op": "rename",
  "from", "to"}`, `{"op": "merge", "source", "target"}`, `{"op": "split",
  "region", "fraction", "first", "second"}`, applied in order with
  `CurveGeometry`'s methods.
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
  solution at `snapshots` + 1 equal times from t0 to t1. A case written
  before the solvers has `"solve": null`, which means the default
  section (a static solve).
- `view` and
  `notes` are free for the page and for people. The page keeps
  `view.region_edit_undo` (one undo record per region edit) and
  `view.condition_archive` (conditions of regions that are gone) there.
- `version` is checked: a case from a newer app is refused, not misread.

## The API

All on `127.0.0.1:8765` by default. Every POST body is a case (or, for
`/api/field`, `{"case", "field"}`) and must be sent as
`application/json`.

| Call | Gives |
| --- | --- |
| `GET /api/catalog` | the catalog (below) and the default case |
| `GET /api/default-case` | the case a new session starts with |
| `GET /api/examples`, `GET /api/examples/<name>.json` | the example cases |
| `POST /api/geometry` | loops (each curve sampled for drawing, with its region and kind), regions and lengths, bounding box, domain size; for a mesh file its boundary edges |
| `POST /api/mesh` | the mesh (flat arrays), stats, `target_met`, the resolved bound, every stage report |
| `POST /api/problem` | the check: `ok`, `errors` (`path`, `message`), facts per path, each condition's kind, time order, needed initial conditions |
| `POST /api/assemble` | the `MeshedProblem` report and Dirichlet nodes; the assembled operators' facts and sizes, a sparsity picture, the assembly report |
| `POST /api/solve` | `mode`, `times` and `values` (one array of node values per time), `min` and `max` over all of them, `time_order` (assembly's, `Assembler::time_order`) and `problem_time_order` (as written), `free_unknowns`, the reports (`assembly` and `static_solver`, or `integrator`) |
| `POST /api/field` | values at the nodes (`null` where there is none) of `{"kind": "expression", "text"}`, `initial_u`, `initial_u_t`, `dirichlet` or `right_hand_side`, at `t` (default: `assembly.t`; the right-hand side is assembled at that t) |
| `POST /api/export/<kind>` | a file: `mesh.pde2d`, `mesh.svg`, `geometry.svg` or `solution.csv` (x, y and u at every kept time, one row per node) |

A failure is `{"error": {"stage", "kind", "message"}}`. The HTTP status
is 400 for a body that is not JSON, 422 when the library refused the input
(`kind: "input"`, from `std::invalid_argument`) or an algorithm could not
finish (`kind: "algorithm"`, from `std::runtime_error`), and 500 for
anything else. A message that concerns a field starts with its path in the
case (`mesh.refiner.options.min_angle_degrees: ...`); the page marks that
field.

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
  templates of common equations and conditions.
- `default_case`.

## Adding a component

1. Add one entry to `make_mesh_catalog()` or `make_solve_catalog()` in
   `app/src/catalog.cpp`: its
   `type` (the component's `name()`), label, description, its options with
   defaults read from the library's options struct, and `make`, which
   builds the component from an `OptionReader`.
2. That is all for the page. `tests/session.cpp` and `tests/solve.cpp`
   check that every type equals `name()`; add its defaults to the
   defaults test.

Ranges are never checked in the app: the component's constructor
validates its options, and its message is shown at the field.

## The Solve step

`Session::solve` is cached like the other stages, keyed by the mesh,
problem and solve sections (not by `assembly`). The page shows the
solution in the picture as a field with one colour scale for every
snapshot (from `min` to `max`), so a colour means the same value at
every time; a slider and a player walk through the snapshots, changing
only the picture.

## Decisions

Made by agents under the delegation above. Change them freely when there
is a reason; record the new choice here.

- **A local web server with a browser page,** not a desktop toolkit:
  works from WSL with the Windows browser, needs no GUI library, and the
  page can be edited without rebuilding.
- **In this repository, behind `PDE2D_BUILD_APP`** (on by default), so a
  library change that breaks the app fails the same build and tests.
- **`pde2d::pde2d`** bundles the modules into one target; the modules stay
  separate libraries.
- **cpp-httplib, vendored** (one header, MIT, 0.18.3), used only in
  `server.cpp`; **nlohmann/json** from the system. Neither reaches the
  library.
- **No build step for the page**: plain JavaScript modules, no npm.
- **One case object** holds everything, including the SVG or mesh text,
  so a saved case is complete and can run on the command line.
- **Strict reading**: unknown keys are errors; ranges are left to the
  library, so there is one source of truth for every rule.
- **The server keeps the last result of each stage**, keyed by the case
  sections it was built from, so changing the problem does not remesh.
  All work runs under one mutex: SymEngine's reference counts are not
  thread-safe in this build, and the app has one user.
- **Only this machine can drive the server**: it listens on 127.0.0.1;
  API requests must name a loopback host (against DNS rebinding) and send
  JSON (which a cross-site form cannot).
- **Region edits carry their boundary conditions and undo exactly.**
  Adding an edit moves the conditions at once, as the library will apply
  the edit (a renamed region keeps its condition, the halves of a split
  copy it, a merged region takes its target's), and stores the earlier
  condition of every name it touched in `view.region_edit_undo`, one
  record per edit. Edits are undone last first, which restores those
  conditions exactly, whether or not the library accepted the edit. The
  case is consistent at every moment, so saving or reloading never loses
  a move. Any other change of the regions (another SVG or shape) is
  reconciled after the library accepts the geometry: a condition whose
  region disappears is kept in `view.condition_archive` and comes back
  with its region; a region without either gets u = 0 (`core.js`:
  `applyRegionEdit`, `undoRegionEdit`, `reconcileConditions`).
- **Only the latest answer counts.** Every request of a stage carries a
  generation, and every loaded case an id; an answer to an older request,
  or for a case that was replaced, is dropped. Conditions are synced only
  for the geometry that is still in the case.
- **Region names are arbitrary keys**: the page keeps region-keyed
  conditions in prototype-free dictionaries, so names such as
  `constructor` or `__proto__` (valid region names) behave like any other.
- **A case is normalized before it is shown** (`normalizeCase`): a missing
  or malformed section, down to the containers the forms write into
  (shape options, component choices and their options, region edits,
  each boundary condition), is replaced by the default's, with a notice, and
  a case the forms still cannot show is replaced by a new case instead of
  failing on every visit.
- **Request bodies are limited** to 64 MiB (`ServerOptions::
  max_body_bytes`); larger ones get 413.
- **Whole numbers are checked before they are narrowed** (`OptionReader::
  small_integer`, priorities): a value that does not fit is an error.
- **Fields are drawn with exact linear interpolation** per pixel; values
  given at some nodes only (Dirichlet values, b) are drawn as dots.
- **Colours** follow a fixed categorical order for regions (eight, then
  one neutral colour), one hue light to dark for values of one sign, and
  two hues around a neutral zero for values of both signs. There is a
  light and a dark theme.
- **The browser keeps the current case** in local storage as a
  convenience; saving a case file is the reliable way to keep it.
- **One double-click starts it** (`app/Start pde2d app.cmd`, 2026-11-15).
  The `.cmd` only finds its WSL distribution and folder from its own path
  and runs `app/start-app.sh` there; the script holds all the logic
  (incremental build of `pde2d_app`, start, wait until `/api/catalog`
  answers, open the default browser), so it is the same in Linux. The app
  lives as long as the window: closing it stops the app, so no server is
  left behind unseen. A second start finds the running app and only opens
  the page. Pass the distribution name to `wsl.exe -d` unquoted: it takes
  quotes as part of the name.
- **The Solve step offers what pde2d_solve offers** (2026-11-15): a static
  solve or a time integration, every linear solver, time method and step
  controller from the solve catalog. Nothing is pre-checked in the app:
  a method that does not support the problem's time order, or a solver
  whose symmetry requirement a matrix misses, is refused by the library
  and its message is shown. The page only shows the declared traits as
  chips, and a hint when a static solve would drop time terms.
- **A new case's dt is 0.01**, not the library's default: `FixedStepOptions`
  has `dt = 0`, which its constructor refuses. The catalog still shows the
  library's default; only `default_solve_section` sets a usable step.
- **Snapshots are equal intervals of [t0, t1]** (at most 1000, the app's
  own limit, since each is a value per node sent to the page). A dt that
  does not divide the interval shortens steps to land on them, which makes
  the integrator prepare the linear solver again for each changed step:
  pick snapshots and dt so that they fit.
- **A long run blocks the app until it ends.** The Integrator offers no
  progress or cancellation, and all work runs under the session mutex.
  If long runs become common, a progress and cancel callback in the
  library is the change to ask for.
- **The solution is sent as node values per snapshot** (JSON numbers), not
  compressed: simple, and fine for meshes of thousands of nodes. A binary
  format is the change to make if the page gets slow on large runs.
- **Fields are one column wide in the grid** (`.field` uses
  `minmax(0, 1fr)`): a long option name in a select no longer widens the
  panel on narrow screens.
