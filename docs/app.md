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
    LIB["pde2d::pde2d (geometry, mesh, io, fem)"]

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
| `app/src/catalog.*` | the mesh catalog: every component, its options with the library's defaults, a function that builds it; `build_mesh_setup` |
| `app/src/domain_input.*` | the `geometry` section: shapes, SVG, mesh files, region edits; the geometry view |
| `app/src/problem_input.*` | the `problem` section: `build_problem`, `check_problem` (every error at once, with paths), the problem catalog and templates |
| `app/src/views.*` | meshes, reports and sparsity as JSON |
| `app/src/session.*` | `Session`: the stages with their cache, `StageError`, the default case, exports, `run` for the command line |
| `app/src/server.*`, `app/src/main.cpp` | HTTP (the only file that includes `httplib.h`) and the command line |
| `app/web/` | `index.html`, `style.css`, `js/`: `core.js` (pure helpers, tested in Node), `api.js`, `state.js`, `actions.js`, `dom.js` (forms from specs), `view.js` (the picture), `panels/*.js` (one per step), `main.js` |
| `app/examples/` | example cases and an example SVG |
| `app/tests/` | `session.cpp`, `server.cpp`, `web.test.mjs` (pure helpers), `web_actions.test.mjs` (overlapping requests, with a mocked fetch) |
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
  "assembly": {"t": 0},
  "solve": null
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
- `solve` is reserved for the solvers and ignored for now; `view` and
  `notes` are free for the page and for people. The page keeps
  `view.condition_archive` there (conditions of regions that are gone).
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
| `POST /api/field` | values at the nodes (`null` where there is none) of `{"kind": "expression", "text"}`, `initial_u`, `initial_u_t`, `dirichlet` or `right_hand_side`, at `t` (default: `assembly.t`; the right-hand side is assembled at that t) |
| `POST /api/export/<kind>` | a file: `mesh.pde2d`, `mesh.svg` or `geometry.svg` |

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
  `choices` or `unit` where they apply. The solve section has
  `available: false` and a message until the library has solvers.
- `shapes`: the ready-made shapes with their options.
- `problem`: the coefficients of the equation, of a condition and of the
  initial conditions in order, with the term each multiplies; help text;
  templates of common equations and conditions.
- `default_case`.

## Adding a component

1. Add one entry to `make_mesh_catalog()` in `app/src/catalog.cpp`: its
   `type` (the component's `name()`), label, description, its options with
   defaults read from the library's options struct, and `make`, which
   builds the component from an `OptionReader`.
2. That is all for the page. `tests/session.cpp` checks that every type
   equals `name()`; add its defaults to the defaults test.

Ranges are never checked in the app: the component's constructor
validates its options, and its message is shown at the field.

## Adding the solvers

When `pde2d_solve` exists:

1. Add a solve catalog (linear solvers, time methods, step controllers as
   slots, like the mesh catalog) and replace the placeholder in
   `Session::catalog()` with it (`available: true`).
2. Read the case's `solve` section into the library's objects, add
   `Session::solve` (cached after the assembly stage) and `POST
   /api/solve`.
3. Return the solution as node values and the snapshot times. The
   picture already draws node values (`view.js`, the field layer); the
   solve panel needs a time slider over the snapshots.

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
- **Region edits move boundary conditions** in the page, but only after
  the library accepted a geometry that includes them; every edit added
  since the last accepted geometry is applied in order (an edit removed
  before it ran is not). A renamed region keeps its condition,
  the halves of a split region copy it, and a merged region takes its
  target's. A condition whose region disappears is kept in
  `view.condition_archive` and comes back with its region (an edit
  removed again); a region without either gets u = 0
  (`reconcileConditions` in `core.js`).
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
