# Agent guide: app/

The graphical interface of pde2d. Read [docs/app.md](../docs/app.md)
first (shape, case format, API, catalog, runs: what is true now), then
[README.md](README.md) (how it is used); the reasons are in
[docs/app-decisions.md](../docs/app-decisions.md).

## Quick start

```sh
tools/test.sh                                   # build and every test (from the root)
build/app/pde2d_app                             # the page on http://127.0.0.1:8765/
build/app/pde2d_app --run app/examples/heat_annulus.json --output /tmp/heat
ctest --test-dir build -R app_ --output-on-failure   # the app's tests only
```

- **Where things are:** HTTP in `src/server.cpp` only; everything else
  goes through `Session` (`src/session.*`), which owns the stage cache
  (`stage_cache.*`), the runs (`runs.*`) and the two locks. Case sections
  are read by `domain_input`, `mesh_catalog`/`solve_catalog`,
  `problem_input` and `stage_cache` (assembly), each against its own key
  list, from which `case_schema.cpp` is generated. Exports are one table
  in `exports.cpp`.
- **A new library component:** one entry in `mesh_catalog.cpp` or
  `solve_catalog.cpp`; the page, the schema and the tests of names and
  defaults follow (docs/app.md, "Adding a component").
- **A new case key:** add it to its reader's key list and describe it in
  `case_schema.cpp` (the schema throws on a key it does not describe). A
  change older cases cannot be read with raises `case_version` and adds
  a `migrate_vN_to_vN+1` to `case_format.cpp`.
- **A new API call:** a route in `server.cpp`, a `Session` method; say
  which lock it takes (below) and pass `waits = false` to the route when
  it never takes the session mutex.

## You decide the app's design

**The owner has delegated the app (2026-11-14).** Inside `app/` (and
`docs/app.md`), agents make the design decisions themselves: layout,
case format, API, dependencies of the app, what the page offers. You do
not need to ask the owner first. Record each decision with its reason at
the end of `docs/app-decisions.md`, and keep `docs/app.md` a description
of the app as it is (update it, do not append to it).

The delegation stops at the library. The root [AGENTS.md](../AGENTS.md)
still holds for everything outside `app/`: the library's architecture
belongs to the owner. If the app needs something the library does not
offer, raise it as a library change; do not work around the library's
rules in the app, and never reimplement library logic (checks, physics,
meshing) in the app or the page.

## Rules that keep the app honest

- **The library is the only source of truth.** The app forwards the
  library's errors with the case path (a `PathError`, sent as
  `error.path`); it does not check ranges or physics itself, and
  evaluates expressions only through `fem::CompiledExpression`.
- **Components come from the catalog** (`src/mesh_catalog.cpp`, `src/solve_catalog.cpp`): a `type` is
  the component's `name()`, defaults are read from the library's options
  structs. The page must not name components.
- **Strict reading**: an unknown key in a case is an error.
- **One thread for symbolic work**: every `Session` call that uses the
  library runs under the session mutex (`Session::Work`, which also tells
  `/api/status` what it does). Do not add work that builds or drops
  `Expression`s outside it. The short **run lock** guards only the runs'
  bookkeeping (`RunManager`, every `Run::published`, the activity);
  progress, stop, values, probes and status take only it and read
  `RunSnapshots` (the mesh and the values, no expressions). Order: the
  session mutex first, then the run lock. A run is dropped under the
  session mutex only (`RunManager::take_dropped`, emptied by `Work`).
- **`httplib.h` only in `src/server.cpp`.** The library never includes
  anything from `app/`.
- **Exports never compute a run**, and a stream that outlives the mutex
  holds no expressions (only the mesh and the values).
- **Runs belong to clients** (`X-PDE2D-Client`); lookups never create a
  client.
- **Every case edit goes through `editCase`** (state.js): stage keys are
  kept per section and recomputed only for the sections an edit names.
- **The page has no build step**: plain ES modules. Pure helpers go to
  the module of their topic in `web/js/pure/` (no DOM) and get a test in
  `tests/web.test.mjs`.
- **Every asynchronous answer takes a request ticket** (`requestTicket`
  in `state.js`, `pure/tickets.js`): its kind, the case it was made in
  and the input key it was made for; apply an answer only while its
  ticket holds.
- **Number fields are `numberInput`** (`dom.js`): text with the page's
  own decimal parsing; invalid text is recorded with its case path and
  blocks runs.

## Before finishing a change

1. `tools/format.sh --check` (C++ of the app included).
2. `tools/test.sh`: the app's tests are `app_session`, `app_solve`, `app_runs`,
   `app_compare`, `app_exports`, `app_server`, `app_run_*` (every case in
   `examples/`, `--set` and `--sweep`), `app_check_*` (each example's
   figures against `tests/example_checks.json`; a new example needs an
   entry there), `app_web` and
   `app_web_actions` (Node), `app_browser` (headless Chrome, when
   found; `app/tests/browser/run.sh --shots DIR` also saves screenshots).
3. For a page change, add or extend a scenario in `tests/browser/`,
   look at the screenshots and use the changed step.
4. Update `docs/app.md` (and `README.md` if use changes); add the
   decision to `docs/app-decisions.md`.
