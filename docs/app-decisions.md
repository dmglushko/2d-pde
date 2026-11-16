# The app's decisions

The decisions behind the app (`app/`), made by agents under the owner's
delegation (2026-11-14), oldest first, each with its reason. What the app
is now is described in [app.md](app.md); this file is the history. A
later decision may replace an earlier one: the entry says so ("was: ..."),
and the earlier entry stays as it was written. Change any of them freely
when there is a reason, and add the new choice at the end.

## Until 2026-11-16 (the first rounds)


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
  one neutral colour), one hue dark to light for values of one sign, and
  two hues around a neutral zero for values of both signs.
- **One dark theme** (2026-11-16, the owner's request; was: light and
  dark from the system setting). Every colour is a token on `:root` with
  `color-scheme: dark`; the contrasts are listed at the top of
  `style.css` (text at least 5.5:1, control borders and status dots at
  least 3.2:1). The field ramps and the smallest-angle colouring go
  through a middle stop (dark blue, clear blue, pale blue): a two-stop
  ramp from near-surface navy to pale blue had a grey, muddy middle where
  most triangles and values sit.
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
- **Time runs go in chunks that end on snapshot times** (2026-11-15; was:
  a long run blocked the app). The library's continuation
  (`integrate(..., &previous.final)`) gives progress, Stop and the
  snapshots during the run without a library change, at the price of
  granularity (one snapshot interval) and, for Newmark, a recomputed
  acceleration at each chunk start (see "Runs in chunks").
- **Every solve is a run with an id**, even a static one (done at once):
  one way for the page to get values (by run and snapshot range) and one
  progress shape. The values are no longer fetched by sending the case
  again.
- **One run at a time**, replaced by a solve of other inputs; a stopped
  run continues when the same inputs are solved again, since the server
  still has its state.
- **The first chunk runs inside `POST /api/solve`**, so the library's
  refusals keep landing on their fields; it is one snapshot interval, so
  the answer is quick.
- **A case change stops a running run** (the page sends Stop): the run
  would answer a question nobody asks any more and hold the server.
- **Typing stays instant** (2026-11-15). A keystroke used to redraw the
  picture (a per-pixel field raster), stringify the whole case for local
  storage and for every stage key (an SVG or mesh file included, several
  times), and send the whole case to the problem check. Now: stage keys
  are kept per section and recomputed only for the sections an edit
  names (`editCase(..., {sections})`; every edit must go through
  `editCase`); the picture is drawn only when something it shows changed
  (`Viewer.set` returns whether it drew; the legend is rebuilt only then);
  the case is written to storage 600 ms after the last edit and at once
  when the page is hidden or closed; a stage's request carries only its
  sections; the problem check runs after a pause (a setting) and an answer
  for a problem edited since is dropped. Measured in Chrome (heat annulus
  solved, colouring by u, 1280×800 at 2×): 50 input events in f took
  11.5 ms each before, 6.1 ms after on average and 0.38 ms each once the
  first one has updated the panel.
- **Pan and zoom move the last picture** (2026-11-15): during a gesture,
  at most one frame per animation frame shows the last full picture moved
  and scaled (`viewTransform`, `drawImage`); the full picture is drawn
  once the gesture pauses for 120 ms. What does not depend on the view is
  computed once: per mesh the quality buckets (one path per colour), per
  field the colour position of each node (interpolated per pixel instead
  of the value), per theme the colour tokens and tables. Measured: a pan
  frame took 34 ms before (a full redraw per move), now a move costs
  0.01 ms and the frame is a blit (16 ms per frame is the display's
  rate); the full redraw takes 4.2 ms at 1720×1328 pixels.
- **Every section folds** (2026-11-15): the title of each card and
  section is a button with `aria-expanded`; the state is remembered per
  section id in local storage (a per-viewer preference). Rarely changed
  ones start folded: the boundary discretizer and initial triangulator,
  the linear solver, every Reports section, region conditions when there
  are more than four regions. An error marked inside a folded section
  opens it (without changing the stored state).
- **Settings are the viewer's, not the case's** (2026-11-15): kept in
  local storage, read with `normalizeSettings` (a bad value takes its
  default). Problem check while typing (on/off, delay), playback speed,
  following the newest snapshot, significant digits, picture quality (the
  largest pixel ratio). Parallel assembly and solves are shown disabled
  with the note that the library request is recorded: visible and
  honest, not a dead control that pretends.
- **Presets come from the server** (2026-11-15) and are applied there,
  where the geometry is known: the page stays free of physics and
  presets are tested in C++ on two shapes. Applying one is undone from
  its toast.
- **The solution travels in binary** (2026-11-15): `POST /api/solve`
  answers the summary, `POST /api/solve/values` the values as float32
  (snapshots × nodes), decoded into one `Float32Array` per snapshot. JSON
  numbers took about 20 bytes a value and a parse; float32 takes 4, which
  matters at 1000 snapshots. Float32 is plenty for colours and the
  readout; `min` and `max` stay exact doubles in the summary (the colour
  scale), and the CSV export keeps full doubles. The values are asked for by
  run and snapshot range (since 2026-11-15, see "Runs in chunks"), so a
  run's new snapshots travel once. The Solve step estimates the size from the fresh mesh
  before a run and warns above 50 MB (the page keeps every snapshot in
  memory).
- **One stage registry** (`STAGES` in `web/js/core.js`): each stage's id,
  label, the case sections its result depends on, the stages that must be
  fresh first, its API call and its "running" text. `stageKey`, the
  store, the step tabs and panels, `runWithPrerequisites` (the Mesh,
  Assemble, Solve and Show buttons) and Run all all read it, so a new
  stage is one entry. A failure anywhere gives the same toast and opens
  the failing step.
- **Run all ends on the solution**: after a solve (from Run all or the
  Solve button) the picture colours by the solution. The colour choice
  lives in the store (`store.view.colorBy`); the toolbar only shows and
  sets it.
- **A problem check never queues behind a long stage.** The server runs
  everything under one mutex (the library's thread-safety rule), so a
  check sent during a mesh, assembly or solve would wait for it anyway;
  the page holds it and sends one check when the stage ends.
- **Errors belong to their input.** A stage's error keeps the key of the
  case it was produced for. Once the case changes, it is shown greyed out
  "for the previous input", its field mark is cleared, and the step's dot
  says outdated, not failed. Exports are offered only for results that
  are up to date, and the Problem step greys out a check of an earlier
  problem and says "Checking…" until the new one arrives.
- **Option errors land on their field**: `build_component` puts the
  option's name in the path when the library's message names one of the
  component's options as a word. The library's messages are not changed;
  a message that names no option keeps the path of the options object.
- **Number fields never write a guess**: text that is not a number yet
  marks the field "not a number" and leaves the case as it was; an empty
  option field means the default (as before), an empty parameter value
  is refused the same way.
- **Imported meshes take rename and merge.** `mesh::Mesh` keeps region
  names apart from the edges' tags, so both edits are a change of names
  and tags, with no geometry; the library's `validate` confirms the
  result. Split needs curves and stays refused, with a message saying
  so. The page's region editor offers rename and merge for mesh files.
- **The case remembers its file name** (the opened file's or example's,
  `case.json` for a new case) and whether it changed since it was opened
  or saved (an "unsaved changes" chip). New, Open and Examples keep the
  replaced case while their toast offers to restore it (12 s; a toast
  with a button stays longer than a plain one); afterwards it is dropped.
  Sections replaced by defaults when a case is read are listed in a
  notice that stays until dismissed: a toast was too easy to miss.
- **The assembled system exports as Matrix Market**, the format every
  sparse tool reads (SciPy, MATLAB, Julia): the free parts of A, B and M2
  and b, at `assembly.t`, so the system can be checked or solved
  elsewhere.
- **Fields are one column wide in the grid** (`.field` uses
  `minmax(0, 1fr)`): a long option name in a select no longer widens the
  panel on narrow screens.
- **Few words on screen** (2026-11-16, the owner's request). No lead
  paragraphs; descriptions of components, options, shapes and sections
  sit behind a small "i" toggle (a real button with `aria-expanded`, its
  text also the tooltip, `dom.js`: `infoToggle`); what stays printed is
  what the user needs now: errors, warnings, a hint that changes what to
  do. Plain labels: "min u", "max u", "Time order 1" (both orders only
  when assembly's differs from the problem's), "‖b‖₂, right-hand side".
- **Action and result first** (2026-11-16). Every step starts with a
  head that stays at the top while its panel scrolls: the title, the
  primary action (Solve becomes Stop during a run and Continue after a
  stop; Problem's is its preset menu; Geometry is read on its own) and a
  one-line status (running with the seconds, up to date, outdated,
  failed, stopped), with "Next: <step>" once the result is up to date
  (`panels/head.js`). The result follows; the settings come last, under
  a "Settings" rule. A result of an earlier case is shown muted.
- **Tabs always fit** (2026-11-16): five equal grid columns; in a sidebar
  narrower than 410 px (a container query) the dot sits above the label.
  The tabs take the arrow keys, Home and End.
- **The time bar lives under the picture** (2026-11-16; was: a slider
  and player in the Solve panel): what it controls is the picture, and at
  phone width it stays next to it. There is one copy, so they cannot
  disagree.
- **Probes and charts are page-side** (2026-11-16): the snapshots are
  already in the page as float32, so u(t) at a point is an interpolation,
  not a request. Charts are hand-written SVG (`charts.js`, no library):
  2 px lines, a hairline grid, round ticks, a marker at the shown time,
  a crosshair with values, the probe list as the legend, end labels left
  out where two would touch. Probe colours are series slots 1 to 4
  (validated as a set on the dark surface).
- **Dot fields read the nearest dot** (Dirichlet values, b): the readout
  under the pointer gives the value of the nearest node with a value
  within 8 px (`nearestValuedNode`).
- **Isolines** (2026-11-16): marching triangles on the P1 field
  (`core.js`: `isolines`), at levels evenly inside the colour scale
  (count from Settings). Over the field's own colours they are drawn
  light on a dark halo (lines in the field's colours would vanish);
  without the colouring, each line takes its level's colour.
- **The comparison asks the library per snapshot** (2026-11-16): one
  `/api/field` request per snapshot time. The library keeps the
  expression rules and its messages; the page only subtracts. It costs a
  request per snapshot (fast for the usual 50 to 100; a run of 1000
  snapshots takes a few seconds and shows its progress).
- **Save picture** (2026-11-16) saves the canvas as drawn (at the
  picture-quality pixel ratio) with the legend drawn underneath as a
  PNG, named after the case and the shown time (`pictureName`).
- **Keyboard** (2026-11-16): the picture is focusable (arrows pan, Shift
  for larger steps, + and - zoom, 0 fits, Enter pins a probe at the
  centre); file choosers are real buttons that open hidden inputs; every
  control has a visible focus ring.
- **Phone layout** (2026-11-16): below 860 px the tabs come first and
  stay at the top, the picture with its toolbar, time bar and legend
  stays under them (28% of the height), then the probes and the panel
  scroll. On a short screen (a phone held sideways) nothing sticks.
- **A real-browser test** (2026-11-16): `app/tests/browser/run.sh`
  starts `pde2d_app` on a free port with the page and a harness, runs
  each scenario in headless Chrome and fails on a failed check, a page
  error or a scenario that does not finish. CTest runs it as
  `app_browser` when Chrome is found (`PDE2D_CHROME`, Chrome or
  Chromium on PATH, or the Windows Chrome from WSL, which reaches the
  WSL port only after a moment: a page that did not load is tried
  again). `--shots DIR` saves a screenshot per scenario.

## 2026-11-16, round 2, part A (server and C++ side)

- **The server compares in double** (replaces "The comparison asks the
  library per snapshot" above). The page subtracted an exact expression
  from its float32 copy of u, so it reported rounding (3e-8 for the
  Laplace preset u = x, whose true error is about 1e-15), and it sent one
  request per snapshot. `POST /api/solve/compare` evaluates the
  expression with `CompiledExpression` (the library stays the only
  evaluator) at the nodes at each snapshot's time, under the session
  mutex, and answers max |u − exact|, the nodal RMS and the lumped L2
  norm per snapshot. The nodal difference of one snapshot comes on
  request in float32, for the picture only.
- **Lumped L2, not consistent mass.** sqrt(Σ w_i d_i²) with w_i a third
  of the area around node i is the L2 norm of the P1 difference with
  nodal quadrature: cheap, computed from the mesh alone, and of the same
  order as the consistent-mass norm. The consistent norm needs the mass
  matrix, which the app can only get by assembling a problem made for it;
  a library function for norms is requested (library-requests.md).
- **Reference runs compare node by node on the same mesh object.** A
  comparison of two meshes needs interpolation between them, which is
  mathematics the library should own; the same mesh object (pointer
  equality, which the stage cache gives while the geometry and mesh
  sections stay the same) makes node i the same point in both, so the
  comparison is exact and needs nothing new. Another mesh is refused with
  a message saying why. A static reference stands for every time (a
  steady state against a transient), snapshots are matched by time
  otherwise. Only a done run can be a reference: a reference that still
  changes is not one.
- **Solution exports never compute** (was: `solution.csv` solved the case
  to the end first). An export that silently ran (or continued) a solve
  made a download take minutes and changed the client's run; now it takes
  the client's done run of the same inputs, found by their text alone,
  or answers 409. The page offers it only for a done, fresh result
  anyway.
- **Large exports stream.** The CSV and VTK files are written as they are
  sent (an httplib chunked content provider), from shared pointers to the
  mesh and the snapshots, which hold no expressions: the stream may run
  after the mutex is released without breaking the SymEngine rule.
- **One run per client** (replaces "One run at a time" above). Two tabs
  replaced each other's run and could stop each other. Each page instance
  makes a random id when it loads (in memory: a duplicated tab must not
  share it, which sessionStorage would) and sends it in
  `X-PDE2D-Client`. At most 8 clients, the least recently used dropped
  (with its runs) when a new one solves; lookups never create a client,
  so a stale tab cannot push a live one out. The stage cache stays shared
  (keyed by text, so sharing is correct) and so does the mutex
  (SymEngine). The background runner takes running runs round robin over
  clients.
- **Errors carry `path` as a field** (was: the path only at the start of
  the message, which the page parsed). `PathError` (an
  `std::invalid_argument`, so the library's error policy still holds)
  carries the path and the detail; `what()` stays "path: detail" so the
  command line and old readers still read a path in front. `staged`
  turns it into a `StageError` with the path, and the server sends it as
  `error.path`. The page uses the field and parses the text only for an
  error without one. Shape options now get the option's own path
  (`geometry.options.width`) through the same `named_option` as
  component options.
- **No facts for an expression that did not parse.** The check left the
  default (zero) expression in its place and reported `zero: true` next
  to the parse error, which contradicted it.
- **Provenance in every export**, in the format's own comment form (`%`
  in Matrix Market after the banner, `#` lines on top of the CSV, an XML
  comment in SVG, the title line in VTK), and in full in
  `provenance.json`. A result file that cannot say which settings made
  it is not reproducible. `#` comments in CSV are not standard, but
  pandas, NumPy and R read them with one option (documented in app.md);
  a separate metadata file alone would get lost when the CSV is copied.
  `.pde2d` gets none: its format must stay exactly the old one.
- **The app's version is the project's.** The app is built from the same
  tree as the library, with the same CMake project version; a separate
  number would only drift. A git commit was considered and left out: it
  would be stale between configures and wrong for an uncommitted tree.
- **VTK legacy for ParaView**: one ASCII file, readable everywhere, with
  the mesh and one point-data array per kept snapshot (`u(t=...)`). One
  file per snapshot with a `.pvd` index would give ParaView a time axis,
  but many files per export; arrays by time suit the sizes of this app.
- **The mesh travels in binary** (was: JSON arrays of numbers). `/api/mesh`
  answers a summary with the mesh's id and where its arrays are; `POST
  /api/mesh/arrays {"mesh": id}` sends float64 nodes, uint32 triangles,
  boundary edges and region tags. JSON took about 20 bytes a number and a
  parse; the nodes stay exact doubles. The mesh id (not the case) names
  the arrays, so a second request never meshes again, and the last 16
  meshes (and every kept run's) stay findable.
- **The page keeps a bounded window of snapshots** (was: every snapshot
  in memory). Values come by range around the shown snapshot (2 behind,
  8 ahead, one request) and the farthest are dropped beyond about 64 MB.
  What needs every snapshot (comparisons, u(t) at probes) is computed on
  the server in double instead, so nothing in the page needs them all.
- **Probes on the server** (replaces "Probes and charts are page-side"
  above for u(t)): with a window of snapshots the page cannot interpolate
  u(t) over all of them. A uniform grid of about one triangle per cell is
  simple and fast for meshes from our mesher; barycentric coordinates in
  double with a 1e-10 tolerance decide inside (not exact predicates: a
  probe on an edge may land in either neighbour, which gives the same
  value up to rounding). Point location belongs in the library; it is
  requested.
- **`session.cpp` split by concern**: the stage cache, the run manager,
  field evaluation and exporters each have their own unit; `Session`
  stays the facade, so the server and `--run` did not change shape. Run
  states and export kinds are enums; exports are a table of kinds with
  what each needs and its writer, so a new export is one row.
- **`--set PATH=VALUE`** for parameter studies from the shell: VALUE as
  JSON else as text (so `--set solve.mode=time` needs no quotes), every
  step but the last must exist (a typo in the middle fails at once,
  naming the path), the last may be new (strict reading refuses an
  unknown key anyway). With `view.exact_solution` in the case, `--run`
  reports the comparison, so a convergence study is a loop over `--set`.
- **A token beyond loopback** (was: no login, a warning in the README).
  Listening on another address printed nothing and let anyone on the
  network drive the app. Now it prints a warning and requires a token
  (random, or `--token`), printed in the address the page is opened
  with; the page passes it on from its URL. The page's files need none
  (they hold nothing), and a token in the URL is visible in the browser's
  history: acceptable on a trusted network, which the warning asks for.
- **Origin checked on POST** when present: a browser always sends it on
  a cross-site POST, so a page elsewhere is refused even when the Host
  looks right.
- **429 instead of an unbounded queue**: every API call waits for one
  mutex, so a burst behind a long mesh used to pile up in httplib's
  threads. More than 6 waiting requests get 429 at once with a message;
  the page shows it like any error.
- **The launcher finds SymEngine**: a hand-built SymEngine under
  `~/.local/opt` is the common setup here and was the usual first-start
  failure. On a failed configure the script retries with the newest
  `~/.local/opt/symengine*/lib/cmake/symengine`, names nlohmann/json and
  its package when that is what is missing, and configures again when a
  failed configure left a cache without build files.
- **Two more examples**: an L-shaped domain with the r^(2/3) corner
  singularity and its exact solution in `view.exact_solution` (a
  convergence study shows the reduced rate), and steady
  convection-diffusion past a heated obstacle in a channel drawn as an
  SVG in the case (a flow that is not symmetric: `sparse_lu`, and a
  diffusion large enough that the cell Peclet number stays below about 2
  on the example's mesh).
- **The design doc describes, the decisions file explains** (this file):
  app.md had become a changelog where the current API had to be pieced
  together from decisions that replaced each other.

### API changes for the page (part B)

What changed for the page in this round, all documented in app.md:

- Every request sends `X-PDE2D-Client` (`api.js`, `clientId`) and, when
  the page's address has `?token=`, `X-PDE2D-Token` (`apiHeaders`).
  Exports send `X-PDE2D-Case-Name` (URL-encoded): `downloadExport(kind,
  caseData, caseName)`.
- `/api/mesh` answers `mesh` as a summary (`id`, counts, `region_names`,
  `has_links`, `arrays`); `api.mesh` fetches `/api/mesh/arrays` and
  returns `mesh` decoded (`decodeMesh`: `Float64Array` nodes,
  `Uint32Array` triangles and `[a, b, region]` boundary edges).
- Errors have `path`; `ApiError.path`, `markError` uses `errorPathOf`.
- A solve result keeps `layout` (the summary's `values`) and `values` with
  `null` for snapshots not loaded; use `snapshotValues(data, k)` or
  `loadAround(data, k)` (`snapshots.js`) before reading `values[k]`.
  `api.solveValues` is now `api.solveRange(job, first, count, layout)`.
- Progress has `mesh_id`; a failed run's `error` has `path`.
- New calls: `api.compare(job, {exact} | {reference: true})`,
  `api.compareDifference(job, against, k, nodes)`, `api.keepReference(job)`,
  `api.clearReference()`, `api.probes(job, points)`.
- `compare.js` keeps the server's `linf`, `rms`, `l2` per snapshot and
  fetches `errors[k]` on demand (`comparisonValues(k)`); its expression
  input's path is `exact`.
- `probes.js` draws u(t) from `/api/solve/probes`; `probeSeries` in
  `core.js` is no longer used by the page (kept, with its test, for part
  B to remove or reuse).
- Exports `solution.vtk` and `provenance.json` exist; the page offers
  neither yet. A solution export before the run is done answers 409
  (`kind: "not_ready"`).
- Not in the page yet: the reference run's controls, the comparison's
  RMS, a warning text for 429 (`kind: "busy"`).
