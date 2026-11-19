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

## 2026-11-16, round 2, part B (the page)

- **One rule for asynchronous answers: request tickets** (was: stages
  used generations, presets compared only the case id, an example load
  checked nothing, comparisons had their own counter). A ticket carries
  its kind, a generation, the case it was made in and the key of the
  input it was made for; `live()` and `holds()` are the two questions
  every caller asks (may it change the state; is it about what is shown
  now). Two races were reproduced and are tested: an example still
  loading replaced a case started with New meanwhile (now the `open`
  ticket of the example is superseded), and a preset's answer overwrote
  edits made after the request (now its key is the revision of the
  geometry, problem and solve sections, and the answer is refused with a
  toast). Revisions, not section texts, are the preset's key: comparing
  texts would let an edit that was typed back pass, and cost a stringify
  of the problem per answer.
- **Invalid entries block, visibly** (was: a field marked "not a number"
  that changed nothing, while Solve ran the previous value). A run with
  a value the page does not show is a silent wrong answer, so every run
  button is disabled with the reason, every step head names the field
  with a link to it, the field says which value the case still has, and
  Save warns (it still saves: losing the rest of the case would be
  worse). The record is cleared whenever the forms are built again, since
  every field then shows the case's own value. The automatic geometry
  read after an edit is not blocked: it reads the case, which is valid.
- **Number fields are text** with `inputmode="decimal"` and the page's
  own parser (was: `type="number"`, which Windows Chrome shows as
  "0,05" in some locales and which reports "" for text it cannot parse).
  A decimal comma is refused with a message naming the decimal point
  rather than read as a thousands separator or a decimal: either guess
  would be silently wrong for some users. The integer kind is left to the
  library, as every range check.
- **Undo is per section, by text.** A step keeps only the sections an
  edit changed, as JSON text before and after, so a large SVG or mesh
  file costs memory only in the steps that changed it; 100 steps. Typing
  merges into the step of the input element being typed in until a field
  loses the focus (the grouping is set around each `input` event, so no
  form needs to pass anything). Which Ctrl+Z wins: the field's own
  while it was typed in since it got the focus (letter-by-letter undo is
  what a user expects there; its `input` events still update the case and
  the same step), the case's everywhere else. A new case clears the
  history: undoing across a replacement would mix two cases; **Restore
  previous case** in the top bar covers that, until the next replacement
  (was: a toast button that vanished after 12 s).
- **Comparisons are tied to their solution and their expression.**
  Editing the expression greys the figures with "for the previous
  expression" instead of dropping them: the numbers stay readable while
  the user decides. RMS joins max and lumped L2, the three in one chart
  per snapshot (same unit, one axis); the notes next to the figures say
  that this is the nodal error in double and not the solver's residual,
  and that a difference picture is float32. Reference runs got their
  controls; a refusal (another mesh) shows the server's own message.
- **The legend names what is shown and can lock its range.** A locked
  range is kept per quantity (the field, |∇|) until unlocked, so two runs
  or two snapshots can be compared by eye; its ends can be typed. It
  applies on Enter or when a field is left: applying while typing would
  redraw the legend under the cursor. A constant field says "constant:
  value" instead of a bar with equal ends.
- **|∇u| and the line profile are display-only post-processing** of the
  shown snapshot in float32 (the gradient of the P1 interpolant per
  triangle; the interpolant at 201 points along the line). They are
  pictures, not results: the figures a user may cite (errors, probes)
  stay on the server in double.
- **Linear iterations are hidden when there were none.** The library
  declares no trait that says which solvers iterate (requested,
  library-requests.md item 10); 0 is what every direct solver reports.
- **Touch:** pinch to zoom and pan with pointer events (the canvas
  already had `touch-action: none`), zoom buttons on the picture, 24 px
  info buttons with a 44 px invisible target on coarse pointers, and a
  drag handle (also arrow keys) for the picture's height at phone width,
  kept in this browser. The toolbar's "Colour" label and "picture" in
  "Save picture" are dropped at phone width so the toolbar fits two lines.
- **Warnings visible without unfolding:** a result whose reports hold
  messages starts with a chip giving their count and the first one; it
  opens the reports. **Exports:** VTK and provenance next to the CSV;
  409 and 429 answers are said as what to do.
- **`core.js` split by topic into `web/js/pure/`** (format, color,
  mesh_math, case_model, stages, run_values, prefs, tickets, history) and
  `view.js` into `web/js/view/` (the viewer's state and layer order, the
  layers, the legend and saved picture, the interaction). `probeSeries`,
  `errorValues` and `maxAbs` were unused and are gone.
- **Browser tests wait on conditions** (a page's requests in flight are
  counted by the harness; `idle()` waits until none for a few turns of its
  event loop, since headless Chrome under virtual time may not run
  animation frames). One fixed wait remains, where the check is that
  nothing moves (Pause). New scenarios: invalid entries, the example and
  preset races, undo and redo, two tabs, the locked range with a pixel
  check of the field's colours, |∇u|, the line profile, zoom, pinch and
  the picture's height on a phone, and the reference run with the
  outdated comparison.

## 2026-11-16, round 3, part A (server and C++ side)

- **Chunks sized by their measured cost** (was: scaled towards 0.4 s per
  chunk). Every `integrate` call prepares the linear solver and assembles
  at its start, so a chunk of one interval with a snapshot at every step
  refactorized every step: at 110k unknowns, 1.7 s a step, prepares =
  steps. The run now fits seconds = F + c · steps to its chunks (least
  squares over the last 8, or all when those are equal) and picks the
  fewest steps that keep F at most 20% and the chunk at least 0.4 s,
  within max(3 s, 2F). The cap wins over the share: Stop and progress
  come between chunks only (library-requests.md, item 2), and a page
  that does not answer for 10 s is worse than a run that takes a third
  longer. 2F rather than 3 s alone when F is large: otherwise a chunk
  that is all factorization would never grow past one interval and the
  share would stay near 100%. Without a fit the chunk doubles (within
  max(3 s, twice the last)), which is what gives the fit its second
  point. Measured on the 110,768-unknown heat run with 30 snapshot
  intervals of one step each: 30 chunks, 30 prepares, 51.9 s before; 4
  chunks, 4 prepares, 8.0 s after (100 steps: 5 chunks, 11.9 s, where it
  was about 170 s). Wall time, not the integrator's `time`, is the cost:
  it is what the user waits for. Rejected: a fixed larger first chunk
  (wrong for a cheap problem with expensive steps), and assuming the
  fixed cost is the first chunk minus one step (one chunk cannot tell the
  two apart).
- **Two locks** (was: one session mutex for everything). The session
  mutex stays for anything that touches the library (it must: SymEngine's
  reference counts, and a `Run` holds expressions); a short run lock
  guards the bookkeeping. What made it safe: kept snapshots are
  `shared_ptr<const Solution>`, published under the run lock and never
  changed after, so a reader copies the pointers (`RunSnapshots`: the
  mesh and the snapshots, no expressions) and works without any lock;
  runs are dropped only under the session mutex, after the run lock is
  released (`take_dropped`, emptied when `Work` ends), so a reader under
  the run lock never sees one destroyed. Comparisons keep the session
  mutex: they construct a `CompiledExpression`. Calling a compiled
  function is thread-safe, but no lock-free caller needs it, so none was
  added. Probes, values, progress, stop and status take only the run
  lock; the 429 bound counts only calls that wait for the session (a
  progress poll during a mesh was refused as "busy" before).
- **A byte budget for runs** (was: only 8 clients and 1000 snapshots, so
  8 tabs × 1000 snapshots × 110k nodes could ask for 7 GB). The
  reservation of a run is fixed when it is made, from its snapshot count
  and nodes, so the check happens before anything is solved and a run in
  progress never needs more than it reserved. Dropping least recently
  used finished runs of other clients first: a tab that is gone should
  not block one that is used; a client's own reference and running runs
  are never dropped for its own solve (the user asked for them). Kept
  snapshots no longer carry velocities (only the state a chunk ends with
  needs them), which halves a wave run. The mesh and the stage cache are
  not counted: they are bounded by count already (16 meshes, one of each
  stage) and their size is the same order as one snapshot.
- **Gone runs say why**: `replaced` (409) for a run a newer solve of the
  same page replaced, `expired` (410) for one dropped (budget, client
  limit) or never known (a restarted server; also an unknown client). The
  page needs the difference: "replaced" means it is following the wrong
  run, "expired" that the result is gone and solving again is the answer.
  A record of gone runs (the last 4096 ids) gives the reason; ids are not
  reused within a server's life.
- **`GET /api/status`** without the session mutex: the call holding it
  (task, client, job, since), waiting requests, running runs, memory. It
  is a GET, so it carries no body and is never refused as busy. Every
  stage call now names its client, so "busy: meshing for another tab" can
  be told from "your own mesh".
- **Per-snapshot min and max in double** in the progress (was: the time
  bar computed them in float32 from the loaded values, and only for
  loaded snapshots). The whole list in every progress answer: at most
  1000 pairs, smaller than the times already sent.
- **Provenance: the commit at build time, not configure time** (the task
  said configure time; the earlier decision had rejected a commit for
  being stale between configures). A custom target runs
  `cmake/build_info.cmake` at every build and rewrites the generated file
  only when its text changes, so it is right after a commit without a
  reconfigure and an unchanged tree rebuilds nothing. `dirty` counts
  tracked files only (an untracked scratch file does not change the
  build). Outside git, or without git, both are "unknown" and the build
  goes on. The case's SHA-256 is of its canonical JSON (nlohmann's sorted
  object, compact), so saving a case with keys in another order keeps
  its hash; a small SHA-256 in the app (no new dependency for one hash).
- **The VTK time series as a stored zip** (was: one file with an array
  per time, which ParaView shows as separate arrays, not as a time axis).
  `.vtk.series` is ParaView's own JSON manifest for legacy files, simpler
  than `.pvd` (which wants XML VTK files). Stored, not deflated: no zlib
  dependency, and ASCII VTK is what users open rather than archive. Each
  member is built in memory before it is written (its CRC and size go in
  the local header, so no data descriptors, which some readers mishandle
  for stored entries); offsets past 4 GiB use Zip64 (1000 snapshots of a
  110k mesh pass it). The zip's provenance member is called
  `solution-vtk.provenance.json` so that `--output`, which writes the
  members into the directory, does not overwrite `provenance.json`. The
  single `solution.vtk` stays (one file is handier for a static solve).
- **The case schema is generated** from the catalogs and from key lists
  that the readers themselves now check against (the readers' `known[]`
  arrays became named constants in their headers). A key in a list
  without a description in `case_schema.cpp` throws, so the two cannot
  drift; the test validates every example and the default case with a
  small validator of the keywords used, and compares the schema's
  options and kinds with the catalog's. Ranges stay out of the schema:
  they are the library's, and a second copy would drift from it.
- **The inactive solve part is checked** for unknown keys and value
  types (not built), with paths: a misspelt key there was silently kept
  and surfaced only when the mode was switched. A value the library would
  refuse stays allowed, since the library is not asked.
- **Case migrations**: a chain of `migrate_vN_to_vN+1`, one per version,
  applied when a case is read (`CaseInput`), the keys checked after the
  migration (an old case may have had other keys). A current case is not
  copied (cases carry SVG and mesh texts). The test runs the mechanism on
  a fake chain of two steps.
- **The token in a header only** (was: header or `?token=` on every API
  call). The address with `?token=` is only for opening the page, which
  moves it to `sessionStorage` (per tab, survives a reload, gone with the
  tab) and out of the address bar. `Referrer-Policy: no-referrer` on
  every answer, so the first load's address does not leak through a
  link.
- **Code structure**: a table of field kinds in `Session::field`; the
  catalogs in two files with a private header for what they share;
  `combine_report` from declared rules, with a test that the rules are
  exactly the integrator's metrics (a new library metric fails loudly
  instead of being added up wrongly).
- **`--sweep`**: values separated by commas (simple to type; a value
  with a comma cannot be swept, which no current case option needs), the
  first axis slowest (reads like nested loops), one session so unchanged
  stages are reused. The observed order between consecutive rows is in
  the table, since that is what a convergence study is for; a row where
  the mesh did not change has none. A text table on stdout (unlike
  `--run`'s JSON) because it is read by people; `--output` adds JSON and
  CSV for scripts and plots.
- **Example checks in one file** (`app/tests/example_checks.json`), not
  next to each example: `examples/` is what the page offers, and a
  `.checks.json` there would be listed as an example. Bounds are physical
  (maximum principle for the heat and Laplace cases, the L-shape's error
  against its exact solution, the wave's amplitude), loose enough for
  another mesh order but tight enough to catch a sign or a scale. `--run
  --output` writes `summary.json`, which the check reads (a CTest
  fixture orders the two).

### API changes for the page (part B)

- **Token:** `api.js` now moves `?token=` into `sessionStorage`
  (`pde2d-token`) and removes it from the address (done, minimal). The
  API refuses `?token=` (403): every request, downloads included, must
  send `X-PDE2D-Token` (`apiHeaders` does).
- **Gone runs:** progress, stop, values, probes, compare, reference and
  exports answer 409 `kind: "replaced"` for a run a newer solve of this
  page replaced and 410 `kind: "expired"` for a dropped or unknown run
  (was 422 `input`). Following a run should stop on either; `expired`
  wants "the result is gone (…); Solve again", with the server's message.
  An expired reference (compare with `reference: true`) is 410 too.
- **Memory:** a solve may answer 422 `kind: "memory"` with `path`
  `solve.time.snapshots` (time mode). The progress has `memory: {bytes,
  budget}`; `/api/mesh` has `memory: {snapshot_bytes, budget}`, so the
  Solve panel can say "this run keeps about X of the server's Y" before
  solving (snapshot_bytes × (snapshots + 1)); `/api/status` has
  `memory.room` for this client. The page's current 50 MB warning can
  use these.
- **Status:** `GET /api/status` (`busy` with `task`, `yours`, `seconds`;
  `waiting`; `running`; `memory`). For "server busy: meshing (12 s)"
  while a request waits, and to explain a 429.
- **Per-snapshot range:** progress has `snapshot_min` and `snapshot_max`
  (double, one per ready snapshot); the time bar should show these
  instead of computing float32 min/max from loaded values.
- **Exports:** a new kind `solution-vtk.zip` (`application/zip`) for
  "Export VTK series (ParaView)"; provenance now has `app.commit`,
  `app.dirty` and `case_sha256` (the page may show the commit in an
  "about" line).
- **Schema:** `GET /api/case-schema` exists (for editors; the page need
  not use it). Inactive solve parts are now checked: a stored case with a
  wrong type in the unused mode's part is refused at its path, which the
  page already marks.
- Light calls (progress, stop, values, probes, status) are never 429.

## 2026-11-16, round 3, part B (the page)

- **One resolved display** (was: the picture took `shownField()` while
  the legend, the Colour menu and the comparison controls each asked
  their own questions; after the exact expression changed, the controls
  went outdated while the picture still drew the old u − exact, labelled
  as current). `pure/display.js`, `resolveDisplay`, decides from plain
  inputs what is shown: the quantity, its source and snapshot, the
  comparison and its state (`none`, `busy`, `failed`, `outdated`,
  `current`), whether the result is outdated, loading or failed, the
  precision note and the scale; `display.js` gathers the store into it.
  The picture, the legend, the Colour menu, the overlays, the comparison
  checkbox and the probes all read that one answer. Rule: a difference is
  drawn only while its comparison is `current` for the fresh solution;
  otherwise u is drawn, labelled u, and the legend says why ("u − exact is
  for the previous expression: press Compare"). The checkbox keeps the
  user's choice (it is a request, not a state) and says beside it why the
  picture does not draw it, so comparing again brings the difference back
  without a second click. Pure, so every case is a Node test.
- **An outdated result is drawn greyed with a stamp** (was: an empty
  picture as soon as the case changed). The last solution is drawn at 40%
  opacity on the mesh it was computed on (only while that mesh is the one
  the picture has: its id is checked), with "Outdated: the case changed
  since this result" over the picture and "(outdated)" in the legend; it
  never draws a comparison. An empty picture hid what the user was
  looking at; drawing it at full colour would pass it as current.
- **Failed secondary requests are kept and said** (was: snapshots,
  differences and probes swallowed their errors, so the picture stayed
  on a stand-in or "loading" forever). Each keeps its last failure and
  is not asked again until Retry, so a failing server is not asked on
  every redraw; the failure is said over the picture (snapshots,
  differences) or at its chart (probes) in plain words
  (`pure/requests.js`, `plainError`: `replaced`, `expired` with "Solve
  again", `memory`, `busy`, `network`). A difference that failed is
  never replaced by u. Progress polls, the one request a run depends on,
  retry by themselves after `busy` or no answer, doubling from 250 ms to
  4 s, and say so in the Solve step's head; the run goes on on the server
  meanwhile. Other failures end the following with the server's reason.
  The network failure got its own kind (`network`, was `internal`).
- **Recovery after a reload.** The client id is kept in the tab's
  `sessionStorage`, so a reload is the same client. A duplicated tab
  copies `sessionStorage` too, so on load a page with a restored id asks
  the other tabs on a `BroadcastChannel` whether one uses it and takes a
  new id if so (150 ms, while the catalog loads; the reloading tab is gone
  by then and does not answer). Rejected: Web Locks (not in insecure
  contexts, which `--host` gives, and a reload's lock release can race
  the new page). A new call, `POST /api/solve/current` (case → `{"run":
  progress or null}`, the run lock only, never makes a client), finds the
  client's run of the case; the page fetches the mesh and shows the run,
  following it if it still runs, else shows nothing (no stale picture).
- **The picture's state is the tab's, not the case's**: probes, the
  profile line, locked ranges, the colouring and the comparison asked
  for are kept in `sessionStorage` per tab (a reload finds them), not in
  the case's `view`. They describe how one person looks at a result, not
  the problem; writing them into the case would make every probe an
  unsaved change, put them in files shared with others, and sync them
  between tabs that look at different things. Colour maps are a
  preference of the viewer (localStorage, Settings).
- **The page's own edits are not the user's** (was: opening an example
  and Run all marked it "unsaved"). `editCase(..., {automatic: true})`
  leaves `dirty` as it was and records no undo step: the conditions that
  follow a new geometry, the refinement study's mesh levels. Normalizing
  a case when it is read no longer marks it unsaved either (the notice
  still says which sections were replaced).
- **Numbers from the right source.** The probe table's value of u comes
  from the server's double series (`/api/solve/probes`, for static
  solutions too) at the shown snapshot; a difference is interpolated in
  the page from the float32 values drawn, and the table says which. The
  time bar's min and max are the progress's `snapshot_min` and
  `snapshot_max` (double, every snapshot, loaded or not). The notes say
  what is float32: only the drawn values; the differences are computed in
  double on the server, then drawn as float32.
- **Server status in the top bar** while a request of this page that
  waits for the session has waited 0.6 s: "server busy: meshing for
  another tab (12 s)" from `GET /api/status`, polled each second only
  then, and hidden when the server is busy with exactly the stage this
  page shows as running (its step already says so). A chunk of this
  page's own run is said, since other requests wait for it. **Memory
  before solving:** from the mesh's `memory.snapshot_bytes` the Solve
  step says what the run will keep on the server against its budget once
  it is a quarter of it or more (and that it will be refused above it);
  the page's own float32 share is said beside it. A `memory` refusal
  lands on `solve.time.snapshots` (its path), as before.
- **Colour bar ticks and colour maps.** Round ticks (1, 2, 5 × 10ⁿ,
  zero always on a scale that spans it) under the bar, in tabular
  numbers, as few digits as keep them apart (`colorbarTicks`); the ends'
  exact values are the bar's tooltip and accessible name. Ticks are an
  SVG under the bar (the page has no inline styles). Maps per the
  dataviz method: sequential maps are one hue, dark (low, near the dark
  surface) to light — blue (default), orange, green; diverging maps two
  hues with a grey middle — blue–red (default), blue–orange. No rainbow.
  The choice is the viewer's (Settings), not the case's.
- **Charts without a mouse**: the plot takes the focus; the arrows move
  the readout point by point (Shift: a tenth of the points; Home, End),
  Enter picks the snapshot, and the readout is announced in a live
  region. Each chart offers "Download data (CSV)" of exactly the numbers
  it draws (full precision, empty cells for gaps): the numbers behind a
  picture must be reachable without reading pixels.
- **A boundary click selects its region** (a click within 7 px of a
  boundary edge; elsewhere it pins a probe, as before): the region is
  drawn wider on a halo, its legend entry is pressed, and its condition
  opens in the Problem step (its row in the Geometry step when that step
  is open). The legend's regions are buttons, which is the keyboard path.
- **Refinement study** in the Solve step, with an exact solution in the
  case: k levels (3–5) of the mesh's max edge length (the relative bound,
  or the absolute one when only it is set), halved level by level, each
  run with the existing actions (`runWithPrerequisites`, `compare`) and
  their tickets, one after another; a table of h (the longest edge),
  nodes, the largest nodal and lumped L2 errors over the snapshots and
  the observed orders between levels (`pure/study.js`), and a log–log
  chart with h¹ and h² reference slopes through the first L2 point. The
  mesh bound is set as automatic edits and put back at the end (also
  when stopped or failed), so the case is as it was; any edit by the
  user meanwhile stops it, and an edit of the mesh bound is kept. After a
  whole study the case's own level (the first, the cheapest) is solved
  again, so the picture and the figures are current, not the finest
  level's outdated solution; after Stop it is not (the user asked for
  the work to end), and the last solution shows as outdated.
- **Layout.** New, Open, Save, Restore, Undo, Redo and the examples are
  in one case menu (the menu button pattern: arrows, Home, End, Escape,
  Tab); the button names the case; the bar is one row at phone width
  ("unsaved" chip, server status, Run all, Settings beside it). Ctrl+Z
  and Ctrl+Shift+Z stay. A ▴ button folds the picture away at phone width
  (kept in this browser). Toasts sit at the foot of the sidebar on wide
  screens (the top of the sidebar, suggested, would cover the step's
  head and its Solve button) and at the bottom at phone width (the
  picture is at the top there): never over the picture, its time bar or
  its charts. A long component name wraps the card's head instead of
  being cut (the select also has it as a tooltip). Expert terms: the
  operators are "Mass (u_tt terms)", "Damping (u_t terms)", "Stiffness
  (u terms)" with the formula as the tooltip (and for screen readers);
  time orders read "0 · stationary", "1 · first order", "2 · second
  order". Contrast checked against WCAG AA: every text token passes 4.5:1
  on every surface (the lowest, muted text on `--surface-3`, is 4.77:1),
  so no token changed.
- **CSS by area**: `css/tokens.css` (tokens and base), `css/panels.css`
  (controls, step heads, forms, results, toast, settings),
  `css/picture.css` (viewer, time bar, legend, overlays, charts),
  `css/layout.css` (top bar, columns, tabs, narrow screens), loaded in that
  order with `<link>` tags, so the narrow-screen rules come last.
- **Tests.** Node: the resolved display (every comparison state, loading,
  failures, outdated), retries and plain messages, ticks, CSV, colour
  maps, the study's levels and orders, the condition sync that leaves the
  case saved, progress polls that retry, a run that is gone, the
  reattached run, a failed snapshot. Browser: the reproduced outdated
  comparison (display_state), a slow domain file (domain_file), failed
  snapshots and probes with Retry and the keyboard chart (failures), a
  reload that reattaches and a copied tab that does not (reload), no
  false "unsaved" (unsaved), a boundary click (region_click), the study
  on the L-shape (study), the case menu by keyboard at desktop and phone
  width (case_menu, phone_menu), the phone's one-row top bar and the
  folded picture (phone).

## 2026-11-16, round 4: the geometry editor

The owner asked for an in-app geometry editor, above all an easy way to
split boundary regions (a typed fraction is not something anyone can
guess).

- **Split on the picture, for every source.** A split tool (scissors,
  **S**) shows a marker on the boundary under the pointer, computed in
  the page with no server call per move, snapping to a curve's end (a
  corner, also every region's end) within 12 px; a click splits there.
  Why: the user points at the place; the corner snap makes the common
  case (split exactly at a corner of a merged or SVG region) exact.
- **The fraction is the server's, the edit stays `split`.** For a shape
  or an SVG the page sends the marker's point to `POST
  /api/geometry/locate`, which projects it on the exact curves and
  computes `CurveGeometry::split_region`'s fraction as the library
  measures it (the region's run in the normalized loop, lengths summed
  curve by curve in the library's order). A corner's fraction is a sum of
  whole curves, which the library's junction tolerance takes as exactly
  that corner, so no sliver curve appears. The split is tried on a copy,
  so "not splittable here" is the library's own message. Why server
  side: the curves are exact there (the page has samples), and old cases
  and `--run` read the same `{"op": "split", ...}` edit. The run logic is
  repeated from the library (library request 11 would remove that).
- **Names are offered at once.** A split names the parts `region` (kept:
  its condition stays where it was) and `region_2` (the next free name),
  in a card at the click with the second name selected: type and press
  Enter. Renaming a part right after its split rewrites that split's
  names instead of adding a rename edit, so the edit list stays readable.
- **A mesh file splits at a node.** New edit `{"op": "split_at_node",
  "region", "node", "first", "second"}`, applied to the imported `Mesh`
  like rename and merge: the region's edges must form one chain (as
  `split_region` needs one connected piece); the part from the chain's
  start to the node keeps the tag, the rest gets a new last tag. The
  locator snaps to the nearest node of the nearest boundary edge. Why a
  node: a mesh file has no curves, its boundary edges are the only
  places a region can change; node numbers are stable (edits never
  change nodes). Library request 5 now covers it.
- **Region actions on the picture.** In the Geometry step a click on a
  region opens a small card at the click: its name (Enter renames),
  Merge into, Split… (the tool). Why: the user is looking at the region,
  not at the table; the table's Edit stays (and works for every source).
- **A drawing source.** `geometry.source: "drawing"` holds loops of
  vertices and edges (line, arc, quadratic, cubic), edge i from vertex i
  to vertex i + 1, so a vertex is shared and dragging it moves both
  edges. Arcs keep the library's exact centre form (not a bulge from the
  vertices): converting a shape or an SVG then gives the same curves bit
  for bit, which the owner asked for, and the library's ellipses and
  rotated arcs fit. The price: an arc's ends are stored twice (its own
  and the vertices), so the app checks that they agree within the
  library's closure tolerance (else a vertex between two arcs could sit
  anywhere unnoticed); the editor keeps them together (an arc follows its
  moved ends by the similarity of its chord). A drawing has no region
  edits: its edges carry the final regions, so the editor shows the
  regions the library sees, and renames, merges and splits are drawing
  edits whose conditions follow like region edits'
  (`applyDrawingChange`).
- **Errors at the edge.** The library's `CurveGeometry` messages name
  loops and curves by input index; the app turns them into
  `geometry.loops[i].edges[j]` (the later loop for two that cross), and
  the page draws that edge red and dashed and keeps the user's edit.
  Indices are shown from 0 in the inspector, as in the case file and the
  messages.
- **Edit as drawing / a blank drawing.** `POST /api/geometry/drawing`
  writes the geometry after its region edits as a drawing (the server
  has the exact curves); choosing Drawing as the source starts from a
  blank 2 × 1 rectangle with the rectangle shape's four region names (so
  the default conditions still apply).
- **Speed.** The editor is the viewer's overlay: the viewer keeps its
  last full frame (mesh, field, layers) and the overlay is drawn over it
  at most once per animation frame (a timer where frames do not run). A
  drag edits a copy built from the drawing at its start; the case changes
  once when the pointer is released (one `editCase`, one undo step) and
  the geometry is read 250 ms later. No panel is rebuilt during a drag:
  the inspector's x and y fields are set in place. The view holds still
  while a drawing is edited (a new bounding box does not refit it).
- **Tools.** Select (vertices, Bézier handles, a bend handle in the
  middle of a line or an arc that makes the arc through the pointer,
  outlines and a ✥ handle that move a loop; Shift+click for several
  edges; double-click adds a vertex), rectangle, circle (one vertex, one
  full arc), polygon (close on the first point or Enter), and a grid of
  1, 2 or 5 × 10ⁿ near a fortieth of the domain. A new loop is a `hole`
  inside the domain, else `boundary` (the library decides holes; the
  name is only a guess the user can change). Keys: arrows nudge (one undo
  step per vertex until the picture loses the focus), Delete, Escape,
  Enter, and V, S, R, C, P, G for the tools. A new curve bulges by 0.15
  of its edge so its handles stay on screen.
- **Assigning a region to selected edges is the split inside a drawing**:
  a new name for part of one region copies its condition (a split), a new
  name for all of it is a rename, an existing name is a merge.
- **Visuals**: the existing tokens only; region colours as in the
  legend; vertices as small squares, handles as diamonds and rings in
  the accent colour, selection wider on the halo used for selected
  regions; no explanatory text on the picture (the help is behind the
  step's ⓘ).
- **Tests.** C++ (`app_drawing`): reading drawings and their errors'
  paths, the exact round trip of every catalog shape, an edited shape,
  every example and the example SVG, the locator (fractions, corners
  exact, refusals), a mesh file's split at a node and its refusals,
  presets and the SVG export on a drawing; the schema accepts drawings
  and `split_at_node` (`app_session`); the routes (`app_server`); a
  drawn example (`cooling_fin`) with `--run`, its figures and `--set` on
  a vertex. Node: curves, nearest points, hit testing, snapping, arcs
  through three points and arcs that follow their ends, every drawing
  edit, conditions that follow. Browser (`geometry_editor`): a new
  drawing, a polygon hole, a vertex dragged and typed, undo of the whole
  drag, a split by a click with its name typed, a corner split of a
  merged shape region through the server (fraction exactly 2/3), Edit as
  drawing, and a reload.

### Review fixes (the same day)

- **A split part renamed keeps its condition.** Renaming a part right
  after its split rewrote the split through `undoRegionEdit`, which put
  back the conditions from before the split: a condition set on the new
  part in between was lost. `renameRegionByEdit` now rewrites the split
  and then gives both parts the conditions they have at that moment
  (`from`'s under the new name); the split's undo record is unchanged,
  so undoing it still restores what was there before.
- **Gestures are bound to the geometry's revision.** An undo (or any
  change of the geometry) during a drag used to be overwritten when the
  pointer was released (the drag's copy was committed). Now a gesture
  records `sectionRevision("geometry")` when it starts and ends without
  a change when that revision moves; Escape, a cancelled pointer and a
  moved revision share one cancel path, which also redraws the
  inspector from the case (its x and y showed the dragged values).
- **Keyboard selection and typed splits.** `]`/`[` step through a loop's
  vertices and edges, L/Shift+L through loops; a live region says the
  selection and a vertex's coordinates. A drawing's edge splits at a
  typed fraction of its length (arc length: exact for lines and circular
  arcs, from 512 chords otherwise; the drawing is the case, so the
  inserted vertex is simply where it lands); a mesh file's region at a
  typed node; shapes and SVGs keep their typed fraction. Why: selecting
  and splitting must not need a pointer.
- **Incomplete drawings are kept, not replaced.** A drawing read back
  with a loop without vertices or a Bézier without controls passed
  normalization and then threw while drawing. Normalization now keeps a
  drawing as it is (it used to replace some malformed ones with a blank
  rectangle, losing the user's input), and every function of
  `pure/drawing.js` that draws, hits or selects skips incomplete parts
  (`edgeCurve` returns null, `loopsOf`/`verticesOf`/`edgesOf` give lists);
  the server's error names what is missing.
- Tests: Node (the split rename with an edited condition, also for
  `split_at_node`; keyboard stepping, the live region's words, typed
  splits by arc length; incomplete drawings never throw), browser
  (Escape and an undo during a drag, keyboard selection and a typed
  split, a renamed split part's condition, the corner split checked by
  its curves; `drawing_incomplete`: an incomplete drawing after a
  reload), C++ (the locator on a region across the loop's start and on
  holes the library turned).

## 2026-11-17: the adaptive solve

The library gained `pde2d_adapt` (error estimators, marking strategies,
`MeshRefinement`, `AdaptiveStaticSolver`) and two bisection refiners
(newest vertex bisection, red-green-blue). The app offers them as follows.

- **A third solve mode, `adaptive`**, beside `static` and `time`, rather
  than a separate step: the loop ends in a static solution, which every
  part of the Solve step already knows how to show, compare and export.
  Its part of the solve section holds the target (`t`, `tolerance`,
  `max_cycles`, `max_nodes`) and three slots (`estimator`, `marking`,
  `refiner`), built from the catalog like every component; the page's
  forms needed no new code beyond the option kind below.
- **The refiner comes from the mesh catalog**, which gains
  `newest_vertex_bisection` and `red_green_blue` (they are mesh refiners
  too). The adaptive default is newest vertex bisection: it keeps the
  parents and the angles bounded over many cycles (26.6° after 17 cycles
  on the L-shape). Improvers are not offered for the adaptive refiner:
  the flips and smoothing drop the parents, which the coming solution
  transfer will need, and an adaptive mesh that is never flipped keeps
  NVB's similarity classes. A missing slot is its default, and an older
  case gets the default `adaptive` part without a notice (it is new, not
  wrong).
- **A new option kind `tolerance`** (`{"absolute", "relative"}`, the
  relative part times the first estimate). The length bound's widget
  is reused with its label "× first estimate"; reusing `length_bound`
  itself would have said "× domain size".
- **Every budget is required in the case** (`max_cycles` 10, `max_nodes`
  50000 by default) although the library allows leaving them out: an
  adaptive run without a node budget could fill the server's memory,
  and the memory reservation needs a bound. The tolerance may be empty.
- **One library call, no progress.** The run calls
  `AdaptiveStaticSolver::solve` once and is done when started, like a
  static solve. Running the cycles one by one in the app would give
  progress and Stop, but only by reimplementing the loop's stopping
  rules (tolerance from the first estimate, node budget before solving),
  which the app must not do. Requested from the library instead
  (library-requests.md, item 2). Typical runs take well under a second
  (17 cycles to 2294 nodes: 0.13 s).
- **The run's mesh is the last mesh, under an id of its own.** Every
  reader of a run (values, probes, comparisons, exports) reads its
  `RunSnapshots`, so replacing `mesh` and `mesh_id` once, under the run
  lock, puts all of them on the adapted mesh at once; the stage cache
  reserves the id (`new_mesh_id`) so ids never collide, and
  `RunManager::mesh_of` serves its arrays like a reference run's mesh.
  The alternative (a mesh per snapshot) would have touched every reader
  and the page's snapshot machinery for no gain: only the last solution
  is shown.
- **Every cycle's solution is kept** (on its own mesh) so a comparison
  with an exact solution gives each cycle's error: the chart of the
  estimate and the true error against N is what adaptivity is about.
  Memory: the reservation is `max_cycles × (max_nodes × 64 + 512)`
  bytes, fixed when the run is made, generous because the meshes grow
  geometrically (38 MB for the example's budgets of 30 cycles and 20000
  nodes; it uses about 0.3 MB).
- **Error indicators** travel as float32 per triangle
  (`/api/solve/indicators`, run lock only), not in the progress JSON
  (tens of thousands of numbers in an answer the page polls). The
  picture colours them by log₁₀: they span decades, and a linear scale
  shows only the largest few. A zero indicator takes the lowest colour.
- **The picture draws an adaptive solution on its own mesh**
  (`pictureMesh`) while the solution is shown; a field of the Assemble
  step is still drawn on the Mesh step's mesh, and the Dirichlet layer
  is hidden on the adapted mesh (its nodes are the Mesh step's). An
  outdated adaptive result is drawn while the Mesh step's mesh is the
  one it started from (`start_mesh_id`), as other results are while
  their mesh is.
- **The cycle table and chart** show N, the estimate and its observed
  rate, and after a comparison the L2 error and its rate, with N^(−1/2)
  and N^(−1) slopes: the rates a well-adapted P1 mesh reaches in the
  energy and L2 norms. The max error and the marked count are in the
  chart and a tooltip; with them as columns the table overflowed the
  sidebar.
- **Not offered for an adaptive result:** the reference comparison (two
  adaptive runs end on different meshes; the server would refuse) and
  the refinement study (halving the starting mesh of a loop that refines
  itself measures little). A stop message after the tolerance was met is
  not counted as a warning; one after a budget is.
- **`solution-mesh.pde2d`**: the mesh the solution is on, so an adapted
  mesh can be opened as a mesh file and solved again or refined further
  (`.pde2d` keeps the vertex order, so newest vertex bisection continues
  from it).
- Example `l_shape_adaptive.json` (the corner singularity from a 28-node
  mesh to the tolerance 0.1 × the first estimate in 17 cycles; L2 error
  ∝ N^(−1)), with its entry in `example_checks.json`; tests in
  `app_solve` (the catalog against the library, the mode read strictly
  with paths, the L-shape's convergence, the refusals' paths),
  `web.test.mjs` (`adaptiveRows`, the run sizes) and the browser
  scenario `adaptive`.

## 2026-11-18: the equation as one line, no waiting, playback in time

Asked by the owner: type the equation inline (also with nabla), remove
the lag after every input, stop fields jumping, and play a run in real
time from its start.

- **Lines are the library's** (`fem::parse_equation`,
  `fem::parse_condition`, decided by the owner the same day). The app
  only stores `{"text": ...}` in place of the coefficients and forwards
  the library's messages at `.text`. Why: the expansion of
  `div(k*grad(u))` needs the coefficients' derivatives, which only the
  library may take; the app never reimplements maths.
- **A line or coefficients, never both, no version change.** The case
  keeps the form the user chose; old cases with coefficients are read as
  they are. Why: nothing to migrate, and the coefficient form stays
  available (and is what a line means, shown under it).
- **The check returns both forms** (`forms`: the line printed by
  `fem::equation_text`/`condition_text`, the coefficients read from the
  line). The page switches forms from them and shows "means …" under a
  line. Why: the page cannot convert by itself, and the meaning of a
  line is what the user must see to learn the convention (a_xx = −k for
  −∇·(k∇u)).
- **Lines everywhere new** (the default case, templates, presets, the
  seven examples; their figures are unchanged, the coefficients being the
  same). Why: what the owner asked for is the normal way in.
- **Symbol buttons, not shortcuts.** ∇ · ² Δ beside each line insert at
  the caret; ASCII names (grad, div, lap, dot) are always accepted. Why:
  no keyboard has ∇, and a button is discoverable.
- **No fixed pause before a check.** The problem check and the geometry
  run at once and coalesce (one request in flight, the newest text
  next); the "Delay after the last key" setting is gone. Why: the
  measured cost was the 350 ms and 250 ms pauses, not the work (a
  keystroke costs about 1 ms in the page, a check 3 to 5 ms on the
  server).
- **Nothing above a field changes while checking.** The summary keeps the
  last check (greyed only after 0.4 s), the status keeps its words for a
  quick check, and a line's error waits for a 0.7 s pause in typing in
  it. Why: the summary emptied and refilled on every key, moving every
  field below it; immediate errors would flicker on every key of a line.
- **Playback in simulated time.** 1× is one simulated second per second
  (a speed menu on the time bar, kept as a preference; 0.001× to 1000×
  for runs of milliseconds or hours); the snapshot shown is the last one
  at or before the clock. Why: frames per second made the speed depend
  on the number of snapshots.
- **A run plays from its start, at the run's pace.** While computed, a
  run starts playing once 4 snapshots are there and plays at
  min(speed, 90 % of the measured pace over the last 4 s), waiting at the
  newest snapshot if it catches up; the bar says the speed it plays at.
  Why: showing only the newest snapshot jumped to the end; the owner
  asked to start at the start and adapt to a slower computation (the 90 %
  keeps the waits rare). A timer drives the clock (60 ticks a second,
  at most 0.25 s per tick), not animation frames, which do not run in a
  frame that is not painted.
- **After the review (same day).** No form is offered while any field of
  the equation or condition has an error (switching would drop the text
  being typed); a coefficient form the library cannot print (a parameter
  named u) has `text: null` and the library's `reason`, said in the
  toast. The status keeps the last finished check's words also while the
  next check is only queued. Playback never moves onto a snapshot that is
  not loaded (the picture would show the previous one under the next
  time), and the fetch ahead starts at the first missing snapshot.
- **At phone width the snapshot's min and max are the time's tooltip**,
  so the time bar with the speed menu keeps one row (the picture stays
  in the top 60 % of the screen).
- Tests: C++ (lines and both forms in the check, errors at `.text`, a
  line with coefficients refused), Node (the playback clock, the pace,
  coalesced checks, `isLine`), browser (`equation_line`: typing without
  the field moving, the meaning, a symbol at the caret, an error after a
  pause, switching forms, a condition line; `time_bar`: a run plays from
  its start).
- **The worst solve combines over chunks** (2026-11-18): the integrator's
  new `max_backward_error` takes the largest over the chunks and
  `min_pivot_ratio` the smallest above 0 (a new rule,
  `Combine::min_measured`), since the library reports 0 for "not
  measured" and a plain minimum would let a chunk without a measured
  ratio hide the others'. The page does not show them yet; the pivot
  ratio warning reaches it as one of the report's messages.
- **A NaN stays in the largest over chunks** (2026-11-18, review
  finding): `Combine::max` keeps a NaN from any chunk instead of
  `std::max`, which dropped a NaN `max_backward_error` of a later chunk
  behind an earlier finite one. The integrator keeps NaN in its own
  maximum, so the combined report says the same as one long run would.
- **`t_end` combines as the last chunk's** (2026-11-18, the library's
  integrator gained the metric `t_end`, where a run ended, with its
  `max_steps`): like `t1`, the combined run ends where its last chunk
  did. The app never passes `max_steps`, so `t_end` equals `t1`.

## 2026-11-18: threads, the parallel direct solvers, the adaptive loop in time

The library gained thread counts (`AssemblerOptions::threads`, the
iterative solvers' `threads`), two parallel direct solvers
(`SupernodalCholesky`, `MultifrontalLu`) and the adaptive loop for
time-dependent problems (`AdaptiveIntegrator`, solution transfer, damping
steps). The owner asked the app to offer everything new.

- **Threads are case options, not a browser preference.** The disabled
  "Sequential / Parallel" group of the Settings dialog is gone. The
  library made the count an option of each component (and a thread count
  can change the last digits of the parallel direct solvers), so it
  belongs where the component is chosen: the solvers' own `threads`
  option comes from the catalog like any other, and a new
  `solve.assembler` part (`{"threads"}`, may be missing: one thread)
  goes to the static solver, the integrator and both adaptive loops. A
  browser preference would have made a run depend on something the case
  does not record, and `--run` could not reproduce it. The Assemble step
  keeps one thread: its stage key has no solve section, and its timing
  matters little.
- **The parallel direct solvers are two more linear solvers** with their
  `residual_tolerance` and `threads`; their declared requirement
  (symmetry for Cholesky) reaches the page as a trait like the others'.
  The default stays sparse LU.
- **A fourth mode, `adaptive_time`**, rather than an option of the time
  mode: its target, budgets and four slots would have made the time
  mode's form twice as long for every run in time. It reads the `time`
  part as it is (t0, t1, snapshots, method, step) plus its own part; the
  catalog says so with `with: ["time"]`, and the page shows the time
  part first. Switching between In time and Adaptive in time keeps the
  same interval and method.
- **A new option kind `schedule`** (`{"interval", "steps"}`, both
  optional, as the library's `AdaptationSchedule`), drawn with the length
  bound's two-field widget (one table, `PAIR_FIELDS`, for the three pair
  kinds). The default is every 10 steps: settings never know the domain
  or the interval, and a step count is meaningful for any.
- **`max_nodes` and `max_refinements` are required in the case** (20000
  and 20 by default) although the library allows leaving them out, as
  for the static loop: they bound the memory a run reserves (the meshes
  and the snapshots at the node budget), and the reserved mesh ids.
- **The damping method is built only with damping steps.** With 0 steps
  the library takes no method; building one anyway would refuse a case
  whose (unused) choice does not fit the problem's time order.
- **One library call, no progress or Stop**, as for the static loop.
  Chunking the run the way a time run is chunked would restart the
  schedule, the relative tolerance (relative to the first adaptation
  time) and the budgets at every chunk: that would be a different
  algorithm, which the app must not make. Recorded in
  library-requests.md, item 2. The example takes 0.4 s.
- **Each snapshot stays on its own mesh**; the run does not interpolate
  them to one mesh. Readers take the mesh from the snapshot
  (`Solution::mesh`), which was already there, so comparisons, probes
  and the VTK series needed a per-snapshot mesh instead of the run's and
  nothing else; each mesh gets an id (the starting mesh keeps its own,
  the new ones take ids reserved when the run is made) so the page can
  fetch it like any mesh. Interpolating to the last mesh would have shown
  the early snapshots with an error they never had.
- **Exports with one mesh refuse** such a run (`solution.csv`,
  `solution.vtk`), and the page does not offer them: a CSV row per node
  has no meaning across meshes. The VTK series writes each snapshot on
  its own mesh (ParaView follows a changing mesh); `solution-mesh.pde2d`
  gives the last mesh. A reference comparison with or of such a run is
  refused, since it compares node by node.
- **The page fetches every mesh with the result** (at most
  max_refinements + 1, typically a few MB) and draws the shown snapshot
  on its own (`pictureMesh`), so the readout, probes and profile follow
  the time bar. A snapshot that is not loaded yet is stood in for only
  by one on the same mesh: values of another mesh would be drawn on the
  wrong nodes.
- **Adaptation times** are a table (t, N, estimate, outcome) and a chart
  of N against t, not the static loop's log–log chart: the estimate at
  successive times measures different solutions, so its rate against N
  means nothing.
- **`comparison.final` in the `--run` summary**: the last snapshot's
  error besides the largest over the snapshots. For a run that starts on
  a coarse mesh the largest is the early snapshots', which hides what
  adapting did; the example's checks read the final one.
- Example `l_shape_heat_adaptive.json`: heat with the L-shape's corner
  singularity growing as 1 − exp(−t), from 28 nodes, adapting every 5
  steps: at t = 2 about 5600 nodes and an L2 error of 1.5e−4, against
  7.4e−4 on a uniform mesh of 6200 nodes. A moving bump was tried first:
  without coarsening the mesh keeps the bump's whole path, and at the
  budget the bump leaves the refined region, so it showed the loop at
  its worst. Tests in `app_solve` (the run's meshes, values, probes,
  comparison and exports; the refusals' paths; the parallel solvers and
  threads), `web.test.mjs` (`splitCounted`, `snapshotNodes`,
  `nearestLoaded` on one mesh, the run sizes) and the browser scenario
  `adaptive_time`.
- **Newmark's chunks are now exact** (the library's `Solution` carries
  Newmark's acceleration): the app needed no change to continue from it,
  only to drop it from the kept snapshots (like the velocities) and to
  count it in the state a chunk starts from (`run_bytes`). This replaces
  "a recomputed acceleration at each chunk start" of 2026-11-15.

### After the review (GPT-6.2 Sol, 2026-11-19)

- **Correction to 2026-11-17:** an adapted mesh saved as
  `solution-mesh.pde2d` does not continue newest vertex bisection's
  sequence when it is opened again. The file keeps the vertex order, but
  a mesh file goes through `Remesher::run`, which gives every triangle
  its longest edge as refinement edge (the library's rule), and `.pde2d`
  has no curve links, so later boundary splits are at chord midpoints.
  The mesh is still the same mesh; only the next refinements differ. An
  import that keeps the order would be a library change (not asked).
- **An adaptive run found running after a reload** (the page reattached
  while the one library call was under way) fetches its meshes and
  indicators when the poll sees it done, before its values are shown;
  before, its values could be drawn on the starting mesh. Meshes are
  fetched one at a time, retried while the server is busy (429), and the
  Mesh step's mesh is not fetched again.
- **u − other has a stand-in only on the same mesh**, as the solution's
  snapshots do; the readout never reads values whose length does not
  fit the drawn mesh.
- **The indicator colouring and u − other are exclusive**: choosing one
  turns the other off (the indicators are of the solution).
- **No refinement study in an adaptive mode**, also when old rows or a
  message of an earlier study remain.
- **Warnings of earlier cycles and pieces are kept**: they are added to
  the loop's report (`cycle 3, mesh_refinement: ...`, `t = 0.2 to 0.3,
  integrator: ...`, `adaptation at t = 0.3, ...`), each once, since only
  the last cycle's or piece's reports are published whole.
- **The stop reason `nothing_changed` reads "mesh unchanged"**: triangles
  may have been marked while no boundary edge could be split.
- **The cycle help names the estimator**: only the residual estimator's
  total bounds the energy error; Kelly and recovery indicate it.
- **Memory**: a node of an adapted mesh is counted as 96 bytes (88 for a
  mesh of a run in time) instead of 64 (56), with size_t triangle
  indices; a run in time of order 2 also counts each snapshot's u_t and
  u_tt, which the library holds until it returns. Velocities and
  accelerations dropped from kept snapshots now give their memory back
  (assigning `{}` kept the capacity).
- **Sweeps of an adaptive case** give the last mesh's nodes, no longest
  edge (so no rate against h: a graded mesh has no one h) and the loop's
  time once.
- **A probe** of a run in time is `inside` when any snapshot's mesh holds
  the point; each snapshot is interpolated on its own mesh or `null`.
- Not changed: the indicators the library keeps for every adaptation
  time (a run that checks often with its tolerance met keeps one array
  per check until it returns; a library matter, recorded in
  library-requests.md, item 2), the app's own point location for probes
  (library-requests.md, item 8), and `mesh_stats_json`, which measures a
  mesh the library returned without `MeshProperties`.
- **Error-controlled time steps** (`AdaptiveStep`, new in the library on
  2026-11-18) are a second step controller of the catalog, with every
  option of `AdaptiveStepOptions`; `max_dt`, infinite by default, is an
  optional number (empty: no bound), a new flag of `OptionSpec`, since
  JSON has no infinity. A chunk of a time run now continues from the
  previous chunk's `StepAnchor`: without it each chunk would start again
  at `initial_dt`, and a chunked run would differ from one call. The
  run keeps the anchor beside its last state.
- **After the re-check** (same day): the warning chip of an adaptive
  solve that met its tolerance drops only the stop message, not the
  earlier cycles' warnings; the page's memory estimate of a run in time
  of order 2 counts u_t and u_tt as the server does (the order from the
  problem check); a refinement study asks before each stage whether the
  case changed, so a mode switched while a level meshes never starts a
  solve; an adaptive run (one library call) shows no Stop, and a Stop
  that meets a done run still fetches its meshes; an error-controlled
  time run reserves the anchor's earlier steps, which a done run drops.

## 2026-11-19: speed and a steady page (owner's feedback)

- **Library changes the app needs are the app agent's** (the owner): the
  instructions (root and app AGENTS.md, decisions.md, app.md,
  library-requests.md) no longer forbid them; such changes follow the
  library's conventions and are recorded in decisions.md, marked "for
  the app, by an agent".
- **The mesh edges are a cached layer.** Measured in Chrome on a 26k
  triangle annulus: a step switch, a new snapshot and the first key typed
  each took about 1.05 s, almost all of it one `stroke()` of a path
  holding every triangle's three edges; without edges, 20 ms. Each edge
  is now kept once (`meshEdges`) and drawn in paths of 2048 into an
  offscreen layer redrawn only when the mesh, the view, the size or the
  edge colour changes; it is copied at the opacity needed (now 20 to
  30 ms with edges). The boundary is one path per region. Why a layer and
  not WebGL: the raster of the field already costs ~20 ms at this size,
  and a layer keeps one drawing code path for the picture and the saved
  PNG.
- **The sidebar is resizable** by a handle on its edge (300 px to what
  leaves the picture 360 px; double click resets; remembered).
- **Nothing moves while a run plays or a fact changes**: the time, the
  range of u and the legend's time are sized for the run's widest text by
  a hidden copy in the same grid cell (no inline styles, which the page
  avoids); the problem's summary has always the same four chips (each
  says yes or no) instead of 1 to 4, and an unreadable equation keeps the
  last readable one's facts greyed (it said "time order 0").
- **The colour range of a run in time can follow the snapshot** (legend:
  whole run, the default, keeps colours comparable across times; this
  snapshot shows a solution that decays from large initial values). From
  the server's double `snapshot_min`/`snapshot_max`, so no values are
  scanned in the page. Hidden at phone width (the legend keeps two rows).
- **Threads in one open card** for every mode: the default linear
  solver (sparse LU) has no threads, so the only thread field was the
  folded Assembly card and the owner could not find the solver's. The
  card shows both counts, or says the solver runs in one thread and names
  the ones that do (from the catalog: no component named by the page).
- **The presets moved to the case menu** ("Problems on this geometry"),
  next to the examples: ready-made starts were in three places (case
  menu, Problem step head, equation). The equation's and conditions'
  templates stay at their fields (the owner likes them there).
- **Reset view** in the toolbar (the ⌂ button on the picture, a double
  click and 0 did the same but were not found).
- **Segmented choices** (solve mode, geometry source) are equal cells
  with a filled selection: the mode row wrapped "Adaptive in time" onto
  a second line inside its border and the selection was a faint tint.
- **Progress and Stop inside runs** (library-requests.md item 2, made in
  the library by this agent: `IntegrationOutput::on_step`,
  `AdaptiveStaticSolver::solve`'s `CycleObserver`). A time run's progress
  moves at every step and Stop takes effect at the next step instead of
  the chunk's end; Continue goes on from that step with the anchor (bit
  for bit, tested). Adaptive runs left the `/api/solve` request for the
  background worker, so the page follows them like a time run: the cycle
  table fills as cycles come, the head says "Running: cycle k" (or t for
  adaptive in time) and Stop ends the loop after the current cycle or
  step, keeping what was solved; Solve on a stopped adaptive run starts
  it again (its loop is one call). Time runs stay chunked: one call
  would hold the session mutex, which the problem check needs, for the
  whole run.
- **After the review of the observers** (GPT-6.2 Sol): a stop asked on
  the step that lands on an adaptation time was lost (the integrator had
  reached that piece's end); the adaptive integrator now keeps the
  observer's answer and stops there before adapting. Stop reads
  "stopped" only once the run has published what it made (the page
  follows it until then; before, a stopped chunk's last snapshots stayed
  unseen until Continue); a Stop request that fails clears "stopping";
  a reload that finds a running adaptive run follows it before the mesh
  (whose answer waits for the loop); a stopped adaptive run's chart ends
  where it stopped and its exports say to solve again. The snapshot an
  observer gets during damping steps at order 2 has no acceleration (the
  result's copy does): documented, a run continues from the result.
- **After the review of the page changes** (GPT-6.2 Sol): the legend is
  patched around a focused control (it was rebuilt every frame and the
  range choice lost the focus during playback); the range choice stays
  at phone width, compact, with the colour bar's parts sharing the
  regions' row (hiding it left "this snapshot" active with no way back);
  the last readable equation's facts are kept per case, not on the
  summary element (a panel rebuild dropped them); the stand-in snapshot's
  label has no "(loading …)" (it moved the colour bar; the overlay says
  it); the sidebar keeps the chosen width apart from the clamped one, and
  its separator always has finite values; the docs' Process rule and the
  app's Shape section name the delegation.

## 2026-11-19: reports, the page that scrolled, no "means" line

- **The remesher's plan is a note, not a warning.** The library writes
  "plan: refiner ruppert; local: none; final: none" as the first message
  of the "pipeline" report (decisions.md, Meshing), so the warning chip
  said "1 warning" for every mesh. `report_json` sends it as `notes`
  (`is_plan_message`, views.cpp) and `gather_messages` leaves it out of
  the earlier adaptive cycles' warnings; the page shows notes muted in
  the report. *Why:* the chip is for "worked, but…"; a plan on every
  mesh trained the user to ignore it. The library keeps the plan as a
  message (the owner's decision); the app separates it.
- **Reports say where their time went.** The integrator reports
  `assembly_seconds`, `prepare_seconds` and `solve_seconds` (a library
  change for the app, decisions.md, Solvers); the chunks of a run add
  them up; any report's metrics named `*_seconds` are shown as "Where the
  time went" in ms and as a share of its time, with the rest as "other"
  (none when overlapping parts exceed the total). Report rows get a
  caret: a flex `summary` loses its marker, and a row of a stage name
  and a time did not look as if it opened.
- **The page never scrolls on a wide screen.** The layout's one grid row
  is `minmax(0, 1fr)` (an auto row grew to the sidebar's content), the
  picture column scrolls itself when the window is shorter than the
  picture's least height and its bars, and the scrolling columns are
  `position: relative`, so hidden radios and visually hidden text are
  placed inside them, not against the page (they stretched it below the
  interface). The picture with its bars is never shorter than its least
  height (`.viewer-main` `min-height: min-content`), so in a short window
  a probe's chart follows below it in the column's scroll instead of
  covering the time bar (a review finding). The `reports` browser
  scenario checks every step at 1280×560, and Play with a probe.
- **No "means" line under the equation and the conditions.** It doubled
  the field's height for what the coefficient form already shows; a
  line's error still shows under it. The check still sends `forms`
  (the switch between the forms uses them).
- **No toast when a case is opened.** "Opened X. The previous case can
  be restored from the top bar. Restore it" came with every example and
  file, and the owner found it annoying: the top bar already names the
  case and offers Restore. New case and Restore show none either.
  Failures still toast.
- **What was just used stays where it was.** Choosing another solve
  mode outdated the result, whose comparison sections then left the
  panel, so the settings (and the choice under the pointer) jumped up;
  the picture's "outdated: run again" note wrapped the toolbar onto a
  second row and pushed the picture down. Now every refresh keeps the
  focused control's place on the screen (the sidebar scrolls by what
  changed above it), the note never takes a row of its own (the stamp on
  the picture says the same), and the segmented choice stretches its
  cells (`.row` centred them, so a one-line cell sat inside a two-line
  row). Keeping the place is general rather than keeping the sections:
  any change above a control (an error box, a warning chip) moved it.
- **A chart's key hides and shows its lines.** The owner wanted to turn
  off the errors or the reference slopes; the key's names are toggles,
  the axes fit what is shown (so hiding the estimate zooms onto the
  errors) and the CSV keeps every series.
- **An exact solution can be removed.** It stayed in the case and was
  compared again after every solve, also after the problem changed so
  that it no longer fitted; "Remove" clears the expression and the
  comparison (emptying the field and pressing Compare did too, which
  nobody would find).
- **Each adaptive cycle can be looked at.** The run already kept every
  cycle's solution (for the comparisons); its meshes now get ids when the
  run is published (from the stage cache, so `/api/mesh/arrays` serves
  them like an adaptive run in time's), the progress lists them
  (`adaptive.cycle_meshes`) and `/api/solve/cycle` sends one cycle's
  values. Ids at publication rather than reserved at the start: an
  adaptive run's number of cycles has no bound when `max_cycles` is not
  set. The page shows the chosen cycle as a view of the result (its
  mesh and values) rather than turning the cycles into snapshots: the
  time bar, the exports and the comparisons are about the result's one
  solution, and all of them stay as they were. The run's colour range is
  kept so the cycles compare at a glance; the indicators and the
  comparisons are the last cycle's and are not drawn on an earlier one.
- **No text that comes and goes around the picture.** The hint "Click or
  tap the picture to pin a probe…" stood in the panel under the picture
  and left it when the mesh went outdated, so the picture grew and
  shrank; the probes' and the profile's notes were noise to the owner.
  The hint and the notes are gone (where u comes from is the value
  column's tooltip, the profile's axis its title), the panel shows only
  with a probe or a line, and its height is the user's (dragged), never
  its content's. A cycle's "(loading…)" went for the same reason.
- **Panes can be hidden.** The steps panel (◧), the view options (☰)
  and the probes' panel (dragged low, or Hide) fold away, each
  remembered in the browser: the owner wants the picture's room. "Reset
  view" went from the toolbar: it was ⌂ again, and looked like doing
  nothing when the picture was already fitted.
- **Coordinates are typed.** A probe's x and y and the line's ends are
  number fields (applied on Enter or leaving the field, six significant
  digits shown), for placing a probe exactly.
- **"N messages", not "N warnings".** Reports carry notes (a marking, a
  transfer by parents, a corner's average) with the warnings, and the
  library sends them alike; an adaptive run in time gave "36 warnings"
  that were mostly notes. The chip counts messages and says so.
- **No memory budget by default.** The owner works in one tab; the 1 GiB
  budget only refused runs and dropped other tabs' results.
  `--memory-budget` still sets one for a shared server.
- **Fixes from the review of the panes and cycles (GPT-6.2 Sol).** Probes
  on an earlier adaptive cycle read the values drawn (the server's are
  the last cycle's); the line's ends stay mounted while a run plays (only
  its chart is drawn again); a typed coordinate is applied only when its
  whole text is a number; on a phone ☰ is not offered (▴ in the toolbar
  folds the picture, and the two must not hide each other); a chart's
  click is read in the SVG's own units; the cycles table marks the cycle
  the picture shows (once it has arrived), choosing one shows the
  solution, and a superseded cycle request says nothing; a chart with
  every line hidden keeps its CSV.
- **No toast when a preset is applied.** "Reaction-diffusion: the problem
  and the solve settings are filled in. Undo" annoyed the owner like the
  one on opening a case: the forms show what changed, and Undo (Ctrl+Z,
  the case menu) takes a preset back like any edit. Failures still toast.
