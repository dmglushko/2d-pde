# pde2d app

A graphical interface for the pde2d library, in a browser: choose a
domain, mesh it, write the problem, assemble it, solve it (statically, in
time with the snapshots shown while the run goes on, or adaptively,
refining the mesh where the error is estimated, statically or in time),
compare the result
with an exact solution or another run, and export it (CSV, VTK for
ParaView, Matrix Market, provenance). The same cases also run from the
command line, alone or as parameter sweeps.
It is a separate program on top of the library; the library never depends
on it. How it is built is described in [docs/app.md](../docs/app.md), and
why in [docs/app-decisions.md](../docs/app-decisions.md).

## Start it

**In Windows, double-click `app/Start pde2d app.cmd`** (in Explorer, the
checkout is under `\\wsl.localhost\Ubuntu\...`; a desktop shortcut to the
file works the same). A window opens, builds the app if something
changed, starts it and opens it in your browser. Closing that window
stops the app. Double-clicking again while it runs just opens the page.
In Linux or WSL, `app/start-app.sh` does the same.

The first start configures the build directory (`build/`) if there is
none. When SymEngine is not found on its own, the launcher tries the
newest `~/.local/opt/symengine*/lib/cmake/symengine`; when nlohmann/json
is missing, it says so with the package to install. After that, the
launcher rebuilds only what changed.

By hand, from the project root, in Linux or WSL:

```sh
tools/test.sh                 # builds everything, the app included
build/app/pde2d_app           # prints: open http://127.0.0.1:8765/
```

Open the printed address in a browser. Under WSL, the Windows browser
reaches it at the same address. Stop the app with Ctrl+C.

- `--port 9000` picks another port (if 8765 is taken).
- `--host 0.0.0.0` makes it reachable from other machines on the network.
  The app then prints a warning and needs a token on every request: a
  random one, or `--token TOKEN`. Open the printed address (it ends in
  `?token=...`); the page keeps the token for this tab, removes it from
  the address bar and sends it with every request. Anyone with the
  address can drive the app, so do this only on a network you trust.
- `--memory-budget 2G` bounds what the kept runs of all tabs may hold
  (default 1G; K, M and G are powers of 1024). A solve that would not fit
  even after dropping other tabs' finished runs is refused with advice
  (fewer snapshots, a coarser mesh).
- `--web DIR` serves the page from another directory. By default the page
  is served from `app/web` in this checkout, so editing a page file needs
  only a browser reload, not a rebuild.

The app is built when `PDE2D_BUILD_APP` is on (the default). It needs
nlohmann/json (`nlohmann-json3-dev`); cpp-httplib is included in
`app/third_party/`.

## Use it

The left side has five steps; the picture on the right shows the latest
result. A dot on each step says whether its result is up to date (green),
outdated because the case changed since (yellow), failed (red), or is
running (a pulsing blue dot). Each step opens with its action (Mesh,
Assemble, Solve) and a status line that stays at the top while you
scroll: running with the seconds, up to date, outdated or failed, and
**Next: …** to the following step once the result is up to date. The
result comes next, the settings below it. Descriptions of options and
components are behind the small **i** buttons. An
error shown after the case changed is greyed out and marked "for the
previous input". Numbers are written with a decimal point (`0.05`, `1e-3`)
whatever the browser's language. A number field whose text is not a
number (`-`, `1e`, `0,05`) changes nothing: it says what the case still
has, every step's head names it with a link to it, and Solve, Mesh,
Assemble and Run all wait until it is fixed (Save warns that the file
gets the previous value). A warning chip at the top of a result shows how
many messages its reports hold and the first one; click it to open the
reports.

**Undo and redo** (in the **Case** menu of the top bar, or Ctrl+Z and
Ctrl+Shift+Z or Ctrl+Y) take back and redo edits of the case, up to 100;
typing in one field is one step. What the page fills in by itself (the
conditions of new regions) is not a step and does not make the case
"unsaved". While you type in a text field, Ctrl+Z belongs to that
field (the browser's own undo, letter by letter); once you leave the
field, Ctrl+Z undoes the whole edit of the field.

1. **Geometry.** Pick a ready-made shape, load an SVG, load a mesh file
   (Gmsh `.msh` or `.pde2d`), or draw the domain (**Drawing**). You can
   also drop a file onto the picture. The regions of the boundary are
   listed with their lengths; **Edit** renames a region, merges it into
   another, or splits it in two (the boundary conditions follow), and
   **Undo** takes back the last edit.
   In an SVG, each `<path>` gives the region named by its `data-region`
   attribute or else its `id`; `app/examples/plate_with_hole.svg` is a
   small example.

   **Splitting a region** is done on the picture: the scissors tool
   (top left of the picture, or **S**), then a click on the boundary. A
   marker follows the pointer along the boundary and snaps to corners;
   the click splits the region there (exactly at a corner) and offers
   the two names right away: type a name and press Enter. This works for
   shapes, SVGs and drawings, and for mesh files at the nearest boundary
   node. A click on a region (with the Geometry step open) shows its name
   (type a new one and press Enter), **Merge into** and **Split…** right
   there.

   **Drawing.** **Edit as drawing** turns a shape or an SVG (with its
   region edits) into a drawing with exactly the same curves; choosing
   **Drawing** as the source starts from a blank rectangle. On the
   picture: drag a vertex (both of its edges follow), drag an outline or
   the ✥ handle to move a whole loop; click an edge to select it
   (Shift+click for more): the Geometry step then offers its kind (line,
   arc, quadratic or cubic Bézier, with handles to drag; the round
   handle in the middle of a line or an arc bends it into an arc), its
   region (an existing name or a new one: the way to split a region in a
   drawing), **Add vertex** and **Delete loop**. Double-click an edge to
   add a vertex there; a selected vertex has x and y fields, the arrow
   keys nudge it (Shift: ten steps) and Delete removes it. Without a
   mouse: with the picture focused, `]` and `[` step through the
   vertices and edges of a loop and L through the loops (a screen reader
   hears what is selected), and a selected edge can be split at a typed
   fraction of its length (**Split at … then [region] Split**). The tools add a
   rectangle (drag), a circle (drag from its centre) or a polygon (click
   its corners, close on the first or with Enter); inside the domain a new
   loop is a hole, outside it another piece. The grid button (**G**)
   snaps to a grid of about a fortieth of the domain. Escape cancels a
   drag, a polygon or a tool. Every drag is one undo step; the server
   reads the geometry once you let go. A geometry the library refuses
   keeps your edit: the bad edge is drawn red and dashed, and the step
   says why.
2. **Mesh.** Set the maximum edge length (absolute, or a fraction of the
   domain size) and pick the components: boundary discretizer, initial
   triangulator, refiner, and a list of improvers, each with its options.
   **Mesh** runs the mesher and shows the counts, the smallest angle,
   whether the target was met, and every stage's report. Export the mesh as
   `.pde2d` (it opens in `tools/mesh_viewer`) or as an SVG picture. A component's option that the library refuses is
   marked at its field.
3. **Problem.** Write the equation as one line, the way it is written on
   paper: `u_t - div(k*grad(u)) = 1`, or with nabla `u_t - ∇·(k∇u) = 1`
   (the ∇ · ² Δ buttons beside the field insert the symbols at the
   caret); u and its derivatives u_x, u_y, u_xx, u_xy, u_yy, u_t, u_tt,
   grad, div, lap and dot, vectors `(a, b)` and matrices
   `[[a, b], [c, d]]`, with coefficients in x, y, t and named parameters.
   Under the line, "means" shows the coefficients the library read from
   it (`a_xx = -k`, `b_x = …`); an error shows there once you pause
   typing. Each boundary condition is a line too (`u = 0`, `u_n = 0`,
   `k*u_n + h*u = h*u_out`, `(k ∇u)·n = q`: n is the outward normal, tau
   the tangent). **Coefficients** switches a line to the coefficient form
   (a_xx u_xx + … = f) and **As a line** back; there each form shows the
   coefficients in use and the right side, and **Show all coefficients**
   reveals the others. **Preset…** fills in a whole problem for the current
   geometry with matching solve settings (**Use a preset…** in the step's head): Poisson, Laplace with u = x,
   heat (a bump, a heated boundary, cooling), the wave equation with and
   without damping, steady convection-diffusion and reaction-diffusion;
   the message that follows has **Undo**. A preset is not applied when the
   case changed while it was prepared (choose it again). The **Start from…** and
   **Type…** menus fill in common equations and conditions. Every edit is
   checked at once (see Settings): errors appear under their field, and
   each coefficient field shows what it depends on. Each condition
   shows its kind (Dirichlet, natural, dynamic), and the summary shows
   the time order and which initial conditions are needed. Nothing above
   the fields changes while a check runs; a check that takes longer than
   0.4 s says "Checking…" and greys out the old facts. Renaming a parameter to a name that exists is refused.
4. **Assemble.** Builds the P1 system at a time t, meshing first if
   needed: the number of unknowns and Dirichlet nodes, which operators are
   zero, change with t or are symmetric (always, or only at this t), and a
   picture of where the stiffness matrix has entries. **Time terms to
   build** leaves out u_tt or every time term, as a static solve does. **Show a field on the mesh** draws an expression, the initial
   values, the Dirichlet values or the assembled right-hand side.
   **Export A/B/M2/b .mtx** saves the assembled system at that t in
   Matrix Market form (the free part: unknowns × unknowns). An error of
   the problem shown here has a button that opens the Problem step.
5. **Solve.** **Static** solves A U = b once at a time t (time terms are
   dropped; a note says so when the problem has them). **In time**
   integrates from t0 to t1 with a time method (implicit Euler, implicit
   midpoint, Newmark), a step dt and a number of snapshots. Pick the linear
   solver for either (sparse LU, simplicial LDLT, conjugate gradient,
   BiCGSTAB); chips show what each needs or supports, and the library says
   when a choice does not fit the problem. **Solve** meshes first if
   needed, then shows the counts, min u and max u, the steps and the
   solver's iterations and residual, and draws the solution. For a run
   in time, the time bar under the picture (**Play**, the playback speed,
   the slider, the shown time with that snapshot's min and max) walks
   through the snapshots: at 1× one simulated second takes one second,
   whatever the number of steps. A run plays from its start while it is
   computed, once a few snapshots are there, and no faster than the run
   computes (the bar then says the speed it plays at); the colours keep one scale for every time. **Compare with
   an exact solution** takes an expression in x, y, t: it shows the
   largest nodal error max |u − exact|, its RMS over the nodes and the
   lumped L2 error at the shown time, the largest over all snapshots and,
   for a run in time, the three per snapshot in a chart; it can show
   u − exact in the picture. The server computes the figures and the
   difference in double precision; only the picture is drawn from 32-bit
   floats. Editing the expression greys the figures out ("for the
   previous expression") and the picture goes back to u, saying why,
   until you compare again. **Refinement study** (with an exact solution
   written): solves on 3 to 5 meshes, halving the max edge length of the
   Mesh step each time, and shows h, the nodes, the largest nodal and L2
   errors and the observed order between levels, with a log–log chart
   and h¹, h² reference slopes; the mesh setting is put back afterwards
   and **Stop study** ends it. **Compare with a reference
   run**: **Keep as reference** keeps a finished run on the server;
   change the case (dt, the method, a coefficient), solve again and
   **Compare with reference** shows the largest difference per snapshot
   and can show u − reference in the picture. Both runs must be on the
   same mesh: after a change of the geometry or the mesh the server
   refuses and says so. **Forget reference** drops it. **Export .csv**
   saves x, y and u at every kept time (full precision), **Export .vtk**
   the mesh and every snapshot for ParaView, **Export VTK series** one
   file per snapshot in a zip that ParaView plays in time, **Export
   provenance** the whole case with the versions and the run, once the run is done; an
   export that is not ready, or a server that is busy, says so in plain
   words. Before a large run, the panel says how much of the server's
   memory budget for runs it will keep (the server refuses a run that
   does not fit) and how much the page holds. A run in time shows its
   snapshots while it goes on: the status shows t reached, steps and
   snapshots with a progress bar, and the picture the newest snapshot,
   unless you moved the time bar. **Stop** (in place of Solve while it
   runs) keeps what arrived; **Continue** goes on from there.
   **Adaptive** solves statically at t, estimates the error of every
   triangle (residual, Kelly or gradient-recovery estimator), marks some
   (Dörfler, maximum, fixed fraction, equidistribution), refines them
   (newest vertex bisection by default, or any refiner of the Mesh step)
   and solves again, until the estimate is below the tolerance (absolute,
   or relative to the first estimate) or the maximum cycles or nodes are
   reached. The picture then shows the solution on the last mesh, and
   the **Colour** menu offers **error indicator η** (log₁₀, per
   triangle). **Adaptive cycles** lists each cycle's nodes N, estimate and
   observed rate (e ~ N^−rate), with a log–log chart and an N^(−1/2)
   slope; comparing with an exact solution adds each cycle's L2 error
   (and an N^(−1) slope). **Export adapted mesh** saves the last mesh as
   `.pde2d`, which opens as a mesh file. An adaptive solve runs in one
   piece: it has no progress and no Stop.
   **Adaptive in time** integrates like **In time** (its settings are
   shown first) and, every so much time or so many steps (**Adapt
   every**), estimates the error, refines the marked triangles and moves
   the solution to the new mesh by interpolation, until **Maximum nodes**
   or **Maximum refinements**; with a **Tolerance** it refines only when
   the estimate is above it. The mesh only grows (no coarsening yet).
   **Damping steps** take that many implicit Euler steps after each
   transfer, which methods that do not damp (midpoint, Newmark) need.
   Each snapshot is drawn on its own mesh as the time bar moves;
   **Adaptation times** lists each time with its nodes, estimate and what
   happened, with a chart of the nodes against t
   (`app/examples/l_shape_heat_adaptive.json`). It runs in one piece too,
   and exports as the VTK series (a file per snapshot) and the last mesh,
   not as one CSV or VTK file.
   **Linear solver** offers the direct solvers sparse LU, simplicial LDLᵀ,
   supernodal Cholesky and multifrontal LU (the last two parallel, for
   large problems) and the iterative conjugate gradient and BiCGSTAB;
   **Threads** (1: one thread, 0: every processor) is an option of each
   solver that has one and of **Assembly** (folded under the linear
   solver). Assembly and the iterative solvers give the same numbers for
   every thread count; the parallel direct solvers may differ in the last
   digits.
   Changing the case stops a run. If the server is busy or does not
   answer for a moment, the page keeps asking (and says so); a snapshot,
   a difference or u(t) that could not be loaded says why beside it, with
   **Retry**.

**Run all** runs geometry, mesh, assembly and the solve in turn, skipping
the steps that are up to date, and ends with the picture coloured by the
solution. When a step fails, it opens with a message. Exports are offered
only for results that are up to date.

Every card and section folds: click its title (or press Enter or Space
on it). The browser remembers which ones you folded; rarely changed ones
(the boundary discretizer, the initial triangulator, the linear solver,
reports) start folded.

**Settings** (the gear in the top bar; Escape closes it) are kept in this
browser, not in the case: the problem check while typing (on or off),
whether a run plays as it is computed (the playback speed is chosen on
the time bar and kept too), the significant digits shown, the number of isolines, the
colour maps (for values of one sign: blue, orange or green; for both
signs: blue–red or blue–orange), and the picture quality (lower is faster to pan and draw, higher is sharper). Threads are
not here: they belong to the case (the Solve step's linear solver and
Assembly).

In the picture, drag to pan, use the wheel or pinch with two fingers to
zoom, and double-click (or the **⤢** button) to fit; **+** and **−** at
the picture's corner zoom too. With the picture focused (Tab), the arrow
keys pan (Shift for bigger steps), + and - zoom, 0 fits and Enter pins a
probe at the centre. The toolbar turns layers on and off (boundary, mesh
edges, Dirichlet nodes, isolines) and chooses the colouring: the
smallest angle of each triangle, the field shown (the menu names it:
`field: u`, `field: Dirichlet values`…), or **|∇|** of that field, one
value per triangle. The legend names the field and its time, with round
values under the colour bar; a field with one value says "constant: …".
A result the case no longer matches is drawn greyed with an "Outdated"
stamp. Click a boundary (or a region in the legend) to select it: its
boundary condition opens in the Problem step (in the Geometry step, its
row and its actions at the click). **Lock range** keeps the colour range for
every snapshot and run until **Unlock** (its ends can be edited), so two
runs can be compared by eye. **Line profile**: drag a line across the
picture to plot the shown field along it (it follows the time bar);
**Remove line** drops it. Point at the picture to read the coordinates,
the region, the triangle and the value there (for Dirichlet values and
b, the value of the nearest dot). **Save picture** saves the picture with
its legend as a PNG named after the case and the shown time.

**Probes.** Click or tap the picture to pin a probe (up to four). The list under
the picture gives each probe's x, y and value; for a run in time a chart
shows u(t) at every probe (computed on the server over all snapshots),
with the shown time marked; click the chart to show that time. The value
of u in the list is the server's, in double. **✕** removes a probe. Every
chart can be read with the keyboard (Tab to it, then the arrow keys) and
offers **Download data (CSV)**.

The page keeps the snapshots around the shown one, not all of them: a
long run with a large mesh does not fill the browser's memory. When you
jump far, the picture shows the nearest loaded snapshot for a moment
("loading t = ..."). Each browser tab has its own run: two tabs do not
replace or stop each other's solve. A reload of the tab shows its run
again if the server still has it, with the probes and the picture's
settings it had. While the server works for another tab (or a long
step), the top bar says what it is busy with.

The page has one, dark, theme. On a phone, the steps stay at the top and
the picture with its time bar stays under them while the panel scrolls;
drag the bar under the picture to make it taller or shorter, or fold the
picture away with **▴** (the browser remembers both).

## Cases

Everything you set up is one **case**: a JSON file with the geometry
(SVG and mesh texts included), the mesh settings, the problem and the
assembly time. **Save case** downloads it under the name of the file or
example it was opened from (`case.json` for a new case), and **Open
case…** loads one (both in the **Case** menu, which names the case);
"unsaved" next to it shows that the case changed since it was opened or
saved. The browser also keeps the current case between visits. The
examples at the end of the **Case** menu open the cases in
`app/examples/`. After **New case**, **Open case…** or an example,
**Restore previous case** in the menu brings back the case they replaced (until
the next replacement). An example or file that is still loading never
replaces a case you started or opened meanwhile. When
parts of an opened case had to be replaced by defaults, a notice above
the steps lists them until you dismiss it.

A case also runs without the browser:

```sh
build/app/pde2d_app --run app/examples/heat_annulus.json --output /tmp/heat
```

This prints a JSON summary of every stage, the solve included, and, with
`--output`, writes there `summary.json` (the same summary), `mesh.pde2d`,
`mesh.svg`, `solution.csv`, `solution.vtk` (one file, one array per kept
time), the time series `solution_0000.vtk`, `solution_0001.vtk`, … with
`solution.vtk.series` (open the `.series` file in ParaView: it plays the
snapshots with their times; the page's export is the same as one zip,
`solution-vtk.zip`), `provenance.json` (the whole case, its SHA-256, the
app's version and git commit, and the run's report) and the assembled
system (`stiffness.mtx`, `damping.mtx`, `mass.mtx`, `rhs.mtx`); an
adaptive solve (`app/examples/l_shape_adaptive.json`) also writes its
last mesh, `solution-mesh.pde2d`, and its summary lists every cycle
(`solve.adaptive.cycles`) and, with an exact solution, each cycle's error
(`comparison.cycles`). An adaptive run in time
(`app/examples/l_shape_heat_adaptive.json`) writes the last mesh and the
VTK series but no `solution.csv` or `solution.vtk` (its snapshots are on
several meshes), and its summary lists the adaptation times
(`solve.adaptive_time.adaptations`). With an exact solution the summary's
`comparison` also gives the last snapshot's error (`comparison.final`).
Each
file but `.pde2d` says what produced it where its format allows: `#`
lines at the top of the CSV (read it with
`pandas.read_csv("solution.csv", comment="#")`), `%` lines after the
Matrix Market banner, an XML comment in the SVGs, and in the VTK files
the title line (their second line: legacy VTK has no comments).
`.pde2d` has none (its format stays the old one). The exit code is 0
when every stage ran. `pde2d_app --default-case` prints a case to start
from.

`--set PATH=VALUE` (repeatable) changes one value of the case before it
runs: PATH is dotted keys with `[i]` for list items, VALUE is JSON or else
text, and a path that does not exist is an error naming it.

```sh
build/app/pde2d_app --run app/examples/heat_annulus.json \
  --set mesh.spec.max_edge_length.relative=0.02 \
  --set solve.time.method.type=implicit_midpoint
```

When the case has an exact solution (`view.exact_solution`, set by the
Solve step's comparison), the summary also has `comparison`: the largest
max |u − exact|, RMS and L2 error over the snapshots, the number of nodes
and the longest edge.

**A convergence study** is one `--sweep PATH=V1,V2,...` (repeatable: every
combination runs, the first `--sweep` varying slowest; one session, so
what a combination does not change is not built again):

```sh
build/app/pde2d_app --run app/examples/l_shape_corner.json \
  --sweep mesh.spec.max_edge_length.relative=0.08,0.04,0.02
```

```
mesh.spec.max_edge_length.relative    ok  nodes  longest_edge  solve_seconds  max_abs    max_l2  rate_max  rate_l2
                              0.08  true    165        0.2256       0.000253   0.0249  0.007104         -        -
                              0.04  true    648         0.113       0.001137  0.01416  0.003112    0.8165    1.194
                              0.02  true   2452       0.05655       0.004727  0.01209  0.001249    0.2282    1.318
```

`rate_l2` and `rate_max` are the observed orders log(e₁/e₂) / log(h₁/h₂)
between a row and the one before it, with h the longest edge. The
L-shaped example's L2 error falls about like h^(4/3), the rate its
corner singularity allows (h^2 for a smooth solution); its nodal maximum
error, at the corner, falls much more slowly. With `--output DIR` the
sweep is also written as `sweep.json` and `sweep.csv` (for a plot). A
sweep over a time step or a method works the same way
(`--sweep solve.time.step.options.dt=0.02,0.01,0.005`).

## Tests

`ctest --test-dir build -R app_` runs the app's tests: the stages and
the catalog (`app_session`), the Solve step against exact solutions
(`app_solve`), runs in chunks and the presets (`app_runs`), comparisons,
reference runs, probes and clients (`app_compare`), exports, provenance,
error paths, the VTK series zip, `--set` and `--sweep` (`app_exports`),
the HTTP API with its token, Origin, 429 and status checks
(`app_server`), drawings, the boundary locator and the split of a mesh
file's region (`app_drawing`), every example case on the command line (`app_run_*`;
`app_check_*` compares each one's figures with the bounds in
`app/tests/example_checks.json`; `app_run_set*` for `--set`,
`app_run_sweep` for `--sweep`) and the page's code in Node.js
(`app_web`, `app_web_actions`; skipped without Node), and the page in
headless Chrome (`app_browser`; skipped without Chrome: set
`PDE2D_CHROME`, or have `google-chrome`/`chromium` on PATH, or the
Windows Chrome under WSL). `app/tests/browser/run.sh --shots DIR`
runs those scenarios by hand and saves a screenshot of each.
