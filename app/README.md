# pde2d app

A graphical interface for the pde2d library: choose a domain, mesh it,
write the problem and assemble it, and look at the results, in a browser.
It is a separate program on top of the library; the library never depends
on it. How it is built is described in [docs/app.md](../docs/app.md).

## Start it

**In Windows, double-click `app/Start pde2d app.cmd`** (in Explorer, the
checkout is under `\\wsl.localhost\Ubuntu\...`; a desktop shortcut to the
file works the same). A window opens, builds the app if something
changed, starts it and opens it in your browser. Closing that window
stops the app. Double-clicking again while it runs just opens the page.
In Linux or WSL, `app/start-app.sh` does the same.

The first start needs a configured build directory (`build/`; run
`tools/test.sh` once, see the root README). After that, the launcher
rebuilds only what changed.

By hand, from the project root, in Linux or WSL:

```sh
tools/test.sh                 # builds everything, the app included
build/app/pde2d_app           # prints: open http://127.0.0.1:8765/
```

Open the printed address in a browser. Under WSL, the Windows browser
reaches it at the same address. Stop the app with Ctrl+C.

- `--port 9000` picks another port (if 8765 is taken).
- `--host 0.0.0.0` makes it reachable from other machines on the network.
  There is no login, so do this only on a network you trust.
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
previous input". A number field whose text is not a number yet (`-`,
`1e`) is marked "not a number" and changes nothing until it parses.

1. **Geometry.** Pick a ready-made shape, load an SVG drawing, or load a
   mesh file (Gmsh `.msh` or `.pde2d`). You can also drop a file onto the
   picture. The regions of the boundary are listed with their lengths;
   **Edit** renames a region, merges it into another, or splits it in two
   (the boundary conditions follow), and **Undo** takes back the last
   edit. The regions of a mesh file can be renamed and merged, not
   split.
   In an SVG, each `<path>` gives the region named by its `data-region`
   attribute or else its `id`; `app/examples/plate_with_hole.svg` is a
   small example.
2. **Mesh.** Set the maximum edge length (absolute, or a fraction of the
   domain size) and pick the components: boundary discretizer, initial
   triangulator, refiner, and a list of improvers, each with its options.
   **Mesh** runs the mesher and shows the counts, the smallest angle,
   whether the target was met, and every stage's report. Export the mesh as
   `.pde2d` (it opens in `tools/mesh_viewer`) or as an SVG picture. A component's option that the library refuses is
   marked at its field.
3. **Problem.** Write the equation's coefficients, named parameters, one
   boundary condition per region and the initial conditions as expressions
   in x, y and t. **Preset…** fills in a whole problem for the current
   geometry with matching solve settings (**Use a preset…** in the step's head): Poisson, Laplace with u = x,
   heat (a bump, a heated boundary, cooling), the wave equation with and
   without damping, steady convection-diffusion and reaction-diffusion;
   the message that follows has **Undo**. The **Start from…** and
   **Type…** menus fill in common equations and conditions. Every edit is
   checked after a short pause in typing (see Settings): errors appear
   under their field, and each field shows what it depends on. Each condition
   shows its kind (Dirichlet, natural, dynamic), and the summary shows
   the time order and which initial conditions are needed. While an edit
   waits for its check, the summary says "Checking…" and the old facts
   are greyed out. Renaming a parameter to a name that exists is refused.
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
   in time, the time bar under the picture (**Play**, the slider, the
   shown time with that snapshot's min and max) walks through the
   snapshots; the colours keep one scale for every time. **Compare with
   an exact solution** takes an expression in x, y, t: it shows
   max |u − exact| at the shown time and over all snapshots, a chart of
   it per snapshot, and can show u − exact in the picture. **Export solution .csv** saves
   x, y and u at every kept time (full precision; the picture gets the
   values as 32-bit floats). Before a run, the panel says how large the
   solution will be and warns above 50 MB. A run in time shows its
   snapshots while it goes on: the status shows t reached, steps and
   snapshots with a progress bar, and the picture the newest snapshot,
   unless you moved the time bar. **Stop** (in place of Solve while it
   runs) keeps what arrived; **Continue** goes on from there.
   Changing the case stops a run.

**Run all** runs geometry, mesh, assembly and the solve in turn, skipping
the steps that are up to date, and ends with the picture coloured by the
solution. When a step fails, it opens with a message. Exports are offered
only for results that are up to date.

Every card and section folds: click its title (or press Enter or Space
on it). The browser remembers which ones you folded; rarely changed ones
(the boundary discretizer, the initial triangulator, the linear solver,
reports) start folded.

**Settings** (the gear in the top bar; Escape closes it) are kept in this
browser, not in the case: the problem check while typing (on or off, and
its delay), the playback speed, whether a run's newest snapshot is
followed, the significant digits shown, the number of isolines, and the
picture quality (lower is faster to pan and draw, higher is sharper). The Computation group
(sequential or parallel assembly and linear solves) is shown disabled: the
library runs sequentially for now.

In the picture, drag to pan, use the wheel to zoom, and double-click (or
press **Fit**) to fit. With the picture focused (Tab), the arrow keys pan
(Shift for bigger steps), + and - zoom, 0 fits and Enter pins a probe at
the centre. The toolbar turns layers on and off (boundary, mesh edges,
Dirichlet nodes, isolines) and chooses the colouring: the smallest angle
of each triangle, or the last field shown. Point at the picture to read
the coordinates, the region, the triangle and the field value there (for
Dirichlet values and b, the value of the nearest dot). **Save picture**
saves the picture with its legend as a PNG named after the case and the
shown time.

**Probes.** Click the picture to pin a probe (up to four). The list under
the picture gives each probe's x, y and value; for a run in time a chart
shows u(t) at every probe, with the shown time marked; click the chart to
show that time. **✕** removes a probe.

The page has one, dark, theme. On a phone, the steps stay at the top and
the picture with its time bar stays under them while the panel scrolls.

## Cases

Everything you set up is one **case**: a JSON file with the geometry
(SVG and mesh texts included), the mesh settings, the problem and the
assembly time. **Save case** downloads it under the name of the file or
example it was opened from (`case.json` for a new case), and **Open
case…** loads one; "unsaved changes" next to Save shows that the case
changed since it was opened or saved. The browser also keeps the current
case between visits. The **Examples** menu opens the cases in
`app/examples/`. After **New**, **Open case…** or **Examples**, the
message offers **Restore the previous case** for a few seconds. When
parts of an opened case had to be replaced by defaults, a notice above
the steps lists them until you dismiss it.

A case also runs without the browser:

```sh
build/app/pde2d_app --run app/examples/heat_annulus.json --output /tmp/heat
```

This prints a JSON summary of every stage, the solve included, and, with
`--output`, writes `mesh.pde2d`, `mesh.svg`, `solution.csv` and the
assembled system (`stiffness.mtx`, `damping.mtx`, `mass.mtx`, `rhs.mtx`).
The exit code is 0 when every stage ran.
`pde2d_app --default-case` prints a case to start from.

## Tests

`ctest --test-dir build -R app_` runs the app's tests: the stages and
the catalog (`app_session`), the Solve step against exact solutions
(`app_solve`), runs in chunks and the presets (`app_runs`), the HTTP API
(`app_server`), every example
case on the command line (`app_run_*`) and the page's code in Node.js
(`app_web`, `app_web_actions`; skipped without Node), and the page in
headless Chrome (`app_browser`; skipped without Chrome: set
`PDE2D_CHROME`, or have `google-chrome`/`chromium` on PATH, or the
Windows Chrome under WSL). `app/tests/browser/run.sh --shots DIR`
runs those scenarios by hand and saves a screenshot of each.
