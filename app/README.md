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
outdated because the case changed since (yellow), or failed (red).

1. **Geometry.** Pick a ready-made shape, load an SVG drawing, or load a
   mesh file (Gmsh `.msh` or `.pde2d`). You can also drop a file onto the
   picture. The regions of the boundary are listed with their lengths;
   **Edit** renames a region, merges it into another, or splits it in two
   (the boundary conditions follow), and **Undo** takes back the last
   edit.
   In an SVG, each `<path>` gives the region named by its `data-region`
   attribute or else its `id`; `app/examples/plate_with_hole.svg` is a
   small example.
2. **Mesh.** Set the maximum edge length (absolute, or a fraction of the
   domain size) and pick the components: boundary discretizer, initial
   triangulator, refiner, and a list of improvers, each with its options.
   **Mesh** runs the mesher and shows the counts, the smallest angle,
   whether the target was met, and every stage's report. Export the mesh as
   `.pde2d` (it opens in `tools/mesh_viewer`) or as an SVG picture.
3. **Problem.** Write the equation's coefficients, named parameters, one
   boundary condition per region and the initial conditions as expressions
   in x, y and t. The **Start from…** and **Type…** menus fill in common
   equations and conditions. Every edit is checked: errors appear under
   their field, and each field shows what it depends on. Each condition
   shows its kind (Dirichlet, natural, dynamic), and the summary shows
   the time order and which initial conditions are needed.
4. **Assemble.** Builds the P1 system at a time t, meshing first if
   needed: the number of unknowns and Dirichlet nodes, which operators are
   zero, change with t or are symmetric (always, or only at this t), and a
   picture of where the stiffness matrix has entries. **Time terms to
   build** leaves out u_tt or every time term, as a static solve does. **Show a field on the mesh** draws an expression, the initial
   values, the Dirichlet values or the assembled right-hand side.
5. **Solve.** **Static** solves A U = b once at a time t (time terms are
   dropped; a note says so when the problem has them). **In time**
   integrates from t0 to t1 with a time method (implicit Euler, implicit
   midpoint, Newmark), a step dt and a number of snapshots. Pick the linear
   solver for either (sparse LU, simplicial LDLT, conjugate gradient,
   BiCGSTAB); chips show what each needs or supports, and the library says
   when a choice does not fit the problem. **Solve** meshes first if
   needed, then shows the counts, the range of u, the steps and the
   solver's iterations and residual, and draws the solution. For a run
   in time, the slider and **Play** walk through the snapshots; the
   colours keep one scale for every time. **Export solution .csv** saves
   x, y and u at every kept time. A long run keeps the app busy until it
   ends.

**Run all** runs geometry, mesh, assembly and the solve in turn, skipping
the steps that are up to date.

In the picture, drag to pan, use the wheel to zoom, and double-click (or
press **Fit**) to fit. The toolbar turns layers on and off and chooses the
colouring: the smallest angle of each triangle, or the last field shown.
Point at the picture to read the coordinates, the region, the triangle
and the field value there.

## Cases

Everything you set up is one **case**: a JSON file with the geometry
(SVG and mesh texts included), the mesh settings, the problem and the
assembly time. **Save case** downloads it and **Open case…** loads one;
the browser also keeps the current case between visits. The
**Examples** menu opens the cases in `app/examples/`.

A case also runs without the browser:

```sh
build/app/pde2d_app --run app/examples/heat_annulus.json --output /tmp/heat
```

This prints a JSON summary of every stage, the solve included, and, with
`--output`, writes `mesh.pde2d`, `mesh.svg` and `solution.csv`. The exit code is 0 when every stage ran.
`pde2d_app --default-case` prints a case to start from.

## Tests

`ctest --test-dir build -R app_` runs the app's tests: the stages and
the catalog (`app_session`), the Solve step against exact solutions
(`app_solve`), the HTTP API (`app_server`), every example
case on the command line (`app_run_*`) and the page's code in Node.js
(`app_web`, `app_web_actions`; skipped without Node).
