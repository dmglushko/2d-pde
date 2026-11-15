// Tests of the page's pure helpers (app/web/js/core.js), without a
// browser. Run by CTest as app_web when Node.js is installed.
import assert from "node:assert/strict";
import {
  barycentric,
  errorPath,
  fieldScale,
  fileKind,
  formatNumber,
  oklabToRgb,
  rampTable,
  regionToken,
  rgbToOklab,
  smallestAngle,
  stageKey,
  applyRegionEdit,
  normalizeCase,
  reconcileConditions,
  undoRegionEdit,
  triangleAngles,
  STAGES,
  STEPS,
  RUN_ALL,
  stagesToRun,
  solutionBytes,
  LARGE_SOLUTION_BYTES,
  splitSnapshots,
  parseNumberText,
  renameParameter,
} from "../web/js/core.js";

const cases = [];
const test = (name, body) => cases.push([name, body]);

test("OKLab round trip", () => {
  for (const rgb of [[0, 0, 0], [255, 255, 255], [42, 120, 214], [235, 104, 52]]) {
    const back = oklabToRgb(rgbToOklab(rgb));
    back.forEach((v, i) => assert.ok(Math.abs(v - rgb[i]) <= 1, `${rgb} -> ${back}`));
  }
});

test("a ramp starts and ends at its stops", () => {
  const table = rampTable([[205, 226, 251], [13, 54, 107]]);
  assert.equal(table.length, 256 * 3);
  const near = (actual, expected) => actual.every((v, i) => Math.abs(v - expected[i]) <= 1);
  assert.ok(near([...table.slice(0, 3)], [205, 226, 251]), `first ${table.slice(0, 3)}`);
  assert.ok(near([...table.slice(255 * 3)], [13, 54, 107]), `last ${table.slice(255 * 3)}`);
});

test("field scales: sequential, diverging around zero, constant, empty", () => {
  const seq = fieldScale([1, 2, null, 3]);
  assert.equal(seq.kind, "sequential");
  assert.equal(seq.position(1), 0);
  assert.equal(seq.position(3), 1);
  const div = fieldScale([-1, 0.5, 4]);
  assert.equal(div.kind, "diverging");
  assert.equal(div.position(0), 0.5, "zero is the neutral middle");
  assert.equal(div.position(4), 1);
  assert.equal(div.position(-4), 0);
  assert.equal(fieldScale([2, 2]).kind, "constant");
  assert.equal(fieldScale([null, NaN]).kind, "empty");
});

test("regions take the categorical slots in order, then one neutral colour", () => {
  assert.equal(regionToken(0), "--series-1");
  assert.equal(regionToken(7), "--series-8");
  assert.equal(regionToken(8), "--series-other");
  assert.equal(regionToken(20), "--series-other");
});

test("smallest angles", () => {
  assert.ok(Math.abs(smallestAngle(0, 0, 1, 0, 0.5, Math.sqrt(3) / 2) - 60) < 1e-12);
  assert.ok(Math.abs(smallestAngle(0, 0, 1, 0, 0, 1) - 45) < 1e-12);
  const angles = triangleAngles({ nodes: [0, 0, 1, 0, 0, 1, 1, 1], triangles: [0, 1, 2, 1, 3, 2] });
  assert.equal(angles.length, 2);
  assert.ok(angles.every((a) => Math.abs(a - 45) < 1e-12));
});

test("barycentric coordinates", () => {
  const w = barycentric(0.25, 0.25, 0, 0, 1, 0, 0, 1);
  assert.deepEqual(w.map((v) => Math.round(v * 1e12) / 1e12), [0.5, 0.25, 0.25]);
  assert.equal(barycentric(0, 0, 0, 0, 1, 1, 2, 2), null);
});

test("regions without a condition get one; vanished ones are archived", () => {
  const dirichlet = { alpha: "1", g: "0" };
  const neumann = { beta_n: "1", g: "0" };
  const base = { problem: { boundary_conditions: { left: neumann, right: dirichlet, gone: { beta_n: "2" } } } };
  let next = reconcileConditions(base, ["left", "right", "top"], dirichlet);
  assert.deepEqual(Object.keys(next.problem.boundary_conditions), ["left", "right", "top"]);
  assert.deepEqual(next.problem.boundary_conditions.top, dirichlet);
  assert.deepEqual(next.view.condition_archive.gone, { beta_n: "2" });
  assert.ok("gone" in base.problem.boundary_conditions, "the input is not changed");
  next = reconcileConditions(next, ["left", "right", "gone"], dirichlet);
  assert.deepEqual(next.problem.boundary_conditions.gone, { beta_n: "2" }, "restored with its region");
  assert.deepEqual(Object.keys(next.view.condition_archive), ["top"]);
  next = reconcileConditions(next, ["left", "right", "gone", "top"], dirichlet);
  assert.equal(next.view, undefined, "an empty archive is removed");
  next = reconcileConditions({ problem: { boundary_conditions: {} }, view: { condition_archive: { left: null } } }, ["left"], dirichlet);
  assert.deepEqual(next.problem.boundary_conditions.left, dirichlet, "an archived entry that is not a condition is never restored");
});

const g = (caseData, name) => caseData.problem.boundary_conditions[name]?.g;
const names = (caseData) => Object.keys(caseData.problem.boundary_conditions).sort();
const plate = () => ({ geometry: { source: "shape", region_edits: [] }, problem: { boundary_conditions: { bottom: { alpha: "1", g: "11" }, top: { alpha: "1", g: "22" }, side: { beta_n: "1", g: "0" } } } });

test("region edits move conditions and undo restores them exactly", () => {
  const start = plate();
  let c = applyRegionEdit(start, { op: "rename", from: "bottom", to: "floor" });
  assert.equal(g(c, "floor"), "11");
  assert.deepEqual(names(c), ["floor", "side", "top"]);
  c = applyRegionEdit(c, { op: "rename", from: "top", to: "bottom" });
  assert.equal(g(c, "bottom"), "22", "a freed name can be reused");
  c = undoRegionEdit(c);
  assert.deepEqual(names(c), ["floor", "side", "top"]);
  assert.equal(g(c, "top"), "22");
  c = undoRegionEdit(c);
  assert.deepEqual(names(c), ["bottom", "side", "top"]);
  assert.equal(g(c, "bottom"), "11", "the original bottom, not the reused one");
  assert.equal(g(c, "top"), "22");
  assert.deepEqual(c.geometry.region_edits, []);
  assert.equal(c.view, undefined, "no undo records left");
  assert.equal(g(start, "bottom"), "11", "the input is not changed");
});

test("split, merge, and an edit the library refuses", () => {
  let c = applyRegionEdit(plate(), { op: "split", region: "side", fraction: 0.5, first: "side", second: "side_b" });
  assert.deepEqual(c.problem.boundary_conditions.side_b, { beta_n: "1", g: "0" });
  assert.deepEqual(c.problem.boundary_conditions.side, { beta_n: "1", g: "0" });
  c = applyRegionEdit(c, { op: "merge", source: "side_b", target: "top" });
  assert.deepEqual(names(c), ["bottom", "side", "top"]);
  c = undoRegionEdit(undoRegionEdit(c));
  assert.deepEqual(names(c), ["bottom", "side", "top"]);
  // Rename onto an existing name (the library refuses it): undo restores both.
  c = applyRegionEdit(plate(), { op: "rename", from: "bottom", to: "top" });
  assert.equal(g(c, "top"), "11");
  c = undoRegionEdit(c);
  assert.equal(g(c, "bottom"), "11");
  assert.equal(g(c, "top"), "22");
});

test("edits survive saving: the case is consistent at every moment", () => {
  let c = applyRegionEdit(plate(), { op: "rename", from: "bottom", to: "floor" });
  c = JSON.parse(JSON.stringify(c)); // saved and opened again
  assert.equal(g(c, "floor"), "11");
  c = undoRegionEdit(c);
  assert.equal(g(c, "bottom"), "11", "undo works after a reload");
  // A hand-written case without undo records: the edit is only removed.
  const bare = { geometry: { region_edits: [{ op: "rename", from: "a", to: "b" }] }, problem: { boundary_conditions: { b: { g: "1" } } } };
  assert.deepEqual(undoRegionEdit(bare).geometry.region_edits, []);
});

test("region names that are names of object properties", () => {
  const neumann = { beta_n: "1", g: "0" };
  const base = { geometry: { region_edits: [] }, problem: { boundary_conditions: { left: neumann, right: { alpha: "1", g: "0" } } } };
  let c = applyRegionEdit(base, { op: "rename", from: "left", to: "constructor" });
  assert.deepEqual({ ...c.problem.boundary_conditions.constructor }, neumann);
  c = applyRegionEdit(base, { op: "rename", from: "left", to: "__proto__" });
  assert.ok(Object.hasOwn(c.problem.boundary_conditions, "__proto__"), "an own entry, not the prototype");
  const text = JSON.stringify(c);
  assert.ok(text.includes('"__proto__":{"beta_n"'), "saved like any other region");
  c = reconcileConditions(JSON.parse(text), ["__proto__", "toString", "right"], { alpha: "1", g: "0" });
  assert.deepEqual({ ...c.problem.boundary_conditions.__proto__ }, neumann, "and read back");
  assert.deepEqual({ ...c.problem.boundary_conditions.toString }, { alpha: "1", g: "0" });
  c = undoRegionEdit(applyRegionEdit(base, { op: "rename", from: "left", to: "__proto__" }));
  assert.deepEqual(Object.keys(c.problem.boundary_conditions).sort(), ["left", "right"]);
});

const SOLVE_DEFAULTS = {
  mode: "static",
  linear_solver: { type: "sparse_lu", options: {} },
  static: { t: 0 },
  time: { t0: 0, t1: 1, snapshots: 50, method: { type: "implicit_euler" }, step: { type: "fixed_step", options: { dt: 0.01 } } },
};

test("the solve section is normalized", () => {
  const defaults = { geometry: { source: "shape", shape: "rectangle" }, mesh: { spec: {}, discretizer: { type: "d" }, triangulator: { type: "t" }, refiner: null, improvers: [] }, problem: {}, assembly: { t: 0 }, solve: SOLVE_DEFAULTS };
  const base = { format: "pde2d-app-case", geometry: defaults.geometry, mesh: defaults.mesh, problem: {}, assembly: { t: 0 } };
  // A case from before the solvers: "solve": null, quietly given the defaults.
  const old = normalizeCase({ ...base, solve: null }, defaults);
  assert.deepEqual(old.replaced, []);
  assert.deepEqual(old.caseData.solve, SOLVE_DEFAULTS);
  assert.notEqual(old.caseData.solve, SOLVE_DEFAULTS, "a copy, not the catalog's object");
  const wrongMode = normalizeCase({ ...base, solve: { ...SOLVE_DEFAULTS, mode: "dynamic" } }, defaults);
  assert.deepEqual(wrongMode.replaced, ["solve"]);
  const broken = normalizeCase({ ...base, solve: { mode: "time", linear_solver: { type: "bicgstab" }, static: 3, time: { t0: 0, method: "x" } } }, defaults);
  assert.deepEqual(broken.replaced, ["solve.static", "solve.time.method", "solve.time.step"]);
  assert.equal(broken.caseData.solve.time.t0, 0, "the rest of time is kept");
  assert.equal(broken.caseData.solve.linear_solver.type, "bicgstab");
  // The solution depends on the solve section, not on the assembly time.
  const a = old.caseData;
  const b = { ...a, assembly: { t: 5 } };
  const c = { ...a, solve: { ...a.solve, mode: "time" } };
  assert.equal(stageKey(a, "solve"), stageKey(b, "solve"));
  assert.notEqual(stageKey(a, "solve"), stageKey(c, "solve"));
  assert.notEqual(stageKey(a, "assemble"), stageKey(b, "assemble"));
});

test("cases are normalized before they are shown", () => {
  const defaults = { geometry: { source: "shape", shape: "rectangle" }, mesh: { spec: {}, discretizer: { type: "d" }, triangulator: { type: "t" }, refiner: { type: "r" }, improvers: [] }, problem: { equation: {} }, assembly: { t: 0 }, solve: SOLVE_DEFAULTS };
  assert.throws(() => normalizeCase({ geometry: {} }, defaults), /not a pde2d app case/);
  assert.throws(() => normalizeCase(null, defaults), /not a pde2d app case/);
  const bare = normalizeCase({ format: "pde2d-app-case" }, defaults);
  assert.deepEqual(bare.replaced, ["geometry", "mesh", "problem", "assembly"]);
  assert.deepEqual(bare.caseData.mesh.discretizer, { type: "d" });
  const partial = normalizeCase({ format: "pde2d-app-case", geometry: { source: "svg", svg: "" }, mesh: { refiner: null, improvers: "x" }, problem: { equation: 3 }, assembly: { t: 2 } }, defaults);
  assert.deepEqual(partial.replaced, ["mesh.spec", "mesh.discretizer", "mesh.triangulator", "mesh.improvers", "problem.equation"]);
  assert.equal(partial.caseData.mesh.refiner, null, "no refiner is kept");
  assert.equal(partial.caseData.geometry.source, "svg");
  assert.equal(partial.caseData.assembly.t, 2);
  const nested = normalizeCase(
    {
      format: "pde2d-app-case",
      geometry: { source: "shape", shape: "rectangle", options: 7, region_edits: [1] },
      mesh: { spec: {}, discretizer: { type: "d", options: 3 }, triangulator: "cdt", refiner: { options: {} }, improvers: [{ type: "i" }, 5] },
      problem: { boundary_conditions: { left: 3, right: {} } },
      assembly: { t: 0 },
      view: { condition_archive: [] },
    },
    defaults,
  );
  const archived = normalizeCase({ format: "pde2d-app-case", view: { condition_archive: { bottom: null, top: { g: "1" } } } }, defaults);
  assert.ok(archived.replaced.includes("view.condition_archive.bottom"));
  const misaligned = normalizeCase({ format: "pde2d-app-case", geometry: { source: "shape", shape: "rectangle", region_edits: [] }, view: { region_edit_undo: [{}] } }, defaults);
  assert.ok(misaligned.replaced.includes("view.region_edit_undo"), "undo records must match the edits");
  assert.deepEqual(Object.keys(archived.caseData.view.condition_archive), ["top"]);
  assert.deepEqual(nested.replaced, [
    "geometry.options",
    "geometry.region_edits",
    "mesh.discretizer",
    "mesh.triangulator",
    "mesh.refiner",
    "mesh.improvers",
    "problem.boundary_conditions.left",
    "view.condition_archive",
  ]);
  assert.deepEqual(nested.caseData.geometry.options, {});
  nested.caseData.geometry.options.width = 3; // what editing a field does
});

test("error paths and stage keys", () => {
  assert.equal(errorPath("mesh.refiner.options.min_angle_degrees: expected a number"), "mesh.refiner.options.min_angle_degrees");
  assert.equal(errorPath("mesh.improvers[0].options: x"), "mesh.improvers[0].options");
  assert.equal(errorPath("Something without a path"), null);
  const a = { geometry: { s: 1 }, mesh: { m: 1 }, problem: { p: 1 } };
  const b = { ...a, problem: { p: 2 } };
  assert.equal(stageKey(a, "mesh"), stageKey(b, "mesh"), "the mesh does not depend on the problem");
  assert.notEqual(stageKey(a, "assemble"), stageKey(b, "assemble"));
});

test("file kinds and numbers", () => {
  assert.equal(fileKind("Plate.SVG"), "svg");
  assert.equal(fileKind("a.msh"), "gmsh");
  assert.equal(fileKind("a.pde2d"), "pde2d");
  assert.equal(fileKind("case.json"), "case");
  assert.equal(fileKind("a.txt"), null);
  assert.equal(formatNumber(1234), "1,234");
  assert.equal(formatNumber(0.123456), "0.1235");
  assert.equal(formatNumber(1e-6), "1.000e-6");
  assert.equal(formatNumber(null), "–");
});

test("the stage registry: steps, keys and prerequisites", () => {
  assert.deepEqual(STEPS, ["geometry", "mesh", "problem", "assemble", "solve"]);
  assert.deepEqual(RUN_ALL, ["geometry", "mesh", "assemble", "solve"]);
  for (const stage of STAGES) for (const id of stage.requires) assert.ok(STAGES.findIndex((s) => s.id === id) < STAGES.indexOf(stage), `${id} before ${stage.id}`);
  const a = { geometry: { s: 1 }, mesh: { m: 1 }, problem: { p: 1 }, assembly: { t: 0 }, solve: { mode: "static" } };
  assert.equal(stageKey(a, "field"), stageKey(a, "assemble"), "a field depends on what assembly depends on");
  assert.notEqual(stageKey(a, "solve"), stageKey({ ...a, solve: { mode: "time" } }, "solve"));
  assert.equal(stageKey(a, "solve"), stageKey({ ...a, assembly: { t: 3 } }, "solve"), "the solve does not depend on assembly.t");
  assert.throws(() => stageKey(a, "nope"));
  assert.deepEqual(stagesToRun("solve", () => false), ["geometry", "mesh", "solve"]);
  assert.deepEqual(stagesToRun("solve", (id) => id === "geometry"), ["mesh", "solve"]);
  assert.deepEqual(stagesToRun("assemble", () => true), ["assemble"], "the stage itself always runs");
  assert.deepEqual(stagesToRun("problem", () => false), ["problem"]);
});

test("solution sizes and binary snapshots", () => {
  assert.equal(solutionBytes({ mode: "static" }, 1000), 4000);
  assert.equal(solutionBytes({ mode: "time", time: { snapshots: 9 } }, 1000), 40000);
  assert.equal(solutionBytes({ mode: "time", time: {} }, 1000), null);
  assert.equal(solutionBytes({ mode: "static" }, undefined), null);
  assert.ok(solutionBytes({ mode: "time", time: { snapshots: 1000 } }, 2000) < LARGE_SOLUTION_BYTES, "8 MB is not large");
  assert.ok(solutionBytes({ mode: "time", time: { snapshots: 1000 } }, 20000) > LARGE_SOLUTION_BYTES, "80 MB is");
  const buffer = new ArrayBuffer(4 * 6);
  const view = new DataView(buffer);
  [0, 0.5, -1, 2, 3.25, NaN].forEach((v, i) => view.setFloat32(4 * i, v, true));
  const snapshots = splitSnapshots(buffer, 2, 3);
  assert.equal(snapshots.length, 2);
  assert.ok(snapshots[0] instanceof Float32Array);
  assert.deepEqual([...snapshots[0]], [0, 0.5, -1]);
  assert.deepEqual([...snapshots[1]].slice(0, 2), [2, 3.25]);
  assert.ok(Number.isNaN(snapshots[1][2]));
  assert.throws(() => splitSnapshots(buffer, 3, 3), /expected 36/);
});

test("number texts and parameter names", () => {
  assert.deepEqual(parseNumberText("2.5"), { kind: "number", value: 2.5 });
  assert.deepEqual(parseNumberText(" -1e-3 "), { kind: "number", value: -0.001 });
  assert.deepEqual(parseNumberText(""), { kind: "empty" });
  assert.deepEqual(parseNumberText("-"), { kind: "invalid" });
  assert.deepEqual(parseNumberText("1e"), { kind: "invalid" });
  assert.deepEqual(parseNumberText("", true), { kind: "invalid" }, "a number input reports '' for '1e'");
  assert.deepEqual(parseNumberText("Infinity"), { kind: "invalid" });
  const parameters = { k: 1, c: 2 };
  assert.deepEqual(Object.entries(renameParameter(parameters, "k", "kappa")), [["kappa", 1], ["c", 2]], "order kept");
  assert.deepEqual(renameParameter(parameters, "k", " k "), parameters);
  assert.throws(() => renameParameter(parameters, "k", "c"), /a parameter named c already exists/);
  assert.throws(() => renameParameter(parameters, "k", "  "), /needs a name/);
  assert.deepEqual(parameters, { k: 1, c: 2 }, "the input is not changed");
});

test("a mesh file keeps its region edits through normalization", () => {
  const defaults = { geometry: { source: "shape", shape: "rectangle" }, mesh: { spec: {}, discretizer: { type: "a" }, triangulator: { type: "b" }, refiner: null, improvers: [] }, problem: {}, assembly: { t: 0 }, solve: { mode: "static", linear_solver: { type: "x" }, static: {}, time: { method: { type: "m" }, step: { type: "s" } } } };
  const edits = [{ op: "rename", from: "a", to: "b" }];
  const input = { ...structuredClone(defaults), format: "pde2d-app-case", geometry: { source: "mesh_file", format: "gmsh", text: "", region_edits: edits } };
  const { caseData, replaced } = normalizeCase(input, defaults);
  assert.deepEqual(caseData.geometry.region_edits, edits);
  assert.deepEqual(replaced, []);
  input.geometry.region_edits = "nope";
  assert.deepEqual(normalizeCase(input, defaults).replaced, ["geometry.region_edits"]);
});

let failed = 0;
for (const [name, body] of cases) {
  try {
    body();
    console.log(`PASS ${name}`);
  } catch (error) {
    failed++;
    console.log(`FAIL ${name}: ${error.message}`);
  }
}
console.log(`${cases.length - failed}/${cases.length} cases passed`);
process.exit(failed ? 1 : 0);
