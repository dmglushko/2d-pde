// Tests of the page's pure helpers (app/web/js/core.js), without a
// browser. Run by CTest as app_web when Node.js is installed.
import assert from "node:assert/strict";
import { applyRegionEdit, fileKind, normalizeCase, reconcileConditions, renameParameter, undoRegionEdit, visibleCoefficients } from "../web/js/pure/case_model.js";
import { fieldScale, lockedScale, nodePositions, oklabToRgb, rampTable, regionToken, rgbToOklab } from "../web/js/pure/color.js";
import { formatNumber, niceTicks, numberText, parseNumberText, pictureName, setDefaultDigits } from "../web/js/pure/format.js";
import { History } from "../web/js/pure/history.js";
import {
  barycentric,
  decodeMesh,
  gradientMagnitudes,
  interpolateAt,
  isolineLevels,
  isolines,
  locatePoint,
  meshGrid,
  nearestValuedNode,
  sampleLine,
  smallestAngle,
  triangleAngles,
  viewTransform,
} from "../web/js/pure/mesh_math.js";
import { SETTINGS_DEFAULTS, clampPictureHeight, freeProbeSlot, normalizeSettings, parseFolds } from "../web/js/pure/prefs.js";
import { keptSnapshots, nearestLoaded, snapshotPlan, splitSnapshots, valueRange } from "../web/js/pure/run_values.js";
import { LARGE_SOLUTION_BYTES, RUN_ALL, STAGES, STEPS, errorPath, errorPathOf, requestCase, solutionBytes, stageKey, stagesToRun, stepOfPath } from "../web/js/pure/stages.js";
import { Tickets } from "../web/js/pure/tickets.js";

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
  assert.deepEqual([seq.low, seq.high, div.low, div.high], [1, 3, -4, 4], "the ends of the colour bar");
});

test("a locked colour range is exactly the range given", () => {
  const scale = lockedScale(-1, 3);
  assert.equal(scale.kind, "diverging", "it spans zero");
  assert.deepEqual([scale.low, scale.high], [-1, 3]);
  assert.equal(scale.position(-1), 0);
  assert.equal(scale.position(3), 1);
  assert.equal(scale.position(1), 0.5);
  assert.equal(lockedScale(0, 2).kind, "sequential");
  assert.equal(lockedScale(2, 2), null, "low must be below high");
  assert.equal(lockedScale(NaN, 2), null);
  const positions = nodePositions([0, 4], scale);
  assert.equal(positions[1], 1.25, "a value beyond the range lies beyond the bar (the drawing clamps it)");
});

test("request tickets: the newest of a kind, in the same scope, for the same key", () => {
  const tickets = new Tickets();
  const first = tickets.issue("mesh", 1, "k1");
  assert.ok(tickets.holds(first, 1, "k1"));
  assert.ok(!tickets.holds(first, 1, "k2"), "the input changed");
  assert.ok(tickets.isLive(first, 1), "still live: kept as an outdated result");
  assert.ok(!tickets.isLive(first, 2), "another case");
  const second = tickets.issue("mesh", 1, "k1");
  assert.ok(!tickets.isLive(first, 1), "a newer request of the kind replaces it");
  assert.ok(tickets.holds(second, 1, "k1"));
  const other = tickets.issue("problem", 1, "p");
  assert.ok(tickets.holds(second, 1, "k1"), "kinds are independent");
  tickets.supersede("problem");
  assert.ok(!tickets.isLive(other, 1), "superseded");
  const a = tickets.issue("difference", 1, "c", { exclusive: false });
  const b = tickets.issue("difference", 1, "c", { exclusive: false });
  assert.ok(tickets.holds(a, 1, "c") && tickets.holds(b, 1, "c"), "requests side by side");
  assert.ok(!tickets.holds(a, 1, "d"), "but only for the same key");
});

test("undo history: typing in one field is one step, bounded, redo after undo", () => {
  const history = new History({ limit: 3 });
  const field = {};
  assert.equal(history.record({ problem: "a" }, { problem: "a" }, field), false, "no change, no step");
  history.record({ problem: "a" }, { problem: "ab" }, field);
  history.record({ problem: "ab" }, { problem: "abc" }, field);
  assert.equal(history.undoSteps.length, 1, "typing merges");
  history.seal();
  history.record({ problem: "abc" }, { problem: "abcd" }, field);
  assert.equal(history.undoSteps.length, 2, "a sealed step does not take more typing");
  history.record({ problem: "abcd", mesh: "m" }, { problem: "abcd", mesh: "n" }, null);
  assert.deepEqual(history.undoSteps.at(-1).before, { mesh: "m" }, "only the sections that changed are kept");
  assert.deepEqual(history.undo(), { mesh: "m" });
  assert.ok(history.canRedo());
  assert.deepEqual(history.redo(), { mesh: "n" });
  history.undo();
  history.record({ geometry: "g" }, { geometry: "h" }, null);
  assert.ok(!history.canRedo(), "a new edit drops the redo steps");
  history.record({ geometry: "h" }, { geometry: "i" }, null);
  history.record({ geometry: "i" }, { geometry: "j" }, null);
  assert.equal(history.undoSteps.length, 3, "bounded: the oldest go");
  assert.deepEqual(history.undo(), { geometry: "i" });
  // Typing back to where the step began leaves no step.
  const again = new History();
  again.record({ view: "1" }, { view: "12" }, field);
  again.record({ view: "12" }, { view: "1" }, field);
  assert.ok(!again.canUndo());
  // Another field is another step.
  again.record({ view: "1" }, { view: "2" }, field);
  again.record({ view: "2" }, { view: "3" }, {});
  assert.equal(again.undoSteps.length, 2);
});

test("the coefficients a form shows", () => {
  const fields = [{ name: "alpha", term: "u" }, { name: "beta_n", term: "u_n" }, { name: "gamma", term: "u_t" }, { name: "g", term: "" }];
  assert.deepEqual(visibleCoefficients(fields, { alpha: "1", g: "0" }), ["alpha", "g"], "a Dirichlet condition");
  assert.deepEqual(visibleCoefficients(fields, { beta_n: "1", alpha: " " }), ["beta_n", "g"], "blank is not set; the right side always shows");
  assert.deepEqual(visibleCoefficients(fields, {}, new Set(["gamma"])), ["gamma", "g"], "revealed by a template");
});

test("the step of a case path, and the picture's height", () => {
  assert.equal(stepOfPath("mesh.spec.max_edge_length.relative"), "mesh");
  assert.equal(stepOfPath("problem.parameters.k"), "problem");
  assert.equal(stepOfPath("assembly.t"), "assemble");
  assert.equal(stepOfPath("solve.time.step.options.dt"), "solve");
  assert.equal(stepOfPath("geometry.options.width"), "geometry");
  assert.equal(stepOfPath("geometry.region_edits[0]"), "geometry");
  assert.equal(clampPictureHeight(50, 800), 160);
  assert.equal(clampPictureHeight(700, 800), 560);
  assert.equal(clampPictureHeight(300.4, 800), 300);
  assert.equal(clampPictureHeight(NaN, 800), null);
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
  // The server's path field wins; the text is the fallback.
  assert.equal(errorPathOf({ path: "geometry.options.width", message: "geometry.options.width: no" }), "geometry.options.width");
  assert.equal(errorPathOf({ path: null, message: "mesh.spec: no" }), "mesh.spec");
  assert.equal(errorPathOf({ message: "RuppertOptions::x must be" }), null);
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
  assert.deepEqual(parseNumberText(".5"), { kind: "number", value: 0.5 });
  assert.deepEqual(parseNumberText("+3E2"), { kind: "number", value: 300 });
  assert.deepEqual(parseNumberText("7."), { kind: "number", value: 7 });
  assert.deepEqual(parseNumberText(""), { kind: "empty" });
  assert.deepEqual(parseNumberText("   "), { kind: "empty" });
  for (const text of ["-", "1e", "Infinity", "0x10", "1 000", "abc", "1.2.3", "1e5.5"]) assert.equal(parseNumberText(text).kind, "invalid", text);
  assert.deepEqual(parseNumberText("-"), { kind: "invalid", reason: "not a number" });
  assert.deepEqual(parseNumberText("0,05"), { kind: "invalid", reason: "use a decimal point (0.05), not a comma" }, "a decimal comma is named, not misread");
  assert.equal(parseNumberText("1e400").reason, "too large for a number");
  assert.equal(numberText(0.05), "0.05", "a decimal point whatever the locale");
  assert.equal(numberText(1e-7), "1e-7");
  assert.equal(numberText(null), "");
  for (const value of [0.1, -2.5e-12, 123456.789, 1 / 3]) assert.equal(parseNumberText(numberText(value)).value, value, "shown text reads back exactly");
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

test("a request carries the stage's sections and nothing else", () => {
  const caseData = { format: "pde2d-app-case", version: 1, notes: "n", view: {}, geometry: { svg: "big" }, mesh: {}, problem: { equation: {} }, assembly: { t: 0 }, solve: { mode: "static" } };
  assert.deepEqual(requestCase(caseData, "problem"), { format: "pde2d-app-case", version: 1, problem: { equation: {} } });
  assert.deepEqual(Object.keys(requestCase(caseData, "solve")).sort(), ["format", "geometry", "mesh", "problem", "solve", "version"]);
  assert.deepEqual(Object.keys(requestCase(caseData, "field")).sort(), ["assembly", "format", "geometry", "mesh", "problem", "version"]);
  // The server's keys come from the same sections.
  assert.equal(stageKey(requestCase(caseData, "assemble"), "assemble"), stageKey(caseData, "assemble"));
});

test("a frame drawn at one view lands where the new view puts it", () => {
  const from = { scale: 10, offset: { x: 5, y: 100 } };
  const to = { scale: 25, offset: { x: -40, y: 160 } };
  const t = viewTransform(from, to);
  // A world point: its pixel on the old frame, moved by the transform, is
  // its pixel in the new view.
  for (const [wx, wy] of [
    [0, 0],
    [3, -2],
    [-7.5, 11],
  ]) {
    const old = [from.offset.x + wx * from.scale, from.offset.y - wy * from.scale];
    const now = [to.offset.x + wx * to.scale, to.offset.y - wy * to.scale];
    assert.ok(Math.abs(t.x + old[0] * t.scale - now[0]) < 1e-9 && Math.abs(t.y + old[1] * t.scale - now[1]) < 1e-9);
  }
  assert.deepEqual(viewTransform(from, from), { scale: 1, x: 0, y: 0 });
});

test("node colour positions are the scale's, NaN without a value", () => {
  const scale = fieldScale([-2, 1]);
  const positions = nodePositions([-2, 0, 1, null, NaN], scale);
  assert.equal(positions[0], 0);
  assert.equal(positions[1], 0.5);
  assert.equal(positions[2], 0.75);
  assert.ok(Number.isNaN(positions[3]) && Number.isNaN(positions[4]));
  // Linear: the position of a mean is the mean of the positions.
  const s = fieldScale([1, 5]);
  const [a, b] = nodePositions([1.5, 4.5], s);
  assert.ok(Math.abs(s.position(3) - (a + b) / 2) < 1e-6);
});

test("settings are made whole and kept in range", () => {
  assert.deepEqual(normalizeSettings(null), SETTINGS_DEFAULTS);
  assert.deepEqual(normalizeSettings({ digits: 99, playbackFps: 0.2, checkWhileTyping: "yes", pixelRatioCap: 1.5, other: 1 }), { ...SETTINGS_DEFAULTS, digits: 10, playbackFps: 1, pixelRatioCap: 1.5 });
  assert.equal(normalizeSettings({ checkDelayMs: 777.4 }).checkDelayMs, 777);
  assert.equal(normalizeSettings({ followRun: false }).followRun, false);
});

test("folded sections are read back safely", () => {
  assert.deepEqual({ ...parseFolds('{"mesh.discretizer": true, "x": 3, "__proto__": false}') }, { "mesh.discretizer": true, ["__proto__"]: false });
  assert.deepEqual({ ...parseFolds("not json") }, {});
  assert.deepEqual({ ...parseFolds(null) }, {});
});

test("the significant digits setting is formatNumber's default", () => {
  setDefaultDigits(6);
  assert.equal(formatNumber(Math.PI), "3.14159");
  setDefaultDigits(4);
  assert.equal(formatNumber(Math.PI), "3.142");
  setDefaultDigits(1);
  assert.equal(formatNumber(Math.PI), "3.142", "out of range: unchanged");
});


// A 2 × 1 rectangle split into four triangles around its centre, as a
// mesh view (flat arrays). Node 4 is the centre (1, 0.5).
const square = {
  nodes: [0, 0, 2, 0, 2, 1, 0, 1, 1, 0.5],
  triangles: [0, 1, 4, 1, 2, 4, 2, 3, 4, 3, 0, 4],
};
const linear = (f) => Array.from({ length: 5 }, (_, i) => f(square.nodes[2 * i], square.nodes[2 * i + 1]));

test("a point is located and a P1 field interpolated exactly", () => {
  const grid = meshGrid(square);
  const values = linear((x, y) => 3 * x - 2 * y + 1);
  for (const [x, y] of [[0.3, 0.2], [1.7, 0.9], [1, 0.5], [2, 1], [0.01, 0.99]]) {
    const found = locatePoint(square, grid, x, y);
    assert.ok(found, `(${x}, ${y}) is inside`);
    assert.ok(Math.abs(interpolateAt(found, values) - (3 * x - 2 * y + 1)) < 1e-12, `value at (${x}, ${y})`);
    assert.ok(Math.abs(found.weights.reduce((a, b) => a + b) - 1) < 1e-12, "weights sum to one");
  }
  assert.equal(locatePoint(square, grid, 2.5, 0.5), null, "outside");
  assert.ok(Number.isNaN(interpolateAt(null, values)));
  const holes = values.slice();
  holes[4] = NaN;
  assert.ok(Number.isNaN(interpolateAt(locatePoint(square, grid, 1, 0.4), holes)), "a missing value gives NaN");
});

test("the gradient of a P1 field per triangle", () => {
  // A linear field has the same gradient (3, -4) on every triangle.
  const grads = gradientMagnitudes(square.nodes, square.triangles, linear((x, y) => 3 * x - 4 * y + 2));
  assert.equal(grads.length, 4);
  for (const g of grads) assert.ok(Math.abs(g - 5) < 1e-12, `|∇u| = ${g}`);
  // u = x² at the nodes: on the bottom triangle (0,0), (2,0), (1,0.5) with
  // values 0, 4, 1 the interpolant is 2x - 2y, so |∇u| = sqrt(8).
  const quad = gradientMagnitudes(square.nodes, square.triangles, linear((x) => x * x));
  assert.ok(Math.abs(quad[0] - Math.sqrt(8)) < 1e-12, `${quad[0]}`);
  const holes = linear((x) => x);
  holes[4] = NaN;
  assert.ok(gradientMagnitudes(square.nodes, square.triangles, holes).every(Number.isNaN), "a missing value gives NaN");
  assert.ok(Number.isNaN(gradientMagnitudes([0, 0, 1, 0, 2, 0], [0, 1, 2], [0, 1, 2])[0]), "a degenerate triangle gives NaN");
});

test("a line profile samples the interpolant along the line", () => {
  const grid = meshGrid(square);
  const values = linear((x, y) => x + 2 * y);
  const { distance, values: along } = sampleLine(square, grid, values, [0, 0.5], [2, 0.5], 4);
  assert.deepEqual([...distance], [0, 0.5, 1, 1.5, 2]);
  along.forEach((v, i) => assert.ok(Math.abs(v - (0.5 * i + 1)) < 1e-12, `${v} at ${i}`));
  const out = sampleLine(square, grid, values, [1, 0.5], [3, 0.5], 2);
  assert.ok(Math.abs(out.values[0] - 2) < 1e-12);
  assert.ok(Number.isNaN(out.values[2]), "outside the mesh is a gap");
});

test("the nearest node with a value, within a radius", () => {
  const values = [NaN, 5, null, 7, NaN];
  assert.equal(nearestValuedNode(square.nodes, values, 1.9, 0.1, 0.5), 1);
  assert.equal(nearestValuedNode(square.nodes, values, 0.1, 0.9, 0.5), 3);
  assert.equal(nearestValuedNode(square.nodes, values, 1, 0.5, 0.2), -1, "the centre has no value, the others are far");
  assert.equal(nearestValuedNode(square.nodes, values, 0.05, 0.05, 0.5), -1, "node 0 has no value");
  assert.equal(nearestValuedNode([0, 0, 2, 0], [1, 2], 1, 0, 1), 0, "a tie goes to the lower index");
});

test("probes keep their slot", () => {
  assert.equal(freeProbeSlot([]), 0);
  assert.equal(freeProbeSlot([{ slot: 0 }, { slot: 2 }]), 1);
  assert.equal(freeProbeSlot([{ slot: 1 }, { slot: 0 }, { slot: 3 }, { slot: 2 }]), -1);
});

test("isoline levels lie inside the range", () => {
  assert.deepEqual(isolineLevels(0, 1, 3), [0.25, 0.5, 0.75]);
  assert.deepEqual(isolineLevels(1, 1, 5), [], "a constant field has none");
  assert.deepEqual(isolineLevels(0, 1, 0), []);
});

test("marching triangles: the isoline of x = c is the segment x = c", () => {
  const values = linear((x) => x);
  const [segments] = isolines(square.nodes, square.triangles, values, [0.5]);
  assert.equal(segments.length % 4, 0);
  assert.ok(segments.length > 0);
  for (let i = 0; i < segments.length; i += 2) {
    assert.ok(Math.abs(segments[i] - 0.5) < 1e-12, `x = ${segments[i]}`);
    assert.ok(segments[i + 1] >= -1e-12 && segments[i + 1] <= 1 + 1e-12, `y = ${segments[i + 1]}`);
  }
  // The segments cover y from 0 to 1 once: their lengths add up to 1.
  let length = 0;
  for (let i = 0; i < segments.length; i += 4) length += Math.hypot(segments[i + 2] - segments[i], segments[i + 3] - segments[i + 1]);
  assert.ok(Math.abs(length - 1) < 1e-12, `length ${length}`);
});

test("marching triangles: a level through a node is drawn once", () => {
  const values = linear((x) => x);
  // x = 1 passes through the centre node: the line x = 1 from y = 0 to 1.
  const [segments] = isolines(square.nodes, square.triangles, values, [1]);
  let length = 0;
  for (let i = 0; i < segments.length; i += 4) length += Math.hypot(segments[i + 2] - segments[i], segments[i + 3] - segments[i + 1]);
  assert.ok(Math.abs(length - 1) < 1e-12, `length ${length}`);
  const [none] = isolines(square.nodes, square.triangles, values, [5]);
  assert.equal(none.length, 0, "outside the range: no segment");
  const holes = values.slice();
  holes[4] = NaN;
  const [skipped] = isolines(square.nodes, square.triangles, holes, [0.5]);
  assert.equal(skipped.length, 0, "triangles with a missing value are skipped");
});

test("the range of one snapshot", () => {
  assert.deepEqual(valueRange([3, NaN, -1, 2]), { min: -1, max: 3 });
  assert.ok(Number.isNaN(valueRange([NaN]).min));
});

test("chart ticks are round and cover the range", () => {
  assert.deepEqual(niceTicks(0, 1, 4), [0, 0.2, 0.4, 0.6, 0.8, 1]);
  assert.deepEqual(niceTicks(-0.13, 0.97, 4), [-0.2, 0, 0.2, 0.4, 0.6, 0.8, 1]);
  assert.deepEqual(niceTicks(0, 2.5, 5), [0, 0.5, 1, 1.5, 2, 2.5]);
  const flat = niceTicks(3, 3, 4);
  assert.ok(flat[0] <= 3 && flat.at(-1) >= 3, "a constant gets a range around it");
  assert.deepEqual(niceTicks(NaN, 1), []);
});

test("picture names carry the case and the time", () => {
  assert.equal(pictureName("heat_annulus.json", 0.25), "heat_annulus-t0.25.png");
  assert.equal(pictureName("case.json"), "case.png");
  assert.equal(pictureName("my case (2).json", 1 / 3), "my_case_2_-t0.333333.png");
});

test("a mesh decodes from its summary and binary arrays", () => {
  // Two nodes, one triangle (0 1 0), one boundary edge 1 -> 0 in region 2.
  const buffer = new ArrayBuffer(32 + 12 + 8 + 4);
  const view = new DataView(buffer);
  [0, 0.5, 1, -2].forEach((v, i) => view.setFloat64(8 * i, v, true));
  [0, 1, 0].forEach((v, i) => view.setUint32(32 + 4 * i, v, true));
  view.setUint32(44, 1, true);
  view.setUint32(48, 0, true);
  view.setUint32(52, 2, true);
  const summary = {
    id: 3,
    region_names: ["a", "b", "c"],
    has_links: true,
    arrays: {
      bytes: 56,
      parts: [
        { name: "nodes", count: 4, offset: 0 },
        { name: "triangles", count: 3, offset: 32 },
        { name: "boundary_edges", count: 2, offset: 44 },
        { name: "boundary_regions", count: 1, offset: 52 },
      ],
    },
  };
  const mesh = decodeMesh(summary, buffer);
  assert.ok(mesh.nodes instanceof Float64Array);
  assert.deepEqual([...mesh.nodes], [0, 0.5, 1, -2]);
  assert.deepEqual([...mesh.triangles], [0, 1, 0]);
  assert.deepEqual([...mesh.boundary_edges], [1, 0, 2], "a, b and the region, as the picture reads them");
  assert.equal(mesh.id, 3);
  assert.throws(() => decodeMesh(summary, new ArrayBuffer(8)), /bytes/);
});

test("the page keeps the shown snapshot, its neighbours and a bounded number", () => {
  const loaded = (n, which) => Array.from({ length: n }, (_, i) => (which.includes(i) ? new Float32Array(1) : null));
  assert.deepEqual(snapshotPlan([], 0), { fetch: null, evict: [] });
  // Nothing loaded: the shown one, two behind and eight ahead in one range.
  assert.deepEqual(snapshotPlan(loaded(20, []), 5).fetch, { first: 3, count: 11 });
  // Only the missing span is fetched.
  assert.deepEqual(snapshotPlan(loaded(20, [3, 4, 5, 6]), 5).fetch, { first: 7, count: 7 });
  assert.equal(snapshotPlan(loaded(10, [8, 9]), 9).fetch.first, 7);
  assert.equal(snapshotPlan(loaded(3, [0, 1, 2]), 1).fetch, null, "all there");
  // Beyond `kept`, the loaded ones farthest from the shown one go.
  const plan = snapshotPlan(loaded(30, [0, 1, 2, 20, 21]), 20, { kept: 6, ahead: 1, behind: 0 });
  assert.equal(plan.fetch, null, "21 is loaded");
  assert.deepEqual(plan.evict, []);
  const tight = snapshotPlan(loaded(30, [0, 1, 2, 20]), 20, { kept: 3, ahead: 1, behind: 0 });
  assert.deepEqual(tight.fetch, { first: 21, count: 1 });
  assert.deepEqual(tight.evict.sort(), [0, 1], "the farthest two make room");
  assert.equal(keptSnapshots(1000), 1000);
  assert.equal(keptSnapshots(4e6), 8, "at least 8");
  assert.equal(nearestLoaded(loaded(10, [2, 7]), 5), 7);
  assert.equal(nearestLoaded(loaded(10, [2, 8]), 4), 2);
  assert.equal(nearestLoaded(loaded(3, []), 1), -1);
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
