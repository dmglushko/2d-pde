// Tests of the page's pure helpers (app/web/js/core.js), without a
// browser. Run by CTest as app_web when Node.js is installed.
import assert from "node:assert/strict";
import {
  applyDrawingChange,
  applyRegionEdit,
  fileKind,
  normalizeCase,
  reconcileConditions,
  renameParameter,
  renameRegionByEdit,
  replaceLastRegionEdit,
  undoRegionEdit,
  visibleCoefficients,
} from "../web/js/pure/case_model.js";
import * as drawingModel from "../web/js/pure/drawing.js";
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
  meshEdges,
  meshGrid,
  nearestValuedNode,
  sampleLine,
  smallestAngle,
  triangleAngles,
  viewTransform,
} from "../web/js/pure/mesh_math.js";
import { SETTINGS_DEFAULTS, clampPictureHeight, clampSidebarWidth, freeProbeSlot, normalizeSettings, parseFolds } from "../web/js/pure/prefs.js";
import { addPaceSample, frameAt, measuredPace, PACE_SHARE, playbackStep, speedLabel, START_SNAPSHOTS } from "../web/js/pure/playback.js";
import { isLine } from "../web/js/pure/case_model.js";
import { keptSnapshots, nearestLoaded, snapshotNodes, snapshotPlan, splitCounted, splitSnapshots, valueRange } from "../web/js/pure/run_values.js";
import { LARGE_SOLUTION_BYTES, RUN_ALL, STAGES, STEPS, errorPath, errorPathOf, requestCase, solutionBytes, stageKey, stagesToRun, stepOfPath } from "../web/js/pure/stages.js";
import { Tickets } from "../web/js/pure/tickets.js";
import { displayKey, resolveDisplay } from "../web/js/pure/display.js";
import { backoffDelay, isTransient, plainError, serverBusyText } from "../web/js/pure/requests.js";
import { adaptiveRows, observedOrders, referenceSlope, studyLevels } from "../web/js/pure/study.js";
import { bytesText, colorbarTicks, csvText, logTicks, seriesCsv, widestText } from "../web/js/pure/format.js";
import { COLOR_MAPS, colorMapOf } from "../web/js/pure/color.js";
import { isTimeRun, serverRunBytes } from "../web/js/pure/stages.js";

const cases = [];
const test = (name, body) => cases.push([name, body]);

test("mesh edges: each edge once, two triangles share one", () => {
  // Two triangles of a square share the diagonal 0-2.
  const edges = meshEdges([0, 1, 2, 0, 2, 3], 4);
  assert.equal(edges.length, 2 * 5);
  const keys = new Set();
  for (let i = 0; i < edges.length; i += 2) keys.add(`${Math.min(edges[i], edges[i + 1])}-${Math.max(edges[i], edges[i + 1])}`);
  assert.deepEqual([...keys].sort(), ["0-1", "0-2", "0-3", "1-2", "2-3"]);
});

test("the sidebar's width stays between 300 px and the picture's 360 px", () => {
  assert.equal(clampSidebarWidth(500, 1600), 500);
  assert.equal(clampSidebarWidth(100, 1600), 300);
  assert.equal(clampSidebarWidth(1500, 1600), 1240);
  assert.equal(clampSidebarWidth(400, 500), 300);
  assert.equal(clampSidebarWidth(Number.NaN, 1600), null);
});

test("the widest text: the longest, the first of equal ones", () => {
  assert.equal(widestText(["t = 1", "t = 14.65", "t = 9.5", "t = 10.25"]), "t = 14.65");
  assert.equal(widestText([]), "");
});

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
  // An adaptive solve sends its last solution: at most max_nodes values.
  assert.equal(solutionBytes({ mode: "adaptive", adaptive: { max_nodes: 500 } }, 1000), 2000);
  assert.equal(solutionBytes({ mode: "adaptive", adaptive: {} }, 1000), null);
  // Adaptive in time: every snapshot up to max_nodes values, and the meshes.
  const inTime = { mode: "adaptive_time", time: { snapshots: 4 }, adaptive_time: { max_nodes: 100, max_refinements: 2 } };
  assert.equal(solutionBytes(inTime, 10), 4 * 100 * 5 + 28 * 100 * 3);
  assert.equal(solutionBytes({ ...inTime, adaptive_time: {} }, 10), null);
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
  assert.deepEqual(normalizeSettings({ digits: 99, playbackSpeed: 0, checkWhileTyping: "yes", pixelRatioCap: 1.5, other: 1 }), { ...SETTINGS_DEFAULTS, digits: 10, playbackSpeed: 1e-6, pixelRatioCap: 1.5 });
  // A speed is not rounded; settings that are gone are dropped.
  assert.equal(normalizeSettings({ playbackSpeed: 0.25 }).playbackSpeed, 0.25);
  assert.equal(SETTINGS_DEFAULTS.playbackSpeed, 1, "real time");
  assert.equal(normalizeSettings({ checkDelayMs: 777.4 }).checkDelayMs, undefined);
  assert.equal(normalizeSettings({ followRun: false }).followRun, false);
});

test("playback: simulated time moves with real time at the speed", () => {
  // Snapshots every 0.5 s of simulated time: 1× shows the next one after
  // half a second of real time, whatever the steps between them.
  const times = [0, 0.5, 1, 1.5, 2];
  assert.equal(frameAt(times, 0.49), 0);
  assert.equal(frameAt(times, 0.5), 1);
  assert.equal(frameAt(times, 0.5 - 1e-12), 1, "rounding of the clock does not hold a frame back");
  assert.equal(frameAt(times, 9), 4);
  let step = playbackStep({ t: 0, seconds: 0.3, speed: 1, times });
  assert.deepEqual([step.t, step.frame, step.waiting, step.finished], [0.3, 0, null, false]);
  step = playbackStep({ t: 0.3, seconds: 0.3, speed: 1, times });
  assert.ok(Math.abs(step.t - 0.6) < 1e-12 && step.frame === 1);
  // 10×: three seconds of simulated time in 0.3 s; the end of a finished
  // run is reached and said.
  step = playbackStep({ t: 0, seconds: 0.3, speed: 10, times });
  assert.deepEqual([step.t, step.frame, step.finished], [2, 4, true]);
  // Not past the snapshots that are loaded.
  step = playbackStep({ t: 0, seconds: 1, speed: 1, times, upTo: 1 });
  assert.deepEqual([step.t, step.frame, step.waiting], [0.5, 1, "loading"]);
  assert.equal(speedLabel(0.25), "0.25×");
  assert.equal(speedLabel(1000), "1000×");
});

test("playback of a running run: a few snapshots first, then no faster than the run", () => {
  const few = [0, 0.1];
  let step = playbackStep({ t: 0, seconds: 0.5, speed: 1, times: few, running: true });
  assert.deepEqual([step.t, step.waiting], [0, "starting"], "it waits for a few snapshots");
  const times = Array.from({ length: START_SNAPSHOTS }, (_, i) => i * 0.1);
  // The run computes 0.2 simulated seconds per second: 1× would outrun
  // it, so it plays at 90 % of that.
  step = playbackStep({ t: 0, seconds: 1, speed: 1, times, running: true, pace: 0.2 });
  assert.ok(Math.abs(step.t - PACE_SHARE * 0.2) < 1e-12 && step.speed === PACE_SHARE * 0.2);
  // A run faster than the speed: the speed holds.
  step = playbackStep({ t: 0, seconds: 0.1, speed: 1, times, running: true, pace: 50 });
  assert.ok(Math.abs(step.t - 0.1) < 1e-12 && step.speed === 1);
  // Caught up with the run: it waits at the newest snapshot.
  step = playbackStep({ t: 0.25, seconds: 1, speed: 1, times, running: true, pace: 50 });
  assert.deepEqual([step.t, step.frame, step.waiting, step.finished], [0.30000000000000004, 3, "computing", false]);
  // The pace over the last seconds of progress.
  const samples = [];
  addPaceSample(samples, 0, 0);
  addPaceSample(samples, 100, 0.1);
  assert.equal(measuredPace(samples), null, "too short a window");
  addPaceSample(samples, 1000, 0.5);
  addPaceSample(samples, 1000, 0.5);
  assert.equal(samples.length, 3, "a sample that did not move is not kept");
  assert.equal(measuredPace(samples), 0.5);
  addPaceSample(samples, 9000, 1.3);
  assert.ok(samples[0].at >= 1000, "old samples leave the window");
});

test("an equation or condition is a line or coefficients", () => {
  assert.equal(isLine({ text: "u = 0", priority: 1 }), true);
  assert.equal(isLine({ text: "" }), true, "an empty line is still a line");
  assert.equal(isLine({ alpha: "1", g: "0" }), false);
  assert.equal(isLine(null), false);
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

// --- the resolved display (pure/display.js) ------------------------------------

const solveData = (extra = {}) => ({ times: [0, 0.5, 1], values: [], scale: { kind: "sequential" }, ...extra });
const u0 = new Float32Array([1, 2]);
const u2 = new Float32Array([3, 4]);
const d2 = new Float32Array([1e-3, -1e-3]);

test("the display: u, its snapshot, the nearest stand-in while loading", () => {
  const shown = resolveDisplay({ colorBy: "field", source: "solve", solve: { data: solveData(), fresh: true, frame: 2, snapshot: { values: u2, nearest: null, failure: null } } });
  assert.equal(shown.kind, "solution");
  assert.equal(shown.label, "u at t = 1");
  assert.equal(shown.values, u2);
  assert.equal(shown.outdated, false);
  assert.equal(shown.loading, null);
  assert.match(shown.precision, /float32/);
  const loading = resolveDisplay({ colorBy: "field", source: "solve", solve: { data: solveData(), fresh: true, frame: 2, snapshot: { values: null, nearest: { index: 0, values: u0 }, failure: null } } });
  assert.equal(loading.values, u0, "the nearest loaded snapshot stands in");
  assert.equal(loading.label, "u at t = 0", "the stand-in's own time; the loading is said over the picture");
  assert.match(loading.loading, /Loading u at t = 1/);
  const none = resolveDisplay({ colorBy: "field", source: "solve", solve: { data: solveData(), fresh: true, frame: 1, snapshot: { values: null, nearest: null, failure: null } } });
  assert.equal(none.values, null);
  assert.ok(none.loading, "never a silent empty picture");
});

test("the display: the \"this snapshot\" range is the drawn snapshot's own", () => {
  const scales = [{ kind: "s0" }, { kind: "s1" }, { kind: "s2" }];
  const snapshotScale = (index) => scales[index];
  const whole = resolveDisplay({ colorBy: "field", source: "solve", solve: { data: solveData(), fresh: true, frame: 2, snapshot: { values: u2, nearest: null, failure: null } } });
  assert.equal(whole.scale.kind, "sequential", "the whole run's scale without the choice");
  const own = resolveDisplay({ colorBy: "field", source: "solve", snapshotScale, solve: { data: solveData(), fresh: true, frame: 2, snapshot: { values: u2, nearest: null, failure: null } } });
  assert.equal(own.scale, scales[2]);
  const standIn = resolveDisplay({ colorBy: "field", source: "solve", snapshotScale, solve: { data: solveData(), fresh: true, frame: 2, snapshot: { values: null, nearest: { index: 0, values: u0 }, failure: null } } });
  assert.equal(standIn.scale, scales[0], "the scale of the snapshot drawn, while the shown one loads");
  const none = resolveDisplay({ colorBy: "field", source: "solve", snapshotScale: () => null, solve: { data: solveData(), fresh: true, frame: 2, snapshot: { values: u2, nearest: null, failure: null } } });
  assert.equal(none.scale.kind, "sequential", "a snapshot without a range keeps the run's");
});

test("the display: a failed snapshot is a failure with retry, never loading forever", () => {
  const error = { kind: "network", message: "no answer" };
  const failed = resolveDisplay({ colorBy: "field", source: "solve", solve: { data: solveData(), fresh: true, frame: 1, snapshot: { values: null, nearest: null, failure: error } } });
  assert.equal(failed.failure.error, error);
  assert.equal(failed.failure.retry, "snapshot");
  assert.equal(failed.loading, null);
});

test("the display: a difference only while its comparison is current", () => {
  const solve = { data: solveData(), fresh: true, frame: 2, snapshot: { values: u2, nearest: null, failure: null } };
  const scale = { kind: "diverging" };
  const current = resolveDisplay({ colorBy: "field", source: "solve", difference: "exact", solve, comparison: { kind: "exact", state: "current", scale, values: { values: d2, nearest: null, failure: null } } });
  assert.equal(current.kind, "difference");
  assert.equal(current.label, "u − exact at t = 1");
  assert.equal(current.values, d2);
  assert.equal(current.scale, scale);
  assert.equal(current.difference.shown, true);
  assert.match(current.precision, /double/);
  // The reproduced case: the expression changed, the comparison is
  // outdated, and u is drawn, labelled u, with the reason.
  const outdated = resolveDisplay({ colorBy: "field", source: "solve", difference: "exact", solve, comparison: { kind: "exact", state: "outdated" } });
  assert.equal(outdated.kind, "solution");
  assert.equal(outdated.label, "u at t = 1");
  assert.equal(outdated.values, u2);
  assert.equal(outdated.difference.shown, false);
  assert.equal(outdated.difference.state, "outdated");
  assert.match(outdated.difference.note, /previous expression/);
  for (const state of ["none", "busy", "failed"]) {
    const other = resolveDisplay({ colorBy: "field", source: "solve", difference: "reference", solve, comparison: { kind: "reference", state } });
    assert.equal(other.kind, "solution", state);
    assert.match(other.difference.note, /u − reference/);
  }
  // A difference being fetched shows its nearest fetched one, said so; a
  // failed one is a failure (never plain u in its place).
  const near = resolveDisplay({ colorBy: "field", source: "solve", difference: "exact", solve, comparison: { kind: "exact", state: "current", scale, values: { values: null, nearest: { index: 0, values: d2 }, failure: null } } });
  assert.equal(near.label, "u − exact at t = 0");
  const failed = resolveDisplay({ colorBy: "field", source: "solve", difference: "exact", solve, comparison: { kind: "exact", state: "current", scale, values: { values: null, nearest: null, failure: { kind: "busy" } } } });
  assert.equal(failed.kind, "difference");
  assert.equal(failed.values, null);
  assert.equal(failed.failure.retry, "difference");
});

test("the display: an earlier adaptive cycle names itself and draws no comparison", () => {
  const solve = { data: solveData({ times: [0], when: "in cycle 2 of 5" }), fresh: true, frame: 0, snapshot: { values: u0, nearest: null, failure: null } };
  const shown = resolveDisplay({ colorBy: "field", source: "solve", difference: "exact", solve, comparison: { kind: "exact", state: "cycle" } });
  assert.equal(shown.kind, "solution", "u, never the last cycle's difference");
  assert.equal(shown.label, "u in cycle 2 of 5");
  assert.equal(shown.values, u0);
  assert.equal(shown.difference.state, "cycle");
  assert.match(shown.difference.note, /last cycle's/);
});

test("the display: an outdated result is marked, and draws no comparison", () => {
  const solve = { data: solveData(), fresh: false, frame: 0, snapshot: { values: u0, nearest: null, failure: null } };
  const shown = resolveDisplay({ colorBy: "field", source: "solve", difference: "exact", solve, comparison: { kind: "exact", state: "current", values: {} } });
  assert.equal(shown.outdated, true);
  assert.equal(shown.kind, "solution");
  assert.equal(shown.difference.state, "stale");
  const field = resolveDisplay({ colorBy: "field", source: "field", field: { data: { label: "Dirichlet values", kind: "dirichlet", t: 0, values: [1, 2] }, fresh: false } });
  assert.equal(field.kind, "field");
  assert.equal(field.label, "Dirichlet values at t = 0");
  assert.equal(field.outdated, true);
  const initial = resolveDisplay({ colorBy: "field", source: "field", field: { data: { label: "initial u", kind: "initial_u", t: 0, values: [1] }, fresh: true } });
  assert.equal(initial.label, "initial u", "no time for an initial condition");
  assert.equal(resolveDisplay({ colorBy: "none", source: "solve" }).kind, "none");
  assert.notEqual(displayKey(shown), displayKey({ ...shown, outdated: false }));
});

// --- requests, ticks, data, colour maps, the study ---------------------------------

test("failed requests: what is retried, after how long, in which words", () => {
  assert.ok(isTransient({ kind: "busy" }) && isTransient({ kind: "network" }));
  assert.ok(!isTransient({ kind: "expired" }) && !isTransient({ kind: "input" }) && !isTransient(null));
  assert.deepEqual([0, 1, 2, 3, 4, 5, 10].map((n) => backoffDelay(n)), [250, 500, 1000, 2000, 4000, 4000, 4000]);
  assert.equal(backoffDelay(2, { base: 100, max: 300 }), 300);
  const expired = plainError({ kind: "expired", message: "this server has no run 3 (it was restarted); solve again" });
  assert.equal(expired.action, "solve");
  assert.match(expired.text, /^The result is gone from the server: this server has no run 3/);
  assert.equal(plainError({ kind: "replaced", message: "x" }).action, null);
  assert.match(plainError({ kind: "memory", message: "too big" }).text, /no room.*Too big\./);
  assert.equal(plainError({ kind: "network" }, "u at the probes").action, "retry");
  assert.match(plainError({ kind: "network" }, "u at the probes").text, /u at the probes did not arrive/);
  assert.equal(plainError({ kind: "input", message: "bad" }, "the snapshot").text, "The snapshot could not be loaded: Bad.");
  assert.equal(serverBusyText({ busy: null }), null);
  assert.equal(serverBusyText({ busy: { task: "mesh", yours: false, seconds: 12.7 } }), "server busy: meshing for another tab (12 s)");
  assert.equal(serverBusyText({ busy: { task: "run", yours: true, seconds: 0.2 } }), "server busy: running a solve (0 s)");
});

test("colour bar ticks are round, inside the range, and include zero", () => {
  const ticks = colorbarTicks(0.013, 0.987, 4);
  assert.ok(ticks.length >= 3, JSON.stringify(ticks));
  for (const t of ticks) {
    assert.ok(t.value >= 0.013 && t.value <= 0.987);
    assert.ok(Math.abs(t.position - (t.value - 0.013) / 0.974) < 1e-12);
  }
  assert.deepEqual(ticks.map((t) => t.text), ["0.2", "0.4", "0.6", "0.8"]);
  const diverging = colorbarTicks(-3e-4, 3e-4, 4);
  assert.ok(diverging.some((t) => t.value === 0 && Math.abs(t.position - 0.5) < 1e-12), "zero in the middle");
  assert.equal(new Set(diverging.map((t) => t.text)).size, diverging.length, "distinct texts");
  const close = colorbarTicks(1.0001, 1.0009, 4);
  assert.equal(new Set(close.map((t) => t.text)).size, close.length, "as many digits as keep them apart");
  assert.deepEqual(colorbarTicks(1, 1), []);
  assert.deepEqual(colorbarTicks(NaN, 1), []);
});

test("logarithmic ticks", () => {
  assert.deepEqual(logTicks(0.003, 0.5), [0.01, 0.1]);
  assert.deepEqual(logTicks(0.02, 0.09), [0.02, 0.05]);
  assert.deepEqual(logTicks(1e-6, 1e-2), [1e-6, 1e-5, 1e-4, 1e-3, 1e-2]);
  assert.deepEqual(logTicks(0, 1), []);
});

test("chart data as CSV, sizes in bytes", () => {
  assert.equal(seriesCsv("t", [0, 0.5], [{ name: "P1", values: [1, NaN] }, { name: 'a "b"', values: [0.1 + 0.2, 2] }]), 't,P1,"a ""b"""\n0,1,0.30000000000000004\n0.5,,2\n');
  assert.equal(csvText(["x"], [[null], ["a,b"]]), 'x\n\n"a,b"\n');
  assert.equal(bytesText(512), "512 B");
  assert.equal(bytesText(3.4 * 1024 * 1024), "3.4 MB");
  assert.equal(bytesText(1024 ** 3), "1 GB");
  assert.deepEqual(serverRunBytes({ mode: "time", time: { snapshots: 9 } }, { snapshot_bytes: 100, budget: 2000 }), { bytes: 1000, budget: 2000, share: 0.5 });
  assert.equal(serverRunBytes({ mode: "static" }, { snapshot_bytes: 100, budget: 1000 }).bytes, 100);
  // An adaptive run reserves max_cycles meshes of max_nodes (runs.cpp).
  assert.equal(serverRunBytes({ mode: "adaptive", adaptive: { max_nodes: 1000, max_cycles: 3 } }, { snapshot_bytes: 100, budget: 1e9 }).bytes, 3 * (1000 * 96 + 512));
  assert.equal(serverRunBytes({ mode: "adaptive", adaptive: { max_nodes: 0, max_cycles: 3 } }, { snapshot_bytes: 100, budget: 1e9 }), null);
  // Adaptive in time: max_refinements + 1 meshes and the snapshots, each of
  // at most max_nodes nodes (runs.cpp, adaptive_time_run_bytes).
  const inTime = { mode: "adaptive_time", time: { snapshots: 4 }, adaptive_time: { max_nodes: 1000, max_refinements: 2 } };
  assert.equal(serverRunBytes(inTime, { snapshot_bytes: 100, budget: 1e9 }).bytes, 3 * (1000 * 88 + 512) + 5 * (1000 * 8 + 128) + 1000 * 8);
  assert.equal(serverRunBytes({ ...inTime, adaptive_time: { max_nodes: 1000 } }, { snapshot_bytes: 100, budget: 1e9 }), null);
  // Time order 2: u_t and u_tt of every snapshot and of the state too.
  assert.equal(serverRunBytes(inTime, { snapshot_bytes: 100, budget: 1e9 }, 2).bytes, 3 * (1000 * 88 + 512) + 5 * (1000 * 24 + 128) + 1000 * 24);
  assert.equal(serverRunBytes({ mode: "time", time: {} }, { snapshot_bytes: 1, budget: 1 }), null);
  assert.equal(serverRunBytes({ mode: "static" }, null), null);
});

test("colour maps: a default per kind, chosen by name", () => {
  for (const kind of ["sequential", "diverging"]) {
    assert.ok(COLOR_MAPS[kind].length >= 2);
    assert.equal(colorMapOf(kind, "no such map"), COLOR_MAPS[kind][0]);
    for (const map of COLOR_MAPS[kind]) assert.equal(map.tokens.length, 3);
  }
  assert.equal(colorMapOf("sequential", "orange").id, "orange");
  assert.equal(normalizeSettings({ sequentialMap: "orange" }).sequentialMap, "orange");
  assert.equal(normalizeSettings({ sequentialMap: "rainbow" }).sequentialMap, "blue", "an unknown map is the default");
  assert.equal(normalizeSettings({ divergingMap: 3 }).divergingMap, "blue-red");
});

test("an adaptive solve's cycle rows: errors by cycle and rates against N", () => {
  const cycles = [
    { cycle: 1, nodes: 100, triangles: 180, estimate: 0.4, marked: 20 },
    { cycle: 2, nodes: 400, triangles: 760, estimate: 0.2, marked: 90 },
    { cycle: 3, nodes: 1600, triangles: 3100, estimate: 0.1, marked: null },
  ];
  const plain = adaptiveRows(cycles);
  assert.equal(plain.length, 3);
  assert.equal(plain[0].estimateRate, null);
  // 0.4 -> 0.2 while N grows 4 times: N^(-1/2).
  assert.ok(Math.abs(plain[1].estimateRate - 0.5) < 1e-12);
  assert.ok(Math.abs(plain[2].estimateRate - 0.5) < 1e-12);
  assert.equal(plain[1].l2, null);
  assert.equal(plain[1].l2Rate, null);
  // The comparison's cycles may come in any order; a missing one is null.
  const compared = [
    { cycle: 3, max_abs: 0.01, l2: 0.000625 },
    { cycle: 1, max_abs: 0.1, l2: 0.01 },
    { cycle: 2, max_abs: 0.05, l2: 0.0025 },
  ];
  const rows = adaptiveRows(cycles, compared);
  assert.equal(rows[0].max, 0.1);
  assert.ok(Math.abs(rows[1].l2Rate - 1) < 1e-12);
  assert.ok(Math.abs(rows[2].l2Rate - 1) < 1e-12);
  assert.deepEqual(adaptiveRows(null), []);
  assert.equal(adaptiveRows(cycles, [{ cycle: 2, max_abs: 1, l2: 1 }])[0].l2, null);
});

test("a refinement study: levels and observed orders", () => {
  assert.deepEqual(studyLevels({ absolute: null, relative: 0.08 }, 3), [
    { key: "relative", value: 0.08 },
    { key: "relative", value: 0.04 },
    { key: "relative", value: 0.02 },
  ]);
  assert.deepEqual(studyLevels({ absolute: 0.5, relative: null }, 2), [
    { key: "absolute", value: 0.5 },
    { key: "absolute", value: 0.25 },
  ]);
  assert.equal(studyLevels({ absolute: null, relative: null }, 3), null);
  // e = C h^2 exactly: order 2 between every pair of levels.
  const rows = [0.2, 0.1, 0.05].map((h) => ({ h, e: 3 * h * h }));
  const orders = observedOrders(rows, (r) => r.e);
  assert.equal(orders[0], null);
  for (const o of orders.slice(1)) assert.ok(Math.abs(o - 2) < 1e-12, String(o));
  // A level whose mesh did not change, or a missing error, has none.
  assert.deepEqual(observedOrders([{ h: 1, e: 1 }, { h: 1, e: 0.5 }, { h: 0.5, e: 0 }], (r) => r.e), [null, null, null]);
  const slope = referenceSlope([1, 0.5, 0.25], 2, 1, 4);
  assert.deepEqual(slope, [4, 1, 0.25]);
});

// --- the drawing editor (pure/drawing.js) ---------------------------------------

const D = drawingModel;
const close = (a, b, tolerance = 1e-12) => Math.abs(a - b) <= tolerance;
const closePoint = (p, q, tolerance = 1e-12) => close(p[0], q[0], tolerance) && close(p[1], q[1], tolerance);
// The square [0, 2]² with its sides named, counterclockwise.
const drawnSquare = () => {
  const d = D.blankDrawing(2, 2);
  return D.moveLoop(d, 0, [1, 1]);
};

test("drawing curves: points, derivatives and samples of every kind", () => {
  const a = [0, 0];
  const b = [2, 0];
  assert.deepEqual(D.pointAt({ kind: "line", start: a, end: b }, 0.25), [0.5, 0]);
  const q = { kind: "quadratic", start: a, end: b, control: [1, 2] };
  assert.deepEqual(D.pointAt(q, 0.5), [1, 1]);
  const c = { kind: "cubic", start: a, end: b, control1: [0, 1], control2: [2, 1] };
  assert.deepEqual(D.pointAt(c, 0.5), [1, 0.75]);
  const arc = { kind: "arc", center: [1, 0], radius_x: 1, radius_y: 1, rotation: 0, start_angle: Math.PI, sweep_angle: -Math.PI, start: a, end: b };
  assert.ok(closePoint(D.pointAt(arc, 0.5), [1, 1]), "an arc clockwise over the top");
  // The derivative against a difference quotient.
  for (const curve of [q, c, arc]) {
    const h = 1e-6;
    const d = D.derivativeAt(curve, 0.3);
    const p1 = D.pointAt(curve, 0.3 + h);
    const p0 = D.pointAt(curve, 0.3 - h);
    assert.ok(closePoint(d, [(p1[0] - p0[0]) / (2 * h), (p1[1] - p0[1]) / (2 * h)], 1e-6), curve.kind);
  }
  // A Bezier's samples end exactly on its vertices.
  const samples = D.sampleCurve({ ...c, end: [0.1 + 0.2, 0] });
  assert.equal(samples.length, 2 * (D.CURVE_SAMPLES + 1));
  assert.equal(samples[samples.length - 2], 0.1 + 0.2);
  assert.deepEqual(D.sampleCurve({ kind: "line", start: a, end: b }), [0, 0, 2, 0]);
});

test("drawing: the nearest point of a curve, and hit testing", () => {
  const arc = { kind: "arc", center: [0, 0], radius_x: 1, radius_y: 1, rotation: 0, start_angle: 0, sweep_angle: Math.PI, start: [1, 0], end: [-1, 0] };
  const found = D.nearestOnCurve(arc, [0, 2]);
  assert.ok(close(found.t, 0.5, 1e-9) && closePoint(found.point, [0, 1], 1e-9) && close(found.distance, 1, 1e-9), JSON.stringify(found));
  const line = D.nearestOnCurve({ kind: "line", start: [0, 0], end: [4, 0] }, [1, 3]);
  assert.deepEqual([line.t, line.point, line.distance], [0.25, [1, 0], 3]);
  const d = drawnSquare();
  // A vertex wins over its edges; an edge has its parameter.
  assert.deepEqual(D.hitTest(d, [2.05, 0.02], 0.1), { type: "vertex", loop: 0, vertex: 1, point: [2, 0] });
  const edge = D.hitTest(d, [1, 2.04], 0.1);
  assert.equal(edge.type, "edge");
  assert.equal(edge.edge, 2);
  assert.ok(close(edge.t, 0.5));
  assert.equal(D.hitTest(d, [1, 1], 0.1), null, "nothing in the middle");
  // A selected edge shows its handles first: the bend handle of a line,
  // and its loop's move handle.
  const selected = [{ loop: 0, edge: 0 }];
  assert.equal(D.hitTest(d, [1, 0.03], 0.1, selected).type, "bend");
  assert.equal(D.hitTest(d, [1, 1.02], 0.1, selected).type, "move");
  const curved = D.setEdgeKind(d, 0, 0, "cubic");
  const handles = D.edgeHandles(curved, 0, 0);
  assert.deepEqual(handles.map((h) => h.which), ["control1", "control2"]);
  assert.equal(D.hitTest(curved, handles[1].point, 0.1, selected).which, "control2");
});

test("drawing: snapping to the grid and to corners", () => {
  assert.equal(D.gridStep(2.236), 0.05);
  assert.equal(D.gridStep(100), 2);
  assert.equal(D.gridStep(0.3), 0.01);
  assert.deepEqual(D.snapToGrid([0.31, -0.29], 0.1), [0.3, -0.3]);
  assert.deepEqual(D.snapToGrid([7.4, 2.6], 5), [5, 5]);
  assert.deepEqual(D.snapToGrid([0.123, 4], null), [0.123, 4]);
  const d = drawnSquare();
  // Near a corner: exactly the corner, on the edge pointed at.
  const corner = D.drawingSplitPoint(d, [1.97, 0.01], 0.05);
  assert.deepEqual(corner, { loop: 0, edge: 0, t: 1, vertex: 1, point: [2, 0], region: "bottom" });
  const right = D.drawingSplitPoint(d, [2.03, 0.04], 0.05);
  assert.deepEqual([right.edge, right.t, right.vertex, right.region], [1, 0, 1, "right"]);
  const inside = D.drawingSplitPoint(d, [0.5, -0.1], 0.05);
  assert.deepEqual([inside.edge, inside.vertex, inside.region], [0, null, "bottom"]);
  assert.ok(close(inside.t, 0.25));
  // The server's view: curves sampled, a mesh file's edges.
  const view = { kind: "curves", loops: [{ curves: [{ region: "a", points: [0, 0, 1, 0] }, { region: "b", points: [1, 0, 1, 1, 0, 0] }] }] };
  assert.deepEqual(D.viewSplitPoint(view, [0.98, 0.01], 0.05), { point: [1, 0], region: "a", corner: true });
  assert.deepEqual(D.viewSplitPoint(view, [0.5, 0.01], 0.05), { point: [0.5, 0], region: "a", corner: false });
  const meshView = { kind: "mesh_file", regions: [{ name: "r" }], boundary_edges: [[0, 0, 1, 0, 0]] };
  assert.deepEqual(D.viewSplitPoint(meshView, [0.7, 0.2], 0.01), { point: [1, 0], region: "r", corner: true });
});

test("drawing: arcs through three points and arcs that follow their ends", () => {
  const arc = D.arcThrough([1, 0], [0, 1], [-1, 0]);
  assert.ok(closePoint(arc.center, [0, 0]) && close(arc.radius_x, 1) && close(arc.sweep_angle, Math.PI), JSON.stringify(arc));
  const cw = D.arcThrough([1, 0], [0, -1], [-1, 0]);
  assert.ok(close(cw.sweep_angle, -Math.PI), "clockwise through the bottom");
  const big = D.arcThrough([1, 0], [-1, 0], [0, 1]);
  assert.ok(close(big.sweep_angle, -1.5 * Math.PI), "three quarters, through the far side");
  assert.equal(D.arcThrough([0, 0], [1, 1e-14], [2, 0]), null, "a straight line has no arc");
  // Moving an end: the arc is scaled and turned with its chord, its ends
  // land on the new ends.
  const curve = { ...arc, start: [1, 0], end: [-1, 0] };
  const moved = D.followEnds(curve, [1, 0], [-1, 0], [1, 0], [1, 4]);
  assert.ok(closePoint(D.pointAt(moved, 0), [1, 0]) && closePoint(D.pointAt(moved, 1), [1, 4]), JSON.stringify(moved));
  assert.ok(close(moved.radius_x, 2) && close(moved.radius_y, 2));
  // A closed arc turns and scales about its centre.
  const circle = D.circleLoop([0, 0], 1, "c").edges[0];
  const grown = D.followEnds(circle, [1, 0], [1, 0], [0, 2], [0, 2]);
  assert.ok(close(grown.radius_x, 2) && closePoint(D.pointAt(grown, 0), [0, 2]) && closePoint(grown.center, [0, 0]));
});

test("drawing edits: move, bend, kinds, insert and delete vertices", () => {
  const d = drawnSquare();
  // Dragging a vertex moves both of its edges (they take their ends from
  // it); the original is not changed.
  const moved = D.moveVertex(d, 0, 2, [3, 3]);
  assert.deepEqual(moved.loops[0].vertices[2], [3, 3]);
  assert.deepEqual(d.loops[0].vertices[2], [2, 2]);
  // An arc next to it follows; a cubic's handle moves with its vertex.
  let arcs = D.setEdgeKind(d, 0, 1, "arc");
  const bulge = D.pointAt(D.edgeCurve(arcs.loops[0], 1), 0.5);
  assert.ok(closePoint(bulge, [2.3, 1]), `outward by 0.15 of the edge: ${bulge}`);
  arcs = D.moveVertex(arcs, 0, 2, [2, 4]);
  const followed = D.edgeCurve(arcs.loops[0], 1);
  assert.ok(closePoint(D.pointAt(followed, 1), [2, 4]) && closePoint(D.pointAt(followed, 0), [2, 0]));
  const cubic = D.setEdgeKind(d, 0, 1, "cubic");
  const before = cubic.loops[0].edges[1].control2;
  const after = D.moveVertex(cubic, 0, 2, [2.5, 2]).loops[0].edges[1].control2;
  assert.deepEqual(after, [before[0] + 0.5, before[1]]);
  // Bending a line makes the arc through the dragged point.
  const bent = D.bendEdge(d, 0, 0, [1, -0.5]);
  assert.ok(closePoint(D.pointAt(D.edgeCurve(bent.loops[0], 0), 0.5), [1, -0.5]));
  assert.equal(D.bendEdge(d, 0, 0, [1, 0]), d, "a straight drag changes nothing");
  // Kinds: a quadratic as a cubic is the same curve.
  const quad = D.setEdgeKind(d, 0, 0, "quadratic");
  const asCubic = D.setEdgeKind(quad, 0, 0, "cubic");
  for (const t of [0.2, 0.5, 0.9]) assert.ok(closePoint(D.pointAt(D.edgeCurve(quad.loops[0], 0), t), D.pointAt(D.edgeCurve(asCubic.loops[0], 0), t)));
  assert.equal(D.setEdgeKind(asCubic, 0, 0, "line").loops[0].edges[0].kind, "line");
  const circle = { source: "drawing", loops: [D.circleLoop([0, 0], 1, "c")] };
  assert.throws(() => D.setEdgeKind(circle, 0, 0, "line"), /full circle/);
  const grown = D.bendEdge(circle, 0, 0, [-3, 0]);
  assert.ok(close(grown.loops[0].edges[0].radius_x, 3) && closePoint(grown.loops[0].vertices[0], [3, 0]));
  // Inserting a vertex splits the curve exactly where it was.
  for (const kind of ["line", "arc", "quadratic", "cubic"]) {
    const shaped = kind === "line" ? d : D.setEdgeKind(d, 0, 0, kind);
    const curve = D.edgeCurve(shaped.loops[0], 0);
    const { drawing: split, vertex } = D.insertVertex(shaped, 0, 0, 0.3);
    assert.equal(vertex, 1);
    assert.equal(split.loops[0].vertices.length, 5);
    assert.ok(closePoint(split.loops[0].vertices[1], D.pointAt(curve, 0.3)), kind);
    for (const [t, piece, u] of [
      [0.15, 0, 0.5],
      [0.65, 1, 0.5],
    ])
      assert.ok(closePoint(D.pointAt(curve, t), D.pointAt(D.edgeCurve(split.loops[0], piece), u)), `${kind} keeps its shape`);
    assert.equal(split.loops[0].edges[1].region, "bottom");
  }
  // Deleting a vertex joins its edges with a line; three stay.
  const fewer = D.deleteVertex(D.insertVertex(d, 0, 0, 0.5).drawing, 0, 1);
  assert.deepEqual(fewer, d);
  assert.throws(() => D.deleteVertex(D.deleteVertex(d, 0, 0), 0, 0), /three vertices/);
});

test("drawing edits: regions, loops and a split at a point", () => {
  const d = drawnSquare();
  assert.deepEqual(D.drawingRegions(d), ["bottom", "right", "top", "left"]);
  assert.equal(D.uniqueName(["a", "a_2"], "a"), "a_3");
  assert.equal(D.uniqueName(["a", "a_2"], "a_2"), "a_3");
  assert.equal(D.uniqueName(["a"], "b"), "b");
  const walls = D.assignRegion(d, [{ loop: 0, edge: 1 }, { loop: 0, edge: 3 }], "walls");
  assert.deepEqual(D.drawingRegions(walls), ["bottom", "walls", "top"]);
  assert.deepEqual(D.drawingRegions(D.renameRegion(walls, "walls", "top")), ["bottom", "top"]);
  // A split inside an edge inserts a vertex; the edge's run is cut there.
  const split = D.splitRegion(d, 0, 0, 0.25, "bottom", "bottom_2");
  assert.deepEqual(split.loops[0].vertices[1], [0.5, 0]);
  assert.deepEqual(split.loops[0].edges.map((e) => e.region), ["bottom", "bottom_2", "right", "top", "left"]);
  // At a vertex inside a run of two edges: no new vertex.
  const merged = D.renameRegion(d, "right", "bottom");
  const atCorner = D.splitRegion(merged, 0, 0, 1, "floor", "wall");
  assert.deepEqual(atCorner.loops[0].edges.map((e) => e.region), ["floor", "wall", "top", "left"]);
  assert.equal(atCorner.loops[0].vertices.length, 4);
  // At an end of the run there is nothing to split.
  assert.equal(D.splitRegion(d, 0, 0, 1, "a", "b"), null);
  assert.equal(D.splitRegion(d, 0, 1, 0, "a", "b"), null);
  // A whole loop starts at vertex 0.
  const ring = D.renameRegion(D.renameRegion(D.renameRegion(d, "right", "bottom"), "top", "bottom"), "left", "bottom");
  const lid = D.splitRegion(ring, 0, 2, 0, "rest", "lid");
  assert.deepEqual(lid.loops[0].edges.map((e) => e.region), ["rest", "rest", "lid", "lid"]);
  assert.equal(D.splitRegion(ring, 0, 0, 0, "a", "b"), null, "vertex 0 is its end");
  // New loops: inside the square a hole, outside another piece.
  assert.equal(D.newLoopRegion(d, [1, 1]), "hole");
  assert.equal(D.newLoopRegion(d, [5, 5]), "boundary");
  const holed = D.addLoop(d, D.circleLoop([1, 1], 0.4, "hole"));
  assert.equal(D.newLoopRegion(holed, [1.1, 1]), "boundary", "inside the hole is outside the domain");
  assert.equal(D.newLoopRegion(holed, [0.2, 0.2]), "hole_2");
  const rect = D.rectangleLoop([3, 2], [1, 0], "r");
  assert.deepEqual(rect.vertices, [[1, 0], [3, 0], [3, 2], [1, 2]]);
  assert.deepEqual(D.polygonLoop([[0, 0], [1, 0], [0, 1]], "t").edges.length, 3);
  const moved = D.moveLoop(holed, 1, [0.5, 0]);
  assert.deepEqual(moved.loops[1].edges[0].center, [1.5, 1]);
  assert.deepEqual(moved.loops[1].vertices[0], [1.9, 1]);
  assert.equal(D.deleteLoop(holed, 1).loops.length, 1);
  assert.throws(() => D.deleteLoop(d, 0), /one loop/);
  const box = D.drawingBounds(holed);
  assert.deepEqual(box, { min: [0, 0], max: [2, 2] });
  // Errors' paths name the place; selections are checked against edits.
  assert.deepEqual(D.drawingErrorTarget("geometry.loops[1].edges[3]"), { loop: 1, edge: 3 });
  assert.deepEqual(D.drawingErrorTarget("geometry.loops[0].vertices[2][1]"), { loop: 0, vertex: 2 });
  assert.deepEqual(D.drawingErrorTarget("geometry.loops[2]"), { loop: 2 });
  assert.equal(D.drawingErrorTarget("geometry.svg"), null);
  assert.ok(D.selectionValid(d, { loop: 0, vertex: 3 }));
  assert.ok(!D.selectionValid(d, { loop: 0, vertex: 4 }));
  assert.ok(!D.selectionValid(d, { edges: [{ loop: 1, edge: 0 }] }));
});

test("conditions follow a drawing's region changes and a replaced split", () => {
  const base = { geometry: drawnSquare(), problem: { boundary_conditions: { bottom: { g: "1" }, top: { g: "2" } } } };
  const geometry = D.renameRegion(base.geometry, "bottom", "floor");
  const renamed = applyDrawingChange(base, geometry, { op: "rename", from: "bottom", to: "floor" });
  assert.deepEqual({ ...renamed.problem.boundary_conditions }, { top: { g: "2" }, floor: { g: "1" } });
  assert.equal(renamed.geometry.loops[0].edges[0].region, "floor");
  assert.equal(renamed.view, undefined, "no edit record for a drawing");
  const split = applyDrawingChange(base, base.geometry, { op: "split", region: "top", first: "top", second: "lid" });
  assert.deepEqual(split.problem.boundary_conditions.lid, { g: "2" });
  // A shape's split renamed right after: the edit is replaced, and the
  // conditions go to the new names.
  const shape = { geometry: { source: "shape", region_edits: [] }, problem: { boundary_conditions: { top: { g: "2" } } } };
  const once = applyRegionEdit(shape, { op: "split", region: "top", fraction: 0.5, first: "top", second: "top_2" });
  const named = replaceLastRegionEdit(once, { op: "split", region: "top", fraction: 0.5, first: "top", second: "inlet" });
  assert.deepEqual(named.geometry.region_edits, [{ op: "split", region: "top", fraction: 0.5, first: "top", second: "inlet" }]);
  assert.deepEqual({ ...named.problem.boundary_conditions }, { top: { g: "2" }, inlet: { g: "2" } });
  // A mesh file's split at a node moves conditions as a split does.
  const node = applyRegionEdit(shape, { op: "split_at_node", region: "top", node: 4, first: "a", second: "b" });
  assert.deepEqual(Object.keys(node.problem.boundary_conditions), ["a", "b"]);
  // A drawing case reads back; one without loops gets a blank one.
  const defaults = { geometry: { source: "shape", shape: "rectangle" }, mesh: { spec: {}, discretizer: { type: "x" }, triangulator: { type: "y" }, refiner: null, improvers: [] }, problem: {}, assembly: { t: 0 }, solve: { mode: "static", linear_solver: { type: "z" }, static: {}, time: { method: { type: "m" }, step: { type: "s" } } } };
  const read = normalizeCase({ format: "pde2d-app-case", geometry: drawnSquare() }, defaults);
  assert.deepEqual(read.caseData.geometry, drawnSquare());
  assert.ok(!read.replaced.includes("geometry.loops"));
  // An incomplete drawing is kept as it is (the server names what is
  // missing).
  for (const geometry of [{ source: "drawing", loops: [] }, { source: "drawing" }, { source: "drawing", loops: [{ vertices: [], edges: [{ kind: "line", region: "a" }] }] }]) {
    const kept = normalizeCase({ format: "pde2d-app-case", geometry }, defaults);
    assert.deepEqual(kept.caseData.geometry, geometry);
    assert.ok(!kept.replaced.some((path) => path.startsWith("geometry")), kept.replaced.join(","));
  }
});

test("renaming a split part keeps the conditions edited since the split", () => {
  for (const op of ["split", "split_at_node"]) {
    const start = { geometry: { source: "shape", region_edits: [] }, problem: { boundary_conditions: { bottom: { alpha: "1", g: "0" }, top: { g: "2" } } } };
    const where = op === "split" ? { fraction: 0.5 } : { node: 7 };
    let c = applyRegionEdit(start, { op, region: "bottom", ...where, first: "bottom", second: "bottom_2" });
    // The user sets the new part's condition, then renames it.
    c.problem.boundary_conditions.bottom_2 = { alpha: "1", g: "5" };
    c = renameRegionByEdit(c, "bottom_2", "inlet");
    assert.deepEqual(c.geometry.region_edits, [{ op, region: "bottom", ...where, first: "bottom", second: "inlet" }], "the split takes the name");
    assert.deepEqual({ ...c.problem.boundary_conditions }, { bottom: { alpha: "1", g: "0" }, top: { g: "2" }, inlet: { alpha: "1", g: "5" } }, op);
    // The kept part renamed too: its own condition, also edited.
    c.problem.boundary_conditions.bottom = { g: "9" };
    c = renameRegionByEdit(c, "bottom", "floor");
    assert.equal(c.geometry.region_edits.length, 1);
    assert.deepEqual(c.problem.boundary_conditions.floor, { g: "9" });
    assert.deepEqual(c.problem.boundary_conditions.inlet, { alpha: "1", g: "5" });
    // Undoing the split still restores the condition from before it.
    const undone = undoRegionEdit(c);
    assert.deepEqual({ ...undone.problem.boundary_conditions }, { bottom: { alpha: "1", g: "0" }, top: { g: "2" } }, `${op} undone`);
  }
  // Another region, or a name the split used: a plain rename edit.
  const start = { geometry: { source: "shape", region_edits: [] }, problem: { boundary_conditions: { a: { g: "1" }, b: { g: "3" } } } };
  const split = applyRegionEdit(start, { op: "split", region: "a", fraction: 0.5, first: "a", second: "a_2" });
  const other = renameRegionByEdit(split, "b", "c");
  assert.equal(other.geometry.region_edits.length, 2);
  assert.deepEqual(other.problem.boundary_conditions.c, { g: "3" });
});

test("drawing: keyboard selection, its words, and a split at a typed fraction", () => {
  const d = D.addLoop(drawnSquare(), D.circleLoop([1, 1], 0.4, "hole"));
  // ] from nothing: vertex 0 of loop 0, then edge 0, vertex 1, ...; [ back.
  let sel = D.stepSelection(d, null, 1);
  assert.deepEqual(sel, { loop: 0, vertex: 0 });
  sel = D.stepSelection(d, sel, 1);
  assert.deepEqual(sel, { edges: [{ loop: 0, edge: 0 }] });
  assert.deepEqual(D.stepSelection(d, sel, 1), { loop: 0, vertex: 1 });
  assert.deepEqual(D.stepSelection(d, { loop: 0, vertex: 0 }, -1), { edges: [{ loop: 0, edge: 3 }] }, "wraps around");
  assert.deepEqual(D.stepSelection(d, null, -1), { edges: [{ loop: 0, edge: 3 }] });
  assert.deepEqual(D.stepLoop(d, sel, 1), { loop: 1, vertex: 0 });
  assert.deepEqual(D.stepLoop(d, { loop: 1, vertex: 0 }, 1), { loop: 0, vertex: 0 });
  assert.deepEqual(D.stepLoop(d, null, -1), { loop: 1, vertex: 0 });
  assert.deepEqual(D.stepSelection(d, { loop: 1, vertex: 0 }, 1), { edges: [{ loop: 1, edge: 0 }] });
  assert.equal(D.describeSelection(d, { loop: 0, vertex: 2 }), "Loop 0, vertex 2: x 2, y 2");
  assert.equal(D.describeSelection(d, { edges: [{ loop: 1, edge: 0 }] }), "Loop 1, edge 0: arc, region hole");
  assert.equal(D.describeSelection(d, { edges: [{ loop: 0, edge: 0 }, { loop: 0, edge: 1 }] }), "2 edges selected");
  assert.equal(D.describeSelection(d, null), "Nothing selected");
  // A typed split: a vertex at that fraction of the length, the rest named.
  const split = D.splitEdgeAt(d, 0, 0, 0.25, "inlet");
  assert.deepEqual(split.loops[0].vertices[1], [0.5, 0]);
  assert.deepEqual(split.loops[0].edges.map((e) => e.region), ["bottom", "inlet", "right", "top", "left"]);
  assert.throws(() => D.splitEdgeAt(d, 0, 0, 1, "x"), /strictly between/);
  // The fraction is of the length, also on a curve whose t is not uniform.
  const curved = D.setEdgeKind(drawnSquare(), 0, 0, "quadratic");
  const curve = D.edgeCurve(curved.loops[0], 0);
  const t = D.parameterAtLengthFraction(curve, 0.3);
  const length = (a, b, n = 2000) => {
    let sum = 0;
    for (let i = 0; i < n; i++) {
      const p = D.pointAt(curve, a + ((b - a) * i) / n);
      const q = D.pointAt(curve, a + ((b - a) * (i + 1)) / n);
      sum += Math.hypot(q[0] - p[0], q[1] - p[1]);
    }
    return sum;
  };
  assert.ok(Math.abs(length(0, t) / length(0, 1) - 0.3) < 1e-4, String(t));
  assert.equal(D.parameterAtLengthFraction({ kind: "line", start: [0, 0], end: [1, 0] }, 0.3), 0.3);
  // On a full circle the split gives two arcs.
  const ring = D.splitEdgeAt(d, 1, 0, 0.5, "hole_2");
  assert.equal(ring.loops[1].vertices.length, 2);
  assert.ok(closePoint(ring.loops[1].vertices[1], [0.6, 1]));
});

test("drawing: incomplete parts are skipped, never thrown on", () => {
  const broken = {
    source: "drawing",
    loops: [
      { vertices: [], edges: [{ kind: "line", region: "a" }] },
      { vertices: [[0, 0], [2, 0], [2, 2], [0, 2]], edges: [{ kind: "line", region: "b" }, { kind: "quadratic", region: "c" }, { kind: "cubic", region: "d", control1: [1, 3] }, { kind: "arc", region: "e", center: [1, 1] }] },
      { vertices: [[5, 5], "x", [6, 6]], edges: [{ kind: "line", region: "f" }, null, { kind: "spline", region: "g" }] },
      "not a loop",
    ],
  };
  // Only the complete edge (b) is a curve.
  assert.equal(D.edgeCurve(broken.loops[0], 0), null);
  assert.ok(D.edgeCurve(broken.loops[1], 0));
  for (const e of [1, 2, 3]) assert.equal(D.edgeCurve(broken.loops[1], e), null, `edge ${e}`);
  assert.equal(D.edgeCurve(broken.loops[2], 0), null, "a bad vertex");
  assert.deepEqual(D.drawingBounds(broken), { min: [0, 0], max: [2, 0] });
  assert.deepEqual(D.drawingRegions(broken), ["a", "b", "c", "d", "e", "f", "g"]);
  assert.equal(D.nearestEdge(broken, [1, 0.1]).edge, 0);
  assert.equal(D.hitTest(broken, [1, 1.95], 0.1), null, "an incomplete edge is not hit");
  assert.equal(D.hitTest(broken, [5, 5], 0.1).type, "vertex");
  assert.equal(D.drawingSplitPoint(broken, [1, 0.01], 0.05).region, "b");
  assert.doesNotThrow(() => D.newLoopRegion(broken, [1, 1]));
  assert.deepEqual(D.edgeHandles(broken, 1, 1), []);
  assert.ok(!D.selectionValid(broken, { loop: 2, vertex: 1 }));
  assert.equal(D.describeSelection(broken, { loop: 2, vertex: 1 }), "Nothing selected");
  assert.equal(D.stepSelection(broken, null, 1), null, "loop 0 has no vertices");
  assert.deepEqual(D.stepSelection(broken, { loop: 1, vertex: 0 }, 1), { edges: [{ loop: 1, edge: 0 }] });
  assert.deepEqual(D.stepLoop(broken, { loop: 1, vertex: 0 }, 1), { loop: 2, vertex: 0 }, "loop 3 is not one");
  assert.deepEqual(D.stepLoop(broken, { loop: 2, vertex: 0 }, 1), { loop: 1, vertex: 0 }, "loop 0 has no vertex");
  assert.equal(D.stepLoop({ loops: [{ vertices: [] }] }, null, 1), null);
  // A vertex next to an incomplete edge still moves; the edge stays.
  const moved = D.moveVertex(broken, 1, 2, [3, 3]);
  assert.deepEqual(moved.loops[1].vertices[2], [3, 3]);
  assert.deepEqual(moved.loops[1].edges[2], broken.loops[1].edges[2]);
  assert.throws(() => D.setEdgeKind(broken, 1, 1, "line"), /incomplete/);
  assert.throws(() => D.insertVertex(broken, 1, 3, 0.5), /incomplete/);
  // A drawing without loops: nothing to draw, a loop can be added.
  for (const empty of [{ source: "drawing" }, { source: "drawing", loops: "x" }]) {
    assert.equal(D.drawingBounds(empty), null);
    assert.equal(D.nearestEdge(empty, [0, 0]), null);
    assert.equal(D.addLoop(empty, D.circleLoop([0, 0], 1, "c")).loops.length, 1);
  }
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

test("a run on several meshes: values split by each snapshot's count, a stand-in only on the same mesh", () => {
  const buffer = new ArrayBuffer(4 * 5);
  const view = new DataView(buffer);
  [1, 2, 3, 4, 5].forEach((v, i) => view.setFloat32(4 * i, v, true));
  const [a, b] = splitCounted(buffer, [2, 3]);
  assert.deepEqual([...a], [1, 2]);
  assert.deepEqual([...b], [3, 4, 5]);
  assert.throws(() => splitCounted(buffer, [2, 2]), /expected 16/);
  assert.equal(snapshotNodes({ nodes: 7, snapshot_nodes: [2, 3] }, 1), 3);
  assert.equal(snapshotNodes({ nodes: 7 }, 1), 7);
  // Snapshot 2 is not loaded; 1 is nearer, but only 3 is on its mesh.
  const values = [null, [1], null, [2]];
  const mesh = [0, 0, 1, 1];
  assert.equal(nearestLoaded(values, 2), 1);
  assert.equal(nearestLoaded(values, 2, (i) => mesh[i] === mesh[2]), 3);
  assert.equal(nearestLoaded([null, [1], null], 2, (i) => mesh[i] === mesh[2]), -1);
  assert.equal(isTimeRun({ mode: "adaptive_time" }), true);
  assert.equal(isTimeRun({ mode: "time" }), true);
  assert.equal(isTimeRun({ mode: "adaptive" }), false);
  assert.equal(isTimeRun(null), false);
});
