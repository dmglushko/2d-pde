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
  normalizeCase,
  reconcileConditions,
  triangleAngles,
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

test("conditions follow the regions through accepted edits", () => {
  const dirichlet = { alpha: "1", g: "0" };
  const neumann = { beta_n: "1", g: "0" };
  const base = { problem: { boundary_conditions: { left: neumann, right: dirichlet, gone: { beta_n: "2" } } } };
  // Plain sync: a new region gets the default, a vanished one is archived.
  let next = reconcileConditions(base, ["left", "right", "top"], dirichlet);
  assert.deepEqual(Object.keys(next.problem.boundary_conditions), ["left", "right", "top"]);
  assert.deepEqual(next.problem.boundary_conditions.top, dirichlet);
  assert.deepEqual(next.view.condition_archive.gone, { beta_n: "2" });
  assert.ok("gone" in base.problem.boundary_conditions, "the input is not changed");
  // The region comes back: its condition is restored from the archive.
  next = reconcileConditions(next, ["left", "right", "gone"], dirichlet);
  assert.deepEqual(next.problem.boundary_conditions.gone, { beta_n: "2" });
  assert.deepEqual(Object.keys(next.view.condition_archive), ["top"], "restored ones leave the archive; top is gone now");
  next = reconcileConditions(next, ["left", "right", "gone", "top"], dirichlet);
  assert.equal(next.view, undefined, "an empty archive is removed");
  // Rename: the new name keeps the condition; removing the edit restores the old name.
  next = reconcileConditions(base, ["walls", "right", "gone"], dirichlet, { op: "rename", from: "left", to: "walls" });
  assert.deepEqual(next.problem.boundary_conditions.walls, neumann);
  next = reconcileConditions(next, ["left", "right", "gone"], dirichlet);
  assert.deepEqual(next.problem.boundary_conditions.left, neumann, "restored after the edit was removed");
  // Split: both halves copy the whole; the old name may be one of them.
  next = reconcileConditions(base, ["left", "left_b", "right", "gone"], dirichlet, { op: "split", region: "left", fraction: 0.5, first: "left", second: "left_b" });
  assert.deepEqual(next.problem.boundary_conditions.left_b, neumann);
  assert.deepEqual(next.problem.boundary_conditions.left, neumann);
  // Merge: the target keeps its own; the source is archived.
  next = reconcileConditions(base, ["right", "gone"], dirichlet, { op: "merge", source: "left", target: "right" });
  assert.deepEqual(next.problem.boundary_conditions.right, dirichlet);
  assert.deepEqual(next.view.condition_archive.left, neumann);
});

test("cases are normalized before they are shown", () => {
  const defaults = { geometry: { source: "shape" }, mesh: { spec: {}, discretizer: { type: "d" }, triangulator: { type: "t" }, refiner: { type: "r" }, improvers: [] }, problem: { equation: {} }, assembly: { t: 0 } };
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
