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

test("cases are normalized before they are shown", () => {
  const defaults = { geometry: { source: "shape", shape: "rectangle" }, mesh: { spec: {}, discretizer: { type: "d" }, triangulator: { type: "t" }, refiner: { type: "r" }, improvers: [] }, problem: { equation: {} }, assembly: { t: 0 } };
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
