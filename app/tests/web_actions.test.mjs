// Overlapping requests in the page (app/web/js/actions.js), with a mocked
// fetch: an answer to an older request must not change the state. Run by
// CTest as app_web_actions when Node.js is installed.
import assert from "node:assert/strict";
import { runAll, runStage, runWithPrerequisites, scheduleProblemCheck, showField } from "../web/js/actions.js";
import { applyRegionEdit, stageKey } from "../web/js/core.js";
import { currentError, editCase, outdatedError, setCase, stageStatus, store } from "../web/js/state.js";

// Every fetch waits until the test answers it: answer(data) with JSON or
// an ArrayBuffer, fail(error, status) with an error body.
const pending = [];
globalThis.fetch = (path, options) =>
  new Promise((resolve) =>
    pending.push({
      path,
      body: JSON.parse(options.body),
      answer: (data) => resolve({ ok: true, json: async () => data, arrayBuffer: async () => data }),
      fail: (error, status = 422) => resolve({ ok: false, status, statusText: "", json: async () => ({ error }) }),
    }),
  );
const flush = () => new Promise((resolve) => setTimeout(resolve, 0));
const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms));
// Waits until a request to `path` is pending and takes it.
async function next(path) {
  for (let i = 0; i < 200; i++) {
    const index = pending.findIndex((p) => p.path === path);
    if (index >= 0) return pending.splice(index, 1)[0];
    await sleep(5);
  }
  throw new Error(`no request to ${path}; pending: ${pending.map((p) => p.path).join(", ")}`);
}
const float32 = (values) => {
  const buffer = new ArrayBuffer(4 * values.length);
  const view = new DataView(buffer);
  values.forEach((v, i) => view.setFloat32(4 * i, v, true));
  return buffer;
};
const geometryView = (...regions) => ({ kind: "curves", loops: [], regions: regions.map((name) => ({ name, length: 1 })), bounding_box: { min: [0, 0], max: [1, 1] }, domain_size: 1 });

store.catalog = { problem: { default_condition: { alpha: "1", g: "0" } } };
const rectangle = { format: "pde2d-app-case", geometry: { source: "shape", shape: "rectangle" }, problem: { boundary_conditions: { left: { alpha: "1", g: "0" } } } };
const circle = { format: "pde2d-app-case", geometry: { source: "shape", shape: "circle" }, problem: { boundary_conditions: { boundary: { beta_n: "1", g: "2" } } } };

const cases = [];
const test = (name, body) => cases.push([name, body]);

test("an answer for a case that was replaced is dropped", async () => {
  setCase(structuredClone(rectangle));
  const old = runStage("geometry");
  setCase(structuredClone(circle));
  const fresh = runStage("geometry");
  await flush();
  pending.splice(1, 1)[0].answer(geometryView("boundary"));
  assert.equal(await fresh, true);
  pending.splice(0, 1)[0].answer(geometryView("bottom", "right", "top", "left"));
  assert.equal(await old, null, "superseded");
  assert.deepEqual(store.results.geometry.data.regions.map((r) => r.name), ["boundary"]);
  assert.deepEqual(store.caseData.problem.boundary_conditions, { boundary: { beta_n: "1", g: "2" } }, "the circle's condition survives");
  assert.equal(store.busy.has("geometry"), false);
});

test("an older field answer does not replace a newer one", async () => {
  setCase(structuredClone(circle));
  const first = showField({ kind: "expression", text: "x" });
  const second = showField({ kind: "expression", text: "y" });
  await flush();
  assert.equal(store.busy.has("field"), true);
  pending.splice(1, 1)[0].answer({ label: "y", values: [] });
  assert.equal(await second, true);
  pending.splice(0, 1)[0].answer({ label: "x", values: [] });
  assert.equal(await first, null);
  assert.equal(store.results.field.data.label, "y");
  assert.equal(store.busy.has("field"), false);
});

test("an answer for an edited geometry does not touch the conditions", async () => {
  setCase(structuredClone(rectangle));
  const run = runStage("geometry");
  store.caseData.geometry.shape = "circle"; // edited while the request runs
  await flush();
  pending.splice(0, 1)[0].answer(geometryView("bottom", "right", "top", "left"));
  assert.equal(await run, true, "kept as a (stale) result");
  assert.deepEqual(Object.keys(store.caseData.problem.boundary_conditions), ["left"], "no conditions synced for a geometry that changed");
});

test("an edit saved and reopened before its geometry ran keeps its conditions", async () => {
  const start = { format: "pde2d-app-case", geometry: { source: "shape", shape: "rectangle", region_edits: [] }, problem: { boundary_conditions: { bottom: { alpha: "1", g: "11" }, top: { alpha: "1", g: "22" } } } };
  const edited = applyRegionEdit(start, { op: "rename", from: "bottom", to: "floor" });
  setCase(JSON.parse(JSON.stringify(edited))); // saved and opened again
  const run = runStage("geometry");
  await flush();
  pending.splice(0, 1)[0].answer(geometryView("floor", "top"));
  assert.equal(await run, true);
  assert.equal(store.caseData.problem.boundary_conditions.floor.g, "11");
  assert.equal(store.caseData.problem.boundary_conditions.top.g, "22");
});

const fullCase = () => ({ format: "pde2d-app-case", geometry: { source: "shape", shape: "rectangle" }, mesh: { m: 1 }, problem: { boundary_conditions: { left: { alpha: "1", g: "0" } } }, assembly: { t: 0 }, solve: { mode: "static" } });
// A new case with no results (as main.js loads one).
const start = () => {
  pending.length = 0;
  for (const stage of Object.keys(store.results)) store.results[stage] = null;
  setCase(fullCase());
};
const fresh = (stage, data = {}) => (store.results[stage] = { key: stageKey(store.caseData, stage), data });

test("a problem check waits while a long stage runs, and runs once after", async () => {
  start();
  const mesh = runStage("mesh");
  await flush();
  scheduleProblemCheck();
  scheduleProblemCheck();
  await sleep(400);
  assert.equal(pending.filter((p) => p.path === "/api/problem").length, 0, "no check while the mesh runs");
  (await next("/api/mesh")).answer({ mesh: {}, stats: { nodes: 3 } });
  await mesh;
  const check = await next("/api/problem");
  check.answer({ ok: true });
  await flush();
  assert.equal(pending.length, 0, "one check only");
  assert.equal(stageStatus("problem"), "done");
});

test("a stage runs after its prerequisites that are not fresh", async () => {
  start();
  store.step = "problem";
  fresh("geometry", { regions: [{ name: "left" }] });
  const done = runWithPrerequisites("assemble");
  (await next("/api/mesh")).answer({ mesh: {}, stats: { nodes: 3 } });
  (await next("/api/assemble")).answer({ assembly: {} });
  assert.equal(await done, true);
  assert.equal(pending.length, 0, "the fresh geometry did not run again");
});

test("a failing prerequisite stops the run and opens its step", async () => {
  start();
  store.step = "solve";
  const done = runWithPrerequisites("solve");
  (await next("/api/geometry")).answer({ regions: [{ name: "left" }] });
  (await next("/api/mesh")).fail({ stage: "mesh", kind: "input", message: "mesh.spec: no" });
  assert.equal(await done, false);
  assert.equal(store.step, "mesh", "the failing step is open");
  assert.equal(pending.length, 0, "the solve did not run");
  assert.equal(currentError("mesh").message, "mesh.spec: no");
});

test("an error belongs to the input it was produced for", async () => {
  start();
  const run = runStage("mesh");
  (await next("/api/mesh")).fail({ stage: "mesh", kind: "input", message: "mesh.spec: no" });
  assert.equal(await run, false);
  assert.equal(stageStatus("mesh"), "failed");
  editCase((c) => (c.mesh.m = 2));
  assert.equal(currentError("mesh"), null, "not current once the mesh section changed");
  assert.equal(outdatedError("mesh").message, "mesh.spec: no");
  assert.equal(stageStatus("mesh"), "stale");
  editCase((c) => (c.mesh.m = 1));
  assert.ok(currentError("mesh"), "current again for the same input");
});

test("a solve fetches its values in binary and colours the picture", async () => {
  start();
  store.view.colorBy = "quality";
  const run = runStage("solve");
  const summary = { mode: "time", times: [0, 1], min: -1, max: 2, values: { path: "/api/solve/values", format: "float32le", snapshots: 2, nodes: 3 } };
  (await next("/api/solve")).answer(summary);
  const values = await next("/api/solve/values");
  assert.deepEqual(values.body, store.caseData, "the same case");
  values.answer(float32([0, 1, 2, -1, 0.5, 0]));
  assert.equal(await run, true);
  const data = store.results.solve.data;
  assert.ok(data.values[1] instanceof Float32Array);
  assert.deepEqual([...data.values[1]], [-1, 0.5, 0]);
  assert.equal(data.min, -1, "min and max from the summary");
  assert.equal(store.view.colorBy, "field");
  assert.equal(store.fieldSource, "solve");
  assert.equal(store.solveFrame, 1, "the last snapshot");
});

test("run all skips fresh stages and ends colouring by the solution", async () => {
  start();
  store.view.colorBy = "none";
  fresh("geometry", { regions: [{ name: "left" }] });
  fresh("mesh", { stats: { nodes: 1 } });
  const done = runAll();
  (await next("/api/assemble")).answer({ assembly: {} });
  (await next("/api/solve")).answer({ mode: "static", times: [0], min: 0, max: 0, values: { path: "/api/solve/values", format: "float32le", snapshots: 1, nodes: 1 } });
  (await next("/api/solve/values")).answer(float32([0]));
  assert.equal(await done, true);
  assert.equal(store.view.colorBy, "field");
  // A second Run all has nothing to do, and still ends on the solution.
  store.view.colorBy = "quality";
  assert.equal(await runAll(), true);
  assert.equal(pending.length, 0);
  assert.equal(store.view.colorBy, "field");
});

let failed = 0;
for (const [name, body] of cases) {
  try {
    await body();
    console.log(`PASS ${name}`);
  } catch (error) {
    failed++;
    console.log(`FAIL ${name}: ${error.message}`);
  }
}
console.log(`${cases.length - failed}/${cases.length} cases passed`);
process.exit(failed ? 1 : 0);
