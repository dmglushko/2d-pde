// Overlapping requests in the page (app/web/js/actions.js), with a mocked
// fetch: an answer to an older request must not change the state. Run by
// CTest as app_web_actions when Node.js is installed.
import assert from "node:assert/strict";
// Storage that counts its writes (the page keeps the case there).
const storage = new Map();
let writes = 0;
globalThis.localStorage = {
  getItem: (key) => (storage.has(key) ? storage.get(key) : null),
  setItem: (key, value) => {
    writes++;
    storage.set(key, String(value));
  },
  removeItem: (key) => storage.delete(key),
};
const { applyPreset, caseChanged, followedRun, runAll, runStage, runWithPrerequisites, scheduleProblemCheck, showField, stopRun } = await import("../web/js/actions.js");
const { applyRegionEdit, stageKey } = await import("../web/js/core.js");
const { settings, setSetting } = await import("../web/js/settings.js");
const { currentError, currentKey, editCase, flushPersist, outdatedError, setCase, stageStatus, store } = await import("../web/js/state.js");

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
  editCase((c) => (c.geometry.shape = "circle")); // edited while the request runs
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

const fullCase = () => ({ format: "pde2d-app-case", version: 1, geometry: { source: "shape", shape: "rectangle" }, mesh: { m: 1 }, problem: { boundary_conditions: { left: { alpha: "1", g: "0" } } }, assembly: { t: 0 }, solve: { mode: "static" } });
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
  const summary = { job: 7, state: "done", mode: "time", times: [0, 1], min: -1, max: 2, values: { path: "/api/solve/values", format: "float32le", snapshots: 2, nodes: 3 } };
  (await next("/api/solve")).answer(summary);
  const values = await next("/api/solve/values");
  assert.deepEqual(values.body, { job: 7, first: 0, count: 2 }, "the run's snapshots by range");
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

const layout = (snapshots) => ({ path: "/api/solve/values", format: "float32le", snapshots, nodes: 2 });
const timeRun = (state, snapshots, t, extra = {}) => ({ job: 3, state, mode: "time", t0: 0, t1: 1, t, snapshots_total: 5, times: [0, 0.25, 0.5, 0.75, 1].slice(0, snapshots), min: 0, max: snapshots, reports: [{ metrics: { steps: snapshots } }], values: layout(snapshots), ...extra });

test("a time run streams its snapshots and follows the newest", async () => {
  start();
  const run = runStage("solve");
  (await next("/api/solve")).answer(timeRun("running", 2, 0.25));
  (await next("/api/solve/values")).answer(float32([0, 0, 1, 1]));
  await flush();
  assert.equal(followedRun()?.job, 3, "followed");
  assert.equal(stageStatus("solve"), "busy");
  assert.equal(store.solveFrame, 1);
  // While it runs, a problem check is not held back (the server answers
  // between chunks).
  scheduleProblemCheck({ now: true });
  (await next("/api/problem")).answer({ ok: true });
  const progress = await next("/api/solve/progress");
  assert.deepEqual(progress.body, { job: 3 });
  progress.answer(timeRun("running", 4, 0.75));
  const more = await next("/api/solve/values");
  assert.deepEqual(more.body, { job: 3, first: 2, count: 2 }, "only the new snapshots");
  more.answer(float32([2, 2, 3, 3]));
  await sleep(10);
  const data = store.results.solve.data;
  assert.equal(data.values.length, 4);
  assert.equal(store.solveFrame, 3, "the newest is shown");
  assert.equal(data.scale.max, 4, "the scale follows the range so far");
  // The slider was moved: the picture stays there.
  store.solveFrameChosen = true;
  store.solveFrame = 1;
  (await next("/api/solve/progress")).answer(timeRun("done", 5, 1));
  (await next("/api/solve/values")).answer(float32([4, 4]));
  assert.equal(await run, true);
  assert.equal(data.values.length, 5);
  assert.equal(store.solveFrame, 1, "not moved");
  assert.equal(data.state, "done");
  assert.equal(stageStatus("solve"), "done");
  assert.equal(followedRun(), null);
});

test("Stop stops the run and keeps what arrived", async () => {
  start();
  const run = runStage("solve");
  (await next("/api/solve")).answer(timeRun("running", 2, 0.25));
  (await next("/api/solve/values")).answer(float32([0, 0, 1, 1]));
  await flush();
  const stopping = stopRun();
  const stop = await next("/api/solve/stop");
  assert.deepEqual(stop.body, { job: 3 });
  stop.answer(timeRun("stopped", 2, 0.25));
  await stopping;
  assert.equal(await run, null);
  assert.equal(store.results.solve.data.values.length, 2);
  assert.equal(stageStatus("solve"), "stopped");
  assert.equal(store.busy.has("solve"), false);
});

test("a change of the solve's inputs stops its run", async () => {
  start();
  const run = runStage("solve");
  (await next("/api/solve")).answer(timeRun("running", 2, 0.25));
  (await next("/api/solve/values")).answer(float32([0, 0, 1, 1]));
  await flush();
  editCase((c) => (c.problem.boundary_conditions.left.g = "1"), "edit", { sections: ["problem"] });
  caseChanged();
  const stop = await next("/api/solve/stop");
  assert.deepEqual(stop.body, { job: 3 });
  stop.answer({});
  assert.equal(await run, null);
  assert.equal(store.busy.has("solve"), false);
  assert.equal(followedRun(), null);
});

test("a failure during a run is the solve's error", async () => {
  start();
  const run = runStage("solve");
  (await next("/api/solve")).answer(timeRun("running", 2, 0.25));
  (await next("/api/solve/values")).answer(float32([0, 0, 1, 1]));
  (await next("/api/solve/progress")).answer(timeRun("failed", 2, 0.25, { error: { stage: "solve", kind: "algorithm", message: "the linear solver failed" } }));
  assert.equal(await run, false);
  assert.equal(currentError("solve").message, "the linear solver failed");
  assert.equal(store.results.solve.data.values.length, 2, "the snapshots before stay");
});

test("a request carries only the sections its stage reads", async () => {
  start();
  editCase((c) => (c.geometry.svg = "x".repeat(1000)));
  const check = runStage("problem");
  const request = await next("/api/problem");
  assert.deepEqual(Object.keys(request.body).sort(), ["format", "problem", "version"]);
  request.answer({ ok: true });
  await check;
  const mesh = runStage("mesh");
  const meshRequest = await next("/api/mesh");
  assert.deepEqual(Object.keys(meshRequest.body).sort(), ["format", "geometry", "mesh", "version"]);
  meshRequest.answer({ mesh: {} });
  await mesh;
});

test("a check of a problem edited since is dropped", async () => {
  start();
  const check = runStage("problem");
  editCase((c) => (c.problem.parameters = { k: 1 }), "edit", { sections: ["problem"] });
  (await next("/api/problem")).answer({ ok: true });
  assert.equal(await check, null);
  assert.equal(store.results.problem, null, "not kept");
});

test("the check while typing follows the settings", async () => {
  start();
  setSetting("checkWhileTyping", false);
  scheduleProblemCheck();
  await sleep(settings.checkDelayMs + 50);
  assert.equal(pending.length, 0, "no check while typing");
  scheduleProblemCheck({ now: true });
  (await next("/api/problem")).answer({ ok: true });
  setSetting("checkWhileTyping", true);
  setSetting("checkDelayMs", 120);
  scheduleProblemCheck();
  await sleep(60);
  assert.equal(pending.length, 0, "not before the delay");
  (await next("/api/problem")).answer({ ok: true });
  setSetting("checkDelayMs", 350);
  assert.ok(JSON.parse(storage.get("pde2d-app-settings")).checkDelayMs === 350, "settings are kept");
});

test("edits write the case to storage after a pause, and at once when flushed", async () => {
  start();
  await sleep(700);
  const before = writes;
  for (let i = 0; i < 20; i++) editCase((c) => (c.problem.parameters = { k: i }), "edit", { sections: ["problem"] });
  assert.equal(writes, before, "not on every edit");
  flushPersist();
  assert.ok(writes > before, "written when flushed");
  assert.equal(JSON.parse(storage.get("pde2d-app-case")).problem.parameters.k, 19);
  const flushed = writes;
  await sleep(700);
  assert.equal(writes, flushed, "nothing left to write");
});

test("stage keys are computed per section and match stageKey", () => {
  start();
  for (const stage of ["geometry", "mesh", "problem", "assemble", "solve", "field"]) assert.equal(currentKey(stage), stageKey(store.caseData, stage));
  editCase((c) => (c.mesh.m = 5), "edit", { sections: ["mesh"] });
  assert.equal(currentKey("mesh"), stageKey(store.caseData, "mesh"));
  editCase((c) => (c.assembly.t = 2));
  assert.equal(currentKey("assemble"), stageKey(store.caseData, "assemble"));
});

test("a preset replaces the problem and the solve settings", async () => {
  start();
  const done = applyPreset({ id: "wave", label: "Wave" });
  const request = await next("/api/preset");
  assert.equal(request.body.preset, "wave");
  assert.deepEqual(Object.keys(request.body.case).sort(), ["format", "geometry", "problem", "solve", "version"]);
  request.answer({ ...fullCase(), problem: { equation: { m2: "1" }, boundary_conditions: { left: { alpha: "1", g: "0" } } }, solve: { mode: "time" } });
  assert.equal(await done, true);
  assert.equal(store.caseData.problem.equation.m2, "1");
  assert.equal(store.caseData.solve.mode, "time");
  assert.equal(store.caseData.mesh.m, 1, "the rest is kept");
  pending.length = 0;
});

test("run all skips fresh stages and ends colouring by the solution", async () => {
  start();
  store.view.colorBy = "none";
  fresh("geometry", { regions: [{ name: "left" }] });
  fresh("mesh", { stats: { nodes: 1 } });
  const done = runAll();
  (await next("/api/assemble")).answer({ assembly: {} });
  (await next("/api/solve")).answer({ job: 1, state: "done", mode: "static", times: [0], min: 0, max: 0, values: { path: "/api/solve/values", format: "float32le", snapshots: 1, nodes: 1 } });
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
