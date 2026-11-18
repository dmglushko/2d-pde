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
const { applyPreset, caseChanged, followedRun, reattachRun, runAll, runStage, runWithPrerequisites, scheduleProblemCheck, showField, stopRun } = await import("../web/js/actions.js");
const { loadAround, retrySnapshots, snapshotState, snapshotValues } = await import("../web/js/snapshots.js");
const { applyRegionEdit } = await import("../web/js/pure/case_model.js");
const { stageKey } = await import("../web/js/pure/stages.js");
const { settings, setSetting } = await import("../web/js/settings.js");
const { blockedReason, canRedo, canUndo, currentError, currentKey, editCase, flushPersist, outdatedError, redoCase, setCase, setInputGroup, setInvalid, sealHistory, stageStatus, store, undoCase } = await import("../web/js/state.js");

// Every fetch waits until the test answers it: answer(data) with JSON or
// an ArrayBuffer, fail(error, status) with an error body.
const pending = [];
globalThis.fetch = (path, options) =>
  new Promise((resolve, reject) =>
    pending.push({
      path,
      headers: options.headers,
      body: options.body === undefined ? null : JSON.parse(options.body),
      answer: (data) => resolve({ ok: true, json: async () => data, arrayBuffer: async () => data }),
      fail: (error, status = 422) => resolve({ ok: false, status, statusText: "", json: async () => ({ error }) }),
      // No answer at all (the server is gone).
      drop: () => reject(new TypeError("Failed to fetch")),
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
// A mesh of `nodes` nodes (no triangles, no edges): the summary and its
// arrays, as POST /api/mesh and /api/mesh/arrays answer.
const meshSummary = (nodes = 0) => ({
  id: 5,
  nodes,
  triangles: 0,
  boundary_edges: 0,
  region_names: [],
  has_links: false,
  arrays: {
    path: "/api/mesh/arrays",
    format: "little-endian",
    bytes: 16 * nodes,
    parts: [
      { name: "nodes", type: "float64", count: 2 * nodes, offset: 0 },
      { name: "triangles", type: "uint32", count: 0, offset: 16 * nodes },
      { name: "boundary_edges", type: "uint32", count: 0, offset: 16 * nodes },
      { name: "boundary_regions", type: "uint32", count: 0, offset: 16 * nodes },
    ],
  },
});
async function answerMesh(nodes = 3) {
  (await next("/api/mesh")).answer({ mesh: meshSummary(nodes), stats: { nodes } });
  const arrays = await next("/api/mesh/arrays");
  assert.deepEqual(arrays.body, { mesh: 5 }, "the arrays of the mesh by its id");
  const buffer = new ArrayBuffer(16 * nodes);
  const view = new DataView(buffer);
  for (let i = 0; i < 2 * nodes; i++) view.setFloat64(8 * i, i + 0.5, true);
  arrays.answer(buffer);
}
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
  await answerMesh();
  await mesh;
  assert.deepEqual([...store.results.mesh.data.mesh.nodes], [0.5, 1.5, 2.5, 3.5, 4.5, 5.5], "nodes in float64 from the binary answer");
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
  await answerMesh();
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

test("a time run streams its snapshots and plays from its start", async () => {
  start();
  const run = runStage("solve");
  (await next("/api/solve")).answer(timeRun("running", 2, 0.25));
  (await next("/api/solve/values")).answer(float32([0, 0, 1, 1]));
  await flush();
  assert.equal(followedRun()?.job, 3, "followed");
  assert.equal(stageStatus("solve"), "busy");
  assert.equal(store.solveFrame, 0, "the start is shown");
  assert.equal(store.playRun, true, "the time bar plays it as it comes");
  // While it runs, a problem check is not held back (the server answers
  // between chunks).
  scheduleProblemCheck({ now: true });
  (await next("/api/problem")).answer({ ok: true });
  const progress = await next("/api/solve/progress");
  assert.deepEqual(progress.body, { job: 3 });
  progress.answer(timeRun("running", 4, 0.75));
  await sleep(10);
  const data = store.results.solve.data;
  assert.equal(data.values.length, 4);
  assert.equal(store.solveFrame, 0, "the playback moves the picture, not the progress");
  assert.equal(data.values[3], null, "the playback fetches what it shows");
  assert.equal(data.scale.max, 4, "the scale follows the range so far");
  // The slider was moved: the picture stays there.
  store.solveFrameChosen = true;
  store.solveFrame = 1;
  (await next("/api/solve/progress")).answer(timeRun("done", 5, 1));
  assert.equal(await run, true);
  assert.equal(data.values.length, 5);
  assert.equal(data.values[4], null, "a snapshot not shown is not fetched");
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
  meshRequest.answer({ mesh: meshSummary(0), stats: { nodes: 0 } });
  (await next("/api/mesh/arrays")).answer(new ArrayBuffer(0));
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

test("the check while typing follows the settings, at once and one at a time", async () => {
  start();
  setSetting("checkWhileTyping", false);
  scheduleProblemCheck();
  await sleep(50);
  assert.equal(pending.length, 0, "no check while typing");
  scheduleProblemCheck({ now: true });
  (await next("/api/problem")).answer({ ok: true });
  setSetting("checkWhileTyping", true);
  // Typing: the first key is checked at once, the keys typed while it
  // runs are checked together when it answers.
  editCase((c) => (c.problem.parameters = { k: 1 }), "edit", { sections: ["problem"] });
  scheduleProblemCheck();
  const first = await next("/api/problem");
  for (let k = 2; k < 5; k++) {
    editCase((c) => (c.problem.parameters = { k }), "edit", { sections: ["problem"] });
    scheduleProblemCheck();
  }
  await sleep(20);
  assert.equal(pending.filter((p) => p.path === "/api/problem").length, 0, "one check at a time");
  first.answer({ ok: true });
  const second = await next("/api/problem");
  assert.deepEqual(second.body.problem.parameters, { k: 4 }, "the newest text");
  second.answer({ ok: true });
  await sleep(20);
  assert.equal(pending.length, 0, "nothing more");
  assert.ok(JSON.parse(storage.get("pde2d-app-settings")).checkWhileTyping === true, "settings are kept");
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

test("every request carries this page's client id", async () => {
  start();
  const run = runStage("geometry");
  const request = await next("/api/geometry");
  assert.match(request.headers["X-PDE2D-Client"], /^[0-9a-f]{24}$/);
  assert.equal(request.headers["X-PDE2D-Token"], undefined, "no token on a loopback server");
  request.answer(geometryView("left"));
  await run;
});

test("the comparison is the server's, in double", async () => {
  start();
  const { compareWith, comparisons, comparisonValues, currentComparison, isOutdated } = await import("../web/js/compare.js");
  fresh("solve", { job: 9, state: "done", mode: "time", times: [0, 1], values: [new Float32Array(2), null], layout: layout(2) });
  const comparing = compareWith("x");
  const request = await next("/api/solve/compare");
  assert.deepEqual(request.body, { job: 9, exact: "x" });
  request.answer({ snapshots: [{ t: 0, max_abs: 1e-15, rms: 5e-16, l2: 4e-16 }, { t: 1, max_abs: 2e-15, rms: 1e-15, l2: 8e-16 }], max_abs: 2e-15 });
  await comparing;
  const comparison = comparisons.exact;
  assert.equal(currentComparison("exact"), comparison);
  assert.deepEqual([...comparison.linf], [1e-15, 2e-15], "the server's figures, not float32 rounding");
  assert.deepEqual([...comparison.rms], [5e-16, 1e-15]);
  assert.equal(comparison.scale.min, -2e-15, "one symmetric scale");
  // u − exact of a snapshot is fetched when shown.
  assert.equal(comparisonValues("exact", 1), null);
  const difference = await next("/api/solve/compare/difference");
  assert.deepEqual(difference.body, { job: 9, exact: "x", snapshot: 1 });
  difference.answer(float32([1e-15, -2e-15]));
  await sleep(5);
  assert.ok(comparisonValues("exact", 1) instanceof Float32Array);
  // The comparison belongs to its expression: another one in the case
  // marks it outdated, the same one again makes it current.
  editCase((c) => ((c.view ??= {}).exact_solution = "x"), "edit", { sections: ["view"] });
  assert.equal(isOutdated("exact"), false);
  editCase((c) => (c.view.exact_solution = "x + 1"), "edit", { sections: ["view"] });
  assert.equal(isOutdated("exact"), true, "for the previous expression");
  assert.equal(currentComparison("exact"), comparison, "still shown, greyed");
});

test("an answer to an older comparison is dropped", async () => {
  start();
  const { compare, comparisons } = await import("../web/js/compare.js");
  fresh("solve", { job: 9, state: "done", mode: "static", times: [0], values: [new Float32Array(2)], layout: layout(1) });
  const first = compare("exact", { exact: "x" });
  const older = await next("/api/solve/compare");
  const second = compare("exact", { exact: "y" });
  const newer = await next("/api/solve/compare");
  newer.answer({ snapshots: [{ t: 0, max_abs: 2, rms: 2, l2: 2 }], max_abs: 2 });
  await second;
  older.answer({ snapshots: [{ t: 0, max_abs: 1, rms: 1, l2: 1 }], max_abs: 1 });
  await first;
  assert.equal(comparisons.exact.linf[0], 2, "the newer comparison stays");
  assert.equal(comparisons.exact.against.exact, "y");
});

test("a reference run is kept, compared and forgotten; a refusal says why", async () => {
  start();
  const { compare, comparisons, forgetReference, keepReference } = await import("../web/js/compare.js");
  fresh("solve", { job: 4, state: "done", mode: "static", times: [0], values: [new Float32Array(2)], layout: layout(1) });
  const keeping = keepReference();
  const keep = await next("/api/solve/reference");
  assert.deepEqual(keep.body, { job: 4 });
  keep.answer({ reference: { job: 4, mode: "static", times: [0], mesh_id: 2 } });
  assert.equal(await keeping, null);
  assert.equal(store.reference.job, 4);
  fresh("solve", { job: 5, state: "done", mode: "static", times: [0], values: [new Float32Array(2)], layout: layout(1) });
  const comparing = compare("reference", { reference: 4 });
  const request = await next("/api/solve/compare");
  assert.deepEqual(request.body, { job: 5, reference: true });
  request.fail({ stage: "compare", kind: "input", message: "the reference run 4 is on another mesh than run 5" });
  await comparing;
  assert.match(comparisons.reference.error.message, /another mesh/);
  const forgetting = forgetReference();
  (await next("/api/solve/reference/clear")).answer({ reference: null });
  assert.equal(await forgetting, null);
  assert.equal(store.reference, null);
});

test("a preset answer does not overwrite an edit made after the request", async () => {
  start();
  const done = applyPreset({ id: "wave", label: "Wave" });
  const request = await next("/api/preset");
  editCase((c) => (c.problem.parameters = { k: 7 }), "edit", { sections: ["problem"] });
  request.answer({ ...fullCase(), problem: { equation: { m2: "1" } }, solve: { mode: "time" } });
  assert.equal(await done, false);
  assert.deepEqual(store.caseData.problem.parameters, { k: 7 }, "the edit stays");
  assert.equal(store.caseData.problem.equation, undefined);
});

test("a preset answer for a case that was replaced is dropped", async () => {
  start();
  const done = applyPreset({ id: "wave", label: "Wave" });
  const request = await next("/api/preset");
  setCase(fullCase());
  request.answer({ ...fullCase(), problem: { equation: { m2: "1" } }, solve: { mode: "time" } });
  assert.equal(await done, false);
  assert.equal(store.caseData.problem.equation, undefined);
});

test("invalid number fields block every run", async () => {
  start();
  setInvalid("mesh.spec.max_edge_length.relative", { label: "Max edge length, relative", reason: "not a number" });
  assert.equal(blockedReason(), "1 field is not a number: Max edge length, relative");
  assert.equal(await runWithPrerequisites("solve"), false);
  assert.equal(await runAll(), false);
  await flush();
  assert.equal(pending.length, 0, "nothing was sent");
  setInvalid("mesh.spec.max_edge_length.relative", null);
  assert.equal(blockedReason(), null);
  setInvalid("a", { label: "A" });
  setCase(fullCase());
  assert.equal(blockedReason(), null, "a new case has no invalid fields");
});


test("the conditions the page adds after a geometry leave the case as saved", async () => {
  setCase(structuredClone(rectangle), { dirty: false });
  const run = runStage("geometry");
  (await next("/api/geometry")).answer(geometryView("bottom", "left"));
  assert.equal(await run, true);
  assert.deepEqual(Object.keys(store.caseData.problem.boundary_conditions).sort(), ["bottom", "left"], "synced");
  assert.equal(store.dirty, false, "no unsaved changes from the page's own edit");
  assert.equal(canUndo(), false, "and no undo step");
  editCase((c) => (c.problem.boundary_conditions.left.g = "3"), "edit", { sections: ["problem"] });
  assert.equal(store.dirty, true, "the user's edit is one");
});

test("a progress poll refused as busy, or unanswered, is tried again and the run followed on", async () => {
  start();
  const run = runStage("solve");
  (await next("/api/solve")).answer(timeRun("running", 2, 0.25));
  (await next("/api/solve/values")).answer(float32([0, 0, 1, 1]));
  (await next("/api/solve/progress")).fail({ stage: "request", kind: "busy", message: "busy" }, 429);
  await sleep(5);
  assert.match(store.runNotice, /busy.*Trying again in 1 s/);
  assert.equal(followedRun()?.job, 3, "still followed");
  (await next("/api/solve/progress")).drop();
  await sleep(5);
  assert.match(store.runNotice, /does not answer/);
  (await next("/api/solve/progress")).answer(timeRun("done", 2, 1));
  assert.equal(await run, true);
  assert.equal(store.runNotice, null, "the notice goes once it answers");
  assert.equal(stageStatus("solve"), "done");
});

test("a run that is gone ends the following with its reason", async () => {
  start();
  const run = runStage("solve");
  (await next("/api/solve")).answer(timeRun("running", 2, 0.25));
  (await next("/api/solve/values")).answer(float32([0, 0, 1, 1]));
  (await next("/api/solve/progress")).fail({ stage: "solve", kind: "expired", message: "the run 3 was dropped to make room" }, 410);
  assert.equal(await run, false);
  assert.equal(currentError("solve").kind, "expired");
  assert.equal(followedRun(), null);
  assert.equal(store.results.solve.data.values.length, 2, "what arrived stays");
});

test("a reloaded tab finds its run on the server and shows it", async () => {
  start();
  fresh("geometry", geometryView("left"));
  fresh("mesh", { mesh: { id: 5, nodes: new Float64Array(4) }, stats: { nodes: 2 } });
  const found = reattachRun();
  const current = await next("/api/solve/current");
  assert.ok(current.body.solve, "asked with the solve's sections");
  current.answer({ run: timeRun("done", 2, 1) });
  (await next("/api/solve/values")).answer(float32([0, 0, 1, 1]));
  assert.equal(await found, true);
  assert.equal(store.results.solve.data.job, 3);
  assert.equal(stageStatus("solve"), "done");
  assert.equal(store.fieldSource, "solve");
  // None on the server: nothing is shown.
  start();
  const none = reattachRun();
  (await next("/api/solve/current")).answer({ run: null });
  assert.equal(await none, false);
  assert.equal(store.results.solve, null);
});

test("a failed snapshot is not asked again until Retry", async () => {
  start();
  const data = { job: 3, layout: layout(3), values: [null, null, null], times: [0, 0.5, 1] };
  const loading = loadAround(data, 1);
  (await next("/api/solve/values")).fail({ stage: "solve", kind: "expired", message: "gone" }, 410);
  assert.equal(await loading, false);
  assert.equal(snapshotState(data, 1).failure.kind, "expired");
  assert.equal(snapshotValues(data, 1), null);
  await sleep(5);
  assert.equal(pending.length, 0, "no new request while it failed");
  retrySnapshots(data);
  assert.equal(snapshotValues(data, 1), null);
  (await next("/api/solve/values")).answer(float32([0, 0, 1, 1, 2, 2]));
  await sleep(5);
  assert.ok(snapshotState(data, 1).values instanceof Float32Array);
  assert.equal(snapshotState(data, 1).failure, null);
});

test("undo and redo of case edits; typing in one field is one step", () => {
  start();
  assert.equal(canUndo(), false, "a new case starts without history");
  const field = {};
  setInputGroup(field);
  for (const g of ["1", "12", "123"]) editCase((c) => (c.problem.boundary_conditions.left.g = g), "edit", { sections: ["problem"] });
  setInputGroup(null);
  sealHistory();
  editCase((c) => (c.mesh.m = 9), "structure");
  assert.equal(canUndo(), true);
  assert.deepEqual(undoCase(), ["mesh"], "only the changed section");
  assert.equal(store.caseData.mesh.m, 1);
  assert.equal(currentKey("mesh"), stageKey(store.caseData, "mesh"), "keys follow");
  assert.deepEqual(undoCase(), ["problem"]);
  assert.equal(store.caseData.problem.boundary_conditions.left.g, "0", "the whole typing at once");
  assert.equal(canUndo(), false);
  redoCase();
  assert.equal(store.caseData.problem.boundary_conditions.left.g, "123");
  assert.equal(canRedo(), true);
  redoCase();
  assert.equal(store.caseData.mesh.m, 9);
  // A section the edit created is removed by undo.
  editCase((c) => (c.notes = "hello"));
  undoCase();
  assert.equal("notes" in store.caseData, false);
  // Conditions that follow a new geometry are not a step of their own.
  const before = canUndo();
  editCase((c) => (c.problem.boundary_conditions.right = { alpha: "1" }), "structure", { sections: ["problem"], record: false });
  assert.equal(canUndo(), before);
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
