// Overlapping requests in the page (app/web/js/actions.js), with a mocked
// fetch: an answer to an older request must not change the state. Run by
// CTest as app_web_actions when Node.js is installed.
import assert from "node:assert/strict";
import { runStage, showField } from "../web/js/actions.js";
import { setCase, store } from "../web/js/state.js";

// Every fetch waits until the test answers it.
const pending = [];
globalThis.fetch = (path, options) =>
  new Promise((resolve) => pending.push({ path, body: JSON.parse(options.body), answer: (data) => resolve({ ok: true, json: async () => data }) }));
const flush = () => new Promise((resolve) => setTimeout(resolve, 0));
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
