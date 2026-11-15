// Running the stages on the server and keeping their results.

import { api, ApiError } from "./api.js";
import { fieldScale, reconcileConditions, stageKey } from "./core.js";
import { editCase, notify, store } from "./state.js";

let toastTimer = null;

export function toast(message) {
  const element = globalThis.document?.getElementById("toast");
  if (!element) return;
  element.textContent = message;
  element.classList.add("show");
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => element.classList.remove("show"), 3200);
}

const calls = {
  geometry: (caseData) => api.geometry(caseData),
  mesh: (caseData) => api.mesh(caseData),
  problem: (caseData) => api.problem(caseData),
  assemble: (caseData) => api.assemble(caseData),
  field: (caseData, field) => api.field(caseData, field),
  solve: (caseData) => api.solve(caseData),
};

// Requests overlap (a slow mesh, a quick edit, a new case). Each run of a
// stage gets a generation; only the latest run of a stage, for the case
// that is still loaded, may change the state. Older answers are dropped.
const latest = {};

// Runs one stage for the current case. Returns true when it worked, false
// when it failed, and null when a newer run or another case superseded it
// (its answer is then dropped).
async function run(stage, field) {
  const generation = (latest[stage] = (latest[stage] ?? 0) + 1);
  const caseId = store.caseId;
  const caseData = structuredClone(store.caseData);
  const key = stageKey(caseData, stage === "field" ? "assemble" : stage);
  const current = () => generation === latest[stage] && caseId === store.caseId;
  store.busy.add(stage);
  notify("result");
  try {
    const data = await calls[stage](caseData, field);
    if (!current()) return null;
    if (stage === "solve") {
      // One colour scale for every snapshot, so the colours mean the same
      // value at every time.
      data.scale = fieldScale([data.min, data.max]);
      store.solveFrame = data.times.length - 1;
      store.fieldSource = "solve";
    }
    if (stage === "field") store.fieldSource = "field";
    store.results[stage] = { key, data };
    store.errors[stage] = null;
    if (stage === "geometry") syncConditions(data, key);
    return true;
  } catch (error) {
    if (!current()) return null;
    store.errors[stage] = error instanceof ApiError ? error : new ApiError({ stage, message: String(error) });
    return false;
  } finally {
    if (generation === latest[stage]) store.busy.delete(stage);
    notify("result");
  }
}

export function runStage(stage) {
  return run(stage);
}

// Evaluates a field on the mesh (docs/app.md, "The API").
export function showField(field) {
  return run("field", field);
}

// After a geometry the library accepted: every region gets a boundary
// condition (core.js, reconcileConditions). Only for the geometry that is
// still in the case.
function syncConditions(geometryView, key) {
  if (key !== stageKey(store.caseData, "geometry")) return;
  const regions = geometryView.regions.map((r) => r.name);
  const next = reconcileConditions(store.caseData, regions, store.catalog.problem.default_condition);
  const same = JSON.stringify([next.problem, next.view]) === JSON.stringify([store.caseData.problem, store.caseData.view]);
  if (same) return;
  editCase((c) => {
    c.problem = next.problem;
    c.view = next.view;
  }, "structure");
}

// Geometry, mesh, assembly and solve in turn, skipping a stage whose
// result is still fresh; stops at the first failure.
export async function runAll() {
  for (const stage of ["geometry", "mesh", "assemble", "solve"]) {
    const result = store.results[stage];
    if (result && result.key === stageKey(store.caseData, stage) && !store.errors[stage]) continue;
    const ok = await runStage(stage);
    if (ok === null) return false;
    if (!ok) {
      toast(`${stage} failed: see the ${stage[0].toUpperCase() + stage.slice(1)} step`);
      return false;
    }
  }
  return true;
}

// Checks the problem a moment after the last edit.
let checkTimer = null;
export function scheduleProblemCheck() {
  clearTimeout(checkTimer);
  checkTimer = setTimeout(() => runStage("problem"), 350);
}
