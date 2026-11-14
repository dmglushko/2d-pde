// Running the stages on the server and keeping their results.

import { api, ApiError } from "./api.js";
import { stageKey, withConditionsFor } from "./core.js";
import { editCase, notify, store } from "./state.js";

let toastTimer = null;

export function toast(message) {
  const element = document.getElementById("toast");
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
};

// Runs one stage for the current case. Returns true when it worked.
export async function runStage(stage) {
  const caseData = structuredClone(store.caseData);
  const key = stageKey(caseData, stage);
  store.busy.add(stage);
  notify("result");
  try {
    const data = await calls[stage](caseData);
    store.results[stage] = { key, data };
    store.errors[stage] = null;
    if (stage === "geometry") syncConditions(data);
    return true;
  } catch (error) {
    store.errors[stage] = error instanceof ApiError ? error : new ApiError({ stage, message: String(error) });
    return false;
  } finally {
    store.busy.delete(stage);
    notify("result");
  }
}

// Every region of the geometry gets a boundary condition (u = 0 for a new
// one), and conditions of regions that are gone are dropped, so the
// problem always matches the domain.
function syncConditions(geometryView) {
  const regions = geometryView.regions.map((r) => r.name);
  const current = Object.keys(store.caseData.problem?.boundary_conditions ?? {});
  if (current.length === regions.length && regions.every((r) => current.includes(r))) return;
  const next = withConditionsFor(store.caseData, regions, store.catalog.problem.default_condition);
  editCase((c) => {
    c.problem = next.problem;
  }, "structure");
  scheduleProblemCheck();
}

// Geometry, mesh and assembly in turn, skipping a stage whose result is
// still fresh; stops at the first failure.
export async function runAll() {
  for (const stage of ["geometry", "mesh", "assemble"]) {
    const result = store.results[stage];
    if (result && result.key === stageKey(store.caseData, stage) && !store.errors[stage]) continue;
    if (!(await runStage(stage))) {
      toast(`${stage} failed: see the ${stage === "assemble" ? "Assemble" : stage[0].toUpperCase() + stage.slice(1)} step`);
      return false;
    }
  }
  return true;
}

// Evaluates a field on the mesh (docs/app.md, "Fields").
export async function showField(field) {
  const caseData = structuredClone(store.caseData);
  store.busy.add("field");
  notify("result");
  try {
    const data = await api.field(caseData, field);
    store.results.field = { key: stageKey(caseData, "assemble"), data };
    store.errors.field = null;
    return true;
  } catch (error) {
    store.errors.field = error;
    return false;
  } finally {
    store.busy.delete("field");
    notify("result");
  }
}

// Checks the problem a moment after the last edit.
let checkTimer = null;
export function scheduleProblemCheck() {
  clearTimeout(checkTimer);
  checkTimer = setTimeout(() => runStage("problem"), 350);
}
