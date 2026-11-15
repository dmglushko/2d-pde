// Running the stages on the server and keeping their results.

import { api, ApiError } from "./api.js";
import { RUN_ALL, STAGES, fieldScale, reconcileConditions, stageInfo, stageKey, stagesToRun } from "./core.js";
import { editCase, freshResult, currentError, notify, openStep, setColorBy, store } from "./state.js";

// --- the toast ------------------------------------------------------------------

let toastTimer = null;
let toastEnd = null;
let toastCount = 0;

// Shows a short message. With `action` ({label, run}) it carries one
// button and stays longer; `onEnd` runs when it hides or is replaced.
export function toast(message, { action = null, onEnd = null } = {}) {
  toastCount += 1;
  const ending = toastEnd;
  toastEnd = onEnd;
  ending?.();
  const element = globalThis.document?.getElementById("toast");
  if (!element) return;
  element.replaceChildren(globalThis.document.createTextNode(message));
  if (action) {
    const button = globalThis.document.createElement("button");
    button.type = "button";
    button.className = "small";
    button.textContent = action.label;
    button.addEventListener("click", () => {
      // The action may show a toast of its own, which replaces this one.
      const shown = toastCount;
      action.run();
      if (toastCount === shown) hideToast();
    });
    element.append(" ", button);
  }
  element.classList.toggle("has-action", Boolean(action));
  element.classList.add("show");
  clearTimeout(toastTimer);
  toastTimer = setTimeout(hideToast, action ? 12000 : 3600);
}

function hideToast() {
  clearTimeout(toastTimer);
  globalThis.document?.getElementById("toast")?.classList.remove("show", "has-action");
  const ending = toastEnd;
  toastEnd = null;
  ending?.();
}

// --- running stages ---------------------------------------------------------------

const calls = {
  geometry: (caseData) => api.geometry(caseData),
  mesh: (caseData) => api.mesh(caseData),
  problem: (caseData) => api.problem(caseData),
  assemble: (caseData) => api.assemble(caseData),
  field: (caseData, field) => api.field(caseData, field),
  // The summary, then the values in binary (docs/app.md, "The API").
  solve: async (caseData) => {
    const summary = await api.solve(caseData);
    summary.values = await api.solveValues(caseData, summary.values);
    return summary;
  },
};
for (const stage of STAGES) if (!calls[stage.call]) throw new Error(`no call for ${stage.id}`);

// Requests overlap (a slow mesh, a quick edit, a new case). Each run of a
// stage gets a generation; only the latest run of a stage, for the case
// that is still loaded, may change the state. Older answers are dropped.
const latest = {};

const longRunning = () => STAGES.some((s) => s.long && store.busy.has(s.id));

// Runs one stage for the current case. Returns true when it worked, false
// when it failed, and null when a newer run or another case superseded it
// (its answer is then dropped).
async function run(stage, field) {
  const generation = (latest[stage] = (latest[stage] ?? 0) + 1);
  const caseId = store.caseId;
  const caseData = structuredClone(store.caseData);
  const key = stageKey(caseData, stage);
  const current = () => generation === latest[stage] && caseId === store.caseId;
  store.busy.add(stage);
  store.busySince[stage] = globalThis.performance?.now() ?? Date.now();
  notify("result");
  try {
    const data = await calls[stageInfo(stage).call](caseData, field);
    if (!current()) return null;
    if (stage === "solve") {
      // One colour scale for every snapshot, so the colours mean the same
      // value at every time.
      data.scale = fieldScale([data.min, data.max]);
      store.solveFrame = data.times.length - 1;
      store.fieldSource = "solve";
      store.view.colorBy = "field";
    }
    if (stage === "field") {
      store.fieldSource = "field";
      store.view.colorBy = "field";
    }
    store.results[stage] = { key, data };
    store.errors[stage] = null;
    if (stage === "geometry") syncConditions(data, key);
    return true;
  } catch (error) {
    if (!current()) return null;
    const failure = error instanceof ApiError ? error : new ApiError({ stage, message: String(error) });
    failure.key = key;
    store.errors[stage] = failure;
    return false;
  } finally {
    if (generation === latest[stage]) {
      store.busy.delete(stage);
      delete store.busySince[stage];
    }
    notify("result");
    if (checkWaiting && !longRunning()) {
      checkWaiting = false;
      run("problem");
    }
  }
}

export function runStage(stage) {
  return run(stage);
}

// A stage is fresh when its result was built from the current case and
// its last run did not fail for it.
const isFresh = (stage) => freshResult(stage) !== null && !currentError(stage);

function failed(stage) {
  const label = stageInfo(stage).label;
  toast(`The ${label} step failed; it is open now.`);
  openStep(stage);
}

// Runs `stage` after its prerequisites (core.js, STAGES) that are not
// fresh, stopping at the first failure with a toast that opens the
// failing step. `field` is the field of a "field" run. Returns true when
// the stage ran and worked, false after a failure, null when superseded.
export async function runWithPrerequisites(stage, field) {
  for (const id of stagesToRun(stage, isFresh)) {
    const ok = await run(id, id === stage ? field : undefined);
    if (ok === null) return null;
    if (!ok) {
      failed(id === "field" ? "assemble" : id);
      return false;
    }
  }
  return true;
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

// Geometry, mesh, assembly and solve in turn, skipping a stage that is
// fresh; stops at the first failure (with the toast of
// runWithPrerequisites). Returns true when all are up to date.
export async function runAll() {
  for (const stage of RUN_ALL) {
    if (isFresh(stage)) continue;
    const ok = await run(stage);
    if (ok === null) return false;
    if (!ok) {
      failed(stage);
      return false;
    }
  }
  setColorBy("field");
  return true;
}

// Checks the problem a moment after the last edit. While a long stage
// runs, the server is busy (one mutex for all symbolic work): the check
// waits and runs once when the stage ends.
let checkTimer = null;
let checkWaiting = false;
export function scheduleProblemCheck() {
  clearTimeout(checkTimer);
  checkTimer = setTimeout(() => {
    if (longRunning()) checkWaiting = true;
    else run("problem");
  }, 350);
}
