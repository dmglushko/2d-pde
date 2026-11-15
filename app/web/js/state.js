// The page's state: the catalog, the case being edited and the last result
// of each stage. Panels and the viewer subscribe and redraw on changes.

import { STAGES, stageKey } from "./core.js";

const STORAGE_KEY = "pde2d-app-case";
const NAME_KEY = "pde2d-app-case-name";
const DIRTY_KEY = "pde2d-app-case-dirty";

const perStage = (value) => Object.fromEntries(STAGES.map((s) => [s.id, value]));

export const store = {
  catalog: null,
  caseData: null,
  // The file name the case is saved under: the opened file's or the
  // example's, else "case.json".
  caseName: "case.json",
  // True once the case changed since it was loaded or saved.
  dirty: false,
  // Per stage: {key, data}, where key is stageKey() of the case the result
  // was built from.
  results: perStage(null),
  // Per stage: the ApiError of its last run, or null. Its `key` is the
  // stageKey() of the case it was produced for.
  errors: perStage(null),
  busy: new Set(),
  // Per running stage: when it started (performance.now()).
  busySince: {},
  // Changes whenever a case replaces the current one, so answers to
  // requests made for the old case can be told apart.
  caseId: 0,
  // What the picture colours by "field" shows: "field" (the last field of
  // the Assemble step) or "solve" (the solution), whichever came last.
  fieldSource: "field",
  // The snapshot of the solution shown (an index into its times).
  solveFrame: 0,
  // The picture's settings: colorBy is "none", "quality" or "field".
  view: { colorBy: "none" },
  // The open step.
  step: "geometry",
  // A notice about the loaded case (sections replaced by defaults), shown
  // until dismissed: {text} or null.
  notice: null,
};

const listeners = new Set();

export function subscribe(listener) {
  listeners.add(listener);
}

// reason: "case" (a new case was loaded: forms rebuild), "edit" (a value
// changed), "structure" (a change that needs a form rebuilt, such as a new
// component type), "result" (a stage started or finished), "frame"
// (another snapshot of the solution is shown; only the picture changes),
// "view" (a setting of the picture), "step" (another step was opened).
export function notify(reason) {
  for (const listener of listeners) listener(reason);
}

// Replaces the case. `name` is the file name to save it under; `dirty`
// whether it differs from that file.
export function setCase(caseData, { name = store.caseName, dirty = false } = {}) {
  store.caseId += 1;
  store.caseData = caseData;
  store.caseName = name;
  store.dirty = dirty;
  for (const stage of Object.keys(store.errors)) store.errors[stage] = null;
  persist();
  notify("case");
}

export function editCase(mutate, reason = "edit") {
  mutate(store.caseData);
  store.dirty = true;
  persist();
  notify(reason);
}

// After the case was written to its file.
export function markSaved() {
  store.dirty = false;
  persist();
  notify("view");
}

export function openStep(step) {
  store.step = step;
  notify("step");
}

export function setColorBy(colorBy) {
  if (store.view.colorBy === colorBy) return;
  store.view.colorBy = colorBy;
  notify("view");
}

// A stage's result, if it was built from the current case.
export function freshResult(stage) {
  const result = store.results[stage];
  return result && result.key === stageKey(store.caseData, stage) ? result.data : null;
}

// The last result of a stage even if the case changed since.
export function lastResult(stage) {
  return store.results[stage]?.data ?? null;
}

// The error of a stage's last run if it was produced for the current
// case, else null.
export function currentError(stage) {
  const error = store.errors[stage];
  return error && error.key === stageKey(store.caseData, stage) ? error : null;
}

// The error of a stage's last run when it was produced for an earlier
// input, else null.
export function outdatedError(stage) {
  const error = store.errors[stage];
  return error && error.key !== stageKey(store.caseData, stage) ? error : null;
}

export function stageStatus(stage) {
  if (store.busy.has(stage)) return "busy";
  if (currentError(stage)) return "failed";
  const result = store.results[stage];
  if (result && result.key === stageKey(store.caseData, stage)) return "done";
  if (result || store.errors[stage]) return "stale";
  return "none";
}

// The case is kept in this browser between visits (a convenience; saving
// a case file is the reliable way to keep it).
function persist() {
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(store.caseData));
    localStorage.setItem(NAME_KEY, store.caseName);
    localStorage.setItem(DIRTY_KEY, store.dirty ? "1" : "0");
  } catch {
    // Storage may be unavailable (private window, blocked site data).
  }
}

// {caseData, name, dirty} kept from the last visit, or null.
export function savedCase() {
  try {
    const text = localStorage.getItem(STORAGE_KEY);
    if (!text) return null;
    return { caseData: JSON.parse(text), name: localStorage.getItem(NAME_KEY) || "case.json", dirty: localStorage.getItem(DIRTY_KEY) === "1" };
  } catch {
    return null;
  }
}
