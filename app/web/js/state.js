// The page's state: the catalog, the case being edited and the last result
// of each stage. Panels and the viewer subscribe and redraw on changes.

import { stageKey } from "./core.js";

const STORAGE_KEY = "pde2d-app-case";

export const store = {
  catalog: null,
  caseData: null,
  // Per stage: {key, data}, where key is stageKey() of the case the result
  // was built from.
  results: { geometry: null, mesh: null, problem: null, assemble: null, field: null, solve: null },
  // Per stage: the ApiError of its last run, or null.
  errors: { geometry: null, mesh: null, problem: null, assemble: null, field: null, solve: null },
  busy: new Set(),
  // Changes whenever a case replaces the current one, so answers to
  // requests made for the old case can be told apart.
  caseId: 0,
  // What the picture colours by "field" shows: "field" (the last field of
  // the Assemble step) or "solve" (the solution), whichever came last.
  fieldSource: "field",
  // The snapshot of the solution shown (an index into its times).
  solveFrame: 0,
};

const listeners = new Set();

export function subscribe(listener) {
  listeners.add(listener);
}

// reason: "case" (a new case was loaded: forms rebuild), "edit" (a value
// changed), "structure" (a change that needs a form rebuilt, such as a new
// component type), "result" (a stage finished), "frame" (another snapshot
// of the solution is shown; only the picture changes).
export function notify(reason) {
  for (const listener of listeners) listener(reason);
}

export function setCase(caseData) {
  store.caseId += 1;
  store.caseData = caseData;
  for (const stage of Object.keys(store.errors)) store.errors[stage] = null;
  persist();
  notify("case");
}

export function editCase(mutate, reason = "edit") {
  mutate(store.caseData);
  persist();
  notify(reason);
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

export function stageStatus(stage) {
  if (store.busy.has(stage)) return "busy";
  if (store.errors[stage]) return "failed";
  const result = store.results[stage];
  if (!result) return "none";
  return result.key === stageKey(store.caseData, stage) ? "done" : "stale";
}

// The case is kept in this browser between visits (a convenience; saving
// a case file is the reliable way to keep it).
function persist() {
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(store.caseData));
  } catch {
    // Storage may be unavailable (private window, blocked site data).
  }
}

export function savedCase() {
  try {
    const text = localStorage.getItem(STORAGE_KEY);
    return text ? JSON.parse(text) : null;
  } catch {
    return null;
  }
}
