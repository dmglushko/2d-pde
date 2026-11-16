// The page's state: the catalog, the case being edited and the last result
// of each stage. Panels and the viewer subscribe and redraw on changes.

import { STAGES, stageInfo } from "./core.js";

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
  // True once the user moved the slider during the current run: the
  // picture then stops following the newest snapshot.
  solveFrameChosen: false,
  // The picture's settings: colorBy is "none", "quality" or "field";
  // isolines draws the shown field's isolines; showError shows u − exact
  // (compare.js) instead of u while a comparison is current.
  view: { colorBy: "none", isolines: false, showError: false },
  // The probes pinned on the picture: {slot (its colour), x, y}, at most
  // MAX_PROBES. The viewer's, not the case's.
  probes: [],
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
// "view" (a setting of the picture), "step" (another step was opened),
// "probes" (a probe was pinned or removed), "compare" (a comparison with
// an exact solution moved on).
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
  forgetKeys(null);
  persist();
  notify("case");
}

// `sections`: the case sections the edit changes (their keys are
// computed again); by default every section.
export function editCase(mutate, reason = "edit", { sections = null } = {}) {
  mutate(store.caseData);
  store.dirty = true;
  forgetKeys(sections);
  schedulePersist();
  notify(reason);
}

// After the case was written to its file.
export function markSaved() {
  store.dirty = false;
  persist();
  notify("view");
}

// --- stage keys ------------------------------------------------------------------

// Each section's JSON text, computed once per change of the section: a
// keystroke in the problem must not stringify an SVG or a mesh file again
// (every panel asks for stage keys on every change).
let sectionTexts = new Map();

function forgetKeys(sections) {
  if (!sections) sectionTexts = new Map();
  else for (const section of sections) sectionTexts.delete(section);
}

function sectionText(section) {
  let text = sectionTexts.get(section);
  if (text === undefined) {
    text = JSON.stringify(store.caseData?.[section] ?? null);
    sectionTexts.set(section, text);
  }
  return text;
}

// The current case's key for `stage` (core.js stageKey: the same text).
export function currentKey(stage) {
  return `[${stageInfo(stage).sections.map(sectionText).join(",")}]`;
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

// One of the picture's switches (isolines, showError).
export function setViewOption(name, value) {
  if (store.view[name] === value) return;
  store.view[name] = value;
  notify("view");
}

export function setProbes(probes) {
  store.probes = probes;
  notify("probes");
}

// A stage's result, if it was built from the current case.
export function freshResult(stage) {
  const result = store.results[stage];
  return result && result.key === currentKey(stage) ? result.data : null;
}

// The last result of a stage even if the case changed since.
export function lastResult(stage) {
  return store.results[stage]?.data ?? null;
}

// The error of a stage's last run if it was produced for the current
// case, else null.
export function currentError(stage) {
  const error = store.errors[stage];
  return error && error.key === currentKey(stage) ? error : null;
}

// The error of a stage's last run when it was produced for an earlier
// input, else null.
export function outdatedError(stage) {
  const error = store.errors[stage];
  return error && error.key !== currentKey(stage) ? error : null;
}

export function stageStatus(stage) {
  if (store.busy.has(stage)) return "busy";
  if (currentError(stage)) return "failed";
  const result = store.results[stage];
  if (result && result.key === currentKey(stage)) return result.data?.state === "stopped" ? "stopped" : "done";
  if (result || store.errors[stage]) return "stale";
  return "none";
}

// The case is kept in this browser between visits (a convenience; saving
// a case file is the reliable way to keep it). Writing it stringifies the
// whole case, so edits write it a moment after the last one, and a page
// that is hidden or closed writes it at once.
let persistTimer = null;
const PERSIST_DELAY_MS = 600;

function schedulePersist() {
  clearTimeout(persistTimer);
  persistTimer = setTimeout(persist, PERSIST_DELAY_MS);
}

export function flushPersist() {
  if (persistTimer === null) return;
  persist();
}

globalThis.addEventListener?.("pagehide", flushPersist);
globalThis.document?.addEventListener("visibilitychange", () => {
  if (globalThis.document.visibilityState === "hidden") flushPersist();
});

function persist() {
  clearTimeout(persistTimer);
  persistTimer = null;
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
