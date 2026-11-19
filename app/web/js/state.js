// The page's state: the catalog, the case being edited and the last result
// of each stage. Panels and the viewer subscribe and redraw on changes.
// Every case edit goes through editCase (stage keys, revisions, undo).

import { History } from "./pure/history.js";
import { STAGES, stageInfo } from "./pure/stages.js";
import { Tickets } from "./pure/tickets.js";

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
  // Changes whenever a case replaces the current one: the scope of every
  // request ticket.
  caseId: 0,
  // What the picture colours by "field" shows: "field" (the last field of
  // the Assemble step) or "solve" (the solution), whichever came last.
  fieldSource: "field",
  // The snapshot of the solution shown (an index into its times).
  solveFrame: 0,
  // True once the user moved the slider during the current run.
  solveFrameChosen: false,
  // True while a run being computed should play from its start as it
  // comes (timebar.js); the slider or Pause ends it.
  playRun: false,
  // The picture's settings: colorBy is "none", "quality", "field" or
  // "gradient" (|∇u| of the shown field) or "indicator" (an adaptive
  // solution's error indicators, log10); isolines draws the shown field's
  // isolines; difference shows u − exact ("exact") or u − reference
  // ("reference") instead of u while that comparison is current; locked
  // holds the colour range the viewer fixed, per quantity ("field",
  // "gradient", "indicator"), kept until unlocked; tool is "pan" or "profile" (a drag
  // draws the line of the profile).
  view: { colorBy: "none", isolines: false, difference: null, locked: { field: null, gradient: null, indicator: null }, tool: "pan" },
  // The probes pinned on the picture: {slot (its colour), x, y}, at most
  // MAX_PROBES. The viewer's, not the case's.
  probes: [],
  // The line of the line profile ({from: [x, y], to: [x, y]}) or null.
  profile: null,
  // The boundary region selected in the picture or its legend (a name),
  // drawn highlighted; null for none.
  selectedRegion: null,
  // The client's reference run on the server ({job, mode, times,
  // mesh_id}) or null.
  reference: null,
  // Number fields whose text is not a number: path -> {label, reason}.
  // While any exist, runs are blocked (actions.js) and Save warns.
  invalid: new Map(),
  // The case the last New, Open, Example or file replaced ({caseData,
  // name, dirty}), offered back until the next replacement.
  previousCase: null,
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
// component type, or an undo), "result" (a stage started or finished),
// "frame" (another snapshot of the solution is shown; only the picture
// changes), "view" (a setting of the picture), "step" (another step was
// opened), "probes" (a probe was pinned or removed, or the profile line
// changed), "compare" (a comparison moved on), "invalid" (a number field
// became invalid or valid again), "history" (undo or redo became
// possible or not).
export function notify(reason) {
  for (const listener of listeners) listener(reason);
}

// --- request tickets (pure/tickets.js) ---------------------------------------

const tickets = new Tickets();

// A ticket for a request of `kind`, made now for the input keyOf()
// returns (read again when the answer arrives). ticket.live(): no newer
// request of its kind and the same case; ticket.holds(): live and the
// input is still the same. `exclusive: false` for requests that may run
// side by side (the differences of several snapshots).
export function requestTicket(kind, keyOf = () => null, { exclusive = true } = {}) {
  const ticket = tickets.issue(kind, store.caseId, keyOf(), { exclusive });
  return {
    ...ticket,
    newest: () => tickets.isNewest(ticket),
    live: () => tickets.isLive(ticket, store.caseId),
    holds: () => tickets.holds(ticket, store.caseId, keyOf()),
  };
}

// Every ticket of `kind` taken so far no longer holds.
export function supersede(kind) {
  tickets.supersede(kind);
}

// --- the case ---------------------------------------------------------------------

const history = new History({ limit: 100 });

// Replaces the case. `name` is the file name to save it under; `dirty`
// whether it differs from that file. Undo starts over (a replaced case
// comes back with "Restore previous case", not with undo).
export function setCase(caseData, { name = store.caseName, dirty = false } = {}) {
  store.caseId += 1;
  store.caseData = caseData;
  store.caseName = name;
  store.dirty = dirty;
  for (const stage of Object.keys(store.errors)) store.errors[stage] = null;
  store.invalid.clear();
  store.profile = null;
  store.selectedRegion = null;
  keepView();
  history.clear();
  forgetKeys(null);
  bumpRevisions(null);
  persist();
  notify("case");
}

// The group of the edit being made: the element whose "input" event is
// being handled (main.js sets it around each input event), so typing in
// one field is one undo step.
let inputGroup = null;
export function setInputGroup(group) {
  inputGroup = group;
}

// `sections`: the case sections the edit changes (their keys are
// computed again); by default every section. `record: false` leaves it
// out of undo (a consequence of another edit, such as the conditions
// following the regions of a new geometry). `automatic: true` is an edit
// the page makes by itself, not the user (the conditions that follow a
// geometry, a refinement study's mesh levels): it leaves the case as
// saved or not as it was, and is never an undo step.
export function editCase(mutate, reason = "edit", { sections = null, record = true, automatic = false } = {}) {
  if (automatic) record = false;
  const names = sections ?? Object.keys(store.caseData);
  const before = record ? textsOf(names) : null;
  mutate(store.caseData);
  if (!automatic) store.dirty = true;
  const touched = sections ?? [...new Set([...names, ...Object.keys(store.caseData)])];
  forgetKeys(touched);
  bumpRevisions(touched);
  if (record) {
    for (const name of touched) if (!(name in before)) before[name] = "null";
    history.record(before, textsOf(touched), inputGroup);
  }
  schedulePersist();
  notify(reason);
  if (record) notify("history");
}

// The next edit starts a new undo step (a field was left).
export function sealHistory() {
  history.seal();
}

export const canUndo = () => history.canUndo();
export const canRedo = () => history.canRedo();

// Puts back sections ({section: JSON text}) from the history.
function restoreSections(texts) {
  if (!texts) return null;
  for (const [section, text] of Object.entries(texts)) {
    if (text === "null") delete store.caseData[section];
    else store.caseData[section] = JSON.parse(text);
  }
  const sections = Object.keys(texts);
  forgetKeys(sections);
  bumpRevisions(sections);
  store.dirty = true;
  // The forms are built again from the case: their invalid texts go.
  store.invalid.clear();
  schedulePersist();
  notify("structure");
  notify("history");
  return sections;
}

// Undoes the last case edit; returns the sections it changed, or null.
export function undoCase() {
  return restoreSections(history.undo());
}

// Redoes the edit undone last; returns the sections it changed, or null.
export function redoCase() {
  return restoreSections(history.redo());
}

// After the case was written to its file.
export function markSaved() {
  store.dirty = false;
  persist();
  notify("view");
}

// --- invalid entries -------------------------------------------------------------

// Marks the number field at `path` as holding text that is not a number
// ({label, reason}), or valid again (null).
export function setInvalid(path, entry) {
  const had = store.invalid.has(path);
  if (entry) store.invalid.set(path, entry);
  else store.invalid.delete(path);
  if (had !== Boolean(entry) || entry) notify("invalid");
}

// The forms were built again from the case: no field holds invalid text.
export function clearInvalid() {
  if (store.invalid.size === 0) return;
  store.invalid.clear();
  notify("invalid");
}

// Why runs are blocked now ("1 field is not a number: Width"), or null.
export function blockedReason() {
  const entries = [...store.invalid.values()];
  if (entries.length === 0) return null;
  const count = entries.length === 1 ? "1 field is not a number" : `${entries.length} fields are not numbers`;
  return `${count}: ${entries.map((e) => e.label).join(", ")}`;
}

// --- stage keys and revisions -------------------------------------------------------

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

function textsOf(sections) {
  return Object.fromEntries(sections.map((s) => [s, sectionText(s)]));
}

// How many times each section was changed: the key of a request whose
// answer replaces sections (a preset) is their revisions.
const revisions = new Map();
let allRevision = 0;

function bumpRevisions(sections) {
  if (!sections) allRevision += 1;
  else for (const section of sections) revisions.set(section, (revisions.get(section) ?? 0) + 1);
}

// The revision of `sections`, as one text.
export function sectionRevision(...sections) {
  return `${allRevision}:${sections.map((s) => revisions.get(s) ?? 0).join(",")}`;
}

// The current case's key for `stage` (pure/stages.js stageKey: the same
// text).
export function currentKey(stage) {
  return `[${stageInfo(stage).sections.map(sectionText).join(",")}]`;
}

// --- the viewer's state ------------------------------------------------------------

export function openStep(step) {
  store.step = step;
  notify("step");
}

export function setColorBy(colorBy) {
  if (store.view.colorBy === colorBy) return;
  store.view.colorBy = colorBy;
  // The indicators are of the solution, not of u − other: one colouring
  // owns the picture.
  if (colorBy === "indicator") store.view.difference = null;
  keepView();
  notify("view");
}

// One of the picture's settings (isolines, difference, tool).
export function setViewOption(name, value) {
  if (store.view[name] === value) return;
  store.view[name] = value;
  // u − other is coloured as a field, never by the solution's indicators.
  if (name === "difference" && value && store.view.colorBy === "indicator") store.view.colorBy = "field";
  keepView();
  notify("view");
}

// Fixes the colour range of `quantity` ("field", "gradient" or "indicator") to
// {low, high}, or frees it (null).
export function setLockedRange(quantity, range) {
  store.view.locked = { ...store.view.locked, [quantity]: range };
  keepView();
  notify("view");
}

export function setProbes(probes) {
  store.probes = probes;
  keepView();
  notify("probes");
}

export function setProfile(profile) {
  store.profile = profile;
  keepView();
  notify("probes");
}

// Selects a boundary region (highlighted in the picture), or none.
export function setSelectedRegion(name) {
  if (store.selectedRegion === name) return;
  store.selectedRegion = name;
  notify("view");
}

// --- the picture's state in this tab --------------------------------------------

// The probes, the profile line, the locked ranges and the colour choice
// are the viewer's, not the case's: they are kept in this tab's
// sessionStorage (a reload of the tab finds them; another tab, or a saved
// case, does not carry them), and dropped when another case replaces this
// one.
const VIEW_KEY = "pde2d-app-view";

function keepView() {
  try {
    const { colorBy, isolines, difference, locked } = store.view;
    globalThis.sessionStorage?.setItem(VIEW_KEY, JSON.stringify({ colorBy, isolines, difference, locked, probes: store.probes, profile: store.profile }));
  } catch {
    // Not kept: the picture starts plain after a reload.
  }
}

// The picture's state kept in this tab, as stored (read before the case
// is loaded: loading a case keeps the state it starts with).
export function keptView() {
  try {
    return JSON.parse(globalThis.sessionStorage?.getItem(VIEW_KEY) ?? "null");
  } catch {
    return null;
  }
}

// Puts back the picture's state kept in this tab (after a reload), as far
// as it reads back whole; returns whether anything was.
export function restoreView(kept) {
  if (!kept || typeof kept !== "object") return false;
  const point = (p) => Array.isArray(p) && p.length === 2 && p.every(Number.isFinite);
  const range = (r) => (r && Number.isFinite(r.low) && Number.isFinite(r.high) && r.low < r.high ? { low: r.low, high: r.high } : null);
  if (["none", "quality", "field", "gradient", "indicator"].includes(kept.colorBy)) store.view.colorBy = kept.colorBy;
  store.view.isolines = kept.isolines === true;
  store.view.difference = ["exact", "reference"].includes(kept.difference) ? kept.difference : null;
  store.view.locked = { field: range(kept.locked?.field), gradient: range(kept.locked?.gradient), indicator: range(kept.locked?.indicator) };
  if (Array.isArray(kept.probes))
    store.probes = kept.probes.filter((p) => Number.isInteger(p?.slot) && p.slot >= 0 && p.slot < 4 && Number.isFinite(p.x) && Number.isFinite(p.y)).slice(0, 4);
  store.profile = kept.profile && point(kept.profile.from) && point(kept.profile.to) ? { from: kept.profile.from, to: kept.profile.to } : null;
  keepView();
  notify("view");
  return true;
}

// --- results -------------------------------------------------------------------------

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

// --- keeping the case in this browser ---------------------------------------------

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
