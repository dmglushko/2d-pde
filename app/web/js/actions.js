// Running the stages on the server and keeping their results.

import { api, ApiError } from "./api.js";
import { RUN_ALL, STAGES, fieldScale, reconcileConditions, requestCase, stageInfo, stagesToRun } from "./core.js";
import { settings } from "./settings.js";
import { growSnapshots, loadAround } from "./snapshots.js";
import { currentError, currentKey, editCase, freshResult, notify, openStep, setColorBy, store } from "./state.js";

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

// The solve: the server answers its first chunk at once; a time run then
// goes on in the background, and followRun fetches the snapshots as they
// come (docs/app.md, "Runs in chunks").
const calls = {
  geometry: (caseData) => api.geometry(caseData),
  mesh: (caseData) => api.mesh(caseData),
  problem: (caseData) => api.problem(caseData),
  assemble: (caseData) => api.assemble(caseData),
  field: (caseData, field) => api.field(caseData, field),
  // The summary, then the newest snapshot and its neighbours (the page
  // keeps some snapshots only, snapshots.js).
  solve: async (caseData) => {
    const summary = await api.solve(caseData);
    summary.layout = summary.values;
    summary.values = [];
    growSnapshots(summary, summary.layout.snapshots);
    if (summary.values.length > 0) await loadAround(summary, summary.values.length - 1);
    return summary;
  },
};
for (const stage of STAGES) if (!calls[stage.call]) throw new Error(`no call for ${stage.id}`);

// Requests overlap (a slow mesh, a quick edit, a new case). Each run of a
// stage gets a generation; only the latest run of a stage, for the case
// that is still loaded, may change the state. Older answers are dropped.
const latest = {};

// A time run being followed: {job, key, caseId, generation}; while one is,
// the solve stage is busy but the server answers other requests between
// its chunks.
let following = null;

const longRunning = () => STAGES.some((s) => s.long && store.busy.has(s.id) && !(s.id === "solve" && following));

// One colour scale for every snapshot, so the colours mean the same value
// at every time (while a run goes on, the range so far).
function solveScale(data) {
  data.scale = fieldScale([data.min, data.max]);
}

// Runs one stage for the current case. Returns true when it worked, false
// when it failed, and null when a newer run or another case superseded it
// (its answer is then dropped).
async function run(stage, field) {
  const generation = (latest[stage] = (latest[stage] ?? 0) + 1);
  const caseId = store.caseId;
  const key = currentKey(stage);
  const caseData = structuredClone(requestCase(store.caseData, stage));
  const current = () => generation === latest[stage] && caseId === store.caseId;
  if (stage === "solve") stopFollowing();
  store.busy.add(stage);
  store.busySince[stage] = globalThis.performance?.now() ?? Date.now();
  notify("result");
  try {
    const data = await calls[stageInfo(stage).call](caseData, field);
    if (!current()) return null;
    // A check of a problem that was edited since is not worth showing: a
    // newer one is on its way.
    if (stage === "problem" && key !== currentKey("problem")) return null;
    if (stage === "solve") {
      solveScale(data);
      store.solveFrame = data.times.length - 1;
      store.solveFrameChosen = false;
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
    if (stage === "solve" && data.state === "running") {
      following = { job: data.job, key, caseId, generation };
      notify("result");
      return await followRun(following, data);
    }
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

const POLL_MS = 250;
const wait = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

// Follows a time run until it is done, fails or is stopped: polls its
// progress, fetches the new snapshots and shows the newest (unless the
// user chose one with the slider, or the setting is off). Returns true
// when done, false when it failed, null when it was stopped or replaced.
async function followRun(run, data) {
  const live = () => following === run && store.caseId === run.caseId && latest.solve === run.generation;
  while (live()) {
    await wait(POLL_MS);
    if (!live()) return null;
    const progress = await api.solveProgress(run.job);
    if (!live()) return null;
    const values = data.values;
    const layout = progress.values;
    Object.assign(data, progress, { values, layout });
    growSnapshots(data, layout.snapshots);
    data.times = progress.times.slice(0, values.length);
    solveScale(data);
    const follow = settings.followRun && !store.solveFrameChosen;
    if (follow) store.solveFrame = values.length - 1;
    // The newest snapshot when it is shown; others load when shown.
    if (follow && values.length > 0) {
      await loadAround(data, values.length - 1);
      if (!live()) return null;
    }
    notify("progress");
    if (progress.state === "running") continue;
    following = null;
    if (progress.state === "failed") {
      const failure = new ApiError(progress.error);
      failure.key = run.key;
      store.errors.solve = failure;
      return false;
    }
    return progress.state === "done" ? true : null;
  }
  return null;
}

function stopFollowing() {
  if (!following) return;
  const job = following.job;
  following = null;
  api.solveStop(job).catch(() => {});
}

// Stops the time run being followed; what arrived stays.
export async function stopRun() {
  const run = following;
  if (!run) return;
  following = null;
  const data = store.results.solve?.data;
  if (data && data.job === run.job) data.state = "stopped";
  store.busy.delete("solve");
  delete store.busySince.solve;
  notify("result");
  try {
    const progress = await api.solveStop(run.job);
    // It may have ended before the stop arrived.
    if (data && data.job === run.job && progress.state === "done") {
      const values = data.values;
      Object.assign(data, progress, { values, layout: progress.values });
      growSnapshots(data, progress.values.snapshots);
      data.times = progress.times.slice(0, values.length);
      solveScale(data);
    }
  } catch (error) {
    toast(error.message);
  }
  notify("result");
}

// The run being followed, or null.
export function followedRun() {
  return following;
}

// A change of the case that changes the solve's inputs stops its run
// (main.js calls this after every change).
export function caseChanged() {
  if (following && (store.caseId !== following.caseId || currentKey("solve") !== following.key)) {
    stopFollowing();
    latest.solve = (latest.solve ?? 0) + 1;
    store.busy.delete("solve");
    delete store.busySince.solve;
    const data = store.results.solve?.data;
    if (data) data.state = "stopped";
  }
}

export function runStage(stage) {
  return run(stage);
}

// A stage is fresh when its result was built from the current case, its
// last run did not fail for it, and (a solve) it ran to the end.
const isFresh = (stage) => {
  const result = freshResult(stage);
  return result !== null && !currentError(stage) && result.state !== "stopped" && result.state !== "running";
};

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
  if (key !== currentKey("geometry")) return;
  const regions = geometryView.regions.map((r) => r.name);
  const next = reconcileConditions(store.caseData, regions, store.catalog.problem.default_condition);
  const same = JSON.stringify([next.problem, next.view]) === JSON.stringify([store.caseData.problem, store.caseData.view]);
  if (same) return;
  editCase(
    (c) => {
      c.problem = next.problem;
      c.view = next.view;
    },
    "structure",
    { sections: ["problem", "view"] },
  );
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

// Checks the problem after a pause in typing (the settings: on or off,
// and the delay). `now`: a committed change (a field left, a template, a
// new case), checked after a short moment whatever the settings say.
// While a long stage runs, the server is busy (one mutex for all symbolic
// work): the check waits and runs once when the stage ends. A time run
// being followed does not hold it: the server answers between chunks.
let checkTimer = null;
let checkWaiting = false;
export function scheduleProblemCheck({ now = false } = {}) {
  if (!now && !settings.checkWhileTyping) return;
  clearTimeout(checkTimer);
  checkTimer = setTimeout(
    () => {
      if (longRunning()) checkWaiting = true;
      else run("problem");
    },
    now ? 50 : settings.checkDelayMs,
  );
}

// Fills the problem and the solve settings from a preset of the server's
// catalog, for the case's geometry. The toast offers to take it back.
export async function applyPreset(preset) {
  const caseId = store.caseId;
  const before = { problem: structuredClone(store.caseData.problem), solve: structuredClone(store.caseData.solve) };
  let next;
  try {
    const { format, version, geometry, problem, solve } = store.caseData;
    next = await api.preset({ format, version, geometry, problem, solve }, preset.id);
  } catch (error) {
    toast(`The preset needs a geometry the library accepts: ${error.message}`);
    return false;
  }
  if (caseId !== store.caseId) return false;
  const replace = (values) =>
    editCase(
      (c) => {
        c.problem = structuredClone(values.problem);
        c.solve = structuredClone(values.solve);
      },
      "structure",
      { sections: ["problem", "solve"] },
    );
  replace(next);
  scheduleProblemCheck({ now: true });
  toast(`${preset.label}: the problem and the solve settings are filled in.`, {
    action: {
      label: "Undo",
      run: () => {
        if (caseId !== store.caseId) return;
        replace(before);
        scheduleProblemCheck({ now: true });
        toast("The previous problem is back.");
      },
    },
  });
  return true;
}
