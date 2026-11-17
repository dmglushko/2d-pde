// Running the stages on the server and keeping their results.

import { api, ApiError, downloadExport } from "./api.js";
import { reconcileConditions } from "./pure/case_model.js";
import { fieldScale } from "./pure/color.js";
import { RUN_ALL, STAGES, requestCase, stageInfo, stagesToRun } from "./pure/stages.js";
import { settings } from "./settings.js";
import { backoffDelay, isTransient, plainError } from "./pure/requests.js";
import { growSnapshots, loadAround, retrySnapshots } from "./snapshots.js";
import {
  blockedReason,
  currentError,
  currentKey,
  editCase,
  freshResult,
  notify,
  openStep,
  requestTicket,
  sectionRevision,
  setColorBy,
  store,
  supersede,
} from "./state.js";

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

let geometryTimer = null;

// Reads the geometry a moment after the last change (typing, the end of
// a drag in the drawing editor).
export function scheduleGeometry() {
  clearTimeout(geometryTimer);
  geometryTimer = setTimeout(() => runStage("geometry"), 250);
}

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
  solve: async (caseData) => solveData(await api.solve(caseData)),
};

// A run's progress made into the page's solve result: its values are
// loaded on demand (snapshots.js), the newest first.
async function solveData(summary) {
  summary.layout = summary.values;
  summary.values = [];
  growSnapshots(summary, summary.layout.snapshots);
  // An adaptive run's solution is on its own (last) mesh, with an error
  // indicator per triangle: both come with the result.
  const adapted = summary.adaptive?.mesh;
  if (adapted) {
    summary.mesh = await api.meshArrays(adapted);
    if (summary.adaptive.indicators) summary.indicators = await api.indicators(summary.job, summary.adaptive.indicators);
  }
  if (summary.values.length > 0) await loadAround(summary, summary.values.length - 1);
  return summary;
}
for (const stage of STAGES) if (!calls[stage.call]) throw new Error(`no call for ${stage.id}`);

// Requests overlap (a slow mesh, a quick edit, a new case). Each run of a
// stage takes a request ticket (state.js) of the stage's kind for its
// stage key: only the newest run of a stage, for the case that is still
// loaded, may change the state (a result whose key changed since is kept,
// as outdated). Older answers are dropped.

// A time run being followed: {job, key, ticket}; while one is,
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
// (its answer is then dropped). `found`: for the solve, the progress of a
// run the server already has (reattachRun): it is followed, not started.
async function run(stage, field, found = null) {
  const ticket = requestTicket(stage, () => currentKey(stage));
  const key = ticket.key;
  const caseData = structuredClone(requestCase(store.caseData, stage));
  const current = () => ticket.live();
  if (stage === "solve") stopFollowing();
  store.busy.add(stage);
  store.busySince[stage] = globalThis.performance?.now() ?? Date.now();
  notify("result");
  try {
    const data = found ? await solveData(found) : await calls[stageInfo(stage).call](caseData, field);
    if (!current()) return null;
    // A check of a problem that was edited since is not worth showing: a
    // newer one is on its way.
    if (stage === "problem" && !ticket.holds()) return null;
    if (stage === "solve") {
      solveScale(data);
      store.solveFrame = data.times.length - 1;
      store.solveFrameChosen = false;
      store.fieldSource = "solve";
      // A reattached run keeps the colouring this tab had (restoreView).
      if (!(found && (store.view.colorBy === "gradient" || store.view.colorBy === "indicator"))) store.view.colorBy = "field";
    }
    if (stage === "field") {
      store.fieldSource = "field";
      store.view.colorBy = "field";
    }
    store.results[stage] = { key, data };
    store.errors[stage] = null;
    if (stage === "geometry") syncConditions(data, key);
    if (stage === "solve" && data.state === "running") {
      following = { job: data.job, key, ticket };
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
    if (ticket.newest()) {
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
// user chose one with the slider, or the setting is off). A poll the
// server refused for the moment (busy) or that got no answer is tried
// again after a growing pause (pure/requests.js, backoffDelay), and the
// Solve step says so (store.runNotice); a run that was replaced or is gone
// ends the following with its plain message. Returns true when done,
// false when it failed, null when it was stopped or replaced.
async function followRun(run, data) {
  const live = () => following === run && run.ticket.live();
  let attempt = 0;
  while (live()) {
    await wait(attempt === 0 ? POLL_MS : backoffDelay(attempt - 1));
    if (!live()) return null;
    let progress;
    try {
      progress = await api.solveProgress(run.job);
    } catch (error) {
      if (!live()) return null;
      if (isTransient(error)) {
        const delay = backoffDelay(attempt);
        attempt += 1;
        store.runNotice = `${plainError(error, "the run's progress").text} Trying again in ${Math.max(1, Math.round(delay / 1000))} s; the run goes on on the server.`;
        notify("progress");
        continue;
      }
      following = null;
      store.runNotice = null;
      const failure = error instanceof ApiError ? error : new ApiError({ stage: "solve", message: String(error) });
      failure.key = run.key;
      store.errors.solve = failure;
      data.state = "stopped";
      return false;
    }
    if (!live()) return null;
    if (attempt > 0 || store.runNotice) {
      attempt = 0;
      store.runNotice = null;
      // The server answers again: snapshots that failed meanwhile are
      // tried again.
      retrySnapshots(data, { quiet: true });
    }
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

// After a reload of the tab (the same client id, api.js): the server may
// still have this client's run of the case. If it does, the page shows it
// (and follows it while it runs) instead of nothing; the mesh is fetched
// first, since the solution is drawn on it. A run that is gone, or one of
// other inputs, shows nothing. Returns true when a run was found.
export async function reattachRun() {
  const ticket = requestTicket("solve", () => currentKey("solve"));
  let answer;
  try {
    answer = await api.currentRun(structuredClone(requestCase(store.caseData, "solve")));
  } catch {
    return false;
  }
  const progress = answer?.run;
  if (!progress || progress.state === "failed" || !ticket.holds()) return false;
  if (!isFresh("mesh")) {
    for (const id of stagesToRun("mesh", isFresh)) if ((await run(id)) !== true) return false;
  }
  // A solve started meanwhile wins.
  if (!ticket.holds() || !ticket.newest()) return false;
  return (await run("solve", undefined, progress)) === true;
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
  if (following && !following.ticket.holds()) {
    stopFollowing();
    supersede("solve");
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

// Runs `stage` after its prerequisites (pure/stages.js, STAGES) that are not
// fresh, stopping at the first failure with a toast that opens the
// failing step. `field` is the field of a "field" run. Returns true when
// the stage ran and worked, false after a failure, null when superseded.
export async function runWithPrerequisites(stage, field) {
  if (refuseWhileInvalid()) return false;
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

// While a number field holds text that is not a number, the case still
// has its previous value: running would compute something the page does
// not show. Says so and returns true.
function refuseWhileInvalid() {
  const reason = blockedReason();
  if (!reason) return false;
  toast(`Fix it first: ${reason}. The case still has the previous value.`);
  return true;
}

// Evaluates a field on the mesh (docs/app.md, "The API").
export function showField(field) {
  return run("field", field);
}

// After a geometry the library accepted: every region gets a boundary
// condition (pure/case_model.js, reconcileConditions). Only for the geometry that is
// still in the case.
function syncConditions(geometryView, key) {
  if (key !== currentKey("geometry")) return;
  const regions = geometryView.regions.map((r) => r.name);
  const next = reconcileConditions(store.caseData, regions, store.catalog.problem.default_condition);
  const same = JSON.stringify([next.problem, next.view]) === JSON.stringify([store.caseData.problem, store.caseData.view]);
  if (same) return;
  // The page's own edit: the case stays as saved (or not) as it was.
  editCase(
    (c) => {
      c.problem = next.problem;
      c.view = next.view;
    },
    "structure",
    { sections: ["problem", "view"], automatic: true },
  );
}

// Geometry, mesh, assembly and solve in turn, skipping a stage that is
// fresh; stops at the first failure (with the toast of
// runWithPrerequisites). Returns true when all are up to date.
export async function runAll() {
  if (refuseWhileInvalid()) return false;
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
// catalog, for the case's geometry. The answer is applied only when the
// geometry, problem and solve sections are still the ones it was made for
// (an edit made meanwhile is never overwritten). The toast offers to take
// it back.
export async function applyPreset(preset) {
  const ticket = requestTicket("preset", () => sectionRevision("geometry", "problem", "solve"));
  const before = { problem: structuredClone(store.caseData.problem), solve: structuredClone(store.caseData.solve) };
  let next;
  try {
    const { format, version, geometry, problem, solve } = store.caseData;
    next = await api.preset({ format, version, geometry, problem, solve }, preset.id);
  } catch (error) {
    if (ticket.live()) toast(`The preset needs a geometry the library accepts: ${error.message}`);
    return false;
  }
  if (!ticket.live()) return false;
  if (!ticket.holds()) {
    toast(`${preset.label} was not applied: the case changed while it was prepared. Choose it again to apply it.`);
    return false;
  }
  const caseId = store.caseId;
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

// --- exports ------------------------------------------------------------------------

// An error of an export in plain words: a result that is not ready (409)
// or a server that is busy (429) says what to do.
export function exportErrorText(kind, error) {
  const message = String(error?.message ?? error);
  const sentence = message.charAt(0).toUpperCase() + message.slice(1);
  if (error?.kind === "not_ready") return `${kind} is not ready: ${sentence.replace(/\.$/, "")}.`;
  if (error?.kind === "busy") return "The app is busy with earlier requests. Try the export again in a moment.";
  return `${kind} could not be exported: ${sentence}`;
}

// Saves an export of the case, named after the case in its provenance.
export async function exportFile(kind) {
  try {
    await downloadExport(kind, store.caseData, store.caseName);
  } catch (error) {
    toast(exportErrorText(kind, error));
  }
}
