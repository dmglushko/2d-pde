// Wires the page: the catalog, the steps, the case file actions, undo
// and redo, and the picture.

import { api, saveBlob } from "./api.js";
import { caseChanged, runAll, runStage, scheduleProblemCheck, toast } from "./actions.js";
import { LABELS, comparisonValues, currentComparison, recompareIfNeeded } from "./compare.js";
import { fill, guardButtons, h, readFileText } from "./dom.js";
import { assemblePanel } from "./panels/assemble.js";
import { geometryPanel, loadDomainFile, scheduleGeometry } from "./panels/geometry.js";
import { dotClass, setInvalidNotice } from "./panels/head.js";
import { meshPanel } from "./panels/mesh.js";
import { problemPanel } from "./panels/problem.js";
import { solvePanel } from "./panels/solve.js";
import { forgetChartSize, profileDrawn, updateProbes } from "./probes.js";
import { fileKind, normalizeCase } from "./pure/case_model.js";
import { formatNumber, getDefaultDigits, pictureName } from "./pure/format.js";
import { MAX_PROBES, clampPictureHeight, freeProbeSlot } from "./pure/prefs.js";
import { nearestLoaded } from "./pure/run_values.js";
import { STEPS, stageInfo } from "./pure/stages.js";
import { onSettings, settings, wireSettings } from "./settings.js";
import { snapshotValues } from "./snapshots.js";
import {
  blockedReason,
  canRedo,
  canUndo,
  clearInvalid,
  currentError,
  currentKey,
  freshResult,
  lastResult,
  markSaved,
  openStep,
  redoCase,
  requestTicket,
  savedCase,
  sealHistory,
  setCase,
  setColorBy,
  setInputGroup,
  setLockedRange,
  setProbes,
  setViewOption,
  stageStatus,
  store,
  subscribe,
  undoCase,
} from "./state.js";
import { timeSolution, updateTimebar, wireTimebar } from "./timebar.js";
import { Viewer } from "./view/viewer.js";

const panels = { geometry: geometryPanel, mesh: meshPanel, problem: problemPanel, assemble: assemblePanel, solve: solvePanel };
for (const step of STEPS) if (!panels[step]) throw new Error(`no panel for the step ${step}`);
const STEP_KEY = "pde2d-app-step";
const HEIGHT_KEY = "pde2d-app-picture-height";
let viewer;

function element(id) {
  return document.getElementById(id);
}

// One tab and one panel per step of the stage registry. The tabs take
// the arrow keys, Home and End (the tablist pattern).
function buildSteps() {
  const tab = (step) =>
    h(
      "button",
      {
        role: "tab",
        type: "button",
        id: `tab-${step}`,
        "aria-controls": `panel-${step}`,
        dataset: { step },
        onclick: () => openStep(step),
      },
      h("span", { class: "dot", "aria-hidden": "true" }),
      h("span", { class: "tab-label" }, stageInfo(step).label),
      h("span", { class: "visually-hidden dot-text" }),
    );
  const list = document.querySelector(".steps");
  fill(list, STEPS.map(tab));
  list.addEventListener("keydown", (event) => {
    const i = STEPS.indexOf(store.step);
    const next = { ArrowRight: i + 1, ArrowLeft: i - 1, Home: 0, End: STEPS.length - 1 }[event.key];
    if (next === undefined) return;
    event.preventDefault();
    const step = STEPS[(next + STEPS.length) % STEPS.length];
    openStep(step);
    element(`tab-${step}`).focus();
  });
  fill(
    document.querySelector(".panels"),
    STEPS.map((step) => h("section", { id: `panel-${step}`, class: "panel", role: "tabpanel", "aria-labelledby": `tab-${step}`, hidden: true })),
  );
}

function applyStep() {
  const step = store.step;
  for (const button of document.querySelectorAll(".steps button")) {
    const selected = button.dataset.step === step;
    button.setAttribute("aria-selected", String(selected));
    button.tabIndex = selected ? 0 : -1;
  }
  for (const name of STEPS) element(`panel-${name}`).hidden = name !== step;
  try {
    localStorage.setItem(STEP_KEY, step);
  } catch {
    // A remembered step is only a convenience.
  }
}

let recovering = false;

// Builds every panel from the case. The fields then show the case's
// values, so no field holds invalid text any more.
function buildPanels() {
  try {
    for (const [name, panel] of Object.entries(panels)) panel.build(element(`panel-${name}`));
    clearInvalid();
  } catch (error) {
    // A case the forms cannot show: start over rather than fail on every
    // visit (the broken case would otherwise be restored from storage).
    console.error(error);
    if (recovering) throw error;
    recovering = true;
    try {
      toast(`This case cannot be shown (${error.message}); a new case was started.`);
      loadCase(structuredClone(store.catalog.default_case), { offerRestore: false });
    } finally {
      recovering = false;
    }
  }
}

const DOT_TITLES = { done: "up to date", stale: "outdated: the case changed", stopped: "stopped before the end", failed: "failed", busy: "running", none: "not run" };

function problemStatus() {
  if (store.busy.has("problem")) return "busy";
  if (currentError("problem")) return "failed";
  const check = freshResult("problem");
  if (check) return check.ok ? "done" : "failed";
  return stageStatus("problem");
}

function updateDots() {
  for (const button of document.querySelectorAll(".steps button")) {
    const step = button.dataset.step;
    const s = step === "problem" ? problemStatus() : stageStatus(step);
    const dot = button.querySelector(".dot");
    dot.className = `dot ${dotClass(s)}`.trim();
    button.title = DOT_TITLES[s];
    button.querySelector(".dot-text").textContent = `, ${DOT_TITLES[s]}`;
  }
}

// "Solving… 12 s" in the panel of every running stage.
function tick() {
  const now = performance.now();
  for (const note of document.querySelectorAll("[data-elapsed-stage]")) {
    const since = store.busySince[note.dataset.elapsedStage];
    if (since === undefined) continue;
    note.textContent = `${note.dataset.label}… ${Math.floor((now - since) / 1000)} s`;
  }
}

// --- the picture -------------------------------------------------------------

// The field the picture colours by: the solution snapshot (or its
// difference from the exact solution or the reference run, compare.js) or
// the last field of the Assemble step, whichever came last, while it is up
// to date. {name ("u", "u − exact", "Dirichlet values"), when ("at t =
// 0.5"), label (both), values, scale}.
function shownField() {
  const solved = freshResult("solve");
  if (store.fieldSource === "solve" && solved) {
    const i = Math.min(store.solveFrame, solved.times.length - 1);
    const t = formatNumber(solved.times[i], 5);
    const kind = store.view.difference;
    const comparison = kind ? currentComparison(kind) : null;
    const difference = comparison ? comparisonValues(kind, i) : null;
    const named = (name, when, values, scale) => ({ name, when, label: `${name} ${when}`, values, scale });
    if (difference) return named(LABELS[kind], `at t = ${t}`, difference, comparison.scale);
    // The page keeps some snapshots only (snapshots.js): until the shown
    // one arrives, the nearest loaded one stands in, with its own time.
    if (snapshotValues(solved, i)) return named("u", `at t = ${t}`, solved.values[i], solved.scale);
    const near = nearestLoaded(solved.values, i);
    if (near < 0) return null;
    return named("u", `at t = ${formatNumber(solved.times[near], 5)} (loading t = ${t})`, solved.values[near], solved.scale);
  }
  const field = store.results.field;
  if (!field || field.key !== currentKey("field")) return null;
  const data = field.data;
  const timeless = data.kind === "initial_u" || data.kind === "initial_u_t";
  const when = timeless || !Number.isFinite(data.t) ? "" : `at t = ${formatNumber(data.t, 5)}`;
  return { name: data.label, when, label: [data.label, when].filter(Boolean).join(" "), values: data.values, data };
}

// The field shown keeps its identity while nothing about it changed, so
// the picture is not drawn again.
let shown = null;
function stableField() {
  const next = shownField();
  if (next && shown && next.values === shown.values && next.scale === shown.scale && next.label === shown.label) return shown;
  // The Assemble step's field keeps its identity while its data does.
  if (next?.data && shown?.data === next.data) return shown;
  shown = next;
  return next;
}

// The Colour menu names the field it would show.
function updateColorMenu(field) {
  const menu = element("color-by");
  menu.value = store.view.colorBy;
  const name = field?.name ?? null;
  menu.querySelector('option[value="field"]').textContent = name ? `field: ${name}` : "field (none yet)";
  menu.querySelector('option[value="gradient"]').textContent = name ? `|∇| of ${name}` : "|∇| of the field";
}

function updateViewer() {
  const geometry = freshResult("geometry") ?? lastResult("geometry");
  const meshResult = lastResult("mesh");
  const assembled = lastResult("assemble");
  const mesh = meshResult?.mesh ?? null;
  const meshFresh = freshResult("mesh") !== null;
  const field = stableField();
  updateColorMenu(field);
  element("layer-isolines").checked = store.view.isolines;
  const profiling = store.view.tool === "profile";
  element("profile-tool").setAttribute("aria-pressed", String(profiling));
  element("profile-tool").disabled = !meshFresh;
  const drawn = viewer.set({
    geometry,
    mesh,
    meshStale: mesh !== null && !meshFresh,
    dirichlet: assembled && freshResult("assemble") ? assembled.meshed_problem.dirichlet_nodes : null,
    field,
    colorBy: store.view.colorBy,
    locked: store.view.locked,
    tool: store.view.tool,
    probes: store.probes,
    profile: meshFresh ? store.profile : null,
    layers: {
      geometry: element("layer-geometry").checked,
      mesh: element("layer-mesh").checked,
      dirichlet: element("layer-dirichlet").checked,
      isolines: store.view.isolines,
      isolineCount: settings.isolineCount,
    },
  });
  if (drawn) element("legend").replaceChildren(...viewer.legend());
  element("empty-view").hidden = Boolean(geometry || mesh);
  updateTimebar();
  const solution = timeSolution();
  const colouring = store.view.colorBy === "field" || store.view.colorBy === "gradient";
  updateProbes(element("probes"), {
    mesh: meshFresh ? mesh : null,
    field: colouring ? field : null,
    solution,
    frame: solution ? Math.min(store.solveFrame, solution.values.length - 1) : 0,
    digits: getDefaultDigits() + 1,
  });
}

// A click on the picture (or Enter) pins a probe there.
function pinProbe(x, y) {
  const slot = freeProbeSlot(store.probes);
  if (slot < 0) {
    toast(`Up to ${MAX_PROBES} probes: remove one first.`);
    return;
  }
  setProbes([...store.probes, { slot, x, y }]);
}

async function savePicture() {
  const solved = freshResult("solve");
  const t = store.view.colorBy !== "none" && store.view.colorBy !== "quality" && store.fieldSource === "solve" && solved ? solved.times[Math.min(store.solveFrame, solved.times.length - 1)] : null;
  const blob = await viewer.picture();
  if (!blob) return toast("The picture could not be saved.");
  saveBlob(blob, pictureName(store.caseName, t));
}

// At phone width the picture's height follows a drag handle under it (or
// its arrow keys), remembered in this browser.
function wireHeightHandle() {
  const handle = element("picture-handle");
  const wrap = element("canvas-wrap");
  const apply = (height) => {
    const clamped = clampPictureHeight(height, window.innerHeight);
    if (clamped === null) return;
    document.documentElement.style.setProperty("--picture-height", `${clamped}px`);
    handle.setAttribute("aria-valuenow", String(clamped));
    try {
      localStorage.setItem(HEIGHT_KEY, String(clamped));
    } catch {
      // Not kept: the height holds for this visit.
    }
  };
  try {
    const stored = Number(localStorage.getItem(HEIGHT_KEY));
    if (stored > 0) apply(stored);
  } catch {
    // The default height.
  }
  let start = null;
  handle.addEventListener("pointerdown", (event) => {
    start = { y: event.clientY, height: wrap.getBoundingClientRect().height };
    try {
      handle.setPointerCapture(event.pointerId);
    } catch {
      // A synthetic pointer.
    }
    event.preventDefault();
  });
  handle.addEventListener("pointermove", (event) => {
    if (start) apply(start.height + event.clientY - start.y);
  });
  const end = () => (start = null);
  handle.addEventListener("pointerup", end);
  handle.addEventListener("pointercancel", end);
  handle.addEventListener("keydown", (event) => {
    const delta = { ArrowUp: -24, ArrowDown: 24 }[event.key];
    if (!delta) return;
    event.preventDefault();
    apply(wrap.getBoundingClientRect().height + delta);
  });
}

// --- the case bar -------------------------------------------------------------

// The case's file name, whether it changed since, the case it replaced,
// undo and redo, and the notice.
function updateCaseBar() {
  element("unsaved").hidden = !store.dirty;
  element("save-case").title = `Save as ${store.caseName}`;
  const restore = element("restore-case");
  restore.hidden = !store.previousCase;
  if (store.previousCase) restore.title = `Open ${store.previousCase.name} again, as it was when it was replaced`;
  updateHistoryButtons();
  const notice = element("case-notice");
  if (notice.shown === store.notice) return;
  notice.shown = store.notice;
  fill(
    notice,
    store.notice
      ? h(
          "div",
          { class: "notice case-notice", role: "status" },
          h("span", {}, store.notice.text),
          h("button", { type: "button", class: "icon", "aria-label": "Dismiss", onclick: () => ((store.notice = null), updateCaseBar()) }, "✕"),
        )
      : null,
  );
}

function updateHistoryButtons() {
  element("undo").disabled = !canUndo();
  element("redo").disabled = !canRedo();
}

// Every step's head says which number fields are invalid; the run
// buttons are blocked while any are.
function updateInvalid() {
  for (const name of STEPS) setInvalidNotice(element(`panel-${name}`).head);
  guardButtons();
  const runAllButton = element("run-all");
  const reason = blockedReason();
  if (runAllButton.dataset.running === undefined) runAllButton.disabled = reason !== null;
  runAllButton.title = reason ? `Blocked: ${reason}` : "Geometry, mesh, assembly and solve, each only when it is not up to date";
}

function refresh(reason) {
  if (reason === "history") return updateHistoryButtons();
  if (reason === "invalid") return updateInvalid();
  if (reason === "frame" || reason === "probes") {
    updateViewer();
    panels.solve.frame?.(element("panel-solve"));
    return;
  }
  if (reason === "step") {
    applyStep();
    return;
  }
  if (reason === "progress" || reason === "compare") {
    // A time run or a comparison moved on: its panel, the dots and the
    // picture.
    panels.solve.update(element("panel-solve"));
    updateDots();
    updateViewer();
    if (reason === "progress") recompareIfNeeded();
    return;
  }
  caseChanged();
  // The problem check is cheap: keep it current whatever changed the case
  // (typing waits for the pause the settings say).
  const problem = stageStatus("problem");
  if (problem === "stale" || problem === "none") scheduleProblemCheck({ now: reason !== "edit" });
  if (reason === "case" || reason === "structure") buildPanels();
  else for (const [name, panel] of Object.entries(panels)) panel.update(element(`panel-${name}`));
  updateDots();
  updateViewer();
  updateCaseBar();
  updateInvalid();
  tick();
  if (reason === "result") recompareIfNeeded();
}

// A case read from a file or an example replaces the current one; every
// result is dropped. `name` is the file name it is saved under. The case
// it replaced can be restored until the next replacement.
function loadCase(input, { name = "case.json", dirty = false, offerRestore = true, message = null } = {}) {
  let normalized;
  try {
    normalized = normalizeCase(input, store.catalog.default_case);
  } catch (error) {
    toast(error.message);
    return false;
  }
  const replaced = store.caseData ? { caseData: store.caseData, name: store.caseName, dirty: store.dirty } : null;
  store.notice = normalized.replaced.length ? { text: `Replaced by defaults when the case was read: ${normalized.replaced.join(", ")}.` } : null;
  store.previousCase = offerRestore ? replaced : null;
  for (const stage of Object.keys(store.results)) store.results[stage] = null;
  setCase(normalized.caseData, { name, dirty: dirty || normalized.replaced.length > 0 });
  runStage("geometry");
  const text = message ?? `Opened ${name}.`;
  if (offerRestore && replaced) toast(`${text} The previous case can be restored from the top bar.`, { action: { label: "Restore it", run: restorePrevious } });
  else if (message) toast(message);
  return true;
}

// Opens the case the last replacement replaced (the one replaced now can
// in turn be restored).
function restorePrevious() {
  const back = store.previousCase;
  if (!back) return;
  requestTicket("open");
  loadCase(back.caseData, { name: back.name, dirty: back.dirty, message: `Restored ${back.name}.` });
}

// Every way to replace the case takes an "open" ticket: an example or a
// file still loading never replaces a case opened or started meanwhile.
function newCase() {
  requestTicket("open");
  loadCase(structuredClone(store.catalog.default_case), { message: "Started a new case." });
}

function saveCase() {
  const text = JSON.stringify(store.caseData, null, 2);
  saveBlob(new Blob([text + "\n"], { type: "application/json" }), store.caseName);
  markSaved();
  const reason = blockedReason();
  if (reason) toast(`Saved ${store.caseName}, but ${reason[0].toLowerCase()}${reason.slice(1)}: the file has the previous value.`);
}

async function openFile(file) {
  const kind = fileKind(file.name);
  if (kind === "case") {
    const ticket = requestTicket("open");
    let input;
    try {
      input = JSON.parse(await readFileText(file));
    } catch {
      if (ticket.live()) toast(`${file.name} is not valid JSON.`);
      return;
    }
    if (ticket.live()) loadCase(input, { name: file.name });
  } else {
    await loadDomainFile(file);
    openStep("geometry");
  }
}

async function openExample(name) {
  const ticket = requestTicket("open");
  try {
    const example = await api.example(name);
    if (ticket.live()) loadCase(example, { name });
  } catch (error) {
    if (ticket.live()) toast(error.message);
  }
}

function wireDrop() {
  const wrap = element("canvas-wrap");
  wrap.addEventListener("dragover", (event) => {
    event.preventDefault();
    wrap.classList.add("over");
  });
  wrap.addEventListener("dragleave", () => wrap.classList.remove("over"));
  wrap.addEventListener("drop", (event) => {
    event.preventDefault();
    wrap.classList.remove("over");
    const file = event.dataTransfer.files[0];
    if (file) openFile(file);
  });
}

// --- undo and redo -------------------------------------------------------------

// Undo or redo, then the geometry is read again when it changed.
function history(redo) {
  const sections = redo ? redoCase() : undoCase();
  if (!sections) return;
  if (sections.includes("geometry")) scheduleGeometry();
  toast(redo ? "Redone." : "Undone.");
}

// A text field that was typed in since it got the focus keeps Ctrl+Z
// for its own undo (the browser's, character by character); anywhere
// else, Ctrl+Z and Ctrl+Shift+Z (or Ctrl+Y) undo and redo case edits.
// Typing in one field is one step of the case's history.
function wireHistory() {
  const typed = new WeakSet();
  const isText = (target) => target instanceof HTMLTextAreaElement || (target instanceof HTMLInputElement && ["text", "search", ""].includes(target.type));
  document.addEventListener(
    "input",
    (event) => {
      setInputGroup(event.target);
      if (isText(event.target)) typed.add(event.target);
    },
    true,
  );
  document.addEventListener("input", () => setInputGroup(null));
  document.addEventListener("focusin", (event) => typed.delete(event.target));
  document.addEventListener("focusout", () => sealHistory());
  document.addEventListener("keydown", (event) => {
    if (!(event.ctrlKey || event.metaKey) || event.altKey) return;
    const key = event.key.toLowerCase();
    const redo = (key === "z" && event.shiftKey) || key === "y";
    if (key !== "z" && key !== "y") return;
    if (isText(event.target) && typed.has(event.target)) return;
    if (event.target.closest?.("dialog")) return;
    event.preventDefault();
    history(redo);
  });
  element("undo").addEventListener("click", () => history(false));
  element("redo").addEventListener("click", () => history(true));
}

// --- start -----------------------------------------------------------------------

async function start() {
  buildSteps();
  try {
    store.catalog = await api.catalog();
  } catch (error) {
    fill(document.querySelector(".panels"), h("div", { class: "alert" }, error.message));
    return;
  }
  element("version").textContent = `library ${store.catalog.library_version}`;
  viewer = new Viewer(element("view"), element("tooltip"), element("view-status"), {
    onPin: pinProbe,
    onProfile: profileDrawn,
    onLock: (quantity, range) => setLockedRange(quantity, range),
  });
  wireTimebar(element("timebar"));
  new ResizeObserver(() => {
    forgetChartSize();
    if (store.catalog) updateViewer();
  }).observe(element("probes"));

  let step = "geometry";
  try {
    step = localStorage.getItem(STEP_KEY) ?? step;
  } catch {
    // Start on the first step.
  }
  store.step = STEPS.includes(step) ? step : "geometry";
  applyStep();

  for (const id of ["layer-geometry", "layer-mesh", "layer-dirichlet"]) element(id).addEventListener("change", updateViewer);
  wireSettings(element("open-settings"), element("settings"));
  onSettings((key) => {
    if (key === "pixelRatioCap") viewer.resize();
    else if (key === "isolineCount") updateViewer();
    // Digits and the playback speed show in the panels.
    else if (key === "digits" || key === "playbackFps") {
      buildPanels();
      updateDots();
      viewer.draw();
      element("legend").replaceChildren(...viewer.legend());
    }
  });
  element("color-by").addEventListener("change", (event) => setColorBy(event.target.value));
  element("layer-isolines").addEventListener("change", (event) => setViewOption("isolines", event.target.checked));
  element("fit-view").addEventListener("click", () => viewer.fit());
  element("zoom-in").addEventListener("click", () => viewer.zoomBy(1.25));
  element("zoom-out").addEventListener("click", () => viewer.zoomBy(0.8));
  element("profile-tool").addEventListener("click", () => setViewOption("tool", store.view.tool === "profile" ? "pan" : "profile"));
  element("save-picture").addEventListener("click", () => savePicture().catch((error) => toast(error.message)));
  element("new-case").addEventListener("click", newCase);
  element("restore-case").addEventListener("click", restorePrevious);
  element("save-case").addEventListener("click", saveCase);
  element("open-case-button").addEventListener("click", () => element("open-case").click());
  element("open-case").addEventListener("change", (event) => {
    const file = event.target.files[0];
    if (file) openFile(file);
    event.target.value = "";
  });
  element("run-all").addEventListener("click", async (event) => {
    const button = event.currentTarget;
    button.disabled = true;
    button.dataset.running = "";
    try {
      if (await runAll()) toast("Geometry, mesh, assembly and solve are up to date.");
    } finally {
      delete button.dataset.running;
      updateInvalid();
    }
  });
  const examples = element("examples");
  api
    .examples()
    .then((list) => {
      for (const example of list) examples.append(h("option", { value: example.name, title: example.notes ?? "" }, example.name.replace(/\.json$/, "").replaceAll("_", " ")));
    })
    .catch(() => {});
  examples.addEventListener("change", () => {
    const name = examples.value;
    examples.value = "";
    if (name) openExample(name);
  });
  wireDrop();
  wireHistory();
  wireHeightHandle();
  setInterval(tick, 1000);

  subscribe(refresh);
  const saved = savedCase();
  if (!saved || !loadCase(saved.caseData, { name: saved.name, dirty: saved.dirty, offerRestore: false })) loadCase(structuredClone(store.catalog.default_case), { offerRestore: false });
}

start();
