// Wires the page: the catalog, the steps, the case file actions, undo
// and redo, and the picture.

import { api, claimClientId, saveBlob } from "./api.js";
import { applyPreset, caseChanged, reattachRun, runAll, runStage, runWithPrerequisites, scheduleProblemCheck, toast } from "./actions.js";
import { wireCaseMenu } from "./case_menu.js";
import { clearStudy } from "./refinement.js";
import { watchServer } from "./server_status.js";
import { setColorMaps } from "./view/tokens.js";
import { recompareIfNeeded, retryDifference } from "./compare.js";
import { currentDisplay, drawnSolve, offersSnapshotRange, pictureMesh } from "./display.js";
import { plainError } from "./pure/requests.js";
import { fill, guardButtons, h, readFileText, revealFolded } from "./dom.js";
import { assemblePanel } from "./panels/assemble.js";
import { geometryPanel, loadDomainFile, scheduleGeometry } from "./panels/geometry.js";
import { initEditor, updateEditor } from "./geometry_editor.js";
import { openRegionPopover, wirePopoverClosing } from "./region_popover.js";
import { dotClass, setInvalidNotice } from "./panels/head.js";
import { meshPanel } from "./panels/mesh.js";
import { problemPanel } from "./panels/problem.js";
import { solvePanel } from "./panels/solve.js";
import { forgetChartSize, profileDrawn, updateProbes } from "./probes.js";
import { wirePanes } from "./panes.js";
import { fileKind, normalizeCase } from "./pure/case_model.js";
import { formatNumber, getDefaultDigits, pictureName } from "./pure/format.js";
import { MAX_PROBES, clampPictureHeight, clampSidebarWidth, freeProbeSlot } from "./pure/prefs.js";
import { STEPS, stageInfo } from "./pure/stages.js";
import { timeSizers } from "./timebar.js";
import { onSettings, settings, wireSettings } from "./settings.js";
import { retrySnapshots } from "./snapshots.js";
import {
  keptView,
  restoreView,
  setSelectedRegion,
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
import { speedChanged, timeSolution, updateTimebar, wireTimebar } from "./timebar.js";
import { Viewer } from "./view/viewer.js";

const panels = { geometry: geometryPanel, mesh: meshPanel, problem: problemPanel, assemble: assemblePanel, solve: solvePanel };
for (const step of STEPS) if (!panels[step]) throw new Error(`no panel for the step ${step}`);
const STEP_KEY = "pde2d-app-step";
const HEIGHT_KEY = "pde2d-app-picture-height";
const SIDEBAR_KEY = "pde2d-app-sidebar-width";
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

// What the picture shows is resolved once per refresh (display.js); the
// field drawn keeps its identity while nothing about it changed, so the
// picture is not drawn again.
let shown = null;
function drawnField(display) {
  if (!display.values) return (shown = null);
  const label = display.outdated ? `${display.label} (outdated)` : display.label;
  const next = { name: display.name, when: display.when, label, values: display.values, scale: display.scale, data: display.data, outdated: display.outdated, indicators: display.indicators ?? null };
  if (shown && shown.values === next.values && shown.scale === next.scale && shown.label === next.label && shown.outdated === next.outdated && shown.indicators === next.indicators) return shown;
  // The Assemble step's field keeps its identity while its data does.
  if (next.data && shown?.data === next.data && shown.outdated === next.outdated) return shown;
  shown = next;
  return next;
}

// The Colour menu names the quantity the picture would show.
function updateColorMenu(display) {
  const menu = element("color-by");
  menu.value = store.view.colorBy;
  const name = display.name;
  menu.querySelector('option[value="field"]').textContent = name ? `field: ${name}${display.outdated ? " (outdated)" : ""}` : "field (none yet)";
  menu.querySelector('option[value="gradient"]').textContent = name ? `|∇| of ${name}` : "|∇| of the field";
  // Only an adaptive solution has error indicators.
  const indicator = menu.querySelector('option[value="indicator"]');
  indicator.disabled = !display.indicators && store.view.colorBy !== "indicator";
  indicator.title = display.indicators ? "" : "Only an adaptive solve gives error indicators";
}

// Over the picture: the "outdated" stamp of a result built for an earlier
// case, and what is loading or failed (with Retry, or Solve again for a
// result the server no longer has).
function updateOverlay(display, colouring) {
  const stamp = element("view-stamp");
  stamp.hidden = !(colouring && display.values && display.outdated);
  const overlay = element("view-overlay");
  const failure = colouring ? display.failure : null;
  const loading = colouring && !display.values && !failure ? display.loading : null;
  const key = failure ? `f|${display.label}|${failure.error?.message}` : loading ? `l|${loading}` : "";
  if (overlay.shown === key) return;
  overlay.shown = key;
  overlay.hidden = !key;
  overlay.classList.toggle("failed", Boolean(failure));
  if (!key) return fill(overlay);
  if (loading) return fill(overlay, h("span", { class: "busy", "aria-hidden": "true" }), h("span", {}, loading));
  const what = failure.retry === "difference" ? `${display.name} at this snapshot` : "this snapshot";
  const plain = plainError(failure.error, what);
  const retry = () => {
    if (failure.retry === "difference") retryDifference(display.difference.requested, display.frame);
    else retrySnapshots(freshResult("solve") ?? lastResult("solve"));
  };
  fill(
    overlay,
    h("span", {}, plain.text),
    plain.action === "solve" ? h("button", { type: "button", class: "small", onclick: () => runWithPrerequisites("solve") }, "Solve again") : null,
    plain.action === "retry" ? h("button", { type: "button", class: "small", onclick: retry }, "Retry") : null,
  );
}

function updateViewer() {
  const geometry = freshResult("geometry") ?? lastResult("geometry");
  const assembled = lastResult("assemble");
  // An adaptive solution is drawn on its own mesh (display.js).
  const { mesh, fresh: meshFresh } = pictureMesh();
  const ownMesh = mesh !== null && mesh !== lastResult("mesh")?.mesh;
  const display = currentDisplay();
  const field = drawnField(display);
  const colouring = ["field", "gradient", "indicator"].includes(store.view.colorBy);
  updateColorMenu(display);
  updateOverlay(display, colouring);
  element("layer-isolines").checked = store.view.isolines;
  const profiling = store.view.tool === "profile";
  element("profile-tool").setAttribute("aria-pressed", String(profiling));
  element("profile-tool").disabled = !meshFresh;
  const drawn = viewer.set({
    geometry,
    mesh,
    meshStale: mesh !== null && !meshFresh,
    // The Dirichlet nodes are the Mesh step's mesh's.
    dirichlet: assembled && freshResult("assemble") && !ownMesh ? assembled.meshed_problem.dirichlet_nodes : null,
    field,
    colorBy: store.view.colorBy,
    locked: store.view.locked,
    tool: store.view.tool,
    probes: store.probes,
    profile: meshFresh ? store.profile : null,
    selectedRegion: store.selectedRegion,
    legendNote: colouring ? display.difference.note : null,
    rangeChoice: display.kind === "solution" && offersSnapshotRange(drawnSolve()?.data) ? store.view.range : null,
    // The legend's time keeps the room of the run's widest one.
    whenSizer: timeSizers(drawnSolve()?.data).when,
    layers: {
      geometry: element("layer-geometry").checked,
      mesh: element("layer-mesh").checked,
      dirichlet: element("layer-dirichlet").checked,
      isolines: store.view.isolines,
      isolineCount: settings.isolineCount,
    },
  });
  if (drawn) showLegend();
  element("empty-view").hidden = Boolean(geometry || mesh);
  updateTimebar();
  const solution = timeSolution();
  updateProbes(element("probes"), {
    mesh: meshFresh ? mesh : null,
    field: colouring ? field : null,
    display: colouring ? display : null,
    solution,
    frame: display.frame,
    digits: getDefaultDigits() + 1,
  });
}

// The legend again, every frame of a run that plays. A control in it that
// has the focus (the range choice, a locked range's ends) is kept where
// it is, with what is around it patched (patchChildren), so it keeps the
// focus and what is typed in it.
function showLegend() {
  patchChildren(element("legend"), viewer.legend());
}

function patchChildren(parent, next) {
  const active = document.activeElement;
  const old = [...parent.childNodes];
  if (!active || active === document.body || !parent.contains(active) || old.length !== next.length) return parent.replaceChildren(...next);
  next.forEach((node, i) => {
    const kept = old[i];
    if (kept === active) {
      if (kept instanceof HTMLSelectElement) kept.value = node.value;
      return;
    }
    if (kept.contains?.(active) && kept.nodeName === node.nodeName && kept.className === node.className) return patchChildren(kept, [...node.childNodes]);
    kept.replaceWith(node);
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

// On a wide screen the sidebar's width follows the handle on its right
// edge (or its arrow keys; a double click gives the default back),
// remembered in this browser. The picture follows through its resize
// observer, once per animation frame of the drag.
function wireSidebarHandle() {
  const handle = element("sidebar-handle");
  const sidebar = element("sidebar");
  const root = document.documentElement;
  // The width chosen (null: the default), apart from the width shown,
  // which a narrower window clamps: a window made wide again gets the
  // chosen width back.
  let preferred = null;
  const show = () => {
    const width = preferred === null ? null : clampSidebarWidth(preferred, window.innerWidth);
    if (width === null) root.style.removeProperty("--sidebar-width");
    else root.style.setProperty("--sidebar-width", `${width}px`);
    // The separator's values: the width shown and its finite limits.
    handle.setAttribute("aria-valuemin", "300");
    handle.setAttribute("aria-valuemax", String(Math.max(300, Math.round(window.innerWidth - 360))));
    handle.setAttribute("aria-valuenow", String(Math.round(sidebar.getBoundingClientRect().width)));
  };
  const choose = (width) => {
    const clamped = clampSidebarWidth(width, window.innerWidth);
    if (clamped === null) return;
    preferred = clamped;
    show();
    try {
      localStorage.setItem(SIDEBAR_KEY, String(clamped));
    } catch {
      // Not kept: the width holds for this visit.
    }
  };
  try {
    const stored = Number(localStorage.getItem(SIDEBAR_KEY));
    if (stored > 0) preferred = stored;
  } catch {
    // The default width.
  }
  show();
  let start = null;
  let pending = null;
  handle.addEventListener("pointerdown", (event) => {
    if (event.button !== 0) return;
    start = { x: event.clientX, width: sidebar.getBoundingClientRect().width };
    try {
      handle.setPointerCapture(event.pointerId);
    } catch {
      // A synthetic pointer.
    }
    handle.classList.add("dragging");
    document.body.classList.add("resizing-sidebar");
    event.preventDefault();
  });
  handle.addEventListener("pointermove", (event) => {
    if (!start) return;
    const width = start.width + event.clientX - start.x;
    if (pending === null) requestAnimationFrame(() => (choose(pending), (pending = null)));
    pending = width;
  });
  const end = () => {
    start = null;
    handle.classList.remove("dragging");
    document.body.classList.remove("resizing-sidebar");
  };
  handle.addEventListener("pointerup", end);
  handle.addEventListener("pointercancel", end);
  handle.addEventListener("dblclick", () => {
    preferred = null;
    try {
      localStorage.removeItem(SIDEBAR_KEY);
    } catch {
      // Nothing kept.
    }
    show();
  });
  handle.addEventListener("keydown", (event) => {
    const delta = { ArrowLeft: -24, ArrowRight: 24 }[event.key];
    if (!delta) return;
    event.preventDefault();
    choose(sidebar.getBoundingClientRect().width + delta);
  });
  window.addEventListener("resize", show);
}

// --- the case bar -------------------------------------------------------------

// The case's file name, whether it changed since, the case it replaced,
// undo and redo, and the notice.
function updateCaseBar() {
  element("unsaved").hidden = !store.dirty;
  element("unsaved").title = `${store.caseName} has changes that are not saved`;
  // The case's name, unless it is the name every new case has.
  element("case-name").textContent = store.caseName === "case.json" ? "" : `: ${store.caseName.replace(/\.json$/i, "")}`;
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

// The control the user just used keeps its place on the screen while the
// panels change around it: a result that goes outdated drops sections
// above the settings, and a choice must not jump away under the pointer.
// The panel may be rebuilt, so the control is found again by its id, its
// name (a radio group, by position) or its field's case path.
function placeOf(element) {
  if (element.id) return { element, find: (root) => root.querySelector(`#${CSS.escape(element.id)}`) };
  if (element.name) {
    const selector = `[name="${CSS.escape(element.name)}"]`;
    const index = [...document.querySelectorAll(selector)].indexOf(element);
    return { element, find: (root) => root.querySelectorAll(selector)[index] ?? null };
  }
  const field = element.closest("[data-path]");
  if (!field) return null;
  const selector = `[data-path="${CSS.escape(field.dataset.path)}"]`;
  return { element: field, find: (root) => root.querySelector(selector) };
}

function keepingPlace(update) {
  const scroller = document.querySelector(".panels");
  const active = document.activeElement;
  const place = scroller && active && active !== document.body && scroller.contains(active) ? placeOf(active) : null;
  if (!place) return update();
  const before = place.element.getBoundingClientRect().top;
  try {
    return update();
  } finally {
    const now = place.element.isConnected ? place.element : place.find(scroller);
    if (now?.offsetParent) {
      const moved = now.getBoundingClientRect().top - before;
      if (Math.abs(moved) >= 1) scroller.scrollTop += moved;
    }
  }
}

function refresh(reason) {
  keepingPlace(() => refreshNow(reason));
}

function refreshNow(reason) {
  if (reason === "history") return updateHistoryButtons();
  if (reason === "invalid") return updateInvalid();
  if (reason === "frame" || reason === "probes") {
    updateViewer();
    panels.solve.frame?.(element("panel-solve"));
    return;
  }
  if (reason === "step") {
    applyStep();
    // The editor's tools and what the picture draws of a drawing follow
    // the step.
    updateEditor(reason);
    viewer.draw();
    return;
  }
  if (reason === "study") {
    panels.solve.study?.(element("panel-solve"));
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
  if (reason === "case") clearStudy();
  if (reason === "case" || reason === "structure") buildPanels();
  else for (const [name, panel] of Object.entries(panels)) panel.update(element(`panel-${name}`));
  updateDots();
  updateEditor(reason);
  updateViewer();
  // A drawing edited (or undone) redraws over the picture even when the
  // server's view of it did not change yet.
  viewer.requestOverlay();
  updateCaseBar();
  updateInvalid();
  tick();
  if (reason === "result") recompareIfNeeded();
}

// A case read from a file or an example replaces the current one; every
// result is dropped. `name` is the file name it is saved under. The case
// it replaced can be restored until the next replacement.
function loadCase(input, { name = "case.json", dirty = false, offerRestore = true } = {}) {
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
  // Normalizing is the page's own work: the notice says what it replaced,
  // and the case stays as saved (or not) as it was.
  setCase(normalized.caseData, { name, dirty });
  runStage("geometry");
  // No toast: the case's name is in the top bar, and so is Restore.
  return true;
}

// Opens the case the last replacement replaced (the one replaced now can
// in turn be restored).
function restorePrevious() {
  const back = store.previousCase;
  if (!back) return;
  requestTicket("open");
  loadCase(back.caseData, { name: back.name, dirty: back.dirty });
}

// Every way to replace the case takes an "open" ticket: an example or a
// file still loading never replaces a case opened or started meanwhile.
function newCase() {
  requestTicket("open");
  loadCase(structuredClone(store.catalog.default_case));
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

// --- boundary regions ------------------------------------------------------------

// A boundary region chosen in the picture (a click on it) or its legend:
// it is highlighted, and its condition opens in the Problem step (or its
// row in the Geometry step, when that step is open).
function selectRegion(name, at = null) {
  setSelectedRegion(name);
  if (store.step === "geometry") {
    // A click on the picture: the region's actions right there.
    if (at) openRegionPopover(at, { region: name });
    const row = [...document.querySelectorAll("#geometry-regions tr[data-region]")].find((r) => r.dataset.region === name);
    for (const other of document.querySelectorAll("#geometry-regions tr.selected")) other.classList.remove("selected");
    if (row) {
      row.classList.add("selected");
      revealFolded(row);
      row.scrollIntoView({ block: "nearest" });
      if (!at) row.querySelector("button")?.focus({ preventScroll: true });
    }
    return;
  }
  openStep("problem");
  const card = document.querySelector(`#panel-problem [data-path="${CSS.escape(`problem.boundary_conditions.${name}`)}"]`);
  if (!card) return toast(`${name} has no condition yet: the geometry is read first.`);
  const toggle = card.querySelector(":scope > .card-head .fold-toggle");
  if (toggle?.getAttribute("aria-expanded") === "false") toggle.click();
  revealFolded(card);
  for (const other of document.querySelectorAll("#panel-problem .selected-region")) other.classList.remove("selected-region");
  card.classList.add("selected-region");
  card.scrollIntoView({ block: "nearest" });
  (card.querySelector("input.expr") ?? toggle)?.focus({ preventScroll: true });
}

// At phone width the picture can be folded away ("Hide picture"), so the
// panel has the screen; kept in this browser.
const COLLAPSE_KEY = "pde2d-app-picture-collapsed";
function wireCollapse() {
  const button = element("collapse-picture");
  const apply = (collapsed) => {
    document.querySelector(".viewer-main").classList.toggle("collapsed", collapsed);
    button.setAttribute("aria-expanded", String(!collapsed));
    button.textContent = collapsed ? "▾" : "▴";
    const words = collapsed ? "Show the picture" : "Hide the picture";
    button.setAttribute("aria-label", words);
    button.title = words;
    try {
      localStorage.setItem(COLLAPSE_KEY, collapsed ? "1" : "0");
    } catch {
      // Not kept.
    }
  };
  let collapsed = false;
  try {
    collapsed = localStorage.getItem(COLLAPSE_KEY) === "1";
  } catch {
    // Shown.
  }
  apply(collapsed);
  button.addEventListener("click", () => {
    apply(button.getAttribute("aria-expanded") === "true");
    if (button.getAttribute("aria-expanded") === "true") viewer.resize();
  });
}

// --- start -----------------------------------------------------------------------

async function start() {
  buildSteps();
  // The client id is settled (a duplicated tab takes a new one) while the
  // catalog comes.
  const claimed = claimClientId();
  try {
    store.catalog = await api.catalog();
    await claimed;
  } catch (error) {
    fill(document.querySelector(".panels"), h("div", { class: "alert" }, error.message));
    return;
  }
  element("version").textContent = `library ${store.catalog.library_version}`;
  viewer = new Viewer(element("view"), element("tooltip"), element("view-status"), {
    onPin: pinProbe,
    onProfile: profileDrawn,
    onLock: (quantity, range) => setLockedRange(quantity, range),
    onRange: (range) => setViewOption("range", range),
    onRegion: selectRegion,
  });
  initEditor(viewer, element("canvas-wrap"));
  wirePopoverClosing();
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
  setColorMaps(settings.sequentialMap, settings.divergingMap);
  onSettings((key) => {
    if (key === "sequentialMap" || key === "divergingMap") {
      setColorMaps(settings.sequentialMap, settings.divergingMap);
      viewer.prepareField();
      viewer.prepareIsolines();
      viewer.draw();
      showLegend();
    } else if (key === "pixelRatioCap") viewer.resize();
    else if (key === "isolineCount") updateViewer();
    else if (key === "playbackSpeed") speedChanged();
    // Digits show in the panels.
    else if (key === "digits") {
      buildPanels();
      updateDots();
      viewer.draw();
      showLegend();
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
      for (const example of list)
        examples.append(
          h(
            "button",
            { type: "button", role: "menuitem", dataset: { example: example.name }, title: example.notes ?? "", onclick: () => openExample(example.name) },
            example.name.replace(/\.json$/, "").replaceAll("_", " "),
          ),
        );
    })
    .catch(() => {});
  // The problems the library's presets fill in on the current geometry:
  // in the same menu as the examples, so every ready-made start is in one
  // place (the equation's templates stay at the equation).
  for (const preset of store.catalog.problem?.presets ?? [])
    element("problem-presets").append(
      h(
        "button",
        {
          type: "button",
          role: "menuitem",
          dataset: { preset: preset.id },
          title: preset.description ?? "",
          onclick: async () => {
            if (await applyPreset(preset)) openStep("problem");
          },
        },
        preset.label,
      ),
    );
  wireCaseMenu(element("case-menu-button"), element("case-menu"));
  wireCollapse();
  wireDrop();
  wireHistory();
  wireHeightHandle();
  wireSidebarHandle();
  wirePanes();
  setInterval(tick, 1000);

  subscribe(refresh);
  watchServer(element("server-status"));
  // A reload of this tab: the picture's state it had, and its run if the
  // server still has it.
  const view = keptView();
  const saved = savedCase();
  if (!saved || !loadCase(saved.caseData, { name: saved.name, dirty: saved.dirty, offerRestore: false })) loadCase(structuredClone(store.catalog.default_case), { offerRestore: false });
  else {
    restoreView(view);
    reattachRun().then((found) => {
      if (found) toast("This tab's run is shown again: the server still had it.");
    });
  }
}

start();
