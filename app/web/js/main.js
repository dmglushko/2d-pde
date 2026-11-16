// Wires the page: the catalog, the steps, the case file actions and the
// picture.

import { api, saveBlob } from "./api.js";
import { caseChanged, runAll, runStage, scheduleProblemCheck, toast } from "./actions.js";
import { currentComparison, recompareIfNeeded } from "./compare.js";
import { MAX_PROBES, STEPS, fileKind, formatNumber, freeProbeSlot, getDefaultDigits, normalizeCase, pictureName, stageInfo } from "./core.js";
import { fill, h, readFileText } from "./dom.js";
import { assemblePanel } from "./panels/assemble.js";
import { geometryPanel, loadDomainFile } from "./panels/geometry.js";
import { meshPanel } from "./panels/mesh.js";
import { problemPanel } from "./panels/problem.js";
import { solvePanel } from "./panels/solve.js";
import { dotClass } from "./panels/head.js";
import { forgetChartSize, updateProbes } from "./probes.js";
import { onSettings, settings, wireSettings } from "./settings.js";
import {
  currentError,
  currentKey,
  freshResult,
  lastResult,
  markSaved,
  openStep,
  savedCase,
  setCase,
  setColorBy,
  setProbes,
  setViewOption,
  stageStatus,
  store,
  subscribe,
} from "./state.js";
import { timeSolution, updateTimebar, wireTimebar } from "./timebar.js";
import { Viewer } from "./view.js";

const panels = { geometry: geometryPanel, mesh: meshPanel, problem: problemPanel, assemble: assemblePanel, solve: solvePanel };
for (const step of STEPS) if (!panels[step]) throw new Error(`no panel for the step ${step}`);
const STEP_KEY = "pde2d-app-step";
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

function buildPanels() {
  try {
    for (const [name, panel] of Object.entries(panels)) panel.build(element(`panel-${name}`));
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

// The field the picture colours by: the solution snapshot (or its error
// against the exact solution, compare.js) or the last field of the
// Assemble step, whichever came last, while it is up to date.
function shownField() {
  const solved = freshResult("solve");
  if (store.fieldSource === "solve" && solved) {
    const i = Math.min(store.solveFrame, solved.times.length - 1);
    const t = formatNumber(solved.times[i], 5);
    const comparison = currentComparison();
    if (store.view.showError && comparison?.errors[i])
      return { label: `u − exact at t = ${t}`, short: "u − exact", values: comparison.errors[i], scale: comparison.scale };
    return { label: `u at t = ${t}`, short: "u", values: solved.values[i], scale: solved.scale };
  }
  const field = store.results.field;
  return field && field.key === currentKey("field") ? { ...field.data, short: "value", data: field.data } : null;
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

function updateViewer() {
  const geometry = freshResult("geometry") ?? lastResult("geometry");
  const meshResult = lastResult("mesh");
  const assembled = lastResult("assemble");
  const mesh = meshResult?.mesh ?? null;
  const meshFresh = freshResult("mesh") !== null;
  element("color-by").value = store.view.colorBy;
  const field = stableField();
  element("layer-isolines").checked = store.view.isolines;
  const drawn = viewer.set({
    geometry,
    mesh,
    meshStale: mesh !== null && !meshFresh,
    dirichlet: assembled && freshResult("assemble") ? assembled.meshed_problem.dirichlet_nodes : null,
    field,
    colorBy: store.view.colorBy,
    probes: store.probes,
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
  updateProbes(element("probes"), {
    mesh: meshFresh ? mesh : null,
    field: store.view.colorBy === "field" ? field : null,
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
  const t = store.view.colorBy === "field" && store.fieldSource === "solve" && solved ? solved.times[Math.min(store.solveFrame, solved.times.length - 1)] : null;
  const blob = await viewer.picture();
  if (!blob) return toast("The picture could not be saved.");
  saveBlob(blob, pictureName(store.caseName, t));
}

// The case's file name, whether it changed since, and the notice.
function updateCaseBar() {
  element("unsaved").hidden = !store.dirty;
  element("save-case").title = `Save as ${store.caseName}`;
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

function refresh(reason) {
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
  tick();
  if (reason === "result") recompareIfNeeded();
}

// The case a New, Open or Examples replaced, while the toast offers it
// back.
let previousCase = null;

// A case read from a file or an example replaces the current one; every
// result is dropped. `name` is the file name it is saved under.
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
  for (const stage of Object.keys(store.results)) store.results[stage] = null;
  setCase(normalized.caseData, { name, dirty: dirty || normalized.replaced.length > 0 });
  runStage("geometry");
  if (offerRestore && replaced) {
    previousCase = replaced;
    toast(message ?? `Opened ${name}.`, {
      action: {
        label: "Restore the previous case",
        run: () => {
          const back = previousCase;
          if (back) loadCase(back.caseData, { name: back.name, dirty: back.dirty, message: `Restored ${back.name}.` });
        },
      },
      onEnd: () => {
        if (previousCase === replaced) previousCase = null;
      },
    });
  } else if (message) toast(message);
  return true;
}

function saveCase() {
  const text = JSON.stringify(store.caseData, null, 2);
  saveBlob(new Blob([text + "\n"], { type: "application/json" }), store.caseName);
  markSaved();
}

async function openFile(file) {
  const kind = fileKind(file.name);
  if (kind === "case") {
    let input;
    try {
      input = JSON.parse(await readFileText(file));
    } catch {
      toast(`${file.name} is not valid JSON.`);
      return;
    }
    loadCase(input, { name: file.name });
  } else {
    await loadDomainFile(file);
    openStep("geometry");
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

async function start() {
  buildSteps();
  try {
    store.catalog = await api.catalog();
  } catch (error) {
    fill(document.querySelector(".panels"), h("div", { class: "alert" }, error.message));
    return;
  }
  element("version").textContent = `library ${store.catalog.library_version}`;
  viewer = new Viewer(element("view"), element("tooltip"), element("view-status"), { onPin: pinProbe });
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
  element("save-picture").addEventListener("click", () => savePicture().catch((error) => toast(error.message)));
  element("new-case").addEventListener("click", () => loadCase(structuredClone(store.catalog.default_case), { message: "Started a new case." }));
  element("save-case").addEventListener("click", saveCase);
  element("open-case-button").addEventListener("click", () => element("open-case").click());
  element("open-case").addEventListener("change", (event) => {
    const file = event.target.files[0];
    if (file) openFile(file);
    event.target.value = "";
  });
  element("run-all").addEventListener("click", async (event) => {
    event.target.disabled = true;
    try {
      if (await runAll()) toast("Geometry, mesh, assembly and solve are up to date.");
    } finally {
      event.target.disabled = false;
    }
  });
  const examples = element("examples");
  api
    .examples()
    .then((list) => {
      for (const example of list) examples.append(h("option", { value: example.name, title: example.notes ?? "" }, example.name.replace(/\.json$/, "").replaceAll("_", " ")));
    })
    .catch(() => {});
  examples.addEventListener("change", async () => {
    const name = examples.value;
    examples.value = "";
    if (!name) return;
    try {
      loadCase(await api.example(name), { name });
    } catch (error) {
      toast(error.message);
    }
  });
  wireDrop();
  setInterval(tick, 1000);

  subscribe(refresh);
  const saved = savedCase();
  if (!saved || !loadCase(saved.caseData, { name: saved.name, dirty: saved.dirty, offerRestore: false })) loadCase(structuredClone(store.catalog.default_case), { offerRestore: false });
}

start();
