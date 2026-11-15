// Wires the page: the catalog, the steps, the case file actions and the
// picture.

import { api, saveBlob } from "./api.js";
import { runAll, runStage, scheduleProblemCheck, toast } from "./actions.js";
import { fileKind, formatNumber, normalizeCase, stageKey } from "./core.js";
import { h, readFileText, fill } from "./dom.js";
import { assemblePanel } from "./panels/assemble.js";
import { geometryPanel, loadDomainFile } from "./panels/geometry.js";
import { meshPanel } from "./panels/mesh.js";
import { problemPanel } from "./panels/problem.js";
import { solvePanel } from "./panels/solve.js";
import { freshResult, lastResult, savedCase, setCase, stageStatus, store, subscribe } from "./state.js";
import { Viewer } from "./view.js";

const panels = { geometry: geometryPanel, mesh: meshPanel, problem: problemPanel, assemble: assemblePanel, solve: solvePanel };
const STEP_KEY = "pde2d-app-step";
let viewer;

function element(id) {
  return document.getElementById(id);
}

function showStep(step) {
  for (const button of document.querySelectorAll(".steps button")) button.setAttribute("aria-selected", String(button.dataset.step === step));
  for (const [name] of Object.entries(panels)) element(`panel-${name}`).hidden = name !== step;
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
      loadCase(structuredClone(store.catalog.default_case));
    } finally {
      recovering = false;
    }
  }
}

function updateDots() {
  const status = {
    geometry: stageStatus("geometry"),
    mesh: stageStatus("mesh"),
    problem: store.errors.problem ? "failed" : freshResult("problem") ? (freshResult("problem").ok ? "done" : "failed") : stageStatus("problem"),
    assemble: stageStatus("assemble"),
    solve: stageStatus("solve"),
  };
  for (const button of document.querySelectorAll(".steps button")) {
    const dot = button.querySelector(".dot");
    const s = status[button.dataset.step];
    dot.className = `dot ${s === "done" ? "done" : s === "stale" ? "stale" : s === "failed" ? "failed" : ""}`;
    dot.title = { done: "up to date", stale: "outdated: the case changed", failed: "failed", busy: "running", none: "not run" }[s];
  }
}

// The field the picture colours by: the solution snapshot or the last
// field of the Assemble step, whichever came last, while it is up to date.
function shownField() {
  const solved = freshResult("solve");
  if (store.fieldSource === "solve" && solved) {
    const i = Math.min(store.solveFrame, solved.times.length - 1);
    return { label: `u at t = ${formatNumber(solved.times[i], 5)}`, values: solved.values[i], scale: solved.scale };
  }
  const field = store.results.field;
  return field && field.key === stageKey(store.caseData, "assemble") ? field.data : null;
}

function updateViewer() {
  const geometry = freshResult("geometry") ?? lastResult("geometry");
  const meshResult = lastResult("mesh");
  const assembled = lastResult("assemble");
  const mesh = meshResult?.mesh ?? null;
  const meshFresh = freshResult("mesh") !== null;
  viewer.set({
    geometry,
    mesh,
    meshStale: mesh !== null && !meshFresh,
    dirichlet: assembled && freshResult("assemble") ? assembled.meshed_problem.dirichlet_nodes : null,
    field: shownField(),
    colorBy: element("color-by").value,
    layers: { geometry: element("layer-geometry").checked, mesh: element("layer-mesh").checked, dirichlet: element("layer-dirichlet").checked },
  });
  element("legend").replaceChildren(...viewer.legend());
  element("empty-view").hidden = Boolean(geometry || mesh);
}

function refresh(reason) {
  if (reason === "frame") return updateViewer();
  // The problem check is cheap: keep it current whatever changed the case.
  const problem = stageStatus("problem");
  if (problem === "stale" || problem === "none") scheduleProblemCheck();
  if (reason === "case" || reason === "structure") buildPanels();
  else for (const [name, panel] of Object.entries(panels)) panel.update(element(`panel-${name}`));
  updateDots();
  updateViewer();
}

// A case read from a file or an example replaces the current one; every
// result is dropped.
function loadCase(input) {
  let normalized;
  try {
    normalized = normalizeCase(input, store.catalog.default_case);
  } catch (error) {
    toast(error.message);
    return false;
  }
  if (normalized.replaced.length) toast(`Replaced by defaults: ${normalized.replaced.join(", ")}`);
  for (const stage of Object.keys(store.results)) store.results[stage] = null;
  setCase(normalized.caseData);
  runStage("geometry");
  return true;
}

function saveCase() {
  const text = JSON.stringify(store.caseData, null, 2);
  saveBlob(new Blob([text + "\n"], { type: "application/json" }), "case.json");
}

async function openFile(file) {
  const kind = fileKind(file.name);
  if (kind === "case") {
    try {
      loadCase(JSON.parse(await readFileText(file)));
    } catch {
      toast(`${file.name} is not valid JSON.`);
    }
  } else {
    await loadDomainFile(file);
    showStep("geometry");
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
  try {
    store.catalog = await api.catalog();
  } catch (error) {
    fill(document.querySelector(".panels"), h("div", { class: "alert" }, error.message));
    return;
  }
  element("version").textContent = `library ${store.catalog.library_version}`;
  viewer = new Viewer(element("view"), element("tooltip"), element("view-status"));

  for (const button of document.querySelectorAll(".steps button")) button.addEventListener("click", () => showStep(button.dataset.step));
  let step = "geometry";
  try {
    step = localStorage.getItem(STEP_KEY) ?? step;
  } catch {
    // Start on the first step.
  }
  showStep(panels[step] ? step : "geometry");

  for (const id of ["layer-geometry", "layer-mesh", "layer-dirichlet", "color-by"]) element(id).addEventListener("change", updateViewer);
  element("fit-view").addEventListener("click", () => viewer.fit());
  element("new-case").addEventListener("click", () => loadCase(structuredClone(store.catalog.default_case)));
  element("save-case").addEventListener("click", saveCase);
  element("open-case").addEventListener("change", (event) => {
    const file = event.target.files[0];
    if (file) openFile(file);
    event.target.value = "";
  });
  element("run-all").addEventListener("click", async (event) => {
    event.target.disabled = true;
    try {
      if (await runAll()) toast("Geometry, mesh and assembly are up to date.");
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
      loadCase(await api.example(name));
    } catch (error) {
      toast(error.message);
    }
  });
  wireDrop();

  subscribe(refresh);
  const saved = savedCase();
  if (!saved || !loadCase(saved)) loadCase(structuredClone(store.catalog.default_case));
}

start();
