// Step 5: solving, built from the catalog's solve section: the linear
// solver, and per mode (static, in time) its settings and slots. The
// result is drawn in the picture; a time run has a slider and a player
// over its snapshots.

import { downloadExport } from "../api.js";
import { runStage, toast } from "../actions.js";
import { formatNumber } from "../core.js";
import { actionButton, errorBox, fill, h, markError, optionsForm, reportsView, stat } from "../dom.js";
import { editCase, freshResult, lastResult, notify, stageStatus, store } from "../state.js";
import { choiceCard } from "./components.js";

// Geometry and mesh first when they are not up to date, then the solve.
async function solve() {
  for (const stage of ["geometry", "mesh"]) {
    if (stageStatus(stage) === "done") continue;
    if (!(await runStage(stage))) return toast(`The ${stage} step failed; see that step.`);
  }
  if (await runStage("solve")) colourByField();
  // The form is long: bring the result (or the error) into view.
  document.getElementById("solve-result")?.scrollIntoView({ block: "nearest", behavior: "smooth" });
}

// The picture colours by the field, which now is the solution.
function colourByField() {
  const colorBy = document.getElementById("color-by");
  if (colorBy.value === "field") return;
  colorBy.value = "field";
  colorBy.dispatchEvent(new Event("change"));
}

// --- the player -------------------------------------------------------------

const FRAMES_PER_SECOND = 12;
let playTimer = null;

function stopPlaying() {
  clearInterval(playTimer);
  playTimer = null;
}

function showFrame(index) {
  store.solveFrame = index;
  store.fieldSource = "solve";
  colourByField();
  notify("frame");
}

function player(result, container) {
  const last = result.times.length - 1;
  const label = h("span", { class: "num", style: "min-width:9em" });
  const slider = h("input", { type: "range", min: "0", max: String(last), step: "1", value: String(Math.min(store.solveFrame, last)), "aria-label": "Snapshot", style: "flex:1;min-width:120px" });
  const play = h("button", { type: "button", class: "small", "aria-label": "Play" }, "▶ Play");
  const sync = () => {
    const i = Number(slider.value);
    label.textContent = `t = ${formatNumber(result.times[i], 5)}`;
    play.textContent = playTimer ? "❚❚ Pause" : "▶ Play";
    play.setAttribute("aria-label", playTimer ? "Pause" : "Play");
  };
  slider.addEventListener("input", () => {
    stopPlaying();
    showFrame(Number(slider.value));
    sync();
  });
  play.addEventListener("click", () => {
    if (playTimer) {
      stopPlaying();
      return sync();
    }
    // From the start when at the end.
    if (Number(slider.value) >= last) slider.value = "0";
    showFrame(Number(slider.value));
    playTimer = setInterval(() => {
      // Stop when the panel shows another result, was rebuilt or left,
      // or another field took the picture (the Assemble step's).
      if (!container.isConnected || container.hidden || store.fieldSource !== "solve" || store.results.solve?.data !== result) {
        stopPlaying();
        return sync();
      }
      const next = Number(slider.value) + 1;
      if (next > last) {
        stopPlaying();
        return sync();
      }
      slider.value = String(next);
      showFrame(next);
      sync();
    }, 1000 / FRAMES_PER_SECOND);
    sync();
  });
  sync();
  return h(
    "div",
    { class: "card" },
    h("div", { class: "card-head" }, h("span", { class: "title" }, "Snapshots"), h("span", { class: "spacer" }), h("span", { class: "note" }, `${result.times.length} times`)),
    h("div", { class: "row" }, play, slider, label),
    h("p", { class: "note" }, `The colours use one scale for every time: ${formatNumber(result.min)} to ${formatNumber(result.max)}.`),
  );
}

// --- the form ---------------------------------------------------------------

function modeSwitch(section, solveCase) {
  return h(
    "div",
    { class: "row", role: "radiogroup", "aria-label": "Mode" },
    section.modes.map((mode) =>
      h(
        "label",
        { class: "chip", style: "cursor:pointer;padding:4px 10px", title: mode.description },
        h("input", {
          type: "radio",
          name: "solve-mode",
          checked: solveCase.mode === mode.id,
          onchange: () => editCase((c) => (c.solve.mode = mode.id), "structure"),
        }),
        mode.label,
      ),
    ),
  );
}

// A note on how the mode fits the problem's time order (from the problem
// check), when they do not obviously match.
function orderHint(solveCase) {
  const check = freshResult("problem");
  if (!check?.ok || typeof check.time_order !== "number") return null;
  const order = check.time_order;
  if (solveCase.mode === "static" && order > 0)
    return h(
      "div",
      { class: "notice" },
      `The problem has time order ${order}: a static solve drops its time terms (u_t, u_tt). `,
      h("button", { type: "button", class: "small", onclick: () => editCase((c) => (c.solve.mode = "time"), "structure") }, "Solve in time"),
    );
  if (solveCase.mode === "time" && order === 0) return h("p", { class: "note" }, "The problem has time order 0: each step is a static solve at its time, which only some methods support.");
  return null;
}

export const solvePanel = {
  build(container) {
    stopPlaying();
    const section = store.catalog.sections.find((s) => s.id === "solve");
    if (!section?.available) {
      fill(container, h("h2", {}, "Solve"), h("div", { class: "notice" }, section?.message ?? "Not available."));
      return;
    }
    const solveCase = store.caseData.solve;
    const mode = section.modes.find((m) => m.id === solveCase.mode) ?? section.modes[0];
    const settings = (solveCase[mode.settings.key] ??= {});
    const pickLinear = (next) => editCase((c) => (c.solve.linear_solver = next), "structure");
    fill(
      container,
      h("h2", {}, "Solve"),
      h("p", { class: "lead" }, "Solves the assembled system on the mesh: once (static), or step by step from t0 to t1."),
      modeSwitch(section, solveCase),
      h("p", { class: "note" }, mode.description),
      h("div", { id: "solve-hint" }),
      h(
        "div",
        { class: "card", dataset: { path: `solve.${mode.settings.key}` } },
        h("div", { class: "card-head" }, h("span", { class: "title" }, mode.settings.label)),
        optionsForm(mode.settings.options, settings, `solve.${mode.settings.key}`, () => editCase(() => {})),
      ),
      mode.slots.map((slot) => choiceCard(slot, settings[slot.id], `solve.${mode.settings.key}.${slot.id}`, (next) => editCase((c) => (c.solve[mode.settings.key][slot.id] = next), "structure"))),
      choiceCard(section.linear_solver, solveCase.linear_solver, "solve.linear_solver", pickLinear),
      h("div", { class: "row", style: "margin-top:12px" }, actionButton("Solve", solve, { primary: true })),
      h("div", { class: "result", id: "solve-result" }),
    );
    this.update(container);
  },

  update(container) {
    const slot = container.querySelector("#solve-result");
    if (!slot) return;
    const hint = container.querySelector("#solve-hint");
    const check = freshResult("problem");
    if (hint.shown !== check) {
      hint.shown = check;
      fill(hint, orderHint(store.caseData.solve));
    }
    const error = store.errors.solve;
    const fresh = freshResult("solve");
    const result = fresh ?? lastResult("solve");
    const busy = store.busy.has("solve");
    if (slot.shown && slot.shown.result === result && slot.shown.error === error && slot.shown.busy === busy && slot.shown.fresh === fresh) return;
    slot.shown = { result, error, busy, fresh };
    stopPlaying();
    const placed = markError(container, error);
    const children = [];
    if (error && !placed) children.push(errorBox(error));
    if (error && ["problem", "meshed_problem"].includes(error.stage)) children.push(h("p", { class: "note" }, "Fix it in the Problem step."));
    if (busy) children.push(h("p", { class: "note" }, "Solving… (a long time run keeps the app busy until it ends)"));
    if (result) {
      const report = (stage) => result.reports.find((r) => r.stage === stage)?.metrics ?? {};
      const integrator = report("integrator");
      const staticReport = report("static_solver");
      children.push(
        h("div", { class: "row" }, h("h3", { style: "margin:8px 0" }, result.mode === "time" ? `Solution from t = ${formatNumber(result.times[0])} to ${formatNumber(result.times.at(-1))}` : `Solution at t = ${formatNumber(result.times[0])}`), fresh ? null : h("span", { class: "chip" }, "outdated: the case changed")),
        h(
          "div",
          { class: "stats" },
          stat("Unknowns", formatNumber(result.free_unknowns)),
          stat("Time order", result.time_order === result.problem_time_order ? String(result.time_order) : `${result.time_order} (as written ${result.problem_time_order})`),
          stat("u from", formatNumber(result.min)),
          stat("u to", formatNumber(result.max)),
          result.mode === "time"
            ? [
                stat("Steps", formatNumber(integrator.steps)),
                stat("dt", formatNumber(integrator.min_dt) === formatNumber(integrator.max_dt) ? formatNumber(integrator.max_dt) : `${formatNumber(integrator.min_dt)} – ${formatNumber(integrator.max_dt)}`),
                stat("Linear iterations", formatNumber(integrator.linear_iterations)),
                stat("Largest residual", formatNumber(integrator.max_residual, 2)),
              ]
            : [stat("Linear iterations", formatNumber(staticReport.linear_iterations)), stat("Residual", formatNumber(staticReport.residual, 2))],
        ),
        fresh && result.mode === "time" ? player(result, container) : null,
        fresh ? null : h("p", { class: "note" }, "The picture shows the solution only while it is up to date: solve again."),
        h("h3", {}, "Reports"),
        reportsView(result.reports),
        h("div", { class: "row", style: "margin-top:12px" }, actionButton("Export solution .csv", () => downloadExport("solution.csv", store.caseData).catch((e) => toast(e.message)), { small: true })),
        h("p", { class: "note" }, "One row per mesh node: x, y and u at every kept time."),
      );
    }
    fill(slot, ...children);
  },
};
