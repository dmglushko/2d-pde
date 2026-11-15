// Step 5: solving, built from the catalog's solve section: the linear
// solver, and per mode (static, in time) its settings and slots. The
// result is drawn in the picture; a time run has a slider and a player
// over its snapshots.

import { downloadExport } from "../api.js";
import { followedRun, runWithPrerequisites, stopRun, toast } from "../actions.js";
import { LARGE_SOLUTION_BYTES, formatNumber, solutionBytes } from "../core.js";
import { actionButton, busyNote, errorBox, exportButton, fill, foldable, h, markError, optionsForm, reportsView, stat } from "../dom.js";
import { settings } from "../settings.js";
import { currentError, editCase, freshResult, lastResult, notify, openStep, outdatedError, store } from "../state.js";
import { choiceCard } from "./components.js";

// The prerequisites that are not fresh, then the solve; the picture
// colours by the solution once it arrives (actions.js).
async function solve() {
  await runWithPrerequisites("solve");
  // The form is long: bring the result (or the error) into view.
  if (store.step === "solve") document.getElementById("solve-result")?.scrollIntoView({ block: "nearest", behavior: "smooth" });
}

const openProblem = () => openStep("problem");

// How much the values of a run take, from the fresh mesh; a notice when
// it is large.
function sizeEstimate() {
  const nodes = freshResult("mesh")?.stats?.nodes;
  const bytes = solutionBytes(store.caseData.solve, nodes);
  if (bytes === null) return h("p", { class: "note" }, "Mesh first to see how large the solution will be.");
  const megabytes = bytes / (1024 * 1024);
  const text = `The values take about ${formatNumber(megabytes < 0.1 ? Math.round(megabytes * 1000) / 1000 : Math.round(megabytes * 10) / 10)} MB (${formatNumber(nodes)} nodes × ${formatNumber(bytes / 4 / nodes)} kept times × 4 bytes).`;
  if (bytes <= LARGE_SOLUTION_BYTES) return h("p", { class: "note" }, text);
  return h("div", { class: "notice" }, `${text} That is large: the page keeps every snapshot in memory. Fewer snapshots or a coarser mesh make it smaller.`);
}

// --- the player -------------------------------------------------------------

let playTimer = null;

function stopPlaying() {
  clearInterval(playTimer);
  playTimer = null;
}

function showFrame(index) {
  store.solveFrame = index;
  store.fieldSource = "solve";
  store.view.colorBy = "field";
  notify("frame");
}

// The slider and the player over the snapshots. While a run goes on, new
// snapshots extend the slider (refresh) and the newest is shown unless
// the slider was moved.
function player(result, container) {
  const last = () => result.values.length - 1;
  const label = h("span", { class: "num", style: "min-width:9em" });
  const count = h("span", { class: "note" });
  const scaleNote = h("p", { class: "note" });
  const slider = h("input", { type: "range", min: "0", max: String(last()), step: "1", value: String(Math.min(store.solveFrame, last())), "aria-label": "Snapshot", style: "flex:1;min-width:120px" });
  const play = h("button", { type: "button", class: "small", "aria-label": "Play" }, "▶ Play");
  const sync = () => {
    const i = Number(slider.value);
    label.textContent = `t = ${formatNumber(result.times[i], 5)}`;
    play.textContent = playTimer ? "❚❚ Pause" : "▶ Play";
    play.setAttribute("aria-label", playTimer ? "Pause" : "Play");
    count.textContent = result.state === "running" ? `${result.values.length} of ${result.snapshots_total} times` : `${result.times.length} times`;
    scaleNote.textContent = `The colours use one scale for every time: ${formatNumber(result.min)} to ${formatNumber(result.max)}${result.state === "running" ? " so far (it grows while the run goes on)" : ""}.`;
  };
  slider.addEventListener("input", () => {
    stopPlaying();
    store.solveFrameChosen = true;
    showFrame(Number(slider.value));
    sync();
  });
  play.addEventListener("click", () => {
    if (playTimer) {
      stopPlaying();
      return sync();
    }
    // From the start when at the end.
    if (Number(slider.value) >= last()) slider.value = "0";
    store.solveFrameChosen = true;
    showFrame(Number(slider.value));
    playTimer = setInterval(() => {
      // Stop when the panel shows another result, was rebuilt or left,
      // or another field took the picture (the Assemble step's).
      if (!container.isConnected || container.hidden || store.fieldSource !== "solve" || store.results.solve?.data !== result) {
        stopPlaying();
        return sync();
      }
      const next = Number(slider.value) + 1;
      if (next > last()) {
        stopPlaying();
        return sync();
      }
      slider.value = String(next);
      showFrame(next);
      sync();
    }, 1000 / settings.playbackFps);
    sync();
  });
  sync();
  const element = h("div", { class: "card" }, h("div", { class: "card-head" }, h("span", { class: "title" }, "Snapshots"), h("span", { class: "spacer" }), count), h("div", { class: "row" }, play, slider, label), scaleNote);
  // More snapshots arrived.
  element.refresh = () => {
    slider.max = String(last());
    if (!playTimer) slider.value = String(Math.min(store.solveFrame, last()));
    sync();
  };
  return element;
}

// How far a time run got, with Stop.
function progressView(result) {
  const bar = h("progress", { max: "1", value: "0", style: "flex:1;min-width:120px" });
  const text = h("span", { class: "num" });
  const element = h(
    "div",
    { class: "card run-progress", role: "status" },
    h("div", { class: "row" }, bar, actionButton("Stop", stopRun, { small: true })),
    text,
  );
  element.refresh = () => {
    const span = result.t1 - result.t0;
    const done = span > 0 ? (result.t - result.t0) / span : 0;
    bar.value = Math.max(0, Math.min(1, done));
    const steps = result.reports?.[0]?.metrics?.steps ?? 0;
    text.textContent = `Running: t = ${formatNumber(result.t, 5)} of ${formatNumber(result.t1)} · ${formatNumber(steps)} steps · ${result.values.length} of ${result.snapshots_total} snapshots`;
  };
  element.refresh();
  return element;
}

function resultStats(result) {
  const report = (stage) => result.reports.find((r) => r.stage === stage)?.metrics ?? {};
  const integrator = report("integrator");
  const staticReport = report("static_solver");
  return [
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
  ];
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
    const modeSettings = (solveCase[mode.settings.key] ??= {});
    const pickLinear = (next) => editCase((c) => (c.solve.linear_solver = next), "structure");
    fill(
      container,
      h("h2", {}, "Solve"),
      h("p", { class: "lead" }, "Solves the assembled system on the mesh: once (static), or step by step from t0 to t1."),
      modeSwitch(section, solveCase),
      h("p", { class: "note" }, mode.description),
      h("div", { id: "solve-hint" }),
      foldable(
        { id: `solve.${mode.settings.key}`, title: mode.settings.label, attributes: { dataset: { path: `solve.${mode.settings.key}` } } },
        optionsForm(mode.settings.options, modeSettings, `solve.${mode.settings.key}`, () => editCase(() => {}, "edit", { sections: ["solve"] })),
      ),
      mode.slots.map((slot) => choiceCard(slot, modeSettings[slot.id], `solve.${mode.settings.key}.${slot.id}`, (next) => editCase((c) => (c.solve[mode.settings.key][slot.id] = next), "structure"))),
      // Its options are rarely changed: folded for a new viewer.
      choiceCard(section.linear_solver, solveCase.linear_solver, "solve.linear_solver", pickLinear, { folded: true }),
      h("div", { id: "solve-size" }),
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
    const size = container.querySelector("#solve-size");
    const sizeKey = JSON.stringify([freshResult("mesh")?.stats?.nodes ?? null, store.caseData.solve?.mode, store.caseData.solve?.time?.snapshots]);
    if (size.shown !== sizeKey) {
      size.shown = sizeKey;
      fill(size, sizeEstimate());
    }
    const error = currentError("solve");
    const old = outdatedError("solve");
    const fresh = freshResult("solve");
    const result = fresh ?? lastResult("solve");
    const busy = store.busy.has("solve");
    const running = Boolean(followedRun()) && result?.state === "running";
    const state = result?.state;
    if (slot.shown && slot.shown.result === result && slot.shown.error === error && slot.shown.old === old && slot.shown.busy === busy && slot.shown.fresh === fresh && slot.shown.running === running && slot.shown.state === state) {
      // The same run, further on: update what shows progress in place
      // (rebuilding would take the slider from under the pointer).
      for (const live of slot.querySelectorAll("[data-live]")) live.refresh?.();
      return;
    }
    slot.shown = { result, error, old, busy, fresh, running, state };
    stopPlaying();
    const placed = markError(container, error);
    const children = [];
    if (error && !placed) children.push(errorBox(error, { openProblem }));
    if (old) children.push(errorBox(old, { outdated: true }));
    if (busy && !running) children.push(busyNote("solve", "Solving"));
    if (result) {
      const live = (element) => ((element.dataset.live = ""), element);
      const stats = live(h("div", { class: "stats" }, resultStats(result)));
      stats.refresh = () => fill(stats, resultStats(result));
      const stopped = result.state === "stopped";
      const title = result.mode === "time" ? `Solution from t = ${formatNumber(result.t0 ?? result.times[0])} to ${formatNumber(result.t1 ?? result.times.at(-1))}` : `Solution at t = ${formatNumber(result.times[0])}`;
      children.push(
        running ? live(progressView(result)) : null,
        stopped && fresh ? h("div", { class: "row" }, actionButton("Continue", solve, { small: true }), h("span", { class: "note" }, "Goes on from where it stopped.")) : null,
        foldable(
          {
            id: "solve.result",
            title,
            kind: "section",
            extras: [
              fresh ? null : h("span", { class: "chip" }, "outdated: the case changed"),
              stopped ? h("span", { class: "chip" }, `stopped at t = ${formatNumber(result.t, 5)}`) : null,
              result.state === "failed" ? h("span", { class: "chip bad" }, `failed after t = ${formatNumber(result.t, 5)}`) : null,
            ],
          },
          stats,
          fresh && result.mode === "time" && result.values.length > 0 ? live(player(result, container)) : null,
          fresh ? null : h("p", { class: "note" }, "The picture shows the solution only while it is up to date: solve again."),
        ),
        running ? null : foldable({ id: "solve.reports", title: "Reports", kind: "section", folded: true }, reportsView(result.reports)),
        running
          ? null
          : h(
              "div",
              { class: "row", style: "margin-top:12px" },
              exportButton("Export solution .csv", () => downloadExport("solution.csv", store.caseData).catch((e) => toast(e.message)), {
                fresh: fresh && result.state === "done",
                staleReason: result.state === "done" ? "The solution shown is outdated: solve again to export it." : "The run did not reach t1: continue it to export.",
              }),
            ),
        running ? null : h("p", { class: "note" }, "One row per mesh node: x, y and u at every kept time."),
      );
    }
    fill(slot, ...children);
  },
};
