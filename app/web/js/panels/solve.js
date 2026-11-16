// Step 5: solving, built from the catalog's solve section: the linear
// solver, and per mode (static, in time) its settings and slots. The head
// holds Solve (Continue after a stop, Stop during a run) and the status
// with the run's progress; the result and the comparison with an exact
// solution follow; the settings come last. The shown snapshot is the time
// bar's under the picture (timebar.js).

import { downloadExport } from "../api.js";
import { followedRun, runWithPrerequisites, stopRun, toast } from "../actions.js";
import { lineChart } from "../charts.js";
import { clearComparison, compareWith, currentComparison, comparison } from "../compare.js";
import { LARGE_SOLUTION_BYTES, formatNumber, solutionBytes } from "../core.js";
import { actionButton, errorBox, exportButton, fill, foldable, h, infoToggle, markError, optionsForm, reportsView, stat } from "../dom.js";
import { currentError, editCase, freshResult, lastResult, openStep, outdatedError, setViewOption, store } from "../state.js";
import { showFrame } from "../timebar.js";
import { timeOrderText } from "./assemble.js";
import { choiceCard } from "./components.js";
import { setStepStatus, stepHead } from "./head.js";

const solve = () => runWithPrerequisites("solve");
const openProblem = () => openStep("problem");

// A warning when the values of a run would be large (from the fresh
// mesh); nothing otherwise.
function sizeWarning() {
  const nodes = freshResult("mesh")?.stats?.nodes;
  const bytes = solutionBytes(store.caseData.solve, nodes);
  if (bytes === null || bytes <= LARGE_SOLUTION_BYTES) return null;
  const megabytes = Math.round(bytes / (1024 * 1024));
  return h(
    "div",
    { class: "notice" },
    `The solution will take about ${formatNumber(megabytes)} MB in this page (${formatNumber(nodes)} nodes × ${formatNumber(bytes / 4 / nodes)} times). Fewer snapshots or a coarser mesh make it smaller.`,
  );
}

function resultStats(result) {
  const report = (stage) => result.reports.find((r) => r.stage === stage)?.metrics ?? {};
  const integrator = report("integrator");
  const staticReport = report("static_solver");
  const dt = formatNumber(integrator.min_dt) === formatNumber(integrator.max_dt) ? formatNumber(integrator.max_dt) : `${formatNumber(integrator.min_dt)} – ${formatNumber(integrator.max_dt)}`;
  return [
    stat("Unknowns", formatNumber(result.free_unknowns)),
    stat("Time order", timeOrderText(result.time_order, result.problem_time_order)),
    stat("min u", formatNumber(result.min), "Over every snapshot"),
    stat("max u", formatNumber(result.max), "Over every snapshot"),
    result.mode === "time"
      ? [
          stat("Snapshots", formatNumber(result.values.length)),
          stat("Steps", formatNumber(integrator.steps)),
          stat("dt", dt),
          stat("Linear iterations", formatNumber(integrator.linear_iterations)),
          stat("Largest residual", formatNumber(integrator.max_residual, 2)),
        ]
      : [stat("Linear iterations", formatNumber(staticReport.linear_iterations)), stat("Residual", formatNumber(staticReport.residual, 2))],
  ];
}

// --- the head ---------------------------------------------------------------

function setLabel(button, text) {
  for (const node of button.childNodes) if (node.nodeType === Node.TEXT_NODE) node.textContent = text;
}

// The status of the step: running (t reached, steps, snapshots, with a
// progress bar), stopped at t, failed after t, or the usual words.
function headStatus(result, fresh, running) {
  if (running) {
    const span = result.t1 - result.t0;
    const bar = h("progress", { max: "1", value: String(span > 0 ? Math.max(0, Math.min(1, (result.t - result.t0) / span)) : 0), "aria-label": "Run progress" });
    const steps = result.reports?.[0]?.metrics?.steps ?? 0;
    return {
      status: "busy",
      text: `Running: t = ${formatNumber(result.t, 5)} of ${formatNumber(result.t1)}`,
      detail: `${formatNumber(steps)} steps · ${result.values.length} of ${result.snapshots_total} snapshots`,
      extra: bar,
    };
  }
  if (fresh && result.state === "stopped") return { status: "stopped", text: `Stopped at t = ${formatNumber(result.t, 5)}` };
  if (fresh && result.state === "failed") return { status: "failed", text: `Failed after t = ${formatNumber(result.t, 5)}` };
  if (fresh) return { detail: result.mode === "time" ? `${result.values.length} snapshots` : "static" };
  return {};
}

// --- comparing with an exact solution -------------------------------------------

function compareSection(result) {
  const text = h("input", {
    type: "text",
    class: "expr",
    spellcheck: false,
    autocomplete: "off",
    value: store.caseData.view?.exact_solution ?? "",
    placeholder: "for example exp(-t)*sin(pi*x)",
    "aria-label": "Exact solution",
    oninput: (event) =>
      editCase(
        (c) => {
          const value = event.target.value;
          if (value.trim() === "") {
            if (c.view) delete c.view.exact_solution;
          } else (c.view ??= {}).exact_solution = value;
        },
        "edit",
        { sections: ["view"] },
      ),
    onkeydown: (event) => {
      if (event.key === "Enter") run();
    },
  });
  const run = () => {
    const value = text.value.trim();
    if (value === "") return clearComparison();
    return compareWith(value);
  };
  const info = infoToggle(
    "The library evaluates the expression (in x, y, t and the parameters) at every snapshot; the page subtracts it from u. u travels to the page as 32-bit floats, so errors below about 1e-7 × max |u| are rounding.",
    "the comparison",
  );
  const showError = h("input", { type: "checkbox", id: "show-error", onchange: (event) => setViewOption("showError", event.target.checked) });
  return foldable(
    { id: "solve.compare", title: "Compare with an exact solution", kind: "section", extras: [info.button], info: info.panel },
    h("div", { class: "field", dataset: { path: "field.text" } }, h("div", { class: "row compare-line" }, text, actionButton("Compare", run, { small: true }))),
    h("div", { class: "compare-result", id: "compare-result" }),
    h("label", { class: "row inline-label" }, showError, "Show u − exact in the picture"),
    result.mode === "time" ? h("div", { class: "compare-chart", id: "compare-chart" }) : null,
  );
}

// A figure that may not be computed yet ("…" until it is).
const shownNumber = (value) => (Number.isFinite(value) ? formatNumber(value, 3) : "…");

// The comparison's figures and chart for the shown snapshot.
function updateCompare(container) {
  const section = container.querySelector("#compare-result");
  if (!section) return;
  const result = freshResult("solve");
  const current = currentComparison();
  const frame = result ? Math.min(store.solveFrame, result.values.length - 1) : 0;
  const box = container.querySelector("#show-error");
  box.disabled = !current || current.done === 0;
  box.checked = store.view.showError && !box.disabled;
  // The library's message about the expression lands on the input.
  const form = section.closest(".fold-section");
  const error = comparison.result === result ? comparison.error : null;
  if (form.shownError !== error) {
    form.shownError = error;
    if (!markError(form, error)) fill(section, error ? errorBox(error) : null);
  }
  if (error) return;
  if (!current) return fill(section);
  const linf = current.linf;
  let overall = NaN;
  for (const v of linf) if (Number.isFinite(v) && !(v <= overall)) overall = v;
  fill(
    section,
    h(
      "div",
      { class: "stats" },
      stat("max |u − exact|", shownNumber(linf[frame]), `At t = ${formatNumber(result.times[frame], 5)}`),
      result.mode === "time" ? stat("Largest over the snapshots", shownNumber(overall)) : null,
      current.busy ? stat("Compared", `${current.done} of ${linf.length}`) : null,
    ),
  );
  const chartSlot = container.querySelector("#compare-chart");
  if (!chartSlot) return;
  if (chartSlot.chart?.comparison !== current || chartSlot.chart.done !== current.done) {
    const chart = lineChart({
      times: result.times.slice(0, linf.length),
      series: [{ name: "max |u − exact|", cls: "s1", values: Array.from(linf) }],
      current: frame,
      width: Math.max(200, chartSlot.clientWidth || 340),
      onPick: showFrame,
      label: "Largest error per snapshot",
    });
    chartSlot.chart = { comparison: current, done: current.done, chart };
    fill(chartSlot, h("div", { class: "chart-title" }, "max |u − exact| per snapshot"), chart);
  } else chartSlot.chart.chart.setCurrent(frame);
}

// --- the settings ---------------------------------------------------------------

function modeSwitch(section, solveCase) {
  return h(
    "div",
    { class: "row segmented", role: "radiogroup", "aria-label": "Mode" },
    section.modes.map((mode) =>
      h(
        "label",
        { class: "choice", title: mode.description },
        h("input", { type: "radio", name: "solve-mode", checked: solveCase.mode === mode.id, onchange: () => editCase((c) => (c.solve.mode = mode.id), "structure") }),
        mode.label,
      ),
    ),
  );
}

// A hint when the mode does not fit the problem's time order (from the
// problem check).
function orderHint(solveCase) {
  const check = freshResult("problem");
  if (!check?.ok || typeof check.time_order !== "number") return null;
  const order = check.time_order;
  if (solveCase.mode === "static" && order > 0)
    return h(
      "div",
      { class: "notice" },
      `The problem has time order ${order}: a static solve drops u_t and u_tt. `,
      h("button", { type: "button", class: "small", onclick: () => editCase((c) => (c.solve.mode = "time"), "structure") }, "Solve in time"),
    );
  if (solveCase.mode === "time" && order === 0) return h("div", { class: "notice" }, "The problem has time order 0: each step is a static solve, which only some methods support.");
  return null;
}

function settingsPart(section, solveCase, mode) {
  const key = mode.settings.key;
  const modeSettings = (solveCase[key] ??= {});
  const pickLinear = (next) => editCase((c) => (c.solve.linear_solver = next), "structure");
  const modeInfo = infoToggle(mode.description, mode.label);
  return h(
    "div",
    { class: "settings-part" },
    h("h3", { class: "part-title" }, "Settings"),
    h("div", { class: "row" }, modeSwitch(section, solveCase), modeInfo.button),
    modeInfo.panel,
    h("div", { id: "solve-hint" }),
    h("div", { id: "solve-size" }),
    foldable(
      { id: `solve.${key}`, title: mode.settings.label, attributes: { dataset: { path: `solve.${key}` } } },
      optionsForm(mode.settings.options, modeSettings, `solve.${key}`, () => editCase(() => {}, "edit", { sections: ["solve"] })),
    ),
    mode.slots.map((slot) => choiceCard(slot, modeSettings[slot.id], `solve.${key}.${slot.id}`, (next) => editCase((c) => (c.solve[key][slot.id] = next), "structure"))),
    // Its options are rarely changed: folded for a new viewer.
    choiceCard(section.linear_solver, solveCase.linear_solver, "solve.linear_solver", pickLinear, { folded: true }),
  );
}

export const solvePanel = {
  build(container) {
    const section = store.catalog.sections.find((s) => s.id === "solve");
    if (!section?.available) {
      fill(container, stepHead("solve"), h("div", { class: "notice" }, section?.message ?? "Not available."));
      return;
    }
    const solveCase = store.caseData.solve;
    const mode = section.modes.find((m) => m.id === solveCase.mode) ?? section.modes[0];
    const solveButton = actionButton("Solve", solve, { primary: true });
    const stopButton = actionButton("Stop", stopRun);
    const head = stepHead("solve", solveButton, stopButton);
    fill(container, head, h("div", { class: "result", id: "solve-result" }), settingsPart(section, solveCase, mode));
    Object.assign(container, { head, solveButton, stopButton });
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
      fill(size, sizeWarning());
    }
    const error = currentError("solve");
    const old = outdatedError("solve");
    const fresh = freshResult("solve");
    const result = fresh ?? lastResult("solve");
    const busy = store.busy.has("solve");
    const running = Boolean(followedRun()) && result?.state === "running";
    const state = result?.state;
    container.stopButton.hidden = !running;
    container.solveButton.hidden = running;
    setLabel(container.solveButton, fresh && state === "stopped" ? "Continue" : "Solve");
    container.solveButton.title = fresh && state === "stopped" ? "Goes on from where the run stopped" : "";
    setStepStatus(container.head, "solve", result ? headStatus(result, fresh, running) : {});
    const shown = slot.shown;
    const same =
      shown && shown.result === result && shown.error === error && shown.old === old && shown.busy === busy && shown.fresh === fresh && shown.running === running && shown.state === state;
    if (same) {
      // The same run, further on: its figures in place.
      const stats = slot.querySelector("#solve-stats");
      if (stats && running) fill(stats, resultStats(result));
      updateCompare(container);
      return;
    }
    slot.shown = { result, error, old, busy, fresh, running, state };
    const placed = markError(container, error);
    const save = () => downloadExport("solution.csv", store.caseData).catch((e) => toast(e.message));
    fill(
      slot,
      error && !placed ? errorBox(error, { openProblem }) : null,
      old ? errorBox(old, { outdated: true }) : null,
      result ? h("div", { class: "stats", id: "solve-stats" }, resultStats(result)) : null,
      result && !running
        ? h(
            "div",
            { class: "row actions-row" },
            exportButton("Export solution .csv", save, {
              fresh: fresh && result.state === "done",
              staleReason: result.state === "done" ? "The solution shown is outdated: solve again to export it." : "The run did not reach t1: continue it to export.",
              about: "One row per mesh node: x, y and u at every kept time, in full precision.",
            }),
          )
        : null,
      fresh && !running && result.values.length > 0 ? compareSection(result) : null,
      result && !running ? foldable({ id: "solve.reports", title: "Reports", kind: "section", folded: true }, reportsView(result.reports)) : null,
    );
    slot.classList.toggle("outdated-result", Boolean(result) && !fresh);
    updateCompare(container);
  },

  // Another snapshot is shown: the comparison's figures follow it.
  frame(container) {
    updateCompare(container);
  },
};
