// Step 5: solving, built from the catalog's solve section: the linear
// solver, and per mode (static, in time) its settings and slots. The head
// holds Solve (Continue after a stop, Stop during a run) and the status
// with the run's progress; the result, its exports, the comparison with
// an exact solution and with the reference run follow; the settings come
// last. The shown snapshot is the time bar's under the picture
// (timebar.js).

import { exportFile, followedRun, runWithPrerequisites, stopRun, toast } from "../actions.js";
import { lineChart } from "../charts.js";
import { LABELS, clearComparison, compare, compareWith, comparisons, currentComparison, forgetReference, isOutdated, keepReference } from "../compare.js";
import { formatNumber } from "../pure/format.js";
import { LARGE_SOLUTION_BYTES, solutionBytes } from "../pure/stages.js";
import { actionButton, errorBox, exportButton, fill, foldable, h, infoToggle, markError, optionsForm, reportsView, stat, warningChip } from "../dom.js";
import { currentError, editCase, freshResult, lastResult, notify, openStep, outdatedError, setViewOption, store } from "../state.js";
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

// The figures of a run. Linear iterations are left out when there were
// none: a direct solver (LU, LDLᵀ) always reports 0, and the library
// declares no trait that says which solvers are iterative.
function resultStats(result) {
  const report = (stage) => result.reports.find((r) => r.stage === stage)?.metrics ?? {};
  const integrator = report("integrator");
  const staticReport = report("static_solver");
  const dt = formatNumber(integrator.min_dt) === formatNumber(integrator.max_dt) ? formatNumber(integrator.max_dt) : `${formatNumber(integrator.min_dt)} – ${formatNumber(integrator.max_dt)}`;
  const iterations = (count) => (count > 0 ? stat("Linear iterations", formatNumber(count)) : null);
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
          iterations(integrator.linear_iterations),
          stat("Largest residual", formatNumber(integrator.max_residual, 2), "The largest ‖b − A U‖ of the linear solves: how well each system was solved, not the error of u"),
        ]
      : [iterations(staticReport.linear_iterations), stat("Residual", formatNumber(staticReport.residual, 2), "‖b − A U‖ of the linear solve: how well the system was solved, not the error of u")],
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

// --- exports -------------------------------------------------------------------

const EXPORTS = [
  ["solution.csv", "Export .csv", "One row per mesh node: x, y and u at every kept time, in full precision."],
  ["solution.vtk", "Export .vtk", "The mesh and u at every kept time, for ParaView or VisIt (VTK legacy)."],
  ["provenance.json", "Export provenance", "The whole case, the app and library versions and this run: what made the results."],
];

function exportsRow(result, fresh) {
  const done = fresh && result.state === "done";
  const stale = result.state === "done" ? "The solution shown is outdated: solve again to export it." : "The run did not reach t1: continue it to export.";
  return h(
    "div",
    { class: "row actions-row" },
    EXPORTS.map(([kind, label, about]) => exportButton(label, () => exportFile(kind), { fresh: done, staleReason: stale, about })),
  );
}

// --- comparisons -------------------------------------------------------------------

// A figure that may not be computed yet ("…" until it is).
const shownNumber = (value) => (Number.isFinite(value) ? formatNumber(value, 3) : "…");
const largest = (values) => {
  let best = NaN;
  for (const v of values) if (Number.isFinite(v) && !(v <= best)) best = v;
  return best;
};

// What the figures are, said next to them.
const NOTES = {
  exact: "Nodal error: u − exact at the mesh nodes, in double. Not the solver's residual ‖b − A U‖ (above), which only says how well each linear system was solved.",
  reference: "u − reference at the mesh nodes, in double, at the same times.",
};
const FLOAT_NOTE = "The picture of a difference is drawn in float32: below about 1e−7 × |u| it shows rounding, not the solution.";

// The checkbox that shows u − other in the picture.
function differenceBox(kind) {
  const id = `show-${kind}`;
  const box = h("input", { type: "checkbox", id, onchange: (event) => setViewOption("difference", event.target.checked ? kind : null) });
  return h("label", { class: "row inline-label", for: id }, box, `Show ${LABELS[kind]} in the picture`);
}

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
    if (value === "") return clearComparison("exact");
    return compareWith(value);
  };
  const info = infoToggle(
    "The server evaluates the expression (in x, y, t and the parameters) with the library at every snapshot and compares it with u in double precision: the largest nodal error, its root mean square over the nodes, and the lumped L2 norm sqrt(Σ w_i d_i²) with w_i a third of the area of the triangles around node i.",
    "the comparison",
  );
  return foldable(
    { id: "solve.compare", title: "Compare with an exact solution", kind: "section", extras: [info.button], info: info.panel, attributes: { class: "comparison", dataset: { kind: "exact" } } },
    h("div", { class: "field", dataset: { path: "exact" } }, h("div", { class: "row compare-line" }, text, actionButton("Compare", run, { small: true }))),
    h("div", { class: "compare-result" }),
    differenceBox("exact"),
    result.mode === "time" ? h("div", { class: "compare-chart" }) : null,
  );
}

function referenceSection(result, fresh) {
  const reference = store.reference;
  const done = fresh && result.state === "done";
  const keep = async () => {
    const error = await keepReference();
    if (error) toast(`This run cannot be the reference: ${error.message}`);
    notify("compare");
  };
  const forget = async () => {
    const error = await forgetReference();
    if (error) toast(error.message);
    notify("compare");
  };
  const keepButton = actionButton(reference ? "Keep this run instead" : "Keep as reference", keep, { small: true });
  keepButton.disabled = !done || reference?.job === result.job;
  if (!done) keepButton.title = "Only a finished, up-to-date run can be the reference.";
  else if (reference?.job === result.job) keepButton.title = "This run is the reference.";
  const describe = (r) => `run ${r.job}, ${r.mode === "time" ? `in time, ${r.times.length} snapshots` : "static"}`;
  const info = infoToggle(
    "Keep a finished run, change the case (dt, the method, a coefficient) and solve again: the server compares the new run with it node by node, at the same times, in double. Both runs must be on the same mesh object: changing the geometry or the mesh makes another mesh, and the comparison is refused.",
    "the reference run",
  );
  return foldable(
    { id: "solve.reference", title: "Compare with a reference run", kind: "section", folded: true, extras: [info.button], info: info.panel, attributes: { class: "comparison", dataset: { kind: "reference" } } },
    reference
      ? h(
          "p",
          { class: "note" },
          reference.job === result.job ? `This run is the reference (${describe(reference)}).` : `The reference is ${describe(reference)}. Compare this run with it.`,
        )
      : h("p", { class: "note" }, "No reference yet: keep a finished run, change the case and solve again."),
    h(
      "div",
      { class: "row" },
      reference && reference.job !== result.job ? actionButton("Compare with reference", () => compare("reference", { reference: reference.job }), { small: true }) : null,
      keepButton,
      reference ? actionButton("Forget reference", forget, { small: true }) : null,
    ),
    h("div", { class: "compare-result" }),
    reference && reference.job !== result.job ? differenceBox("reference") : null,
    result.mode === "time" ? h("div", { class: "compare-chart" }) : null,
  );
}

// The figures of one comparison at the shown snapshot, and for a run in
// time the chart per snapshot. An exact-solution comparison made for
// another expression is shown greyed, marked "for the previous
// expression".
function updateComparison(section, kind, result, frame) {
  const slot = section.querySelector(".compare-result");
  if (!slot) return;
  const c = currentComparison(kind);
  const box = section.querySelector(`#show-${kind}`);
  const outdated = isOutdated(kind);
  if (box) {
    box.disabled = !c || c.done === 0 || outdated;
    box.checked = store.view.difference === kind && !box.disabled;
  }
  section.classList.toggle("outdated-comparison", outdated);
  // The library's message about the expression lands on its input; a
  // refused reference (another mesh) is said in the section.
  const error = comparisons[kind].result === result ? comparisons[kind].error : null;
  if (section.shownError !== error) {
    section.shownError = error;
    if (kind !== "exact" || !markError(section, error)) fill(slot, error ? errorBox(error) : null);
  }
  if (error) return;
  const chartSlot = section.querySelector(".compare-chart");
  if (!c) {
    fill(slot);
    if (chartSlot) fill(chartSlot);
    return;
  }
  const label = kind === "exact" ? "|u − exact|" : "|u − reference|";
  const at = result.mode === "time" ? `At t = ${formatNumber(result.times[frame], 5)}` : null;
  fill(
    slot,
    outdated ? h("div", { class: "row" }, h("span", { class: "chip" }, "for the previous expression"), h("span", { class: "note" }, "Compare again to update.")) : null,
    h(
      "div",
      { class: "stats" },
      stat(`max ${label}`, shownNumber(c.linf[frame]), at),
      kind === "exact" ? stat("RMS over the nodes", shownNumber(c.rms[frame]), "sqrt(Σ d_i² / N) over the N nodes") : null,
      kind === "exact" ? stat("L2 (lumped)", shownNumber(c.l2[frame]), "sqrt(Σ w_i d_i²), w_i a third of the area around node i") : null,
      result.mode === "time" ? stat(`largest max ${label}`, shownNumber(largest(c.linf)), "Over every snapshot") : null,
      c.busy ? stat("Compared", "…") : null,
    ),
    h("p", { class: "note" }, NOTES[kind], " ", FLOAT_NOTE),
  );
  if (!chartSlot) return;
  if (chartSlot.chart?.comparison !== c || chartSlot.chart.done !== c.done) {
    const series =
      kind === "exact"
        ? [
            { name: "max", cls: "s1", values: Array.from(c.linf) },
            { name: "RMS", cls: "s2", values: Array.from(c.rms) },
            { name: "L2", cls: "s3", values: Array.from(c.l2) },
          ]
        : [{ name: `max ${label}`, cls: "s1", values: Array.from(c.linf) }];
    const chart = lineChart({
      times: result.times.slice(0, c.linf.length),
      series,
      current: frame,
      width: Math.max(200, chartSlot.clientWidth || 340),
      onPick: showFrame,
      label: kind === "exact" ? "The error against the exact solution per snapshot: max, RMS and lumped L2" : "The largest difference from the reference per snapshot",
    });
    chartSlot.chart = { comparison: c, done: c.done, chart };
    const key =
      series.length > 1
        ? h(
            "div",
            { class: "chart-key" },
            series.map((s) => h("span", { class: "item" }, h("span", { class: `key-line ${s.cls}`, "aria-hidden": "true" }), s.name)),
          )
        : null;
    fill(chartSlot, h("div", { class: "chart-title" }, kind === "exact" ? "Error per snapshot" : `max ${label} per snapshot`), key, chart);
  } else chartSlot.chart.chart.setCurrent(frame);
}

function updateComparisons(container) {
  const result = freshResult("solve");
  if (!result) return;
  const frame = Math.min(store.solveFrame, result.values.length - 1);
  for (const section of container.querySelectorAll(".comparison")) updateComparison(section, section.dataset.kind, result, frame);
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
    const solveButton = actionButton("Solve", solve, { primary: true, guarded: true });
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
    const reference = store.reference;
    container.stopButton.hidden = !running;
    container.solveButton.hidden = running;
    setLabel(container.solveButton, fresh && state === "stopped" ? "Continue" : "Solve");
    if (!container.solveButton.title.startsWith("Blocked: ")) container.solveButton.title = fresh && state === "stopped" ? "Goes on from where the run stopped" : "";
    setStepStatus(container.head, "solve", result ? headStatus(result, fresh, running) : {});
    const shown = slot.shown;
    const same =
      shown &&
      shown.result === result &&
      shown.error === error &&
      shown.old === old &&
      shown.busy === busy &&
      shown.fresh === fresh &&
      shown.running === running &&
      shown.state === state &&
      shown.reference === reference;
    if (same) {
      // The same run, further on: its figures in place.
      const stats = slot.querySelector("#solve-stats");
      if (stats && running) fill(stats, resultStats(result));
      updateComparisons(container);
      return;
    }
    slot.shown = { result, error, old, busy, fresh, running, state, reference };
    const placed = markError(container, error);
    const compared = fresh && !running && result.values.length > 0;
    fill(
      slot,
      error && !placed ? errorBox(error, { openProblem }) : null,
      old ? errorBox(old, { outdated: true }) : null,
      result && !running ? warningChip(result.reports, "solve.reports") : null,
      result ? h("div", { class: "stats", id: "solve-stats" }, resultStats(result)) : null,
      result && !running ? exportsRow(result, fresh) : null,
      compared ? compareSection(result) : null,
      compared ? referenceSection(result, fresh) : null,
      result && !running ? foldable({ id: "solve.reports", title: "Reports", kind: "section", folded: true }, reportsView(result.reports)) : null,
    );
    slot.classList.toggle("outdated-result", Boolean(result) && !fresh);
    updateComparisons(container);
  },

  // Another snapshot is shown: the comparisons' figures follow it.
  frame(container) {
    updateComparisons(container);
  },
};
