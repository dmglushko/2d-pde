// Step 5: solving, built from the catalog's solve section: the linear
// solver, and per mode (static, in time, adaptive) its settings and slots. The head
// holds Solve (Continue after a stop, Stop during a run) and the status
// with the run's progress; the result, its exports, the comparison with
// an exact solution and with the reference run follow; the settings come
// last. The shown snapshot is the time bar's under the picture
// (timebar.js).

import { exportFile, followedRun, runWithPrerequisites, stopping, stopRun, toast } from "../actions.js";
import { lineChart } from "../charts.js";
import { LABELS, clearComparison, compare, compareWith, comparisonState, comparisons, currentComparison, forgetReference, keepReference } from "../compare.js";
import { comparisonControls, currentDisplay } from "../display.js";
import { chooseCycle, chosenCycle, cycleCount, cycleLoading } from "../cycles.js";
import { runStudy, stopStudy, study, studyOrders } from "../refinement.js";
import { bytesText, formatNumber } from "../pure/format.js";
import { isTransient, plainError } from "../pure/requests.js";
import { isTimeRun, LARGE_SOLUTION_BYTES, serverRunBytes, solutionBytes } from "../pure/stages.js";
import { adaptiveRows, referenceSlope } from "../pure/study.js";
import { actionButton, errorBox, exportButton, fill, foldable, h, infoToggle, markError, optionsForm, reportsView, stat, warningChip } from "../dom.js";
import { currentError, editCase, freshResult, lastResult, notify, openStep, outdatedError, setViewOption, store } from "../state.js";
import { showFrame } from "../timebar.js";
import { timeOrderText } from "./assemble.js";
import { choiceCard } from "./components.js";
import { setStepStatus, stepHead } from "./head.js";

const solve = () => runWithPrerequisites("solve");
const openProblem = () => openStep("problem");

// Above this share of the server's memory budget for runs, the Solve
// step says what a run will keep there before it is started.
const LARGE_SERVER_SHARE = 0.25;

// A warning when a run would be large, from the fresh mesh: what it keeps
// on the server against the server's budget (the server refuses a run
// that does not fit), and what the page holds of it; nothing otherwise.
function sizeWarning() {
  const mesh = freshResult("mesh");
  const nodes = mesh?.stats?.nodes;
  const server = serverRunBytes(store.caseData.solve, mesh?.memory, freshResult("problem")?.time_order ?? null);
  const lines = [];
  if (server && server.share >= LARGE_SERVER_SHARE) {
    const share = `${Math.round(100 * server.share)}%`;
    lines.push(
      server.share > 1
        ? `This run would keep about ${bytesText(server.bytes)} on the server, more than its whole memory budget for runs (${bytesText(server.budget)}): it will be refused. ${{ adaptive: "Lower the maximum nodes or cycles.", adaptive_time: "Lower the maximum nodes or refinements, or keep fewer snapshots." }[store.caseData.solve?.mode] ?? "Keep fewer snapshots or use a coarser mesh."}`
        : `This run will keep about ${bytesText(server.bytes)} on the server: ${share} of its memory budget for runs (${bytesText(server.budget)}). Other tabs' finished runs may be dropped to make room.`,
    );
  }
  const bytes = solutionBytes(store.caseData.solve, nodes);
  if (bytes !== null && bytes > LARGE_SOLUTION_BYTES)
    lines.push(
      `The page holds up to ${bytesText(bytes)} of it as float32 (${formatNumber(nodes)} nodes × ${formatNumber(bytes / 4 / nodes)} times), loading snapshots as they are shown.`,
    );
  const path = { adaptive: "solve.adaptive.max_nodes", adaptive_time: "solve.adaptive_time.max_nodes" }[store.caseData.solve?.mode] ?? "solve.time.snapshots";
  return lines.length ? h("div", { class: "notice", dataset: { path } }, lines.map((line) => h("p", {}, line))) : null;
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
  if (result.mode === "adaptive") {
    const adaptive = result.adaptive ?? {};
    const cycles = adaptive.cycles ?? [];
    const last = cycles.at(-1);
    return [
      stat("Cycles", formatNumber(cycles.length)),
      stat("Stopped", STOP_REASONS[adaptive.stop_reason] ?? adaptive.stop_reason ?? "…", adaptive.tolerance ? `Tolerance ${formatNumber(adaptive.tolerance, 3)}` : null),
      last ? stat("Nodes", formatNumber(last.nodes), `${formatNumber(last.triangles)} triangles in the last mesh`) : null,
      stat("Unknowns", formatNumber(result.free_unknowns)),
      last && Number.isFinite(last.estimate) ? stat("Estimate", formatNumber(last.estimate, 3), "The error estimator's total √(Σ η_T²) on the last mesh") : null,
      adaptive.mesh_stats ? stat("Smallest angle", `${formatNumber(adaptive.mesh_stats.min_angle_degrees, 3)}°`, "Of the last mesh") : null,
      stat("min u", formatNumber(result.min)),
      stat("max u", formatNumber(result.max)),
      iterations(staticReport.linear_iterations),
      stat("Residual", formatNumber(staticReport.residual, 2), "‖b − A U‖ of the last linear solve: how well the system was solved, not the error of u"),
    ];
  }
  if (result.mode === "adaptive_time") {
    const adaptive = result.adaptive_time ?? {};
    const meshes = adaptive.meshes ?? [];
    const last = meshes.at(-1);
    return [
      stat("Snapshots", formatNumber(result.values.length)),
      stat("Steps", formatNumber(adaptive.steps), adaptive.damping_steps > 0 ? `${formatNumber(adaptive.damping_steps)} of them damping steps` : null),
      stat("Refinements", formatNumber(adaptive.refinements), `${formatNumber(adaptive.adaptations?.length ?? 0)} adaptation times`),
      last ? stat("Nodes", formatNumber(last.nodes), `From ${formatNumber(meshes[0].nodes)} at t0 to the last mesh`) : null,
      stat("Unknowns", formatNumber(result.free_unknowns), "Of the last mesh"),
      stat("Time order", timeOrderText(result.time_order, result.problem_time_order)),
      adaptive.mesh_stats ? stat("Smallest angle", `${formatNumber(adaptive.mesh_stats.min_angle_degrees, 3)}°`, "Of the last mesh") : null,
      stat("min u", formatNumber(result.min), "Over every snapshot"),
      stat("max u", formatNumber(result.max), "Over every snapshot"),
    ];
  }
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
  // An adaptive run: its cycles as they come (the loop is one call: Stop
  // takes effect after the current cycle).
  if (running && result.mode === "adaptive") {
    const cycles = result.adaptive?.cycles ?? [];
    const last = cycles.at(-1);
    return {
      status: "busy",
      text: stopping(result) ? "Stopping after this cycle…" : `Running: cycle ${cycles.length + 1}`,
      detail: last ? `cycle ${last.cycle}: ${formatNumber(last.nodes)} nodes${Number.isFinite(last.estimate) ? `, estimate ${formatNumber(last.estimate, 3)}` : ""}` : "solving on the starting mesh",
    };
  }
  if (running) {
    const span = result.t1 - result.t0;
    const bar = h("progress", { max: "1", value: String(span > 0 ? Math.max(0, Math.min(1, (result.t - result.t0) / span)) : 0), "aria-label": "Run progress" });
    const steps = result.reports?.[0]?.metrics?.steps ?? 0;
    // A progress poll that is being tried again says so (actions.js).
    const notice = store.runNotice ? h("div", { class: "inline-failure", role: "status" }, store.runNotice) : null;
    // An adaptive run in time shows its snapshots and meshes at its end.
    const detail = result.mode === "adaptive_time" ? "the snapshots come when the run ends" : `${formatNumber(steps)} steps · ${result.values.length} of ${result.snapshots_total} snapshots`;
    return {
      status: "busy",
      text: stopping(result) ? `Stopping at t = ${formatNumber(result.t, 5)}…` : `Running: t = ${formatNumber(result.t, 5)} of ${formatNumber(result.t1)}`,
      detail,
      extra: notice ? h("div", {}, bar, notice) : bar,
    };
  }
  if (fresh && result.state === "stopped")
    return { status: "stopped", text: result.mode === "adaptive" ? `Stopped after cycle ${result.adaptive?.cycles?.length ?? 0}` : `Stopped at t = ${formatNumber(result.t, 5)}` };
  if (fresh && result.state === "failed") return { status: "failed", text: `Failed after t = ${formatNumber(result.t, 5)}` };
  if (fresh)
    return {
      detail:
        result.mode === "time"
          ? `${result.values.length} snapshots`
          : result.mode === "adaptive"
            ? `adaptive · ${result.adaptive?.cycles?.length ?? 0} cycles`
            : result.mode === "adaptive_time"
              ? `adaptive in time · ${result.values.length} snapshots · ${formatNumber(result.adaptive_time?.refinements ?? 0)} refinements`
              : "static",
    };
  return {};
}

// --- exports -------------------------------------------------------------------

const EXPORTS = [
  ["solution.csv", "Export .csv", "One row per mesh node: x, y and u at every kept time, in full precision."],
  ["solution.vtk", "Export .vtk", "The mesh and u at every kept time, for ParaView or VisIt (VTK legacy)."],
  ["solution-vtk.zip", "Export VTK series", "One VTK file per snapshot and a .vtk.series manifest in a zip: ParaView plays it at the snapshots' times."],
  ["provenance.json", "Export provenance", "The whole case, the app and library versions and this run: what made the results."],
];

// An adaptive solve's last mesh, which the Mesh step does not have.
const ADAPTED_MESH_EXPORT = ["solution-mesh.pde2d", "Export adapted mesh", "The last mesh of the adaptive solve in the .pde2d format: open it as a mesh file to start from it."];

// The exports of a result: a run on several meshes (adaptive in time) has
// no one-mesh CSV or VTK file, only the series (a file per snapshot).
function exportKinds(result) {
  if (result.mode === "adaptive") return [...EXPORTS, ADAPTED_MESH_EXPORT];
  if (result.mode === "adaptive_time") return [...EXPORTS.filter(([kind]) => kind !== "solution.csv" && kind !== "solution.vtk"), ADAPTED_MESH_EXPORT];
  return EXPORTS;
}

function exportsRow(result, fresh) {
  const done = fresh && result.state === "done";
  const adaptive = result.mode === "adaptive" || result.mode === "adaptive_time";
  const stale =
    result.state === "done"
      ? "The solution shown is outdated: solve again to export it."
      : adaptive
        ? "The run was stopped before its end: solve again to export it."
        : "The run did not reach t1: continue it to export.";
  const kinds = exportKinds(result);
  return h(
    "div",
    { class: "row actions-row" },
    kinds.map(([kind, label, about]) => exportButton(label, () => exportFile(kind), { fresh: done, staleReason: stale, about })),
  );
}

// --- the adaptive cycles -------------------------------------------------------------

// The reports whose messages the warning chip counts: an adaptive solve
// that met its tolerance says why it stopped, which is no warning (the
// stats say it); one stopped by a budget is.
function warningReports(result) {
  if (result.mode !== "adaptive" || result.adaptive?.stop_reason !== "tolerance_met") return result.reports;
  // Only the stop message goes: earlier cycles' warnings (prefixed "cycle
  // k, ", runs.cpp) stay.
  return result.reports.map((r) => (r.stage === "adaptive_static_solver" ? { ...r, messages: (r.messages ?? []).filter((m) => /^cycle \d+, /.test(m)) } : r));
}

const RATE_TITLE = "The observed rate: e ~ N^(−rate) between this cycle and the one before";

// Why the adaptive loop stopped, in words.
const STOP_REASONS = {
  tolerance_met: "tolerance met",
  max_cycles: "maximum cycles",
  node_budget: "node budget",
  nothing_changed: "mesh unchanged",
};

// The cycles of an adaptive solve: a table (nodes, the estimate, the
// marked triangles, and with a comparison with an exact solution each
// cycle's errors) and a log–log chart against the number of nodes with
// the slopes N^(-1/2) and N^(-1). Drawn again when the result or its
// comparison changes.
// What an outcome of an adaptation time means, in words.
const OUTCOMES = {
  refined: "refined",
  tolerance_met: "tolerance met",
  nothing_changed: "mesh unchanged",
  node_budget: "node budget: stopped",
};

// The adaptation times of an adaptive run in time: a row per time (the
// step count, the mesh's nodes, the estimate and what happened), and a
// chart of the nodes against t.
function adaptationsSection(result) {
  const rows = result.adaptive_time?.adaptations ?? [];
  if (rows.length === 0) return null;
  const number = (v, digits = 3) => (v === null || v === undefined ? "–" : formatNumber(v, digits));
  const table = h(
    "table",
    { class: "data study-table cycles-table adaptations-table" },
    h(
      "thead",
      {},
      h(
        "tr",
        {},
        h("th", { class: "num" }, "t"),
        h("th", { class: "num", title: "Nodes of the mesh the solution was on" }, "N"),
        h("th", { class: "num", title: "The error estimator's total √(Σ η_T²) at that time" }, "Estimate"),
        h("th", {}, "Outcome"),
      ),
    ),
    h(
      "tbody",
      {},
      rows.map((r) =>
        h(
          "tr",
          {},
          h("td", { class: "num", title: `after ${formatNumber(r.steps)} steps` }, formatNumber(r.time, 4)),
          h("td", { class: "num" }, formatNumber(r.nodes)),
          h("td", { class: "num" }, number(r.estimate)),
          h(
            "td",
            { title: r.marked === null || r.marked === undefined ? "" : `${formatNumber(r.marked)} triangles marked` },
            r.outcome === "refined" ? `refined → ${formatNumber(r.nodes_after)}` : (OUTCOMES[r.outcome] ?? r.outcome),
          ),
        ),
      ),
    ),
  );
  // To where the run got (t1, or where it was stopped).
  const end = result.state === "stopped" && Number.isFinite(result.t) ? result.t : result.t1;
  const times = [result.t0, ...rows.map((r) => r.time), end];
  const nodes = [rows[0].nodes, ...rows.map((r) => r.nodes_after), rows.at(-1).nodes_after];
  const chart = h(
    "div",
    { class: "study-chart" },
    h("div", { class: "chart-title" }, "Nodes of the mesh against t"),
    lineChart({
      times,
      series: [{ name: "nodes", cls: "s1", values: nodes }],
      current: null,
      xName: "t",
      width: 340,
      height: 140,
      label: "The number of nodes of the mesh against time: it grows at each refinement",
      csvName: "adaptations.csv",
      endLabels: false,
    }),
  );
  const messages = result.adaptive_time?.messages ?? [];
  const info = infoToggle(
    "At each adaptation time the run estimates the error of the solution on every triangle; above the tolerance (or always, without one) it marks triangles, refines them and moves the solution to the new mesh by interpolation, then goes on. The mesh only grows: the library has no coarsening yet, so a feature that moves leaves refined triangles behind. Each snapshot is drawn on the mesh it was computed on.",
    "the adaptation times",
  );
  return foldable(
    { id: "solve.adaptations", title: "Adaptation times", kind: "section", extras: [info.button], info: info.panel, attributes: { class: "cycles" } },
    table,
    chart,
    messages.map((m) => h("p", { class: "note" }, m)),
  );
}

function updateCycles(container) {
  const slot = container.querySelector("#solve-cycles");
  if (!slot) return;
  const result = freshResult("solve") ?? lastResult("solve");
  const compared = currentComparison("exact");
  const errors = compared && comparisonState("exact") === "current" ? (compared.summary?.cycles ?? null) : null;
  // A running adaptive run's result grows in place: its cycles' count is
  // part of the key.
  const key = [result, errors, result?.adaptive?.cycles?.length ?? 0, result?.state, chosenCycle(result), cycleLoading(result)];
  if (slot.shown && key.every((part, i) => slot.shown[i] === part)) return;
  slot.shown = key;
  if (result?.mode === "adaptive_time") return fill(slot, adaptationsSection(result));
  if (result?.mode !== "adaptive" || !result.adaptive?.cycles?.length) return fill(slot);
  const rows = adaptiveRows(result.adaptive.cycles, errors);
  const number = (v, digits = 3) => (v === null || v === undefined ? "–" : formatNumber(v, digits));
  const withErrors = Boolean(errors);
  // Each cycle's solution in the picture (cycles.js): once the run is
  // published, a cycle's number shows it; the last is the result itself.
  const count = cycleCount(result);
  const chosen = chosenCycle(result) ?? count;
  const pick = (cycle) => chooseCycle(result, cycle === count ? null : cycle);
  const cycleCell = (r) =>
    count > 0
      ? h(
          "button",
          {
            type: "button",
            class: "link cycle-pick",
            "aria-pressed": String(r.cycle === chosen),
            title: r.cycle === chosen ? `The picture shows cycle ${r.cycle}` : `Show cycle ${r.cycle}'s solution on its mesh in the picture`,
            onclick: () => pick(r.cycle),
          },
          String(r.cycle),
        )
      : String(r.cycle);
  const shownLine =
    count > 0
      ? h(
          "p",
          { class: "note cycle-shown", role: "status" },
          chosen === count
            ? "The picture shows the last cycle. Click a cycle's number (or a point of the chart) to see its solution on its mesh."
            : [
                `The picture shows cycle ${chosen} of ${count}${cycleLoading(result) ? " (loading…)" : ""}: its mesh and its solution, on the run's colour range. `,
                h("button", { type: "button", class: "link", onclick: () => pick(count) }, "Show the last"),
              ],
        )
      : null;
  const table = h(
    "table",
    { class: "data study-table cycles-table" },
    h(
      "thead",
      {},
      h(
        "tr",
        {},
        h("th", { class: "num" }, "Cycle"),
        h("th", { class: "num", title: "Nodes of the cycle's mesh" }, "N"),
        h("th", { class: "num" }, "Estimate"),
        h("th", { class: "num", title: RATE_TITLE }, "rate"),
        withErrors ? [h("th", { class: "num", title: "The lumped L2 norm of u − exact" }, "L2 error"), h("th", { class: "num", title: RATE_TITLE }, "rate")] : null,
      ),
    ),
    h(
      "tbody",
      {},
      rows.map((r) =>
        h(
          "tr",
          { class: count > 0 && r.cycle === chosen ? "shown-cycle" : null },
          h("td", { class: "num", title: r.marked === null ? "The last cycle: nothing marked" : `${formatNumber(r.marked)} of ${formatNumber(r.triangles)} triangles marked` }, cycleCell(r)),
          h("td", { class: "num" }, formatNumber(r.nodes)),
          h("td", { class: "num" }, number(r.estimate)),
          h("td", { class: "num cycle-rate", dataset: { of: "estimate" } }, number(r.estimateRate, 2)),
          withErrors ? [h("td", { class: "num" }, number(r.l2)), h("td", { class: "num cycle-rate", dataset: { of: "l2" } }, number(r.l2Rate, 2))] : null,
        ),
      ),
    ),
  );
  let chart = null;
  const plotted = rows.filter((r) => r.nodes > 0 && r.estimate > 0);
  if (plotted.length >= 2) {
    const nodes = plotted.map((r) => r.nodes);
    const first = plotted[0];
    const series = [{ name: "estimate", cls: "s1", values: plotted.map((r) => r.estimate) }];
    if (withErrors) {
      series.push({ name: "L2 error", cls: "s2", values: plotted.map((r) => (r.l2 > 0 ? r.l2 : NaN)) });
      series.push({ name: "max error", cls: "s3", values: plotted.map((r) => (r.max > 0 ? r.max : NaN)) });
    }
    series.push({ name: "N^(−1/2)", cls: "ref", values: referenceSlope(nodes, -0.5, first.nodes, first.estimate) });
    if (withErrors && first.l2 > 0) series.push({ name: "N^(−1)", cls: "ref ref-2", values: referenceSlope(nodes, -1, first.nodes, first.l2) });
    chart = h(
      "div",
      { class: "study-chart" },
      h("div", { class: "chart-title" }, "Against the number of nodes N (log–log)"),
      lineChart({
        key: true,
        times: nodes,
        series,
        current: count > 0 && plotted.some((r) => r.cycle === chosen) ? plotted.findIndex((r) => r.cycle === chosen) : null,
        onPick: count > 0 ? (index) => pick(plotted[index].cycle) : null,
        log: true,
        xName: "N",
        width: Math.max(220, slot.clientWidth || 340),
        height: 180,
        label: "The error estimate of each adaptive cycle against its number of nodes N, on log–log axes, with the errors against the exact solution when compared, and reference slopes N^(−1/2) and N^(−1)",
        csvName: "adaptive-cycles.csv",
        endLabels: false,
      }),
    );
  }
  const info = infoToggle(
    "Each cycle solves on a mesh, estimates the error of every triangle, marks some and refines them. The residual estimator's total bounds the error in the energy norm up to a constant; the Kelly and recovery estimators only indicate it (the tolerance applies to the chosen estimator's total). On a well-adapted mesh of P1 elements the estimate falls like N^(−1/2), and an L2 error like N^(−1), also for a solution with a corner singularity, where uniform refinement is slower. Compare with an exact solution (below) to add each cycle's error. Colour the picture by the error indicator (Colour menu) to see where the last estimate puts the error.",
    "the adaptive cycles",
  );
  fill(
    slot,
    foldable(
      { id: "solve.cycles", title: "Adaptive cycles", kind: "section", extras: [info.button], info: info.panel, attributes: { class: "cycles" } },
      shownLine,
      table,
      chart,
      withErrors ? null : h("p", { class: "note" }, "Compare with an exact solution to add each cycle's error."),
    ),
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
  exact: "Nodal error u − exact, computed on the server in double: not the solver's residual ‖b − A U‖.",
  reference: "u − reference at the nodes, at the same times, computed on the server in double.",
};
const FLOAT_NOTE = "Only the picture is float32 (the difference is computed in double, then drawn): below about 1e−7 × |u| it shows rounding.";

// The checkbox that asks for u − other in the picture, and beside it why
// the picture does not draw it when it does not (display.js).
function differenceBox(kind) {
  const id = `show-${kind}`;
  const box = h("input", { type: "checkbox", id, onchange: (event) => setViewOption("difference", event.target.checked ? kind : null) });
  return h(
    "div",
    { class: "difference-line" },
    h("label", { class: "row inline-label", for: id }, box, `Show ${LABELS[kind]} in the picture`),
    h("p", { class: "note difference-note", hidden: true }),
  );
}

// The case's exact solution (view.exact_solution; blank removes it).
function setExact(value) {
  editCase(
    (c) => {
      if (value.trim() === "") {
        if (c.view) delete c.view.exact_solution;
      } else (c.view ??= {}).exact_solution = value;
    },
    "edit",
    { sections: ["view"] },
  );
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
    oninput: (event) => {
      setExact(event.target.value);
      remove.hidden = event.target.value.trim() === "" && !currentComparison("exact");
    },
    onkeydown: (event) => {
      if (event.key === "Enter") run();
    },
  });
  const run = () => {
    const value = text.value.trim();
    if (value === "") return clearComparison("exact");
    return compareWith(value);
  };
  // Forgets the exact solution: the expression leaves the case, and the
  // errors leave the figures, the cycles' table and chart and the picture.
  const remove = h(
    "button",
    {
      type: "button",
      class: "small",
      title: "Remove the exact solution and its errors",
      hidden: text.value.trim() === "" && !currentComparison("exact"),
      onclick: () => {
        text.value = "";
        setExact("");
        clearComparison("exact");
        remove.hidden = true;
      },
    },
    "Remove",
  );
  const info = infoToggle(
    "The server evaluates the expression (in x, y, t and the parameters) with the library at every snapshot and compares it with u in double precision: the largest nodal error, its root mean square over the nodes, and the lumped L2 norm sqrt(Σ w_i d_i²) with w_i a third of the area of the triangles around node i.",
    "the comparison",
  );
  return foldable(
    { id: "solve.compare", title: "Compare with an exact solution", kind: "section", extras: [info.button], info: info.panel, attributes: { class: "comparison", dataset: { kind: "exact" } } },
    h("div", { class: "field", dataset: { path: "exact" } }, h("div", { class: "row compare-line" }, text, actionButton("Compare", run, { small: true }), remove)),
    h("div", { class: "compare-result" }),
    differenceBox("exact"),
    isTimeRun(result) ? h("div", { class: "compare-chart" }) : null,
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
  const describe = (r) => `run ${r.job}, ${isTimeRun(r) ? `in time, ${r.times.length} snapshots` : "static"}`;
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
    isTimeRun(result) ? h("div", { class: "compare-chart" }) : null,
  );
}

// The figures of one comparison at the shown snapshot, and for a run in
// time the chart per snapshot. An exact-solution comparison made for
// another expression is shown greyed, marked "for the previous
// expression".
function updateComparison(section, kind, result, frame, display) {
  const slot = section.querySelector(".compare-result");
  if (!slot) return;
  const c = currentComparison(kind);
  const box = section.querySelector(`#show-${kind}`);
  const controls = comparisonControls(kind, display);
  const outdated = controls.state === "outdated";
  if (box) {
    // The user's choice, as asked; the note says when the picture cannot
    // draw it (an outdated or missing comparison is never drawn).
    const requested = store.view.difference === kind;
    box.disabled = !c && !requested;
    box.checked = requested;
    const note = section.querySelector(".difference-note");
    const text = requested && !controls.shown ? display.difference.note : null;
    note.hidden = !text;
    note.textContent = text ?? "";
  }
  section.classList.toggle("outdated-comparison", outdated);
  // The library's message about the expression lands on its input; a
  // refused reference (another mesh) is said in the section.
  const error = comparisons[kind].result === result ? comparisons[kind].error : null;
  if (section.shownError !== error) {
    section.shownError = error;
    // The figures of the failed attempt go either way. A failure that may
    // pass (the server busy or not answering) offers to try again.
    const again = () => compare(kind, comparisons[kind].against);
    const retry = error && isTransient(error) ? h("div", { class: "inline-failure", role: "status" }, plainError(error, "the comparison").text, actionButton("Retry", again, { small: true })) : null;
    if (retry) fill(slot, retry);
    else if (kind !== "exact" || !markError(section, error)) fill(slot, error ? errorBox(error) : null);
    else fill(slot);
    const chart = section.querySelector(".compare-chart");
    if (error && chart) {
      fill(chart);
      chart.chart = null;
    }
  }
  if (error) return;
  const chartSlot = section.querySelector(".compare-chart");
  if (!c) {
    fill(slot);
    if (chartSlot) fill(chartSlot);
    return;
  }
  const label = kind === "exact" ? "|u − exact|" : "|u − reference|";
  const at = isTimeRun(result) ? `At t = ${formatNumber(result.times[frame], 5)}` : null;
  fill(
    slot,
    outdated
      ? h("div", { class: "row" }, h("span", { class: "chip" }, kind === "exact" ? "for the previous expression" : "for the previous reference run"), h("span", { class: "note" }, "Compare again to update."))
      : null,
    h(
      "div",
      { class: "stats" },
      stat(`max ${label}`, shownNumber(c.linf[frame]), at),
      kind === "exact" ? stat("RMS (nodes)", shownNumber(c.rms[frame]), "sqrt(Σ d_i² / N) over the N nodes") : null,
      kind === "exact" ? stat("L2 (lumped)", shownNumber(c.l2[frame]), "sqrt(Σ w_i d_i²), w_i a third of the area around node i") : null,
      isTimeRun(result) ? stat(`largest max ${label}`, shownNumber(largest(c.linf)), "Over every snapshot") : null,
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
      key: series.length > 1,
      times: result.times.slice(0, c.linf.length),
      series,
      current: frame,
      width: Math.max(200, chartSlot.clientWidth || 340),
      onPick: showFrame,
      label: kind === "exact" ? "The error against the exact solution per snapshot: max, RMS and lumped L2" : "The largest difference from the reference per snapshot",
      csvName: kind === "exact" ? "error-per-snapshot.csv" : "reference-difference.csv",
    });
    chartSlot.chart = { comparison: c, done: c.done, chart };
    fill(chartSlot, h("div", { class: "chart-title" }, kind === "exact" ? "Error per snapshot" : `max ${label} per snapshot`), chart);
  } else chartSlot.chart.chart.setCurrent(frame);
}

function updateComparisons(container) {
  updateStudy(container);
  updateCycles(container);
  const result = freshResult("solve");
  if (!result) return;
  const frame = Math.min(store.solveFrame, result.values.length - 1);
  const display = currentDisplay();
  for (const section of container.querySelectorAll(".comparison")) updateComparison(section, section.dataset.kind, result, frame, display);
}

// A run's error: a run that is gone (expired) or replaced is said in
// plain words, a gone one with "Solve again"; others are the library's.
function runErrorBox(error) {
  if (error.kind !== "expired" && error.kind !== "replaced") return errorBox(error, { openProblem });
  const plain = plainError(error);
  return h(
    "div",
    { class: "alert", role: "alert" },
    h("span", { class: "stage" }, error.kind === "expired" ? "The result is gone" : "Replaced"),
    plain.text,
    plain.action === "solve" ? h("div", { class: "alert-actions" }, actionButton("Solve again", solve, { small: true, guarded: true })) : null,
  );
}

// --- the refinement study ---------------------------------------------------------

const STUDY_LEVELS = [3, 4, 5];

// The study's controls, table and chart (refinement.js). It is offered
// with an exact solution in the case; it stays while it runs, whatever
// the solve result does meanwhile.
function updateStudy(container) {
  const slot = container.querySelector("#solve-study");
  if (!slot) return;
  const expression = (store.caseData.view?.exact_solution ?? "").trim();
  // Not for an adaptive solve, which refines its mesh itself.
  const adaptive = ["adaptive", "adaptive_time"].includes(store.caseData.solve?.mode);
  // Old rows or a message stay shown, but not in an adaptive mode, where
  // a study cannot be started (refinement.js).
  const offered = study.running || (!adaptive && (study.rows.length > 0 || study.message || (expression && freshResult("solve")?.state === "done")));
  const key = JSON.stringify([offered, study.running, study.stopping, study.rows.length, study.message, expression !== ""]);
  if (slot.shown === key) return;
  slot.shown = key;
  if (!offered) return fill(slot);
  const levels = h(
    "select",
    { "aria-label": "Number of levels", disabled: study.running },
    STUDY_LEVELS.map((n) => h("option", { value: String(n), selected: n === (study.levels || 3) }, `${n} levels`)),
  );
  const start = actionButton("Run study", () => runStudy(Number(levels.value)), { small: true, guarded: true });
  start.disabled = study.running || !expression;
  if (!expression) start.title = "Write an exact solution above first.";
  const stop = actionButton(study.stopping ? "Stopping…" : "Stop study", stopStudy, { small: true });
  stop.disabled = study.stopping;
  const info = infoToggle(
    "Solves the case on meshes whose max edge length is halved level by level (the setting of the Mesh step, put back at the end), compares each solution with the exact solution on the server in double, and gives the observed order log(e₁/e₂) / log(h₁/h₂) between levels, with h the longest edge. P1 elements give about h² in L2 for a smooth solution; a corner singularity gives less.",
    "the refinement study",
  );
  const rows = study.rows;
  const orders = studyOrders(rows);
  const order = (v) => (v === null ? "–" : formatNumber(v, 3));
  const table = rows.length
    ? h(
        "table",
        { class: "data study-table" },
        h(
          "thead",
          {},
          h("tr", {}, h("th", { class: "num" }, "h"), h("th", { class: "num" }, "Nodes"), h("th", { class: "num" }, "max error"), h("th", { class: "num" }, "order"), h("th", { class: "num" }, "L2 error"), h("th", { class: "num" }, "order")),
        ),
        h(
          "tbody",
          {},
          rows.map((r, i) =>
            h(
              "tr",
              {},
              h("td", { class: "num" }, formatNumber(r.h, 3)),
              h("td", { class: "num" }, formatNumber(r.nodes)),
              h("td", { class: "num" }, formatNumber(r.max, 3)),
              h("td", { class: "num study-order", dataset: { norm: "max" } }, order(orders[i].max)),
              h("td", { class: "num" }, formatNumber(r.l2, 3)),
              h("td", { class: "num study-order", dataset: { norm: "l2" } }, order(orders[i].l2)),
            ),
          ),
        ),
      )
    : null;
  let chart = null;
  if (rows.length >= 2) {
    const hs = rows.map((r) => r.h);
    const anchor = rows[0];
    const series = [
      { name: "max error", cls: "s1", values: rows.map((r) => r.max) },
      { name: "L2 error", cls: "s2", values: rows.map((r) => r.l2) },
      { name: "h¹", cls: "ref", values: referenceSlope(hs, 1, anchor.h, anchor.l2) },
      { name: "h²", cls: "ref ref-2", values: referenceSlope(hs, 2, anchor.h, anchor.l2) },
    ];
    chart = h(
      "div",
      { class: "study-chart" },
      h("div", { class: "chart-title" }, "Error against h (log–log)"),
      lineChart({
        key: true,
        times: hs,
        series,
        current: null,
        log: true,
        xName: "h",
        width: Math.max(220, slot.clientWidth || 340),
        height: 180,
        label: "The largest nodal and lumped L2 errors against the longest edge h, on log–log axes, with reference slopes h¹ and h² through the first L2 point",
        csvName: "refinement-study.csv",
        endLabels: false,
      }),
    );
  }
  fill(
    slot,
    foldable(
      { id: "solve.study", title: "Refinement study", kind: "section", extras: [info.button], info: info.panel, attributes: { class: "study" } },
      h("div", { class: "row" }, levels, study.running ? stop : start),
      study.running ? h("p", { class: "note", role: "status" }, `Level ${rows.length + 1} of ${study.levels}…`) : null,
      study.message ? h("p", { class: study.failed ? "note study-failed" : "note", role: "status" }, study.message) : null,
      table,
      chart,
    ),
  );
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
  if (!isTimeRun(solveCase) && order > 0)
    return h(
      "div",
      { class: "notice" },
      `The problem has time order ${order}: a ${solveCase.mode === "adaptive" ? "adaptive (static) " : "static "}solve drops u_t and u_tt. `,
      h("button", { type: "button", class: "small", onclick: () => editCase((c) => (c.solve.mode = solveCase.mode === "adaptive" ? "adaptive_time" : "time"), "structure") }, solveCase.mode === "adaptive" ? "Adapt in time" : "Solve in time"),
    );
  if (isTimeRun(solveCase) && order === 0) return h("div", { class: "notice" }, "The problem has time order 0: each step is a static solve, which only some methods support.");
  return null;
}

// One mode's part of the solve section: its settings and its slots.
function modePart(mode, solveCase) {
  const key = mode.settings.key;
  const modeSettings = (solveCase[key] ??= {});
  return [
    foldable(
      { id: `solve.${key}`, title: mode.settings.label, attributes: { dataset: { path: `solve.${key}` } } },
      optionsForm(mode.settings.options, modeSettings, `solve.${key}`, () => editCase(() => {}, "edit", { sections: ["solve"] })),
    ),
    mode.slots.map((slot) => choiceCard(slot, modeSettings[slot.id], `solve.${key}.${slot.id}`, (next) => editCase((c) => (c.solve[key][slot.id] = next), "structure"))),
  ];
}

function settingsPart(section, solveCase, mode) {
  // A mode may also use other modes' parts (adaptive in time: the time
  // integration's), shown first.
  const others = (mode.with ?? []).map((id) => section.modes.find((m) => m.id === id)).filter(Boolean);
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
    others.map((other) => modePart(other, solveCase)),
    modePart(mode, solveCase),
    threadsPart(section, solveCase),
    // Its options are rarely changed: folded for a new viewer. Its
    // threads are in the Threads card.
    choiceCard(section.linear_solver, solveCase.linear_solver, "solve.linear_solver", pickLinear, { folded: true, hidden: [THREADS] }),
  );
}

// The option every parallel component names its thread count with.
const THREADS = "threads";

// Threads, in one place for every mode: the assembly's (the assembler's
// options) and the linear solver's when it has the option; when it has
// not, which solvers do (from the catalog).
function threadsPart(section, solveCase) {
  const changed = () => editCase(() => {}, "edit", { sections: ["solve"] });
  const rows = [];
  if (section.assembler) {
    const values = (solveCase.assembler ??= {});
    const specs = section.assembler.options.map((o) => (o.name === THREADS ? { ...o, label: "Assembly" } : o));
    rows.push(h("div", { dataset: { path: "solve.assembler" } }, optionsForm(specs, values, "solve.assembler", changed)));
  }
  const choice = solveCase.linear_solver;
  const solvers = section.linear_solver.components;
  const solver = solvers.find((c) => c.type === choice?.type);
  const spec = solver?.options.find((o) => o.name === THREADS);
  if (spec) {
    choice.options ??= {};
    rows.push(h("div", { dataset: { path: "solve.linear_solver.options" } }, optionsForm([{ ...spec, label: `Linear solver (${solver.label})` }], choice.options, "solve.linear_solver.options", changed)));
  } else if (solver) {
    const parallel = solvers.filter((c) => c.options.some((o) => o.name === THREADS)).map((c) => c.label);
    rows.push(
      h(
        "div",
        { class: "note threads-note" },
        `The linear solver, ${solver.label}, runs in one thread.`,
        parallel.length ? ` Solvers with threads (Linear solver card below): ${parallel.join("; ")}.` : "",
      ),
    );
  }
  return foldable({ id: "solve.threads", title: "Threads" }, rows);
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
    fill(container, head, h("div", { class: "result", id: "solve-result" }), h("div", { id: "solve-cycles" }), h("div", { id: "solve-study" }), settingsPart(section, solveCase, mode));
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
    const sizeKey = JSON.stringify([freshResult("mesh")?.stats?.nodes ?? null, store.caseData.solve?.mode, store.caseData.solve?.time?.snapshots, store.caseData.solve?.adaptive?.max_nodes, store.caseData.solve?.adaptive?.max_cycles, store.caseData.solve?.adaptive_time?.max_nodes, store.caseData.solve?.adaptive_time?.max_refinements, freshResult("problem")?.time_order ?? null]);
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
    // A stopped adaptive run is not continued: Solve starts it again.
    const continues = fresh && state === "stopped" && result.mode !== "adaptive" && result.mode !== "adaptive_time";
    const reference = store.reference;
    container.stopButton.hidden = !running;
    container.solveButton.hidden = running;
    setLabel(container.solveButton, continues ? "Continue" : "Solve");
    container.stopButton.disabled = running && stopping(result);
    if (!container.solveButton.title.startsWith("Blocked: ")) container.solveButton.title = continues ? "Goes on from where the run stopped" : "";
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
      updateCycles(container);
      updateComparisons(container);
      return;
    }
    slot.shown = { result, error, old, busy, fresh, running, state, reference };
    const placed = markError(container, error);
    const compared = fresh && !running && result.values.length > 0;
    fill(
      slot,
      error && !placed ? runErrorBox(error) : null,
      old ? errorBox(old, { outdated: true }) : null,
      result && !running ? warningChip(warningReports(result), "solve.reports") : null,
      result ? h("div", { class: "stats", id: "solve-stats" }, resultStats(result)) : null,
      result && !running ? exportsRow(result, fresh) : null,
      compared ? compareSection(result) : null,
      // Two adaptive solves end on different meshes: no node-by-node
      // comparison between them.
      compared && result.mode !== "adaptive" && result.mode !== "adaptive_time" ? referenceSection(result, fresh) : null,
      result && !running ? foldable({ id: "solve.reports", title: "Reports", kind: "section", folded: true }, reportsView(result.reports)) : null,
    );
    slot.classList.toggle("outdated-result", Boolean(result) && !fresh);
    updateComparisons(container);
  },

  // Another snapshot is shown: the comparisons' figures follow it.
  frame(container) {
    updateComparisons(container);
  },

  // The refinement study moved on.
  study(container) {
    updateStudy(container);
  },
};
