// The probes pinned on the picture: a list with x, y and the shown
// field's value at each (interpolated page-side from the node values),
// and for a solution in time the u(t) of every probe over all snapshots,
// with the shown time marked. The list is the chart's legend.

import { lineChart } from "./charts.js";
import { formatNumber, interpolateAt, locatePoint, meshGrid, probeSeries } from "./core.js";
import { fill, h } from "./dom.js";
import { setProbes, store } from "./state.js";
import { showFrame } from "./timebar.js";

const grids = new WeakMap();
function gridOf(mesh) {
  let grid = grids.get(mesh);
  if (grid === undefined) {
    grid = meshGrid(mesh);
    grids.set(mesh, grid);
  }
  return grid;
}

const probeName = (probe) => `P${probe.slot + 1}`;
const remove = (probe) => setProbes(store.probes.filter((p) => p !== probe));

function probeRow(probe, found, field, digits) {
  const value = field ? interpolateAt(found, field.values) : NaN;
  return h(
    "tr",
    {},
    h("td", {}, h("span", { class: "probe-key" }, h("span", { class: `probe-dot s${probe.slot + 1}`, "aria-hidden": "true" }), probeName(probe))),
    h("td", { class: "num" }, formatNumber(probe.x)),
    h("td", { class: "num" }, formatNumber(probe.y)),
    h("td", { class: "num" }, found ? formatNumber(value, digits) : "outside"),
    h(
      "td",
      { class: "num" },
      h("button", { type: "button", class: "icon", "aria-label": `Remove probe ${probeName(probe)}`, title: "Remove", onclick: () => remove(probe) }, "✕"),
    ),
  );
}

let chartState = null;

// context: {mesh (fresh mesh view or null), field (the shown field or
// null), solution (a time solution or null), frame, digits}.
export function updateProbes(root, { mesh, field, solution, frame, digits }) {
  root.hidden = !mesh;
  if (!mesh) return;
  const probes = store.probes;
  const list = root.querySelector(".probe-list");
  const grid = gridOf(mesh);
  const located = probes.map((p) => locatePoint(mesh, grid, p.x, p.y));
  if (probes.length === 0) {
    fill(list, h("p", { class: "note" }, "Click the picture to pin a probe (up to 4)."));
  } else {
    const header = h(
      "tr",
      {},
      h("th", {}, "Probe"),
      h("th", { class: "num" }, "x"),
      h("th", { class: "num" }, "y"),
      h("th", { class: "num" }, field?.short ?? "value"),
      h("th", {}, h("button", { type: "button", class: "link", onclick: () => setProbes([]) }, "Remove all")),
    );
    fill(list, h("table", { class: "data probes-table" }, h("thead", {}, header), h("tbody", {}, probes.map((p, i) => probeRow(p, located[i], field, digits)))));
  }
  updateChart(root.querySelector(".probe-chart"), { probes, located, solution, frame });
}

function updateChart(slot, { probes, located, solution, frame }) {
  if (!solution || probes.length === 0) {
    chartState = null;
    fill(slot);
    slot.hidden = true;
    return;
  }
  slot.hidden = false;
  const width = Math.max(200, Math.round(slot.clientWidth || slot.parentElement?.clientWidth || 480));
  const same =
    chartState && chartState.probes === probes && chartState.solution === solution && chartState.count === solution.values.length && chartState.width === width;
  if (same) {
    chartState.chart.setCurrent(frame);
    return;
  }
  const series = probes.map((p, i) => ({ name: probeName(p), cls: `s${p.slot + 1}`, values: probeSeries(located[i], solution.values) }));
  const chart = lineChart({
    times: solution.times.slice(0, solution.values.length),
    series,
    current: frame,
    width,
    onPick: (k) => showFrame(k),
    label: `u over time at ${probes.map(probeName).join(", ")}`,
  });
  chartState = { probes, solution, count: solution.values.length, width, chart };
  fill(slot, h("div", { class: "chart-title" }, "u(t) at the probes"), chart);
}

// The chart is drawn for the area's width: after a resize, again.
export function forgetChartSize() {
  if (chartState) chartState.width = -1;
}
