// The probes pinned on the picture: a list with x, y and the shown
// quantity's value at each, and for a solution in time the u(t) of every
// probe over all snapshots, with the shown time marked. Where the picture
// shows u, the values come from the server in double (POST
// /api/solve/probes, for a static solution too); for a difference they are
// interpolated in the page from the float32 values drawn, and the table
// says so; for a field of the Assemble step from its values (double, as
// the server sent them). The list is the chart's legend. Under them, the
// line profile: the shown field along the line drawn with the profile
// tool (display-only, from the shown values).

import { api } from "./api.js";
import { plainError } from "./pure/requests.js";
import { lineChart } from "./charts.js";
import { formatNumber } from "./pure/format.js";
import { interpolateAt, locatePoint, meshGrid, sampleLine } from "./pure/mesh_math.js";
import { fill, h } from "./dom.js";
import { freshResult, lastResult, notify, requestTicket, setProbes, setProfile, setViewOption, store } from "./state.js";
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

// The value shown for probe i: from the server's series at the shown
// snapshot (source "server"), interpolated in the page ("page"), or
// pending.
function probeValue(i, found, field, server, frame) {
  if (server) {
    if (!server.data) return { text: server.failure ? "–" : "…" };
    const v = server.data.probes[i]?.values?.[frame];
    return { text: v === null || v === undefined ? "–" : null, value: v };
  }
  return { value: field ? interpolateAt(found, field.values) : NaN };
}

function probeRow(probe, found, shown, digits) {
  const value = shown.value;
  return h(
    "tr",
    {},
    h("td", {}, h("span", { class: "probe-key" }, h("span", { class: `probe-dot s${probe.slot + 1}`, "aria-hidden": "true" }), probeName(probe))),
    h("td", { class: "num" }, formatNumber(probe.x)),
    h("td", { class: "num" }, formatNumber(probe.y)),
    h("td", { class: "num" }, !found ? "outside" : (shown.text ?? formatNumber(value, digits))),
    h(
      "td",
      { class: "num" },
      h("button", { type: "button", class: "icon", "aria-label": `Remove probe ${probeName(probe)}`, title: "Remove", onclick: () => remove(probe) }, "✕"),
    ),
  );
}

let chartState = null;

// context: {mesh (fresh mesh view or null), field (the shown field or
// null), display (pure/display.js, or null when the picture is not
// coloured by a field), solution (a time solution or null), frame,
// digits}.
export function updateProbes(root, { mesh, field, display, solution, frame, digits }) {
  root.hidden = !mesh;
  if (!mesh) return;
  updateProfile(root.querySelector(".profile-chart"), { mesh, field });
  const probes = store.probes;
  const list = root.querySelector(".probe-list");
  const grid = gridOf(mesh);
  const located = probes.map((p) => locatePoint(mesh, grid, p.x, p.y));
  // u of a solution (fresh or outdated): the server's values in double.
  const run = display?.kind === "solution" && field ? (freshResult("solve") ?? lastResult("solve")) : null;
  const server = run && probes.length ? serverSeries(run, probes) : null;
  if (probes.length === 0) {
    fill(list, h("p", { class: "note" }, "Click or tap the picture to pin a probe (up to 4); a click on the boundary selects its region."));
  } else {
    const header = h(
      "tr",
      {},
      h("th", {}, "Probe"),
      h("th", { class: "num" }, "x"),
      h("th", { class: "num" }, "y"),
      h("th", { class: "num" }, field?.name ?? "value"),
      h("th", {}, h("button", { type: "button", class: "link", onclick: () => setProbes([]) }, "Remove all")),
    );
    const source = !field
      ? null
      : server
        ? `${field.name} in double from the server, at the shown snapshot.`
        : display?.kind === "difference"
          ? `${field.name} interpolated in the page from the float32 values drawn (display precision).`
          : `${field.name} interpolated in the page from the field's values.`;
    fill(
      list,
      h("table", { class: "data probes-table" }, h("thead", {}, header), h("tbody", {}, probes.map((p, i) => probeRow(p, located[i], probeValue(i, located[i], field, server, frame), digits)))),
      source ? h("p", { class: "note probe-source" }, source) : null,
      server?.failure ? failureLine(server.failure) : null,
    );
  }
  updateChart(root.querySelector(".probe-chart"), { probes, solution, frame });
}

// A failed probe request in plain words, with Retry.
function failureLine(error) {
  const plain = plainError(error, "u at the probes");
  return h(
    "div",
    { class: "inline-failure", role: "status" },
    h("span", {}, plain.text),
    plain.action === "retry" ? h("button", { type: "button", class: "small", onclick: retryProbes }, "Retry") : null,
  );
}

// u at the probes from the server, for one run, snapshot count and set
// of probes (the key); asked again when any of them changes. A failed
// request is kept (`failure`) until Retry, so a failing server is not
// asked on every redraw.
const series = { key: null, data: null, asking: null, failure: null, failedKey: null };

function seriesKey(solution, probes) {
  return `${solution.job}|${solution.values.length}|${probes.map((p) => `${p.x},${p.y}`).join(";")}`;
}

function askSeries(solution, probes, key) {
  if (series.asking === key || series.failedKey === key) return;
  series.asking = key;
  const ticket = requestTicket("probes", () => series.asking);
  api
    .probes(
      solution.job,
      probes.map((p) => [p.x, p.y]),
    )
    .then((data) => {
      if (!ticket.holds()) return;
      series.key = key;
      series.data = data;
      series.failure = null;
      series.failedKey = null;
      notify("frame");
    })
    .catch((error) => {
      if (!ticket.holds()) return;
      series.failure = error;
      series.failedKey = key;
      notify("frame");
    })
    .finally(() => {
      if (series.asking === key) series.asking = null;
    });
}

function retryProbes() {
  series.failure = null;
  series.failedKey = null;
  notify("frame");
}

// The server's values for the table: {data (for these probes, or null
// while they come), failure}.
function serverSeries(run, probes) {
  const key = seriesKey(run, probes);
  if (series.key !== key) askSeries(run, probes, key);
  const failure = series.failedKey === key ? series.failure : null;
  return { data: series.key === key ? series.data : null, failure };
}

function updateChart(slot, { probes, solution, frame }) {
  if (!solution || probes.length === 0) {
    chartState = null;
    fill(slot);
    slot.hidden = true;
    return;
  }
  slot.hidden = false;
  const key = seriesKey(solution, probes);
  if (series.key !== key) askSeries(solution, probes, key);
  // Until the new series arrive, the last ones for the same probes stay.
  const data = series.key === key || (chartState && chartState.probes === probes) ? series.data : null;
  if (series.failedKey === key && (!data || data.probes.length !== probes.length)) {
    chartState = null;
    fill(slot, h("div", { class: "chart-title" }, "u(t) at the probes"), failureLine(series.failure));
    return;
  }
  if (!data || data.probes.length !== probes.length) {
    if (!chartState) fill(slot, h("div", { class: "chart-title" }, "u(t) at the probes"), h("p", { class: "note" }, "Loading u(t)…"));
    else chartState.chart.setCurrent(frame);
    return;
  }
  const width = Math.max(200, Math.round(slot.clientWidth || slot.parentElement?.clientWidth || 480));
  const same = chartState && chartState.data === data && chartState.width === width;
  if (same) {
    chartState.chart.setCurrent(frame);
    return;
  }
  const lines = probes.map((p, i) => ({ name: probeName(p), cls: `s${p.slot + 1}`, values: Float64Array.from(data.probes[i].values, (v) => (v === null ? NaN : v)) }));
  const chart = lineChart({
    times: data.times,
    series: lines,
    current: frame,
    width,
    onPick: (k) => showFrame(k),
    label: `u over time at ${probes.map(probeName).join(", ")}`,
    csvName: "probes-u-t.csv",
  });
  chartState = { probes, data, width, chart };
  fill(slot, h("div", { class: "chart-title" }, "u(t) at the probes"), chart);
}

// The chart is drawn for the area's width: after a resize, again.
export function forgetChartSize() {
  if (chartState) chartState.width = -1;
  if (profileState) profileState.width = -1;
}

// --- the line profile ---------------------------------------------------------

let profileState = null;

const PROFILE_SAMPLES = 200;

// The shown field along the profile's line, drawn again when the line,
// the mesh, the field's values or the width change.
function updateProfile(slot, { mesh, field }) {
  const line = store.profile;
  if (!line) {
    profileState = null;
    fill(slot);
    slot.hidden = true;
    return;
  }
  slot.hidden = false;
  const width = Math.max(200, Math.round(slot.clientWidth || slot.parentElement?.clientWidth || 480));
  const values = field?.values ?? null;
  if (profileState && profileState.line === line && profileState.mesh === mesh && profileState.values === values && profileState.width === width) return;
  profileState = { line, mesh, values, width };
  const remove = h("button", { type: "button", class: "link", onclick: () => setProfile(null) }, "Remove line");
  const length = Math.hypot(line.to[0] - line.from[0], line.to[1] - line.from[1]);
  const title = h(
    "div",
    { class: "chart-head" },
    h("span", { class: "chart-title" }, field ? `${field.label} along A → B (length ${formatNumber(length)})` : "Line profile"),
    h("span", { class: "spacer" }),
    remove,
  );
  if (!values) {
    fill(slot, title, h("p", { class: "note" }, "Colour the picture by a field to see it along the line."));
    return;
  }
  const sampled = sampleLine(mesh, gridOf(mesh), values, line.from, line.to, PROFILE_SAMPLES);
  const chart = lineChart({
    times: Array.from(sampled.distance),
    series: [{ name: field.name ?? "value", cls: "ink", values: sampled.values }],
    current: null,
    width,
    xName: "distance from A",
    label: `${field.label} along the profile line`,
    csvName: "line-profile.csv",
  });
  fill(slot, title, chart, h("p", { class: "note" }, "Distance from A along the line; gaps are outside the mesh."));
}

// Drawing a line ends the profile tool (the picture pans again).
export function profileDrawn(line) {
  setProfile(line);
  setViewOption("tool", "pan");
}
