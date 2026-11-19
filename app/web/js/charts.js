// A small line chart (SVG): the u(t) of the probes, the error per
// snapshot, the line profile and the refinement study. 2 px lines on one
// y axis, hairline grid, round ticks, a marker at the shown index, a
// crosshair with a readout under the pointer; a click picks that index.
// Without a mouse: the chart takes the focus (Tab), the arrow keys move
// the readout point by point (Home, End; Enter picks it), and the readout
// is announced. "Download data (CSV)" saves the numbers drawn. Colours
// come from the tokens through classes (css/picture.css, .chart .s1 to
// .s4, .ref for reference lines).

import { saveBlob } from "./api.js";
import { formatNumber, logTicks, niceTicks, seriesCsv } from "./pure/format.js";
import { h } from "./dom.js";

const SVG = "http://www.w3.org/2000/svg";

function svg(tag, attributes = {}, ...children) {
  const element = document.createElementNS(SVG, tag);
  for (const [name, value] of Object.entries(attributes)) if (value !== null && value !== undefined) element.setAttribute(name, String(value));
  for (const child of children.flat()) if (child) element.append(child);
  return element;
}

// series: [{name, cls ("s1".."s4", "ink" for a single line that is no
// series of a set, "ref" for a dashed reference line), values (one per
// x)}]. `times` are the x values (`xName` names them in the readout and
// the CSV: "t", "distance from A", "h"). `width` in CSS pixels;
// `onPick(index)` when the chart is clicked (or Enter is pressed);
// `label` names the chart for screen readers; `current` is the index
// marked (null: no marker); `log`: both axes logarithmic (values must be
// positive); `csvName`: the file name of the data download (null: none);
// `endLabels`: name the lines at their ends (several series; off where a
// key beside the chart names them and the ends would collide).
// `key`: a key above the chart naming every series, each name a toggle
// that hides or shows its line (the axes then fit the lines shown; the CSV
// keeps every series); what is hidden is remembered per `keyId` (default
// `csvName`) while the page is open.
// Returns the element; .setCurrent(index) moves the marker without
// drawing the chart again.
export function lineChart(options) {
  if (!options.key) return plainChart(options);
  const id = options.keyId ?? options.csvName ?? options.label;
  const hidden = hiddenSeries.get(id) ?? new Set();
  hiddenSeries.set(id, hidden);
  const outer = h("div", { class: "keyed-chart" });
  const keyRow = h("div", { class: "chart-key", role: "group", "aria-label": "Lines shown" });
  let inner = null;
  let current = options.current;
  const draw = () => {
    const shown = options.series.filter((s) => !hidden.has(s.name));
    keyRow.replaceChildren(
      ...options.series.map((s) =>
        h(
          "button",
          {
            type: "button",
            class: "item key-toggle",
            "aria-pressed": String(!hidden.has(s.name)),
            title: hidden.has(s.name) ? `Show ${s.name}` : `Hide ${s.name}`,
            onclick: () => {
              if (hidden.has(s.name)) hidden.delete(s.name);
              else hidden.add(s.name);
              draw();
            },
          },
          h("span", { class: `key-line ${s.cls}`, "aria-hidden": "true" }),
          s.name,
        ),
      ),
    );
    const next = plainChart({ ...options, current, series: shown, csvSeries: options.series, emptyText: "Every line is hidden: click a name above to show it." });
    if (inner) inner.replaceWith(next);
    else outer.append(keyRow, next);
    inner = next;
  };
  draw();
  outer.setCurrent = (index) => {
    current = index;
    inner.setCurrent(index);
  };
  return outer;
}

// The series hidden by a chart's key, per keyId.
const hiddenSeries = new Map();

function plainChart({
  times,
  series,
  current = 0,
  width = 480,
  height = 128,
  onPick = null,
  label = "chart",
  format = formatNumber,
  xName = "t",
  log = false,
  csvName = "chart.csv",
  endLabels = true,
  csvSeries = series,
  emptyText = "No values here.",
}) {
  const margin = { left: 52, right: series.length > 1 ? 28 : 12, top: 8, bottom: 22 };
  const plotW = Math.max(40, width - margin.left - margin.right);
  const plotH = Math.max(30, height - margin.top - margin.bottom);
  const usable = (v) => Number.isFinite(v) && (!log || v > 0);
  let low = Infinity;
  let high = -Infinity;
  for (const s of series)
    for (const v of s.values)
      if (usable(v)) {
        low = Math.min(low, v);
        high = Math.max(high, v);
      }
  const wrap = h("div", { class: "chart" });
  if (low === Infinity || times.length === 0) {
    wrap.append(h("p", { class: "note" }, emptyText));
    wrap.setCurrent = () => {};
    return wrap;
  }
  const f = log ? Math.log10 : (v) => v;
  const yTicks = log ? logTicks(low, high) : niceTicks(low, high, 3);
  const y0 = f(Math.min(yTicks[0], low));
  const y1 = f(Math.max(yTicks.at(-1), high));
  const xs = Array.from(times);
  const xMin = Math.min(...xs.filter(usable));
  const xMax = Math.max(...xs.filter(usable));
  const t0 = f(xMin);
  const t1 = f(xMax) > t0 ? f(xMax) : t0 + 1;
  const x = (t) => margin.left + ((f(t) - t0) / (t1 - t0)) * plotW;
  const y = (v) => margin.top + (1 - (f(v) - y0) / (y1 - y0 || 1)) * plotH;

  const grid = yTicks
    .filter((v) => f(v) >= y0 - 1e-9 && f(v) <= y1 + 1e-9)
    .map((v) =>
      svg(
        "g",
        {},
        svg("line", { class: "grid", x1: margin.left, x2: margin.left + plotW, y1: y(v), y2: y(v) }),
        svg("text", { class: "tick", x: margin.left - 6, y: y(v) + 3.5, "text-anchor": "end" }, document.createTextNode(format(v))),
      ),
    );
  const xTicks = (log ? logTicks(xMin, xMax) : niceTicks(xMin, xMax, Math.max(2, Math.floor(plotW / 90)))).filter(
    (t) => f(t) >= t0 - 1e-9 && f(t) <= t1 + 1e-9,
  );
  const xLabels = xTicks.map((t) => svg("text", { class: "tick", x: x(t), y: height - 6, "text-anchor": "middle" }, document.createTextNode(format(t))));
  const axis = svg("line", { class: "axis", x1: margin.left, x2: margin.left + plotW, y1: margin.top + plotH, y2: margin.top + plotH });

  const lines = series.map((s) => {
    let d = "";
    let pen = false;
    s.values.forEach((v, k) => {
      if (!usable(v) || !usable(xs[k])) {
        pen = false;
        return;
      }
      d += `${pen ? "L" : "M"}${x(xs[k]).toFixed(1)},${y(v).toFixed(1)}`;
      pen = true;
    });
    return svg("path", { class: `line ${s.cls}`, d });
  });

  // Direct labels at the line ends (several series only), left out where
  // two would touch: the legend names them anyway.
  const ends = [];
  if (series.length > 1 && endLabels)
    for (const s of series) {
      const k = s.values.findLastIndex((v, i) => usable(v) && usable(xs[i]));
      if (k < 0) continue;
      const py = y(s.values[k]);
      if (ends.some((e) => Math.abs(e - py) < 11)) continue;
      ends.push(py);
      lines.push(svg("text", { class: "end-label", x: x(xs[k]) + 5, y: py + 3.5 }, document.createTextNode(s.name)));
    }

  const marker = svg("line", { class: "marker", y1: margin.top, y2: margin.top + plotH });
  const markerDots = series.map((s) => svg("circle", { class: `dot ${s.cls}`, r: 4 }));
  const cross = svg("line", { class: "cross", y1: margin.top, y2: margin.top + plotH, visibility: "hidden" });
  const hit = svg("rect", { class: "hit", x: margin.left, y: margin.top, width: plotW, height: plotH });
  const chart = svg("svg", { width, height, viewBox: `0 0 ${width} ${height}`, "aria-hidden": "true" }, grid, axis, xLabels, lines, marker, markerDots, cross, hit);
  const tooltip = h("div", { class: "chart-tip", hidden: true });
  // The readout said to a screen reader as the arrow keys move it.
  const spoken = h("span", { class: "visually-hidden", "aria-live": "polite" });
  const plot = h(
    "div",
    {
      class: "chart-plot",
      tabindex: "0",
      role: "group",
      "aria-roledescription": "chart",
      "aria-label": `${label}. Arrow keys read the values point by point${onPick ? "; Enter shows that point" : ""}.`,
    },
    chart,
    tooltip,
    spoken,
  );
  wrap.append(plot);

  wrap.setCurrent = (index) => {
    const hide = index === null || index === undefined;
    marker.setAttribute("visibility", hide ? "hidden" : "visible");
    if (hide) {
      for (const dot of markerDots) dot.setAttribute("visibility", "hidden");
      return;
    }
    const k = Math.max(0, Math.min(xs.length - 1, index));
    marker.setAttribute("x1", x(xs[k]));
    marker.setAttribute("x2", x(xs[k]));
    series.forEach((s, i) => {
      const v = s.values[k];
      markerDots[i].setAttribute("visibility", usable(v) ? "visible" : "hidden");
      if (usable(v)) {
        markerDots[i].setAttribute("cx", x(xs[k]));
        markerDots[i].setAttribute("cy", y(v));
      }
    });
  };
  wrap.setCurrent(current);

  // The readout at index k: the crosshair and the values of every series.
  let reading = current ?? 0;
  const readout = (k, speak = false) => {
    reading = k;
    cross.setAttribute("x1", x(xs[k]));
    cross.setAttribute("x2", x(xs[k]));
    cross.setAttribute("visibility", "visible");
    const head = `${xName} = ${formatNumber(xs[k], 5)}`;
    tooltip.replaceChildren(
      h("div", { class: "tip-head" }, head),
      ...series.map((s) => h("div", { class: "tip-row" }, h("span", { class: `key-line ${s.cls}` }), s.name, h("span", { class: "num" }, formatNumber(s.values[k])))),
    );
    tooltip.hidden = false;
    const left = x(xs[k]) + 10;
    tooltip.style.left = `${Math.max(0, Math.min(left, width - tooltip.offsetWidth - 4))}px`;
    if (speak) spoken.textContent = `${head}: ${series.map((s) => `${s.name} ${formatNumber(s.values[k])}`).join(", ")}`;
  };
  const hideReadout = () => {
    cross.setAttribute("visibility", "hidden");
    tooltip.hidden = true;
  };

  // The point nearest to the pointer.
  const nearest = (event) => {
    const rect = chart.getBoundingClientRect();
    const px = event.clientX - rect.left;
    let best = 0;
    for (let k = 1; k < xs.length; k++) if (Math.abs(x(xs[k]) - px) < Math.abs(x(xs[best]) - px)) best = k;
    return best;
  };
  hit.addEventListener("pointermove", (event) => readout(nearest(event)));
  hit.addEventListener("pointerleave", () => {
    if (document.activeElement !== plot) hideReadout();
  });
  if (onPick) hit.addEventListener("click", (event) => onPick(nearest(event)));
  plot.addEventListener("focus", () => readout(Math.max(0, Math.min(xs.length - 1, reading)), true));
  plot.addEventListener("blur", hideReadout);
  plot.addEventListener("keydown", (event) => {
    const last = xs.length - 1;
    const step = event.shiftKey ? Math.max(1, Math.round(xs.length / 10)) : 1;
    const next = { ArrowRight: reading + step, ArrowUp: reading + step, ArrowLeft: reading - step, ArrowDown: reading - step, Home: 0, End: last }[event.key];
    if (next !== undefined) {
      event.preventDefault();
      readout(Math.max(0, Math.min(last, next)), true);
    } else if (event.key === "Enter" && onPick) {
      event.preventDefault();
      onPick(reading);
    }
  });

  if (csvName) {
    const download = h(
      "button",
      {
        type: "button",
        class: "link chart-download",
        title: "The numbers drawn in this chart, one row per point, in full precision",
        onclick: () => saveBlob(new Blob([seriesCsv(xName, xs, csvSeries)], { type: "text/csv" }), csvName),
      },
      "Download data (CSV)",
    );
    wrap.append(h("div", { class: "chart-actions" }, download));
  }
  return wrap;
}
