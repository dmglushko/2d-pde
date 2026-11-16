// A small line chart over the snapshot times (SVG): the u(t) of the
// probes and the error per snapshot. 2 px lines on one y axis, hairline
// grid, round ticks, a marker at the shown time, a crosshair with a
// tooltip under the pointer; a click picks that snapshot. Colours come
// from the tokens through classes (style.css, .chart .s1 to .s4).

import { formatNumber, niceTicks } from "./core.js";
import { h } from "./dom.js";

const SVG = "http://www.w3.org/2000/svg";

function svg(tag, attributes = {}, ...children) {
  const element = document.createElementNS(SVG, tag);
  for (const [name, value] of Object.entries(attributes)) if (value !== null && value !== undefined) element.setAttribute(name, String(value));
  for (const child of children.flat()) if (child) element.append(child);
  return element;
}

// series: [{name, cls ("s1".."s4"), values (one per time)}]. `width` in
// CSS pixels; `onPick(index)` when the chart is clicked; `label` names
// the chart for screen readers. Returns the element; .setCurrent(index)
// moves the marker without drawing the chart again.
export function lineChart({ times, series, current = 0, width = 480, height = 128, onPick = null, label = "chart", format = formatNumber }) {
  const margin = { left: 52, right: series.length > 1 ? 28 : 12, top: 8, bottom: 22 };
  const plotW = Math.max(40, width - margin.left - margin.right);
  const plotH = Math.max(30, height - margin.top - margin.bottom);
  let low = Infinity;
  let high = -Infinity;
  for (const s of series)
    for (const v of s.values)
      if (Number.isFinite(v)) {
        low = Math.min(low, v);
        high = Math.max(high, v);
      }
  const wrap = h("div", { class: "chart", role: "img", "aria-label": label });
  if (low === Infinity || times.length === 0) {
    wrap.append(h("p", { class: "note" }, "No values here."));
    wrap.setCurrent = () => {};
    return wrap;
  }
  const yTicks = niceTicks(low, high, 3);
  const y0 = yTicks[0];
  const y1 = yTicks.at(-1);
  const t0 = times[0];
  const t1 = times.at(-1) > t0 ? times.at(-1) : t0 + 1;
  const x = (t) => margin.left + ((t - t0) / (t1 - t0)) * plotW;
  const y = (v) => margin.top + (1 - (v - y0) / (y1 - y0)) * plotH;

  const grid = yTicks.map((v) =>
    svg(
      "g",
      {},
      svg("line", { class: "grid", x1: margin.left, x2: margin.left + plotW, y1: y(v), y2: y(v) }),
      svg("text", { class: "tick", x: margin.left - 6, y: y(v) + 3.5, "text-anchor": "end" }, document.createTextNode(format(v))),
    ),
  );
  const xTicks = niceTicks(t0, t1, Math.max(2, Math.floor(plotW / 90))).filter((t) => t >= t0 - 1e-12 && t <= t1 + 1e-12);
  const xLabels = xTicks.map((t) => svg("text", { class: "tick", x: x(t), y: height - 6, "text-anchor": "middle" }, document.createTextNode(format(t))));
  const axis = svg("line", { class: "axis", x1: margin.left, x2: margin.left + plotW, y1: margin.top + plotH, y2: margin.top + plotH });

  const lines = series.map((s) => {
    let d = "";
    let pen = false;
    s.values.forEach((v, k) => {
      if (!Number.isFinite(v)) {
        pen = false;
        return;
      }
      d += `${pen ? "L" : "M"}${x(times[k]).toFixed(1)},${y(v).toFixed(1)}`;
      pen = true;
    });
    return svg("path", { class: `line ${s.cls}`, d });
  });

  // Direct labels at the line ends (several series only), left out where
  // two would touch: the legend (the probe list) names them anyway.
  const ends = [];
  if (series.length > 1)
    for (const s of series) {
      const k = s.values.findLastIndex((v) => Number.isFinite(v));
      if (k < 0) continue;
      const py = y(s.values[k]);
      if (ends.some((e) => Math.abs(e - py) < 11)) continue;
      ends.push(py);
      lines.push(svg("text", { class: "end-label", x: x(times[k]) + 5, y: py + 3.5 }, document.createTextNode(s.name)));
    }

  const marker = svg("line", { class: "marker", y1: margin.top, y2: margin.top + plotH });
  const markerDots = series.map((s) => svg("circle", { class: `dot ${s.cls}`, r: 4 }));
  const cross = svg("line", { class: "cross", y1: margin.top, y2: margin.top + plotH, visibility: "hidden" });
  const hit = svg("rect", { class: "hit", x: margin.left, y: margin.top, width: plotW, height: plotH });
  const chart = svg("svg", { width, height, viewBox: `0 0 ${width} ${height}` }, grid, axis, xLabels, lines, marker, markerDots, cross, hit);
  const tooltip = h("div", { class: "chart-tip", hidden: true });
  wrap.append(chart, tooltip);

  wrap.setCurrent = (index) => {
    const k = Math.max(0, Math.min(times.length - 1, index));
    marker.setAttribute("x1", x(times[k]));
    marker.setAttribute("x2", x(times[k]));
    series.forEach((s, i) => {
      const v = s.values[k];
      markerDots[i].setAttribute("visibility", Number.isFinite(v) ? "visible" : "hidden");
      if (Number.isFinite(v)) {
        markerDots[i].setAttribute("cx", x(times[k]));
        markerDots[i].setAttribute("cy", y(v));
      }
    });
  };
  wrap.setCurrent(current);

  // The snapshot nearest to the pointer.
  const nearest = (event) => {
    const rect = chart.getBoundingClientRect();
    const t = t0 + ((event.clientX - rect.left - margin.left) / plotW) * (t1 - t0);
    let best = 0;
    for (let k = 1; k < times.length; k++) if (Math.abs(times[k] - t) < Math.abs(times[best] - t)) best = k;
    return best;
  };
  hit.addEventListener("pointermove", (event) => {
    const k = nearest(event);
    cross.setAttribute("x1", x(times[k]));
    cross.setAttribute("x2", x(times[k]));
    cross.setAttribute("visibility", "visible");
    tooltip.replaceChildren(
      h("div", { class: "tip-head" }, `t = ${formatNumber(times[k], 5)}`),
      ...series.map((s) => h("div", { class: "tip-row" }, h("span", { class: `key-line ${s.cls}` }), s.name, h("span", { class: "num" }, formatNumber(s.values[k])))),
    );
    tooltip.hidden = false;
    const left = x(times[k]) + 10;
    tooltip.style.left = `${Math.min(left, width - tooltip.offsetWidth - 4)}px`;
  });
  hit.addEventListener("pointerleave", () => {
    cross.setAttribute("visibility", "hidden");
    tooltip.hidden = true;
  });
  if (onPick) hit.addEventListener("click", (event) => onPick(nearest(event)));
  return wrap;
}
