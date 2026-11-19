// The legend of the picture (regions, the colour bar named after the
// field shown, with its range lock; the Dirichlet nodes) and the saved
// picture, which draws the same legend under the canvas.

import { regionToken } from "../pure/color.js";
import { colorbarTicks, formatNumber, getDefaultDigits, numberText, parseNumberText } from "../pure/format.js";
import { h, regionSwatch, steadyText } from "../dom.js";
import { QUALITY_MAX_DEGREES } from "./layers.js";
import { colorTables, tableOf, token } from "./tokens.js";

// "|∇u|", or "|∇(u − exact)|" for a name that is not one word.
export function gradientName(name) {
  return /^[A-Za-z_]\w*$/.test(name) ? `|∇${name}|` : `|∇(${name})|`;
}

// What the legend shows, as data: the region names, the colour bar
// ({quantity, label, table, low, high, lowValue, highValue, locked,
// reversed, name, when, range} or, for a field that has one value,
// {label, constant}) and whether the Dirichlet nodes are drawn. `name`
// and `when` are the label's quantity and time (shown apart, so the bar
// does not move with the time's width); `range` is the range choice of a
// run in time ("run" or "snapshot"), or null where it is not offered;
// `whenSizer` the widest time of the run (the room the time keeps).
export function legendData(v) {
  const { geometry, mesh, meshStale, colorBy, field, layers = {}, selectedRegion = null, legendNote = null, rangeChoice = null, whenSizer = "" } = v.data;
  const current = mesh && !meshStale;
  const regions = current ? mesh.region_names : (geometry?.regions ?? []).map((r) => r.name);
  let bar = null;
  if (current && colorBy === "quality") bar = { label: "smallest angle", table: colorTables().quality, low: "0°", high: `${QUALITY_MAX_DEGREES}°+`, reversed: true };
  const scale = v.shownScale;
  const fieldShown = mesh && field && (current || field.outdated);
  if (fieldShown && scale && (colorBy === "field" || colorBy === "gradient" || (colorBy === "indicator" && v.indicatorLogs))) {
    const outdated = field.outdated ? " (outdated)" : "";
    const name = colorBy === "gradient" ? gradientName(field.name) : colorBy === "indicator" ? "log₁₀ η (error indicator)" : field.name;
    const when = colorBy === "indicator" ? outdated.trim() : [field.when, outdated.trim()].filter(Boolean).join(" ");
    const label = [name, when].filter(Boolean).join(" ");
    const range = colorBy === "field" && !scale.locked ? rangeChoice : null;
    if (scale.kind === "constant") bar = { label, name, when, range, constant: formatNumber(scale.min, getDefaultDigits() + 2) };
    else if (scale.kind !== "empty")
      bar = {
        quantity: colorBy,
        label,
        name,
        when,
        whenSizer: colorBy === "indicator" ? "" : whenSizer,
        range,
        table: tableOf(scale),
        low: formatNumber(scale.low),
        high: formatNumber(scale.high),
        lowValue: scale.low,
        highValue: scale.high,
        locked: Boolean(scale.locked),
        reversed: false,
        outdated: Boolean(field.outdated),
        ticks: colorbarTicks(scale.low, scale.high, 4),
      };
  }
  return { regions, bar, dirichlet: Boolean(layers.dirichlet && v.data.dirichlet && current), selectedRegion, note: legendNote };
}

// The colour bar's width (narrower at phone width) and the room beside
// it for the end ticks' text (CSS pixels; css/picture.css and
// css/layout.css, .bar-scale canvas.bar, use the same widths).
const BAR_PAD = 18;
const barWidth = () => (globalThis.matchMedia?.("(max-width: 860px)").matches ? 128 : 200);

function barCanvas(table, reversed) {
  const canvas = h("canvas", { class: "bar", width: 256, height: 1, "aria-hidden": "true" });
  const context = canvas.getContext("2d");
  const image = context.createImageData(256, 1);
  for (let i = 0; i < 256; i++) {
    const k = reversed ? 255 - i : i;
    image.data.set([table[3 * k], table[3 * k + 1], table[3 * k + 2], 255], 4 * i);
  }
  context.putImageData(image, 0, 0);
  return canvas;
}

// The ends of a locked range, editable: a new range applies when both
// ends are numbers and low < high (on Enter or when a field is left).
function lockInputs(bar, onLock) {
  const input = (value, label) =>
    h("input", { type: "text", inputmode: "decimal", autocomplete: "off", spellcheck: false, class: "range-input num", value: numberText(value), "aria-label": label });
  const low = input(bar.lowValue, `Lowest value of the ${bar.label} colour range`);
  const high = input(bar.highValue, `Highest value of the ${bar.label} colour range`);
  const apply = () => {
    const a = parseNumberText(low.value);
    const b = parseNumberText(high.value);
    const ok = a.kind === "number" && b.kind === "number" && a.value < b.value;
    for (const [element, parsed] of [
      [low, a],
      [high, b],
    ]) {
      const bad = !ok && (parsed.kind !== "number" || (a.kind === "number" && b.kind === "number"));
      element.classList.toggle("local-invalid", bad);
      element.setAttribute("aria-invalid", String(bad));
      element.title = bad ? (parsed.kind === "number" ? "the lowest value must be below the highest" : (parsed.reason ?? "a value is needed")) : "";
    }
    if (ok) onLock(bar.quantity, { low: a.value, high: b.value });
  };
  for (const element of [low, high]) {
    element.addEventListener("change", apply);
    element.addEventListener("keydown", (event) => {
      if (event.key === "Enter") apply();
    });
  }
  return [low, high];
}

// The bar with round ticks under it (pure/format.js, colorbarTicks); a
// bar without ticks (the smallest angle) names its ends beside it.
function scaleBar(bar) {
  const canvas = barCanvas(bar.table, bar.reversed);
  if (!bar.ticks?.length) return [h("span", { class: "num" }, bar.low), canvas, h("span", { class: "num" }, bar.high)];
  // The ticks are an SVG under the bar (positions as attributes: the page
  // has no inline styles), BAR_PAD wider on each side for the end labels.
  const BAR_WIDTH = barWidth();
  const width = BAR_WIDTH + 2 * BAR_PAD;
  const ns = "http://www.w3.org/2000/svg";
  const ticks = document.createElementNS(ns, "svg");
  for (const [name, value] of Object.entries({ class: "ticks", width, height: 16, viewBox: `0 0 ${width} 16`, "aria-hidden": "true" })) ticks.setAttribute(name, String(value));
  for (const t of bar.ticks) {
    const x = BAR_PAD + t.position * BAR_WIDTH;
    const mark = document.createElementNS(ns, "line");
    for (const [name, value] of Object.entries({ x1: x, x2: x, y1: 0, y2: 3 })) mark.setAttribute(name, String(value));
    const label = document.createElementNS(ns, "text");
    for (const [name, value] of Object.entries({ x, y: 14, "text-anchor": "middle" })) label.setAttribute(name, String(value));
    label.textContent = t.text;
    ticks.append(mark, label);
  }
  return [
    h(
      "span",
      { class: "bar-scale", role: "img", "aria-label": `Colour scale from ${bar.low} to ${bar.high}`, title: `From ${bar.low} to ${bar.high}` },
      canvas,
      ticks,
    ),
  ];
}

// The bar's label: the quantity, then its time in a span of its own
// (bar-when), which main.js keeps from shrinking while a run plays.
function barLabel(bar) {
  if (bar.name === undefined) return h("span", { class: "bar-label" }, bar.label);
  return h("span", { class: "bar-label" }, bar.name, bar.when ? [" ", steadyText(bar.when, bar.whenSizer ?? "", "bar-when num")] : null);
}

// Whole run or this snapshot: the colour range of a run in time.
function rangeSelect(bar, onRange) {
  if (!bar.range || !onRange) return null;
  return h(
    "select",
    {
      class: "range-choice",
      "aria-label": "Colour range",
      title: "Whole run: one range for every snapshot, so a colour means the same value at every time. This snapshot: each snapshot's own smallest and largest value, so small values late in a run are seen.",
      onchange: (event) => onRange(event.target.value),
    },
    h("option", { value: "run", selected: bar.range === "run" }, "whole run"),
    h("option", { value: "snapshot", selected: bar.range === "snapshot" }, "this snapshot"),
  );
}

function colorbar(bar, onLock, onRange) {
  const label = barLabel(bar);
  if (bar.constant !== undefined) return h("span", { class: "colorbar" }, label, h("span", { class: "num" }, `constant: ${bar.constant}`), rangeSelect(bar, onRange));
  const canvas = scaleBar(bar);
  if (!bar.quantity || !onLock) return h("span", { class: "colorbar" }, label, canvas);
  if (bar.locked) {
    const [low, high] = lockInputs(bar, onLock);
    const unlock = h("button", { type: "button", class: "small", title: "Colour each field by its own range again", onclick: () => onLock(bar.quantity, null) }, "Unlock");
    return h("span", { class: "colorbar locked" }, label, h("span", { class: "chip" }, "locked"), low, canvas, high, unlock);
  }
  const lock = h(
    "button",
    {
      type: "button",
      class: "small",
      title: "Keep this colour range for every snapshot and run shown, so two runs can be compared by eye; its ends can then be edited",
      onclick: () => onLock(bar.quantity, { low: bar.lowValue, high: bar.highValue }),
    },
    "Lock range",
  );
  const range = rangeSelect(bar, onRange);
  return h("span", { class: "colorbar" }, label, canvas, range ? h("span", { class: "range-pick" }, h("span", { class: "range-word", "aria-hidden": "true" }, "range"), range) : null, lock);
}

// The legend's items. A region is a button: it selects the region (as a
// click on its boundary does) and opens its condition (onRegion).
export function legendNodes({ regions, bar, dirichlet, selectedRegion, note }, onLock, onRegion, onRange) {
  const items = regions.map((name, i) =>
    onRegion
      ? h(
          "button",
          { type: "button", class: "item region-item", "aria-pressed": String(name === selectedRegion), title: `Select ${name} and edit its boundary condition`, onclick: () => onRegion(name) },
          regionSwatch(i),
          name,
        )
      : h("span", { class: "item" }, regionSwatch(i), name),
  );
  if (bar) items.push(colorbar(bar, onLock, onRange));
  if (dirichlet) items.push(h("span", { class: "item" }, h("span", { class: "swatch node", "aria-hidden": "true" }), "Dirichlet node"));
  if (note) items.push(h("span", { class: "legend-note" }, note));
  return items;
}

// The picture as it is shown, with its legend underneath, as a PNG blob.
export function picture(v) {
  const ratio = v.ratio;
  const width = v.canvas.width;
  const { regions, bar, dirichlet } = legendData(v);
  const line = Math.round(22 * ratio);
  const pad = Math.round(12 * ratio);
  const out = document.createElement("canvas");
  out.width = width;
  // Legend rows are laid out first to know the height.
  const ctx = out.getContext("2d");
  const font = `${Math.round(13 * ratio)}px ${token("--font")}`;
  ctx.font = font;
  const entries = regions.map((name, i) => ({ name, color: token(regionToken(i)), round: false }));
  if (dirichlet) entries.push({ name: "Dirichlet node", color: token("--ink"), round: true });
  const rows = [[]];
  let x = pad;
  for (const entry of entries) {
    const w = 18 * ratio + ctx.measureText(entry.name).width + 16 * ratio;
    if (x + w > width - pad && rows.at(-1).length) {
      rows.push([]);
      x = pad;
    }
    rows.at(-1).push({ ...entry, x });
    x += w;
  }
  const legendHeight = pad + rows.length * line + (bar ? line + 16 * ratio : 0) + pad;
  out.height = v.canvas.height + legendHeight;
  ctx.fillStyle = token("--surface");
  ctx.fillRect(0, 0, out.width, out.height);
  ctx.drawImage(v.canvas, 0, 0);
  ctx.font = font;
  ctx.textBaseline = "middle";
  let y = v.canvas.height + pad + line / 2;
  for (const row of rows) {
    for (const entry of row) {
      ctx.fillStyle = entry.color;
      const size = 12 * ratio;
      if (entry.round) {
        ctx.beginPath();
        ctx.arc(entry.x + size / 2, y, size / 3, 0, 2 * Math.PI);
        ctx.fill();
      } else ctx.fillRect(entry.x, y - size / 2, size, size);
      ctx.fillStyle = token("--ink");
      ctx.fillText(entry.name, entry.x + 18 * ratio, y);
    }
    y += line;
  }
  if (bar) {
    let bx = pad;
    ctx.fillStyle = token("--ink");
    ctx.fillText(bar.label, bx, y);
    bx += ctx.measureText(bar.label).width + 12 * ratio;
    ctx.fillStyle = token("--ink-2");
    if (bar.constant !== undefined) ctx.fillText(`constant: ${bar.constant}`, bx, y);
    else {
      const ticks = bar.ticks ?? [];
      if (!ticks.length) {
        ctx.fillText(bar.low, bx, y);
        bx += ctx.measureText(bar.low).width + 8 * ratio;
      }
      const barWidth = Math.min(240 * ratio, Math.max(60 * ratio, width - bx - ctx.measureText(bar.high).width - pad - 8 * ratio));
      for (let i = 0; i < barWidth; i++) {
        const p = i / Math.max(1, barWidth - 1);
        const k = 3 * Math.round(255 * (bar.reversed ? 1 - p : p));
        ctx.fillStyle = `rgb(${bar.table[k]},${bar.table[k + 1]},${bar.table[k + 2]})`;
        ctx.fillRect(bx + i, y - 5 * ratio, 1, 10 * ratio);
      }
      ctx.fillStyle = token("--ink-2");
      if (ticks.length) {
        ctx.save();
        ctx.font = `${Math.round(11 * ratio)}px ${token("--font")}`;
        ctx.textAlign = "center";
        for (const t of ticks) {
          const tx = bx + t.position * (barWidth - 1);
          ctx.fillRect(tx, y + 5 * ratio, 1, 3 * ratio);
          ctx.fillText(t.text, tx, y + 14 * ratio);
        }
        ctx.restore();
      } else ctx.fillText(bar.high, bx + barWidth + 8 * ratio, y);
    }
  }
  return new Promise((resolve) => out.toBlob(resolve, "image/png"));
}
