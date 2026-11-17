// The picture's interaction: one pointer pans (a press that does not
// move selects the boundary region under it, or else pins a probe; with the profile tool it draws the profile's line),
// two pointers pinch to zoom and pan together, the wheel zooms, the keys
// pan, zoom, fit and pin; the readout under the pointer.

import { formatNumber, getDefaultDigits } from "../pure/format.js";
import { locatePoint, nearestValuedNode } from "../pure/mesh_math.js";
import { h } from "../dom.js";
import { gradientName } from "./legend.js";

// A press that moves less than this (CSS pixels) is a click: it pins a
// probe instead of panning. A profile line shorter than this is dropped.
const CLICK_SLOP = 4;
// The readout of a dot field takes the nearest dot within this.
const DOT_PICK_PX = 8;
// Arrow keys pan by this, Shift + arrow by four times as much.
const KEY_PAN_PX = 40;

export function listen(v) {
  const canvas = v.canvas;
  // The pointers down on the picture: id -> {x, y} (CSS pixels in the
  // canvas).
  const pointers = new Map();
  // One pointer: {x, y, offset, moved, profile}; two: {distance, middle,
  // world, scale}.
  let drag = null;
  let pinch = null;
  const local = (event) => {
    const rect = canvas.getBoundingClientRect();
    return { x: event.clientX - rect.left, y: event.clientY - rect.top };
  };
  const middleOf = () => {
    const [a, b] = [...pointers.values()];
    return { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2, distance: Math.hypot(a.x - b.x, a.y - b.y) };
  };
  const startPinch = () => {
    const m = middleOf();
    pinch = { distance: Math.max(1, m.distance), world: v.toWorld(m.x, m.y), scale: v.scale };
    drag = null;
    v.draft = null;
    canvas.classList.add("dragging");
  };

  canvas.addEventListener("pointerdown", (event) => {
    const p = local(event);
    pointers.set(event.pointerId, p);
    try {
      canvas.setPointerCapture(event.pointerId);
    } catch {
      // A pointer the browser does not track (a synthetic event).
    }
    v.tooltip.hidden = true;
    if (pointers.size === 2) {
      if (drag?.overlay) v.overlay?.pointerup?.(v, p, event, true);
      return startPinch();
    }
    if (pointers.size > 2) return;
    // The geometry editor takes a press on something it edits.
    if (v.overlay?.pointerdown?.(v, p, event)) {
      drag = { x: p.x, y: p.y, overlay: true };
      return;
    }
    const profile = v.data.tool === "profile" && v.data.mesh && !v.data.meshStale;
    drag = { x: p.x, y: p.y, offset: { ...v.offset }, moved: false, profile, start: v.toWorld(p.x, p.y) };
  });

  canvas.addEventListener("pointermove", (event) => {
    if (!pointers.has(event.pointerId)) {
      v.overlay?.hover?.(v, local(event), event);
      return hover(v, event);
    }
    const p = local(event);
    pointers.set(event.pointerId, p);
    if (drag?.overlay) {
      v.overlay?.pointermove?.(v, p, event);
      return;
    }
    if (pinch && pointers.size >= 2) {
      // The world point under the fingers' middle stays under it while
      // the scale follows the fingers' distance.
      const m = middleOf();
      v.scale = pinch.scale * (Math.max(1, m.distance) / pinch.distance);
      v.offset = { x: m.x - pinch.world[0] * v.scale, y: m.y + pinch.world[1] * v.scale };
      v.viewMoved();
      return;
    }
    if (!drag) return;
    const dx = p.x - drag.x;
    const dy = p.y - drag.y;
    if (!drag.moved && Math.hypot(dx, dy) < CLICK_SLOP) return;
    drag.moved = true;
    if (drag.profile) {
      v.draft = { from: drag.start, to: v.toWorld(p.x, p.y) };
      v.draw();
      return;
    }
    canvas.classList.add("dragging");
    v.offset = { x: drag.offset.x + dx, y: drag.offset.y + dy };
    v.viewMoved();
  });

  const release = (event, cancelled) => {
    if (!pointers.has(event.pointerId)) return;
    pointers.delete(event.pointerId);
    if (pinch) {
      // The other finger stays down: it neither pans nor pins until lifted.
      if (pointers.size < 2) pinch = null;
      if (pointers.size === 0) canvas.classList.remove("dragging");
      return;
    }
    const ended = drag;
    drag = null;
    canvas.classList.remove("dragging");
    if (ended?.overlay) {
      v.overlay?.pointerup?.(v, local(event), event, cancelled);
      return;
    }
    if (!ended || cancelled) {
      if (v.draft) {
        v.draft = null;
        v.draw();
      }
      return;
    }
    if (ended.profile && ended.moved && v.draft) {
      const line = v.draft;
      v.draft = null;
      v.callbacks.onProfile?.(line);
      return;
    }
    if (!ended.moved && event.button === 0) {
      // The editor may take the click (a click beside the drawing clears
      // its selection); on a boundary (within a few pixels) a click
      // selects its region; elsewhere it pins a probe.
      if (v.overlay?.click?.(v, { x: ended.x, y: ended.y }, event)) return;
      const region = v.callbacks.onRegion ? regionNear(v, ended.x, ended.y) : null;
      if (region) v.callbacks.onRegion(region, { x: ended.x, y: ended.y });
      else pin(v, ended.x, ended.y);
    }
  };
  canvas.addEventListener("pointerup", (event) => release(event, false));
  canvas.addEventListener("pointercancel", (event) => release(event, true));
  canvas.addEventListener("pointerleave", () => {
    v.tooltip.hidden = true;
    v.overlay?.leave?.(v);
  });
  canvas.addEventListener(
    "wheel",
    (event) => {
      event.preventDefault();
      const p = local(event);
      const factor = Math.exp(-event.deltaY * (event.deltaMode === 1 ? 0.05 : 0.0015));
      v.zoomAt(p.x, p.y, factor);
      hover(v, event);
    },
    { passive: false },
  );
  canvas.addEventListener("dblclick", (event) => {
    if (v.overlay?.dblclick?.(v, local(event), event)) return;
    v.fit();
  });
  canvas.addEventListener("keydown", (event) => {
    if (v.overlay?.key?.(v, event)) {
      event.preventDefault();
      return;
    }
    key(v, event);
  });
}

// Keyboard: arrows pan, + and - zoom about the centre, 0 fits, Enter
// pins a probe at the centre.
function key(v, event) {
  const { clientWidth: width, clientHeight: height } = v.canvas.parentElement;
  const step = KEY_PAN_PX * (event.shiftKey ? 4 : 1);
  const pan = { ArrowLeft: [step, 0], ArrowRight: [-step, 0], ArrowUp: [0, step], ArrowDown: [0, -step] }[event.key];
  if (pan) {
    v.offset = { x: v.offset.x + pan[0], y: v.offset.y + pan[1] };
    v.viewMoved();
  } else if (event.key === "+" || event.key === "=") v.zoomBy(1.25);
  else if (event.key === "-" || event.key === "_") v.zoomBy(0.8);
  else if (event.key === "0") v.fit();
  else if (event.key === "Enter") pin(v, width / 2, height / 2);
  else return;
  event.preventDefault();
}

function pin(v, px, py) {
  if (!v.callbacks.onPin || !v.data.mesh || v.data.meshStale) return;
  const [x, y] = v.toWorld(px, py);
  v.callbacks.onPin(x, y);
}

// The readout under the pointer: coordinates, region, triangle and the
// value shown there (the field's, |∇| of the triangle, or for a dot
// field the value of the nearest dot).
function hover(v, event) {
  if (event.pointerType === "touch") return;
  const rect = v.canvas.getBoundingClientRect();
  const px = event.clientX - rect.left;
  const py = event.clientY - rect.top;
  const [x, y] = v.toWorld(px, py);
  const rows = [["x, y", `${formatNumber(x)}, ${formatNumber(y)}`]];
  const digits = getDefaultDigits();
  const { mesh, meshStale, field, geometry, colorBy } = v.data;
  if (!mesh && !geometry) {
    v.tooltip.hidden = true;
    return;
  }
  const region = regionNear(v, px, py);
  if (region) rows.push(["region", region]);
  if (mesh && !meshStale && v.grid) {
    const found = locatePoint(mesh, v.grid, x, y);
    if (found) {
      const angle = formatNumber(v.angles[found.triangle], Math.max(2, digits - 1));
      rows.push(["triangle", `${found.triangle} · smallest angle ${angle}°`]);
      if (field && v.fieldComplete) {
        const value = found.weights.reduce((sum, w, k) => sum + w * field.values[found.nodes[k]], 0);
        rows.push([field.label, formatNumber(value, digits + 2)]);
      }
      if (field && colorBy === "gradient" && v.gradients) rows.push([gradientName(field.name), formatNumber(v.gradients[found.triangle], digits + 2)]);
      if (field && colorBy === "indicator" && v.indicatorLogs && field.indicators) rows.push(["η (error indicator)", formatNumber(field.indicators[found.triangle], digits + 2)]);
    }
    if (field && colorBy === "field" && !v.fieldComplete) {
      const node = nearestValuedNode(mesh.nodes, field.values, x, y, DOT_PICK_PX / v.scale);
      if (node >= 0) rows.push([`${field.label} at node ${node}`, formatNumber(field.values[node], digits + 2)]);
    }
  }
  const tooltip = v.tooltip;
  tooltip.replaceChildren(...rows.map(([k, value]) => h("div", {}, h("span", { class: "key" }, `${k}: `), h("span", { class: "num" }, value))));
  tooltip.hidden = false;
  const wrap = v.canvas.parentElement;
  const left = Math.min(px + 14, wrap.clientWidth - tooltip.offsetWidth - 4);
  const top = py + 14 + tooltip.offsetHeight > wrap.clientHeight ? py - tooltip.offsetHeight - 8 : py + 14;
  tooltip.style.left = `${Math.max(4, left)}px`;
  tooltip.style.top = `${Math.max(4, top)}px`;
}

// The region of the boundary within a few pixels of the pointer.
function regionNear(v, px, py) {
  const { mesh, meshStale, geometry } = v.data;
  let best = 7;
  let name = null;
  const test = (ax, ay, bx, by, region) => {
    const [x1, y1] = v.toScreen(ax, ay);
    const [x2, y2] = v.toScreen(bx, by);
    const d = segmentDistance(px, py, x1, y1, x2, y2);
    if (d < best) {
      best = d;
      name = region;
    }
  };
  if (mesh && !meshStale) {
    const n = mesh.nodes;
    const e = mesh.boundary_edges;
    for (let i = 0; i < e.length; i += 3) test(n[2 * e[i]], n[2 * e[i] + 1], n[2 * e[i + 1]], n[2 * e[i + 1] + 1], mesh.region_names[e[i + 2]]);
  } else if (geometry?.kind === "curves") {
    for (const loop of geometry.loops)
      for (const curve of loop.curves)
        for (let i = 0; i + 3 < curve.points.length; i += 2) test(curve.points[i], curve.points[i + 1], curve.points[i + 2], curve.points[i + 3], curve.region);
  } else if (geometry?.kind === "mesh_file") {
    for (const [ax, ay, bx, by, region] of geometry.boundary_edges) test(ax, ay, bx, by, geometry.regions[region].name);
  }
  return name;
}

function segmentDistance(px, py, ax, ay, bx, by) {
  const dx = bx - ax;
  const dy = by - ay;
  const length = dx * dx + dy * dy;
  const s = length === 0 ? 0 : Math.max(0, Math.min(1, ((px - ax) * dx + (py - ay) * dy) / length));
  return Math.hypot(px - (ax + s * dx), py - (ay + s * dy));
}
