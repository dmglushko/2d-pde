// The picture: geometry, mesh, regions, Dirichlet nodes and a colour layer
// (smallest angle per triangle, or a field with linear interpolation), with
// panning, zooming and a readout under the pointer.

import { barycentric, fieldScale, formatNumber, parseColor, rampTable, regionToken, triangleAngles } from "./core.js";
import { h } from "./dom.js";

const QUALITY_MAX_DEGREES = 60;

function token(name) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
}

export class Viewer {
  constructor(canvas, tooltip, status) {
    this.canvas = canvas;
    this.tooltip = tooltip;
    this.status = status;
    this.context = canvas.getContext("2d");
    this.scale = 1;
    this.offset = { x: 0, y: 0 };
    this.data = {};
    this.fittedKey = null;
    this.offscreen = document.createElement("canvas");

    new ResizeObserver(() => this.resize()).observe(canvas.parentElement);
    matchMedia("(prefers-color-scheme: dark)").addEventListener("change", () => this.draw());
    this.listen();
  }

  // data: {geometry (view), mesh (mesh view), meshStale, regions (names),
  // dirichlet (node indices), field ({label, values, scale?}), colorBy,
  // layers}.
  set(data) {
    const meshChanged = data.mesh !== this.data.mesh;
    this.data = data;
    if (meshChanged) this.prepareMesh();
    const box = this.boundingBox();
    const key = box ? JSON.stringify(box) : null;
    if (key && key !== this.fittedKey) {
      this.fittedKey = key;
      this.fit();
    } else this.draw();
  }

  prepareMesh() {
    const mesh = this.data.mesh;
    this.angles = mesh ? triangleAngles(mesh) : null;
    this.grid = mesh ? buildGrid(mesh) : null;
  }

  boundingBox() {
    const g = this.data.geometry?.bounding_box;
    if (g) return g;
    const mesh = this.data.mesh;
    if (!mesh || mesh.nodes.length === 0) return null;
    let min = [Infinity, Infinity];
    let max = [-Infinity, -Infinity];
    for (let i = 0; i < mesh.nodes.length; i += 2) {
      min = [Math.min(min[0], mesh.nodes[i]), Math.min(min[1], mesh.nodes[i + 1])];
      max = [Math.max(max[0], mesh.nodes[i]), Math.max(max[1], mesh.nodes[i + 1])];
    }
    return { min, max };
  }

  resize() {
    const ratio = window.devicePixelRatio || 1;
    const { clientWidth: width, clientHeight: height } = this.canvas.parentElement;
    this.canvas.width = Math.max(1, Math.round(width * ratio));
    this.canvas.height = Math.max(1, Math.round(height * ratio));
    if (this.fittedKey && !this.userMoved) this.fit();
    else this.draw();
  }

  fit() {
    const box = this.boundingBox();
    const { clientWidth: width, clientHeight: height } = this.canvas.parentElement;
    if (!box || width === 0) return this.draw();
    const w = Math.max(box.max[0] - box.min[0], 1e-12);
    const hgt = Math.max(box.max[1] - box.min[1], 1e-12);
    this.scale = 0.88 * Math.min(width / w, height / hgt);
    this.offset = {
      x: width / 2 - ((box.min[0] + box.max[0]) / 2) * this.scale,
      y: height / 2 + ((box.min[1] + box.max[1]) / 2) * this.scale,
    };
    this.userMoved = false;
    this.draw();
  }

  // World (x up) to CSS pixels (y down), and back.
  toScreen(x, y) {
    return [this.offset.x + x * this.scale, this.offset.y - y * this.scale];
  }
  toWorld(px, py) {
    return [(px - this.offset.x) / this.scale, (this.offset.y - py) / this.scale];
  }

  // --- drawing ---------------------------------------------------------------

  draw() {
    const ctx = this.context;
    const ratio = this.canvas.width / Math.max(1, this.canvas.parentElement.clientWidth);
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.fillStyle = token("--surface");
    ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    this.ratio = ratio;

    const { geometry, mesh, meshStale, layers = {}, colorBy } = this.data;
    const hasMesh = mesh && mesh.triangles.length > 0;
    let note = "";

    if (geometry?.kind === "curves" && (!hasMesh || meshStale)) this.fillGeometry(geometry);
    if (hasMesh && !meshStale) {
      if (colorBy === "quality") this.drawQuality(mesh);
      if (colorBy === "field" && this.fieldIsComplete()) this.rasterField(mesh, this.data.field.values);
    }
    if (hasMesh && layers.mesh) {
      const drawn = this.drawEdges(mesh, meshStale ? 0.35 : 1);
      if (!drawn) note = "edges hidden at this zoom";
    }
    if (layers.geometry) {
      if (hasMesh && !meshStale) this.drawMeshBoundary(mesh);
      else if (geometry?.kind === "curves") this.drawCurves(geometry);
      else if (geometry?.kind === "mesh_file") this.drawEdgeList(geometry);
    }
    if (hasMesh && !meshStale && colorBy === "field" && this.data.field && !this.fieldIsComplete()) this.drawFieldDots(mesh, this.data.field.values);
    if (hasMesh && !meshStale && layers.dirichlet && this.data.dirichlet) this.drawNodes(mesh, this.data.dirichlet);
    if (meshStale) note = "mesh outdated: run the Mesh step";
    this.status.textContent = note;
  }

  fieldIsComplete() {
    const field = this.data.field;
    return field && field.values.length * 2 === this.data.mesh?.nodes.length && field.values.every((v) => Number.isFinite(v));
  }

  fillGeometry(geometry) {
    const ctx = this.context;
    ctx.beginPath();
    for (const loop of geometry.loops) {
      let first = true;
      for (const curve of loop.curves) {
        const p = curve.points;
        for (let i = 0; i < p.length; i += 2) {
          const [x, y] = this.toScreen(p[i], p[i + 1]);
          if (first) ctx.moveTo(x, y);
          else ctx.lineTo(x, y);
          first = false;
        }
      }
      ctx.closePath();
    }
    ctx.fillStyle = token("--geometry-fill");
    ctx.fill("evenodd");
  }

  regionColors(count) {
    return Array.from({ length: count }, (_, i) => token(regionToken(i)));
  }

  drawCurves(geometry) {
    const ctx = this.context;
    const names = geometry.regions.map((r) => r.name);
    const colors = this.regionColors(names.length);
    ctx.lineWidth = 2;
    ctx.lineJoin = "round";
    ctx.lineCap = "round";
    for (const loop of geometry.loops)
      for (const curve of loop.curves) {
        const p = curve.points;
        ctx.beginPath();
        for (let i = 0; i < p.length; i += 2) {
          const [x, y] = this.toScreen(p[i], p[i + 1]);
          if (i === 0) ctx.moveTo(x, y);
          else ctx.lineTo(x, y);
        }
        ctx.strokeStyle = colors[names.indexOf(curve.region)];
        ctx.stroke();
      }
  }

  drawEdgeList(view) {
    const ctx = this.context;
    const colors = this.regionColors(view.regions.length);
    ctx.lineWidth = 2;
    for (const [ax, ay, bx, by, region] of view.boundary_edges) {
      ctx.beginPath();
      ctx.moveTo(...this.toScreen(ax, ay));
      ctx.lineTo(...this.toScreen(bx, by));
      ctx.strokeStyle = colors[region];
      ctx.stroke();
    }
  }

  drawMeshBoundary(mesh) {
    const ctx = this.context;
    const colors = this.regionColors(mesh.region_names.length);
    const n = mesh.nodes;
    const e = mesh.boundary_edges;
    ctx.lineWidth = 2;
    ctx.lineCap = "round";
    for (let i = 0; i < e.length; i += 3) {
      ctx.beginPath();
      ctx.moveTo(...this.toScreen(n[2 * e[i]], n[2 * e[i] + 1]));
      ctx.lineTo(...this.toScreen(n[2 * e[i + 1]], n[2 * e[i + 1] + 1]));
      ctx.strokeStyle = colors[e[i + 2]];
      ctx.stroke();
    }
  }

  // Returns false when the triangles are too small on screen to draw.
  drawEdges(mesh, alpha) {
    const ctx = this.context;
    const n = mesh.nodes;
    const t = mesh.triangles;
    const count = t.length / 3;
    const { clientWidth, clientHeight } = this.canvas.parentElement;
    const box = this.boundingBox();
    const area = (box.max[0] - box.min[0]) * (box.max[1] - box.min[1]) * this.scale * this.scale;
    if (count > 0 && area / count < 6 && area < 4 * clientWidth * clientHeight) return false;
    ctx.globalAlpha = alpha;
    ctx.strokeStyle = token("--mesh-edge");
    ctx.lineWidth = 1 / this.ratio;
    ctx.beginPath();
    for (let i = 0; i < t.length; i += 3) {
      const [ax, ay] = this.toScreen(n[2 * t[i]], n[2 * t[i] + 1]);
      const [bx, by] = this.toScreen(n[2 * t[i + 1]], n[2 * t[i + 1] + 1]);
      const [cx, cy] = this.toScreen(n[2 * t[i + 2]], n[2 * t[i + 2] + 1]);
      ctx.moveTo(ax, ay);
      ctx.lineTo(bx, by);
      ctx.lineTo(cx, cy);
      ctx.closePath();
    }
    ctx.stroke();
    ctx.globalAlpha = 1;
    return true;
  }

  sequentialTable() {
    return rampTable([parseColor(token("--seq-low")), parseColor(token("--seq-high"))]);
  }

  divergingTable() {
    return rampTable([parseColor(token("--div-low")), parseColor(token("--div-mid")), parseColor(token("--div-high"))]);
  }

  // Each triangle filled by its smallest angle: dark for small angles.
  drawQuality(mesh) {
    const ctx = this.context;
    const table = this.sequentialTable();
    const n = mesh.nodes;
    const t = mesh.triangles;
    for (let k = 0; k < this.angles.length; k++) {
      const index = Math.round(255 * (1 - Math.min(this.angles[k], QUALITY_MAX_DEGREES) / QUALITY_MAX_DEGREES));
      ctx.fillStyle = `rgb(${table[3 * index]},${table[3 * index + 1]},${table[3 * index + 2]})`;
      ctx.beginPath();
      ctx.moveTo(...this.toScreen(n[2 * t[3 * k]], n[2 * t[3 * k] + 1]));
      ctx.lineTo(...this.toScreen(n[2 * t[3 * k + 1]], n[2 * t[3 * k + 1] + 1]));
      ctx.lineTo(...this.toScreen(n[2 * t[3 * k + 2]], n[2 * t[3 * k + 2] + 1]));
      ctx.closePath();
      ctx.fill();
      ctx.strokeStyle = ctx.fillStyle;
      ctx.lineWidth = 0.5;
      ctx.stroke();
    }
  }

  // A field may bring its scale (a solution: one scale for every time).
  scaleOf(values) {
    return this.data.field?.scale ?? fieldScale(values);
  }

  fieldTable(scale) {
    return scale.kind === "diverging" ? this.divergingTable() : this.sequentialTable();
  }

  // A P1 field: every pixel inside a triangle gets the linear interpolant
  // of the three node values.
  rasterField(mesh, values) {
    const width = this.canvas.width;
    const height = this.canvas.height;
    const scale = this.scaleOf(values);
    const table = this.fieldTable(scale);
    this.offscreen.width = width;
    this.offscreen.height = height;
    const off = this.offscreen.getContext("2d");
    const image = off.createImageData(width, height);
    const data = image.data;
    const n = mesh.nodes;
    const t = mesh.triangles;
    const r = this.ratio;
    const sx = (i) => (this.offset.x + n[2 * i] * this.scale) * r;
    const sy = (i) => (this.offset.y - n[2 * i + 1] * this.scale) * r;
    for (let k = 0; k < t.length; k += 3) {
      const [a, b, c] = [t[k], t[k + 1], t[k + 2]];
      const ax = sx(a), ay = sy(a), bx = sx(b), by = sy(b), cx = sx(c), cy = sy(c);
      const x0 = Math.max(0, Math.floor(Math.min(ax, bx, cx)));
      const x1 = Math.min(width - 1, Math.ceil(Math.max(ax, bx, cx)));
      const y0 = Math.max(0, Math.floor(Math.min(ay, by, cy)));
      const y1 = Math.min(height - 1, Math.ceil(Math.max(ay, by, cy)));
      if (x0 > x1 || y0 > y1) continue;
      const det = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy);
      if (det === 0) continue;
      const va = values[a], vb = values[b], vc = values[c];
      const tolerance = -1e-6;
      for (let py = y0; py <= y1; py++) {
        const y = py + 0.5;
        for (let px = x0; px <= x1; px++) {
          const x = px + 0.5;
          const l1 = ((by - cy) * (x - cx) + (cx - bx) * (y - cy)) / det;
          if (l1 < tolerance) continue;
          const l2 = ((cy - ay) * (x - cx) + (ax - cx) * (y - cy)) / det;
          if (l2 < tolerance) continue;
          const l3 = 1 - l1 - l2;
          if (l3 < tolerance) continue;
          const v = l1 * va + l2 * vb + l3 * vc;
          const index = Math.max(0, Math.min(255, Math.round(255 * scale.position(v))));
          const o = 4 * (py * width + px);
          data[o] = table[3 * index];
          data[o + 1] = table[3 * index + 1];
          data[o + 2] = table[3 * index + 2];
          data[o + 3] = 255;
        }
      }
    }
    off.putImageData(image, 0, 0);
    const ctx = this.context;
    ctx.save();
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.drawImage(this.offscreen, 0, 0);
    ctx.restore();
  }

  // A field that has values at some nodes only: one dot per value.
  drawFieldDots(mesh, values) {
    const ctx = this.context;
    const scale = this.scaleOf(values);
    const table = this.fieldTable(scale);
    const ring = token("--surface");
    for (let i = 0; i < values.length; i++) {
      if (!Number.isFinite(values[i])) continue;
      const index = Math.max(0, Math.min(255, Math.round(255 * scale.position(values[i]))));
      ctx.beginPath();
      ctx.arc(...this.toScreen(mesh.nodes[2 * i], mesh.nodes[2 * i + 1]), 4.5, 0, 2 * Math.PI);
      ctx.fillStyle = `rgb(${table[3 * index]},${table[3 * index + 1]},${table[3 * index + 2]})`;
      ctx.fill();
      ctx.lineWidth = 1.5;
      ctx.strokeStyle = ring;
      ctx.stroke();
    }
  }

  drawNodes(mesh, nodes) {
    const ctx = this.context;
    ctx.fillStyle = token("--ink");
    for (const i of nodes) {
      ctx.beginPath();
      ctx.arc(...this.toScreen(mesh.nodes[2 * i], mesh.nodes[2 * i + 1]), 2.5, 0, 2 * Math.PI);
      ctx.fill();
    }
  }

  // --- the legend ------------------------------------------------------------

  legend() {
    const { geometry, mesh, meshStale, colorBy, field } = this.data;
    const items = [];
    const names = mesh && !meshStale ? mesh.region_names : (geometry?.regions ?? []).map((r) => r.name);
    names.forEach((name, i) => items.push(h("span", { class: "item" }, h("span", { class: "swatch", style: `background:var(${regionToken(i)})` }), name)));
    if (mesh && !meshStale && colorBy === "quality")
      items.push(colorbar("smallest angle", this.sequentialTable(), `${QUALITY_MAX_DEGREES}°+`, "0°", true));
    if (mesh && !meshStale && colorBy === "field" && field) {
      const scale = this.scaleOf(field.values);
      if (scale.kind === "diverging") items.push(colorbar(field.label, this.divergingTable(), formatNumber(-scale.bound), formatNumber(scale.bound)));
      else items.push(colorbar(field.label, this.sequentialTable(), formatNumber(scale.min), formatNumber(scale.max)));
    }
    if (this.data.layers?.dirichlet && this.data.dirichlet && mesh && !meshStale)
      items.push(h("span", { class: "item" }, h("span", { class: "swatch", style: "background:var(--ink);border-radius:50%;width:7px;height:7px" }), "Dirichlet node"));
    return items;
  }

  // --- interaction -----------------------------------------------------------

  listen() {
    const canvas = this.canvas;
    let drag = null;
    canvas.addEventListener("pointerdown", (event) => {
      drag = { x: event.clientX, y: event.clientY, offset: { ...this.offset } };
      canvas.setPointerCapture(event.pointerId);
      canvas.classList.add("dragging");
    });
    canvas.addEventListener("pointermove", (event) => {
      if (drag) {
        this.offset = { x: drag.offset.x + event.clientX - drag.x, y: drag.offset.y + event.clientY - drag.y };
        this.userMoved = true;
        this.tooltip.hidden = true;
        this.draw();
      } else this.hover(event);
    });
    const end = () => {
      drag = null;
      canvas.classList.remove("dragging");
    };
    canvas.addEventListener("pointerup", end);
    canvas.addEventListener("pointercancel", end);
    canvas.addEventListener("pointerleave", () => (this.tooltip.hidden = true));
    canvas.addEventListener(
      "wheel",
      (event) => {
        event.preventDefault();
        const rect = canvas.getBoundingClientRect();
        const px = event.clientX - rect.left;
        const py = event.clientY - rect.top;
        const [wx, wy] = this.toWorld(px, py);
        const factor = Math.exp(-event.deltaY * (event.deltaMode === 1 ? 0.05 : 0.0015));
        this.scale *= factor;
        this.offset = { x: px - wx * this.scale, y: py + wy * this.scale };
        this.userMoved = true;
        this.draw();
        this.hover(event);
      },
      { passive: false },
    );
    canvas.addEventListener("dblclick", () => this.fit());
  }

  hover(event) {
    const rect = this.canvas.getBoundingClientRect();
    const px = event.clientX - rect.left;
    const py = event.clientY - rect.top;
    const [x, y] = this.toWorld(px, py);
    const rows = [["x, y", `${formatNumber(x)}, ${formatNumber(y)}`]];
    const { mesh, meshStale, field, geometry } = this.data;
    const region = this.regionNear(px, py);
    if (region) rows.push(["region", region]);
    if (mesh && !meshStale && this.grid) {
      const found = locate(mesh, this.grid, x, y);
      if (found) {
        rows.push(["triangle", `${found.triangle} · smallest angle ${formatNumber(this.angles[found.triangle], 3)}°`]);
        if (field && this.fieldIsComplete()) {
          const [a, b, c] = [3 * found.triangle, 3 * found.triangle + 1, 3 * found.triangle + 2].map((k) => mesh.triangles[k]);
          const v = found.weights[0] * field.values[a] + found.weights[1] * field.values[b] + found.weights[2] * field.values[c];
          rows.push([field.label, formatNumber(v, 6)]);
        }
      }
    }
    if (!mesh && !geometry) {
      this.tooltip.hidden = true;
      return;
    }
    this.tooltip.replaceChildren(...rows.map(([k, v]) => h("div", {}, h("span", { class: "key" }, `${k}: `), h("span", { class: "num" }, v))));
    this.tooltip.hidden = false;
    const wrap = this.canvas.parentElement;
    const left = Math.min(px + 14, wrap.clientWidth - this.tooltip.offsetWidth - 4);
    const top = py + 14 + this.tooltip.offsetHeight > wrap.clientHeight ? py - this.tooltip.offsetHeight - 8 : py + 14;
    this.tooltip.style.left = `${Math.max(4, left)}px`;
    this.tooltip.style.top = `${Math.max(4, top)}px`;
  }

  // The region of the boundary within a few pixels of the pointer.
  regionNear(px, py) {
    const { mesh, meshStale, geometry } = this.data;
    let best = 7;
    let name = null;
    const test = (ax, ay, bx, by, region) => {
      const [x1, y1] = this.toScreen(ax, ay);
      const [x2, y2] = this.toScreen(bx, by);
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
}

function segmentDistance(px, py, ax, ay, bx, by) {
  const dx = bx - ax;
  const dy = by - ay;
  const length = dx * dx + dy * dy;
  const s = length === 0 ? 0 : Math.max(0, Math.min(1, ((px - ax) * dx + (py - ay) * dy) / length));
  return Math.hypot(px - (ax + s * dx), py - (ay + s * dy));
}

function colorbar(label, table, low, high, reversed = false) {
  const canvas = h("canvas", { width: 256, height: 1, "aria-hidden": "true" });
  const context = canvas.getContext("2d");
  const image = context.createImageData(256, 1);
  for (let i = 0; i < 256; i++) {
    const k = reversed ? 255 - i : i;
    image.data.set([table[3 * k], table[3 * k + 1], table[3 * k + 2], 255], 4 * i);
  }
  context.putImageData(image, 0, 0);
  return h("span", { class: "colorbar" }, h("span", {}, label), h("span", { class: "num" }, low), canvas, h("span", { class: "num" }, high));
}

// A uniform grid of triangle indices by bounding box, for finding the
// triangle under the pointer.
function buildGrid(mesh) {
  const n = mesh.nodes;
  const t = mesh.triangles;
  const count = t.length / 3;
  if (count === 0) return null;
  let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
  for (let i = 0; i < n.length; i += 2) {
    minX = Math.min(minX, n[i]);
    maxX = Math.max(maxX, n[i]);
    minY = Math.min(minY, n[i + 1]);
    maxY = Math.max(maxY, n[i + 1]);
  }
  const side = Math.max(1, Math.round(Math.sqrt(count / 2)));
  const cellW = (maxX - minX) / side || 1;
  const cellH = (maxY - minY) / side || 1;
  const cells = Array.from({ length: side * side }, () => []);
  const cell = (v, min, size) => Math.max(0, Math.min(side - 1, Math.floor((v - min) / size)));
  for (let k = 0; k < count; k++) {
    const xs = [n[2 * t[3 * k]], n[2 * t[3 * k + 1]], n[2 * t[3 * k + 2]]];
    const ys = [n[2 * t[3 * k] + 1], n[2 * t[3 * k + 1] + 1], n[2 * t[3 * k + 2] + 1]];
    const i0 = cell(Math.min(...xs), minX, cellW), i1 = cell(Math.max(...xs), minX, cellW);
    const j0 = cell(Math.min(...ys), minY, cellH), j1 = cell(Math.max(...ys), minY, cellH);
    for (let j = j0; j <= j1; j++) for (let i = i0; i <= i1; i++) cells[j * side + i].push(k);
  }
  return { minX, minY, maxX, maxY, side, cellW, cellH, cells };
}

function locate(mesh, grid, x, y) {
  if (!grid || x < grid.minX || x > grid.maxX || y < grid.minY || y > grid.maxY) return null;
  const i = Math.min(grid.side - 1, Math.floor((x - grid.minX) / grid.cellW));
  const j = Math.min(grid.side - 1, Math.floor((y - grid.minY) / grid.cellH));
  const n = mesh.nodes;
  const t = mesh.triangles;
  for (const k of grid.cells[j * grid.side + i]) {
    const [a, b, c] = [t[3 * k], t[3 * k + 1], t[3 * k + 2]];
    const w = barycentric(x, y, n[2 * a], n[2 * a + 1], n[2 * b], n[2 * b + 1], n[2 * c], n[2 * c + 1]);
    if (w && w[0] >= -1e-9 && w[1] >= -1e-9 && w[2] >= -1e-9) return { triangle: k, weights: w };
  }
  return null;
}
