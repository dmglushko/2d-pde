// The picture: geometry, mesh, regions, Dirichlet nodes and a colour layer
// (smallest angle per triangle, or a field with linear interpolation), with
// panning, zooming and a readout under the pointer.

import {
  fieldScale,
  formatNumber,
  getDefaultDigits,
  isolineLevels,
  isolines,
  locatePoint,
  meshGrid,
  nearestValuedNode,
  nodePositions,
  parseColor,
  rampTable,
  regionToken,
  triangleAngles,
  viewTransform,
} from "./core.js";
import { h, regionSwatch } from "./dom.js";
import { settings } from "./settings.js";

const QUALITY_MAX_DEGREES = 60;
// A pan or zoom shows the last full picture moved and scaled; the full
// picture is drawn again once the gesture pauses this long.
const SETTLE_MS = 120;

// A press that moves less than this (CSS pixels) is a click: it pins a
// probe instead of panning.
const CLICK_SLOP = 4;
// The readout of a dot field takes the nearest dot within this.
const DOT_PICK_PX = 8;
// Arrow keys pan by this, Shift + arrow by four times as much.
const KEY_PAN_PX = 40;

// The colour tokens, read once (getComputedStyle is not free). The page
// has one, dark, theme.
const tokens = new Map();
function token(name) {
  let value = tokens.get(name);
  if (value === undefined) {
    value = getComputedStyle(document.documentElement).getPropertyValue(name).trim();
    tokens.set(name, value);
  }
  return value;
}

const sameLayers = (a = {}, b = {}) =>
  a.geometry === b.geometry && a.mesh === b.mesh && a.dirichlet === b.dirichlet && a.isolines === b.isolines && a.isolineCount === b.isolineCount;
const sameField = (a, b) => a === b || (a && b && a.values === b.values && a.scale === b.scale && a.label === b.label);

export class Viewer {
  // onPin(x, y): a click (or Enter) at a point of the picture.
  constructor(canvas, tooltip, status, { onPin = null } = {}) {
    this.canvas = canvas;
    this.onPin = onPin;
    this.tooltip = tooltip;
    this.status = status;
    this.context = canvas.getContext("2d");
    this.scale = 1;
    this.offset = { x: 0, y: 0 };
    this.data = {};
    this.fittedKey = null;
    this.offscreen = document.createElement("canvas");
    this.image = null;
    // The last full picture and the view it was drawn at.
    this.frame = document.createElement("canvas");
    this.frameView = null;
    this.blitRequest = null;
    this.settleTimer = null;

    new ResizeObserver(() => this.resize()).observe(canvas.parentElement);
    this.listen();
  }

  // data: {geometry (view), mesh (mesh view), meshStale, regions (names),
  // dirichlet (node indices), field ({label, values, scale?}), colorBy,
  // layers ({geometry, mesh, dirichlet, isolines, isolineCount}), probes}.
  // Draws only when something shown changed; returns whether it did (the
  // legend changes only then).
  set(data) {
    const previous = this.data;
    const keys = ["geometry", "mesh", "meshStale", "dirichlet", "colorBy", "probes"];
    const changed = keys.some((k) => previous[k] !== data[k]) || !sameLayers(previous.layers, data.layers) || !sameField(previous.field, data.field);
    if (!changed) return false;
    const meshChanged = data.mesh !== previous.mesh;
    const fieldChanged = !sameField(previous.field, data.field);
    const linesChanged = previous.layers?.isolines !== data.layers?.isolines || previous.layers?.isolineCount !== data.layers?.isolineCount;
    this.data = data;
    if (meshChanged) this.prepareMesh();
    if (meshChanged || fieldChanged) this.prepareField();
    if (meshChanged || fieldChanged || linesChanged) this.prepareIsolines();
    const box = this.boundingBox();
    const key = box ? JSON.stringify(box) : null;
    if (key && key !== this.fittedKey) {
      this.fittedKey = key;
      this.fit();
    } else this.draw();
    return true;
  }

  // What does not depend on the view, once per mesh.
  prepareMesh() {
    const mesh = this.data.mesh;
    this.angles = mesh ? triangleAngles(mesh) : null;
    this.grid = mesh ? meshGrid(mesh) : null;
    // Triangles by their quality colour, so each colour is one path.
    this.qualityBuckets = null;
    if (mesh) {
      const buckets = Array.from({ length: 256 }, () => []);
      for (let k = 0; k < this.angles.length; k++) buckets[Math.round(255 * (1 - Math.min(this.angles[k], QUALITY_MAX_DEGREES) / QUALITY_MAX_DEGREES))].push(k);
      this.qualityBuckets = buckets;
    }
  }

  // ... and once per field: whether it has a value at every node, and the
  // colour position of each node's value.
  prepareField() {
    const field = this.data.field;
    const mesh = this.data.mesh;
    this.fieldComplete = Boolean(field && mesh && field.values.length * 2 === mesh.nodes.length && Array.prototype.every.call(field.values, (v) => Number.isFinite(v)));
    this.positions = this.fieldComplete ? nodePositions(field.values, this.scaleOf(field.values)) : null;
  }

  // ... and the isolines of a complete field, at levels evenly inside its
  // colour scale (the scale of every snapshot, for a solution).
  prepareIsolines() {
    this.isolines = null;
    const { field, mesh, layers = {} } = this.data;
    if (!layers.isolines || !this.fieldComplete) return;
    const scale = this.scaleOf(field.values);
    const low = scale.kind === "diverging" ? -scale.bound : scale.min;
    const high = scale.kind === "diverging" ? scale.bound : scale.max;
    const levels = isolineLevels(low, high, layers.isolineCount ?? 10);
    this.isolines = { levels, segments: isolines(mesh.nodes, mesh.triangles, field.values, levels), positions: levels.map((v) => scale.position(v)) };
  }

  // The pixel ratio the picture is drawn at: the screen's, at most the
  // "picture quality" setting.
  pixelRatio() {
    return Math.min(window.devicePixelRatio || 1, settings.pixelRatioCap);
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
    const ratio = this.pixelRatio();
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
    this.cancelGesture();
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
      if (colorBy === "field" && this.fieldIsComplete()) this.rasterField(mesh);
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
    if (hasMesh && !meshStale && this.isolines) this.drawIsolines(colorBy === "field");
    if (hasMesh && !meshStale && colorBy === "field" && this.data.field && !this.fieldIsComplete()) this.drawFieldDots(mesh, this.data.field.values);
    if (hasMesh && !meshStale && layers.dirichlet && this.data.dirichlet) this.drawNodes(mesh, this.data.dirichlet);
    if (this.data.probes?.length) this.drawProbes(this.data.probes);
    if (meshStale) note = "mesh outdated: run the Mesh step";
    this.status.textContent = note;
    this.keepFrame();
  }

  fieldIsComplete() {
    return Boolean(this.data.field) && this.fieldComplete;
  }

  // --- gestures --------------------------------------------------------------

  keepFrame() {
    this.frame.width = this.canvas.width;
    this.frame.height = this.canvas.height;
    this.frame.getContext("2d").drawImage(this.canvas, 0, 0);
    this.frameView = { scale: this.scale, offset: { ...this.offset } };
  }

  cancelGesture() {
    if (this.blitRequest !== null) cancelAnimationFrame(this.blitRequest);
    this.blitRequest = null;
    clearTimeout(this.settleTimer);
    this.settleTimer = null;
  }

  // The view moved (pan or zoom): at most one frame per animation frame,
  // the last full picture moved and scaled; the full picture once the
  // gesture pauses.
  viewMoved() {
    this.userMoved = true;
    if (!this.frameView || this.frame.width !== this.canvas.width || this.frame.height !== this.canvas.height) return this.draw();
    if (this.blitRequest === null)
      this.blitRequest = requestAnimationFrame(() => {
        this.blitRequest = null;
        this.blit();
      });
    clearTimeout(this.settleTimer);
    this.settleTimer = setTimeout(() => this.draw(), SETTLE_MS);
  }

  blit() {
    const ctx = this.context;
    const r = this.ratio;
    const t = viewTransform(this.frameView, { scale: this.scale, offset: this.offset });
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.fillStyle = token("--surface");
    ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);
    ctx.drawImage(this.frame, t.x * r, t.y * r, this.frame.width * t.scale, this.frame.height * t.scale);
    ctx.setTransform(r, 0, 0, r, 0, 0);
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

  // The colour tables, once: each ramp goes through a middle stop, so its
  // middle is a clear colour, not a grey mix of the ends.
  colorTables() {
    const ramp = (...names) => rampTable(names.map((name) => parseColor(token(name))));
    this.tables ??= {
      sequential: ramp("--seq-low", "--seq-mid", "--seq-high"),
      diverging: ramp("--div-low", "--div-mid", "--div-high"),
      quality: ramp("--quality-good", "--quality-mid", "--quality-bad"),
    };
    return this.tables;
  }

  sequentialTable() {
    return this.colorTables().sequential;
  }

  divergingTable() {
    return this.colorTables().diverging;
  }

  // Each triangle filled by its smallest angle: bright for small angles,
  // which are the ones to look at. One path per colour.
  drawQuality(mesh) {
    const ctx = this.context;
    const table = this.colorTables().quality;
    const n = mesh.nodes;
    const t = mesh.triangles;
    const { x: ox, y: oy } = this.offset;
    const s = this.scale;
    ctx.lineWidth = 0.5;
    this.qualityBuckets.forEach((bucket, index) => {
      if (bucket.length === 0) return;
      ctx.beginPath();
      for (const k of bucket) {
        const a = t[3 * k], b = t[3 * k + 1], c = t[3 * k + 2];
        ctx.moveTo(ox + n[2 * a] * s, oy - n[2 * a + 1] * s);
        ctx.lineTo(ox + n[2 * b] * s, oy - n[2 * b + 1] * s);
        ctx.lineTo(ox + n[2 * c] * s, oy - n[2 * c + 1] * s);
        ctx.closePath();
      }
      ctx.fillStyle = `rgb(${table[3 * index]},${table[3 * index + 1]},${table[3 * index + 2]})`;
      ctx.strokeStyle = ctx.fillStyle;
      ctx.fill();
      ctx.stroke();
    });
  }

  // A field may bring its scale (a solution: one scale for every time).
  scaleOf(values) {
    return this.data.field?.scale ?? fieldScale(values);
  }

  fieldTable(scale) {
    return scale.kind === "diverging" ? this.divergingTable() : this.sequentialTable();
  }

  // A P1 field: every pixel inside a triangle gets the linear interpolant
  // of the three node values (of their colour positions, prepareField).
  rasterField(mesh) {
    const width = this.canvas.width;
    const height = this.canvas.height;
    const table = this.fieldTable(this.scaleOf(this.data.field.values));
    const positions = this.positions;
    if (this.offscreen.width !== width || this.offscreen.height !== height || !this.image) {
      this.offscreen.width = width;
      this.offscreen.height = height;
      this.image = this.offscreen.getContext("2d").createImageData(width, height);
    }
    const image = this.image;
    const data = image.data;
    data.fill(0);
    const n = mesh.nodes;
    const t = mesh.triangles;
    const r = this.ratio;
    const ox = this.offset.x * r;
    const oy = this.offset.y * r;
    const s = this.scale * r;
    const tolerance = -1e-6;
    for (let k = 0; k < t.length; k += 3) {
      const a = t[k], b = t[k + 1], c = t[k + 2];
      const ax = ox + n[2 * a] * s, ay = oy - n[2 * a + 1] * s;
      const bx = ox + n[2 * b] * s, by = oy - n[2 * b + 1] * s;
      const cx = ox + n[2 * c] * s, cy = oy - n[2 * c + 1] * s;
      const x0 = Math.max(0, Math.floor(Math.min(ax, bx, cx)));
      const x1 = Math.min(width - 1, Math.ceil(Math.max(ax, bx, cx)));
      const y0 = Math.max(0, Math.floor(Math.min(ay, by, cy)));
      const y1 = Math.min(height - 1, Math.ceil(Math.max(ay, by, cy)));
      if (x0 > x1 || y0 > y1) continue;
      const det = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy);
      if (det === 0) continue;
      const pa = positions[a], pb = positions[b], pc = positions[c];
      // l1 and l2 are linear in x along a row: step them.
      const d1x = (by - cy) / det, d2x = (cy - ay) / det;
      for (let py = y0; py <= y1; py++) {
        const y = py + 0.5;
        let l1 = (by - cy) * (x0 + 0.5 - cx) / det + ((cx - bx) * (y - cy)) / det;
        let l2 = (cy - ay) * (x0 + 0.5 - cx) / det + ((ax - cx) * (y - cy)) / det;
        let o = 4 * (py * width + x0);
        for (let px = x0; px <= x1; px++, l1 += d1x, l2 += d2x, o += 4) {
          if (l1 < tolerance || l2 < tolerance) continue;
          const l3 = 1 - l1 - l2;
          if (l3 < tolerance) continue;
          const p = l1 * pa + l2 * pb + l3 * pc;
          const index = 3 * (p <= 0 ? 0 : p >= 1 ? 255 : Math.round(255 * p));
          data[o] = table[index];
          data[o + 1] = table[index + 1];
          data[o + 2] = table[index + 2];
          data[o + 3] = 255;
        }
      }
    }
    this.offscreen.getContext("2d").putImageData(image, 0, 0);
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

  // Isolines: over the field's colours in one ink (lines in the field's
  // own colours would vanish into it), else each in its level's colour of
  // the same scale.
  drawIsolines(overField) {
    const ctx = this.context;
    const { segments, positions } = this.isolines;
    const table = this.fieldTable(this.scaleOf(this.data.field.values));
    const { x: ox, y: oy } = this.offset;
    const s = this.scale;
    ctx.lineCap = "round";
    segments.forEach((list, level) => {
      if (list.length === 0) return;
      ctx.beginPath();
      for (let i = 0; i < list.length; i += 4) {
        ctx.moveTo(ox + list[i] * s, oy - list[i + 1] * s);
        ctx.lineTo(ox + list[i + 2] * s, oy - list[i + 3] * s);
      }
      if (overField) {
        // A dark halo under a light line reads on every colour of a ramp.
        ctx.lineWidth = 3;
        ctx.strokeStyle = token("--isoline-halo");
        ctx.stroke();
        ctx.lineWidth = 1;
        ctx.strokeStyle = token("--isoline");
      } else {
        const k = 3 * Math.max(0, Math.min(255, Math.round(255 * positions[level])));
        ctx.lineWidth = 1.5;
        ctx.strokeStyle = `rgb(${table[k]},${table[k + 1]},${table[k + 2]})`;
      }
      ctx.stroke();
    });
  }

  // Each probe: a dot in its colour with a ring (readable on any field)
  // and its name.
  drawProbes(probes) {
    const ctx = this.context;
    ctx.font = `600 12px ${token("--font")}`;
    ctx.textBaseline = "middle";
    for (const probe of probes) {
      const [px, py] = this.toScreen(probe.x, probe.y);
      ctx.beginPath();
      ctx.arc(px, py, 6, 0, 2 * Math.PI);
      ctx.fillStyle = token(`--series-${probe.slot + 1}`);
      ctx.fill();
      ctx.lineWidth = 2;
      ctx.strokeStyle = token("--surface");
      ctx.stroke();
      ctx.beginPath();
      ctx.arc(px, py, 8, 0, 2 * Math.PI);
      ctx.lineWidth = 1;
      ctx.strokeStyle = token("--ink");
      ctx.stroke();
      const name = `P${probe.slot + 1}`;
      ctx.lineWidth = 3;
      ctx.strokeStyle = token("--surface");
      ctx.strokeText(name, px + 11, py);
      ctx.fillStyle = token("--ink");
      ctx.fillText(name, px + 11, py);
    }
  }

  // --- the legend ------------------------------------------------------------

  // What the legend shows, as data (the page's legend and the saved
  // picture draw it): regions, a colour bar, the Dirichlet nodes.
  legendData() {
    const { geometry, mesh, meshStale, colorBy, field, layers = {} } = this.data;
    const current = mesh && !meshStale;
    const names = current ? mesh.region_names : (geometry?.regions ?? []).map((r) => r.name);
    let bar = null;
    if (current && colorBy === "quality")
      bar = { label: "smallest angle", table: this.colorTables().quality, low: "0°", high: `${QUALITY_MAX_DEGREES}°+`, reversed: true };
    if (current && colorBy === "field" && field) {
      const scale = this.scaleOf(field.values);
      const diverging = scale.kind === "diverging";
      bar = {
        label: field.label,
        table: this.fieldTable(scale),
        low: formatNumber(diverging ? -scale.bound : scale.min),
        high: formatNumber(diverging ? scale.bound : scale.max),
        reversed: false,
      };
    }
    return { regions: names, bar, dirichlet: Boolean(layers.dirichlet && this.data.dirichlet && current) };
  }

  legend() {
    const { regions, bar, dirichlet } = this.legendData();
    const items = regions.map((name, i) => h("span", { class: "item" }, regionSwatch(i), name));
    if (bar) items.push(colorbar(bar.label, bar.table, bar.low, bar.high, bar.reversed));
    if (dirichlet) items.push(h("span", { class: "item" }, h("span", { class: "swatch node", "aria-hidden": "true" }), "Dirichlet node"));
    return items;
  }

  // The picture as it is shown, with its legend underneath, as a PNG blob.
  async picture() {
    this.draw();
    const ratio = this.ratio;
    const width = this.canvas.width;
    const { regions, bar, dirichlet } = this.legendData();
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
    const legendHeight = pad + rows.length * line + (bar ? line + 6 * ratio : 0) + pad;
    out.height = this.canvas.height + legendHeight;
    ctx.fillStyle = token("--surface");
    ctx.fillRect(0, 0, out.width, out.height);
    ctx.drawImage(this.canvas, 0, 0);
    ctx.font = font;
    ctx.textBaseline = "middle";
    let y = this.canvas.height + pad + line / 2;
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
      ctx.fillText(bar.low, bx, y);
      bx += ctx.measureText(bar.low).width + 8 * ratio;
      const barWidth = Math.min(240 * ratio, Math.max(60 * ratio, width - bx - ctx.measureText(bar.high).width - pad - 8 * ratio));
      for (let i = 0; i < barWidth; i++) {
        const p = i / Math.max(1, barWidth - 1);
        const k = 3 * Math.round(255 * (bar.reversed ? 1 - p : p));
        ctx.fillStyle = `rgb(${bar.table[k]},${bar.table[k + 1]},${bar.table[k + 2]})`;
        ctx.fillRect(bx + i, y - 5 * ratio, 1, 10 * ratio);
      }
      bx += barWidth + 8 * ratio;
      ctx.fillStyle = token("--ink-2");
      ctx.fillText(bar.high, bx, y);
    }
    return new Promise((resolve) => out.toBlob(resolve, "image/png"));
  }

  // --- interaction -----------------------------------------------------------

  listen() {
    const canvas = this.canvas;
    let drag = null;
    canvas.addEventListener("pointerdown", (event) => {
      drag = { x: event.clientX, y: event.clientY, offset: { ...this.offset }, moved: false };
      canvas.setPointerCapture(event.pointerId);
    });
    canvas.addEventListener("pointermove", (event) => {
      if (drag) {
        const dx = event.clientX - drag.x;
        const dy = event.clientY - drag.y;
        if (!drag.moved && Math.hypot(dx, dy) < CLICK_SLOP) return;
        drag.moved = true;
        canvas.classList.add("dragging");
        this.offset = { x: drag.offset.x + dx, y: drag.offset.y + dy };
        this.tooltip.hidden = true;
        this.viewMoved();
      } else this.hover(event);
    });
    canvas.addEventListener("pointerup", (event) => {
      const click = drag && !drag.moved;
      drag = null;
      canvas.classList.remove("dragging");
      if (click && event.button === 0) {
        const rect = canvas.getBoundingClientRect();
        this.pin(event.clientX - rect.left, event.clientY - rect.top);
      }
    });
    canvas.addEventListener("pointercancel", () => {
      drag = null;
      canvas.classList.remove("dragging");
    });
    canvas.addEventListener("pointerleave", () => (this.tooltip.hidden = true));
    canvas.addEventListener(
      "wheel",
      (event) => {
        event.preventDefault();
        const rect = canvas.getBoundingClientRect();
        const factor = Math.exp(-event.deltaY * (event.deltaMode === 1 ? 0.05 : 0.0015));
        this.zoomAt(event.clientX - rect.left, event.clientY - rect.top, factor);
        this.hover(event);
      },
      { passive: false },
    );
    canvas.addEventListener("dblclick", () => this.fit());
    canvas.addEventListener("keydown", (event) => this.key(event));
  }

  zoomAt(px, py, factor) {
    const [wx, wy] = this.toWorld(px, py);
    this.scale *= factor;
    this.offset = { x: px - wx * this.scale, y: py + wy * this.scale };
    this.viewMoved();
  }

  // Keyboard: arrows pan, + and - zoom about the centre, 0 fits, Enter
  // pins a probe at the centre.
  key(event) {
    const { clientWidth: width, clientHeight: height } = this.canvas.parentElement;
    const step = KEY_PAN_PX * (event.shiftKey ? 4 : 1);
    const pan = { ArrowLeft: [step, 0], ArrowRight: [-step, 0], ArrowUp: [0, step], ArrowDown: [0, -step] }[event.key];
    if (pan) {
      this.offset = { x: this.offset.x + pan[0], y: this.offset.y + pan[1] };
      this.viewMoved();
    } else if (event.key === "+" || event.key === "=") this.zoomAt(width / 2, height / 2, 1.25);
    else if (event.key === "-" || event.key === "_") this.zoomAt(width / 2, height / 2, 0.8);
    else if (event.key === "0") this.fit();
    else if (event.key === "Enter") this.pin(width / 2, height / 2);
    else return;
    event.preventDefault();
  }

  pin(px, py) {
    if (!this.onPin || !this.data.mesh || this.data.meshStale) return;
    const [x, y] = this.toWorld(px, py);
    this.onPin(x, y);
  }

  // The readout under the pointer: coordinates, region, triangle and the
  // field's value (for a dot field, the value of the nearest dot).
  hover(event) {
    const rect = this.canvas.getBoundingClientRect();
    const px = event.clientX - rect.left;
    const py = event.clientY - rect.top;
    const [x, y] = this.toWorld(px, py);
    const rows = [["x, y", `${formatNumber(x)}, ${formatNumber(y)}`]];
    const digits = getDefaultDigits();
    const { mesh, meshStale, field, geometry, colorBy } = this.data;
    if (!mesh && !geometry) {
      this.tooltip.hidden = true;
      return;
    }
    const region = this.regionNear(px, py);
    if (region) rows.push(["region", region]);
    if (mesh && !meshStale && this.grid) {
      const found = locatePoint(mesh, this.grid, x, y);
      if (found) {
        const angle = formatNumber(this.angles[found.triangle], Math.max(2, digits - 1));
        rows.push(["triangle", `${found.triangle} · smallest angle ${angle}°`]);
        if (field && this.fieldIsComplete()) {
          const v = found.weights.reduce((sum, w, k) => sum + w * field.values[found.nodes[k]], 0);
          rows.push([field.label, formatNumber(v, digits + 2)]);
        }
      }
      if (field && colorBy === "field" && !this.fieldIsComplete()) {
        const node = nearestValuedNode(mesh.nodes, field.values, x, y, DOT_PICK_PX / this.scale);
        if (node >= 0) rows.push([`${field.label} at node ${node}`, formatNumber(field.values[node], digits + 2)]);
      }
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
