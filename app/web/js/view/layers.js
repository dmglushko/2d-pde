// The layers of the picture, each a function that draws on the viewer's
// canvas (`v`: the Viewer, for its context, view transform and prepared
// data): the geometry, the mesh edges and boundary, the colour layers
// (smallest angle, a P1 field, |∇u| per triangle, values at some nodes),
// isolines, nodes, probes and the line of a profile. The only state is
// the edge layer kept on the viewer (v.edgeCache).

import { regionToken } from "../pure/color.js";
import { meshEdges } from "../pure/mesh_math.js";
import { colorTables, tableColor, token } from "./tokens.js";

export const QUALITY_MAX_DEGREES = 60;

function regionColors(count) {
  return Array.from({ length: count }, (_, i) => token(regionToken(i)));
}

export function fillGeometry(v, geometry) {
  const ctx = v.context;
  ctx.beginPath();
  for (const loop of geometry.loops) {
    let first = true;
    for (const curve of loop.curves) {
      const p = curve.points;
      for (let i = 0; i < p.length; i += 2) {
        const [x, y] = v.toScreen(p[i], p[i + 1]);
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

export function drawCurves(v, geometry) {
  const ctx = v.context;
  const names = geometry.regions.map((r) => r.name);
  const colors = regionColors(names.length);
  ctx.lineWidth = 2;
  ctx.lineJoin = "round";
  ctx.lineCap = "round";
  for (const loop of geometry.loops)
    for (const curve of loop.curves) {
      const p = curve.points;
      ctx.beginPath();
      for (let i = 0; i < p.length; i += 2) {
        const [x, y] = v.toScreen(p[i], p[i + 1]);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.strokeStyle = colors[names.indexOf(curve.region)];
      ctx.stroke();
    }
}

export function drawEdgeList(v, view) {
  const ctx = v.context;
  const colors = regionColors(view.regions.length);
  ctx.lineWidth = 2;
  for (const [ax, ay, bx, by, region] of view.boundary_edges) {
    ctx.beginPath();
    ctx.moveTo(...v.toScreen(ax, ay));
    ctx.lineTo(...v.toScreen(bx, by));
    ctx.strokeStyle = colors[region];
    ctx.stroke();
  }
}

export function drawMeshBoundary(v, mesh) {
  const ctx = v.context;
  const colors = regionColors(mesh.region_names.length);
  const n = mesh.nodes;
  const e = mesh.boundary_edges;
  ctx.lineWidth = 2;
  ctx.lineCap = "round";
  // One path per region (a stroke per edge is slow on a fine mesh).
  for (let region = 0; region < colors.length; region++) {
    ctx.beginPath();
    for (let i = 0; i < e.length; i += 3) {
      if (e[i + 2] !== region) continue;
      ctx.moveTo(...v.toScreen(n[2 * e[i]], n[2 * e[i] + 1]));
      ctx.lineTo(...v.toScreen(n[2 * e[i + 1]], n[2 * e[i + 1] + 1]));
    }
    ctx.strokeStyle = colors[region];
    ctx.stroke();
  }
}

// The mesh edges at `alpha` (fainter over a field's colours, and for an
// outdated mesh). Returns false when the triangles are too small on
// screen to draw. The edges are drawn once per mesh and view into a
// layer of their own (edgeLayer) and copied from it, so a new snapshot,
// another step or a key typed does not stroke them again.
export function drawEdges(v, mesh, alpha) {
  const count = mesh.triangles.length / 3;
  const { clientWidth, clientHeight } = v.canvas.parentElement;
  const box = v.boundingBox();
  const area = (box.max[0] - box.min[0]) * (box.max[1] - box.min[1]) * v.scale * v.scale;
  if (count > 0 && area / count < 6 && area < 4 * clientWidth * clientHeight) return false;
  const layer = edgeLayer(v, mesh);
  const ctx = v.context;
  ctx.save();
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.globalAlpha = alpha;
  ctx.drawImage(layer, 0, 0);
  ctx.restore();
  return true;
}

// Segments per stroke: one path of every edge makes the stroke slow
// (seconds for a few ten thousand triangles); short paths stay fast.
const EDGES_PER_PATH = 2048;

// The layer of the mesh edges, opaque in the edge colour, at the view
// and size of the picture: drawn again only when one of them changed.
function edgeLayer(v, mesh) {
  const colour = token("--mesh-edge");
  const key = `${v.canvas.width}x${v.canvas.height}|${v.ratio}|${v.scale}|${v.offset.x}|${v.offset.y}|${colour}`;
  const cache = (v.edgeCache ??= { canvas: document.createElement("canvas"), mesh: null, edges: null, key: null });
  if (cache.mesh === mesh && cache.key === key) return cache.canvas;
  if (cache.mesh !== mesh) cache.edges = meshEdges(mesh.triangles, mesh.nodes.length / 2);
  cache.mesh = mesh;
  cache.key = key;
  const canvas = cache.canvas;
  canvas.width = v.canvas.width;
  canvas.height = v.canvas.height;
  const ctx = canvas.getContext("2d");
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  const n = mesh.nodes;
  const e = cache.edges;
  const r = v.ratio;
  const ox = v.offset.x * r;
  const oy = v.offset.y * r;
  const s = v.scale * r;
  ctx.strokeStyle = colour;
  ctx.lineWidth = 1;
  for (let start = 0; start < e.length; start += 2 * EDGES_PER_PATH) {
    const end = Math.min(e.length, start + 2 * EDGES_PER_PATH);
    ctx.beginPath();
    for (let i = start; i < end; i += 2) {
      ctx.moveTo(ox + n[2 * e[i]] * s, oy - n[2 * e[i] + 1] * s);
      ctx.lineTo(ox + n[2 * e[i + 1]] * s, oy - n[2 * e[i + 1] + 1] * s);
    }
    ctx.stroke();
  }
  return canvas;
}

// Triangles grouped by the colour index (0..255) of a value per triangle,
// so each colour is one path. `position(k)` is triangle k's colour
// position in [0, 1], NaN for none (left out).
export function bucketTriangles(count, position) {
  const buckets = Array.from({ length: 256 }, () => []);
  for (let k = 0; k < count; k++) {
    const p = position(k);
    if (!Number.isFinite(p)) continue;
    buckets[Math.max(0, Math.min(255, Math.round(255 * p)))].push(k);
  }
  return buckets;
}

// Each triangle filled with one colour of `table`, by its bucket.
export function fillBuckets(v, mesh, buckets, table) {
  const ctx = v.context;
  const n = mesh.nodes;
  const t = mesh.triangles;
  const { x: ox, y: oy } = v.offset;
  const s = v.scale;
  ctx.lineWidth = 0.5;
  buckets.forEach((bucket, index) => {
    if (bucket.length === 0) return;
    ctx.beginPath();
    for (const k of bucket) {
      const a = t[3 * k],
        b = t[3 * k + 1],
        c = t[3 * k + 2];
      ctx.moveTo(ox + n[2 * a] * s, oy - n[2 * a + 1] * s);
      ctx.lineTo(ox + n[2 * b] * s, oy - n[2 * b + 1] * s);
      ctx.lineTo(ox + n[2 * c] * s, oy - n[2 * c + 1] * s);
      ctx.closePath();
    }
    ctx.fillStyle = `rgb(${table[3 * index]},${table[3 * index + 1]},${table[3 * index + 2]})`;
    // A hairline in the same colour closes the seams between triangles.
    ctx.strokeStyle = ctx.fillStyle;
    ctx.fill();
    ctx.stroke();
  });
}

// Each triangle filled by its smallest angle: bright for small angles,
// which are the ones to look at.
export function drawQuality(v, mesh) {
  fillBuckets(v, mesh, v.qualityBuckets, colorTables().quality);
}

// A P1 field: every pixel inside a triangle gets the linear interpolant
// of the three node values (of their colour positions, `positions`).
export function rasterField(v, mesh, positions, table) {
  const width = v.canvas.width;
  const height = v.canvas.height;
  if (v.offscreen.width !== width || v.offscreen.height !== height || !v.image) {
    v.offscreen.width = width;
    v.offscreen.height = height;
    v.image = v.offscreen.getContext("2d").createImageData(width, height);
  }
  const image = v.image;
  const data = image.data;
  data.fill(0);
  const n = mesh.nodes;
  const t = mesh.triangles;
  const r = v.ratio;
  const ox = v.offset.x * r;
  const oy = v.offset.y * r;
  const s = v.scale * r;
  const tolerance = -1e-6;
  for (let k = 0; k < t.length; k += 3) {
    const a = t[k],
      b = t[k + 1],
      c = t[k + 2];
    const ax = ox + n[2 * a] * s,
      ay = oy - n[2 * a + 1] * s;
    const bx = ox + n[2 * b] * s,
      by = oy - n[2 * b + 1] * s;
    const cx = ox + n[2 * c] * s,
      cy = oy - n[2 * c + 1] * s;
    const x0 = Math.max(0, Math.floor(Math.min(ax, bx, cx)));
    const x1 = Math.min(width - 1, Math.ceil(Math.max(ax, bx, cx)));
    const y0 = Math.max(0, Math.floor(Math.min(ay, by, cy)));
    const y1 = Math.min(height - 1, Math.ceil(Math.max(ay, by, cy)));
    if (x0 > x1 || y0 > y1) continue;
    const det = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy);
    if (det === 0) continue;
    const pa = positions[a],
      pb = positions[b],
      pc = positions[c];
    // l1 and l2 are linear in x along a row: step them.
    const d1x = (by - cy) / det,
      d2x = (cy - ay) / det;
    for (let py = y0; py <= y1; py++) {
      const y = py + 0.5;
      let l1 = ((by - cy) * (x0 + 0.5 - cx)) / det + ((cx - bx) * (y - cy)) / det;
      let l2 = ((cy - ay) * (x0 + 0.5 - cx)) / det + ((ax - cx) * (y - cy)) / det;
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
  v.offscreen.getContext("2d").putImageData(image, 0, 0);
  const ctx = v.context;
  ctx.save();
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.drawImage(v.offscreen, 0, 0);
  ctx.restore();
}

// A field that has values at some nodes only: one dot per value.
export function drawFieldDots(v, mesh, values, scale, table) {
  const ctx = v.context;
  const ring = token("--surface");
  for (let i = 0; i < values.length; i++) {
    if (!Number.isFinite(values[i])) continue;
    ctx.beginPath();
    ctx.arc(...v.toScreen(mesh.nodes[2 * i], mesh.nodes[2 * i + 1]), 4.5, 0, 2 * Math.PI);
    ctx.fillStyle = tableColor(table, scale.position(values[i]));
    ctx.fill();
    ctx.lineWidth = 1.5;
    ctx.strokeStyle = ring;
    ctx.stroke();
  }
}

export function drawNodes(v, mesh, nodes) {
  const ctx = v.context;
  ctx.fillStyle = token("--ink");
  for (const i of nodes) {
    ctx.beginPath();
    ctx.arc(...v.toScreen(mesh.nodes[2 * i], mesh.nodes[2 * i + 1]), 2.5, 0, 2 * Math.PI);
    ctx.fill();
  }
}

// Isolines: over the field's colours in one ink (lines in the field's
// own colours would vanish into it), else each in its level's colour of
// the same scale.
export function drawIsolines(v, isolines, table, overField) {
  const ctx = v.context;
  const { segments, positions } = isolines;
  const { x: ox, y: oy } = v.offset;
  const s = v.scale;
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
      ctx.lineWidth = 1.5;
      ctx.strokeStyle = tableColor(table, positions[level]);
    }
    ctx.stroke();
  });
}

// A label next to a point, readable on any colour (a halo of the surface).
function haloText(ctx, text, x, y) {
  ctx.lineWidth = 3;
  ctx.strokeStyle = token("--surface");
  ctx.strokeText(text, x, y);
  ctx.fillStyle = token("--ink");
  ctx.fillText(text, x, y);
}

// Each probe: a dot in its colour with a ring (readable on any field)
// and its name.
export function drawProbes(v, probes) {
  const ctx = v.context;
  ctx.font = `600 12px ${token("--font")}`;
  ctx.textBaseline = "middle";
  for (const probe of probes) {
    const [px, py] = v.toScreen(probe.x, probe.y);
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
    haloText(ctx, `P${probe.slot + 1}`, px + 11, py);
  }
}

// The line of the profile from A to B: an ink line on a dark halo, with
// its ends named.
export function drawProfileLine(v, line) {
  const ctx = v.context;
  const [ax, ay] = v.toScreen(...line.from);
  const [bx, by] = v.toScreen(...line.to);
  ctx.lineCap = "round";
  ctx.beginPath();
  ctx.moveTo(ax, ay);
  ctx.lineTo(bx, by);
  ctx.lineWidth = 5;
  ctx.strokeStyle = token("--isoline-halo");
  ctx.stroke();
  ctx.lineWidth = 2;
  ctx.strokeStyle = token("--ink");
  ctx.stroke();
  ctx.font = `600 12px ${token("--font")}`;
  ctx.textBaseline = "middle";
  for (const [x, y, name] of [
    [ax, ay, "A"],
    [bx, by, "B"],
  ]) {
    ctx.beginPath();
    ctx.arc(x, y, 4, 0, 2 * Math.PI);
    ctx.fillStyle = token("--ink");
    ctx.fill();
    haloText(ctx, name, x + 8, y - 10);
  }
}

// The boundary region the user selected (a click on it, or its legend
// entry): drawn wider on a dark halo, in its own colour, so it stands out
// on any field.
export function drawSelectedRegion(v, mesh, geometry, name) {
  const segments = [];
  let index = -1;
  if (mesh) {
    index = mesh.region_names.indexOf(name);
    const n = mesh.nodes;
    const e = mesh.boundary_edges;
    for (let i = 0; i < e.length; i += 3) if (e[i + 2] === index) segments.push([n[2 * e[i]], n[2 * e[i] + 1], n[2 * e[i + 1]], n[2 * e[i + 1] + 1]]);
  } else if (geometry?.kind === "curves") {
    index = geometry.regions.findIndex((r) => r.name === name);
    for (const loop of geometry.loops)
      for (const curve of loop.curves) {
        if (curve.region !== name) continue;
        const p = curve.points;
        for (let i = 0; i + 3 < p.length; i += 2) segments.push([p[i], p[i + 1], p[i + 2], p[i + 3]]);
      }
  } else if (geometry?.kind === "mesh_file") {
    index = geometry.regions.findIndex((r) => r.name === name);
    for (const [ax, ay, bx, by, region] of geometry.boundary_edges) if (region === index) segments.push([ax, ay, bx, by]);
  }
  if (index < 0 || segments.length === 0) return;
  const ctx = v.context;
  ctx.beginPath();
  for (const [ax, ay, bx, by] of segments) {
    ctx.moveTo(...v.toScreen(ax, ay));
    ctx.lineTo(...v.toScreen(bx, by));
  }
  ctx.lineCap = "round";
  ctx.lineJoin = "round";
  ctx.lineWidth = 9;
  ctx.strokeStyle = token("--isoline-halo");
  ctx.stroke();
  ctx.lineWidth = 5;
  ctx.strokeStyle = token(regionToken(index));
  ctx.stroke();
}
