// Mesh geometry: angles, point location, interpolation, isolines, the
// binary mesh, view transforms. Pure.

// The smallest corner angle of a triangle, in degrees.
export function smallestAngle(ax, ay, bx, by, cx, cy) {
  const corner = (px, py, qx, qy, rx, ry) => {
    const ux = qx - px;
    const uy = qy - py;
    const vx = rx - px;
    const vy = ry - py;
    return (Math.atan2(Math.abs(ux * vy - uy * vx), ux * vx + uy * vy) * 180) / Math.PI;
  };
  return Math.min(corner(ax, ay, bx, by, cx, cy), corner(bx, by, cx, cy, ax, ay), corner(cx, cy, ax, ay, bx, by));
}

// One smallest angle per triangle of a mesh view (flat arrays).
export function triangleAngles(mesh) {
  const { nodes, triangles } = mesh;
  const out = new Float64Array(triangles.length / 3);
  for (let t = 0; t < out.length; t++) {
    const a = triangles[3 * t];
    const b = triangles[3 * t + 1];
    const c = triangles[3 * t + 2];
    out[t] = smallestAngle(nodes[2 * a], nodes[2 * a + 1], nodes[2 * b], nodes[2 * b + 1], nodes[2 * c], nodes[2 * c + 1]);
  }
  return out;
}

// Barycentric coordinates of (x, y) in triangle (a, b, c), or null for a
// degenerate triangle.
export function barycentric(x, y, ax, ay, bx, by, cx, cy) {
  const det = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy);
  if (det === 0) return null;
  const l1 = ((by - cy) * (x - cx) + (cx - bx) * (y - cy)) / det;
  const l2 = ((cy - ay) * (x - cx) + (ax - cx) * (y - cy)) / det;
  return [l1, l2, 1 - l1 - l2];
}

// Where a picture drawn at view `from` ({scale, offset: {x, y}}, CSS
// pixels) lands at view `to`: drawn scaled by `scale` with its top left
// corner at (x, y). A point at p on the old picture is at
// to.offset + (p - from.offset) * scale.
export function viewTransform(from, to) {
  const scale = to.scale / from.scale;
  return { scale, x: to.offset.x - from.offset.x * scale, y: to.offset.y - from.offset.y * scale };
}

// The mesh from its summary (POST /api/mesh, "mesh") and its arrays
// (POST /api/mesh/arrays): nodes as a Float64Array [x0, y0, ...],
// triangles as a Uint32Array [a0, b0, c0, ...] and boundary_edges as a
// Uint32Array [a0, b0, region0, ...], with region_names, has_links and id.
// Throws when the bytes do not fit the summary.
export function decodeMesh(summary, buffer) {
  const arrays = summary.arrays;
  if (buffer.byteLength !== arrays.bytes) throw new Error(`the mesh has ${buffer.byteLength} bytes, expected ${arrays.bytes}`);
  const view = new DataView(buffer);
  const part = (name) => {
    const found = arrays.parts.find((p) => p.name === name);
    if (!found) throw new Error(`the mesh has no ${name}`);
    return found;
  };
  const read = (name, Type, size, get) => {
    const p = part(name);
    const out = new Type(p.count);
    for (let i = 0; i < p.count; i++) out[i] = get(p.offset + size * i);
    return out;
  };
  const nodes = read("nodes", Float64Array, 8, (at) => view.getFloat64(at, true));
  const triangles = read("triangles", Uint32Array, 4, (at) => view.getUint32(at, true));
  const ends = read("boundary_edges", Uint32Array, 4, (at) => view.getUint32(at, true));
  const regions = read("boundary_regions", Uint32Array, 4, (at) => view.getUint32(at, true));
  const edges = new Uint32Array(3 * regions.length);
  for (let e = 0; e < regions.length; e++) {
    edges[3 * e] = ends[2 * e];
    edges[3 * e + 1] = ends[2 * e + 1];
    edges[3 * e + 2] = regions[e];
  }
  return { id: summary.id, nodes, triangles, boundary_edges: edges, region_names: summary.region_names, has_links: summary.has_links };
}

// A uniform grid of triangle indices by bounding box, for finding the
// triangle that holds a point. `mesh` is a mesh view (flat arrays).
export function meshGrid(mesh) {
  const n = mesh.nodes;
  const t = mesh.triangles;
  const count = t.length / 3;
  if (count === 0) return null;
  let minX = Infinity;
  let minY = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
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
    const i0 = cell(Math.min(...xs), minX, cellW);
    const i1 = cell(Math.max(...xs), minX, cellW);
    const j0 = cell(Math.min(...ys), minY, cellH);
    const j1 = cell(Math.max(...ys), minY, cellH);
    for (let j = j0; j <= j1; j++) for (let i = i0; i <= i1; i++) cells[j * side + i].push(k);
  }
  return { minX, minY, maxX, maxY, side, cellW, cellH, cells };
}

// The triangle that holds (x, y) and the point's barycentric weights in
// it: {triangle, nodes: [a, b, c], weights: [wa, wb, wc]}, or null
// outside the mesh. The first triangle in index order wins on an edge.
export function locatePoint(mesh, grid, x, y) {
  if (!grid || x < grid.minX || x > grid.maxX || y < grid.minY || y > grid.maxY) return null;
  const i = Math.min(grid.side - 1, Math.floor((x - grid.minX) / grid.cellW));
  const j = Math.min(grid.side - 1, Math.floor((y - grid.minY) / grid.cellH));
  const n = mesh.nodes;
  const t = mesh.triangles;
  for (const k of grid.cells[j * grid.side + i]) {
    const a = t[3 * k];
    const b = t[3 * k + 1];
    const c = t[3 * k + 2];
    const w = barycentric(x, y, n[2 * a], n[2 * a + 1], n[2 * b], n[2 * b + 1], n[2 * c], n[2 * c + 1]);
    if (w && w[0] >= -1e-9 && w[1] >= -1e-9 && w[2] >= -1e-9) return { triangle: k, nodes: [a, b, c], weights: w };
  }
  return null;
}

// The P1 interpolant of nodal `values` at a located point (locatePoint),
// or NaN when the point is outside or a value is missing.
export function interpolateAt(found, values) {
  if (!found) return NaN;
  let sum = 0;
  for (let k = 0; k < 3; k++) {
    const v = values[found.nodes[k]];
    if (v === null || !Number.isFinite(v)) return NaN;
    sum += found.weights[k] * v;
  }
  return sum;
}


// The node nearest to (x, y) among those with a finite value, within
// `radius` (world units), or -1. For fields given at some nodes only
// (Dirichlet values, b), which the picture draws as dots.
export function nearestValuedNode(nodes, values, x, y, radius) {
  let best = -1;
  let bestDistance = radius * radius;
  for (let i = 0; i < values.length; i++) {
    const v = values[i];
    if (v === null || !Number.isFinite(v)) continue;
    const dx = nodes[2 * i] - x;
    const dy = nodes[2 * i + 1] - y;
    const d = dx * dx + dy * dy;
    if (d <= bestDistance) {
      // Ties go to the lower index (the loop runs in index order).
      if (d < bestDistance || best < 0) best = i;
      bestDistance = d;
    }
  }
  return best;
}

// `count` levels evenly inside [min, max] (not at the ends, where a level
// would touch only single nodes). Empty for a constant field.
export function isolineLevels(min, max, count) {
  if (!(max > min) || !Number.isInteger(count) || count < 1) return [];
  return Array.from({ length: count }, (_, k) => min + ((k + 1) * (max - min)) / (count + 1));
}

// Marching triangles on a P1 field: per level, the segments where the
// linear interpolant equals it, as one Float64Array of x1, y1, x2, y2,
// ... A node counts as above a level when its value is >= the level, so
// a level through a node gives each segment once. Triangles with a
// missing value are skipped.
export function isolines(nodes, triangles, values, levels) {
  return levels.map((level) => {
    const out = [];
    for (let k = 0; k < triangles.length; k += 3) {
      const ids = [triangles[k], triangles[k + 1], triangles[k + 2]];
      const v = ids.map((i) => values[i]);
      if (!v.every((value) => value !== null && Number.isFinite(value))) continue;
      const above = v.map((value) => value >= level);
      const count = above.filter(Boolean).length;
      if (count === 0 || count === 3) continue;
      // Each edge whose ends lie on both sides has one crossing.
      for (let e = 0; e < 3; e++) {
        const p = e;
        const q = (e + 1) % 3;
        if (above[p] === above[q]) continue;
        const s = (level - v[p]) / (v[q] - v[p]);
        out.push(nodes[2 * ids[p]] + s * (nodes[2 * ids[q]] - nodes[2 * ids[p]]), nodes[2 * ids[p] + 1] + s * (nodes[2 * ids[q] + 1] - nodes[2 * ids[p] + 1]));
      }
    }
    return Float64Array.from(out);
  });
}

// --- post-processing of a shown field -------------------------------------------

// |∇u| of the P1 field `values` on each triangle (constant there), as a
// Float64Array; NaN where a value is missing or a triangle is degenerate.
export function gradientMagnitudes(nodes, triangles, values) {
  const out = new Float64Array(triangles.length / 3);
  for (let k = 0; k < out.length; k++) {
    const a = triangles[3 * k];
    const b = triangles[3 * k + 1];
    const c = triangles[3 * k + 2];
    const ua = values[a];
    const ub = values[b];
    const uc = values[c];
    if (![ua, ub, uc].every((v) => v !== null && Number.isFinite(v))) {
      out[k] = NaN;
      continue;
    }
    const bx = nodes[2 * b] - nodes[2 * a];
    const by = nodes[2 * b + 1] - nodes[2 * a + 1];
    const cx = nodes[2 * c] - nodes[2 * a];
    const cy = nodes[2 * c + 1] - nodes[2 * a + 1];
    const det = bx * cy - cx * by;
    if (det === 0) {
      out[k] = NaN;
      continue;
    }
    const gx = ((ub - ua) * cy - (uc - ua) * by) / det;
    const gy = ((uc - ua) * bx - (ub - ua) * cx) / det;
    out[k] = Math.hypot(gx, gy);
  }
  return out;
}

// The P1 interpolant of `values` along the segment from `from` to `to`
// ([x, y] each) at `count` + 1 evenly spaced points: {distance, values}
// (Float64Arrays; distance from `from`, NaN outside the mesh). `grid` is
// meshGrid(mesh).
export function sampleLine(mesh, grid, values, from, to, count = 200) {
  const n = Math.max(1, Math.floor(count));
  const length = Math.hypot(to[0] - from[0], to[1] - from[1]);
  const distance = new Float64Array(n + 1);
  const out = new Float64Array(n + 1);
  for (let i = 0; i <= n; i++) {
    const s = i / n;
    distance[i] = s * length;
    const x = from[0] + s * (to[0] - from[0]);
    const y = from[1] + s * (to[1] - from[1]);
    out[i] = interpolateAt(locatePoint(mesh, grid, x, y), values);
  }
  return { distance, values: out };
}
