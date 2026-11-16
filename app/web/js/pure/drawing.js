// The drawing source of the geometry (docs/app.md, "The drawing"): its
// curves, hit testing, snapping and the edits the editor makes. Pure: a
// drawing is the case's geometry section ({source: "drawing", loops:
// [{vertices: [[x, y], ...], edges: [{kind, region, ...}, ...]}]}), and
// every edit returns a new one. The server reads it with the library; the
// page evaluates its curves only to draw them and to place what the user
// points at.
//
// Edge i of a loop runs from vertex i to vertex i + 1 (the last to vertex
// 0). A line or a Bezier takes its ends from the vertices; an arc keeps
// the library's centre form (center, radius_x, radius_y, rotation,
// start_angle, sweep_angle), and its ends lie on its vertices.

const TAU = 2 * Math.PI;
// How far a straight edge bulges (over its length) when it becomes a
// curve.
const BULGE = 0.15;
// Points per curved edge when one is drawn or searched.
export const CURVE_SAMPLES = 64;

const add = (a, b) => [a[0] + b[0], a[1] + b[1]];
const sub = (a, b) => [a[0] - b[0], a[1] - b[1]];
const scale = (s, a) => [s * a[0], s * a[1]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1];
const lerp = (a, b, t) => [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])];
const distance = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1]);

// --- curves ------------------------------------------------------------------

// The curve of edge `e` of `loop`: {kind, start, end, ...} with the ends
// taken from the vertices (an arc keeps its own).
export function edgeCurve(loop, e) {
  const n = loop.vertices.length;
  const edge = loop.edges[e];
  return { ...edge, start: loop.vertices[e], end: loop.vertices[(e + 1) % n] };
}

export function pointAt(curve, t) {
  const { start: a, end: b } = curve;
  if (curve.kind === "quadratic") {
    const s = 1 - t;
    const q = curve.control;
    return [s * s * a[0] + 2 * s * t * q[0] + t * t * b[0], s * s * a[1] + 2 * s * t * q[1] + t * t * b[1]];
  }
  if (curve.kind === "cubic") {
    const s = 1 - t;
    const [c1, c2] = [curve.control1, curve.control2];
    const w = [s * s * s, 3 * s * s * t, 3 * s * t * t, t * t * t];
    return [w[0] * a[0] + w[1] * c1[0] + w[2] * c2[0] + w[3] * b[0], w[0] * a[1] + w[1] * c1[1] + w[2] * c2[1] + w[3] * b[1]];
  }
  if (curve.kind === "arc") {
    const angle = curve.start_angle + t * curve.sweep_angle;
    const x = curve.radius_x * Math.cos(angle);
    const y = curve.radius_y * Math.sin(angle);
    const r = curve.rotation ?? 0;
    return [curve.center[0] + Math.cos(r) * x - Math.sin(r) * y, curve.center[1] + Math.sin(r) * x + Math.cos(r) * y];
  }
  return lerp(a, b, t);
}

// The curve's derivative in t (for the nearest point).
export function derivativeAt(curve, t) {
  const { start: a, end: b } = curve;
  if (curve.kind === "quadratic") {
    const q = curve.control;
    return add(scale(2 * (1 - t), sub(q, a)), scale(2 * t, sub(b, q)));
  }
  if (curve.kind === "cubic") {
    const [c1, c2] = [curve.control1, curve.control2];
    const s = 1 - t;
    return add(add(scale(3 * s * s, sub(c1, a)), scale(6 * s * t, sub(c2, c1))), scale(3 * t * t, sub(b, c2)));
  }
  if (curve.kind === "arc") {
    const angle = curve.start_angle + t * curve.sweep_angle;
    const x = -curve.radius_x * Math.sin(angle) * curve.sweep_angle;
    const y = curve.radius_y * Math.cos(angle) * curve.sweep_angle;
    const r = curve.rotation ?? 0;
    return [Math.cos(r) * x - Math.sin(r) * y, Math.sin(r) * x + Math.cos(r) * y];
  }
  return sub(b, a);
}

export function secondDerivativeAt(curve, t) {
  const { start: a, end: b } = curve;
  if (curve.kind === "quadratic") return scale(2, add(sub(a, scale(2, curve.control)), b));
  if (curve.kind === "cubic") {
    const [c1, c2] = [curve.control1, curve.control2];
    const first = add(sub(c2, scale(2, c1)), a);
    const second = add(sub(b, scale(2, c2)), c1);
    return add(scale(6 * (1 - t), first), scale(6 * t, second));
  }
  if (curve.kind === "arc") {
    const angle = curve.start_angle + t * curve.sweep_angle;
    const w2 = curve.sweep_angle * curve.sweep_angle;
    const x = -curve.radius_x * Math.cos(angle) * w2;
    const y = -curve.radius_y * Math.sin(angle) * w2;
    const r = curve.rotation ?? 0;
    return [Math.cos(r) * x - Math.sin(r) * y, Math.sin(r) * x + Math.cos(r) * y];
  }
  return [0, 0];
}

// Points along the curve as a flat [x0, y0, x1, y1, ...]: the two ends of
// a line, `samples` + 1 points of any other curve.
export function sampleCurve(curve, samples = CURVE_SAMPLES) {
  const pieces = curve.kind === "line" ? 1 : samples;
  const out = new Array(2 * (pieces + 1));
  for (let i = 0; i <= pieces; i++) {
    const p = i === 0 ? pointAt(curve, 0) : i === pieces ? pointAt(curve, 1) : pointAt(curve, i / pieces);
    out[2 * i] = p[0];
    out[2 * i + 1] = p[1];
  }
  // A line or a Bezier ends exactly at its vertices.
  if (curve.kind !== "arc") {
    out[0] = curve.start[0];
    out[1] = curve.start[1];
    out[out.length - 2] = curve.end[0];
    out[out.length - 1] = curve.end[1];
  }
  return out;
}

// The point of `curve` nearest to `p`: {t, point, distance}. The best of
// the samples, refined by golden-section search and Newton's method (as
// the server's locator does).
export function nearestOnCurve(curve, p) {
  const d2 = (t) => {
    const q = pointAt(curve, t);
    return (q[0] - p[0]) ** 2 + (q[1] - p[1]) ** 2;
  };
  if (curve.kind === "line") {
    const ab = sub(curve.end, curve.start);
    const length = dot(ab, ab);
    const t = length === 0 ? 0 : Math.max(0, Math.min(1, dot(sub(p, curve.start), ab) / length));
    const point = t === 0 ? curve.start : t === 1 ? curve.end : lerp(curve.start, curve.end, t);
    return { t, point, distance: distance(point, p) };
  }
  let best = 0;
  let bestD = Infinity;
  for (let i = 0; i <= CURVE_SAMPLES; i++) {
    const d = d2(i / CURVE_SAMPLES);
    if (d < bestD) [best, bestD] = [i, d];
  }
  let lo = Math.max(0, best - 1) / CURVE_SAMPLES;
  let hi = Math.min(CURVE_SAMPLES, best + 1) / CURVE_SAMPLES;
  const ratio = (Math.sqrt(5) - 1) / 2;
  let a = hi - ratio * (hi - lo);
  let b = lo + ratio * (hi - lo);
  let fa = d2(a);
  let fb = d2(b);
  for (let k = 0; k < 60 && hi - lo > 1e-12; k++) {
    if (fa < fb) {
      [hi, b, fb] = [b, a, fa];
      a = hi - ratio * (hi - lo);
      fa = d2(a);
    } else {
      [lo, a, fa] = [a, b, fb];
      b = lo + ratio * (hi - lo);
      fb = d2(b);
    }
  }
  let t = (lo + hi) / 2;
  // Newton on (c(t) - p) . c'(t) = 0: the distance is too flat at its
  // minimum for the search alone.
  for (let k = 0; k < 4; k++) {
    const d = sub(pointAt(curve, t), p);
    const d1 = derivativeAt(curve, t);
    const slope = dot(d1, d1) + dot(d, secondDerivativeAt(curve, t));
    if (!(slope > 0)) break;
    t = Math.max(0, Math.min(1, t - dot(d, d1) / slope));
  }
  for (const end of [0, 1]) if (d2(end) <= d2(t)) t = end;
  const point = t === 0 ? pointAt(curve, 0) : t === 1 ? pointAt(curve, 1) : pointAt(curve, t);
  return { t, point, distance: distance(point, p) };
}

// The curve split at t in (0, 1): [first, second], each {kind, start,
// end, ...} with the point of the split as the end of the first.
export function splitCurve(curve, t) {
  const { start: a, end: b } = curve;
  if (curve.kind === "quadratic") {
    const q0 = lerp(a, curve.control, t);
    const q1 = lerp(curve.control, b, t);
    const m = lerp(q0, q1, t);
    return [
      { kind: "quadratic", start: a, control: q0, end: m },
      { kind: "quadratic", start: m, control: q1, end: b },
    ];
  }
  if (curve.kind === "cubic") {
    const p01 = lerp(a, curve.control1, t);
    const p12 = lerp(curve.control1, curve.control2, t);
    const p23 = lerp(curve.control2, b, t);
    const p012 = lerp(p01, p12, t);
    const p123 = lerp(p12, p23, t);
    const m = lerp(p012, p123, t);
    return [
      { kind: "cubic", start: a, control1: p01, control2: p012, end: m },
      { kind: "cubic", start: m, control1: p123, control2: p23, end: b },
    ];
  }
  if (curve.kind === "arc") {
    const m = pointAt(curve, t);
    const arc = { center: curve.center, radius_x: curve.radius_x, radius_y: curve.radius_y, rotation: curve.rotation ?? 0 };
    return [
      { kind: "arc", ...arc, start_angle: curve.start_angle, sweep_angle: curve.sweep_angle * t, start: a, end: m },
      { kind: "arc", ...arc, start_angle: curve.start_angle + curve.sweep_angle * t, sweep_angle: curve.sweep_angle * (1 - t), start: m, end: b },
    ];
  }
  const m = lerp(a, b, t);
  return [
    { kind: "line", start: a, end: m },
    { kind: "line", start: m, end: b },
  ];
}

// The circular arc from a through m to b (the library's centre form), or
// null when the three points are (nearly) on a line.
export function arcThrough(a, m, b) {
  const d = 2 * (a[0] * (m[1] - b[1]) + m[0] * (b[1] - a[1]) + b[0] * (a[1] - m[1]));
  const size = Math.max(distance(a, m), distance(m, b), distance(a, b));
  if (!(Math.abs(d) > 1e-9 * size * size)) return null;
  const a2 = dot(a, a);
  const m2 = dot(m, m);
  const b2 = dot(b, b);
  const center = [(a2 * (m[1] - b[1]) + m2 * (b[1] - a[1]) + b2 * (a[1] - m[1])) / d, (a2 * (b[0] - m[0]) + m2 * (a[0] - b[0]) + b2 * (m[0] - a[0])) / d];
  const radius = distance(a, center);
  const angle = (p) => Math.atan2(p[1] - center[1], p[0] - center[0]);
  const ta = angle(a);
  const ccw = (x) => ((x % TAU) + TAU) % TAU;
  const toB = ccw(angle(b) - ta);
  const toM = ccw(angle(m) - ta);
  const sweep = toM < toB ? toB : toB - TAU;
  return { kind: "arc", center, radius_x: radius, radius_y: radius, rotation: 0, start_angle: ta, sweep_angle: sweep };
}

// The arc moved by the similarity that takes its old ends (a, b) to new
// ones: every arc stays the same shape, turned and scaled with its chord.
// For a closed arc (a == b) the similarity about its centre that takes a
// to the new a.
export function followEnds(arc, a, b, newA, newB) {
  const chord = sub(b, a);
  let s;
  let anchor;
  let newAnchor;
  if (dot(chord, chord) > 0) {
    s = complexDivide(sub(newB, newA), chord);
    [anchor, newAnchor] = [a, newA];
  } else {
    s = complexDivide(sub(newA, arc.center), sub(a, arc.center));
    [anchor, newAnchor] = [arc.center, arc.center];
  }
  const k = Math.hypot(s[0], s[1]);
  if (!(k > 0) || !Number.isFinite(k)) return arc;
  const rel = sub(arc.center, anchor);
  const center = add(newAnchor, [s[0] * rel[0] - s[1] * rel[1], s[0] * rel[1] + s[1] * rel[0]]);
  return { ...arc, center, radius_x: arc.radius_x * k, radius_y: arc.radius_y * k, rotation: (arc.rotation ?? 0) + Math.atan2(s[1], s[0]) };
}

function complexDivide(p, q) {
  const n = dot(q, q);
  return [(p[0] * q[0] + p[1] * q[1]) / n, (p[1] * q[0] - p[0] * q[1]) / n];
}

// The drawing's curves as polygons (one flat point list per loop), for
// containment and the fill.
export function loopPolygon(loop) {
  const out = [];
  for (let e = 0; e < loop.edges.length; e++) {
    const points = sampleCurve(edgeCurve(loop, e));
    for (let i = 0; i + 2 < points.length; i += 2) out.push([points[i], points[i + 1]]);
  }
  return out;
}

export function signedArea(polygon) {
  let area = 0;
  for (let i = 0; i < polygon.length; i++) {
    const [p, q] = [polygon[i], polygon[(i + 1) % polygon.length]];
    area += p[0] * q[1] - q[0] * p[1];
  }
  return area / 2;
}

export function insidePolygon(polygon, p) {
  let inside = false;
  for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
    const [a, b] = [polygon[i], polygon[j]];
    if (a[1] > p[1] !== b[1] > p[1] && p[0] < ((b[0] - a[0]) * (p[1] - a[1])) / (b[1] - a[1]) + a[0]) inside = !inside;
  }
  return inside;
}

// The box around every curve: {min: [x, y], max: [x, y]}, or null.
export function drawingBounds(drawing, loops = null) {
  let min = [Infinity, Infinity];
  let max = [-Infinity, -Infinity];
  drawing.loops.forEach((loop, l) => {
    if (loops && !loops.includes(l)) return;
    for (let e = 0; e < loop.edges.length; e++) {
      const points = sampleCurve(edgeCurve(loop, e));
      for (let i = 0; i < points.length; i += 2) {
        min = [Math.min(min[0], points[i]), Math.min(min[1], points[i + 1])];
        max = [Math.max(max[0], points[i]), Math.max(max[1], points[i + 1])];
      }
    }
  });
  return Number.isFinite(min[0]) ? { min, max } : null;
}

// --- regions, names, snapping -------------------------------------------------

// The region names in order of first appearance.
export function drawingRegions(drawing) {
  const names = [];
  for (const loop of drawing.loops) for (const edge of loop.edges) if (!names.includes(edge.region)) names.push(edge.region);
  return names;
}

// `base`, or `base_2`, `base_3`, ... : the first that is not in `names`.
export function uniqueName(names, base) {
  if (!names.includes(base)) return base;
  const stem = base.replace(/_\d+$/, "");
  for (let k = 2; ; k++) if (!names.includes(`${stem}_${k}`)) return `${stem}_${k}`;
}

// A grid step for a domain of this size: 1, 2 or 5 times a power of ten,
// about a fortieth of it.
export function gridStep(size) {
  if (!(size > 0) || !Number.isFinite(size)) return 1;
  const raw = size / 40;
  const power = 10 ** Math.floor(Math.log10(raw));
  const unit = raw / power;
  return power * (unit < 1.5 ? 1 : unit < 3.5 ? 2 : unit < 7.5 ? 5 : 10);
}

// The grid point nearest to p; step and the result are multiples, so a
// typed coordinate on the grid stays the same number. Without a step, p.
export function snapToGrid(p, step) {
  if (!step) return p;
  const round = (v) => {
    const k = Math.round(v / step);
    // A step like 0.1 is not exact in binary: 3 * 0.1 is 0.30000000000000004,
    // 3 / 10 is 0.3.
    const inverse = Math.round(1 / step);
    return Math.abs(inverse * step - 1) < 1e-12 ? k / inverse : k * step;
  };
  return [round(p[0]), round(p[1])];
}

// --- hit testing ---------------------------------------------------------------

// Every vertex: [{loop, vertex, point}].
function vertexList(drawing) {
  const out = [];
  drawing.loops.forEach((loop, l) => loop.vertices.forEach((point, v) => out.push({ loop: l, vertex: v, point })));
  return out;
}

// The handles a selected edge shows: its Bezier controls, and a bend
// handle at its middle (lines and arcs; dragging it makes a circular arc
// through it). [{type, loop, edge, which, point}]
export function edgeHandles(drawing, l, e) {
  const loop = drawing.loops[l];
  const curve = edgeCurve(loop, e);
  if (curve.kind === "quadratic") return [{ type: "control", loop: l, edge: e, which: "control", point: curve.control }];
  if (curve.kind === "cubic")
    return [
      { type: "control", loop: l, edge: e, which: "control1", point: curve.control1 },
      { type: "control", loop: l, edge: e, which: "control2", point: curve.control2 },
    ];
  return [{ type: "bend", loop: l, edge: e, point: pointAt(curve, 0.5) }];
}

// The point a whole loop is dragged by: the middle of its box.
export function moveHandle(drawing, l) {
  const box = drawingBounds(drawing, [l]);
  return box ? [(box.min[0] + box.max[0]) / 2, (box.min[1] + box.max[1]) / 2] : null;
}

// What is under p (world units) within `tolerance`: the handles of the
// selected edges first, then vertices, the selected loop's move handle,
// then edges (with the parameter there). Returns {type: "control" |
// "bend" | "vertex" | "move" | "edge", loop, ...} or null.
export function hitTest(drawing, p, tolerance, selectedEdges = []) {
  const nearest = (items) => {
    let best = null;
    let bestD = tolerance;
    for (const item of items) {
      const d = distance(item.point, p);
      if (d <= bestD) [best, bestD] = [item, d];
    }
    return best;
  };
  const handles = selectedEdges.filter((s) => drawing.loops[s.loop]?.edges[s.edge]).flatMap((s) => edgeHandles(drawing, s.loop, s.edge));
  const handle = nearest(handles);
  if (handle) return handle;
  const vertex = nearest(vertexList(drawing).map((v) => ({ type: "vertex", ...v })));
  if (vertex) return vertex;
  const loops = [...new Set(selectedEdges.map((s) => s.loop))];
  if (loops.length === 1 && drawing.loops[loops[0]]) {
    const point = moveHandle(drawing, loops[0]);
    if (point && distance(point, p) <= tolerance) return { type: "move", loop: loops[0], point };
  }
  const edge = nearestEdge(drawing, p);
  return edge && edge.distance <= tolerance ? edge : null;
}

// The edge nearest to p: {type: "edge", loop, edge, t, point, distance}.
export function nearestEdge(drawing, p) {
  let best = null;
  drawing.loops.forEach((loop, l) => {
    for (let e = 0; e < loop.edges.length; e++) {
      const found = nearestOnCurve(edgeCurve(loop, e), p);
      if (!best || found.distance < best.distance) best = { type: "edge", loop: l, edge: e, ...found };
    }
  });
  return best;
}

// Where a split by a click at p lands in a drawing: the nearest edge's
// point, or within `snap` of a vertex that vertex exactly. {loop, edge, t,
// vertex (or null), point, region}. A vertex is reported on the edge it
// starts (t = 0), unless the nearest edge ends there (t = 1), so the
// region is the one pointed at.
export function drawingSplitPoint(drawing, p, snap) {
  const edge = nearestEdge(drawing, p);
  if (!edge) return null;
  const loop = drawing.loops[edge.loop];
  const n = loop.vertices.length;
  let best = null;
  let bestD = snap;
  drawing.loops.forEach((other, l) =>
    other.vertices.forEach((point, v) => {
      const d = distance(point, p);
      if (d <= bestD) [best, bestD] = [{ loop: l, vertex: v, point }, d];
    }),
  );
  if (best) {
    if (best.loop === edge.loop && best.vertex === edge.edge) return { loop: edge.loop, edge: edge.edge, t: 0, vertex: best.vertex, point: best.point, region: loop.edges[edge.edge].region };
    if (best.loop === edge.loop && best.vertex === (edge.edge + 1) % n)
      return { loop: edge.loop, edge: edge.edge, t: 1, vertex: best.vertex, point: best.point, region: loop.edges[edge.edge].region };
    const other = drawing.loops[best.loop];
    return { loop: best.loop, edge: best.vertex, t: 0, vertex: best.vertex, point: best.point, region: other.edges[best.vertex].region };
  }
  return { loop: edge.loop, edge: edge.edge, t: edge.t, vertex: null, point: edge.point, region: loop.edges[edge.edge].region };
}

// The boundary point nearest to p on the geometry view the server drew
// (its curves sampled, or a mesh file's boundary edges), for the split
// tool's marker: {point, region, corner}, snapped to a curve's end (a
// corner; for a mesh file every boundary node) within `snap`. Null for an
// empty view.
export function viewSplitPoint(view, p, snap) {
  let best = null;
  const consider = (ax, ay, bx, by, region) => {
    const ab = [bx - ax, by - ay];
    const length = dot(ab, ab);
    const s = length === 0 ? 0 : Math.max(0, Math.min(1, ((p[0] - ax) * ab[0] + (p[1] - ay) * ab[1]) / length));
    const point = [ax + s * ab[0], ay + s * ab[1]];
    const d = distance(point, p);
    if (!best || d < best.distance) best = { point, region, distance: d, ends: [[ax, ay], [bx, by]] };
  };
  const corners = [];
  if (view?.kind === "curves") {
    for (const loop of view.loops)
      for (const curve of loop.curves) {
        const q = curve.points;
        corners.push([q[0], q[1]]);
        for (let i = 0; i + 3 < q.length; i += 2) consider(q[i], q[i + 1], q[i + 2], q[i + 3], curve.region);
      }
  } else if (view?.kind === "mesh_file") {
    for (const [ax, ay, bx, by, region] of view.boundary_edges) consider(ax, ay, bx, by, view.regions[region].name);
  }
  if (!best) return null;
  if (view.kind === "mesh_file") {
    // A mesh file splits at a node: the nearer end of the nearest edge.
    const [a, b] = best.ends;
    const node = distance(a, p) <= distance(b, p) ? a : b;
    return { point: node, region: best.region, corner: true };
  }
  let corner = null;
  let cornerD = snap;
  for (const c of corners) {
    const d = distance(c, p);
    if (d <= cornerD) [corner, cornerD] = [c, d];
  }
  return corner ? { point: corner, region: best.region, corner: true } : { point: best.point, region: best.region, corner: false };
}

// --- edits ---------------------------------------------------------------------------

const clone = (drawing) => structuredClone(drawing);

// The loop's outward normal side: +1 when the loop runs counterclockwise
// (the right of an edge is outside), -1 otherwise.
function outwardSign(loop) {
  return signedArea(loopPolygon(loop)) >= 0 ? 1 : -1;
}

// The edge `e` of `loop` with a curve's data (start and end dropped).
function edgeData(curve, region) {
  const { start, end, ...rest } = curve;
  void start;
  void end;
  return { ...rest, region };
}

// Vertex v moved to p: the edges into and out of it follow (an arc by the
// similarity of its chord, the handles of a cubic next to v move with it).
export function moveVertex(drawing, l, v, p) {
  const next = clone(drawing);
  const loop = next.loops[l];
  const n = loop.vertices.length;
  const old = loop.vertices[v];
  const delta = sub(p, old);
  const before = (v + n - 1) % n;
  const oldVertices = loop.vertices.map((q) => q.slice());
  loop.vertices[v] = [p[0], p[1]];
  for (const e of new Set([before, v])) {
    const edge = loop.edges[e];
    const a = oldVertices[e];
    const b = oldVertices[(e + 1) % n];
    if (edge.kind === "arc") loop.edges[e] = { ...followEnds(edge, a, b, loop.vertices[e], loop.vertices[(e + 1) % n]), region: edge.region };
    if (edge.kind === "cubic") {
      if (e === v) edge.control1 = add(edge.control1, delta);
      if (e === before) edge.control2 = add(edge.control2, delta);
    }
  }
  return next;
}

// A Bezier's control point moved to p.
export function moveControl(drawing, l, e, which, p) {
  const next = clone(drawing);
  next.loops[l].edges[e][which] = [p[0], p[1]];
  return next;
}

// The middle of edge e dragged to p: a circular arc from its start
// through p to its end. A closed arc (a loop of one vertex) keeps its
// centre and takes the radius that reaches p. Unchanged when the points
// are on a line.
export function bendEdge(drawing, l, e, p) {
  const next = clone(drawing);
  const loop = next.loops[l];
  const curve = edgeCurve(loop, e);
  const region = loop.edges[e].region;
  if (loop.vertices.length === 1 && curve.kind === "arc") {
    const middle = pointAt(curve, 0.5);
    const k = distance(p, curve.center) / distance(middle, curve.center);
    if (!(k > 0) || !Number.isFinite(k)) return drawing;
    const arc = { ...loop.edges[e], radius_x: curve.radius_x * k, radius_y: curve.radius_y * k };
    loop.edges[e] = arc;
    loop.vertices[0] = pointAt({ ...arc, start: null, end: null }, 0);
    return next;
  }
  const arc = arcThrough(curve.start, p, curve.end);
  if (!arc) return drawing;
  loop.edges[e] = { ...arc, region };
  // The arc's ends are the vertices up to rounding; keep the vertices.
  return next;
}

// Edge e as another kind, the same ends: a line becomes an arc or a
// Bezier bulging outward by 0.15 of its length (its handles stay near
// it); a Bezier or an arc
// becomes the arc through its middle, or a Bezier with its end tangents;
// a quadratic becomes the same curve as a cubic. A closed arc stays an
// arc (throws).
export function setEdgeKind(drawing, l, e, kind) {
  const next = clone(drawing);
  const loop = next.loops[l];
  const curve = edgeCurve(loop, e);
  const region = loop.edges[e].region;
  if (curve.kind === kind) return drawing;
  if (loop.vertices.length === 1) throw new Error("a full circle stays an arc: a loop of one vertex has no other edge");
  const { start: a, end: b } = curve;
  const chord = sub(b, a);
  const length = Math.hypot(chord[0], chord[1]);
  const normal = scale(outwardSign(loop) / (length || 1), [chord[1], -chord[0]]);
  const middle = curve.kind === "line" ? add(lerp(a, b, 0.5), scale(BULGE * length, normal)) : pointAt(curve, 0.5);
  let data;
  if (kind === "line") data = { kind: "line" };
  else if (kind === "arc") {
    const arc = arcThrough(a, middle, b);
    if (!arc) throw new Error("this edge is straight: drag its middle to bend it");
    data = arc;
  } else if (kind === "quadratic") {
    if (curve.kind === "cubic") data = { kind, control: scale(0.25, sub(scale(3, add(curve.control1, curve.control2)), add(a, b))) };
    else if (curve.kind === "line") data = { kind, control: add(lerp(a, b, 0.5), scale(2 * BULGE * length, normal)) };
    else data = { kind, control: sub(scale(2, middle), scale(0.5, add(a, b))) };
  } else if (kind === "cubic") {
    if (curve.kind === "quadratic") data = { kind, control1: lerp(a, curve.control, 2 / 3), control2: lerp(b, curve.control, 2 / 3) };
    else if (curve.kind === "line") {
      const lift = scale((4 / 3) * BULGE * length, normal);
      data = { kind, control1: add(lerp(a, b, 1 / 3), lift), control2: add(lerp(a, b, 2 / 3), lift) };
    } else data = { kind, control1: add(a, scale(1 / 3, derivativeAt(curve, 0))), control2: sub(b, scale(1 / 3, derivativeAt(curve, 1))) };
  } else throw new Error(`unknown edge kind ${kind}`);
  loop.edges[e] = { ...data, region };
  return next;
}

// A new vertex on edge e at t in (0, 1): the edge is split into two
// curves of its kind and region, exactly where it was. Returns {drawing,
// vertex} (the new vertex's index, e + 1).
export function insertVertex(drawing, l, e, t) {
  const next = clone(drawing);
  const loop = next.loops[l];
  const region = loop.edges[e].region;
  const [first, second] = splitCurve(edgeCurve(loop, e), t);
  loop.vertices.splice(e + 1, 0, second.start.slice());
  loop.edges.splice(e, 1, edgeData(first, region), edgeData(second, region));
  return { drawing: next, vertex: e + 1 };
}

// Vertex v removed: its two edges become one line from the vertex before
// it to the one after it, in the region of the edge before. A loop keeps
// three vertices at least (throws).
export function deleteVertex(drawing, l, v) {
  const loop = drawing.loops[l];
  const n = loop.vertices.length;
  if (n <= 3) throw new Error("a loop needs three vertices: delete the whole loop instead");
  const next = clone(drawing);
  const target = next.loops[l];
  const before = (v + n - 1) % n;
  target.edges[before] = { kind: "line", region: target.edges[before].region };
  target.vertices.splice(v, 1);
  target.edges.splice(v, 1);
  return next;
}

// Every edge in `edges` ([{loop, edge}]) in the region `name`.
export function assignRegion(drawing, edges, name) {
  const next = clone(drawing);
  for (const { loop, edge } of edges) next.loops[loop].edges[edge].region = name;
  return next;
}

// Every edge of region `from` in region `to` (a rename, or a merge when
// `to` exists).
export function renameRegion(drawing, from, to) {
  const next = clone(drawing);
  for (const loop of next.loops) for (const edge of loop.edges) if (edge.region === from) edge.region = to;
  return next;
}

// The region of edge `e` of loop `l` split at a point of that edge (t,
// or a vertex: t 0 or 1): the run of edges of that region around it (in
// the loop's order) is cut there; the edges before the point are named
// `first`, those after `second`. A run that is the whole loop starts at
// vertex 0. Returns the new drawing, or null when the point is an end of
// the run (nothing to split).
export function splitRegion(drawing, l, e, t, first, second) {
  let next = drawing;
  let edge = e;
  if (t > 0 && t < 1) {
    const inserted = insertVertex(drawing, l, e, t);
    next = inserted.drawing;
    edge = inserted.vertex; // the second half starts at the new vertex
  } else if (t >= 1) edge = (e + 1) % drawing.loops[l].edges.length;
  const loop = next.loops[l];
  const n = loop.edges.length;
  const region = drawing.loops[l].edges[e].region;
  const inRun = (k) => loop.edges[(k + n) % n].region === region;
  // The run around the edge pointed at: its start (the first edge whose
  // predecessor is another region) and its length.
  const pointed = t >= 1 ? (edge + n - 1) % n : edge;
  if (!inRun(pointed)) return null;
  const whole = loop.edges.every((x) => x.region === region);
  let start = pointed;
  if (whole) start = 0;
  else while (inRun(start - 1)) start = (start + n - 1) % n;
  let length = 0;
  while (length < n && inRun(start + length)) length++;
  const offset = (edge - start + n) % n;
  if (offset === 0 || offset >= length) return null;
  if (next === drawing) next = clone(drawing);
  for (let k = 0; k < length; k++) next.loops[l].edges[(start + k) % n].region = k < offset ? first : second;
  return next;
}

// New loops: a rectangle from corner a to corner b, a circle, a polygon,
// each one region.
export function rectangleLoop(a, b, region) {
  const vertices = [
    [Math.min(a[0], b[0]), Math.min(a[1], b[1])],
    [Math.max(a[0], b[0]), Math.min(a[1], b[1])],
    [Math.max(a[0], b[0]), Math.max(a[1], b[1])],
    [Math.min(a[0], b[0]), Math.max(a[1], b[1])],
  ];
  return { vertices, edges: vertices.map(() => ({ kind: "line", region })) };
}

export function circleLoop(center, radius, region) {
  return {
    vertices: [[center[0] + radius, center[1]]],
    edges: [{ kind: "arc", region, center: [center[0], center[1]], radius_x: radius, radius_y: radius, rotation: 0, start_angle: 0, sweep_angle: TAU }],
  };
}

export function polygonLoop(points, region) {
  return { vertices: points.map((p) => [p[0], p[1]]), edges: points.map(() => ({ kind: "line", region })) };
}

// A name for a new loop's region: "hole" (or hole_2, ...) when its first
// point is inside an odd number of loops, else "boundary" (or ...).
export function newLoopRegion(drawing, point) {
  let depth = 0;
  for (const loop of drawing.loops) if (insidePolygon(loopPolygon(loop), point)) depth++;
  return uniqueName(drawingRegions(drawing), depth % 2 === 1 ? "hole" : "boundary");
}

export function addLoop(drawing, loop) {
  const next = clone(drawing);
  next.loops.push(structuredClone(loop));
  return next;
}

// Loop l moved by d: its vertices, arc centres and controls.
export function moveLoop(drawing, l, d) {
  const next = clone(drawing);
  const loop = next.loops[l];
  loop.vertices = loop.vertices.map((p) => add(p, d));
  for (const edge of loop.edges)
    for (const key of ["center", "control", "control1", "control2"]) if (edge[key]) edge[key] = add(edge[key], d);
  return next;
}

// Loop l removed; the last loop stays (throws).
export function deleteLoop(drawing, l) {
  if (drawing.loops.length <= 1) throw new Error("a drawing keeps one loop at least");
  const next = clone(drawing);
  next.loops.splice(l, 1);
  return next;
}

// A blank drawing: a rectangle of this width and height about the origin,
// its sides named as the rectangle shape's.
export function blankDrawing(width = 2, height = 1) {
  const loop = rectangleLoop([-width / 2, -height / 2], [width / 2, height / 2], "bottom");
  ["bottom", "right", "top", "left"].forEach((name, i) => (loop.edges[i].region = name));
  return { source: "drawing", loops: [loop] };
}

// The place a geometry error's path names in a drawing:
// "geometry.loops[1].edges[3]" is {loop: 1, edge: 3},
// "geometry.loops[1].vertices[2]..." {loop: 1, vertex: 2},
// "geometry.loops[1]..." {loop: 1}; null for another path.
export function drawingErrorTarget(path) {
  const match = /^geometry\.loops\[(\d+)\](?:\.(edges|vertices)\[(\d+)\])?/.exec(path ?? "");
  if (!match) return null;
  const out = { loop: Number(match[1]) };
  if (match[2] === "edges") out.edge = Number(match[3]);
  if (match[2] === "vertices") out.vertex = Number(match[3]);
  return out;
}

// Whether a selection still names parts of the drawing.
export function selectionValid(drawing, selection) {
  if (!selection) return true;
  if (selection.vertex !== undefined) return Boolean(drawing.loops[selection.loop]?.vertices[selection.vertex]);
  return (selection.edges ?? []).every((s) => Boolean(drawing.loops[s.loop]?.edges[s.edge]));
}
