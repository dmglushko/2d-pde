// Pure helpers: no DOM, so Node tests them (app/tests/web.test.mjs).

// --- numbers -----------------------------------------------------------------

// The significant digits numbers are shown with (a setting).
let defaultDigits = 4;
export function setDefaultDigits(digits) {
  if (Number.isInteger(digits) && digits >= 2 && digits <= 10) defaultDigits = digits;
}
export function getDefaultDigits() {
  return defaultDigits;
}

// A short, readable form of a number for labels and tables.
export function formatNumber(value, digits = defaultDigits) {
  if (value === null || value === undefined) return "–";
  if (typeof value !== "number") return String(value);
  if (!Number.isFinite(value)) return String(value);
  if (value === 0) return "0";
  const magnitude = Math.abs(value);
  if (Number.isInteger(value) && magnitude < 1e7) return value.toLocaleString("en-US");
  if (magnitude >= 1e5 || magnitude < 1e-3) return value.toExponential(digits - 1);
  return String(Number(value.toPrecision(digits)));
}

// --- colours -------------------------------------------------------------------

export function parseColor(text) {
  const t = text.trim();
  if (t.startsWith("#")) {
    const hex = t.length === 4 ? [...t.slice(1)].map((c) => c + c).join("") : t.slice(1, 7);
    return [0, 2, 4].map((i) => parseInt(hex.slice(i, i + 2), 16));
  }
  const match = t.match(/rgba?\(([^)]+)\)/);
  if (match) return match[1].split(",").slice(0, 3).map((v) => Math.round(parseFloat(v)));
  return [128, 128, 128];
}

// sRGB <-> OKLab, so ramps interpolate evenly in lightness.
function toLinear(c) {
  const v = c / 255;
  return v <= 0.04045 ? v / 12.92 : ((v + 0.055) / 1.055) ** 2.4;
}
function fromLinear(v) {
  const c = v <= 0.0031308 ? 12.92 * v : 1.055 * v ** (1 / 2.4) - 0.055;
  return Math.max(0, Math.min(255, Math.round(c * 255)));
}
export function rgbToOklab([r, g, b]) {
  const [lr, lg, lb] = [toLinear(r), toLinear(g), toLinear(b)];
  const l = Math.cbrt(0.4122214708 * lr + 0.5363325363 * lg + 0.0514459929 * lb);
  const m = Math.cbrt(0.2119034982 * lr + 0.6806995451 * lg + 0.1073969566 * lb);
  const s = Math.cbrt(0.0883024619 * lr + 0.2817188376 * lg + 0.6299787005 * lb);
  return [
    0.2104542553 * l + 0.793617785 * m - 0.0040720468 * s,
    1.9779984951 * l - 2.428592205 * m + 0.4505937099 * s,
    0.0259040371 * l + 0.7827717662 * m - 0.808675766 * s,
  ];
}
export function oklabToRgb([L, a, b]) {
  const l = (L + 0.3963377774 * a + 0.2158037573 * b) ** 3;
  const m = (L - 0.1055613458 * a - 0.0638541728 * b) ** 3;
  const s = (L - 0.0894841775 * a - 1.291485548 * b) ** 3;
  return [
    fromLinear(4.0767416621 * l - 3.3077115913 * m + 0.2309699292 * s),
    fromLinear(-1.2684380046 * l + 2.6097574011 * m - 0.3413193965 * s),
    fromLinear(-0.0041960863 * l - 0.7034186147 * m + 1.707614701 * s),
  ];
}

// A colour table of `size` entries, through the given stops (rgb arrays),
// evenly spaced, interpolated in OKLab.
export function rampTable(stops, size = 256) {
  const labs = stops.map(rgbToOklab);
  const table = new Uint8ClampedArray(size * 3);
  for (let i = 0; i < size; i++) {
    const position = (i / (size - 1)) * (labs.length - 1);
    const k = Math.min(labs.length - 2, Math.floor(position));
    const f = position - k;
    const lab = labs[k].map((v, j) => v + (labs[k + 1][j] - v) * f);
    table.set(oklabToRgb(lab), i * 3);
  }
  return table;
}

// How a field's values map to colours: sequential (one hue, light to dark)
// for values of one sign, diverging (two hues, neutral at zero) when the
// values have both signs. Returns {kind, min, max, position(value) in
// [0, 1]}.
export function fieldScale(values) {
  let min = Infinity;
  let max = -Infinity;
  for (const v of values) {
    if (v === null || !Number.isFinite(v)) continue;
    if (v < min) min = v;
    if (v > max) max = v;
  }
  if (min === Infinity) return { kind: "empty", min: 0, max: 0, position: () => 0.5 };
  if (min < 0 && max > 0) {
    const bound = Math.max(-min, max);
    return { kind: "diverging", min, max, bound, position: (v) => 0.5 + v / (2 * bound) };
  }
  if (max === min) return { kind: "constant", min, max, position: () => 0.5 };
  return { kind: "sequential", min, max, position: (v) => (v - min) / (max - min) };
}

// --- regions -------------------------------------------------------------------

export const SERIES_SLOTS = 8;

// The colour token of region i: the categorical slots in their fixed
// order; past the eighth region, one neutral "other" colour (identity is
// then carried by the legend and the tooltip).
export function regionToken(index) {
  return index < SERIES_SLOTS ? `--series-${index + 1}` : "--series-other";
}

// --- mesh quality --------------------------------------------------------------

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

// --- the case ------------------------------------------------------------------

const isObject = (value) => value !== null && typeof value === "object" && !Array.isArray(value);

// A dictionary for names the user chooses (region names may be
// "constructor" or "__proto__"): no prototype, so every lookup and
// assignment sees only its own entries.
function dictionary(source) {
  const out = Object.create(null);
  for (const [key, value] of Object.entries(source ?? {})) out[key] = value;
  return out;
}

// The names a region edit touches: the regions it removes or creates.
function touchedNames(edit) {
  if (edit.op === "rename") return [edit.from, edit.to];
  if (edit.op === "split") return [edit.region, edit.first, edit.second];
  if (edit.op === "merge") return [edit.source];
  return [];
}

// Adds a region edit to the case and moves the boundary conditions with
// it, as the library will apply the edit: a rename gives the new name the
// old name's condition, a split gives both halves the condition of the
// whole, a merge removes the source's (the target keeps its own). The
// condition each touched name had before goes into an undo record
// (view.region_edit_undo, one per edit), so undoRegionEdit restores it
// exactly, whatever the library later says about the edit. Returns a new
// case.
export function applyRegionEdit(caseData, edit) {
  const next = structuredClone(caseData);
  const geometry = next.geometry;
  const problem = (next.problem ??= {});
  const view = (next.view ??= {});
  const edits = (geometry.region_edits ??= []);
  const undo = Array.isArray(view.region_edit_undo) && view.region_edit_undo.length === edits.length ? view.region_edit_undo : edits.map(() => null);
  const conditions = dictionary(problem.boundary_conditions);
  const before = Object.create(null);
  for (const name of touchedNames(edit)) before[name] = Object.hasOwn(conditions, name) ? conditions[name] : null;
  const move = (from, ...to) => {
    if (!Object.hasOwn(conditions, from)) return;
    const condition = conditions[from];
    delete conditions[from];
    for (const name of to) conditions[name] = structuredClone(condition);
  };
  if (edit.op === "rename") move(edit.from, edit.to);
  else if (edit.op === "split") move(edit.region, edit.first, edit.second);
  else if (edit.op === "merge") move(edit.source);
  edits.push(edit);
  undo.push({ before });
  problem.boundary_conditions = conditions;
  view.region_edit_undo = undo;
  return next;
}

// Removes the last region edit and gives every name it touched the
// condition it had before the edit (none, for a name the edit created).
// An edit without an undo record (a hand-written case) is only removed.
// Returns a new case.
export function undoRegionEdit(caseData) {
  const next = structuredClone(caseData);
  const edits = next.geometry?.region_edits ?? [];
  if (edits.length === 0) return next;
  const view = (next.view ??= {});
  const undo = Array.isArray(view.region_edit_undo) && view.region_edit_undo.length === edits.length ? view.region_edit_undo : edits.map(() => null);
  edits.pop();
  const record = undo.pop();
  const problem = (next.problem ??= {});
  const conditions = dictionary(problem.boundary_conditions);
  if (isObject(record?.before))
    for (const [name, condition] of Object.entries(record.before)) {
      if (isObject(condition)) conditions[name] = condition;
      else delete conditions[name];
    }
  problem.boundary_conditions = conditions;
  if (undo.length === 0 || undo.every((r) => r === null)) delete view.region_edit_undo;
  else view.region_edit_undo = undo;
  if (Object.keys(view).length === 0) delete next.view;
  return next;
}

// The case with exactly one boundary condition per region of `regions`
// (the regions of a geometry the library accepted). Returns a new case;
// the input is not changed. Region edits have already moved their
// conditions (applyRegionEdit); this covers every other change of the
// regions (another SVG, another shape):
// - a condition whose region is gone moves to view.condition_archive, so
//   it comes back when its region does;
// - a region without a condition takes it from the archive, else a copy
//   of `defaultCondition`;
// - the conditions are listed in the order of `regions`.
export function reconcileConditions(caseData, regions, defaultCondition) {
  const next = structuredClone(caseData);
  const problem = (next.problem ??= {});
  const view = (next.view ??= {});
  const archive = dictionary(view.condition_archive);
  const known = dictionary(problem.boundary_conditions);
  for (const name of Object.keys(known)) if (!regions.includes(name)) archive[name] = known[name];
  const conditions = Object.create(null);
  for (const region of regions) {
    if (Object.hasOwn(known, region)) conditions[region] = known[region];
    else if (Object.hasOwn(archive, region) && isObject(archive[region])) conditions[region] = archive[region];
    else conditions[region] = structuredClone(defaultCondition);
    delete archive[region];
  }
  problem.boundary_conditions = conditions;
  if (Object.keys(archive).length === 0) delete view.condition_archive;
  else view.condition_archive = archive;
  if (Object.keys(view).length === 0) delete next.view;
  return next;
}

// A case as the page can show it: the format is checked, and a section
// that is missing or of the wrong type is replaced by the default's (also
// the parts of the mesh section the forms need). Everything else is left
// for the server, which reads the case strictly. Returns {caseData,
// replaced: [paths]}; throws an Error for something that is not a case.
export function normalizeCase(input, defaults) {
  if (!isObject(input) || input.format !== "pde2d-app-case") throw new Error("This file is not a pde2d app case.");
  const caseData = structuredClone(input);
  const replaced = [];
  const ensure = (object, key, valid, fallback, path) => {
    if (!valid(object[key])) {
      object[key] = structuredClone(fallback);
      replaced.push(path);
    }
  };
  const optional = (valid) => (value) => value === undefined || valid(value);
  const isString = (value) => typeof value === "string";
  const isList = (valid) => (value) => Array.isArray(value) && value.every(valid);
  // A component choice: {"type": text, "options": object}.
  const isChoice = (value) => isObject(value) && isString(value.type) && optional(isObject)(value.options);

  const sources = ["shape", "svg", "mesh_file"];
  ensure(caseData, "geometry", (g) => isObject(g) && sources.includes(g.source), defaults.geometry, "geometry");
  const geometry = caseData.geometry;
  if (geometry.source === "shape") {
    ensure(geometry, "shape", isString, defaults.geometry.shape, "geometry.shape");
    ensure(geometry, "options", optional(isObject), {}, "geometry.options");
  }
  if (geometry.source === "svg") ensure(geometry, "svg", optional(isString), "", "geometry.svg");
  if (geometry.source === "mesh_file") ensure(geometry, "text", optional(isString), "", "geometry.text");
  ensure(geometry, "region_edits", optional(isList(isObject)), [], "geometry.region_edits");

  ensure(caseData, "mesh", isObject, defaults.mesh, "mesh");
  ensure(caseData.mesh, "spec", isObject, defaults.mesh.spec, "mesh.spec");
  for (const key of ["discretizer", "triangulator"]) ensure(caseData.mesh, key, isChoice, defaults.mesh[key], `mesh.${key}`);
  ensure(caseData.mesh, "refiner", (r) => r === null || isChoice(r), defaults.mesh.refiner, "mesh.refiner");
  ensure(caseData.mesh, "improvers", isList(isChoice), defaults.mesh.improvers, "mesh.improvers");

  ensure(caseData, "problem", isObject, defaults.problem, "problem");
  for (const key of ["equation", "parameters", "boundary_conditions", "initial"]) ensure(caseData.problem, key, optional(isObject), {}, `problem.${key}`);
  const conditions = caseData.problem.boundary_conditions ?? {};
  for (const region of Object.keys(conditions)) ensure(conditions, region, isObject, {}, `problem.boundary_conditions.${region}`);

  ensure(caseData, "assembly", (a) => isObject(a) && (a.t === undefined || Number.isFinite(a.t)), defaults.assembly, "assembly");
  // Cases written before the solvers had "solve": null; they get the
  // default section without a notice.
  if (caseData.solve === null || caseData.solve === undefined) caseData.solve = structuredClone(defaults.solve);
  ensure(caseData, "solve", (s) => isObject(s) && ["static", "time"].includes(s.mode) && isChoice(s.linear_solver), defaults.solve, "solve");
  ensure(caseData.solve, "static", isObject, defaults.solve.static, "solve.static");
  ensure(caseData.solve, "time", isObject, defaults.solve.time, "solve.time");
  for (const key of ["method", "step"]) ensure(caseData.solve.time, key, isChoice, defaults.solve.time[key], `solve.time.${key}`);
  ensure(caseData, "view", optional(isObject), {}, "view");
  if (caseData.view !== undefined) {
    const edits = caseData.geometry.region_edits ?? [];
    const undo = caseData.view.region_edit_undo;
    if (undo !== undefined && !(Array.isArray(undo) && undo.length === edits.length && undo.every((r) => r === null || isObject(r)))) {
      delete caseData.view.region_edit_undo;
      replaced.push("view.region_edit_undo");
    }
    ensure(caseData.view, "condition_archive", optional(isObject), {}, "view.condition_archive");
    const archive = caseData.view.condition_archive ?? {};
    for (const region of Object.keys(archive))
      if (!isObject(archive[region])) {
        delete archive[region];
        replaced.push(`view.condition_archive.${region}`);
      }
  }
  return { caseData, replaced };
}

// The path an error is about: its "path" field from the server, or for a
// message without one, the path it starts with ("mesh.refiner.options.x:
// ..."), or null.
export function errorPathOf(error) {
  if (typeof error?.path === "string" && error.path) return error.path;
  return errorPath(error?.message);
}

// The path an error message names ("mesh.refiner.options.x: ..."), or
// null when it names none.
export function errorPath(message) {
  const match = /^([A-Za-z_][\w.\[\]-]*): /.exec(message ?? "");
  return match ? match[1] : null;
}

// The stages, in order: one place for their ids, labels, the case
// sections each result depends on, the stages that must be fresh before
// it runs, the API call (a key of api.js's `api`) and what a panel says
// while it runs. `step: false` marks a result that has no step of its own
// (a field shown from the Assemble step).
export const STAGES = [
  { id: "geometry", label: "Geometry", sections: ["geometry"], requires: [], call: "geometry", running: "Reading the geometry", long: true },
  { id: "mesh", label: "Mesh", sections: ["geometry", "mesh"], requires: ["geometry"], call: "mesh", running: "Meshing", long: true },
  { id: "problem", label: "Problem", sections: ["problem"], requires: [], call: "problem", running: "Checking", long: false },
  { id: "assemble", label: "Assemble", sections: ["geometry", "mesh", "problem", "assembly"], requires: ["geometry", "mesh"], call: "assemble", running: "Assembling", long: true },
  { id: "solve", label: "Solve", sections: ["geometry", "mesh", "problem", "solve"], requires: ["geometry", "mesh"], call: "solve", running: "Solving", long: true },
  { id: "field", label: "Field", sections: ["geometry", "mesh", "problem", "assembly"], requires: ["geometry", "mesh"], call: "field", running: "Evaluating", long: true, step: false },
];

export function stageInfo(stage) {
  const info = STAGES.find((s) => s.id === stage);
  if (!info) throw new Error(`unknown stage ${stage}`);
  return info;
}

// The stages shown as steps, in order.
export const STEPS = STAGES.filter((s) => s.step !== false).map((s) => s.id);

// What Run all brings up to date, in order.
export const RUN_ALL = ["geometry", "mesh", "assemble", "solve"];

// The keys a stage's result depends on: a result is stale when the case's
// key differs from the one it was built from.
export function stageKey(caseData, stage) {
  return JSON.stringify(stageInfo(stage).sections.map((p) => caseData?.[p] ?? null));
}

// What a stage's request carries: the case's format and version and the
// sections its result depends on, nothing else (a problem check need not
// send an SVG or a mesh file). The server reads only those sections, so
// its cache keys are the same as for the whole case.
export function requestCase(caseData, stage) {
  const out = { format: caseData?.format, version: caseData?.version };
  for (const section of stageInfo(stage).sections) if (caseData?.[section] !== undefined) out[section] = caseData[section];
  return out;
}

// The stages to run, in order, to run `stage`: its prerequisites that are
// not fresh (isFresh(id)), then the stage itself.
export function stagesToRun(stage, isFresh) {
  return [...stageInfo(stage).requires.filter((id) => !isFresh(id)), stage];
}

// The bytes the solution's values take on the way to the page (float32,
// one per node and kept time), or null when the case's solve section
// cannot tell (the server checks it).
export function solutionBytes(solveSection, nodes) {
  if (!Number.isFinite(nodes) || !solveSection) return null;
  if (solveSection.mode === "static") return 4 * nodes;
  const snapshots = solveSection.time?.snapshots ?? null;
  if (!Number.isInteger(snapshots) || snapshots < 1) return null;
  return 4 * nodes * (snapshots + 1);
}

// Above this the Solve step warns before a run (the values travel to the
// page and stay in its memory).
export const LARGE_SOLUTION_BYTES = 50 * 1024 * 1024;

// Where a picture drawn at view `from` ({scale, offset: {x, y}}, CSS
// pixels) lands at view `to`: drawn scaled by `scale` with its top left
// corner at (x, y). A point at p on the old picture is at
// to.offset + (p - from.offset) * scale.
export function viewTransform(from, to) {
  const scale = to.scale / from.scale;
  return { scale, x: to.offset.x - from.offset.x * scale, y: to.offset.y - from.offset.y * scale };
}

// The colour position (0..1, scale.position) of every node's value, or
// NaN where it has none. Positions are linear in the value for every
// scale kind, so interpolating them is interpolating the values.
export function nodePositions(values, scale) {
  const out = new Float32Array(values.length);
  for (let i = 0; i < values.length; i++) {
    const v = values[i];
    out[i] = v === null || !Number.isFinite(v) ? NaN : scale.position(v);
  }
  return out;
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

// How many snapshots of a run the page keeps at most: about 64 MB of
// float32 values, from 8 to 1000 snapshots.
export function keptSnapshots(nodes) {
  if (!Number.isFinite(nodes) || nodes <= 0) return 1000;
  return Math.max(8, Math.min(1000, Math.floor((64 * 1024 * 1024) / (4 * nodes))));
}

// What to fetch and what to drop so that snapshot `shown` and its
// neighbours are loaded: `values` has one entry per ready snapshot (null
// where not loaded). Fetches the missing ones in [shown − behind, shown +
// ahead] as one range {first, count} (or null), and drops the loaded ones
// farthest from `shown` beyond `kept`.
export function snapshotPlan(values, shown, { ahead = 8, behind = 2, kept = 1000 } = {}) {
  const ready = values.length;
  if (ready === 0) return { fetch: null, evict: [] };
  const k = Math.max(0, Math.min(ready - 1, shown));
  const low = Math.max(0, k - behind);
  const high = Math.min(ready - 1, k + ahead);
  let first = -1;
  let last = -1;
  for (let i = low; i <= high; i++)
    if (!values[i]) {
      if (first < 0) first = i;
      last = i;
    }
  const fetch = first < 0 ? null : { first, count: last - first + 1 };
  const loaded = [];
  for (let i = 0; i < ready; i++) if (values[i] || (fetch && i >= fetch.first && i < fetch.first + fetch.count)) loaded.push(i);
  const evict = [];
  if (loaded.length > kept) {
    loaded.sort((a, b) => Math.abs(b - k) - Math.abs(a - k) || b - a);
    for (const i of loaded.slice(0, loaded.length - kept)) if (values[i]) evict.push(i);
  }
  return { fetch, evict };
}

// The loaded snapshot nearest to k (k itself when loaded), or -1.
export function nearestLoaded(values, k) {
  for (let d = 0; d < values.length; d++) {
    if (values[k - d]) return k - d;
    if (values[k + d]) return k + d;
  }
  return -1;
}

// Splits float32 little-endian values (snapshots x nodes, in order) into
// one Float32Array per snapshot. Throws when the size does not fit.
export function splitSnapshots(buffer, snapshots, nodes) {
  if (buffer.byteLength !== 4 * snapshots * nodes) throw new Error(`the solution has ${buffer.byteLength} bytes, expected ${4 * snapshots * nodes}`);
  const view = new DataView(buffer);
  const out = [];
  for (let k = 0; k < snapshots; k++) {
    const values = new Float32Array(nodes);
    for (let i = 0; i < nodes; i++) values[i] = view.getFloat32(4 * (k * nodes + i), true);
    out.push(values);
  }
  return out;
}

// A number field's text as the case gets it: {kind: "empty"} (no value:
// the default), {kind: "number", value} or {kind: "invalid"} for text
// that is not (yet) a number, such as "-" or "1e". `badInput` is the
// input's validity.badInput (a number input reports "" for such text).
export function parseNumberText(text, badInput = false) {
  if (badInput) return { kind: "invalid" };
  const trimmed = String(text ?? "").trim();
  if (trimmed === "") return { kind: "empty" };
  const value = Number(trimmed);
  return Number.isFinite(value) ? { kind: "number", value } : { kind: "invalid" };
}

// The case's parameters with `from` renamed to `to`, in the same order.
// Throws an Error naming the problem when `to` is empty or already taken.
export function renameParameter(parameters, from, to) {
  const name = String(to ?? "").trim();
  if (name === "") throw new Error("a parameter needs a name");
  if (name !== from && Object.hasOwn(parameters, name)) throw new Error(`a parameter named ${name} already exists`);
  const next = {};
  for (const [key, value] of Object.entries(parameters)) next[key === from ? name : key] = value;
  return next;
}

// The mesh format of a file name: "gmsh", "pde2d", "svg" or null.
export function fileKind(name) {
  const lower = name.toLowerCase();
  if (lower.endsWith(".svg")) return "svg";
  if (lower.endsWith(".msh")) return "gmsh";
  if (lower.endsWith(".pde2d")) return "pde2d";
  if (lower.endsWith(".json")) return "case";
  return null;
}

// --- settings ------------------------------------------------------------------

// The app's preferences (settings.js) and their defaults.
export const SETTINGS_DEFAULTS = Object.freeze({
  checkWhileTyping: true,
  checkDelayMs: 350,
  playbackFps: 12,
  followRun: true,
  digits: 4,
  pixelRatioCap: 2,
  isolineCount: 10,
});

const SETTING_RANGES = {
  checkDelayMs: [100, 3000],
  playbackFps: [1, 60],
  digits: [2, 10],
  pixelRatioCap: [1, 4],
  isolineCount: [2, 40],
};

// Settings as stored, made whole: a missing or malformed value takes its
// default, a number out of range is clamped, unknown keys are dropped.
export function normalizeSettings(input) {
  const out = { ...SETTINGS_DEFAULTS };
  if (!isObject(input)) return out;
  for (const [key, fallback] of Object.entries(SETTINGS_DEFAULTS)) {
    const value = input[key];
    if (typeof fallback === "boolean") {
      if (typeof value === "boolean") out[key] = value;
    } else if (typeof value === "number" && Number.isFinite(value)) {
      const [low, high] = SETTING_RANGES[key];
      const clamped = Math.min(high, Math.max(low, value));
      out[key] = key === "pixelRatioCap" ? clamped : Math.round(clamped);
    }
  }
  return out;
}

// The folded sections, as stored: {id: true (folded) or false (open)}.
export function parseFolds(text) {
  try {
    const value = JSON.parse(text ?? "null");
    if (!isObject(value)) return Object.create(null);
    const out = Object.create(null);
    for (const [key, folded] of Object.entries(value)) if (typeof folded === "boolean") out[key] = folded;
    return out;
  } catch {
    return Object.create(null);
  }
}

// --- finding points in a mesh ----------------------------------------------------

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

// A probe's value at every snapshot (one Float64Array, NaN outside).
export function probeSeries(found, snapshots) {
  const out = new Float64Array(snapshots.length);
  for (let k = 0; k < snapshots.length; k++) out[k] = interpolateAt(found, snapshots[k]);
  return out;
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

// Probes pinned on the picture keep their slot (and so their colour)
// while others come and go: the lowest free slot of `max`, or -1.
export const MAX_PROBES = 4;
export function freeProbeSlot(probes, max = MAX_PROBES) {
  const taken = new Set(probes.map((p) => p.slot));
  for (let slot = 0; slot < max; slot++) if (!taken.has(slot)) return slot;
  return -1;
}

// --- isolines --------------------------------------------------------------------

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

// --- comparing with an exact solution -----------------------------------------------

// u − exact at every node (NaN where either has no finite value).
export function errorValues(u, exact) {
  const out = new Float32Array(u.length);
  for (let i = 0; i < u.length; i++) {
    const a = u[i];
    const b = exact[i];
    out[i] = a === null || b === null || !Number.isFinite(a) || !Number.isFinite(b) ? NaN : a - b;
  }
  return out;
}

// The largest |value| (the L∞ norm over the nodes), NaN when there is no
// finite value.
export function maxAbs(values) {
  let best = NaN;
  for (const v of values) {
    if (v === null || !Number.isFinite(v)) continue;
    const a = Math.abs(v);
    if (!(a <= best)) best = a;
  }
  return best;
}

// The smallest and largest finite value of one snapshot ({min, max},
// NaN when it has none).
export function valueRange(values) {
  let min = Infinity;
  let max = -Infinity;
  for (const v of values) {
    if (v === null || !Number.isFinite(v)) continue;
    if (v < min) min = v;
    if (v > max) max = v;
  }
  return min === Infinity ? { min: NaN, max: NaN } : { min, max };
}

// --- charts -----------------------------------------------------------------------

// About `count` round tick values covering [min, max] (1, 2 or 5 times a
// power of ten apart, the one nearest to an even split), from the first multiple at or below min to the
// last at or above max.
export function niceTicks(min, max, count = 4) {
  if (!Number.isFinite(min) || !Number.isFinite(max)) return [];
  if (max === min) {
    const pad = Math.abs(min) > 0 ? Math.abs(min) * 0.1 : 1;
    return niceTicks(min - pad, max + pad, count);
  }
  const raw = (max - min) / Math.max(1, count);
  const power = 10 ** Math.floor(Math.log10(raw));
  const ratio = raw / power;
  const step = (ratio < 1.5 ? 1 : ratio < 3 ? 2 : ratio < 7 ? 5 : 10) * power;
  const first = Math.floor(min / step + 1e-9) * step;
  const ticks = [];
  for (let v = first; v <= max + step * (1 - 1e-9); v += step) ticks.push(Math.abs(v) < step * 1e-9 ? 0 : Number(v.toPrecision(12)));
  return ticks;
}

// The file name of a saved picture: the case's name and, for a solution
// in time, the shown time ("heat-t0.25.png").
export function pictureName(caseName, t = null) {
  const stem = String(caseName ?? "case").replace(/\.json$/i, "").replace(/[^\w.-]+/g, "_") || "case";
  if (t === null || !Number.isFinite(t)) return `${stem}.png`;
  return `${stem}-t${String(Number(t.toPrecision(6))).replace(/[^\w.-]+/g, "_")}.png`;
}
