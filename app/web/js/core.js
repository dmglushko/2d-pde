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
});

const SETTING_RANGES = {
  checkDelayMs: [100, 3000],
  playbackFps: [1, 60],
  digits: [2, 10],
  pixelRatioCap: [1, 4],
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
