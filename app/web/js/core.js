// Pure helpers: no DOM, so Node tests them (app/tests/web.test.mjs).

// --- numbers -----------------------------------------------------------------

// A short, readable form of a number for labels and tables.
export function formatNumber(value, digits = 4) {
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
  if (geometry.source !== "mesh_file") ensure(geometry, "region_edits", optional(isList(isObject)), [], "geometry.region_edits");

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

// The keys a stage's result depends on: a result is stale when the case's
// key differs from the one it was built from.
export function stageKey(caseData, stage) {
  const parts = { geometry: ["geometry"], mesh: ["geometry", "mesh"], problem: ["problem"], assemble: ["geometry", "mesh", "problem", "assembly"] }[stage];
  return JSON.stringify(parts.map((p) => caseData?.[p] ?? null));
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
