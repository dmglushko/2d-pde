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

// The case with exactly one boundary condition per region of `regions`
// (the regions of a geometry the library accepted). Returns a new case;
// the input is not changed.
//
// - `edit` is the region edit that produced these regions, or null. A
//   rename gives the new name the old name's condition, a split gives both
//   halves the condition of the whole; a merge needs nothing (the target
//   keeps its own).
// - A condition whose region is gone moves to view.condition_archive, so
//   it comes back when its region does (an edit removed again).
// - A region without a condition takes it from the archive, else a copy of
//   `defaultCondition`.
// - The conditions are listed in the order of `regions`.
export function reconcileConditions(caseData, regions, defaultCondition, edit = null) {
  const next = structuredClone(caseData);
  const problem = (next.problem ??= {});
  const old = problem.boundary_conditions ?? {};
  const view = (next.view ??= {});
  const archive = (view.condition_archive ??= {});
  const conditions = {};
  const carried = {};
  if (edit?.op === "rename" && old[edit.from] && regions.includes(edit.to) && !(edit.to in old)) carried[edit.to] = old[edit.from];
  if (edit?.op === "split" && old[edit.region])
    for (const name of [edit.first, edit.second]) if (regions.includes(name) && (name === edit.region || !(name in old))) carried[name] = old[edit.region];
  for (const [name, condition] of Object.entries(old)) if (!regions.includes(name)) archive[name] = condition;
  for (const region of regions) {
    if (region in carried) conditions[region] = structuredClone(carried[region]);
    else if (region in old) conditions[region] = old[region];
    else if (region in archive) conditions[region] = archive[region];
    else conditions[region] = structuredClone(defaultCondition);
    delete archive[region];
  }
  problem.boundary_conditions = conditions;
  if (Object.keys(archive).length === 0) delete view.condition_archive;
  if (Object.keys(view).length === 0) delete next.view;
  return next;
}

const isObject = (value) => value !== null && typeof value === "object" && !Array.isArray(value);

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
  const sources = ["shape", "svg", "mesh_file"];
  ensure(caseData, "geometry", (g) => isObject(g) && sources.includes(g.source), defaults.geometry, "geometry");
  ensure(caseData, "mesh", isObject, defaults.mesh, "mesh");
  for (const key of ["spec", "discretizer", "triangulator"]) ensure(caseData.mesh, key, isObject, defaults.mesh[key], `mesh.${key}`);
  ensure(caseData.mesh, "refiner", (r) => r === null || isObject(r), defaults.mesh.refiner, "mesh.refiner");
  ensure(caseData.mesh, "improvers", (list) => Array.isArray(list) && list.every(isObject), defaults.mesh.improvers, "mesh.improvers");
  ensure(caseData, "problem", isObject, defaults.problem, "problem");
  for (const key of ["equation", "parameters", "boundary_conditions", "initial"])
    ensure(caseData.problem, key, (v) => v === undefined || isObject(v), {}, `problem.${key}`);
  ensure(caseData, "assembly", (a) => isObject(a) && (a.t === undefined || Number.isFinite(a.t)), defaults.assembly, "assembly");
  if (caseData.view !== undefined && !isObject(caseData.view)) {
    delete caseData.view;
    replaced.push("view");
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
