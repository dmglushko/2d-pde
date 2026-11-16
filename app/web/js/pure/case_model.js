// The case: normalization, region edits and the conditions that move with
// them, parameter names, file kinds. Pure.

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

// The coefficients a form shows, in the catalog's order: those that are
// set (a non-empty text), the right side (the field without a term: f or
// g, always) and the names in `revealed` (shown once in this visit: a
// template's, or one being typed). The rest wait behind "Show all".
export function visibleCoefficients(fields, values, revealed = new Set()) {
  return fields.filter((f) => {
    const value = values?.[f.name];
    return (typeof value === "string" && value.trim() !== "") || !f.term || revealed.has(f.name);
  }).map((f) => f.name);
}
