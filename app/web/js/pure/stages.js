// The stage registry, stage keys, requests and error paths. Pure.

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

// Whether a solve result (or the case's solve section) runs in time: the
// mode "time" or the adaptive one in time.
export function isTimeRun(result) {
  return result?.mode === "time" || result?.mode === "adaptive_time";
}

// The bytes the solution's values take on the way to the page (float32,
// one per node and kept time), or null when the case's solve section
// cannot tell (the server checks it).
export function solutionBytes(solveSection, nodes) {
  if (!Number.isFinite(nodes) || !solveSection) return null;
  if (solveSection.mode === "static") return 4 * nodes;
  // An adaptive solve sends its last solution only, on a mesh not known
  // before it is solved (at most max_nodes nodes).
  if (solveSection.mode === "adaptive") {
    const most = solveSection.adaptive?.max_nodes;
    return Number.isInteger(most) && most > 0 ? 4 * most : null;
  }
  const snapshots = solveSection.time?.snapshots ?? null;
  if (!Number.isInteger(snapshots) || snapshots < 1) return null;
  // Adaptive in time: each snapshot on its mesh, of at most max_nodes
  // nodes (and the meshes themselves, a few times the values each).
  if (solveSection.mode === "adaptive_time") {
    const { max_nodes: most, max_refinements: refinements } = solveSection.adaptive_time ?? {};
    if (!Number.isInteger(most) || most < 1 || !Number.isInteger(refinements) || refinements < 0) return null;
    return 4 * most * (snapshots + 1) + 28 * most * (refinements + 1);
  }
  return 4 * nodes * (snapshots + 1);
}

// Above this the Solve step warns before a run (the values travel to the
// page and stay in its memory).
export const LARGE_SOLUTION_BYTES = 50 * 1024 * 1024;

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

// The step whose panel holds the case field at `path` ("mesh.spec...",
// "assembly.t", "view.exact_solution", ...); the Geometry step for a path
// it does not know.
export function stepOfPath(path) {
  const section = String(path ?? "").split(/[.[]/)[0];
  return { mesh: "mesh", problem: "problem", assembly: "assemble", solve: "solve", view: "solve" }[section] ?? "geometry";
}

// What a run of the solve section keeps on the server, from the mesh's
// answer (`memory`: {snapshot_bytes, budget}): {bytes, budget, share}
// with share = bytes / budget, or null when it cannot be told. A time run
// keeps snapshots + 1 snapshots, a static one 1.
export function serverRunBytes(solveSection, memory, timeOrder = null) {
  const perSnapshot = memory?.snapshot_bytes;
  const budget = memory?.budget;
  if (!Number.isFinite(perSnapshot) || !(budget > 0) || !solveSection) return null;
  // An adaptive run reserves room for max_cycles meshes and solutions of
  // at most max_nodes nodes (runs.cpp, adaptive_run_bytes).
  if (solveSection.mode === "adaptive") {
    const { max_nodes: nodes, max_cycles: cycles } = solveSection.adaptive ?? {};
    if (!Number.isInteger(nodes) || !Number.isInteger(cycles) || nodes < 1 || cycles < 1) return null;
    const bytes = cycles * (nodes * 96 + 512);
    return { bytes, budget, share: bytes / budget };
  }
  // An adaptive run in time reserves room for max_refinements + 1 meshes
  // and its snapshots of at most max_nodes nodes (runs.cpp,
  // adaptive_time_run_bytes, with the state of a first-order problem).
  if (solveSection.mode === "adaptive_time") {
    const { max_nodes: nodes, max_refinements: refinements } = solveSection.adaptive_time ?? {};
    const snapshots = solveSection.time?.snapshots;
    if (!Number.isInteger(nodes) || nodes < 1 || !Number.isInteger(refinements) || refinements < 0 || !Number.isInteger(snapshots) || snapshots < 1) return null;
    // Time order 2 (from the problem check; unknown: 1): the state holds
    // u_t and u_tt too, and the library each snapshot's until it returns.
    const second = timeOrder === 2;
    const bytes = (refinements + 1) * (nodes * 88 + 512) + (snapshots + 1) * (nodes * (second ? 24 : 8) + 128) + nodes * (second ? 24 : 8);
    return { bytes, budget, share: bytes / budget };
  }
  let count = 1;
  if (solveSection.mode !== "static") {
    const snapshots = solveSection.time?.snapshots;
    if (!Number.isInteger(snapshots) || snapshots < 1) return null;
    count = snapshots + 1;
  }
  const bytes = perSnapshot * count;
  return { bytes, budget, share: bytes / budget };
}
