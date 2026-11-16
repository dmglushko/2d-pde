// The server's JSON API (docs/app.md, "The API").

import { decodeMesh } from "./pure/mesh_math.js";
import { splitSnapshots } from "./pure/run_values.js";

export class ApiError extends Error {
  constructor(error) {
    super(error?.message ?? "the request failed");
    this.stage = error?.stage ?? "request";
    this.kind = error?.kind ?? "internal";
    // The place in the case the error is about (null when it names none).
    this.path = error?.path ?? null;
  }
}

// This page's client id: runs, reference runs and progress on the server
// are kept per client, so two tabs do not replace each other's run. One
// per page instance (a reload is a new client: it has no run to follow).
function randomId() {
  const bytes = new Uint8Array(12);
  if (globalThis.crypto?.getRandomValues) globalThis.crypto.getRandomValues(bytes);
  else for (let i = 0; i < bytes.length; i++) bytes[i] = Math.floor(Math.random() * 256);
  return Array.from(bytes, (b) => b.toString(16).padStart(2, "0")).join("");
}
export const clientId = randomId();

// The token of a server that listens beyond this machine: it is in the
// address the server printed (?token=...), and goes with every request.
const token = (() => {
  try {
    return new URLSearchParams(globalThis.location?.search ?? "").get("token");
  } catch {
    return null;
  }
})();

// The headers every request carries.
export function apiHeaders(extra = {}) {
  const headers = { "X-PDE2D-Client": clientId, ...extra };
  if (token) headers["X-PDE2D-Token"] = token;
  return headers;
}

const postOptions = (body, extra) => ({ method: "POST", headers: apiHeaders({ "Content-Type": "application/json", ...extra }), body: JSON.stringify(body) });
const noAnswer = (stage = "request") => new ApiError({ stage, kind: "internal", message: "The app server does not answer. Is pde2d_app still running?" });

// A POST of `body` as JSON; the answer as an ArrayBuffer.
async function requestBinary(path, body) {
  let response;
  try {
    response = await fetch(path, postOptions(body));
  } catch {
    throw noAnswer();
  }
  if (!response.ok) {
    const data = await response.json().catch(() => null);
    throw new ApiError(data?.error ?? { message: `${response.status} ${response.statusText}` });
  }
  return response.arrayBuffer();
}

async function request(path, body) {
  let response;
  try {
    response = await fetch(path, body === undefined ? { headers: apiHeaders() } : postOptions(body));
  } catch {
    throw noAnswer();
  }
  const data = await response.json().catch(() => null);
  if (!response.ok) throw new ApiError(data?.error ?? { message: `${response.status} ${response.statusText}` });
  return data;
}

export const api = {
  catalog: () => request("/api/catalog"),
  examples: () => request("/api/examples"),
  example: (name) => request(`/api/examples/${encodeURIComponent(name)}`),
  geometry: (caseData) => request("/api/geometry", caseData),
  // The mesh's summary, then its arrays in binary (pure/mesh_math.js, decodeMesh):
  // the answer has "mesh" with nodes, triangles and boundary_edges as
  // typed arrays, as the picture uses them.
  mesh: async (caseData) => {
    const data = await request("/api/mesh", caseData);
    const summary = data.mesh;
    if (!summary?.arrays) throw new ApiError({ stage: "mesh", message: "the mesh answer has no arrays" });
    const buffer = await requestBinary(summary.arrays.path, { mesh: summary.id });
    data.mesh = decodeMesh(summary, buffer);
    return data;
  },
  problem: (caseData) => request("/api/problem", caseData),
  assemble: (caseData) => request("/api/assemble", caseData),
  field: (caseData, field) => request("/api/field", { case: caseData, field }),
  solve: (caseData) => request("/api/solve", caseData),
  solveProgress: (job) => request("/api/solve/progress", { job }),
  solveStop: (job) => request("/api/solve/stop", { job }),
  // Snapshots `first` to `first + count` of a run as one Float32Array
  // each; `layout` is the summary's "values" ({path, format, snapshots,
  // nodes}).
  solveRange: async (job, first, count, layout) => {
    if (layout?.format !== "float32le") throw new ApiError({ stage: "solve", message: `unknown values format ${layout?.format}` });
    if (count <= 0) return [];
    return splitSnapshots(await requestBinary(layout.path, { job, first, count }), count, layout.nodes);
  },
  // The run against {exact: text} or {reference: true}, in double.
  compare: (job, against) => request("/api/solve/compare", { job, ...against }),
  // u − other at snapshot k, as a Float32Array of `nodes` values.
  compareDifference: async (job, against, snapshot, nodes) => splitSnapshots(await requestBinary("/api/solve/compare/difference", { job, ...against, snapshot }), 1, nodes)[0],
  keepReference: (job) => request("/api/solve/reference", { job }),
  clearReference: () => request("/api/solve/reference/clear", {}),
  // u(t) of the run at points [[x, y], ...].
  probes: (job, points) => request("/api/solve/probes", { job, points }),
  preset: (caseData, preset) => request("/api/preset", { case: caseData, preset }),
};

// Asks the server for an export of the case and saves it as a file.
// `caseName` goes into the file's provenance.
export async function downloadExport(kind, caseData, caseName = "") {
  let response;
  try {
    response = await fetch(`/api/export/${kind}`, postOptions(caseData, caseName ? { "X-PDE2D-Case-Name": encodeURIComponent(caseName) } : {}));
  } catch {
    throw noAnswer("export");
  }
  if (!response.ok) {
    const data = await response.json().catch(() => null);
    throw new ApiError(data?.error ?? { stage: "export", message: response.statusText });
  }
  saveBlob(await response.blob(), kind);
}

export function saveBlob(blob, name) {
  const url = URL.createObjectURL(blob);
  const link = document.createElement("a");
  link.href = url;
  link.download = name;
  document.body.append(link);
  link.click();
  link.remove();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}
