// The server's JSON API (docs/app.md, "The API").

import { splitSnapshots } from "./core.js";

export class ApiError extends Error {
  constructor(error) {
    super(error?.message ?? "the request failed");
    this.stage = error?.stage ?? "request";
    this.kind = error?.kind ?? "internal";
  }
}

// A POST of `body` as JSON; the answer as an ArrayBuffer.
async function requestBinary(path, body) {
  let response;
  try {
    response = await fetch(path, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) });
  } catch {
    throw new ApiError({ stage: "request", kind: "internal", message: "The app server does not answer. Is pde2d_app still running?" });
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
    response = await fetch(path, body === undefined ? {} : { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) });
  } catch {
    throw new ApiError({ stage: "request", kind: "internal", message: "The app server does not answer. Is pde2d_app still running?" });
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
  mesh: (caseData) => request("/api/mesh", caseData),
  problem: (caseData) => request("/api/problem", caseData),
  assemble: (caseData) => request("/api/assemble", caseData),
  field: (caseData, field) => request("/api/field", { case: caseData, field }),
  solve: (caseData) => request("/api/solve", caseData),
  solveProgress: (job) => request("/api/solve/progress", { job }),
  solveStop: (job) => request("/api/solve/stop", { job }),
  // Snapshots `first` up to the ready ones of a run as one Float32Array
  // each; `layout` is the summary's "values" ({path, format, snapshots,
  // nodes}).
  solveValues: async (job, first, layout) => {
    if (layout?.format !== "float32le") throw new ApiError({ stage: "solve", message: `unknown values format ${layout?.format}` });
    const count = layout.snapshots - first;
    if (count <= 0) return [];
    return splitSnapshots(await requestBinary(layout.path, { job, first, count }), count, layout.nodes);
  },
  preset: (caseData, preset) => request("/api/preset", { case: caseData, preset }),
};

// Asks the server for an export of the case and saves it as a file.
export async function downloadExport(kind, caseData) {
  let response;
  try {
    response = await fetch(`/api/export/${kind}`, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(caseData) });
  } catch {
    throw new ApiError({ stage: "export", message: "The app server does not answer." });
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
