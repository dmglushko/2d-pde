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
// are kept per client, so two tabs do not replace each other's run. It is
// kept in this tab's sessionStorage, so a reload of the tab is the same
// client and finds its run again (actions.js, reattachRun). A duplicated
// tab copies sessionStorage too: claimClientId asks the other tabs on a
// BroadcastChannel whether one of them already uses the id, and takes a
// new one if so (a reloading tab is gone by then and does not answer).
function randomId() {
  const bytes = new Uint8Array(12);
  if (globalThis.crypto?.getRandomValues) globalThis.crypto.getRandomValues(bytes);
  else for (let i = 0; i < bytes.length; i++) bytes[i] = Math.floor(Math.random() * 256);
  return Array.from(bytes, (b) => b.toString(16).padStart(2, "0")).join("");
}
const CLIENT_KEY = "pde2d-client";
const storedClient = (() => {
  try {
    const id = globalThis.sessionStorage?.getItem(CLIENT_KEY);
    return id && /^[0-9a-f]{24}$/.test(id) ? id : null;
  } catch {
    return null;
  }
})();
export let clientId = storedClient ?? randomId();
// True when the id came from this tab's storage (a reload, or a copy).
export const clientRestored = storedClient !== null;
function keepClientId() {
  try {
    globalThis.sessionStorage?.setItem(CLIENT_KEY, clientId);
  } catch {
    // Not kept: a reload is then a new client.
  }
}
keepClientId();

const CLAIM_MS = 150;
// This page instance, told apart from a copy that has the same id.
const nonce = randomId();
let channel = null;
// Answers other tabs that ask for this tab's id, and with a restored id
// first asks them: resolves with the id this tab uses.
export function claimClientId({ wait = CLAIM_MS } = {}) {
  const Channel = globalThis.BroadcastChannel;
  if (!Channel) return Promise.resolve(clientId);
  channel ??= new Channel("pde2d-client-ids");
  let taken = false;
  channel.onmessage = (event) => {
    const message = event.data ?? {};
    if (message.ask === clientId && message.from !== nonce) channel.postMessage({ taken: clientId, to: message.from });
    if (message.taken === clientId && message.to === nonce) taken = true;
  };
  if (!clientRestored) return Promise.resolve(clientId);
  channel.postMessage({ ask: clientId, from: nonce });
  return new Promise((resolve) =>
    setTimeout(() => {
      if (taken) {
        clientId = randomId();
        keepClientId();
      }
      resolve(clientId);
    }, wait),
  );
}
// The token of a server that listens beyond this machine: it is in the
// address the server printed (?token=...). The page keeps it in this tab's
// sessionStorage and takes it out of the address bar (so it is not in a
// copied link, a bookmark or the history from then on); it goes with every
// request in X-PDE2D-Token (the server takes it only there).
const tokenKey = "pde2d-token";
const token = (() => {
  try {
    const url = new URL(globalThis.location.href);
    const given = url.searchParams.get("token");
    if (given) {
      try {
        globalThis.sessionStorage?.setItem(tokenKey, given);
      } catch {
        // Storage refused: the token still lives in this page.
      }
      url.searchParams.delete("token");
      globalThis.history?.replaceState(globalThis.history.state, "", url.pathname + url.search + url.hash);
      return given;
    }
    return globalThis.sessionStorage?.getItem(tokenKey) ?? null;
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
const noAnswer = (stage = "request") => new ApiError({ stage, kind: "network", message: "The app server does not answer. Is pde2d_app still running?" });

// Requests that wait for the server's session (every call but the light
// ones: progress, stop, values, probes, status, current), with when each
// started: the page shows what the server is busy with while any waits
// (status.js).
const LIGHT = new Set(["/api/solve/progress", "/api/solve/stop", "/api/solve/values", "/api/solve/probes", "/api/status", "/api/solve/current"]);
const waiting = new Map();
let requestCount = 0;
function track(path) {
  if (LIGHT.has(path)) return () => {};
  const id = ++requestCount;
  waiting.set(id, globalThis.performance?.now() ?? Date.now());
  return () => waiting.delete(id);
}
// How long the oldest request that waits for the session has waited (ms),
// or null when none does.
export function oldestWait() {
  if (waiting.size === 0) return null;
  const now = globalThis.performance?.now() ?? Date.now();
  return now - Math.min(...waiting.values());
}

// A POST of `body` as JSON; the answer as an ArrayBuffer.
async function requestBinary(path, body) {
  let response;
  const done = track(path);
  try {
    response = await fetch(path, postOptions(body));
  } catch {
    throw noAnswer();
  } finally {
    done();
  }
  if (!response.ok) {
    const data = await response.json().catch(() => null);
    throw new ApiError(data?.error ?? { message: `${response.status} ${response.statusText}` });
  }
  return response.arrayBuffer();
}

async function request(path, body) {
  let response;
  const done = track(path);
  try {
    response = await fetch(path, body === undefined ? { headers: apiHeaders() } : postOptions(body));
  } catch {
    throw noAnswer();
  } finally {
    done();
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
  // Where on the boundary a point is, for a split there: {region,
  // fraction or node, point, corner, splittable, reason}.
  locate: (caseData, point, snap) => request("/api/geometry/locate", { case: caseData, point, snap }),
  // The case's curves (after its region edits) as a drawing section.
  toDrawing: (caseData) => request("/api/geometry/drawing", caseData),
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
  // This client's run of the case, if the server still has it: {run:
  // progress or null}. Never starts one.
  currentRun: (caseData) => request("/api/solve/current", caseData),
  status: () => request("/api/status"),
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
