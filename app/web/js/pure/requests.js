// How the page treats a request that failed: which failures are worth
// trying again (and after how long), the plain words for each kind, and
// what the server says it is busy with. Pure: tested in Node
// (app/tests/web.test.mjs).

// A failure that may pass by itself: the server refused for the moment
// (429, kind "busy") or did not answer (kind "network").
export function isTransient(error) {
  return error?.kind === "busy" || error?.kind === "network";
}

// The pause before try number `attempt` (0 for the first retry): doubles
// from `base` up to `max` milliseconds.
export function backoffDelay(attempt, { base = 250, max = 4000 } = {}) {
  const n = Math.max(0, Math.floor(attempt));
  return Math.min(max, base * 2 ** Math.min(n, 20));
}

const sentence = (text) => {
  const s = String(text ?? "").trim();
  if (!s) return "";
  const capital = s.charAt(0).toUpperCase() + s.slice(1);
  return /[.!?]$/.test(capital) ? capital : `${capital}.`;
};

// An error of a run or of a request about it in plain words: {text,
// action}. action: "solve" (the result is gone: solve again), "retry" (try
// the request again), or null (nothing to do here). `what` names what was
// asked for ("the snapshot", "u(t) at the probes").
export function plainError(error, what = "the result") {
  const message = error?.message ?? String(error ?? "");
  switch (error?.kind) {
    case "replaced":
      return { text: "A newer solve in this page replaced this run.", action: null };
    case "expired":
      return { text: `The result is gone from the server: ${sentence(message).replace(/^./, (c) => c.toLowerCase())}`, action: "solve" };
    case "memory":
      return { text: `The server has no room for this run. ${sentence(message)}`, action: null };
    case "busy":
      return { text: `The server is busy with earlier requests; ${what} will be asked for again.`, action: "retry" };
    case "network":
      return { text: `The app server does not answer, so ${what} did not arrive. Is pde2d_app still running?`, action: "retry" };
    default:
      return { text: `${what.charAt(0).toUpperCase()}${what.slice(1)} could not be loaded: ${sentence(message)}`, action: "retry" };
  }
}

const TASKS = {
  geometry: "reading a geometry",
  mesh: "meshing",
  "mesh arrays": "sending a mesh",
  "problem check": "checking a problem",
  assembly: "assembling",
  field: "evaluating a field",
  solve: "starting a solve",
  run: "running a solve",
  comparison: "comparing with a solution",
  reference: "keeping a reference run",
  preset: "preparing a preset",
  export: "writing an export",
};

// "server busy: meshing (12 s)", or "server busy: meshing for another
// tab (12 s)", from GET /api/status; null when it is not busy.
export function serverBusyText(status) {
  const busy = status?.busy;
  if (!busy) return null;
  const task = TASKS[busy.task] ?? busy.task;
  const whose = busy.yours ? "" : " for another tab";
  const seconds = Math.max(0, Math.floor(busy.seconds ?? 0));
  return `server busy: ${task}${whose} (${seconds} s)`;
}
