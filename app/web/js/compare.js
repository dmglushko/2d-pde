// Comparing the solution with an exact solution written as an expression,
// or with the reference run (a run kept on the server). The server
// computes both in double (POST /api/solve/compare): the library
// evaluates the expression at the nodes at each snapshot's time, so its
// checks and errors stay the library's, and the page's float32 copy of u
// plays no part. u − other of one snapshot is fetched when the picture
// shows it (POST /api/solve/compare/difference, float32).
//
// A comparison belongs to the solution it was made for and to what it
// compared with: the expression's text, or the reference's job. Another
// solution drops it; another expression in the case marks it outdated
// (shown greyed until compared again).

import { api, ApiError } from "./api.js";
import { fieldScale } from "./pure/color.js";
import { snapshotNodes } from "./pure/run_values.js";
import { freshResult, notify, requestTicket, setViewOption, store } from "./state.js";

const empty = () => ({
  result: null,
  against: null,
  summary: null,
  errors: [],
  linf: new Float64Array(0),
  rms: new Float64Array(0),
  l2: new Float64Array(0),
  done: 0,
  busy: false,
  error: null,
  scale: null,
  // failures[k]: the error of the last fetch of difference k (no new
  // fetch until retryDifference).
  failures: [],
});

// One comparison per kind ("exact", "reference"): `result` is the solve
// data it was made for; `against` is {exact: text} or {reference: job};
// `linf[k]`, `rms[k]` and `l2[k]` are the server's figures per snapshot;
// `errors[k]` the differences fetched so far (Float32Array).
export const comparisons = { exact: empty(), reference: empty() };

export const LABELS = { exact: "u − exact", reference: "u − reference" };

// The comparison of `kind` for the current solution, or null (none, or
// one made for an earlier solution).
export function currentComparison(kind) {
  const result = freshResult("solve");
  const c = comparisons[kind];
  return result && c.result === result ? c : null;
}

// True when the comparison shown was made for something else than what
// the case has now: another expression, or another reference run.
export function isOutdated(kind) {
  const c = currentComparison(kind);
  if (!c?.against) return false;
  if (kind === "exact") return c.against.exact !== (store.caseData.view?.exact_solution ?? "").trim();
  return store.reference?.job !== c.against.reference;
}

// The state of the comparison of `kind` for the current solution, as the
// picture and the controls read it (pure/display.js): "none", "busy",
// "failed", "outdated" or "current".
export function comparisonState(kind) {
  const c = currentComparison(kind);
  if (!c) return "none";
  if (c.busy) return "busy";
  if (c.error) return "failed";
  if (isOutdated(kind)) return "outdated";
  return c.summary && c.done > 0 ? "current" : "none";
}

const figures = (snapshots, key) => Float64Array.from(snapshots, (s) => (s && Number.isFinite(s[key]) ? s[key] : NaN));
const requestBody = (against) => ("exact" in against ? { exact: against.exact } : { reference: true });

// Compares the current solution with {exact: text} or {reference: job}.
export async function compare(kind, against) {
  const result = freshResult("solve");
  if (!result || result.state === "running" || result.values.length === 0) return;
  const ticket = requestTicket(`compare-${kind}`, () => freshResult("solve"));
  const count = result.values.length;
  comparisons[kind] = {
    ...empty(),
    result,
    against,
    errors: new Array(count).fill(null),
    linf: new Float64Array(count).fill(NaN),
    rms: new Float64Array(count).fill(NaN),
    l2: new Float64Array(count).fill(NaN),
    busy: true,
  };
  const c = comparisons[kind];
  notify("compare");
  let summary;
  try {
    summary = await api.compare(result.job, requestBody(against));
  } catch (error) {
    if (!ticket.holds()) return;
    c.error = error instanceof ApiError ? error : new ApiError({ stage: "compare", message: String(error) });
    c.busy = false;
    notify("compare");
    return;
  }
  if (!ticket.holds()) return;
  c.summary = summary;
  c.linf = figures(summary.snapshots, "max_abs");
  c.rms = figures(summary.snapshots, "rms");
  c.l2 = figures(summary.snapshots, "l2");
  c.done = summary.snapshots.length;
  // One diverging scale over every snapshot, symmetric around zero.
  const largest = Number.isFinite(summary.max_abs) ? summary.max_abs : 0;
  c.scale = fieldScale([-largest, largest]);
  c.busy = false;
  notify("compare");
}

export const compareWith = (text) => compare("exact", { exact: text });

// u − other at snapshot k when it has arrived; otherwise starts fetching
// it (unless its last fetch failed) and returns null (the picture is
// drawn again once it is here).
const fetching = new Set();
export function comparisonValues(kind, k) {
  const c = currentComparison(kind);
  if (!c || !c.summary || c.error) return null;
  if (c.errors[k]) return c.errors[k];
  const id = `${kind}:${k}`;
  if (!c.summary.snapshots[k] || fetching.has(id) || c.failures[k]) return null;
  const ticket = requestTicket(`difference-${kind}`, () => comparisons[kind], { exclusive: false });
  fetching.add(id);
  api
    .compareDifference(c.result.job, requestBody(c.against), k, snapshotNodes(c.result.layout, k))
    .then((values) => {
      if (!ticket.holds()) return;
      c.errors[k] = values;
      notify("compare");
    })
    .catch((error) => {
      if (!ticket.holds()) return;
      c.failures[k] = error;
      notify("compare");
    })
    .finally(() => fetching.delete(id));
  return null;
}

// What the picture can show of u − other at snapshot k (pure/display.js):
// its values, else the nearest fetched one, and the last fetch's failure.
export function differenceState(kind, k) {
  const c = currentComparison(kind);
  const values = comparisonValues(kind, k);
  if (!c) return { values: null, nearest: null, failure: null };
  let near = -1;
  if (!values)
    for (let d = 1; d < c.errors.length && near < 0; d++) {
      if (c.errors[k - d]) near = k - d;
      else if (c.errors[k + d]) near = k + d;
    }
  return { values, nearest: near >= 0 ? { index: near, values: c.errors[near] } : null, failure: c.failures[k] ?? null };
}

// Tries difference k of `kind` again after a failed fetch.
export function retryDifference(kind, k) {
  const c = currentComparison(kind);
  if (!c) return;
  c.failures[k] = null;
  notify("compare");
}

// Forgets the comparison of `kind` (another solution, the text cleared,
// the reference forgotten).
export function clearComparison(kind) {
  requestTicket(`compare-${kind}`);
  comparisons[kind] = empty();
  if (store.view.difference === kind) setViewOption("difference", null);
  notify("compare");
}

// After a new solution: compare it again with what was used last (the
// expression, if it is still the case's; the reference, if it is still
// kept), so a finer mesh or another dt shows its error at once.
export function recompareIfNeeded() {
  const result = freshResult("solve");
  if (!result || result.state !== "done") return;
  for (const kind of ["exact", "reference"]) {
    const c = comparisons[kind];
    if (c.busy || c.result === result || !c.against || c.error) continue;
    if (kind === "exact" && c.against.exact !== (store.caseData.view?.exact_solution ?? "").trim()) continue;
    if (kind === "reference" && store.reference?.job !== c.against.reference) continue;
    compare(kind, c.against);
  }
}

// --- the reference run ---------------------------------------------------------

// Keeps the current (done) solution as the client's reference run.
// Returns null when it worked, else the error.
export async function keepReference() {
  const result = freshResult("solve");
  if (!result || result.state !== "done") return null;
  const ticket = requestTicket("reference");
  try {
    const answer = await api.keepReference(result.job);
    if (!ticket.newest()) return null;
    store.reference = answer.reference;
    clearComparison("reference");
    return null;
  } catch (error) {
    return error;
  }
}

// Forgets the reference run on the server.
export async function forgetReference() {
  const ticket = requestTicket("reference");
  try {
    await api.clearReference();
  } catch (error) {
    return error;
  }
  if (!ticket.newest()) return null;
  store.reference = null;
  clearComparison("reference");
  return null;
}
