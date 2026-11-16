// Comparing the solution with an exact solution written as an expression
// (or, through the API, with the reference run). The server computes it
// in double (POST /api/solve/compare): the library evaluates the
// expression at the nodes at each snapshot's time, so its checks and
// errors stay the library's, and the page's float32 copy of u plays no
// part. u − exact of one snapshot is fetched when the picture shows it
// (POST /api/solve/compare/difference).

import { api, ApiError } from "./api.js";
import { fieldScale } from "./core.js";
import { freshResult, notify, store } from "./state.js";

// The comparison of one solve result: `result` is the solve data it was
// made for; `linf[k]`, `rms[k]` and `l2[k]` are the server's figures per
// snapshot; `errors[k]` the differences fetched so far (Float32Array).
export const comparison = {
  result: null,
  text: "",
  label: "u − exact",
  summary: null,
  errors: [],
  linf: new Float64Array(0),
  rms: new Float64Array(0),
  l2: new Float64Array(0),
  done: 0,
  busy: false,
  error: null,
  scale: null,
  generation: 0,
};

// The comparison of the current solution, or null (none, or one made for
// an earlier solution).
export function currentComparison() {
  const result = freshResult("solve");
  return result && comparison.result === result ? comparison : null;
}

const figures = (snapshots, key) => Float64Array.from(snapshots, (s) => (s && Number.isFinite(s[key]) ? s[key] : NaN));

export async function compareWith(text) {
  const result = freshResult("solve");
  if (!result || result.state === "running" || result.values.length === 0) return;
  const generation = ++comparison.generation;
  const caseId = store.caseId;
  const count = result.values.length;
  Object.assign(comparison, {
    result,
    text,
    summary: null,
    errors: new Array(count).fill(null),
    linf: new Float64Array(count).fill(NaN),
    rms: new Float64Array(count).fill(NaN),
    l2: new Float64Array(count).fill(NaN),
    done: 0,
    busy: true,
    error: null,
    scale: null,
  });
  notify("compare");
  const live = () => generation === comparison.generation && caseId === store.caseId && freshResult("solve") === result;
  let summary;
  try {
    summary = await api.compare(result.job, { exact: text });
  } catch (error) {
    if (!live()) return;
    comparison.error = error instanceof ApiError ? error : new ApiError({ stage: "compare", message: String(error) });
    comparison.busy = false;
    notify("compare");
    return;
  }
  if (!live()) return;
  comparison.summary = summary;
  comparison.linf = figures(summary.snapshots, "max_abs");
  comparison.rms = figures(summary.snapshots, "rms");
  comparison.l2 = figures(summary.snapshots, "l2");
  comparison.done = summary.snapshots.length;
  // One diverging scale over every snapshot, symmetric around zero.
  const largest = Number.isFinite(summary.max_abs) ? summary.max_abs : 0;
  comparison.scale = fieldScale([-largest, largest]);
  comparison.busy = false;
  notify("compare");
}

// u − exact at snapshot k when it has arrived; otherwise starts fetching
// it and returns null (the picture is drawn again once it is here).
const fetching = new Set();
export function comparisonValues(k) {
  const current = currentComparison();
  if (!current || !current.summary || current.error) return null;
  if (current.errors[k]) return current.errors[k];
  if (!current.summary.snapshots[k] || fetching.has(k)) return null;
  const generation = comparison.generation;
  const result = current.result;
  fetching.add(k);
  api
    .compareDifference(result.job, { exact: current.text }, k, result.layout.nodes)
    .then((values) => {
      if (generation !== comparison.generation) return;
      comparison.errors[k] = values;
      notify("compare");
    })
    .catch(() => {})
    .finally(() => fetching.delete(k));
  return null;
}

// Forgets the comparison (another solution, or the text was cleared).
export function clearComparison() {
  comparison.generation += 1;
  Object.assign(comparison, { result: null, summary: null, errors: [], linf: new Float64Array(0), rms: new Float64Array(0), l2: new Float64Array(0), done: 0, busy: false, error: null, scale: null });
  notify("compare");
}

// After a new solution: compare it again with the text used last, so a
// finer mesh or another dt shows its error at once.
export function recompareIfNeeded() {
  const result = freshResult("solve");
  if (!result || result.state !== "done" || comparison.busy) return;
  if (comparison.result === result || !comparison.text || comparison.error) return;
  if (comparison.text !== (store.caseData.view?.exact_solution ?? "")) return;
  compareWith(comparison.text);
}
