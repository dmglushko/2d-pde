// Comparing the solution with an exact solution written as an
// expression. The library evaluates the expression (POST /api/field,
// kind "expression") at each snapshot's time, so its checks and errors
// stay the library's; the page only subtracts: u − exact per node, its
// largest |value| per snapshot, and one colour scale over all snapshots.

import { api, ApiError } from "./api.js";
import { errorValues, fieldScale, maxAbs, requestCase, valueRange } from "./core.js";
import { freshResult, notify, store } from "./state.js";

// The comparison of one solve result: `result` is the solve data it was
// made for; `errors[k]` (Float32Array) and `linf[k]` fill in as the
// snapshots are evaluated (the shown one first).
export const comparison = {
  result: null,
  text: "",
  errors: [],
  linf: new Float64Array(0),
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

// How often a long comparison redraws while it goes on.
const NOTIFY_MS = 150;

export async function compareWith(text) {
  const result = freshResult("solve");
  if (!result || result.state === "running" || result.values.length === 0) return;
  const generation = ++comparison.generation;
  const caseId = store.caseId;
  const count = result.values.length;
  Object.assign(comparison, {
    result,
    text,
    errors: new Array(count).fill(null),
    linf: new Float64Array(count).fill(NaN),
    done: 0,
    busy: true,
    error: null,
    scale: null,
  });
  notify("compare");
  const live = () => generation === comparison.generation && caseId === store.caseId && freshResult("solve") === result;
  const shown = Math.min(store.solveFrame, count - 1);
  const order = [shown, ...Array.from({ length: count }, (_, k) => k).filter((k) => k !== shown)];
  const request = structuredClone(requestCase(store.caseData, "field"));
  let low = Infinity;
  let high = -Infinity;
  let notified = performance.now();
  for (const k of order) {
    let data;
    try {
      data = await api.field(request, { kind: "expression", text, t: result.times[k] });
    } catch (error) {
      if (!live()) return;
      comparison.error = error instanceof ApiError ? error : new ApiError({ stage: "field", message: String(error) });
      comparison.busy = false;
      notify("compare");
      return;
    }
    if (!live()) return;
    const exact = Float64Array.from(data.values, (v) => (v === null ? NaN : v));
    const errors = errorValues(result.values[k], exact);
    comparison.errors[k] = errors;
    comparison.linf[k] = maxAbs(errors);
    comparison.done += 1;
    const range = valueRange(errors);
    if (range.min < low || range.max > high) {
      low = Math.min(low, range.min);
      high = Math.max(high, range.max);
      comparison.scale = fieldScale([low, high]);
    }
    const now = performance.now();
    if (k === shown || now - notified > NOTIFY_MS) {
      notified = now;
      notify("compare");
    }
  }
  comparison.busy = false;
  notify("compare");
}

// Forgets the comparison (another solution, or the text was cleared).
export function clearComparison() {
  comparison.generation += 1;
  Object.assign(comparison, { result: null, errors: [], linf: new Float64Array(0), done: 0, busy: false, error: null, scale: null });
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
