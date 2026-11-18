// The snapshots of a run the page keeps: not all of them. A solve result's
// `values` has one entry per ready snapshot, a Float32Array where loaded
// and null where not. The shown snapshot and its neighbours are fetched on
// demand (by range, POST /api/solve/values), and the ones farthest from
// the shown one are dropped beyond keptSnapshots (pure/run_values.js). Exact numbers
// (comparisons, probes) are the server's, so nothing here needs every
// snapshot.

import { api } from "./api.js";
import { keptSnapshots, nearestLoaded, snapshotPlan } from "./pure/run_values.js";
import { notify } from "./state.js";

// The fetch going on per result, so one runs at a time.
const loading = new WeakMap();
// The last failed fetch per result (an ApiError): no new fetch starts
// until retrySnapshots, so a failing server is not asked on every redraw.
const failures = new WeakMap();

// Makes snapshot `k` of `data` (a solve result with `job`, `layout` and
// `values`) and its neighbours loaded. Resolves true when they are, false
// when the fetch failed (the failure is kept: snapshotState); notifies
// "frame" when it fetched or failed (the picture shows it then).
export async function loadAround(data, k) {
  if (failures.has(data)) return false;
  for (;;) {
    const running = loading.get(data);
    if (!running) break;
    // Its failure is kept (failures) and said by the one that started it.
    await running.catch(() => {});
  }
  const plan = snapshotPlan(data.values, k, { kept: keptSnapshots(data.layout?.nodes) });
  for (const i of plan.evict) data.values[i] = null;
  if (!plan.fetch) return;
  const task = (async () => {
    const { first, count } = plan.fetch;
    const values = await api.solveRange(data.job, first, count, data.layout);
    values.forEach((v, i) => {
      if (first + i < data.values.length) data.values[first + i] = v;
    });
  })();
  loading.set(data, task);
  let ok = true;
  try {
    await task;
  } catch (error) {
    failures.set(data, error);
    ok = false;
  } finally {
    loading.delete(data);
  }
  notify("frame");
  return ok;
}

// Fetches the snapshots a playback at `k` will show next, unless a fetch
// runs or failed (no waiting: the playback holds until they are there).
export function prefetchSnapshots(data, k) {
  if (loading.has(data) || failures.has(data)) return;
  // The window around k (snapshotPlan: 2 behind, 8 ahead) starts at the
  // first missing one, so the next snapshot is always fetched.
  if (!snapshotPlan(data.values, k, { kept: keptSnapshots(data.layout?.nodes) }).fetch) return;
  loadAround(data, k);
}

// Snapshot `k` if it is loaded; otherwise starts loading it and returns
// null (the picture is drawn again once it arrives).
export function snapshotValues(data, k) {
  const values = data.values[k];
  if (values) return values;
  loadAround(data, k);
  return null;
}

// What the picture can show of snapshot `k` (pure/display.js): its
// values, else the nearest loaded one, and the failure of the last fetch.
export function snapshotState(data, k) {
  const values = snapshotValues(data, k);
  const near = values ? -1 : nearestLoaded(data.values, k);
  return { values, nearest: near >= 0 ? { index: near, values: data.values[near] } : null, failure: failures.get(data) ?? null };
}

// Forgets the failure of `data`'s last fetch, so the next one is tried;
// `quiet`: without notifying (the caller fetches at once).
export function retrySnapshots(data, { quiet = false } = {}) {
  if (!failures.delete(data) || quiet) return;
  notify("frame");
}

// Gives the result room for its ready snapshots (null until loaded).
export function growSnapshots(data, ready) {
  while (data.values.length < ready) data.values.push(null);
}
