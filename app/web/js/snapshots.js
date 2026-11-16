// The snapshots of a run the page keeps: not all of them. A solve result's
// `values` has one entry per ready snapshot, a Float32Array where loaded
// and null where not. The shown snapshot and its neighbours are fetched on
// demand (by range, POST /api/solve/values), and the ones farthest from
// the shown one are dropped beyond keptSnapshots (core.js). Exact numbers
// (comparisons, probes) are the server's, so nothing here needs every
// snapshot.

import { api } from "./api.js";
import { keptSnapshots, snapshotPlan } from "./core.js";
import { notify } from "./state.js";

// The fetch going on per result, so one runs at a time.
const loading = new WeakMap();

// Makes snapshot `k` of `data` (a solve result with `job`, `layout` and
// `values`) and its neighbours loaded. Resolves when they are; notifies
// "frame" when it fetched something (the picture shows it then).
export async function loadAround(data, k) {
  for (;;) {
    const running = loading.get(data);
    if (!running) break;
    await running;
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
  try {
    await task;
  } finally {
    loading.delete(data);
  }
  notify("frame");
}

// Snapshot `k` if it is loaded; otherwise starts loading it and returns
// null (the picture is drawn again once it arrives).
export function snapshotValues(data, k) {
  const values = data.values[k];
  if (values) return values;
  loadAround(data, k).catch(() => {});
  return null;
}

// Gives the result room for its ready snapshots (null until loaded).
export function growSnapshots(data, ready) {
  while (data.values.length < ready) data.values.push(null);
}
