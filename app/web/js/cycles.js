// An adaptive run's earlier cycles in the picture. The run's result shows
// its last cycle; choosing another (the Adaptive cycles table or chart)
// shows that cycle's solution on that cycle's mesh, fetched once per
// result (the mesh from /api/mesh/arrays, the values from
// /api/solve/cycle). display.js draws it in place of the result: a view
// of the result with that cycle's mesh and values, the run's colour scale
// (so cycles compare at a glance), and no indicators or comparisons (they
// are the last cycle's).

import { toast } from "./actions.js";
import { api } from "./api.js";
import { lastResult, notify, requestTicket, store } from "./state.js";

// Per result: cycle -> {view, loading}.
const kept = new WeakMap();

// The number of cycles a result can show, 0 for anything else.
export function cycleCount(data) {
  return data?.mode === "adaptive" && data.adaptive?.cycle_meshes ? data.adaptive.cycle_meshes.length : 0;
}

// The cycle chosen for `data` (from 1), or null: its last.
export function chosenCycle(data) {
  const chosen = store.shownCycle;
  return chosen && chosen.data === data ? chosen.cycle : null;
}

// Shows cycle `cycle` of `data` (null or the last: the result itself).
export function chooseCycle(data, cycle) {
  const count = cycleCount(data);
  store.shownCycle = cycle !== null && cycle >= 1 && cycle < count ? { data, cycle } : null;
  if (store.shownCycle) load(data, cycle);
  notify("frame");
}

// What the picture draws for `data`: the chosen cycle's view once it is
// loaded, else null (the result itself).
export function cycleView(data) {
  const cycle = chosenCycle(data);
  return cycle === null ? null : (kept.get(data)?.get(cycle)?.view ?? null);
}

// Whether the chosen cycle is still being fetched.
export function cycleLoading(data) {
  const cycle = chosenCycle(data);
  return cycle !== null && Boolean(kept.get(data)?.get(cycle)?.loading);
}

async function load(data, cycle) {
  let cycles = kept.get(data);
  if (!cycles) kept.set(data, (cycles = new Map()));
  if (cycles.has(cycle)) return;
  const item = { view: null, loading: true };
  cycles.set(cycle, item);
  const ticket = requestTicket("cycle", () => data, { exclusive: false });
  try {
    const summary = data.adaptive.cycle_meshes[cycle - 1];
    const known = [lastResult("mesh")?.mesh, data.mesh].find((m) => m?.id === summary.id);
    const mesh = known ?? (await api.meshArrays(summary));
    const values = await api.cycleValues(data.job, cycle, summary.nodes);
    item.view = {
      ...data,
      mesh,
      values: [values],
      indicators: null,
      cycle,
      when: `in cycle ${cycle} of ${cycleCount(data)}`,
    };
  } catch (error) {
    cycles.delete(cycle);
    if (ticket.live() && chosenCycle(data) === cycle) {
      store.shownCycle = null;
      toast(`Cycle ${cycle} could not be shown: ${error.message}`);
    }
  } finally {
    item.loading = false;
  }
  notify("frame");
}
