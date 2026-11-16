// The one answer to "what does the picture show now": the store's state
// gathered into pure/display.js's resolveDisplay. The picture, the
// legend, the Colour menu, the comparison controls and the probes read it
// (main.js, panels/solve.js, probes.js), so none of them can show a
// comparison or a snapshot the others do not.

import { comparisonState, comparisons, currentComparison, differenceState } from "./compare.js";
import { formatNumber } from "./pure/format.js";
import { resolveDisplay } from "./pure/display.js";
import { snapshotState } from "./snapshots.js";
import { freshResult, lastResult, store } from "./state.js";

// The solve result drawn: the current one, or the last one (outdated,
// drawn greyed) while it is on the mesh the picture has.
export function drawnSolve() {
  const fresh = freshResult("solve");
  const data = fresh ?? lastResult("solve");
  if (!data || !(data.values?.length > 0)) return null;
  const mesh = lastResult("mesh")?.mesh;
  if (!fresh && (!mesh || mesh.id !== data.mesh_id)) return null;
  return { data, fresh: Boolean(fresh) };
}

function drawnField() {
  const result = store.results.field;
  if (!result) return null;
  const mesh = lastResult("mesh")?.mesh;
  const fresh = freshResult("field");
  // An outdated field is drawn only on a mesh with its number of nodes.
  if (!fresh && (!mesh || result.data.values.length * 2 !== mesh.nodes.length)) return null;
  return { data: result.data, fresh: Boolean(fresh) };
}

export function currentDisplay() {
  const drawn = drawnSolve();
  const kind = store.view.difference;
  let solve = null;
  let comparison = null;
  if (drawn && store.fieldSource === "solve") {
    const frame = Math.max(0, Math.min(store.solveFrame, drawn.data.values.length - 1));
    const state = kind && drawn.fresh ? comparisonState(kind) : "none";
    if (state === "current") {
      const c = currentComparison(kind);
      comparison = { kind, state, scale: c.scale, values: differenceState(kind, frame) };
    } else if (kind) comparison = { kind, state };
    // The snapshot is fetched only when it is what is drawn.
    const snapshot = state === "current" ? { values: null, nearest: null, failure: null } : snapshotState(drawn.data, frame);
    solve = { data: drawn.data, fresh: drawn.fresh, frame, snapshot };
  }
  return resolveDisplay({
    colorBy: store.view.colorBy,
    source: store.fieldSource,
    difference: kind,
    solve,
    comparison,
    field: store.fieldSource === "field" || !solve ? drawnField() : null,
    format: (t) => formatNumber(t, 5),
  });
}

// The comparison of `kind` whose controls the Solve step shows, with the
// display's verdict: {state, shown} (shown: u − other is what the picture
// draws).
export function comparisonControls(kind, display = currentDisplay()) {
  return { state: comparisonState(kind), shown: display.difference.requested === kind && display.difference.shown, comparison: comparisons[kind] };
}
