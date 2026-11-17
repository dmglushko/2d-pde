// What the picture shows, resolved in one place from the state: the
// quantity (u, u − exact, u − reference, a field of the Assemble step),
// its source and snapshot, the comparison it belongs to and whether that
// is current, whether the result is outdated, loading or failed, what is
// float32 and the colour scale. The picture, the legend, the Colour menu,
// the comparison controls and the probes all read this answer, so they
// cannot disagree (display.js gathers the input from the store). Pure.

export const DIFFERENCE_NAMES = { exact: "u − exact", reference: "u − reference" };

const PRECISION = {
  solve: "Drawn from float32 copies of u (the server keeps u in double).",
  difference: "The difference is computed in double on the server, then sent and drawn as float32.",
  field: "Values from the server in double; drawn through a 256-step colour map.",
};

// Why a comparison is not drawn, in words for the legend.
const NOT_SHOWN = {
  none: (name) => `${name} is not computed for this solution: press Compare.`,
  busy: (name) => `${name}: comparing…`,
  failed: (name) => `${name} could not be computed (see the Solve step).`,
  outdated: (name) => `${name} is for the previous expression: press Compare to update it.`,
  stale: (name) => `${name} is not shown on an outdated solution.`,
};

// The time label of a snapshot.
const at = (t, format) => (Number.isFinite(t) ? `at t = ${format(t)}` : "");

// input:
//   colorBy: "none" | "quality" | "field" | "gradient" | "indicator"
//   source: "solve" | "field" (store.fieldSource)
//   difference: null | "exact" | "reference" (what the user asked for)
//   solve: null | {data, fresh, frame, snapshot: {values, nearest:
//     {index, values} | null, failure}} (failure: an error or null)
//   comparison: null | {kind, state: "none" | "busy" | "failed" |
//     "outdated" | "current", scale, values: {values, nearest: {index,
//     values} | null, failure}}
//   field: null | {data, fresh}
//   format(t): the time as text
// Returns {kind ("none", "solution", "difference", "field"), source, name,
// when, label, values, scale, frame, t, outdated, loading (text or null),
// failure ({error, retry: "snapshot" | "difference"} or null), difference
// ({requested, state, shown, note}), precision, indicators (an adaptive
// solution's error indicators, one per triangle of its mesh, or null)}.
export function resolveDisplay({ colorBy, source, difference = null, solve = null, comparison = null, field = null, format = String }) {
  const none = {
    kind: "none",
    source: null,
    name: null,
    when: "",
    label: "",
    values: null,
    scale: null,
    frame: 0,
    t: null,
    outdated: false,
    loading: null,
    failure: null,
    difference: { requested: difference, state: "none", shown: false, note: null },
    precision: null,
    indicators: null,
  };
  const colouring = colorBy === "field" || colorBy === "gradient" || colorBy === "indicator";
  if (source === "solve" && solve?.data) return solution({ ...none, colouring }, solve, difference, comparison, format);
  if (field?.data) {
    const data = field.data;
    const timeless = data.kind === "initial_u" || data.kind === "initial_u_t";
    const when = timeless ? "" : at(data.t, format);
    return {
      ...none,
      kind: "field",
      source: "field",
      name: data.label,
      when,
      label: [data.label, when].filter(Boolean).join(" "),
      values: data.values,
      outdated: !field.fresh,
      precision: PRECISION.field,
      data,
    };
  }
  return none;
}

function solution(base, solve, difference, comparison, format) {
  const data = solve.data;
  const last = Math.max(0, data.times.length - 1);
  const frame = Math.max(0, Math.min(solve.frame, last));
  const t = data.times[frame];
  const outdated = !solve.fresh;
  const out = { ...base, kind: "solution", source: "solve", frame, t, outdated, scale: data.scale ?? null };
  delete out.colouring;
  // The comparison the user asked for, drawn only while it is current for
  // this (fresh) solution.
  let state = difference ? (comparison?.state ?? "none") : "none";
  if (difference && outdated) state = "stale";
  const name = difference ? DIFFERENCE_NAMES[difference] : null;
  const shown = Boolean(difference) && state === "current";
  out.difference = { requested: difference, state, shown, note: difference && !shown ? NOT_SHOWN[state](name) : null };
  if (shown) {
    const values = comparison.values ?? {};
    const pick = pickValues(values, frame, data.times, format);
    return {
      ...out,
      kind: "difference",
      name,
      when: pick.when ?? at(t, format),
      label: `${name} ${pick.when ?? at(t, format)}`,
      values: pick.values,
      scale: comparison.scale,
      loading: pick.loading ? `Loading ${name} at t = ${format(t)}…` : null,
      failure: values.failure ? { error: values.failure, retry: "difference" } : null,
      precision: PRECISION.difference,
    };
  }
  const pick = pickValues(solve.snapshot ?? {}, frame, data.times, format);
  const when = pick.when ?? at(t, format);
  return {
    ...out,
    name: "u",
    when,
    label: `u ${when}`,
    values: pick.values,
    loading: pick.loading ? `Loading u at t = ${format(t)}…` : null,
    failure: solve.snapshot?.failure ? { error: solve.snapshot.failure, retry: "snapshot" } : null,
    precision: PRECISION.solve,
    indicators: data.indicators ?? null,
  };
}

// The values of the shown snapshot, or the nearest loaded one standing in
// while it loads (its own time in the label), or none.
function pickValues({ values = null, nearest = null, failure = null }, frame, times, format) {
  if (values) return { values, when: null, loading: false };
  if (nearest?.values) {
    const loading = failure ? "" : ` (loading t = ${format(times[frame])})`;
    return { values: nearest.values, when: `at t = ${format(times[nearest.index])}${loading}`, loading: !failure };
  }
  return { values: null, when: null, loading: !failure };
}

// The parts of a resolved display that change what is drawn: equal keys
// draw the same picture.
export function displayKey(display) {
  return [display.kind, display.label, display.outdated, display.loading, display.failure?.retry ?? null].join("|");
}
