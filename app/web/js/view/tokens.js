// The colour tokens of style.css, read once (getComputedStyle is not
// free; the page has one, dark, theme), and the colour tables made from
// them.

import { parseColor, rampTable } from "../pure/color.js";

const tokens = new Map();

export function token(name) {
  let value = tokens.get(name);
  if (value === undefined) {
    value = getComputedStyle(document.documentElement).getPropertyValue(name).trim();
    tokens.set(name, value);
  }
  return value;
}

// Each ramp goes through a middle stop, so its middle is a clear colour,
// not a grey mix of the ends.
let tables = null;
export function colorTables() {
  const ramp = (...names) => rampTable(names.map((name) => parseColor(token(name))));
  tables ??= {
    sequential: ramp("--seq-low", "--seq-mid", "--seq-high"),
    diverging: ramp("--div-low", "--div-mid", "--div-high"),
    quality: ramp("--quality-good", "--quality-mid", "--quality-bad"),
  };
  return tables;
}

// The table of a field scale (pure/color.js fieldScale or lockedScale).
export function tableOf(scale) {
  return scale.kind === "diverging" ? colorTables().diverging : colorTables().sequential;
}

// The CSS colour of table entry at position p in [0, 1].
export function tableColor(table, p) {
  const k = 3 * Math.max(0, Math.min(255, Math.round(255 * p)));
  return `rgb(${table[k]},${table[k + 1]},${table[k + 2]})`;
}
