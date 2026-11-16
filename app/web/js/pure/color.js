// Colours: OKLab ramps, the scale of a field, region colours. Pure.

export function parseColor(text) {
  const t = text.trim();
  if (t.startsWith("#")) {
    const hex = t.length === 4 ? [...t.slice(1)].map((c) => c + c).join("") : t.slice(1, 7);
    return [0, 2, 4].map((i) => parseInt(hex.slice(i, i + 2), 16));
  }
  const match = t.match(/rgba?\(([^)]+)\)/);
  if (match) return match[1].split(",").slice(0, 3).map((v) => Math.round(parseFloat(v)));
  return [128, 128, 128];
}

// sRGB <-> OKLab, so ramps interpolate evenly in lightness.
function toLinear(c) {
  const v = c / 255;
  return v <= 0.04045 ? v / 12.92 : ((v + 0.055) / 1.055) ** 2.4;
}
function fromLinear(v) {
  const c = v <= 0.0031308 ? 12.92 * v : 1.055 * v ** (1 / 2.4) - 0.055;
  return Math.max(0, Math.min(255, Math.round(c * 255)));
}
export function rgbToOklab([r, g, b]) {
  const [lr, lg, lb] = [toLinear(r), toLinear(g), toLinear(b)];
  const l = Math.cbrt(0.4122214708 * lr + 0.5363325363 * lg + 0.0514459929 * lb);
  const m = Math.cbrt(0.2119034982 * lr + 0.6806995451 * lg + 0.1073969566 * lb);
  const s = Math.cbrt(0.0883024619 * lr + 0.2817188376 * lg + 0.6299787005 * lb);
  return [
    0.2104542553 * l + 0.793617785 * m - 0.0040720468 * s,
    1.9779984951 * l - 2.428592205 * m + 0.4505937099 * s,
    0.0259040371 * l + 0.7827717662 * m - 0.808675766 * s,
  ];
}
export function oklabToRgb([L, a, b]) {
  const l = (L + 0.3963377774 * a + 0.2158037573 * b) ** 3;
  const m = (L - 0.1055613458 * a - 0.0638541728 * b) ** 3;
  const s = (L - 0.0894841775 * a - 1.291485548 * b) ** 3;
  return [
    fromLinear(4.0767416621 * l - 3.3077115913 * m + 0.2309699292 * s),
    fromLinear(-1.2684380046 * l + 2.6097574011 * m - 0.3413193965 * s),
    fromLinear(-0.0041960863 * l - 0.7034186147 * m + 1.707614701 * s),
  ];
}

// A colour table of `size` entries, through the given stops (rgb arrays),
// evenly spaced, interpolated in OKLab.
export function rampTable(stops, size = 256) {
  const labs = stops.map(rgbToOklab);
  const table = new Uint8ClampedArray(size * 3);
  for (let i = 0; i < size; i++) {
    const position = (i / (size - 1)) * (labs.length - 1);
    const k = Math.min(labs.length - 2, Math.floor(position));
    const f = position - k;
    const lab = labs[k].map((v, j) => v + (labs[k + 1][j] - v) * f);
    table.set(oklabToRgb(lab), i * 3);
  }
  return table;
}

// How a field's values map to colours: sequential (one hue, light to dark)
// for values of one sign, diverging (two hues, neutral at zero) when the
// values have both signs. Returns {kind, min, max, low, high,
// position(value) in [0, 1]}: low and high are the values at the ends of
// the colour bar (for a diverging scale -bound and bound, so zero is in
// the middle).
export function fieldScale(values) {
  let min = Infinity;
  let max = -Infinity;
  for (const v of values) {
    if (v === null || !Number.isFinite(v)) continue;
    if (v < min) min = v;
    if (v > max) max = v;
  }
  if (min === Infinity) return { kind: "empty", min: 0, max: 0, low: 0, high: 0, position: () => 0.5 };
  if (min < 0 && max > 0) {
    const bound = Math.max(-min, max);
    return { kind: "diverging", min, max, bound, low: -bound, high: bound, position: (v) => 0.5 + v / (2 * bound) };
  }
  if (max === min) return { kind: "constant", min, max, low: min, high: max, position: () => 0.5 };
  return { kind: "sequential", min, max, low: min, high: max, position: (v) => (v - min) / (max - min) };
}

// A colour range the user fixed ("Lock range"): exactly [low, high], so
// two runs coloured with it can be compared by eye. Diverging colours when
// it spans zero (zero is then in the middle only for a symmetric range),
// sequential otherwise. Null when the range is not one (low >= high, or
// not finite).
export function lockedScale(low, high) {
  if (!Number.isFinite(low) || !Number.isFinite(high) || !(high > low)) return null;
  const kind = low < 0 && high > 0 ? "diverging" : "sequential";
  return { kind, locked: true, min: low, max: high, low, high, bound: Math.max(-low, high), position: (v) => (v - low) / (high - low) };
}

export const SERIES_SLOTS = 8;

// The colour token of region i: the categorical slots in their fixed
// order; past the eighth region, one neutral "other" colour (identity is
// then carried by the legend and the tooltip).
export function regionToken(index) {
  return index < SERIES_SLOTS ? `--series-${index + 1}` : "--series-other";
}

// The colour position (0..1, scale.position) of every node's value, or
// NaN where it has none. Positions are linear in the value for every
// scale kind, so interpolating them is interpolating the values.
export function nodePositions(values, scale) {
  const out = new Float32Array(values.length);
  for (let i = 0; i < values.length; i++) {
    const v = values[i];
    out[i] = v === null || !Number.isFinite(v) ? NaN : scale.position(v);
  }
  return out;
}
