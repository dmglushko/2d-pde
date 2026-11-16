// Numbers as the page shows and reads them, and chart ticks. Pure: no
// DOM, tested in Node (app/tests/web.test.mjs).

// The significant digits numbers are shown with (a setting).
let defaultDigits = 4;
export function setDefaultDigits(digits) {
  if (Number.isInteger(digits) && digits >= 2 && digits <= 10) defaultDigits = digits;
}
export function getDefaultDigits() {
  return defaultDigits;
}

// A short, readable form of a number for labels and tables.
export function formatNumber(value, digits = defaultDigits) {
  if (value === null || value === undefined) return "–";
  if (typeof value !== "number") return String(value);
  if (!Number.isFinite(value)) return String(value);
  if (value === 0) return "0";
  const magnitude = Math.abs(value);
  if (Number.isInteger(value) && magnitude < 1e7) return value.toLocaleString("en-US");
  if (magnitude >= 1e5 || magnitude < 1e-3) return value.toExponential(digits - 1);
  return String(Number(value.toPrecision(digits)));
}

// --- charts -----------------------------------------------------------------------

// About `count` round tick values covering [min, max] (1, 2 or 5 times a
// power of ten apart, the one nearest to an even split), from the first multiple at or below min to the
// last at or above max.
export function niceTicks(min, max, count = 4) {
  if (!Number.isFinite(min) || !Number.isFinite(max)) return [];
  if (max === min) {
    const pad = Math.abs(min) > 0 ? Math.abs(min) * 0.1 : 1;
    return niceTicks(min - pad, max + pad, count);
  }
  const raw = (max - min) / Math.max(1, count);
  const power = 10 ** Math.floor(Math.log10(raw));
  const ratio = raw / power;
  const step = (ratio < 1.5 ? 1 : ratio < 3 ? 2 : ratio < 7 ? 5 : 10) * power;
  const first = Math.floor(min / step + 1e-9) * step;
  const ticks = [];
  for (let v = first; v <= max + step * (1 - 1e-9); v += step) ticks.push(Math.abs(v) < step * 1e-9 ? 0 : Number(v.toPrecision(12)));
  return ticks;
}

// The file name of a saved picture: the case's name and, for a solution
// in time, the shown time ("heat-t0.25.png").
export function pictureName(caseName, t = null) {
  const stem = String(caseName ?? "case").replace(/\.json$/i, "").replace(/[^\w.-]+/g, "_") || "case";
  if (t === null || !Number.isFinite(t)) return `${stem}.png`;
  return `${stem}-t${String(Number(t.toPrecision(6))).replace(/[^\w.-]+/g, "_")}.png`;
}

// A number field's text as the case gets it, whatever the browser's
// locale: digits with a decimal point and an optional exponent ("0.05",
// "-1e-3", ".5"). {kind: "empty"} (no value), {kind: "number", value} or
// {kind: "invalid", reason} for text that is not (yet) a number, such as
// "-", "1e" or "0,05" (a decimal comma is named, since a locale may have
// taught it).
const DECIMAL = /^[+-]?(\d+\.?\d*|\.\d+)(e[+-]?\d+)?$/i;
export function parseNumberText(text) {
  const trimmed = String(text ?? "").trim();
  if (trimmed === "") return { kind: "empty" };
  if (DECIMAL.test(trimmed)) {
    const value = Number(trimmed);
    return Number.isFinite(value) ? { kind: "number", value } : { kind: "invalid", reason: "too large for a number" };
  }
  if (/^[+-]?\d*,\d+(e[+-]?\d+)?$/i.test(trimmed)) return { kind: "invalid", reason: "use a decimal point (0.05), not a comma" };
  return { kind: "invalid", reason: "not a number" };
}

// The text a number field shows for a value: the shortest form that reads
// back as the same double, with a decimal point in every locale ("" for no
// value).
export function numberText(value) {
  return value === null || value === undefined ? "" : String(value);
}
