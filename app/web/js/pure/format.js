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

// The ticks of a colour bar from `low` to `high`: round values inside the
// range (niceTicks), each with its position in [0, 1] along the bar and
// its text. Zero is always a tick of a range that spans it (the middle of
// a diverging scale). The texts are as short as keeps them apart.
export function colorbarTicks(low, high, count = 4) {
  if (!Number.isFinite(low) || !Number.isFinite(high) || !(high > low)) return [];
  const span = high - low;
  const inside = niceTicks(low, high, count).filter((v) => v >= low - span * 1e-9 && v <= high + span * 1e-9);
  if (low < 0 && high > 0 && !inside.includes(0)) inside.push(0);
  inside.sort((a, b) => a - b);
  let digits = 2;
  const texts = () => inside.map((v) => formatNumber(v, digits));
  while (digits < 8 && new Set(texts()).size < inside.length) digits++;
  const shown = texts();
  return inside.map((value, i) => ({ value, position: Math.max(0, Math.min(1, (value - low) / span)), text: shown[i] }));
}

// Ticks of a logarithmic axis over [min, max] (both > 0): the powers of
// ten inside it, and when there are fewer than two, 1, 2 and 5 times them.
export function logTicks(min, max) {
  if (!(min > 0) || !(max > 0) || !Number.isFinite(min) || !Number.isFinite(max)) return [];
  const [lo, hi] = min <= max ? [min, max] : [max, min];
  const first = Math.floor(Math.log10(lo));
  const last = Math.ceil(Math.log10(hi));
  const within = (v) => v >= lo * (1 - 1e-9) && v <= hi * (1 + 1e-9);
  const powers = [];
  for (let e = first; e <= last; e++) powers.push(Number((10 ** e).toPrecision(12)));
  const decades = powers.filter(within);
  if (decades.length >= 2) return decades;
  const fine = [];
  for (const p of powers) for (const m of [1, 2, 5]) fine.push(Number((m * p).toPrecision(12)));
  const shown = fine.filter(within);
  return shown.length >= 2 ? shown : [lo, hi];
}

// A CSV text from a header and rows: numbers in full precision (the
// shortest text that reads back as the same double), an empty cell for a
// missing or non-finite value, text cells quoted when they need it.
export function csvText(header, rows) {
  const cell = (v) => {
    if (typeof v === "number") return Number.isFinite(v) ? String(v) : "";
    if (v === null || v === undefined) return "";
    const text = String(v);
    return /[",\n]/.test(text) ? `"${text.replaceAll('"', '""')}"` : text;
  };
  return [header, ...rows].map((row) => row.map(cell).join(",")).join("\n") + "\n";
}

// The CSV of a chart: its x values (named `xName`) and one column per
// series.
export function seriesCsv(xName, xs, series) {
  const rows = Array.from(xs, (x, k) => [x, ...series.map((s) => s.values[k])]);
  return csvText([xName, ...series.map((s) => s.name)], rows);
}

// A size in bytes for people: "512 B", "3.4 MB", "1 GB" (powers of 1024,
// as the server's --memory-budget).
export function bytesText(bytes) {
  if (!Number.isFinite(bytes) || bytes < 0) return "–";
  const units = ["B", "KB", "MB", "GB", "TB"];
  let value = bytes;
  let unit = 0;
  while (value >= 1024 && unit < units.length - 1) {
    value /= 1024;
    unit++;
  }
  const text = unit === 0 ? String(Math.round(value)) : String(Number(value.toPrecision(value >= 100 ? 3 : 2)));
  return `${text} ${units[unit]}`;
}

// The longest of `texts` (the first of equal lengths), or "": what a
// label that changes on every frame (a time, a range) is sized for, so
// that what follows it does not move.
export function widestText(texts) {
  let widest = "";
  for (const text of texts) if (text.length > widest.length) widest = text;
  return widest;
}
