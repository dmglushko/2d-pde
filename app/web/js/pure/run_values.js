// A run's values in the page: which snapshots to keep, binary snapshots,
// ranges. Pure.

// How many snapshots of a run the page keeps at most: about 64 MB of
// float32 values, from 8 to 1000 snapshots.
export function keptSnapshots(nodes) {
  if (!Number.isFinite(nodes) || nodes <= 0) return 1000;
  return Math.max(8, Math.min(1000, Math.floor((64 * 1024 * 1024) / (4 * nodes))));
}

// What to fetch and what to drop so that snapshot `shown` and its
// neighbours are loaded: `values` has one entry per ready snapshot (null
// where not loaded). Fetches the missing ones in [shown − behind, shown +
// ahead] as one range {first, count} (or null), and drops the loaded ones
// farthest from `shown` beyond `kept`.
export function snapshotPlan(values, shown, { ahead = 8, behind = 2, kept = 1000 } = {}) {
  const ready = values.length;
  if (ready === 0) return { fetch: null, evict: [] };
  const k = Math.max(0, Math.min(ready - 1, shown));
  const low = Math.max(0, k - behind);
  const high = Math.min(ready - 1, k + ahead);
  let first = -1;
  let last = -1;
  for (let i = low; i <= high; i++)
    if (!values[i]) {
      if (first < 0) first = i;
      last = i;
    }
  const fetch = first < 0 ? null : { first, count: last - first + 1 };
  const loaded = [];
  for (let i = 0; i < ready; i++) if (values[i] || (fetch && i >= fetch.first && i < fetch.first + fetch.count)) loaded.push(i);
  const evict = [];
  if (loaded.length > kept) {
    loaded.sort((a, b) => Math.abs(b - k) - Math.abs(a - k) || b - a);
    for (const i of loaded.slice(0, loaded.length - kept)) if (values[i]) evict.push(i);
  }
  return { fetch, evict };
}

// The loaded snapshot nearest to k (k itself when loaded), or -1.
export function nearestLoaded(values, k) {
  for (let d = 0; d < values.length; d++) {
    if (values[k - d]) return k - d;
    if (values[k + d]) return k + d;
  }
  return -1;
}

// Splits float32 little-endian values (snapshots x nodes, in order) into
// one Float32Array per snapshot. Throws when the size does not fit.
export function splitSnapshots(buffer, snapshots, nodes) {
  if (buffer.byteLength !== 4 * snapshots * nodes) throw new Error(`the solution has ${buffer.byteLength} bytes, expected ${4 * snapshots * nodes}`);
  const view = new DataView(buffer);
  const out = [];
  for (let k = 0; k < snapshots; k++) {
    const values = new Float32Array(nodes);
    for (let i = 0; i < nodes; i++) values[i] = view.getFloat32(4 * (k * nodes + i), true);
    out.push(values);
  }
  return out;
}

// The smallest and largest finite value of one snapshot ({min, max},
// NaN when it has none).
export function valueRange(values) {
  let min = Infinity;
  let max = -Infinity;
  for (const v of values) {
    if (v === null || !Number.isFinite(v)) continue;
    if (v < min) min = v;
    if (v > max) max = v;
  }
  return min === Infinity ? { min: NaN, max: NaN } : { min, max };
}
