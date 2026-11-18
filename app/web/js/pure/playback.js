// Playing a run in time: a clock in simulated time that real time moves,
// at the playback speed (1 = one simulated second per second, whatever
// the number of steps or snapshots). While a run is computed, it starts
// once a few snapshots are there and then goes no faster than the run
// (a little slower, so it seldom has to wait). Pure.

// Snapshots a running run must have before it plays.
export const START_SNAPSHOTS = 4;
// The share of the run's measured pace the playback keeps to.
export const PACE_SHARE = 0.9;
// The measured pace looks back this far (ms) and needs at least
// MIN_WINDOW_MS of it.
const WINDOW_MS = 4000;
const MIN_WINDOW_MS = 400;

// The playback speeds offered (simulated time per second of playback).
export const PLAYBACK_SPEEDS = [0.001, 0.01, 0.1, 0.25, 0.5, 1, 2, 5, 10, 100, 1000];

// "1×", "0.25×", "1000×".
export function speedLabel(speed) {
  return `${Number(speed.toPrecision(3))}×`;
}

// The snapshot shown at simulated time t: the last one at or before t
// (a run's times grow).
export function frameAt(times, t) {
  if (times.length === 0) return 0;
  const span = Math.abs(times[times.length - 1] - times[0]);
  const eps = 1e-9 * span;
  let low = 0;
  let high = times.length - 1;
  if (t + eps < times[0]) return 0;
  while (low < high) {
    const mid = (low + high + 1) >> 1;
    if (times[mid] <= t + eps) low = mid;
    else high = mid - 1;
  }
  return low;
}

// Adds a sample {at (ms), t (simulated)} of a running run's progress to
// `samples` (kept to the window), unless t did not move.
export function addPaceSample(samples, at, t) {
  const last = samples[samples.length - 1];
  if (last && last.t === t) return samples;
  samples.push({ at, t });
  while (samples.length > 2 && at - samples[1].at > WINDOW_MS) samples.shift();
  return samples;
}

// The run's pace: simulated time computed per second, over the samples'
// window, or null while too little is known.
export function measuredPace(samples) {
  if (samples.length < 2) return null;
  const first = samples[0];
  const last = samples[samples.length - 1];
  const ms = last.at - first.at;
  if (ms < MIN_WINDOW_MS) return null;
  return Math.max(0, (last.t - first.t) / (ms / 1000));
}

// One step of the clock: `t` the simulated time shown, `seconds` the real
// time since the last step, `speed` the playback speed, `times` the
// snapshot times that are ready, `upTo` how many of them are loaded from
// the shown one on (the clock does not run past them), `running` whether
// the run is still computed, `pace` its measured pace (or null).
// Returns {t, frame, waiting, finished, speed}: the new time, the
// snapshot to show, why it waits ("starting": too few snapshots yet,
// "computing": it caught up with the run, "loading": the next snapshot
// is not loaded yet), whether it reached the end of a finished run, and
// the speed it played at.
export function playbackStep({ t, seconds, speed, times, upTo = Infinity, running = false, pace = null }) {
  const ready = times.length;
  const shown = frameAt(times, t);
  if (ready === 0) return { t, frame: 0, waiting: "starting", finished: false, speed: 0 };
  if (running && ready < START_SNAPSHOTS && pace === null) return { t, frame: shown, waiting: "starting", finished: false, speed: 0 };
  const played = running && pace !== null ? Math.min(speed, PACE_SHARE * pace) : speed;
  const lastLoaded = Math.min(ready - 1, shown + Math.max(0, upTo));
  const limit = times[lastLoaded];
  let next = t + Math.max(0, seconds) * played;
  let waiting = null;
  if (next >= limit) {
    next = limit;
    if (lastLoaded < ready - 1) waiting = "loading";
    else if (running) waiting = "computing";
  }
  if (running && played === 0 && !waiting) waiting = "computing";
  const finished = !running && next >= times[ready - 1];
  return { t: next, frame: frameAt(times, next), waiting, finished, speed: waiting ? 0 : played };
}
