// The time bar under the picture: play or pause, the playback speed, a
// slider over the snapshots and the shown time with that snapshot's
// smallest and largest value of u (the server's, in double: snapshot_min
// and snapshot_max of the progress). It is the one control of the shown
// snapshot (store.solveFrame). During a run it shows how far the run got.
//
// Playing moves a clock in simulated time by real time, at the playback
// speed (1×: one simulated second per second, however many steps or
// snapshots it holds; pure/playback.js). A run being computed plays from
// its start once a few snapshots are there (store.playRun, set by
// actions.js), no faster than the run computes.

import { followedRun } from "./actions.js";
import { drawnSolve } from "./display.js";
import { formatNumber } from "./pure/format.js";
import { isTimeRun } from "./pure/stages.js";
import { addPaceSample, frameAt, measuredPace, PLAYBACK_SPEEDS, playbackStep, speedLabel } from "./pure/playback.js";
import { h } from "./dom.js";
import { prefetchSnapshots } from "./snapshots.js";
import { setSetting, settings } from "./settings.js";
import { notify, store } from "./state.js";

let elements = null;
let shownResult = null;
// The playback going on: {result, t (simulated), last (ms), frame,
// waiting, speed, timer}, or null.
let playing = null;
// The pace samples of each running result (pure/playback.js).
const paces = new WeakMap();
// How many snapshots ahead of the shown one must be loaded to play on.
const LOADED_AHEAD = 1;

// The time solution the bar walks through (the one the picture draws,
// outdated or not), or null.
export function timeSolution() {
  const drawn = drawnSolve();
  const result = drawn?.data;
  return result && isTimeRun(result) && result.values.length > 0 && store.fieldSource === "solve" ? result : null;
}

export function stopPlaying() {
  if (playing) clearTimeout(playing.timer);
  playing = null;
  if (elements) syncPlay();
}

// Shows snapshot `index` of the solution. `chosen`: the viewer chose it
// (the slider), which ends playing a run as it comes.
export function showFrame(index, { chosen = true } = {}) {
  store.solveFrame = index;
  if (chosen) {
    store.solveFrameChosen = true;
    store.playRun = false;
  }
  store.fieldSource = "solve";
  if (!["field", "gradient", "indicator"].includes(store.view.colorBy)) store.view.colorBy = "field";
  notify("frame");
}

function syncPlay() {
  const on = playing !== null;
  elements.play.textContent = on ? "❚❚" : "▶";
  elements.play.setAttribute("aria-label", on ? "Pause" : "Play");
  elements.play.title = on ? "Pause" : `Play at ${speedLabel(settings.playbackSpeed)} (simulated time per second)`;
}

// How many snapshots from `frame` on are loaded (the clock does not run
// past them; they are fetched ahead).
function loadedFrom(result, frame) {
  let n = 0;
  while (frame + n + 1 < result.values.length && result.values[frame + n + 1]) n += 1;
  return n;
}

// A tick of the clock about 60 times a second (a timer, not animation
// frames: those may not run in a frame that is not painted).
const TICK_MS = 16;
const tickLater = () => (playing.timer = setTimeout(() => step(performance.now()), TICK_MS));

function step(now) {
  if (!playing) return;
  const result = timeSolution();
  // Another result or field took the picture.
  if (result !== playing.result) return stopPlaying();
  const seconds = (now - playing.last) / 1000;
  playing.last = now;
  const running = result.state === "running" && Boolean(followedRun());
  const next = playbackStep({
    t: playing.t,
    // A hidden tab or a long pause does not jump ahead.
    seconds: Math.min(seconds, 0.25),
    speed: settings.playbackSpeed,
    times: result.times,
    upTo: loadedFrom(result, frameAt(result.times, playing.t)) + LOADED_AHEAD,
    running,
    pace: running ? measuredPace(paces.get(result) ?? []) : null,
  });
  playing.t = next.t;
  prefetchSnapshots(result, next.frame);
  const changed = next.waiting !== playing.waiting || next.speed !== playing.speed;
  playing.waiting = next.waiting;
  playing.speed = next.speed;
  if (next.frame !== store.solveFrame) showFrame(next.frame, { chosen: false });
  else if (changed) updateTimebar();
  if (next.finished) {
    stopPlaying();
    updateTimebar();
    return;
  }
  tickLater();
}

// Plays from the shown snapshot (from the start when at the end), or
// pauses.
function play({ fromStart = false } = {}) {
  if (playing) return stopPlaying();
  const result = timeSolution();
  if (!result) return;
  const last = result.values.length - 1;
  const atEnd = store.solveFrame >= last && result.state !== "running";
  const frame = fromStart || atEnd ? 0 : Math.min(store.solveFrame, last);
  playing = { result, t: result.times[frame], last: performance.now(), frame, waiting: null, speed: 0, timer: 0 };
  showFrame(frame, { chosen: false });
  tickLater();
  syncPlay();
}

function speedSelect() {
  const select = h(
    "select",
    {
      class: "speed",
      "aria-label": "Playback speed",
      title: "Playback speed: simulated time per second (1× is real time)",
      onchange: (event) => setSetting("playbackSpeed", Number(event.target.value)),
    },
    PLAYBACK_SPEEDS.map((speed) => h("option", { value: String(speed) }, speedLabel(speed))),
  );
  return select;
}

function syncSpeed() {
  const select = elements.speed;
  const speed = settings.playbackSpeed;
  // A stored speed that is not offered is shown as its own option.
  if (![...select.options].some((o) => Number(o.value) === speed)) select.append(h("option", { value: String(speed) }, speedLabel(speed)));
  select.value = [...select.options].find((o) => Number(o.value) === speed).value;
  syncPlay();
}

export function wireTimebar(root) {
  elements = {
    root,
    play: h("button", { type: "button", class: "icon play", id: "time-play", "aria-label": "Play", title: "Play" }, "▶"),
    speed: speedSelect(),
    slider: h("input", { type: "range", id: "time-slider", min: "0", max: "0", step: "1", value: "0", "aria-label": "Snapshot" }),
    time: h("span", { class: "num time-now" }),
    range: h("span", { class: "num time-range" }),
    run: h("div", { class: "time-run", hidden: true }),
  };
  elements.bar = h("progress", { max: "1", value: "0", "aria-label": "Run progress" });
  elements.runText = h("span", { class: "num" });
  elements.playText = h("span", { class: "play-note" });
  elements.run.append(elements.bar, elements.runText, elements.playText);
  elements.play.addEventListener("click", () => {
    // Pausing a run that plays as it comes leaves it where it is.
    store.playRun = false;
    play();
  });
  elements.slider.addEventListener("input", () => {
    stopPlaying();
    showFrame(Number(elements.slider.value));
  });
  root.append(
    elements.play,
    elements.speed,
    elements.slider,
    h("div", { class: "time-label" }, elements.time, elements.range),
    elements.run,
  );
  syncSpeed();
  updateTimebar();
}

// The speed setting changed (also from another control).
export function speedChanged() {
  if (elements) syncSpeed();
}

// What the playback of a running run is doing, in words.
function playNote(running) {
  if (!playing) return "";
  if (playing.waiting === "starting") return "starting when a few snapshots are ready…";
  if (playing.waiting === "computing") return "waiting for the run…";
  if (playing.waiting === "loading") return "loading…";
  if (running && playing.speed > 0 && playing.speed < settings.playbackSpeed) return `playing at ${speedLabel(playing.speed)}: the run computes slower than ${speedLabel(settings.playbackSpeed)}`;
  return "";
}

// Shows the bar for the current time solution, or hides it. A run being
// computed is played from its start (store.playRun).
export function updateTimebar() {
  if (!elements) return;
  const result = timeSolution();
  if (result !== shownResult) {
    stopPlaying();
    shownResult = result;
  }
  elements.root.hidden = !result;
  if (!result) return;
  const running = Boolean(followedRun()) && result.state === "running";
  // The run's pace, from its progress.
  if (running) paces.set(result, addPaceSample(paces.get(result) ?? [], performance.now(), result.t));
  if (running && store.playRun && !playing && settings.followRun) play({ fromStart: store.solveFrame === 0 });
  const last = result.values.length - 1;
  const i = Math.min(store.solveFrame, last);
  elements.slider.max = String(last);
  elements.slider.value = String(i);
  elements.slider.setAttribute("aria-valuetext", `t = ${formatNumber(result.times[i], 5)}`);
  elements.time.textContent = `t = ${formatNumber(result.times[i], 5)}`;
  // The server's range of u at this snapshot, in double (null where a
  // snapshot has no finite value).
  const min = result.snapshot_min?.[i];
  const max = result.snapshot_max?.[i];
  const known = i < (result.snapshot_min?.length ?? 0);
  elements.range.textContent = !known ? "" : Number.isFinite(min) && Number.isFinite(max) ? `u: min ${formatNumber(min)} · max ${formatNumber(max)}` : "u: no finite value";
  // At phone width the range is hidden (picture.css): the time says it.
  elements.time.title = elements.range.textContent;
  elements.run.hidden = !running;
  if (running) {
    const span = result.t1 - result.t0;
    elements.bar.value = span > 0 ? Math.max(0, Math.min(1, (result.t - result.t0) / span)) : 0;
    elements.runText.textContent = `${result.values.length} of ${result.snapshots_total}`;
    elements.playText.textContent = playNote(running);
  }
}
