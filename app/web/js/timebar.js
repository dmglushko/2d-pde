// The time bar under the picture: play or pause, a slider over the
// snapshots and the shown time with that snapshot's smallest and largest
// value. It is the one control of the shown snapshot (store.solveFrame).
// During a run it shows how far the run got.

import { followedRun } from "./actions.js";
import { formatNumber, valueRange } from "./core.js";
import { h } from "./dom.js";
import { settings } from "./settings.js";
import { freshResult, notify, store } from "./state.js";

let elements = null;
let playTimer = null;
let shownResult = null;
// Per snapshot array: its range (computed once).
const ranges = new WeakMap();

function rangeOf(values) {
  let range = ranges.get(values);
  if (!range) {
    range = valueRange(values);
    ranges.set(values, range);
  }
  return range;
}

// The time solution the bar walks through, or null.
export function timeSolution() {
  const result = freshResult("solve");
  return result && result.mode === "time" && result.values.length > 0 && store.fieldSource === "solve" ? result : null;
}

export function stopPlaying() {
  clearInterval(playTimer);
  playTimer = null;
  if (elements) syncPlay();
}

// Shows snapshot `index` of the solution (moving the bar counts as a
// choice: a run in progress no longer moves it).
export function showFrame(index) {
  store.solveFrame = index;
  store.solveFrameChosen = true;
  store.fieldSource = "solve";
  if (store.view.colorBy !== "field") store.view.colorBy = "field";
  notify("frame");
}

function syncPlay() {
  const playing = playTimer !== null;
  elements.play.textContent = playing ? "❚❚" : "▶";
  elements.play.setAttribute("aria-label", playing ? "Pause" : "Play");
  elements.play.title = playing ? "Pause" : "Play";
}

function play() {
  if (playTimer) return stopPlaying();
  const result = timeSolution();
  if (!result) return;
  const last = () => result.values.length - 1;
  // From the start when at the end.
  showFrame(store.solveFrame >= last() ? 0 : store.solveFrame);
  playTimer = setInterval(() => {
    // Stop when another result or field took the picture.
    if (timeSolution() !== result) return stopPlaying();
    const next = store.solveFrame + 1;
    if (next > last()) return stopPlaying();
    showFrame(next);
  }, 1000 / settings.playbackFps);
  syncPlay();
}

export function wireTimebar(root) {
  elements = {
    root,
    play: h("button", { type: "button", class: "icon play", id: "time-play", "aria-label": "Play", title: "Play" }, "▶"),
    slider: h("input", { type: "range", id: "time-slider", min: "0", max: "0", step: "1", value: "0", "aria-label": "Snapshot" }),
    time: h("span", { class: "num time-now" }),
    range: h("span", { class: "num time-range" }),
    run: h("div", { class: "time-run", hidden: true }),
  };
  elements.bar = h("progress", { max: "1", value: "0", "aria-label": "Run progress" });
  elements.runText = h("span", { class: "num" });
  elements.run.append(elements.bar, elements.runText);
  elements.play.addEventListener("click", play);
  elements.slider.addEventListener("input", () => {
    stopPlaying();
    showFrame(Number(elements.slider.value));
  });
  root.append(
    elements.play,
    elements.slider,
    h("div", { class: "time-label" }, elements.time, elements.range),
    elements.run,
  );
  updateTimebar();
}

// Shows the bar for the current time solution, or hides it.
export function updateTimebar() {
  if (!elements) return;
  const result = timeSolution();
  if (result !== shownResult) {
    stopPlaying();
    shownResult = result;
  }
  elements.root.hidden = !result;
  if (!result) return;
  const last = result.values.length - 1;
  const i = Math.min(store.solveFrame, last);
  elements.slider.max = String(last);
  elements.slider.value = String(i);
  elements.slider.setAttribute("aria-valuetext", `t = ${formatNumber(result.times[i], 5)}`);
  elements.time.textContent = `t = ${formatNumber(result.times[i], 5)}`;
  const range = rangeOf(result.values[i]);
  elements.range.textContent = `min ${formatNumber(range.min)} · max ${formatNumber(range.max)}`;
  const running = Boolean(followedRun()) && result.state === "running";
  elements.run.hidden = !running;
  if (running) {
    const span = result.t1 - result.t0;
    elements.bar.value = span > 0 ? Math.max(0, Math.min(1, (result.t - result.t0) / span)) : 0;
    elements.runText.textContent = `${result.values.length} of ${result.snapshots_total}`;
  }
}
