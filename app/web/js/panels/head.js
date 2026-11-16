// The head of every step's panel: its title, its primary action and a
// one-line status (running with the seconds, up to date, outdated,
// failed), with a "Next" link once the result is up to date. It stays at
// the top of the panel while the panel scrolls (style.css, .step-head).

import { STEPS, stageInfo } from "../core.js";
import { h, fill } from "../dom.js";
import { openStep, stageStatus } from "../state.js";

const STATUS_TEXT = {
  done: "Up to date",
  stale: "Outdated: the case changed",
  stopped: "Stopped before the end",
  failed: "Failed",
  none: "Not run yet",
};

// The colour class of a status dot (the step tabs use the same).
export function dotClass(status) {
  if (status === "stopped") return "stale";
  return status === "none" ? "" : status;
}

export function stepHead(step, ...actions) {
  const status = h("div", { class: "step-status", role: "status" });
  const head = h(
    "div",
    { class: "step-head" },
    h("div", { class: "step-title" }, h("h2", {}, stageInfo(step).label), h("div", { class: "step-actions" }, actions)),
    status,
  );
  head.statusLine = status;
  return head;
}

// Shows the status of `step` in its head. `status` defaults to the
// stage's (state.js, stageStatus); `text` replaces the words for it,
// `detail` follows them, `extra` (a node, such as a progress bar) goes on
// its own line. Rebuilt only when something shown changed.
export function setStepStatus(head, step, { status = stageStatus(step), text = null, detail = null, extra = null } = {}) {
  const line = head?.statusLine;
  if (!line) return;
  const key = JSON.stringify([status, text, detail]);
  if (line.shown === key && line.extra === extra) return;
  line.shown = key;
  line.extra = extra;
  const info = stageInfo(step);
  const words =
    status === "busy" && text === null
      ? h("span", { dataset: { elapsedStage: step, label: info.running } }, `${info.running}…`)
      : h("span", {}, text ?? STATUS_TEXT[status]);
  const next = STEPS[STEPS.indexOf(step) + 1];
  const link =
    status === "done" && next
      ? h("button", { type: "button", class: "link", onclick: () => openStep(next) }, `Next: ${stageInfo(next).label} →`)
      : null;
  fill(
    line,
    h("span", { class: `dot ${dotClass(status)}`.trim(), "aria-hidden": "true" }),
    words,
    detail ? h("span", { class: "detail" }, detail) : null,
    h("span", { class: "spacer" }),
    link,
    extra ? h("div", { class: "status-extra" }, extra) : null,
  );
}
