// App preferences: kept in this browser (localStorage), not in the case,
// since they are about the viewer, not the problem. Every one of them
// drives code; the computation group is shown disabled until the library
// can run in parallel (docs/library-requests.md, item 1).

import { normalizeSettings, SETTINGS_DEFAULTS, setDefaultDigits } from "./core.js";
import { h } from "./dom.js";

const SETTINGS_KEY = "pde2d-app-settings";

function load() {
  try {
    return normalizeSettings(JSON.parse(localStorage.getItem(SETTINGS_KEY) ?? "null"));
  } catch {
    return normalizeSettings(null);
  }
}

export const settings = load();
setDefaultDigits(settings.digits);

const listeners = new Set();

export function onSettings(listener) {
  listeners.add(listener);
}

export function setSetting(key, value) {
  const next = normalizeSettings({ ...settings, [key]: value });
  if (next[key] === settings[key]) return;
  Object.assign(settings, next);
  setDefaultDigits(settings.digits);
  try {
    localStorage.setItem(SETTINGS_KEY, JSON.stringify(settings));
  } catch {
    // A preference that is not kept still holds for this visit.
  }
  for (const listener of listeners) listener(key);
}

// --- the dialog -------------------------------------------------------------------

function row(label, help, control) {
  return h("div", { class: "field setting" }, h("label", { class: "label" }, label, control), help ? h("div", { class: "help" }, help) : null);
}

function checkbox(key) {
  return h("input", { type: "checkbox", checked: settings[key], onchange: (e) => setSetting(key, e.target.checked) });
}

function number(key, { min, max, step, unit }) {
  return h(
    "span",
    { class: "inputs" },
    h("input", {
      type: "number",
      min: String(min),
      max: String(max),
      step: String(step),
      value: String(settings[key]),
      style: "width:90px",
      onchange: (e) => {
        setSetting(key, Number(e.target.value));
        e.target.value = String(settings[key]);
      },
    }),
    unit ? h("span", { class: "unit" }, unit) : null,
  );
}

function choice(key, options) {
  return h(
    "select",
    { onchange: (e) => setSetting(key, Number.isNaN(Number(e.target.value)) ? e.target.value : Number(e.target.value)) },
    options.map(([value, label]) => h("option", { value: String(value), selected: settings[key] === value }, label)),
  );
}

function computation(label) {
  const name = `computation-${label}`;
  return h(
    "div",
    { class: "row", role: "radiogroup", "aria-label": label, "aria-disabled": "true" },
    ["Sequential", "Parallel"].map((text, i) =>
      h("label", { class: "chip", style: "padding:4px 10px" }, h("input", { type: "radio", name, disabled: true, checked: i === 0 }), text),
    ),
  );
}

function content() {
  return [
    h("h3", {}, "Problem check"),
    row("Check the problem while typing ", "Off: the problem is checked when a field is left or Enter is pressed.", checkbox("checkWhileTyping")),
    row("Delay after the last key ", "The check runs after this pause in typing.", number("checkDelayMs", { min: 100, max: 3000, step: 50, unit: "ms" })),
    h("h3", {}, "Solution"),
    row("Playback speed ", null, number("playbackFps", { min: 1, max: 60, step: 1, unit: "frames per second" })),
    row("Follow the newest snapshot during a run ", "Moving the slider during a run stops following until the next run.", checkbox("followRun")),
    h("h3", {}, "Numbers and picture"),
    row("Significant digits ", "In tables, stats, labels and the readout under the pointer.", number("digits", { min: 2, max: 10, step: 1 })),
    row(
      "Picture quality ",
      "The pixel ratio the picture is drawn at, at most the screen's: lower is faster (panning, fields), higher is sharper.",
      choice("pixelRatioCap", [
        [1, "fast (1×)"],
        [1.5, "balanced (1.5×)"],
        [2, "sharp (2×)"],
        [4, "the screen's"],
      ]),
    ),
    h("h3", {}, "Computation"),
    row("Assembly ", null, computation("Assembly")),
    row("Linear solves ", null, computation("Linear solves")),
    h(
      "p",
      { class: "note" },
      "The library runs sequentially for now. Parallel assembly and parallel linear solves are recorded as a library request (docs/library-requests.md, item 1); this setting becomes active when the library offers them.",
    ),
  ];
}

// The Settings dialog behind the gear button. A <dialog> opened with
// showModal: it keeps the focus inside and closes with Escape.
export function wireSettings(button, dialog) {
  const body = dialog.querySelector(".settings-body");
  button.addEventListener("click", () => {
    body.replaceChildren(...content());
    dialog.showModal();
  });
  dialog.querySelector("[data-close]").addEventListener("click", () => dialog.close());
  dialog.addEventListener("click", (event) => {
    if (event.target === dialog) dialog.close();
  });
  dialog.addEventListener("close", () => button.focus());
}

export { SETTINGS_DEFAULTS };
