// App preferences: kept in this browser (localStorage), not in the case,
// since they are about the viewer, not the problem. Every one of them
// drives code. Threads are not here: they are options of the case's
// solve section (the linear solver's and the assembly's).

import { parseNumberText, setDefaultDigits } from "./pure/format.js";
import { COLOR_MAPS } from "./pure/color.js";
import { normalizeSettings, SETTINGS_DEFAULTS } from "./pure/prefs.js";
import { h, infoToggle } from "./dom.js";

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

let controlCount = 0;

// One setting: its label (tied to the control), the control and, behind
// an info toggle, what it does.
function row(label, help, control) {
  const id = `setting-${++controlCount}`;
  const input = control.matches?.("input, select") ? control : control.querySelector("input, select");
  if (input) input.id = id;
  const info = infoToggle(help, label);
  return h(
    "div",
    { class: "field setting" },
    h("div", { class: "setting-line" }, h("label", { class: "label", for: id }, label), info.button, h("span", { class: "spacer" }), control),
    info.panel,
  );
}

function checkbox(key) {
  return h("input", { type: "checkbox", checked: settings[key], onchange: (e) => setSetting(key, e.target.checked) });
}

function number(key, { min, max, step, unit }) {
  return h(
    "span",
    { class: "inputs" },
    h("input", {
      type: "text",
      inputmode: step < 1 ? "decimal" : "numeric",
      autocomplete: "off",
      class: "number setting-number",
      title: `From ${min} to ${max}`,
      value: String(settings[key]),
      // A value out of range is clamped, text that is not a number leaves
      // the setting as it was; the field then shows what holds.
      onchange: (e) => {
        const parsed = parseNumberText(e.target.value);
        if (parsed.kind === "number") setSetting(key, parsed.value);
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

function content() {
  return [
    h("p", { class: "note" }, "Kept in this browser, not in the case."),
    h("h3", {}, "Problem check"),
    row("Check while typing", "Off: the problem is checked when a field is left or Enter is pressed.", checkbox("checkWhileTyping")),
    h("h3", {}, "Solution"),
    row(
      "Play a run as it is computed",
      "During a run in time, the picture plays it from the start at the playback speed of the time bar (1× is real time), no faster than the run computes. Moving the time bar or Pause stops it until the next run.",
      checkbox("followRun"),
    ),
    h("h3", {}, "Numbers and picture"),
    row("Significant digits", "In tables, figures, labels and the readout under the pointer.", number("digits", { min: 2, max: 10, step: 1 })),
    row("Isolines", "How many isolines the Isolines layer draws, evenly inside the colour scale.", number("isolineCount", { min: 2, max: 40, step: 1 })),
    row(
      "Colours of one sign",
      "The colour map of a field whose values all have one sign: one hue, dark for the lowest values, light for the highest.",
      choice("sequentialMap", COLOR_MAPS.sequential.map((m, i) => [m.id, i === 0 ? `${m.label} (default)` : m.label])),
    ),
    row(
      "Colours of both signs",
      "The colour map of a field with values of both signs (a difference, an error): two hues, grey at zero.",
      choice("divergingMap", COLOR_MAPS.diverging.map((m, i) => [m.id, i === 0 ? `${m.label} (default)` : m.label])),
    ),
    row(
      "Picture quality",
      "The pixel ratio the picture is drawn at, at most the screen's: lower is faster (panning, fields), higher is sharper.",
      choice("pixelRatioCap", [
        [1, "fast (1×)"],
        [1.5, "balanced (1.5×)"],
        [2, "sharp (2×)"],
        [4, "the screen's"],
      ]),
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
