// Small DOM helpers and the forms built from catalog option specs.

import { regionToken } from "./pure/color.js";
import { formatNumber, numberText, parseNumberText } from "./pure/format.js";
import { parseFolds } from "./pure/prefs.js";
import { errorPathOf } from "./pure/stages.js";
import { blockedReason, setInvalid } from "./state.js";

// h("div", {class: "x", onclick: f}, child, "text", ...)
export function h(tag, attributes = {}, ...children) {
  const element = document.createElement(tag);
  for (const [name, value] of Object.entries(attributes ?? {})) {
    if (value === undefined || value === null || value === false) continue;
    if (name.startsWith("on")) element.addEventListener(name.slice(2), value);
    else if (name === "class") element.className = value;
    // A textarea's value attribute does not set its text: use the property.
    else if (name === "value" && "value" in element && element.tagName !== "OPTION" && element.tagName !== "SELECT") element.value = value;
    else if (name === "dataset") Object.assign(element.dataset, value);
    else if (name in element && typeof value !== "string") element[name] = value;
    else element.setAttribute(name, value === true ? "" : value);
  }
  element.append(...nodes(children));
  return element;
}

// The children as nodes: nested lists flattened, null, undefined and
// false left out, anything else as text.
export function nodes(children) {
  return children
    .flat(Infinity)
    .filter((child) => child !== null && child !== undefined && child !== false)
    .map((child) => (child instanceof Node ? child : document.createTextNode(String(child))));
}

// Replaces the children of `container` (with the rules of nodes()).
export function fill(container, ...children) {
  container.replaceChildren(...nodes(children));
}

// --- foldable sections -----------------------------------------------------------

// Which sections are folded is a per-viewer preference, kept in this
// browser: {id: folded}.
const FOLDS_KEY = "pde2d-app-folds";
let folds = null;

function foldStore() {
  if (!folds) {
    try {
      folds = parseFolds(globalThis.localStorage?.getItem(FOLDS_KEY));
    } catch {
      folds = Object.create(null);
    }
  }
  return folds;
}

export function isFolded(id, fallback = false) {
  const stored = foldStore();
  return Object.hasOwn(stored, id) ? stored[id] : fallback;
}

function setOpen(button, open, remember) {
  const body = document.getElementById(button.getAttribute("aria-controls"));
  button.setAttribute("aria-expanded", String(open));
  if (body) body.hidden = !open;
  if (!remember) return;
  foldStore()[button.dataset.fold] = !open;
  try {
    globalThis.localStorage?.setItem(FOLDS_KEY, JSON.stringify(folds));
  } catch {
    // Not kept: the section stays as it is for this visit.
  }
}

let foldCount = 0;

// A section that folds: a header whose title is a button (aria-expanded,
// so Enter and Space toggle it) and a body. `id` names the section for
// the stored preference; `folded` is how it starts when nothing is
// stored. kind "card" gives a .card, "section" a heading (h3) with the
// body below. `extras` go into the header after the title (controls of
// the section, which do not toggle it); `info` (an info toggle's panel)
// goes under the header, outside the body, so it shows also when folded;
// `attributes` go on the outer element.
export function foldable({ id, title, folded = false, kind = "card", extras = [], info = null, attributes = {} }, ...children) {
  const bodyId = `fold-${++foldCount}`;
  const shut = isFolded(id, folded);
  const button = h(
    "button",
    { type: "button", class: "fold-toggle", "aria-expanded": String(!shut), "aria-controls": bodyId, dataset: { fold: id } },
    h("span", { class: "chevron", "aria-hidden": "true" }),
    h("span", { class: "title" }, title),
  );
  button.addEventListener("click", () => setOpen(button, button.getAttribute("aria-expanded") !== "true", true));
  const body = h("div", { id: bodyId, class: "fold-body", hidden: shut }, children);
  const head = kind === "card" ? h("div", { class: "card-head" }, button, extras) : h("div", { class: "section-head" }, h("h3", {}, button), extras);
  const base = kind === "card" ? "card foldable" : "fold-section";
  return h("div", { ...attributes, class: attributes.class ? `${base} ${attributes.class}` : base }, head, info, body);
}

// Opens every folded section around `element` (an error marked inside
// must be seen), without changing the stored preference.
export function revealFolded(element) {
  for (let node = element?.parentElement; node; node = node.parentElement)
    if (node.classList?.contains("fold-body") && node.hidden) {
      const button = document.querySelector(`[aria-controls="${node.id}"]`);
      if (button) setOpen(button, true, false);
    }
}

// Marks `input`'s field as locally invalid with `message` (null clears
// it). Local marks are the page's own (text that is not a number yet),
// apart from the server's errors.
export function markLocal(input, message) {
  const field = input.closest(".param-row, label, .field") ?? input.parentElement;
  input.classList.toggle("local-invalid", Boolean(message));
  input.setAttribute("aria-invalid", message ? "true" : "false");
  let note = field?.querySelector(":scope > [data-local-error]");
  if (!message) {
    note?.remove();
    if (field && !field.querySelector(".local-invalid")) field.classList.remove("local-field-invalid");
    return;
  }
  field?.classList.add("local-field-invalid");
  revealFolded(input);
  if (!note && field) {
    note = h("div", { class: "error-text", "data-local-error": "" });
    field.append(note);
  }
  if (note) note.textContent = message;
}

// A number field: a text input that shows and reads numbers with a
// decimal point whatever the browser's locale (pure/format.js). Text that
// is not a number changes nothing in the case: the field says so with the
// value the case keeps and, when it has a case `path`, is recorded as
// invalid (state.js), which blocks runs until it is fixed or the forms
// are built again. onNumber(value) for a number; onEmpty() for an empty
// field (without it, an empty field is invalid too).
export function numberInput({ value, label, path = null, placeholder = "", className = null, onNumber, onEmpty = null, onCommit = null }) {
  let kept = numberText(value);
  const input = h("input", {
    type: "text",
    inputmode: "decimal",
    autocomplete: "off",
    spellcheck: false,
    class: className ? `number ${className}` : "number",
    value: kept,
    placeholder,
    "aria-label": label,
    dataset: path ? { path } : {},
    onchange: onCommit,
  });
  input.addEventListener("input", () => {
    const parsed = parseNumberText(input.value);
    if (parsed.kind === "invalid" || (parsed.kind === "empty" && !onEmpty)) {
      const reason = parsed.kind === "invalid" ? parsed.reason : "a value is needed";
      const keeps = kept === "" ? "no value (its default)" : kept;
      markLocal(input, `${reason.charAt(0).toUpperCase()}${reason.slice(1)}. The case still has ${keeps}.`);
      if (path) setInvalid(path, { label, reason });
      return;
    }
    markLocal(input, null);
    if (path) setInvalid(path, null);
    kept = parsed.kind === "empty" ? "" : numberText(parsed.value);
    if (parsed.kind === "empty") onEmpty();
    else onNumber(parsed.value);
  });
  return input;
}

// A small "i" button that shows or hides `text` (a description the user
// may want, not one printed under every field). The text is also the
// button's tooltip. Returns {button, panel}: the caller puts the button
// next to what it explains and the panel below it. Both are null without
// a text.
let infoCount = 0;
export function infoToggle(text, about) {
  if (!text) return { button: null, panel: null };
  const id = `info-${++infoCount}`;
  const panel = h("div", { id, class: "info-text", hidden: true }, text);
  const button = h(
    "button",
    {
      type: "button",
      class: "info",
      title: text,
      "aria-label": `About ${about}`,
      "aria-expanded": "false",
      "aria-controls": id,
    },
    "i",
  );
  button.addEventListener("click", (event) => {
    // Inside a fold header: show the text, do not fold.
    event.stopPropagation();
    const open = button.getAttribute("aria-expanded") !== "true";
    button.setAttribute("aria-expanded", String(open));
    panel.hidden = !open;
  });
  return { button, panel };
}

// A label with its info toggle: [label-row, info panel].
export function labelWithInfo(label, description, tag = "div") {
  const info = infoToggle(description, label);
  return [h("div", { class: "label-row" }, h(tag, { class: "label" }, label), info.button), info.panel];
}

function lengthBoundInputs(spec, value, path, onChange) {
  const bound = { absolute: null, relative: null, ...(value ?? spec.default ?? {}) };
  const set = (key) => (v) => {
    bound[key] = v;
    onChange({ ...bound });
  };
  return h(
    "div",
    { class: "bound", dataset: { path } },
    h("span", {}, "absolute"),
    numberInput({
      value: bound.absolute,
      label: `${spec.label}, absolute`,
      placeholder: "no bound",
      path: `${path}.absolute`,
      onNumber: set("absolute"),
      onEmpty: () => set("absolute")(null),
    }),
    h("span", {}, "× domain size"),
    numberInput({
      value: bound.relative,
      label: `${spec.label}, relative`,
      placeholder: "no bound",
      path: `${path}.relative`,
      onNumber: set("relative"),
      onEmpty: () => set("relative")(null),
    }),
  );
}

function choiceInput(spec, value, path, onChange) {
  const options = spec.choices.map((choice) =>
    h("option", { value: choice, selected: choice === (value ?? spec.default) }, choice.replaceAll("_", " ")),
  );
  return h("select", { "aria-label": spec.label, dataset: { path }, onchange: (event) => onChange(event.target.value) }, options);
}

// One option of a component or a shape, as a form field. `value` is the
// current value (null or undefined: the default); onChange(newValue).
// The option's description is behind its info toggle.
export function optionField(spec, value, path, onChange) {
  let input;
  if (spec.kind === "choice") input = choiceInput(spec, value, path, onChange);
  else if (spec.kind === "length_bound") input = lengthBoundInputs(spec, value, path, onChange);
  else
    input = numberInput({
      value: value ?? spec.default,
      label: spec.label,
      placeholder: `default ${numberText(spec.default)}`,
      path,
      onNumber: onChange,
      onEmpty: () => onChange(spec.default),
    });
  const unit = spec.unit ? h("span", { class: "unit" }, spec.unit) : null;
  const [labelRow, info] = labelWithInfo(spec.label, spec.description);
  return h("div", { class: "field", dataset: { path } }, labelRow, h("div", { class: "inputs" }, input, unit), info);
}

// The options of a component: one field per spec, writing into `options`.
export function optionsForm(specs, options, path, onChange) {
  return specs.map((spec) =>
    optionField(spec, options?.[spec.name], `${path}.${spec.name}`, (value) => {
      options[spec.name] = value;
      onChange();
    }),
  );
}

// Marks the form element of the path an error names, and shows the
// message under it. Returns true when the error was placed in the form.
export function markError(container, error) {
  clearErrors(container);
  if (!error) return false;
  let path = errorPathOf(error);
  // The message without the path it starts with.
  const prefix = path ? `${path}: ` : null;
  const detail = prefix && error.message.startsWith(prefix) ? error.message.slice(prefix.length) : error.message.slice(error.message.indexOf(": ") + 2);
  while (path) {
    const target = container.querySelector(`.field[data-path="${CSS.escape(path)}"], [data-path="${CSS.escape(path)}"]`);
    if (target) {
      const field = target.closest(".field") ?? target;
      field.classList.add("invalid");
      field.append(h("div", { class: "error-text", "data-error": "" }, detail));
      revealFolded(field);
      return true;
    }
    const cut = Math.max(path.lastIndexOf("."), path.lastIndexOf("["));
    path = cut > 0 ? path.slice(0, cut) : null;
  }
  return false;
}

export function clearErrors(container) {
  for (const element of container.querySelectorAll(".invalid")) element.classList.remove("invalid");
  for (const element of container.querySelectorAll("[data-error]")) element.remove();
}

// An error box for a failed stage. `outdated`: the error was produced for
// an earlier input (muted, not an alert). `openProblem(step)`: a button
// that opens the Problem step, for an error of the problem or the
// MeshedProblem shown in another step.
export function errorBox(error, { outdated = false, openProblem = null } = {}) {
  if (!error) return null;
  const kind = { input: "The input was refused", algorithm: "The algorithm could not finish", internal: "Something went wrong" }[error.kind] ?? "Error";
  const aboutProblem = ["problem", "meshed_problem"].includes(error.stage);
  const title = h("span", { class: "stage" }, `${kind} (${error.stage})`, outdated ? h("span", { class: "chip" }, "for the previous input") : null);
  return h(
    "div",
    { class: outdated ? "alert outdated" : "alert", role: outdated ? null : "alert" },
    title,
    error.message,
    aboutProblem && openProblem
      ? h("div", { class: "alert-actions" }, h("button", { type: "button", class: "small", onclick: openProblem }, "Open the Problem step"))
      : null,
  );
}

// A small export button, disabled with a reason when the result it would
// export is not the one shown as current.
// `about` is its tooltip while it is enabled.
export function exportButton(label, action, { fresh, staleReason, about = null }) {
  const button = actionButton(label, action, { small: true });
  if (about) button.title = about;
  if (!fresh) {
    button.disabled = true;
    button.title = staleReason;
  }
  return button;
}

// The reports of a run, one collapsible block per stage.
function reportSummary(report) {
  const count = report.messages.length;
  return h(
    "summary",
    {},
    h("strong", {}, report.stage),
    count ? h("span", { class: "chip" }, `${count} message${count > 1 ? "s" : ""}`) : null,
    h("span", { class: "time" }, `${formatNumber(report.time_ms, 3)} ms`),
  );
}

function metricsTable(metrics) {
  const rows = Object.entries(metrics).map(([name, value]) =>
    h("tr", {}, h("td", {}, name.replaceAll("_", " ")), h("td", { class: "num" }, formatNumber(value, 6))),
  );
  return rows.length ? h("table", { class: "data" }, h("tbody", {}, rows)) : null;
}

export function reportsView(reports) {
  return reports.map((report) =>
    h(
      "details",
      { class: "report", dataset: report.messages.length ? { messages: "" } : {} },
      reportSummary(report),
      h(
        "div",
        { class: "body" },
        report.messages.length ? h("ul", {}, report.messages.map((m) => h("li", {}, m))) : null,
        metricsTable(report.metrics),
      ),
    ),
  );
}

// A chip at the top of a result when its reports hold messages (each a
// "worked, but…" of the library): the count and the first message. A
// click opens the reports (the fold `foldId` of the same panel) and every
// report that has messages.
export function warningChip(reports, foldId) {
  const messages = (reports ?? []).flatMap((r) => r.messages);
  if (messages.length === 0) return null;
  const count = messages.length;
  return h(
    "button",
    {
      type: "button",
      class: "warning-chip",
      title: "Open the reports",
      onclick: (event) => openReports(event.currentTarget, foldId),
    },
    h("span", { class: "warning-mark", "aria-hidden": "true" }, "!"),
    h("span", { class: "warning-count" }, `${count} warning${count > 1 ? "s" : ""}`),
    h("span", { class: "warning-first" }, messages[0]),
  );
}

function openReports(from, foldId) {
  const root = from.closest(".panel") ?? document;
  const toggle = root.querySelector(`[data-fold="${CSS.escape(foldId)}"]`);
  if (!toggle) return;
  setOpen(toggle, true, false);
  const body = document.getElementById(toggle.getAttribute("aria-controls"));
  for (const details of body?.querySelectorAll("details[data-messages]") ?? []) details.open = true;
  toggle.scrollIntoView({ block: "start", behavior: "smooth" });
  toggle.focus({ preventScroll: true });
}

// The colour swatch of region `index` (style.css: .swatch.series-N).
export function regionSwatch(index) {
  return h("span", { class: `swatch ${regionToken(index).slice(2)}`, "aria-hidden": "true" });
}

// A figure of a result: its value over its label. `title` explains it.
export function stat(label, value, title = null) {
  return h("div", { class: "stat", title }, h("div", { class: "value" }, value), h("div", { class: "label" }, label));
}

// A button that shows a spinner while `action` runs. `guarded`: a run
// that number fields holding text block (state.js, blockedReason); the
// button is then disabled with the reason (guardButtons).
export function actionButton(label, action, { primary = false, small = false, guarded = false } = {}) {
  const classes = [primary ? "primary" : "", small ? "small" : ""].join(" ").trim();
  const button = h("button", { type: "button", class: classes || null, dataset: guarded ? { guarded: "" } : {} }, label);
  button.addEventListener("click", async () => {
    button.disabled = true;
    button.dataset.running = "";
    const spinner = h("span", { class: "busy", "aria-hidden": "true" });
    button.prepend(spinner);
    try {
      await action();
    } finally {
      spinner.remove();
      delete button.dataset.running;
      button.disabled = guarded && blockedReason() !== null;
    }
  });
  if (guarded) guardButton(button);
  return button;
}

// Disables a guarded button while runs are blocked, with the reason as
// its tooltip.
export function guardButton(button) {
  if (button.dataset.running !== undefined) return;
  const reason = blockedReason();
  button.disabled = reason !== null;
  if (reason) button.title = `Blocked: ${reason}`;
  else if (button.title.startsWith("Blocked: ")) button.title = "";
}

// Every guarded button of the page.
export function guardButtons() {
  for (const button of document.querySelectorAll("button[data-guarded]")) guardButton(button);
}

// A button that opens a file chooser (a hidden file input): a real
// button, so it takes the focus and Enter or Space like any other.
// onFile(file) gets the chosen file.
export function fileButton(label, accept, onFile, { id = null, small = false } = {}) {
  const input = h("input", {
    type: "file",
    accept,
    hidden: true,
    tabindex: "-1",
    onchange: (event) => {
      const file = event.target.files[0];
      event.target.value = "";
      if (file) onFile(file);
    },
  });
  const button = h("button", { type: "button", id, class: small ? "small" : null, onclick: () => input.click() }, label);
  return h("span", { class: "file-button" }, button, input);
}

// Reads a file chosen by the user as text.
export function readFileText(file) {
  return new Promise((resolve, reject) => {
    const reader = new FileReader();
    reader.onload = () => resolve(String(reader.result));
    reader.onerror = () => reject(reader.error);
    reader.readAsText(file);
  });
}
