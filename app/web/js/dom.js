// Small DOM helpers and the forms built from catalog option specs.

import { errorPath, formatNumber, parseNumberText } from "./core.js";

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
  if (!note && field) {
    note = h("div", { class: "error-text", "data-local-error": "" });
    field.append(note);
  }
  if (note) note.textContent = message;
}

// The handler of a number input: onNumber(value) for a number, onEmpty()
// for an empty field; text that is not a number (such as "-" or "1e")
// changes nothing and marks the field "not a number" until it parses.
export function numberHandler(onNumber, onEmpty = null) {
  return (event) => {
    const input = event.target;
    const parsed = parseNumberText(input.value, input.validity?.badInput);
    if (parsed.kind === "invalid" || (parsed.kind === "empty" && !onEmpty)) {
      markLocal(input, parsed.kind === "invalid" ? "not a number" : "a value is needed");
      return;
    }
    markLocal(input, null);
    if (parsed.kind === "empty") onEmpty();
    else onNumber(parsed.value);
  };
}

// A number input whose empty value is null.
function numberInput(value, { integer = false, placeholder = "", label, path, onChange }) {
  return h("input", {
    type: "number",
    step: integer ? "1" : "any",
    value: value === null || value === undefined ? "" : String(value),
    placeholder,
    "aria-label": label,
    dataset: { path },
    oninput: numberHandler(onChange, () => onChange(null)),
  });
}

// One option of a component or a shape, as a form field. `value` is the
// current value (null or undefined: the default); onChange(newValue).
export function optionField(spec, value, path, onChange) {
  const help = spec.description ? h("div", { class: "help" }, spec.description) : null;
  const unit = spec.unit ? h("span", { class: "unit" }, spec.unit) : null;
  let input;
  if (spec.kind === "choice") {
    input = h(
      "select",
      { "aria-label": spec.label, dataset: { path }, onchange: (event) => onChange(event.target.value) },
      spec.choices.map((choice) => h("option", { value: choice, selected: choice === (value ?? spec.default) }, choice.replaceAll("_", " "))),
    );
  } else if (spec.kind === "length_bound") {
    const bound = { absolute: null, relative: null, ...(value ?? spec.default ?? {}) };
    const set = (key) => (v) => {
      bound[key] = v;
      onChange({ ...bound });
    };
    input = h(
      "div",
      { class: "bound", dataset: { path } },
      h("span", {}, "absolute"),
      numberInput(bound.absolute, { label: `${spec.label}, absolute`, placeholder: "no bound", path: `${path}.absolute`, onChange: set("absolute") }),
      h("span", {}, "× domain size"),
      numberInput(bound.relative, { label: `${spec.label}, relative`, placeholder: "no bound", path: `${path}.relative`, onChange: set("relative") }),
    );
  } else {
    const integer = spec.kind === "integer";
    input = numberInput(value ?? spec.default, {
      integer,
      label: spec.label,
      placeholder: `default ${formatNumber(spec.default)}`,
      path,
      onChange: (v) => onChange(v ?? spec.default),
    });
  }
  return h("div", { class: "field", dataset: { path } }, h("div", { class: "label" }, spec.label), h("div", { class: "inputs" }, input, unit), help);
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
  let path = errorPath(error.message);
  while (path) {
    const target = container.querySelector(`.field[data-path="${CSS.escape(path)}"], [data-path="${CSS.escape(path)}"]`);
    if (target) {
      const field = target.closest(".field") ?? target;
      field.classList.add("invalid");
      field.append(h("div", { class: "error-text", "data-error": "" }, error.message.slice(error.message.indexOf(": ") + 2)));
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
  return h(
    "div",
    { class: outdated ? "alert outdated" : "alert", role: outdated ? null : "alert" },
    h("span", { class: "stage" }, `${kind} (${error.stage})`, outdated ? h("span", { class: "chip" }, "for the previous input") : null),
    error.message,
    aboutProblem && openProblem ? h("div", { style: "margin-top:6px" }, h("button", { type: "button", class: "small", onclick: openProblem }, "Open the Problem step")) : null,
  );
}

// "Meshing… 3 s" while `stage` runs; main.js updates the seconds.
export function busyNote(stage, text) {
  return h("p", { class: "note busy-note", dataset: { elapsedStage: stage, label: text } }, `${text}…`);
}

// A small export button, disabled with a reason when the result it would
// export is not the one shown as current.
export function exportButton(label, action, { fresh, staleReason }) {
  const button = actionButton(label, action, { small: true });
  if (!fresh) {
    button.disabled = true;
    button.title = staleReason;
  }
  return button;
}

// The reports of a run, one collapsible block per stage.
export function reportsView(reports) {
  return reports.map((report) =>
    h(
      "details",
      { class: "report" },
      h("summary", {}, h("strong", {}, report.stage), report.messages.length ? h("span", { class: "chip" }, `${report.messages.length} message${report.messages.length > 1 ? "s" : ""}`) : null, h("span", { class: "time" }, `${formatNumber(report.time_ms, 3)} ms`)),
      h(
        "div",
        { class: "body" },
        report.messages.length ? h("ul", {}, report.messages.map((m) => h("li", {}, m))) : null,
        Object.keys(report.metrics).length
          ? h(
              "table",
              { class: "data" },
              h("tbody", {}, Object.entries(report.metrics).map(([name, value]) => h("tr", {}, h("td", {}, name.replaceAll("_", " ")), h("td", { class: "num" }, formatNumber(value, 6))))),
            )
          : null,
      ),
    ),
  );
}

export function stat(label, value) {
  return h("div", { class: "stat" }, h("div", { class: "value" }, value), h("div", { class: "label" }, label));
}

// A button that shows a spinner while `action` runs.
export function actionButton(label, action, { primary = false, small = false } = {}) {
  const button = h("button", { type: "button", class: [primary ? "primary" : "", small ? "small" : ""].join(" ").trim() || null }, label);
  button.addEventListener("click", async () => {
    button.disabled = true;
    const spinner = h("span", { class: "busy", "aria-hidden": "true" });
    button.prepend(spinner);
    try {
      await action();
    } finally {
      spinner.remove();
      button.disabled = false;
    }
  });
  return button;
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
