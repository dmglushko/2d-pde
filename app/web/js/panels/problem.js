// Step 3: the problem as text: the equation's coefficients, parameters,
// one boundary condition per region and the initial conditions. Every
// edit is checked by the server a moment later; errors and what each
// coefficient depends on are shown next to the fields.

import { applyPreset, scheduleProblemCheck } from "../actions.js";
import { renameParameter, visibleCoefficients } from "../pure/case_model.js";
import { errorBox, fill, foldable, h, infoToggle, markLocal, numberInput, regionSwatch, revealFolded } from "../dom.js";
import { settings } from "../settings.js";
import { currentError, editCase, freshResult, lastResult, outdatedError, store } from "../state.js";
import { setStepStatus, stepHead } from "./head.js";

// Every edit here changes only the problem section, so the other stages'
// keys are not computed again. Typing is checked after a pause (the
// settings); a change of structure (a template, a parameter added) at
// once.
function edit(mutate, reason = "edit") {
  editCase(mutate, reason, { sections: ["problem"] });
  scheduleProblemCheck({ now: reason !== "edit" });
}

// With the check while typing off, a field is checked when it is left.
const checkOnCommit = () => {
  if (!settings.checkWhileTyping) scheduleProblemCheck({ now: true });
};

// Which coefficients each form shows (pure/case_model.js,
// visibleCoefficients): per form path, the names a template revealed in
// this visit, and whether "Show all" is on. The viewer's, not the case's.
const revealed = new Map();
const showingAll = new Set();

function revealedOf(path) {
  let names = revealed.get(path);
  if (!names) revealed.set(path, (names = new Set()));
  return names;
}

// A template was chosen for the form at `path`: its coefficients show.
function reveal(path, values) {
  for (const name of Object.keys(values)) revealedOf(path).add(name);
}

// The coefficient rows of one form: those in use, the rest hidden behind
// a "Show all coefficients" button (rows are only hidden, so a field
// cleared while typing stays where it is).
function coefficientsForm(fields, object, path, placeholder) {
  const visible = new Set(visibleCoefficients(fields, object, revealedOf(path)));
  const rows = fields.map((f) => {
    const [name, cell] = coefficientRow(f, object, path, placeholder);
    const hidden = !visible.has(f.name);
    if (hidden) for (const element of [name, cell]) element.dataset.optional = "";
    return [name, cell];
  });
  const grid = h("div", { class: "coefficients" }, rows);
  const extra = fields.length - visible.size;
  if (extra === 0) return grid;
  const toggle = h("button", { type: "button", class: "link show-all", "aria-expanded": "false" });
  const apply = (all) => {
    grid.classList.toggle("showing-all", all);
    toggle.setAttribute("aria-expanded", String(all));
    toggle.textContent = all ? "Show only the coefficients in use" : `Show all coefficients (${extra} more)`;
    if (all) showingAll.add(path);
    else showingAll.delete(path);
  };
  toggle.addEventListener("click", () => apply(!grid.classList.contains("showing-all")));
  apply(showingAll.has(path));
  return [grid, h("div", { class: "show-all-row" }, toggle)];
}

// One coefficient row: its name, the term it multiplies, a text input.
function coefficientRow(field, object, path, placeholder = "0") {
  const input = h("input", {
    type: "text",
    class: "expr",
    spellcheck: false,
    autocomplete: "off",
    value: object[field.name] ?? "",
    placeholder,
    "aria-label": field.name,
    oninput: (event) =>
      edit(() => {
        const text = event.target.value;
        if (text.trim() === "") delete object[field.name];
        else object[field.name] = text;
      }),
    onchange: checkOnCommit,
  });
  const term = h("small", {}, field.term ? `× ${field.term}` : "right side");
  return [
    h("div", { class: "name" }, field.name, term),
    h("div", { class: "cell field", dataset: { path: `${path}.${field.name}`, coef: "" } }, input, h("div", { class: "facts" })),
  ];
}

function templateSelect(label, templates, apply) {
  return h(
    "select",
    {
      "aria-label": label,
      onchange: (event) => {
        const template = templates[Number(event.target.value)];
        if (template) apply(structuredClone(template.values));
        event.target.value = "";
      },
    },
    h("option", { value: "" }, label),
    templates.map((t, i) => h("option", { value: String(i) }, `${t.label}: ${t.description}`)),
  );
}

// A complete problem for the current geometry, from the server's presets
// (equation, conditions for every region, initial conditions, parameters
// and the solve settings). The toast that follows can undo it.
function presetSelect(presets) {
  return h(
    "select",
    {
      "aria-label": "Problem preset",
      onchange: (event) => {
        const preset = presets.find((p) => p.id === event.target.value);
        event.target.value = "";
        if (preset) applyPreset(preset);
      },
    },
    h("option", { value: "" }, "Use a preset…"),
    presets.map((p) => h("option", { value: p.id, title: p.description }, p.label)),
  );
}

function equationSection(catalog, problem) {
  problem.equation ??= {};
  const help = infoToggle(catalog.expression_help, "expressions");
  const start = templateSelect("Start from…", catalog.equation_templates, (values) => {
    reveal("problem.equation", values);
    edit((c) => (c.problem.equation = values), "structure");
  });
  return foldable(
    { id: "problem.equation", title: "Equation", kind: "section", extras: [help.button, h("span", { class: "spacer" }), start], info: help.panel },
    h("div", { class: "form-line" }, catalog.equation_form),
    coefficientsForm(catalog.equation_fields, problem.equation, "problem.equation"),
  );
}

function parametersSection(problem) {
  problem.parameters ??= {};
  const entries = Object.entries(problem.parameters);
  // A name that is empty or taken is refused at the field; the case keeps
  // the old name.
  const rename = (input, oldName) => {
    let next;
    try {
      next = renameParameter(store.caseData.problem.parameters, oldName, input.value);
    } catch (error) {
      markLocal(input, error.message);
      return;
    }
    markLocal(input, null);
    if (input.value.trim() !== oldName) edit((c) => (c.problem.parameters = next), "structure");
  };
  const add = h(
    "button",
    {
      type: "button",
      class: "small",
      onclick: () =>
        edit((c) => {
          let n = 1;
          while (`p${n}` in c.problem.parameters) n++;
          c.problem.parameters[`p${n}`] = 1;
        }, "structure"),
    },
    "Add",
  );
  const row = ([name, value]) =>
    h(
      "div",
      { class: "row param-row" },
      h("input", {
        type: "text",
        class: "expr param-name",
        value: name,
        "aria-label": "Parameter name",
        onchange: (event) => rename(event.target, name),
      }),
      "=",
      numberInput({
        value,
        label: `Value of ${name}`,
        path: `problem.parameters.${name}`,
        className: "param-value",
        onNumber: (v) => edit((c) => (c.problem.parameters[name] = v)),
        onCommit: checkOnCommit,
      }),
      h(
        "button",
        { type: "button", class: "icon", "aria-label": `Remove ${name}`, onclick: () => edit((c) => delete c.problem.parameters[name], "structure") },
        "✕",
      ),
    );
  return foldable(
    { id: "problem.parameters", title: "Parameters", kind: "section", extras: [h("span", { class: "spacer" }), add] },
    h("div", { class: "field", dataset: { path: "problem.parameters" } }, entries.map(row)),
  );
}

// One region's condition: a type template, its priority and the
// coefficients.
function conditionCard(catalog, conditions, region, index, count) {
  const condition = conditions[region];
  const path = `problem.boundary_conditions.${region}`;
  const type = templateSelect("Type…", catalog.condition_templates, (values) => {
    reveal(path, values);
    edit((c) => (c.problem.boundary_conditions[region] = { ...values, priority: condition.priority ?? 0 }), "structure");
  });
  const priority = numberInput({
    value: condition.priority ?? 0,
    label: `Priority of ${region}`,
    path: `${path}.priority`,
    className: "priority-input",
    onNumber: (v) => edit(() => (condition.priority = v)),
    onEmpty: () => edit(() => delete condition.priority),
    onCommit: checkOnCommit,
  });
  return foldable(
    {
      id: `problem.condition.${region}`,
      title: [regionSwatch(index), " ", region],
      folded: count > 4,
      extras: [h("span", { class: "chip", dataset: { kindOf: region } }, "…"), h("span", { class: "spacer" })],
      attributes: { class: "field", dataset: { path } },
    },
    h("div", { class: "row condition-line" }, type, h("label", { class: "row inline-label" }, "priority", priority)),
    coefficientsForm(catalog.condition_fields, condition, path),
  );
}

function conditionsSection(catalog, problem) {
  const conditions = (problem.boundary_conditions ??= {});
  const regions = Object.keys(conditions);
  const order = (lastResult("geometry")?.regions ?? []).map((r) => r.name);
  const index = (name) => {
    const i = order.indexOf(name);
    return i < 0 ? order.length + regions.indexOf(name) : i;
  };
  const help = infoToggle(catalog.condition_note, "boundary conditions");
  return foldable(
    { id: "problem.conditions", title: "Boundary conditions", kind: "section", extras: [help.button], info: help.panel },
    h("div", { class: "form-line" }, catalog.condition_form),
    regions.length === 0 ? h("p", { class: "note" }, "The regions appear here once the geometry is read.") : null,
    regions.sort((a, b) => index(a) - index(b)).map((region) => conditionCard(catalog, conditions, region, index(region), regions.length)),
  );
}

function initialSection(catalog, problem) {
  problem.initial ??= {};
  const help = infoToggle(catalog.initial_note, "initial conditions");
  const rows = catalog.initial_fields.map((f) => coefficientRow({ ...f, term: "" }, problem.initial, "problem.initial", "not given"));
  return foldable(
    { id: "problem.initial", title: "Initial conditions", kind: "section", extras: [help.button], info: help.panel },
    h("div", { class: "coefficients" }, rows),
  );
}

function factChips(facts) {
  if (!facts || facts.zero) return [];
  const depends = ["x", "y", "t"].filter((v) => facts[`depends_on_${v}`]);
  return depends.length ? [h("span", { class: "chip" }, `depends on ${depends.join(", ")}`)] : [h("span", { class: "chip" }, "constant")];
}

// The status line: checking, no errors, or how many.
function checkStatus(check, fresh, error) {
  if (error) return { status: "failed", text: "The check failed" };
  if (store.busy.has("problem") || !fresh) return { status: "busy", text: "Checking…" };
  if (!check.ok) return { status: "failed", text: `${check.errors.length} error${check.errors.length > 1 ? "s" : ""}` };
  return { status: "done", text: "No errors" };
}

function summaryChips(check) {
  const required = check.required_initial;
  return h(
    "div",
    { class: "row chips" },
    h("span", { class: "chip" }, `Time order ${check.time_order}`),
    check.coefficients_depend_on_t ? h("span", { class: "chip" }, "coefficients depend on t") : null,
    check.data_depends_on_t ? h("span", { class: "chip" }, "data depend on t") : null,
    h("span", { class: "chip" }, required.length ? `needs initial ${required.join(" and ")}` : "stationary"),
  );
}

export const problemPanel = {
  build(container) {
    const catalog = store.catalog.problem;
    const problem = (store.caseData.problem ??= {});
    const head = stepHead("problem", catalog.presets?.length ? presetSelect(catalog.presets) : null);
    fill(
      container,
      head,
      h("div", { id: "problem-summary", class: "result" }),
      h(
        "div",
        { class: "settings-part" },
        equationSection(catalog, problem),
        parametersSection(problem),
        conditionsSection(catalog, problem),
        initialSection(catalog, problem),
      ),
    );
    container.head = head;
    this.update(container, true);
  },

  update(container, force = false) {
    const summary = container.querySelector("#problem-summary");
    if (!summary) return;
    const fresh = freshResult("problem");
    const check = fresh ?? lastResult("problem");
    const error = currentError("problem");
    const old = outdatedError("problem");
    if (!force && summary.shown && summary.shown.check === check && summary.shown.fresh === fresh && summary.shown.error === error && summary.shown.old === old) return;
    summary.shown = { check, fresh, error, old };
    // A check of an earlier problem is shown muted, never as current.
    container.classList.toggle("checking", !fresh);

    // Per field: its facts and its error.
    const errors = new Map((check?.errors ?? []).map((e) => [e.path, e.message]));
    for (const cell of container.querySelectorAll("[data-path]:not(input)")) {
      const path = cell.dataset.path;
      cell.classList.toggle("invalid", errors.has(path));
      for (const old of cell.querySelectorAll(":scope > [data-error]")) old.remove();
      if (errors.has(path)) {
        cell.append(h("div", { class: "error-text", "data-error": "" }, errors.get(path)));
        if (fresh) revealFolded(cell);
      }
      const facts = cell.querySelector(":scope > .facts");
      if (facts) fill(facts, ...(errors.has(path) ? [] : factChips(check?.facts?.[path])));
    }
    for (const chip of container.querySelectorAll("[data-kind-of]")) {
      const conditions = check?.conditions ?? {};
      const kind = Object.hasOwn(conditions, chip.dataset.kindOf) ? conditions[chip.dataset.kindOf] : null;
      chip.textContent = kind ? `${kind.kind}${kind.oblique ? ", oblique" : ""}` : errors.has(`problem.boundary_conditions.${chip.dataset.kindOf}`) ? "not supported" : "…";
      chip.classList.toggle("bad", !kind && errors.has(`problem.boundary_conditions.${chip.dataset.kindOf}`));
    }

    setStepStatus(container.head, "problem", checkStatus(check, fresh, error));
    const children = [];
    if (error) children.push(errorBox(error));
    if (old) children.push(errorBox(old, { outdated: true }));
    if (fresh && !error) {
      children.push(summaryChips(check));
      // Errors without a field of their own (the whole section).
      const unplaced = check.errors.filter((e) => !container.querySelector(`[data-path="${CSS.escape(e.path)}"]:not(input)`));
      for (const e of unplaced) children.push(h("div", { class: "alert" }, `${e.path}: ${e.message}`));
    }
    fill(summary, ...children);
  },
};
