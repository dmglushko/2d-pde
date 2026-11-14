// Step 3: the problem as text: the equation's coefficients, parameters,
// one boundary condition per region and the initial conditions. Every
// edit is checked by the server a moment later; errors and what each
// coefficient depends on are shown next to the fields.

import { scheduleProblemCheck } from "../actions.js";
import { regionToken } from "../core.js";
import { errorBox, h, fill } from "../dom.js";
import { editCase, freshResult, lastResult, store } from "../state.js";

function edit(mutate, reason = "edit") {
  editCase(mutate, reason);
  scheduleProblemCheck();
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
  });
  return [
    h("div", { class: "name" }, field.name, field.term ? h("small", {}, `× ${field.term}`) : h("small", {}, "right side")),
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

function equationSection(catalog, problem) {
  problem.equation ??= {};
  return [
    h(
      "div",
      { class: "row", style: "margin-top:16px" },
      h("h3", { style: "margin:0" }, "Equation"),
      h("span", { class: "spacer" }),
      templateSelect("Start from…", catalog.equation_templates, (values) => edit((c) => (c.problem.equation = values), "structure")),
    ),
    h("div", { class: "form-line" }, catalog.equation_form),
    h("div", { class: "coefficients", style: "margin-top:8px" }, catalog.equation_fields.map((f) => coefficientRow(f, problem.equation, "problem.equation"))),
  ];
}

function parametersSection(problem) {
  problem.parameters ??= {};
  const entries = Object.entries(problem.parameters);
  const rename = (oldName, newName) =>
    edit((c) => {
      const next = {};
      for (const [k, v] of Object.entries(c.problem.parameters)) next[k === oldName ? newName : k] = v;
      c.problem.parameters = next;
    }, "structure");
  return [
    h("div", { class: "row", style: "margin-top:16px" }, h("h3", { style: "margin:0" }, "Parameters"), h("span", { class: "spacer" }), h("button", {
      type: "button",
      class: "small",
      onclick: () =>
        edit((c) => {
          let n = 1;
          while (`p${n}` in c.problem.parameters) n++;
          c.problem.parameters[`p${n}`] = 1;
        }, "structure"),
    }, "Add")),
    h("p", { class: "note" }, "Named constants used in the expressions, such as k = 0.1."),
    h(
      "div",
      { class: "field", dataset: { path: "problem.parameters" } },
      entries.map(([name, value]) =>
        h(
          "div",
          { class: "row", style: "margin:4px 0" },
          h("input", { type: "text", class: "expr", value: name, "aria-label": "Parameter name", style: "width:120px", onchange: (event) => event.target.value.trim() && rename(name, event.target.value.trim()) }),
          "=",
          h("input", {
            type: "number",
            step: "any",
            value: String(value),
            "aria-label": `Value of ${name}`,
            style: "width:120px",
            oninput: (event) => event.target.value.trim() !== "" && edit((c) => (c.problem.parameters[name] = Number(event.target.value))),
          }),
          h("button", { type: "button", class: "icon", "aria-label": `Remove ${name}`, onclick: () => edit((c) => delete c.problem.parameters[name], "structure") }, "✕"),
        ),
      ),
    ),
  ];
}

function conditionsSection(catalog, problem) {
  const conditions = (problem.boundary_conditions ??= {});
  const regions = Object.keys(conditions);
  const order = (lastResult("geometry")?.regions ?? []).map((r) => r.name);
  const index = (name) => {
    const i = order.indexOf(name);
    return i < 0 ? order.length + regions.indexOf(name) : i;
  };
  return [
    h("h3", {}, "Boundary conditions"),
    h("div", { class: "form-line" }, catalog.condition_form),
    h("p", { class: "note" }, catalog.condition_note),
    regions.length === 0 ? h("p", { class: "note" }, "The regions appear here once the geometry is read.") : null,
    regions
      .sort((a, b) => index(a) - index(b))
      .map((region) => {
        const condition = conditions[region];
        const path = `problem.boundary_conditions.${region}`;
        return h(
          "details",
          { class: "card field", open: regions.length <= 4, dataset: { path } },
          h(
            "summary",
            { class: "card-head", style: "cursor:pointer" },
            h("span", { class: "swatch", style: `background:var(${regionToken(index(region))})` }),
            h("span", { class: "title" }, region),
            h("span", { class: "chip", dataset: { kindOf: region } }, "…"),
            h("span", { class: "spacer" }),
          ),
          h(
            "div",
            { class: "row", style: "margin:8px 0" },
            templateSelect("Type…", catalog.condition_templates, (values) => edit((c) => (c.problem.boundary_conditions[region] = { ...values, priority: condition.priority ?? 0 }), "structure")),
            h("label", { class: "row", style: "gap:4px" }, "priority", h("input", {
              type: "number",
              step: "1",
              value: String(condition.priority ?? 0),
              style: "width:70px",
              "aria-label": `Priority of ${region}`,
              oninput: (event) => edit(() => (condition.priority = Number(event.target.value || 0))),
            })),
          ),
          h("div", { class: "coefficients" }, catalog.condition_fields.map((f) => coefficientRow(f, condition, path))),
        );
      }),
  ];
}

function initialSection(catalog, problem) {
  problem.initial ??= {};
  return [
    h("h3", {}, "Initial conditions"),
    h("p", { class: "note" }, catalog.initial_note),
    h("div", { class: "coefficients" }, catalog.initial_fields.map((f) => coefficientRow({ ...f, term: "" }, problem.initial, "problem.initial", "not given"))),
  ];
}

function factChips(facts) {
  if (!facts || facts.zero) return [];
  const depends = ["x", "y", "t"].filter((v) => facts[`depends_on_${v}`]);
  return depends.length ? [h("span", { class: "chip" }, `depends on ${depends.join(", ")}`)] : [h("span", { class: "chip" }, "constant")];
}

export const problemPanel = {
  build(container) {
    const catalog = store.catalog.problem;
    const problem = (store.caseData.problem ??= {});
    fill(container,
      h("h2", {}, "Problem"),
      h("p", { class: "lead" }, "One linear scalar equation in u(x, y, t). Every field takes an expression; an empty field is zero."),
      h("details", { class: "report" }, h("summary", {}, "What an expression may contain"), h("div", { class: "body note" }, catalog.expression_help)),
      h("div", { id: "problem-summary", class: "result" }),
      equationSection(catalog, problem),
      parametersSection(problem),
      conditionsSection(catalog, problem),
      initialSection(catalog, problem),
    );
    this.update(container, true);
  },

  update(container, force = false) {
    const summary = container.querySelector("#problem-summary");
    if (!summary) return;
    const check = freshResult("problem") ?? lastResult("problem");
    const error = store.errors.problem;
    if (!force && summary.shown && summary.shown.check === check && summary.shown.error === error) return;
    summary.shown = { check, error };

    // Per field: its facts and its error.
    const errors = new Map((check?.errors ?? []).map((e) => [e.path, e.message]));
    for (const cell of container.querySelectorAll("[data-path]")) {
      const path = cell.dataset.path;
      cell.classList.toggle("invalid", errors.has(path));
      for (const old of cell.querySelectorAll(":scope > [data-error]")) old.remove();
      if (errors.has(path)) cell.append(h("div", { class: "error-text", "data-error": "" }, errors.get(path)));
      const facts = cell.querySelector(":scope > .facts");
      if (facts) fill(facts, ...(errors.has(path) ? [] : factChips(check?.facts?.[path])));
    }
    for (const chip of container.querySelectorAll("[data-kind-of]")) {
      const kind = check?.conditions?.[chip.dataset.kindOf];
      chip.textContent = kind ? `${kind.kind}${kind.oblique ? ", oblique" : ""}` : errors.has(`problem.boundary_conditions.${chip.dataset.kindOf}`) ? "not supported" : "…";
      chip.classList.toggle("bad", !kind && errors.has(`problem.boundary_conditions.${chip.dataset.kindOf}`));
    }

    const children = [];
    if (error) children.push(errorBox(error));
    if (check) {
      const required = check.required_initial;
      children.push(
        h(
          "div",
          { class: "row" },
          check.ok ? h("span", { class: "chip good" }, "✓ no errors") : h("span", { class: "chip bad" }, `✕ ${check.errors.length} error${check.errors.length > 1 ? "s" : ""}`),
          h("span", { class: "chip" }, `time order ${check.time_order}`),
          check.coefficients_depend_on_t ? h("span", { class: "chip" }, "coefficients depend on t") : null,
          check.data_depends_on_t ? h("span", { class: "chip" }, "data depend on t") : null,
          h("span", { class: "chip" }, required.length ? `needs initial ${required.join(" and ")}` : "stationary: no initial conditions"),
        ),
      );
      // Errors without a field of their own (the whole section).
      const unplaced = check.errors.filter((e) => !container.querySelector(`[data-path="${CSS.escape(e.path)}"]`));
      for (const e of unplaced) children.push(h("div", { class: "alert" }, `${e.path}: ${e.message}`));
    }
    fill(summary, ...children);
  },
};
