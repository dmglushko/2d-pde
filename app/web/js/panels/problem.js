// Step 3: the problem as text: the equation, parameters, one boundary
// condition per region and the initial conditions. The equation and each
// condition are one line ("u_t - div(k*grad(u)) = 0", read by the
// library) or, switched, their coefficients. Every edit is checked by the
// server at once; errors, the coefficients a line means and what each
// coefficient depends on are shown under the fields.

import { scheduleProblemCheck, toast } from "../actions.js";
import { isLine, renameParameter, visibleCoefficients } from "../pure/case_model.js";
import { errorBox, fill, foldable, h, infoToggle, markLocal, numberInput, regionSwatch, revealFolded } from "../dom.js";
import { settings } from "../settings.js";
import { currentError, editCase, freshResult, lastResult, outdatedError, store } from "../state.js";
import { timeOrderWords } from "./assemble.js";
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

// While a line is being typed, its error waits for a pause (a line is
// wrong after almost every key until it is finished); what it meant last
// stays meanwhile. A check that takes longer than SLOW_MS is said.
const ERROR_PAUSE_MS = 700;
const SLOW_MS = 400;
let typedAt = 0;
let typedPath = null;
let pauseTimer = null;
let panelElement = null;
function typed(path) {
  typedAt = performance.now();
  typedPath = path;
  clearTimeout(pauseTimer);
  pauseTimer = setTimeout(() => panelElement && problemPanel.update(panelElement, true), ERROR_PAUSE_MS + 10);
}
const typingIn = (path) => path === typedPath && performance.now() - typedAt < ERROR_PAUSE_MS;

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
    oninput: (event) => {
      typed(`${path}.${field.name}`);
      edit(() => {
        const text = event.target.value;
        if (text.trim() === "") delete object[field.name];
        else object[field.name] = text;
      });
    },
    onchange: checkOnCommit,
  });
  const term = h("small", {}, field.term ? `× ${field.term}` : "right side");
  return [
    h("div", { class: "name" }, field.name, term),
    h("div", { class: "cell field", dataset: { path: `${path}.${field.name}`, coef: "" } }, input, h("div", { class: "facts" })),
  ];
}

// The symbols a keyboard lacks, inserted at the caret of `input`.
const SYMBOLS = [
  ["∇", "nabla: ∇u is grad(u)"],
  ["·", "dot: ∇·F is div(F), a·b the dot product"],
  ["²", "∇²u is lap(u)"],
  ["Δ", "Δu is lap(u)"],
];

function symbolBar(input) {
  const insert = (symbol) => {
    const start = input.selectionStart ?? input.value.length;
    const end = input.selectionEnd ?? start;
    input.setRangeText(symbol, start, end, "end");
    input.focus();
    input.dispatchEvent(new Event("input", { bubbles: true }));
  };
  return h(
    "div",
    { class: "symbol-bar", role: "group", "aria-label": "Insert a symbol" },
    SYMBOLS.map(([symbol, title]) =>
      h(
        "button",
        // The field keeps the focus (and its caret) while one is pressed.
        { type: "button", class: "symbol", title, "aria-label": `Insert ${symbol}`, onmousedown: (event) => event.preventDefault(), onclick: () => insert(symbol) },
        symbol,
      ),
    ),
  );
}

// An equation or condition as one line: the text, the symbols, and under
// it (filled by update) the coefficients the library read from it, or
// its error. `object` is the case's object, `path` its case path.
function lineField(object, path, { label, placeholder }) {
  const input = h("input", {
    type: "text",
    class: "expr line-input",
    spellcheck: false,
    autocomplete: "off",
    value: object.text ?? "",
    placeholder,
    "aria-label": label,
    oninput: (event) => {
      typed(`${path}.text`);
      edit(() => (object.text = event.target.value));
    },
    onchange: checkOnCommit,
  });
  return h(
    "div",
    { class: "line-field field", dataset: { path: `${path}.text`, line: path } },
    h("div", { class: "line-row" }, input, symbolBar(input)),
    h("div", { class: "derived", dataset: { derivedOf: path }, "aria-live": "polite" }),
  );
}

// Switches the equation or a condition at `path` between its line and
// its coefficients, from both forms of the last check (the library reads
// the line and prints the coefficients). `replace(c, value)` puts the new
// form into the case.
function switchForm(path, toLine, replace) {
  const form = freshResult("problem")?.forms?.[path];
  if (!form) {
    toast(toLine ? "It can be written as a line once the check finds no error in it." : "The coefficients are known once the check finds no error in the line.");
    return;
  }
  if (toLine && typeof form.text !== "string") {
    toast(form.reason ?? "It cannot be written as a line.");
    return;
  }
  edit((c) => replace(c, toLine ? { text: form.text } : { ...form.coefficients }), "structure");
}

function switchButton(path, line, replace) {
  return h(
    "button",
    {
      type: "button",
      class: "link small switch-form",
      title: line ? "Edit the coefficients of the form instead of the line" : "Write it as one line, as on paper",
      onclick: () => switchForm(path, !line, replace),
    },
    line ? "Coefficients" : "As a line",
  );
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
function equationSection(catalog, problem) {
  problem.equation ??= {};
  const line = isLine(problem.equation);
  const help = infoToggle(line ? `${catalog.line_help} ${catalog.expression_help}` : catalog.expression_help, line ? "writing the equation" : "expressions");
  const start = templateSelect("Start from…", catalog.equation_templates, (values) => {
    reveal("problem.equation", values);
    edit((c) => (c.problem.equation = values), "structure");
  });
  const toggle = switchButton("problem.equation", line, (c, value) => (c.problem.equation = value));
  return foldable(
    { id: "problem.equation", title: "Equation", kind: "section", extras: [help.button, h("span", { class: "spacer" }), toggle, start], info: help.panel },
    line
      ? lineField(problem.equation, "problem.equation", { label: "Equation", placeholder: "u_t - div(k*grad(u)) = 0" })
      : [h("div", { class: "form-line" }, catalog.equation_form), coefficientsForm(catalog.equation_fields, problem.equation, "problem.equation")],
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
  const line = isLine(condition);
  const type = templateSelect("Type…", catalog.condition_templates, (values) => {
    reveal(path, values);
    edit((c) => (c.problem.boundary_conditions[region] = { ...values, priority: condition.priority ?? 0 }), "structure");
  });
  const toggle = switchButton(path, line, (c, value) => {
    const old = c.problem.boundary_conditions[region];
    c.problem.boundary_conditions[region] = old.priority === undefined ? value : { ...value, priority: old.priority };
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
      extras: [h("span", { class: "chip", dataset: { kindOf: region } }, "…"), h("span", { class: "spacer" }), toggle],
      attributes: { class: "field", dataset: { path } },
    },
    line ? lineField(condition, path, { label: `Condition on ${region}`, placeholder: "u = 0" }) : null,
    h("div", { class: "row condition-line" }, type, h("label", { class: "row inline-label" }, "priority", priority)),
    line ? null : coefficientsForm(catalog.condition_fields, condition, path),
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
  const help = infoToggle(`${catalog.condition_note} Written as a line: ${catalog.condition_line_note}`, "boundary conditions");
  const anyCoefficients = regions.some((region) => !isLine(conditions[region]));
  return foldable(
    { id: "problem.conditions", title: "Boundary conditions", kind: "section", extras: [help.button], info: help.panel },
    anyCoefficients ? h("div", { class: "form-line" }, catalog.condition_form) : null,
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

// How long the running check has run (ms), or null.
function checkingFor() {
  const since = store.busySince.problem;
  return store.busy.has("problem") && since !== undefined ? performance.now() - since : null;
}

// The status line: checking, no errors, or how many. Until a check has
// run SLOW_MS (also while the next one is only queued), the words of the
// last finished one stay (no flicker while typing).
function checkStatus(check, fresh, error, lastDone) {
  if (error) return { status: "failed", text: "The check failed" };
  if (!fresh || store.busy.has("problem")) {
    const running = checkingFor();
    if (lastDone && (running === null || running < SLOW_MS)) return lastDone;
    return { status: "busy", text: "Checking…" };
  }
  if (!check.ok) return { status: "failed", text: `${check.errors.length} error${check.errors.length > 1 ? "s" : ""}` };
  return { status: "done", text: "No errors" };
}

// The problem's facts as four chips, always the same four (each says yes
// or no), so the fields below do not move when an edit changes a fact.
function summaryChips(check) {
  const required = check.required_initial;
  return h(
    "div",
    { class: "row chips summary-chips" },
    h("span", { class: "chip" }, `Time order ${timeOrderWords(check.time_order)}`),
    h("span", { class: "chip", "data-on": String(check.coefficients_depend_on_t) }, check.coefficients_depend_on_t ? "coefficients depend on t" : "coefficients constant in t"),
    h("span", { class: "chip", "data-on": String(check.data_depends_on_t) }, check.data_depends_on_t ? "data depend on t" : "data constant in t"),
    h("span", { class: "chip", "data-on": String(required.length > 0) }, required.length ? `needs initial ${required.join(" and ")}` : "no initial values"),
  );
}

// The coefficients a line means, as "a_xx = -k" chips in the catalog's
// order (powers as ^, as typed; the library prints **).
function derivedChips(fields, coefficients) {
  const named = fields.filter((f) => coefficients?.[f.name] !== undefined);
  if (named.length === 0) return [h("span", { class: "note" }, "no coefficient")];
  return [
    h("span", { class: "derived-label" }, "means"),
    named.map((f) => h("span", { class: "chip coefficient", title: f.term ? `× ${f.term}` : "right side" }, `${f.name} = ${coefficients[f.name].replaceAll("**", "^")}`)),
  ];
}

export const problemPanel = {
  build(container) {
    panelElement = container;
    const catalog = store.catalog.problem;
    const problem = (store.caseData.problem ??= {});
    const head = stepHead("problem");
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
    const catalog = store.catalog.problem;
    const fresh = freshResult("problem");
    const check = fresh ?? lastResult("problem");
    const error = currentError("problem");
    const old = outdatedError("problem");
    const running = checkingFor();
    const quick = !fresh && running !== null && running < SLOW_MS;
    if (quick) {
      // Said once it is slow.
      clearTimeout(summary.slowTimer);
      summary.slowTimer = setTimeout(() => this.update(container), SLOW_MS - running + 10);
    }
    if (!force && summary.shown && summary.shown.check === check && summary.shown.fresh === fresh && summary.shown.error === error && summary.shown.old === old && summary.shown.quick === quick) return;
    summary.shown = { check, fresh, error, old, quick };
    // A check of an earlier problem is shown muted, never as current; a
    // quick one is not worth a flicker.
    container.classList.toggle("checking", !fresh && !quick);

    // Per field: its facts and its error. The error of a line being typed
    // waits for a pause.
    const errors = new Map((check?.errors ?? []).map((e) => [e.path, e.message]));
    for (const cell of container.querySelectorAll("[data-path]:not(input)")) {
      const path = cell.dataset.path;
      const waiting = typingIn(path) && errors.has(path);
      const shownError = errors.has(path) && !waiting;
      cell.classList.toggle("invalid", shownError);
      for (const old of cell.querySelectorAll(":scope > [data-error]")) old.remove();
      if (shownError) {
        cell.append(h("div", { class: "error-text", "data-error": "" }, errors.get(path)));
        if (fresh) revealFolded(cell);
      }
      const facts = cell.querySelector(":scope > .facts");
      if (facts) fill(facts, ...(errors.has(path) ? [] : factChips(check?.facts?.[path])));
      // A line: what it means, from the last check that read it (kept,
      // muted, while it has an error or is being typed).
      const derived = cell.querySelector(":scope > .derived");
      if (derived) {
        const form = check?.forms?.[cell.dataset.line];
        const fields = cell.dataset.line === "problem.equation" ? catalog.equation_fields : catalog.condition_fields;
        if (form) fill(derived, ...derivedChips(fields, form.coefficients));
        derived.hidden = shownError || derived.childElementCount === 0;
        derived.classList.toggle("stale", !form);
      }
    }
    for (const chip of container.querySelectorAll("[data-kind-of]")) {
      const conditions = check?.conditions ?? {};
      const kind = Object.hasOwn(conditions, chip.dataset.kindOf) ? conditions[chip.dataset.kindOf] : null;
      chip.textContent = kind ? `${kind.kind}${kind.oblique ? ", oblique" : ""}` : errors.has(`problem.boundary_conditions.${chip.dataset.kindOf}`) ? "not supported" : "…";
      chip.classList.toggle("bad", !kind && errors.has(`problem.boundary_conditions.${chip.dataset.kindOf}`));
    }

    const status = checkStatus(check, fresh, error, container.head.lastDoneStatus);
    if (status.status !== "busy") container.head.lastDoneStatus = status;
    setStepStatus(container.head, "problem", status);
    // While a check runs, the summary of the last one stays (fields below
    // it do not move on every key).
    if (!fresh && !error && !old && summary.childElementCount > 0) {
      summary.classList.add("stale");
      return;
    }
    summary.classList.remove("stale");
    const children = [];
    if (error) children.push(errorBox(error));
    if (old) children.push(errorBox(old, { outdated: true }));
    if (fresh && !error) {
      // An equation that cannot be read has no facts: those of the last
      // one that could stay, muted, rather than "time order 0".
      const unreadable = check.errors.some((e) => e.path === "problem.equation" || e.path.startsWith("problem.equation."));
      if (!unreadable) summary.lastReadable = check;
      const facts = unreadable ? summary.lastReadable : check;
      if (facts) {
        const chips = summaryChips(facts);
        if (facts !== check) {
          chips.classList.add("stale");
          chips.title = "From the last equation that could be read";
        }
        children.push(chips);
      }
      // Errors without a field of their own (the whole section).
      const unplaced = check.errors.filter((e) => !container.querySelector(`[data-path="${CSS.escape(e.path)}"]:not(input)`));
      for (const e of unplaced) children.push(h("div", { class: "alert" }, `${e.path}: ${e.message}`));
    }
    fill(summary, ...children);
  },
};
