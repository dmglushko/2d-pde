// Step 4: the problem combined with the mesh (MeshedProblem) and assembled
// at one time t; fields evaluated on the mesh. The head holds Assemble
// and the status, the result follows, then the settings and the field
// form.

import { downloadExport } from "../api.js";
import { runWithPrerequisites, toast } from "../actions.js";
import { formatNumber, parseColor, rampTable } from "../core.js";
import { actionButton, errorBox, exportButton, fill, foldable, h, infoToggle, labelWithInfo, markError, numberHandler, reportsView, stat } from "../dom.js";
import { currentError, editCase, freshResult, lastResult, openStep, outdatedError, store } from "../state.js";
import { setStepStatus, stepHead } from "./head.js";

const assemble = () => runWithPrerequisites("assemble");
const openProblem = () => openStep("problem");

const FIELD_KINDS = [
  ["expression", "an expression"],
  ["initial_u", "initial u"],
  ["initial_u_t", "initial u_t"],
  ["dirichlet", "Dirichlet values"],
  ["right_hand_side", "right-hand side b"],
];

// The field form keeps its values for the session (not in the case).
const fieldForm = { kind: "expression", text: "sin(pi*x)*y", t: "" };

async function showFieldFromForm() {
  const field = { kind: fieldForm.kind };
  if (fieldForm.kind === "expression") field.text = fieldForm.text;
  if (String(fieldForm.t).trim() !== "") {
    field.t = Number(fieldForm.t);
    if (!Number.isFinite(field.t)) return toast("t is not a number.");
  }
  // The picture colours by the field once it arrives (actions.js).
  await runWithPrerequisites("field", field);
}

function fieldSection() {
  const text = h("input", {
    type: "text",
    class: "expr",
    value: fieldForm.text,
    "aria-label": "Expression",
    oninput: (e) => (fieldForm.text = e.target.value),
  });
  const textRow = h("div", { class: "field", dataset: { path: "field.text" } }, h("div", { class: "label" }, "Expression in x, y, t and the parameters"), text);
  textRow.hidden = fieldForm.kind !== "expression";
  const kind = h(
    "select",
    {
      "aria-label": "Field",
      onchange: (e) => {
        fieldForm.kind = e.target.value;
        textRow.hidden = fieldForm.kind !== "expression";
      },
    },
    FIELD_KINDS.map(([value, label]) => h("option", { value, selected: value === fieldForm.kind }, label)),
  );
  const t = h("input", {
    type: "number",
    step: "any",
    class: "time-input",
    value: fieldForm.t,
    placeholder: "assembly t",
    "aria-label": "t",
    oninput: (e) => (fieldForm.t = e.target.value),
  });
  const info = infoToggle(
    "Values at the mesh nodes, drawn with linear interpolation. Dirichlet values and b exist only at some nodes and are drawn as dots.",
    "fields",
  );
  return foldable(
    { id: "assemble.field", title: "Show a field on the mesh", kind: "section", extras: [info.button], info: info.panel, attributes: { id: "field-form" } },
    h("div", { class: "row" }, kind, h("label", { class: "row inline-label" }, "t =", t), actionButton("Show", showFieldFromForm)),
    textRow,
    h("div", { id: "field-error" }),
  );
}

function drawSpy(canvas, sparsity) {
  const bins = sparsity.bins;
  canvas.width = bins;
  canvas.height = bins;
  const context = canvas.getContext("2d");
  const image = context.createImageData(bins, bins);
  const style = getComputedStyle(document.documentElement);
  const color = (name) => parseColor(style.getPropertyValue(name));
  const table = rampTable([color("--seq-low"), color("--seq-mid"), color("--seq-high")]);
  const surface = color("--surface");
  const max = Math.max(1, ...sparsity.counts);
  for (let i = 0; i < bins * bins; i++) {
    const count = sparsity.counts[i];
    // Dim for few entries, bright for many, on a log scale.
    const k = Math.round((255 * Math.log1p(count)) / Math.log1p(max));
    const rgb = count === 0 ? surface : table.subarray(3 * k, 3 * k + 3);
    image.data.set([rgb[0], rgb[1], rgb[2], 255], 4 * i);
  }
  context.putImageData(image, 0, 0);
}

// Symmetry from the formulas (every t) or from the numbers (this t only).
function symmetryText(op) {
  if (op.always_symmetric) return "yes";
  if (op.symmetric_now) return "at this t";
  return "no";
}

const OPERATOR_NAMES = { mass: "M2 (u_tt)", damping: "B = M1 − D (u_t)", stiffness: "A (u)" };

function operatorRow(name, op) {
  const label = OPERATOR_NAMES[name] ?? name;
  if (op.always_zero) return h("tr", {}, h("td", {}, label), h("td", { colspan: 3, class: "muted-cell" }, "zero"));
  return h(
    "tr",
    {},
    h("td", {}, label, op.zero_now ? h("span", { class: "chip" }, "zero at this t") : null),
    h("td", {}, op.depends_on_t ? "yes" : "no"),
    h("td", {}, symmetryText(op)),
    h("td", { class: "num" }, formatNumber(op.free_nonzeros + op.dirichlet_nonzeros)),
  );
}

function operatorsTable(operators) {
  const head = h("tr", {}, h("th", {}, "Operator"), h("th", {}, "Changes in t"), h("th", {}, "Symmetric"), h("th", { class: "num" }, "Entries"));
  const rows = Object.entries(operators).map(([name, op]) => operatorRow(name, op));
  return h("table", { class: "data" }, h("thead", {}, head), h("tbody", {}, rows));
}

// "1" when assembly's time order is the problem's, else both.
export function timeOrderText(assembled, written) {
  return assembled === written ? String(assembled) : `${assembled} (problem: ${written})`;
}

const EXPORTS = [
  ["stiffness.mtx", "A"],
  ["damping.mtx", "B"],
  ["mass.mtx", "M2"],
  ["rhs.mtx", "b"],
];

function resultView(result, fresh) {
  const meshed = result.meshed_problem;
  const assembly = result.assembly;
  const spy = h("canvas", { class: "spy", "aria-label": "Where the stiffness matrix has entries" });
  if (assembly.sparsity) drawSpy(spy, assembly.sparsity);
  const operatorsInfo = infoToggle(
    `Symmetric "yes" holds at every t (from the formulas), "at this t" only for these numbers. The system is M2 U'' + B U' + A U = b in the unknowns U. The load ${assembly.load_depends_on_t ? "changes" : "does not change"} with t; b ${assembly.right_hand_side_depends_on_t ? "changes" : "does not change"} with t.`,
    "the operators",
  );
  const save = (kind) => () => downloadExport(kind, store.caseData).catch((e) => toast(e.message));
  return [
    h(
      "div",
      { class: "stats" },
      stat("Unknowns", formatNumber(meshed.free_nodes)),
      stat("Dirichlet nodes", formatNumber(meshed.dirichlet_nodes.length)),
      stat("Time order", timeOrderText(assembly.time_order, meshed.time_order)),
      stat("‖b‖₂, right-hand side", formatNumber(assembly.right_hand_side_norm), "The Euclidean norm of the assembled right-hand side over the unknowns."),
    ),
    h("div", { class: "section-head" }, h("h3", {}, `Operators at t = ${formatNumber(assembly.t)}`), operatorsInfo.button),
    operatorsInfo.panel,
    operatorsTable(assembly.operators),
    assembly.sparsity
      ? foldable(
          { id: "assemble.sparsity", title: `Entries of A (${formatNumber(assembly.sparsity.nonzeros)})`, kind: "section", folded: true },
          spy,
        )
      : null,
    h(
      "div",
      { class: "row actions-row" },
      EXPORTS.map(([kind, name]) =>
        exportButton(`Export ${name} .mtx`, save(kind), {
          fresh,
          staleReason: "The system shown is outdated: assemble again to export it.",
          about: "Matrix Market, the free part (unknowns × unknowns, 1-based), at this t.",
        }),
      ),
    ),
    foldable({ id: "assemble.reports", title: "Reports", kind: "section", folded: true }, reportsView([meshed.report, assembly.report])),
  ];
}

function settingsPart(assembly) {
  const setT = numberHandler((t) => editCase((c) => (c.assembly.t = t), "edit", { sections: ["assembly"] }));
  const order = h(
    "select",
    { "aria-label": "Time terms to build", onchange: (e) => editCase((c) => (c.assembly.max_time_order = Number(e.target.value))) },
    [
      [2, "all (M2, B and A)"],
      [1, "up to u_t (B and A)"],
      [0, "none: A and the load only"],
    ].map(([value, label]) => h("option", { value: String(value), selected: (assembly.max_time_order ?? 2) === value }, label)),
  );
  const [orderLabel, orderInfo] = labelWithInfo(
    "Time terms to build",
    "Left-out operators count as zero; with none, only the boundary values themselves are evaluated, not their time derivatives (a static solve assembles like this).",
  );
  return foldable(
    { id: "assemble.settings", title: "Time and terms" },
    h(
      "div",
      { class: "field", dataset: { path: "assembly.t" } },
      h("div", { class: "label" }, "Time t"),
      h("div", { class: "inputs" }, h("input", { type: "number", step: "any", value: String(assembly.t ?? 0), "aria-label": "Time t", oninput: setT })),
    ),
    h("div", { class: "field", dataset: { path: "assembly.max_time_order" } }, orderLabel, h("div", { class: "inputs" }, order), orderInfo),
  );
}

export const assemblePanel = {
  build(container) {
    const assembly = (store.caseData.assembly ??= { t: 0 });
    const head = stepHead("assemble", actionButton("Assemble", assemble, { primary: true }));
    fill(
      container,
      head,
      h("div", { class: "result", id: "assemble-result" }),
      h("div", { class: "settings-part" }, h("h3", { class: "part-title" }, "Settings"), settingsPart(assembly), fieldSection()),
    );
    container.head = head;
    this.update(container);
  },

  update(container) {
    const slot = container.querySelector("#assemble-result");
    if (!slot) return;
    const error = currentError("assemble");
    const old = outdatedError("assemble");
    const fresh = freshResult("assemble");
    const result = fresh ?? lastResult("assemble");
    const busy = store.busy.has("assemble");
    setStepStatus(container.head, "assemble", { detail: fresh ? `${formatNumber(fresh.meshed_problem.free_nodes)} unknowns` : null });
    const fieldError = currentError("field");
    const fieldSlot = container.querySelector("#field-error");
    if (fieldSlot.shown !== fieldError) {
      fieldSlot.shown = fieldError;
      const form = container.querySelector("#field-form");
      if (!markError(form, fieldError)) fill(fieldSlot, fieldError ? errorBox(fieldError, { openProblem }) : null);
      else fill(fieldSlot);
    }
    const shown = slot.shown;
    if (shown && shown.result === result && shown.error === error && shown.old === old && shown.busy === busy && shown.fresh === fresh) return;
    slot.shown = { result, error, old, busy, fresh };
    fill(
      slot,
      error ? errorBox(error, { openProblem }) : null,
      old ? errorBox(old, { outdated: true }) : null,
      result ? resultView(result, fresh !== null) : null,
    );
    slot.classList.toggle("outdated-result", Boolean(result) && !fresh);
  },
};
