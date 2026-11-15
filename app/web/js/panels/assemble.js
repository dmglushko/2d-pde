// Step 4: the problem combined with the mesh (MeshedProblem) and assembled
// at one time t; fields evaluated on the mesh.

import { downloadExport } from "../api.js";
import { runWithPrerequisites, toast } from "../actions.js";
import { formatNumber, rampTable, parseColor } from "../core.js";
import { actionButton, busyNote, errorBox, exportButton, h, markError, numberHandler, reportsView, stat, fill } from "../dom.js";
import { currentError, editCase, freshResult, lastResult, openStep, outdatedError, store } from "../state.js";

const assemble = () => runWithPrerequisites("assemble");
const openProblem = () => openStep("problem");

const fieldKinds = [
  ["expression", "an expression"],
  ["initial_u", "initial u"],
  ["initial_u_t", "initial u_t"],
  ["dirichlet", "Dirichlet values"],
  ["right_hand_side", "right-hand side b"],
];

// The field form keeps its values for the session (not in the case).
const fieldForm = { kind: "expression", text: "sin(pi*x)*y", t: "" };

function fieldSection() {
  const text = h("input", { type: "text", class: "expr", value: fieldForm.text, "aria-label": "Expression", oninput: (e) => (fieldForm.text = e.target.value) });
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
    fieldKinds.map(([value, label]) => h("option", { value, selected: value === fieldForm.kind }, label)),
  );
  const t = h("input", { type: "number", step: "any", value: fieldForm.t, placeholder: "assembly t", style: "width:110px", "aria-label": "t", oninput: (e) => (fieldForm.t = e.target.value) });
  return h(
    "div",
    { id: "field-form" },
    h("h3", {}, "Show a field on the mesh"),
    h("div", { class: "row" }, kind, h("label", { class: "row", style: "gap:4px" }, "t =", t)),
    textRow,
    h(
      "div",
      { class: "row", style: "margin-top:8px" },
      actionButton("Show", async () => {
        const field = { kind: fieldForm.kind };
        if (fieldForm.kind === "expression") field.text = fieldForm.text;
        if (String(fieldForm.t).trim() !== "") {
          field.t = Number(fieldForm.t);
          if (!Number.isFinite(field.t)) return toast("t is not a number.");
        }
        // The picture colours by the field once it arrives (actions.js).
        await runWithPrerequisites("field", field);
      }),
    ),
    h("p", { class: "note" }, "Values at the mesh nodes, drawn with linear interpolation. Dirichlet values and b exist only at some nodes and are drawn as dots."),
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
  const table = rampTable([parseColor(style.getPropertyValue("--seq-low")), parseColor(style.getPropertyValue("--seq-high"))]);
  const surface = parseColor(style.getPropertyValue("--surface"));
  const max = Math.max(1, ...sparsity.counts);
  for (let i = 0; i < bins * bins; i++) {
    const count = sparsity.counts[i];
    // Light for few entries, dark for many, on a log scale.
    const k = Math.round((255 * Math.log1p(count)) / Math.log1p(max));
    const color = count === 0 ? surface : table.subarray(3 * k, 3 * k + 3);
    image.data.set([color[0], color[1], color[2], 255], 4 * i);
  }
  context.putImageData(image, 0, 0);
}

// Symmetry from the formulas (every t) or from the numbers (this t only).
function symmetryText(op) {
  if (op.always_symmetric) return "yes";
  if (op.symmetric_now) return "at this t";
  return "no";
}

function operatorsTable(operators) {
  const names = { mass: "M2 (u_tt)", damping: "B = M1 − D (u_t)", stiffness: "A (u)" };
  return h(
    "table",
    { class: "data" },
    h("thead", {}, h("tr", {}, h("th", {}, "Operator"), h("th", {}, "Changes in t"), h("th", {}, "Symmetric"), h("th", { class: "num" }, "Entries"))),
    h(
      "tbody",
      {},
      Object.entries(operators).map(([name, op]) =>
        op.always_zero
          ? h("tr", {}, h("td", {}, names[name] ?? name), h("td", { colspan: 3, class: "note" }, "zero: no term contributes, or not built"))
          : h(
              "tr",
              {},
              h("td", {}, names[name] ?? name, op.zero_now ? h("span", { class: "chip", style: "margin-left:6px" }, "zero at this t") : null),
              h("td", {}, op.depends_on_t ? "yes" : "no"),
              h("td", {}, symmetryText(op)),
              h("td", { class: "num" }, formatNumber(op.free_nonzeros + op.dirichlet_nonzeros)),
            ),
      ),
    ),
  );
}

export const assemblePanel = {
  build(container) {
    const assembly = (store.caseData.assembly ??= { t: 0 });
    fill(container,
      h("h2", {}, "Assemble"),
      h("p", { class: "lead" }, "Combines the problem with the mesh (Dirichlet nodes, compiled expressions) and builds the P1 system at one time t."),
      h(
        "div",
        { class: "field", dataset: { path: "assembly.t" } },
        h("div", { class: "label" }, "Time t"),
        h("div", { class: "inputs" }, h("input", { type: "number", step: "any", value: String(assembly.t ?? 0), "aria-label": "Time t", oninput: numberHandler((t) => editCase((c) => (c.assembly.t = t))) })),
      ),
      h(
        "div",
        { class: "field", dataset: { path: "assembly.max_time_order" } },
        h("div", { class: "label" }, "Time terms to build"),
        h(
          "div",
          { class: "inputs" },
          h(
            "select",
            { "aria-label": "Time terms to build", onchange: (e) => editCase((c) => (c.assembly.max_time_order = Number(e.target.value))) },
            [
              [2, "all (M2, B and A)"],
              [1, "up to u_t (B and A)"],
              [0, "none: A and the load only"],
            ].map(([value, label]) => h("option", { value: String(value), selected: (assembly.max_time_order ?? 2) === value }, label)),
          ),
        ),
        h("div", { class: "help" }, "Left-out operators count as zero; with none, only the boundary values themselves are evaluated, not their time derivatives (a static solve assembles like this)."),
      ),
      h("div", { class: "row" }, actionButton("Assemble", assemble, { primary: true })),
      h("div", { class: "result", id: "assemble-result" }),
      fieldSection(),
    );
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
    const fieldError = currentError("field");
    const fieldSlot = container.querySelector("#field-error");
    if (fieldSlot.shown !== fieldError) {
      fieldSlot.shown = fieldError;
      const form = container.querySelector("#field-form");
      if (!markError(form, fieldError)) fill(fieldSlot, ...(fieldError ? [errorBox(fieldError, { openProblem })] : []));
      else fill(fieldSlot);
    }
    if (slot.shown && slot.shown.result === result && slot.shown.error === error && slot.shown.old === old && slot.shown.busy === busy && slot.shown.fresh === fresh) return;
    slot.shown = { result, error, old, busy, fresh };
    const children = [];
    if (error) children.push(errorBox(error, { openProblem }));
    if (old) children.push(errorBox(old, { outdated: true }));
    if (busy) children.push(busyNote("assemble", "Assembling"));
    if (result) {
      const meshed = result.meshed_problem;
      const assembly = result.assembly;
      const spy = h("canvas", { class: "spy", "aria-label": "Where the stiffness matrix has entries" });
      if (assembly.sparsity) drawSpy(spy, assembly.sparsity);
      children.push(
        h("div", { class: "row" }, h("h3", { style: "margin:8px 0" }, `Result at t = ${formatNumber(assembly.t)}`), fresh ? null : h("span", { class: "chip" }, "outdated: the case changed")),
        h(
          "div",
          { class: "stats" },
          stat("Unknowns", formatNumber(meshed.free_nodes)),
          stat("Dirichlet nodes", formatNumber(meshed.dirichlet_nodes.length)),
          stat("Time order", `${assembly.time_order} (as written ${meshed.time_order})`),
          stat("|b|", formatNumber(assembly.right_hand_side_norm)),
        ),
        h("p", { class: "note" }, `Symmetric "yes" holds at every t (from the formulas); "at this t" only for these numbers. The system: M2 U'' + B U' + A U = b in the unknowns U. The load ${assembly.load_depends_on_t ? "changes" : "does not change"} with t; b ${assembly.right_hand_side_depends_on_t ? "changes" : "does not change"} with t.`),
        operatorsTable(assembly.operators),
        assembly.sparsity ? h("div", { style: "margin-top:12px" }, h("div", { class: "label", style: "font-size:13px;font-weight:550" }, `Entries of A (unknowns × unknowns, ${formatNumber(assembly.sparsity.nonzeros)} entries)`), spy) : null,
        h("h3", {}, "Reports"),
        reportsView([meshed.report, assembly.report]),
        h(
          "div",
          { class: "row", style: "margin-top:12px" },
          [
            ["stiffness.mtx", "Export A .mtx"],
            ["damping.mtx", "Export B .mtx"],
            ["mass.mtx", "Export M2 .mtx"],
            ["rhs.mtx", "Export b .mtx"],
          ].map(([kind, label]) => exportButton(label, () => downloadExport(kind, store.caseData).catch((e) => toast(e.message)), { fresh, staleReason: "The system shown is outdated: assemble again to export it." })),
        ),
        h("p", { class: "note" }, "Matrix Market files of the free part (unknowns × unknowns, 1-based) and of b, at this t."),
      );
    }
    fill(slot, ...children);
  },
};
