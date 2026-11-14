// Step 4: the problem combined with the mesh (MeshedProblem) and assembled
// at one time t; fields evaluated on the mesh.

import { runStage, showField, toast } from "../actions.js";
import { formatNumber, rampTable, parseColor } from "../core.js";
import { actionButton, errorBox, h, markError, reportsView, stat, fill } from "../dom.js";
import { editCase, freshResult, lastResult, stageStatus, store } from "../state.js";

// Geometry and mesh first when they are not up to date, then assembly.
async function assemble() {
  for (const stage of ["geometry", "mesh", "assemble"]) {
    if (stage !== "assemble" && stageStatus(stage) === "done") continue;
    if (!(await runStage(stage))) {
      if (stage !== "assemble") toast(`The ${stage} step failed; see that step.`);
      return;
    }
  }
}

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
        if (stageStatus("mesh") !== "done") {
          if (stageStatus("geometry") !== "done" && !(await runStage("geometry"))) return;
          if (!(await runStage("mesh"))) return toast("The mesh step failed; see that step.");
        }
        const field = { kind: fieldForm.kind };
        if (fieldForm.kind === "expression") field.text = fieldForm.text;
        if (String(fieldForm.t).trim() !== "") field.t = Number(fieldForm.t);
        if (await showField(field)) {
          const colorBy = document.getElementById("color-by");
          colorBy.value = "field";
          colorBy.dispatchEvent(new Event("change"));
        }
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

function operatorsTable(operators) {
  const names = { mass: "M2 (u_tt)", damping: "B = M1 − D (u_t)", stiffness: "A (u)" };
  return h(
    "table",
    { class: "data" },
    h("thead", {}, h("tr", {}, h("th", {}, "Operator"), h("th", {}, "Changes in t"), h("th", { class: "num" }, "Entries"))),
    h(
      "tbody",
      {},
      Object.entries(operators).map(([name, op]) =>
        op.is_zero
          ? h("tr", {}, h("td", {}, names[name] ?? name), h("td", { colspan: 2, class: "note" }, "zero: no term contributes"))
          : h("tr", {}, h("td", {}, names[name] ?? name), h("td", {}, op.depends_on_t ? "yes" : "no"), h("td", { class: "num" }, formatNumber(op.free_nonzeros + op.dirichlet_nonzeros))),
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
        h("div", { class: "inputs" }, h("input", { type: "number", step: "any", value: String(assembly.t ?? 0), "aria-label": "Time t", oninput: (e) => e.target.value.trim() !== "" && editCase((c) => (c.assembly.t = Number(e.target.value))) })),
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
    const error = store.errors.assemble;
    const fresh = freshResult("assemble");
    const result = fresh ?? lastResult("assemble");
    const busy = store.busy.has("assemble");
    const fieldError = store.errors.field;
    const fieldSlot = container.querySelector("#field-error");
    if (fieldSlot.shown !== fieldError) {
      fieldSlot.shown = fieldError;
      const form = container.querySelector("#field-form");
      if (!markError(form, fieldError)) fill(fieldSlot, ...(fieldError ? [errorBox(fieldError)] : []));
      else fill(fieldSlot);
    }
    if (slot.shown && slot.shown.result === result && slot.shown.error === error && slot.shown.busy === busy && slot.shown.fresh === fresh) return;
    slot.shown = { result, error, busy, fresh };
    const children = [];
    if (error) children.push(errorBox(error));
    if (error && ["problem", "meshed_problem"].includes(error.stage)) children.push(h("p", { class: "note" }, "Fix it in the Problem step."));
    if (busy) children.push(h("p", { class: "note" }, "Assembling…"));
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
          stat("Time order", `${assembly.time_order} (problem ${meshed.time_order})`),
          stat("|b|", formatNumber(assembly.right_hand_side_norm)),
        ),
        h("p", { class: "note" }, `The system: M2 U'' + B U' + A U = b in the unknowns U. The load ${assembly.load_depends_on_t ? "changes" : "does not change"} with t; b ${assembly.right_hand_side_depends_on_t ? "changes" : "does not change"} with t.`),
        operatorsTable(assembly.operators),
        assembly.sparsity ? h("div", { style: "margin-top:12px" }, h("div", { class: "label", style: "font-size:13px;font-weight:550" }, `Entries of A (unknowns × unknowns, ${formatNumber(assembly.sparsity.nonzeros)} entries)`), spy) : null,
        h("h3", {}, "Reports"),
        reportsView([meshed.report, assembly.report]),
      );
    }
    fill(slot, ...children);
  },
};
