// Step 2: the meshing pipeline, built from the catalog: the target (the
// MeshSpec) and one card per slot (discretizer, triangulator, refiner,
// improvers), each with its components and their options.

import { downloadExport } from "../api.js";
import { runWithPrerequisites, toast } from "../actions.js";
import { formatNumber } from "../core.js";
import { actionButton, busyNote, errorBox, exportButton, foldable, h, markError, optionsForm, reportsView, stat, fill } from "../dom.js";
import { choiceCard, componentBody, defaultChoice } from "./components.js";
import { currentError, editCase, freshResult, lastResult, outdatedError, store } from "../state.js";

// The boundary discretizer and the initial triangulator are rarely
// changed: their cards start folded.
const RARELY_CHANGED = new Set(["discretizer", "triangulator"]);

function singleSlot(slot, mesh) {
  return choiceCard(slot, mesh[slot.id], `mesh.${slot.id}`, (next) => editCase((c) => (c.mesh[slot.id] = next), "structure"), { folded: RARELY_CHANGED.has(slot.id) });
}

function listSlot(slot, mesh) {
  const list = (mesh[slot.id] ??= []);
  const move = (i, j) =>
    editCase((c) => {
      const items = c.mesh[slot.id];
      [items[i], items[j]] = [items[j], items[i]];
    }, "structure");
  const add = h(
    "select",
    {
      "aria-label": `Add to ${slot.label}`,
      onchange: (event) => {
        const component = slot.components.find((c) => c.type === event.target.value);
        if (component) editCase((c) => c.mesh[slot.id].push(defaultChoice(component)), "structure");
      },
    },
    h("option", { value: "" }, "Add…"),
    slot.components.map((c) => h("option", { value: c.type }, c.label)),
  );
  return foldable(
    { id: `mesh.${slot.id}`, title: slot.label, extras: [h("span", { class: "spacer" }), add] },
    h("div", { class: "help" }, slot.description),
    list.length === 0 ? h("p", { class: "note" }, "None.") : null,
    list.map((choice, i) => {
      const path = `mesh.${slot.id}[${i}]`;
      const component = slot.components.find((c) => c.type === choice.type);
      // Folded per position in the list: a moved card keeps its place's
      // state, which is what the eye follows.
      return foldable(
        {
          id: `${path}`,
          title: [h("span", { class: "chip" }, String(i + 1)), " ", component?.label ?? choice.type],
          extras: [
            h("span", { class: "spacer" }),
            h("button", { type: "button", class: "icon", "aria-label": "Move up", disabled: i === 0, onclick: () => move(i, i - 1) }, "↑"),
            h("button", { type: "button", class: "icon", "aria-label": "Move down", disabled: i === list.length - 1, onclick: () => move(i, i + 1) }, "↓"),
            h("button", { type: "button", class: "icon", "aria-label": "Remove", onclick: () => editCase((c) => c.mesh[slot.id].splice(i, 1), "structure") }, "✕"),
          ],
          attributes: { class: "field", dataset: { path } },
        },
        componentBody(slot, choice, path).slice(1),
      );
    }),
  );
}

const STALE = "The mesh shown is outdated: mesh again to export it.";

export const meshPanel = {
  build(container) {
    const section = store.catalog.sections.find((s) => s.id === "mesh");
    const mesh = store.caseData.mesh;
    mesh[section.settings.key] ??= {};
    const fromFile = store.caseData.geometry?.source === "mesh_file";
    fill(container,
      h("h2", {}, "Mesh"),
      h("p", { class: "lead" }, "The stages run in order: boundary discretizer, initial triangulator, refiner, improvers."),
      fromFile ? h("div", { class: "notice" }, "The domain is an imported mesh: only the refiner and the improvers run on it.") : null,
      foldable(
        { id: `mesh.${section.settings.key}`, title: section.settings.label, attributes: { dataset: { path: `mesh.${section.settings.key}` } } },
        optionsForm(section.settings.options, mesh[section.settings.key], `mesh.${section.settings.key}`, () => editCase(() => {}, "edit", { sections: ["mesh"] })),
      ),
      section.slots.map((slot) => (slot.cardinality === "list" ? listSlot(slot, mesh) : singleSlot(slot, mesh))),
      h(
        "div",
        { class: "row", style: "margin-top:12px" },
        actionButton("Mesh", () => runWithPrerequisites("mesh"), { primary: true }),
      ),
      h("div", { class: "result", id: "mesh-result" }),
    );
    this.update(container);
  },

  update(container) {
    const slot = container.querySelector("#mesh-result");
    if (!slot) return;
    const error = currentError("mesh");
    const old = outdatedError("mesh");
    const fresh = freshResult("mesh");
    const result = fresh ?? lastResult("mesh");
    const busy = store.busy.has("mesh");
    if (slot.shown && slot.shown.result === result && slot.shown.error === error && slot.shown.old === old && slot.shown.busy === busy && slot.shown.fresh === fresh) return;
    slot.shown = { result, error, old, busy, fresh };
    const placed = markError(container, error);
    const children = [];
    if (error && !placed) children.push(errorBox(error));
    if (old) children.push(errorBox(old, { outdated: true }));
    if (busy) children.push(busyNote("mesh", "Meshing"));
    if (result) {
      const s = result.stats;
      children.push(
        foldable(
          {
            id: "mesh.result",
            title: "Result",
            kind: "section",
            extras: [fresh ? null : h("span", { class: "chip" }, "outdated: the case changed"), result.target_met ? h("span", { class: "chip good" }, "✓ target met") : h("span", { class: "chip bad" }, "✕ target missed")],
          },
        h(
          "div",
          { class: "stats" },
          stat("Nodes", formatNumber(s.nodes)),
          stat("Triangles", formatNumber(s.triangles)),
          stat("Smallest angle", `${formatNumber(s.min_angle_degrees, 3)}°`),
          stat("Mean smallest angle", `${formatNumber(s.mean_min_angle_degrees, 3)}°`),
          stat("Longest edge", formatNumber(result.longest_edge)),
          stat("Edge bound", formatNumber(result.max_edge_length)),
        ),
        result.target_met ? null : h("p", { class: "note" }, "Some edge is longer than the bound. Without a refiner the mesh has only the boundary points; smoothing can also lengthen edges."),
        foldable({ id: "mesh.reports", title: "Reports", kind: "section", folded: true }, reportsView(result.reports)),
        h(
          "div",
          { class: "row", style: "margin-top:12px" },
          exportButton("Export .pde2d", () => downloadExport("mesh.pde2d", store.caseData).catch((e) => toast(e.message)), { fresh, staleReason: STALE }),
          exportButton("Export mesh .svg", () => downloadExport("mesh.svg", store.caseData).catch((e) => toast(e.message)), { fresh, staleReason: STALE }),
        ),
        h("p", { class: "note" }, "A .pde2d file opens in the old mesh viewer (tools/mesh_viewer) and can be imported here as a mesh file."),
        ),
      );
    }
    fill(slot, ...children);
  },
};
