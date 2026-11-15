// Step 2: the meshing pipeline, built from the catalog: the target (the
// MeshSpec) and one card per slot (discretizer, triangulator, refiner,
// improvers), each with its components and their options.

import { downloadExport } from "../api.js";
import { runStage, toast } from "../actions.js";
import { formatNumber } from "../core.js";
import { actionButton, errorBox, h, markError, optionsForm, reportsView, stat, fill } from "../dom.js";
import { choiceCard, componentBody, defaultChoice } from "./components.js";
import { editCase, freshResult, lastResult, stageStatus, store } from "../state.js";

function singleSlot(slot, mesh) {
  return choiceCard(slot, mesh[slot.id], `mesh.${slot.id}`, (next) => editCase((c) => (c.mesh[slot.id] = next), "structure"));
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
  return h(
    "div",
    { class: "card" },
    h("div", { class: "card-head" }, h("span", { class: "title" }, slot.label), h("span", { class: "spacer" }), add),
    h("div", { class: "help" }, slot.description),
    list.length === 0 ? h("p", { class: "note" }, "None.") : null,
    list.map((choice, i) => {
      const path = `mesh.${slot.id}[${i}]`;
      const component = slot.components.find((c) => c.type === choice.type);
      return h(
        "div",
        { class: "card field", dataset: { path } },
        h(
          "div",
          { class: "card-head" },
          h("span", { class: "chip" }, String(i + 1)),
          h("span", { class: "title" }, component?.label ?? choice.type),
          h("span", { class: "spacer" }),
          h("button", { type: "button", class: "icon", "aria-label": "Move up", disabled: i === 0, onclick: () => move(i, i - 1) }, "↑"),
          h("button", { type: "button", class: "icon", "aria-label": "Move down", disabled: i === list.length - 1, onclick: () => move(i, i + 1) }, "↓"),
          h("button", { type: "button", class: "icon", "aria-label": "Remove", onclick: () => editCase((c) => c.mesh[slot.id].splice(i, 1), "structure") }, "✕"),
        ),
        componentBody(slot, choice, path).slice(1),
      );
    }),
  );
}

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
      h(
        "div",
        { class: "card", dataset: { path: `mesh.${section.settings.key}` } },
        h("div", { class: "card-head" }, h("span", { class: "title" }, section.settings.label)),
        optionsForm(section.settings.options, mesh[section.settings.key], `mesh.${section.settings.key}`, () => editCase(() => {})),
      ),
      section.slots.map((slot) => (slot.cardinality === "list" ? listSlot(slot, mesh) : singleSlot(slot, mesh))),
      h(
        "div",
        { class: "row", style: "margin-top:12px" },
        actionButton(
          "Mesh",
          async () => {
            if (stageStatus("geometry") !== "done") await runStage("geometry");
            if (!store.errors.geometry) await runStage("mesh");
          },
          { primary: true },
        ),
      ),
      h("div", { class: "result", id: "mesh-result" }),
    );
    this.update(container);
  },

  update(container) {
    const slot = container.querySelector("#mesh-result");
    if (!slot) return;
    const error = store.errors.mesh;
    const fresh = freshResult("mesh");
    const result = fresh ?? lastResult("mesh");
    const busy = store.busy.has("mesh");
    if (slot.shown && slot.shown.result === result && slot.shown.error === error && slot.shown.busy === busy && slot.shown.fresh === fresh) return;
    slot.shown = { result, error, busy, fresh };
    const placed = markError(container, error);
    const children = [];
    if (error && !placed) children.push(errorBox(error));
    if (busy) children.push(h("p", { class: "note" }, "Meshing…"));
    if (result) {
      const s = result.stats;
      children.push(
        h(
          "div",
          { class: "row", style: "margin-top:4px" },
          h("h3", { style: "margin:8px 0" }, "Result"),
          fresh ? null : h("span", { class: "chip" }, "outdated: the case changed"),
          result.target_met ? h("span", { class: "chip good" }, "✓ target met") : h("span", { class: "chip bad" }, "✕ target missed"),
        ),
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
        h("h3", {}, "Reports"),
        reportsView(result.reports),
        h(
          "div",
          { class: "row", style: "margin-top:12px" },
          actionButton("Export .pde2d", () => downloadExport("mesh.pde2d", store.caseData).catch((e) => toast(e.message)), { small: true }),
          actionButton("Export mesh .svg", () => downloadExport("mesh.svg", store.caseData).catch((e) => toast(e.message)), { small: true }),
        ),
        h("p", { class: "note" }, "A .pde2d file opens in the old mesh viewer (tools/mesh_viewer) and can be imported here as a mesh file."),
      );
    }
    fill(slot, ...children);
  },
};
