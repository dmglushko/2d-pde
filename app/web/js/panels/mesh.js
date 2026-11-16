// Step 2: the meshing pipeline, built from the catalog: the target (the
// MeshSpec) and one card per slot (discretizer, triangulator, refiner,
// improvers), each with its components and their options. The head holds
// the Mesh button and the status, the result follows, the settings last.

import { exportFile, runWithPrerequisites } from "../actions.js";
import { formatNumber } from "../pure/format.js";
import { actionButton, errorBox, exportButton, fill, foldable, h, markError, optionsForm, reportsView, stat, warningChip } from "../dom.js";
import { currentError, editCase, freshResult, lastResult, outdatedError, store } from "../state.js";
import { choiceCard, componentBody, componentInfo, defaultChoice } from "./components.js";
import { setStepStatus, stepHead } from "./head.js";

// The boundary discretizer and the initial triangulator are rarely
// changed: their cards start folded.
const RARELY_CHANGED = new Set(["discretizer", "triangulator"]);

function singleSlot(slot, mesh) {
  const pick = (next) => editCase((c) => (c.mesh[slot.id] = next), "structure");
  return choiceCard(slot, mesh[slot.id], `mesh.${slot.id}`, pick, { folded: RARELY_CHANGED.has(slot.id) });
}

function addSelect(slot) {
  return h(
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
}

// One improver of the list, with buttons that move or remove it.
function listItem(slot, list, choice, i) {
  const path = `mesh.${slot.id}[${i}]`;
  const component = slot.components.find((c) => c.type === choice.type);
  const move = (j) =>
    editCase((c) => {
      const items = c.mesh[slot.id];
      [items[i], items[j]] = [items[j], items[i]];
    }, "structure");
  const remove = () => editCase((c) => c.mesh[slot.id].splice(i, 1), "structure");
  const info = componentInfo({ ...slot, description: null }, choice);
  const icon = (label, text, onclick, disabled = false) =>
    h("button", { type: "button", class: "icon", "aria-label": label, title: label, disabled, onclick }, text);
  // Folded per position in the list: a moved card keeps its place's
  // state, which is what the eye follows.
  return foldable(
    {
      id: path,
      title: [h("span", { class: "chip" }, String(i + 1)), " ", component?.label ?? choice.type],
      extras: [
        info.button,
        h("span", { class: "spacer" }),
        icon("Move up", "↑", () => move(i - 1), i === 0),
        icon("Move down", "↓", () => move(i + 1), i === list.length - 1),
        icon("Remove", "✕", remove),
      ],
      info: info.panel,
      attributes: { class: "field", dataset: { path } },
    },
    componentBody(slot, choice, path),
  );
}

function listSlot(slot, mesh) {
  const list = (mesh[slot.id] ??= []);
  const info = componentInfo(slot, null);
  return foldable(
    { id: `mesh.${slot.id}`, title: slot.label, extras: [info.button, h("span", { class: "spacer" }), addSelect(slot)], info: info.panel },
    list.length === 0 ? h("p", { class: "note" }, "None.") : null,
    list.map((choice, i) => listItem(slot, list, choice, i)),
  );
}

const STALE = "The mesh shown is outdated: mesh again to export it.";

function resultView(result, fresh) {
  const s = result.stats;
  const save = (kind) => () => exportFile(kind);
  return [
    warningChip(result.reports, "mesh.reports"),
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
    result.target_met
      ? null
      : h(
          "div",
          { class: "notice" },
          "An edge is longer than the bound. Add a refiner, or check the improvers: smoothing can lengthen edges.",
        ),
    h(
      "div",
      { class: "row actions-row" },
      exportButton("Export .pde2d", save("mesh.pde2d"), {
        fresh,
        staleReason: STALE,
        about: "Opens in the old mesh viewer (tools/mesh_viewer) and here as a mesh file.",
      }),
      exportButton("Export mesh .svg", save("mesh.svg"), { fresh, staleReason: STALE }),
    ),
    foldable({ id: "mesh.reports", title: "Reports", kind: "section", folded: true }, reportsView(result.reports)),
  ];
}

export const meshPanel = {
  build(container) {
    const section = store.catalog.sections.find((s) => s.id === "mesh");
    const mesh = store.caseData.mesh;
    const key = section.settings.key;
    mesh[key] ??= {};
    const fromFile = store.caseData.geometry?.source === "mesh_file";
    const head = stepHead("mesh", actionButton("Mesh", () => runWithPrerequisites("mesh"), { primary: true, guarded: true }));
    const spec = foldable(
      { id: `mesh.${key}`, title: section.settings.label, attributes: { dataset: { path: `mesh.${key}` } } },
      optionsForm(section.settings.options, mesh[key], `mesh.${key}`, () => editCase(() => {}, "edit", { sections: ["mesh"] })),
    );
    fill(
      container,
      head,
      h("div", { class: "result", id: "mesh-result" }),
      h(
        "div",
        { class: "settings-part" },
        h("h3", { class: "part-title" }, "Settings"),
        fromFile ? h("div", { class: "notice" }, "An imported mesh: only the refiner and the improvers run on it.") : null,
        spec,
        section.slots.map((slot) => (slot.cardinality === "list" ? listSlot(slot, mesh) : singleSlot(slot, mesh))),
      ),
    );
    container.head = head;
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
    const detail = fresh ? `${formatNumber(fresh.stats.triangles)} triangles` : null;
    setStepStatus(container.head, "mesh", { detail });
    const shown = slot.shown;
    if (shown && shown.result === result && shown.error === error && shown.old === old && shown.busy === busy && shown.fresh === fresh) return;
    slot.shown = { result, error, old, busy, fresh };
    const placed = markError(container, error);
    fill(
      slot,
      error && !placed ? errorBox(error) : null,
      old ? errorBox(old, { outdated: true }) : null,
      result ? resultView(result, fresh !== null) : null,
    );
    slot.classList.toggle("outdated-result", Boolean(result) && !fresh);
  },
};
