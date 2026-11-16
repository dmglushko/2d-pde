// Step 1: where the domain comes from (a shape, an SVG drawing or a mesh
// file), its regions and region edits. The geometry is read a moment
// after each change, so the head has no button, only the status.

import { downloadExport } from "../api.js";
import { runStage, toast } from "../actions.js";
import { applyRegionEdit, fileKind, formatNumber, undoRegionEdit } from "../core.js";
import { errorBox, exportButton, fileButton, fill, foldable, h, infoToggle, labelWithInfo, markError, optionsForm, readFileText, regionSwatch, stat } from "../dom.js";
import { currentError, editCase, freshResult, lastResult, outdatedError, store } from "../state.js";
import { setStepStatus, stepHead } from "./head.js";

let geometryTimer = null;

// Shows the geometry a moment after the last change.
function scheduleGeometry() {
  clearTimeout(geometryTimer);
  geometryTimer = setTimeout(() => runStage("geometry"), 250);
}

function setGeometry(geometry) {
  editCase((c) => {
    c.geometry = geometry;
  }, "structure");
  scheduleGeometry();
}

// Loads a dropped or chosen file as the domain (SVG or mesh file).
export async function loadDomainFile(file) {
  const kind = fileKind(file.name);
  if (kind !== "svg" && kind !== "gmsh" && kind !== "pde2d") {
    toast(`${file.name}: choose an .svg, .msh or .pde2d file`);
    return;
  }
  const text = await readFileText(file);
  if (kind === "svg") setGeometry({ source: "svg", name: file.name, svg: text, region_edits: [] });
  else setGeometry({ source: "mesh_file", format: kind, name: file.name, text, region_edits: [] });
}

const SOURCES = [
  ["shape", "Shape"],
  ["svg", "SVG drawing"],
  ["mesh_file", "Mesh file"],
];

function emptySource(value) {
  if (value === "svg") return { source: "svg", svg: "", region_edits: [] };
  return { source: "mesh_file", format: "gmsh", text: "", region_edits: [] };
}

function sourceSwitch(geometry) {
  const choose = (value) => {
    if (value === "shape") setGeometry(structuredClone(store.catalog.default_case.geometry));
    else if (value !== geometry.source) editCase((c) => (c.geometry = emptySource(value)), "structure");
  };
  return h(
    "div",
    { class: "row segmented", role: "radiogroup", "aria-label": "Source" },
    SOURCES.map(([value, label]) =>
      h(
        "label",
        { class: "choice" },
        h("input", { type: "radio", name: "geometry-source", checked: geometry.source === value, onchange: () => choose(value) }),
        label,
      ),
    ),
  );
}

function shapeForm(geometry) {
  const shapes = store.catalog.shapes;
  const shape = shapes.find((s) => s.type === geometry.shape) ?? shapes[0];
  const pick = (event) => {
    const next = shapes.find((s) => s.type === event.target.value);
    const options = Object.fromEntries(next.options.map((o) => [o.name, o.default]));
    setGeometry({ source: "shape", shape: next.type, options, region_edits: [] });
  };
  const select = h(
    "select",
    { "aria-label": "Shape", dataset: { path: "geometry.shape" }, onchange: pick },
    shapes.map((s) => h("option", { value: s.type, selected: s.type === shape.type }, s.label)),
  );
  geometry.options ??= {};
  const [labelRow, info] = labelWithInfo("Shape", shape.description);
  const changed = () => editCase(() => scheduleGeometry(), "edit", { sections: ["geometry"] });
  return [
    h("div", { class: "field", dataset: { path: "geometry.shape" } }, labelRow, select, info),
    h("div", { dataset: { path: "geometry.options" } }, optionsForm(shape.options, geometry.options, "geometry.options", changed)),
  ];
}

function svgForm(geometry) {
  const info = infoToggle(
    "Or drop the file on the picture. Each <path> is a run of curves; its region is its data-region attribute, else its id.",
    "SVG drawings",
  );
  const loaded = geometry.svg ? `${geometry.name ?? "drawing"} · ${formatNumber(geometry.svg.length)} characters` : "No drawing yet.";
  const editor = h("textarea", {
    rows: 8,
    spellcheck: false,
    "aria-label": "SVG text",
    value: geometry.svg ?? "",
    dataset: { path: "geometry.svg" },
    oninput: (event) =>
      editCase(
        (c) => {
          c.geometry.svg = event.target.value;
          scheduleGeometry();
        },
        "edit",
        { sections: ["geometry"] },
      ),
  });
  return [
    h(
      "div",
      { class: "drop" },
      h("div", { class: "row" }, fileButton("Choose an SVG file…", ".svg,image/svg+xml", loadDomainFile), info.button),
      h("p", { class: "drop-name" }, loaded),
      info.panel,
    ),
    h("details", { class: "report" }, h("summary", {}, "SVG text"), h("div", { class: "body field", dataset: { path: "geometry.svg" } }, editor)),
  ];
}

function meshFileForm(geometry) {
  const info = infoToggle(
    "Gmsh .msh (ASCII 4.1 or 2.2) or .pde2d. The Mesh step refines it with the refiner and improvers. Its regions can be renamed and merged, not split: a mesh file has no curves.",
    "mesh files",
  );
  const loaded = geometry.text ? `${geometry.name ?? "mesh"} · ${geometry.format === "gmsh" ? "Gmsh" : ".pde2d"}` : "No mesh file yet.";
  return [
    h(
      "div",
      { class: "drop" },
      h("div", { class: "row" }, fileButton("Choose a mesh file…", ".msh,.pde2d", loadDomainFile), info.button),
      h("p", { class: "drop-name" }, loaded),
      info.panel,
    ),
    h("div", { class: "field", dataset: { path: "geometry.text" } }),
  ];
}

// --- region edits ---------------------------------------------------------------

function describeEdit(edit) {
  if (edit.op === "rename") return `rename ${edit.from} → ${edit.to}`;
  if (edit.op === "merge") return `merge ${edit.source} into ${edit.target}`;
  if (edit.op === "split") return `split ${edit.region} at ${edit.fraction} into ${edit.first}, ${edit.second}`;
  return JSON.stringify(edit);
}

// Adds a region edit; its boundary conditions move with it, and undoing
// it restores them (core.js, applyRegionEdit and undoRegionEdit).
function replaceCase(next) {
  editCase((c) => {
    for (const key of Object.keys(c)) delete c[key];
    Object.assign(c, next);
  }, "structure");
  scheduleGeometry();
}

function addEdit(edit) {
  replaceCase(applyRegionEdit(store.caseData, edit));
}

function editLine(label, controls, apply) {
  return h("div", { class: "row edit-line" }, label, controls, h("button", { type: "button", class: "small", onclick: apply }, "Apply"));
}

// `canSplit`: false for a mesh file, which has no curves to split.
function regionEditor(region, regions, canSplit) {
  const others = regions.filter((r) => r !== region);
  const text = (value, label, size) => h("input", { type: "text", value, "aria-label": label, class: `name-input ${size}` });
  const renameInput = text(region, "New name", "wide");
  const mergeSelect = h("select", { "aria-label": "Merge into" }, others.map((r) => h("option", { value: r }, r)));
  const fraction = h("input", { type: "number", step: "any", value: "0.5", min: "0", max: "1", "aria-label": "Fraction", class: "fraction-input" });
  const first = text(`${region}_a`, "First name", "narrow");
  const second = text(`${region}_b`, "Second name", "narrow");
  const splitInfo = infoToggle("Measured along the boundary (domain on the left) from the start of the region.", "splitting");
  return h(
    "div",
    { class: "card" },
    editLine("Rename to", renameInput, () => addEdit({ op: "rename", from: region, to: renameInput.value.trim() })),
    others.length ? editLine("Merge into", mergeSelect, () => addEdit({ op: "merge", source: region, target: mergeSelect.value })) : null,
    canSplit
      ? editLine("Split at", [fraction, splitInfo.button, "into", first, second], () =>
          addEdit({ op: "split", region, fraction: Number(fraction.value), first: first.value.trim(), second: second.value.trim() }),
        )
      : null,
    splitInfo.panel,
  );
}

function regionRow(region, i, names, canEdit, canSplit) {
  const editorSlot = h("td", { colspan: 4, class: "editor-cell" });
  const toggle = () => {
    if (editorSlot.childElementCount) fill(editorSlot);
    else fill(editorSlot, regionEditor(region.name, names, canSplit));
  };
  const row = h(
    "tr",
    {},
    h("td", {}, regionSwatch(i)),
    h("td", {}, region.name),
    h("td", { class: "num" }, formatNumber(region.length)),
    h("td", { class: "num" }, canEdit ? h("button", { type: "button", class: "small", onclick: toggle }, "Edit") : null),
  );
  return [row, h("tr", {}, editorSlot)];
}

function regionsTable(view, canEdit, canSplit) {
  const names = view.regions.map((r) => r.name);
  const head = h("tr", {}, h("th", {}, ""), h("th", {}, "Region"), h("th", { class: "num" }, "Length"), h("th", {}, ""));
  const rows = view.regions.map((region, i) => regionRow(region, i, names, canEdit, canSplit));
  return h("table", { class: "data" }, h("thead", {}, head), h("tbody", {}, rows));
}

function editsList(edits) {
  if (edits.length === 0) return null;
  const undo = () => replaceCase(undoRegionEdit(store.caseData));
  return h(
    "div",
    {},
    h("h3", {}, "Region edits"),
    edits.map((edit, i) =>
      h(
        "div",
        { class: "edit-row", dataset: { path: `geometry.region_edits[${i}]` } },
        h("span", { class: "chip" }, `${i + 1}`),
        describeEdit(edit),
        h("span", { class: "spacer" }),
        // Only the last edit can be undone: an earlier one changes the
        // regions the later ones refer to.
        i === edits.length - 1
          ? h("button", { type: "button", class: "small", "aria-label": "Undo the last region edit", onclick: undo }, "Undo")
          : null,
      ),
    ),
  );
}

function domainStats(view) {
  const loops = view.kind === "curves" ? view.loops : [];
  const holes = loops.filter((l) => l.depth % 2 === 1).length;
  const counts =
    view.kind === "curves"
      ? [stat("Loops", loops.length), stat("Holes", holes)]
      : [stat("Nodes", formatNumber(view.nodes)), stat("Triangles", formatNumber(view.triangles))];
  const size = stat("Domain size", formatNumber(view.domain_size), "The diagonal of the bounding box: relative lengths in the Mesh step multiply it.");
  return h("div", { class: "stats" }, counts, size);
}

// --- the panel ------------------------------------------------------------------

export const geometryPanel = {
  build(container) {
    const geometry = store.caseData.geometry;
    const head = stepHead("geometry");
    const form = geometry.source === "shape" ? shapeForm(geometry) : geometry.source === "svg" ? svgForm(geometry) : meshFileForm(geometry);
    fill(
      container,
      head,
      h("div", { class: "result", id: "geometry-summary" }),
      h("div", { class: "settings-part" }, sourceSwitch(geometry), h("div", { class: "source-form" }, form), editsList(geometry.region_edits ?? [])),
      h("div", { id: "geometry-regions" }),
    );
    container.head = head;
    this.update(container);
  },

  update(container) {
    const slot = container.querySelector("#geometry-summary");
    if (!slot) return;
    const error = currentError("geometry");
    const old = outdatedError("geometry");
    const fresh = freshResult("geometry");
    const view = fresh ?? (error ? null : lastResult("geometry"));
    const busy = store.busy.has("geometry");
    setStepStatus(container.head, "geometry");
    // Rebuild only when something shown changed (an open region editor
    // would otherwise close on every notification).
    const shown = slot.shown;
    if (shown && shown.view === view && shown.error === error && shown.old === old && shown.busy === busy && shown.fresh === fresh) return;
    slot.shown = { view, error, old, busy, fresh };
    const placed = markError(container, error);
    fill(
      slot,
      error && !placed ? errorBox(error) : null,
      old && !busy ? errorBox(old, { outdated: true }) : null,
      view ? domainStats(view) : null,
    );
    slot.classList.toggle("outdated-result", Boolean(view) && !fresh);
    const save = () => downloadExport("geometry.svg", store.caseData, store.caseName).catch((e) => toast(e.message));
    fill(
      container.querySelector("#geometry-regions"),
      view
        ? foldable(
            { id: "geometry.regions", title: "Boundary regions", kind: "section" },
            regionsTable(view, true, view.kind === "curves"),
            view.kind === "curves"
              ? h(
                  "div",
                  { class: "row actions-row" },
                  exportButton("Export geometry .svg", save, {
                    fresh: fresh !== null,
                    staleReason: "The geometry shown is outdated: wait for it to be read again.",
                  }),
                )
              : null,
          )
        : null,
    );
  },
};

export { scheduleGeometry };
