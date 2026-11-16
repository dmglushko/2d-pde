// Step 1: where the domain comes from (a shape, an SVG, a mesh file or a
// drawing edited on the picture), its regions and region edits. The
// geometry is read a moment after each change, so the head has no
// button, only the status. The picture's tools are geometry_editor.js;
// for a drawing this panel shows what is selected there.

import { exportFile, scheduleGeometry, toast } from "../actions.js";
import {
  assignRegion,
  deleteLoopAt,
  deleteVertexAt,
  editAsDrawing,
  editorSelection,
  insertMidVertex,
  mergeRegion,
  onEditor,
  renameRegion,
  setEdgeKind,
  setTool,
  setVertex,
} from "../geometry_editor.js";
import { applyRegionEdit, fileKind, undoRegionEdit } from "../pure/case_model.js";
import { blankDrawing, drawingRegions } from "../pure/drawing.js";
import { formatNumber, parseNumberText } from "../pure/format.js";
import { actionButton, errorBox, exportButton, fileButton, fill, foldable, h, infoToggle, labelWithInfo, markError, numberInput, optionsForm, readFileText, regionSwatch, stat } from "../dom.js";
import { currentError, editCase, freshResult, lastResult, outdatedError, requestTicket, sectionRevision, store } from "../state.js";
import { setStepStatus, stepHead } from "./head.js";

function setGeometry(geometry) {
  editCase((c) => {
    c.geometry = geometry;
  }, "structure");
  scheduleGeometry();
}

// Loads a dropped or chosen file as the domain (SVG or mesh file).
// Reading a file takes a while: it takes a ticket of the "open" kind with
// the geometry's revision, so a case opened, another file chosen or the
// geometry edited meanwhile wins, and the late text is dropped. Returns
// whether the file became the geometry.
export async function loadDomainFile(file) {
  const kind = fileKind(file.name);
  if (kind !== "svg" && kind !== "gmsh" && kind !== "pde2d") {
    toast(`${file.name}: choose an .svg, .msh or .pde2d file`);
    return false;
  }
  const ticket = requestTicket("open", () => sectionRevision("geometry"));
  let text;
  try {
    text = await readFileText(file);
  } catch {
    if (ticket.live()) toast(`${file.name} could not be read.`);
    return false;
  }
  if (!ticket.holds()) return false;
  if (kind === "svg") setGeometry({ source: "svg", name: file.name, svg: text, region_edits: [] });
  else setGeometry({ source: "mesh_file", format: kind, name: file.name, text, region_edits: [] });
  return true;
}

const SOURCES = [
  ["shape", "Shape"],
  ["svg", "SVG"],
  ["mesh_file", "Mesh file"],
  ["drawing", "Drawing"],
];

function emptySource(value) {
  if (value === "svg") return { source: "svg", svg: "", region_edits: [] };
  return { source: "mesh_file", format: "gmsh", text: "", region_edits: [] };
}

function sourceSwitch(geometry) {
  const choose = (value) => {
    if (value === geometry.source) return;
    if (value === "shape") setGeometry(structuredClone(store.catalog.default_case.geometry));
    // A new drawing starts from a blank rectangle (Edit as drawing turns a
    // shape or an SVG into one, curve for curve).
    else if (value === "drawing") setGeometry(blankDrawing());
    else editCase((c) => (c.geometry = emptySource(value)), "structure");
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
    asDrawingRow(),
  ];
}

// "Edit as drawing": the curves as they are (region edits applied) become
// a drawing to edit on the picture.
function asDrawingRow() {
  return h(
    "div",
    { class: "row actions-row" },
    actionButton("Edit as drawing", () => editAsDrawing(), { small: true }),
    infoToggle("Turns this geometry, with its region edits, into a drawing with exactly the same curves: drag its vertices, bend its edges, add holes.", "editing as a drawing").button,
  );
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
    geometry.svg ? asDrawingRow() : null,
  ];
}

function meshFileForm(geometry) {
  const info = infoToggle(
    "Gmsh .msh (ASCII 4.1 or 2.2) or .pde2d. The Mesh step refines it with the refiner and improvers. Its regions can be renamed and merged, and split at a boundary node with the split tool on the picture (a mesh file has no curves).",
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

// --- the drawing ---------------------------------------------------------------

const KINDS = [
  ["line", "Line"],
  ["arc", "Arc"],
  ["quadratic", "Quadratic"],
  ["cubic", "Cubic"],
];

function drawingForm() {
  const info = infoToggle(
    "On the picture: drag a vertex, a handle or an outline; double-click an edge to add a vertex; Shift+click selects several edges. Keys: arrows nudge the selected vertex (Shift: ten steps), Delete removes it, Esc cancels. The tools add rectangles, circles and polygons (inside the domain they are holes) and split a region where you click.",
    "the drawing editor",
  );
  // Loops, vertices and edges are numbered from 0, as in the case file
  // and the library's messages; an error about one is shown here.
  return h(
    "div",
    { class: "field drawing-field", dataset: { path: "geometry.loops" } },
    h("div", { class: "row drawing-hint" }, info.button),
    info.panel,
    h("div", { id: "drawing-selection", class: "drawing-selection" }),
  );
}

// The selection on the picture as fields: a vertex's x and y, an edge's
// kind and region, and what can be done with them.
function renderInspector(slot) {
  if (!slot) return;
  const drawing = store.caseData.geometry;
  const selection = editorSelection();
  if (!selection || drawing?.source !== "drawing") return fill(slot);
  if (selection.vertex !== undefined) {
    const { loop: l, vertex: v } = selection;
    const point = drawing.loops[l]?.vertices[v];
    if (!point) return fill(slot);
    const coordinate = (axis, label) =>
      numberInput({
        value: point[axis],
        label,
        path: `geometry.loops[${l}].vertices[${v}][${axis}]`,
        className: "coordinate",
        onNumber: () => {},
        onCommit: (event) => {
          const parsed = parseNumberText(event.target.value);
          if (parsed.kind !== "number") return;
          const now = store.caseData.geometry.loops[l]?.vertices[v];
          if (!now) return;
          setVertex(l, v, axis === 0 ? [parsed.value, now[1]] : [now[0], parsed.value]);
        },
      });
    const x = coordinate(0, "x");
    const y = coordinate(1, "y");
    slot.coordinates = { loop: l, vertex: v, x, y };
    return fill(
      slot,
      h(
        "div",
        { class: "card selection-card", dataset: { path: `geometry.loops[${l}].vertices[${v}]` } },
        h("div", { class: "selection-title" }, `Vertex ${v}`, h("span", { class: "muted" }, ` · loop ${l}`)),
        h("div", { class: "row coordinates" }, h("label", {}, "x", x), h("label", {}, "y", y)),
        h("div", { class: "row" }, h("button", { type: "button", class: "small", onclick: () => deleteVertexAt(l, v) }, "Delete vertex")),
      ),
    );
  }
  slot.coordinates = null;
  const edges = selection.edges.filter((s) => drawing.loops[s.loop]?.edges[s.edge]);
  if (edges.length === 0) return fill(slot);
  const first = edges[0];
  const edge = drawing.loops[first.loop].edges[first.edge];
  const regions = drawingRegions(drawing);
  const listId = "drawing-regions";
  const regionInput = h("input", { type: "text", class: "name-input", list: listId, value: edges.every((s) => drawing.loops[s.loop].edges[s.edge].region === edge.region) ? edge.region : "", "aria-label": "Region of the selected edges", spellcheck: false, autocomplete: "off" });
  const applyRegion = () => assignRegion(edges, regionInput.value);
  regionInput.addEventListener("keydown", (event) => {
    if (event.key === "Enter") {
      event.preventDefault();
      applyRegion();
    }
  });
  const loops = [...new Set(edges.map((s) => s.loop))];
  const single = edges.length === 1;
  const closed = drawing.loops[first.loop].vertices.length === 1;
  const kinds = single
    ? h(
        "div",
        { class: "row segmented kinds", role: "radiogroup", "aria-label": "Edge kind" },
        KINDS.map(([kind, label]) =>
          h(
            "label",
            { class: "choice" },
            h("input", { type: "radio", name: "edge-kind", checked: edge.kind === kind, disabled: closed && kind !== "arc", onchange: () => setEdgeKind(first.loop, first.edge, kind) }),
            label,
          ),
        ),
      )
    : null;
  fill(
    slot,
    h(
      "div",
      { class: "card selection-card", dataset: { path: single ? `geometry.loops[${first.loop}].edges[${first.edge}]` : `geometry.loops[${first.loop}]` } },
      h("div", { class: "selection-title" }, single ? `Edge ${first.edge}` : `${edges.length} edges`, h("span", { class: "muted" }, loops.length === 1 ? ` · loop ${loops[0]}` : ` · ${loops.length} loops`)),
      kinds,
      h(
        "div",
        { class: "row region-row" },
        h("label", { for: null }, "Region"),
        regionInput,
        h("datalist", { id: listId }, regions.map((r) => h("option", { value: r }))),
        h("button", { type: "button", class: "small", onclick: applyRegion }, "Apply"),
      ),
      h(
        "div",
        { class: "row" },
        single && !closed ? h("button", { type: "button", class: "small", onclick: () => insertMidVertex(first.loop, first.edge) }, "Add vertex") : null,
        loops.length === 1 && drawing.loops.length > 1 ? h("button", { type: "button", class: "small", onclick: () => deleteLoopAt(loops[0]) }, "Delete loop") : null,
      ),
    ),
  );
}

// --- region edits ---------------------------------------------------------------

function describeEdit(edit) {
  if (edit.op === "rename") return `rename ${edit.from} → ${edit.to}`;
  if (edit.op === "merge") return `merge ${edit.source} into ${edit.target}`;
  if (edit.op === "split") return `split ${edit.region} at ${formatNumber(edit.fraction)} into ${edit.first}, ${edit.second}`;
  if (edit.op === "split_at_node") return `split ${edit.region} at node ${edit.node} into ${edit.first}, ${edit.second}`;
  return JSON.stringify(edit);
}

// Adds a region edit; its boundary conditions move with it, and undoing
// it restores them (pure/case_model.js, applyRegionEdit and undoRegionEdit).
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

function editLine(label, controls, apply, applyLabel = "Apply") {
  return h("div", { class: "row edit-line" }, label, controls, h("button", { type: "button", class: "small", onclick: apply }, applyLabel));
}

// Rename and merge for every source (geometry_editor.js: a drawing
// renames its edges, the others add an edit); a split at a typed fraction
// for shapes and SVGs, and on the picture for every source.
function regionEditor(region, regions, source) {
  const others = regions.filter((r) => r !== region);
  const text = (value, label, size) => h("input", { type: "text", value, "aria-label": label, class: `name-input ${size}` });
  const renameInput = text(region, "New name", "wide");
  const mergeSelect = h("select", { "aria-label": "Merge into" }, others.map((r) => h("option", { value: r }, r)));
  const fraction = h("input", { type: "text", inputmode: "decimal", autocomplete: "off", value: "0.5", "aria-label": "Fraction", class: "number fraction-input" });
  const first = text(`${region}_a`, "First name", "narrow");
  const second = text(`${region}_b`, "Second name", "narrow");
  const splitInfo = infoToggle("Measured along the boundary (domain on the left) from the start of the region. Or split it on the picture: the split tool, then a click on the boundary.", "splitting");
  const fractionSplit = source === "shape" || source === "svg";
  return h(
    "div",
    { class: "card" },
    editLine("Rename to", renameInput, () => renameRegion(region, renameInput.value)),
    others.length ? editLine("Merge into", mergeSelect, () => mergeRegion(region, mergeSelect.value)) : null,
    fractionSplit
      ? editLine("Split at", [fraction, splitInfo.button, "into", first, second], () => {
          const parsed = parseNumberText(fraction.value);
          if (parsed.kind !== "number") return toast(`The fraction: ${parsed.kind === "empty" ? "a value is needed" : parsed.reason}.`);
          addEdit({ op: "split", region, fraction: parsed.value, first: first.value.trim(), second: second.value.trim() });
        })
      : null,
    fractionSplit ? splitInfo.panel : null,
    h("div", { class: "row edit-line" }, h("button", { type: "button", class: "small", onclick: () => setTool("split") }, "Split on the picture…")),
  );
}

function regionRow(region, i, names, source) {
  const editorSlot = h("td", { colspan: 4, class: "editor-cell" });
  const toggle = () => {
    if (editorSlot.childElementCount) fill(editorSlot);
    else fill(editorSlot, regionEditor(region.name, names, source));
  };
  const row = h(
    "tr",
    { dataset: { region: region.name }, class: store.selectedRegion === region.name ? "selected" : null },
    h("td", {}, regionSwatch(i)),
    h("td", {}, region.name),
    h("td", { class: "num" }, formatNumber(region.length)),
    h("td", { class: "num" }, h("button", { type: "button", class: "small", onclick: toggle }, "Edit")),
  );
  return [row, h("tr", {}, editorSlot)];
}

function regionsTable(view, source) {
  const names = view.regions.map((r) => r.name);
  const head = h("tr", {}, h("th", {}, ""), h("th", {}, "Region"), h("th", { class: "num" }, "Length"), h("th", {}, ""));
  const rows = view.regions.map((region, i) => regionRow(region, i, names, source));
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

function sourceForm(geometry) {
  if (geometry.source === "shape") return shapeForm(geometry);
  if (geometry.source === "svg") return svgForm(geometry);
  if (geometry.source === "drawing") return drawingForm(geometry);
  return meshFileForm(geometry);
}

// The selection's inspector follows the editor: rebuilt when the
// selection changes (or the case does, below), its coordinates set in
// place while a vertex is dragged.
onEditor((kind, detail) => {
  const slot = document.getElementById("drawing-selection");
  if (!slot) return;
  if (kind === "selection") return renderInspector(slot);
  const shown = slot.coordinates;
  if (kind === "drag" && shown && shown.loop === detail.loop && shown.vertex === detail.vertex) {
    shown.x.value = formatNumber(detail.point[0], 12);
    shown.y.value = formatNumber(detail.point[1], 12);
  }
});

export const geometryPanel = {
  build(container) {
    const geometry = store.caseData.geometry;
    const head = stepHead("geometry");
    fill(
      container,
      head,
      h("div", { class: "result", id: "geometry-summary" }),
      h(
        "div",
        { class: "settings-part" },
        sourceSwitch(geometry),
        h("div", { class: "source-form" }, sourceForm(geometry)),
        geometry.source === "drawing" ? null : editsList(geometry.region_edits ?? []),
      ),
      h("div", { id: "geometry-regions" }),
    );
    container.head = head;
    renderInspector(container.querySelector("#drawing-selection"));
    this.update(container);
  },

  update(container) {
    const slot = container.querySelector("#geometry-summary");
    if (!slot) return;
    // The drawing changed (a drag, an undo): its inspector shows the new
    // values, unless a field of it is being typed in.
    const inspector = container.querySelector("#drawing-selection");
    if (inspector && inspector.drawing !== store.caseData.geometry && !inspector.contains(document.activeElement)) {
      inspector.drawing = store.caseData.geometry;
      renderInspector(inspector);
    }
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
    const save = () => exportFile("geometry.svg");
    const source = store.caseData.geometry.source;
    fill(
      container.querySelector("#geometry-regions"),
      view
        ? foldable(
            { id: "geometry.regions", title: "Boundary regions", kind: "section" },
            regionsTable(view, source),
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
