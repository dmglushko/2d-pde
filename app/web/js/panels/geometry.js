// Step 1: where the domain comes from (a shape, an SVG drawing or a mesh
// file), its regions and region edits.

import { downloadExport } from "../api.js";
import { runStage, toast } from "../actions.js";
import { applyRegionEdit, fileKind, formatNumber, regionToken, undoRegionEdit } from "../core.js";
import { busyNote, errorBox, exportButton, foldable, h, markError, optionsForm, readFileText, stat, fill } from "../dom.js";
import { currentError, editCase, freshResult, lastResult, outdatedError, store } from "../state.js";

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

function fileButton(label, accept) {
  const input = h("input", {
    type: "file",
    accept,
    hidden: true,
    onchange: (event) => {
      const file = event.target.files[0];
      if (file) loadDomainFile(file);
      event.target.value = "";
    },
  });
  return h("label", { class: "button" }, label, input);
}

function sourceSwitch(geometry) {
  const sources = [
    ["shape", "Shape"],
    ["svg", "SVG drawing"],
    ["mesh_file", "Mesh file"],
  ];
  return h(
    "div",
    { class: "row", role: "radiogroup", "aria-label": "Source" },
    sources.map(([value, label]) =>
      h(
        "label",
        { class: "chip", style: "cursor:pointer;padding:4px 10px" },
        h("input", {
          type: "radio",
          name: "geometry-source",
          checked: geometry.source === value,
          onchange: () => {
            if (value === "shape") setGeometry(structuredClone(store.catalog.default_case.geometry));
            else if (value === geometry.source) return;
            else editCase((c) => (c.geometry = { source: value, ...(value === "svg" ? { svg: "", region_edits: [] } : { format: "gmsh", text: "", region_edits: [] }) }), "structure");
          },
        }),
        label,
      ),
    ),
  );
}

function shapeForm(geometry) {
  const shapes = store.catalog.shapes;
  const shape = shapes.find((s) => s.type === geometry.shape) ?? shapes[0];
  const select = h(
    "select",
    {
      "aria-label": "Shape",
      dataset: { path: "geometry.shape" },
      onchange: (event) => {
        const next = shapes.find((s) => s.type === event.target.value);
        const options = Object.fromEntries(next.options.map((o) => [o.name, o.default]));
        setGeometry({ source: "shape", shape: next.type, options, region_edits: [] });
      },
    },
    shapes.map((s) => h("option", { value: s.type, selected: s.type === shape.type }, s.label)),
  );
  geometry.options ??= {};
  return [
    h("div", { class: "field", dataset: { path: "geometry.shape" } }, h("div", { class: "label" }, "Shape"), select, h("div", { class: "help" }, shape.description)),
    h("div", { dataset: { path: "geometry.options" } }, optionsForm(shape.options, geometry.options, "geometry.options", () => editCase(() => scheduleGeometry(), "edit", { sections: ["geometry"] }))),
  ];
}

function svgForm(geometry) {
  const drop = h(
    "div",
    { class: "drop" },
    h("p", { style: "margin:0 0 8px" }, geometry.svg ? `${geometry.name ?? "drawing"} · ${formatNumber(geometry.svg.length)} characters` : "No drawing yet."),
    fileButton("Choose an SVG file…", ".svg,image/svg+xml"),
    h("p", { class: "note" }, "Or drop it on the picture. Each <path> is a run of curves; its region is its data-region or id."),
  );
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
  return [drop, h("details", { class: "report", style: "margin-top:8px" }, h("summary", {}, "SVG text"), h("div", { class: "body field", dataset: { path: "geometry.svg" } }, editor))];
}

function meshFileForm(geometry) {
  return [
    h(
      "div",
      { class: "drop" },
      h("p", { style: "margin:0 0 8px" }, geometry.text ? `${geometry.name ?? "mesh"} · ${geometry.format === "gmsh" ? "Gmsh" : ".pde2d"}` : "No mesh file yet."),
      fileButton("Choose a mesh file…", ".msh,.pde2d"),
      h("p", { class: "note" }, "Gmsh .msh (ASCII 4.1 or 2.2) or .pde2d. The Mesh step then refines it with the refiner and improvers; its discretizer and triangulator are not used. Its regions can be renamed and merged (not split: a mesh file has no curves)."),
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

// `canSplit`: false for a mesh file, which has no curves to split.
function regionEditor(region, regions, canSplit) {
  const others = regions.filter((r) => r !== region);
  const renameInput = h("input", { type: "text", value: region, "aria-label": "New name", style: "width:120px" });
  const mergeSelect = h(
    "select",
    { "aria-label": "Merge into" },
    others.map((r) => h("option", { value: r }, r)),
  );
  const fraction = h("input", { type: "number", step: "any", value: "0.5", min: "0", max: "1", "aria-label": "Fraction", style: "width:80px" });
  const first = h("input", { type: "text", value: `${region}_a`, "aria-label": "First name", style: "width:100px" });
  const second = h("input", { type: "text", value: `${region}_b`, "aria-label": "Second name", style: "width:100px" });
  return h(
    "div",
    { class: "card" },
    h("div", { class: "row" }, "Rename to", renameInput, h("button", { type: "button", class: "small", onclick: () => addEdit({ op: "rename", from: region, to: renameInput.value.trim() }) }, "Apply")),
    others.length ? h("div", { class: "row", style: "margin-top:6px" }, "Merge into", mergeSelect, h("button", { type: "button", class: "small", onclick: () => addEdit({ op: "merge", source: region, target: mergeSelect.value }) }, "Apply")) : null,
    canSplit ? h(
      "div",
      { class: "row", style: "margin-top:6px" },
      "Split at",
      fraction,
      "into",
      first,
      second,
      h("button", { type: "button", class: "small", onclick: () => addEdit({ op: "split", region, fraction: Number(fraction.value), first: first.value.trim(), second: second.value.trim() }) }, "Apply"),
    ) : null,
    canSplit ? h("p", { class: "note" }, "A split is measured along the boundary (domain on the left) from the start of the region.") : null,
  );
}

function regionsTable(view, canEdit, canSplit) {
  const names = view.regions.map((r) => r.name);
  const rows = view.regions.map((region, i) => {
    const editorSlot = h("td", { colspan: 4, style: "padding:0" });
    const row = h(
      "tr",
      {},
      h("td", {}, h("span", { class: "swatch", style: `background:var(${regionToken(i)})` })),
      h("td", {}, region.name),
      h("td", { class: "num" }, formatNumber(region.length)),
      h(
        "td",
        { class: "num" },
        canEdit
          ? h(
              "button",
              {
                type: "button",
                class: "small",
                onclick: () => {
                  if (editorSlot.childElementCount) fill(editorSlot);
                  else fill(editorSlot, regionEditor(region.name, names, canSplit));
                },
              },
              "Edit",
            )
          : null,
      ),
    );
    return [row, h("tr", {}, editorSlot)];
  });
  return h("table", { class: "data" }, h("thead", {}, h("tr", {}, h("th", {}, ""), h("th", {}, "Region"), h("th", { class: "num" }, "Length"), h("th", {}, ""))), h("tbody", {}, rows));
}

// --- the panel ------------------------------------------------------------------

export const geometryPanel = {
  build(container) {
    const geometry = store.caseData.geometry;
    const edits = geometry.region_edits ?? [];
    fill(container,
      h("h2", {}, "Geometry"),
      h("p", { class: "lead" }, "The domain: a ready-made shape, an SVG drawing with named boundary regions, or an existing mesh."),
      sourceSwitch(geometry),
      h("div", { style: "margin-top:12px" }, geometry.source === "shape" ? shapeForm(geometry) : geometry.source === "svg" ? svgForm(geometry) : meshFileForm(geometry)),
      edits.length
        ? h(
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
                // Only the last edit can be undone: an earlier one changes
                // the regions the later ones refer to.
                i === edits.length - 1
                  ? h("button", { type: "button", class: "small", "aria-label": "Undo the last region edit", onclick: () => replaceCase(undoRegionEdit(store.caseData)) }, "Undo")
                  : null,
              ),
            ),
          )
        : null,
      h("div", { class: "result", id: "geometry-result" }),
    );
    this.update(container);
  },

  update(container) {
    const slot = container.querySelector("#geometry-result");
    if (!slot) return;
    const error = currentError("geometry");
    const old = outdatedError("geometry");
    const fresh = freshResult("geometry");
    const view = fresh ?? (error ? null : lastResult("geometry"));
    const busy = store.busy.has("geometry");
    // Rebuild only when something shown changed (an open region editor
    // would otherwise close on every notification).
    if (slot.shown && slot.shown.view === view && slot.shown.error === error && slot.shown.old === old && slot.shown.busy === busy && slot.shown.fresh === fresh) return;
    slot.shown = { view, error, old, busy, fresh };
    const placed = markError(container, error);
    const children = [];
    if (error && !placed) children.push(errorBox(error));
    else if (error) children.push(h("p", { class: "note" }, "The geometry was refused; see the marked field."));
    if (old && !busy) children.push(errorBox(old, { outdated: true }));
    if (view) {
      const loops = view.kind === "curves" ? view.loops : [];
      const holes = loops.filter((l) => l.depth % 2 === 1).length;
      children.push(
        foldable(
          { id: "geometry.domain", title: "Domain", kind: "section" },
        h(
          "div",
          { class: "stats" },
          view.kind === "curves" ? [stat("Loops", loops.length), stat("Holes", holes)] : [stat("Nodes", formatNumber(view.nodes)), stat("Triangles", formatNumber(view.triangles))],
          stat("Domain size", formatNumber(view.domain_size)),
        ),
        h("p", { class: "note" }, "Relative lengths in the Mesh step multiply the domain size: the diagonal of the bounding box."),
        ),
        foldable({ id: "geometry.regions", title: "Boundary regions", kind: "section" }, regionsTable(view, true, view.kind === "curves")),
        view.kind === "curves"
          ? h(
              "div",
              { class: "row", style: "margin-top:12px" },
              exportButton("Export geometry .svg", () => downloadExport("geometry.svg", store.caseData).catch((e) => toast(e.message)), { fresh: fresh !== null, staleReason: "The geometry shown is outdated: wait for it to be read again." }),
            )
          : null,
      );
    } else if (!error && busy) {
      children.push(busyNote("geometry", "Reading the geometry"));
    }
    fill(slot, ...children);
  },
};

export { scheduleGeometry };
