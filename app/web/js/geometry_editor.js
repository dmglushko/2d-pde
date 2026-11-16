// The geometry editor on the picture (docs/app.md, "The geometry
// editor"): the tools bar, the split tool for every source with a
// boundary, and the drawing editor (select and drag vertices, handles and
// outlines; add rectangles, circles and polygons; snap to a grid). It is
// the viewer's overlay (view/viewer.js): drawn over the kept frame, so a
// pointer move redraws only the drawing and its handles, at frame rate,
// from the case's drawing (or the one being dragged). The case changes
// once per gesture (editCase: one undo step), and the server reads the
// geometry after it, debounced (scheduleGeometry). The math is
// pure/drawing.js.

import { api } from "./api.js";
import { scheduleGeometry, toast } from "./actions.js";
import { h } from "./dom.js";
import { applyDrawingChange, applyRegionEdit, renameRegionByEdit } from "./pure/case_model.js";
import * as D from "./pure/drawing.js";
import { regionToken } from "./pure/color.js";
import { formatNumber } from "./pure/format.js";
import { requestCase } from "./pure/stages.js";
import { closeRegionPopover, openRegionPopover } from "./region_popover.js";
import { currentError, editCase, freshResult, lastResult, requestTicket, sectionRevision, setInputGroup, setSelectedRegion, store } from "./state.js";
import { token } from "./view/tokens.js";

// Pointer reach of handles, vertices and edges, in CSS pixels (a finger
// needs more).
const REACH_PX = 8;
const TOUCH_REACH_PX = 16;
// A press that moves less than this is a click.
const CLICK_SLOP = 4;
// The grid is drawn only while its points are this far apart on screen.
const GRID_MIN_PX = 9;

const TOOLS = [
  { id: "select", label: "Select and drag", key: "v", icon: "M5 2.5v12l3.2-3.2 2.3 5 2-.9-2.3-4.9H15z", fill: true },
  { id: "split", label: "Split a region: click the boundary", key: "s", icon: "M4.5 5.5a2 2 0 1 0 0 .01M4.5 14.5a2 2 0 1 0 0 .01M6.2 6.6 16 13M6.2 13.4 16 7" },
  { id: "rectangle", label: "Add a rectangle: drag from corner to corner", key: "r", icon: "M3 5h14v10H3z", drawing: true },
  { id: "circle", label: "Add a circle: drag from its centre", key: "c", icon: "M10 3.5a6.5 6.5 0 1 0 0 13a6.5 6.5 0 1 0 0-13", drawing: true },
  { id: "polygon", label: "Add a polygon: click its corners, close on the first or press Enter", key: "p", icon: "M10 3l7 5-2.7 8H5.7L3 8z", drawing: true },
];

const editor = {
  viewer: null,
  bar: null,
  tool: "select",
  grid: false,
  // {loop, vertex} or {edges: [{loop, edge}]} or null.
  selection: null,
  // What the pointer is over (a hit of pure/drawing.js hitTest), or null.
  hover: null,
  // The split tool's marker: {point, region, corner, ...} or null.
  marker: null,
  // The gesture in progress: {kind, start (world), base (the drawing
  // when it started), ...} or null.
  gesture: null,
  // The drawing as the gesture has it (drawn instead of the case's).
  working: null,
  // The polygon being drawn: its points (world), or null.
  polygon: null,
  // The pointer in world coordinates (the polygon's rubber band).
  pointer: null,
  listeners: new Set(),
  // The live region that says what is selected (keyboard users).
  status: null,
};

// --- what is shown -----------------------------------------------------------------

const geometrySection = () => store.caseData?.geometry ?? null;
const isDrawing = () => geometrySection()?.source === "drawing";
// The geometry view the server drew last (it may be a step behind).
const geometryView = () => freshResult("geometry") ?? lastResult("geometry");

// Whether the editor is on: the Geometry step is open and the profile
// tool is not.
export function editorActive() {
  return store.step === "geometry" && store.view.tool !== "profile" && Boolean(geometrySection());
}

function ownsGeometry() {
  return editorActive() && isDrawing();
}

function currentDrawing() {
  return editor.working ?? (isDrawing() ? geometrySection() : null);
}

// The edges selected, as [{loop, edge}].
function selectedEdges() {
  return editor.selection?.edges ?? [];
}

export function editorSelection() {
  return editor.selection;
}

// listener(kind, detail): "selection" (the selection changed), "drag"
// ({loop, vertex, point} while a vertex moves).
export function onEditor(listener) {
  editor.listeners.add(listener);
}

function emit(kind, detail = null) {
  for (const listener of editor.listeners) listener(kind, detail);
}

function select(selection) {
  editor.selection = selection;
  emit("selection");
  announce();
  redraw();
}

// Says the selection (and a vertex's coordinates) in the live region.
function announce() {
  if (!editor.status) return;
  const drawing = isDrawing() ? geometrySection() : null;
  editor.status.textContent = drawing ? D.describeSelection(drawing, editor.selection, (x) => formatNumber(x, 10)) : "";
}

function redraw() {
  editor.viewer?.requestOverlay();
}

// Ends the gesture in progress without changing the case (Escape, a
// cancelled pointer, the geometry changed from elsewhere): the drawing
// and the inspector's fields show the case's drawing again. Returns
// whether there was one.
function cancelGesture() {
  if (!editor.gesture && !editor.working) return false;
  editor.gesture = null;
  editor.working = null;
  emit("selection");
  redraw();
  return true;
}

// A gesture belongs to the geometry it started on: an undo, a redo or any
// other change of the geometry section meanwhile ends it, so releasing
// the pointer never puts back a drawing the change replaced.
function gestureCurrent() {
  if (editor.gesture && editor.gesture.revision !== sectionRevision("geometry")) {
    cancelGesture();
    return false;
  }
  return Boolean(editor.gesture);
}

function startGesture(gesture) {
  editor.gesture = { ...gesture, revision: sectionRevision("geometry") };
}

function reach(event) {
  return (event?.pointerType === "touch" ? TOUCH_REACH_PX : REACH_PX) / (editor.viewer?.scale || 1);
}

// The grid step of the drawing (from the domain's size), or null when
// the grid is off.
export function gridStepNow() {
  const size = geometryView()?.domain_size ?? (() => {
    const box = currentDrawing() && D.drawingBounds(currentDrawing());
    return box ? Math.hypot(box.max[0] - box.min[0], box.max[1] - box.min[1]) : 1;
  })();
  return D.gridStep(size);
}

function snapped(point) {
  return editor.grid ? D.snapToGrid(point, gridStepNow()) : point;
}

// --- committing ------------------------------------------------------------------------

// The drawing becomes the case's geometry: one undo step, then the server
// reads it (debounced). `edit`: a region change the conditions follow
// ({op: "rename" | "merge" | "split", ...}).
function commit(drawing, edit = null) {
  editor.working = null;
  if (!drawing || drawing === geometrySection()) return redraw();
  const next = edit ? applyDrawingChange(store.caseData, drawing, edit) : null;
  editCase(
    (c) => {
      c.geometry = structuredClone(drawing);
      if (next) c.problem = next.problem;
    },
    "edit",
    { sections: next ? ["geometry", "problem"] : ["geometry"] },
  );
  scheduleGeometry();
  if (!D.selectionValid(drawing, editor.selection)) select(null);
  redraw();
}

// --- region actions (every source) ----------------------------------------------------

const regionNames = () => (isDrawing() ? D.drawingRegions(geometrySection()) : (geometryView()?.regions ?? []).map((r) => r.name));

function replaceCase(next) {
  editCase((c) => {
    for (const key of Object.keys(c)) delete c[key];
    Object.assign(c, next);
  }, "structure");
  scheduleGeometry();
}

// Region `from` named `to`, its condition with it. In a drawing its edges
// are renamed; otherwise a rename edit is added, or, right after a split
// that made `from`, that split takes the new name instead.
export function renameRegion(from, to) {
  const name = String(to ?? "").trim();
  if (name === "" || name === from) return false;
  if (isDrawing()) {
    if (D.drawingRegions(geometrySection()).includes(name)) {
      toast(`${name} already exists: merge into it instead.`);
      return false;
    }
    commit(D.renameRegion(geometrySection(), from, name), { op: "rename", from, to: name });
  } else {
    replaceCase(renameRegionByEdit(store.caseData, from, name));
  }
  if (store.selectedRegion === from) setSelectedRegion(name);
  return true;
}

// Region `source` merged into `target` (the target keeps its condition).
export function mergeRegion(source, target) {
  if (isDrawing()) commit(D.renameRegion(geometrySection(), source, target), { op: "merge", source, target });
  else replaceCase(applyRegionEdit(store.caseData, { op: "merge", source, target }));
  if (store.selectedRegion === source) setSelectedRegion(target);
}

// The split tool's click: the region at the marker split there, the new
// part named region_2 (or the next free name), and the names offered for
// editing right there. A drawing is split in the page (a vertex inserted
// where the marker is, or the vertex it snapped to); a shape, an SVG or a
// mesh file asks the server for the exact fraction or node
// (POST /api/geometry/locate) and adds a region edit.
async function splitAtMarker(marker, screen) {
  const names = regionNames();
  const first = marker.region;
  const second = D.uniqueName(names, `${marker.region}_2`);
  if (isDrawing()) {
    const next = D.splitRegion(geometrySection(), marker.loop, marker.edge, marker.t, first, second);
    if (!next) return toast(`That point is an end of ${first}: split it at a point inside.`);
    commit(next, { op: "split", region: first, first, second });
    openRegionPopover(screen, { split: { first, second } });
    return;
  }
  const ticket = requestTicket("locate", () => sectionRevision("geometry"));
  let answer;
  try {
    answer = await api.locate(requestCase(store.caseData, "geometry"), marker.point, reach() * 1.5);
  } catch (error) {
    if (ticket.live()) toast(error.message);
    return;
  }
  if (!ticket.holds()) return;
  if (!answer.splittable) return toast(`${answer.region}: ${answer.reason ?? "it cannot be split there"}.`);
  const region = answer.region;
  const other = D.uniqueName(names, `${region}_2`);
  const edit =
    answer.kind === "mesh_file"
      ? { op: "split_at_node", region, node: answer.node, first: region, second: other }
      : { op: "split", region, fraction: answer.fraction, first: region, second: other };
  replaceCase(applyRegionEdit(store.caseData, edit));
  openRegionPopover(screen, { split: { first: region, second: other } });
}

// "Edit as drawing": the server gives the case's curves, after its region
// edits, as a drawing, curve for curve; it replaces the geometry (one
// undo step). The regions do not change, so the conditions stay.
export async function editAsDrawing() {
  const ticket = requestTicket("drawing", () => sectionRevision("geometry"));
  let answer;
  try {
    answer = await api.toDrawing(requestCase(store.caseData, "geometry"));
  } catch (error) {
    if (ticket.live()) toast(error.message);
    return false;
  }
  if (!ticket.holds()) {
    if (ticket.live()) toast("The geometry changed meanwhile: press Edit as drawing again.");
    return false;
  }
  editCase((c) => {
    c.geometry = answer.geometry;
    if (c.view) {
      delete c.view.region_edit_undo;
      if (Object.keys(c.view).length === 0) delete c.view;
    }
  }, "structure");
  scheduleGeometry();
  setTool("select");
  select(null);
  return true;
}

// --- the drawing's own edits (the panel's inspector uses them) ---------------------------

export function setVertex(l, v, point) {
  commit(D.moveVertex(geometrySection(), l, v, point));
}

export function setEdgeKind(l, e, kind) {
  try {
    commit(D.setEdgeKind(geometrySection(), l, e, kind));
  } catch (error) {
    toast(`${error.message.charAt(0).toUpperCase()}${error.message.slice(1)}.`);
  }
}

// The selected edges in region `name` (an existing one or a new one): the
// way to split a region inside a drawing. A new name gets the condition
// of the region it came from when every edge came from one.
export function assignRegion(edges, name) {
  const region = String(name ?? "").trim();
  if (region === "" || edges.length === 0) return;
  const drawing = geometrySection();
  const from = [...new Set(edges.map((s) => drawing.loops[s.loop].edges[s.edge].region))];
  const isNew = !D.drawingRegions(drawing).includes(region);
  const next = D.assignRegion(drawing, edges, region);
  // A new region made from part of one region is a split of it: both
  // parts keep its condition.
  const stays = D.drawingRegions(next).includes(from[0]);
  const edit = isNew && from.length === 1 ? (stays ? { op: "split", region: from[0], first: from[0], second: region } : { op: "rename", from: from[0], to: region }) : null;
  commit(next, edit);
}

// The edge split at a typed fraction of its length, the part after the
// new vertex in region `name` (a new name copies the condition of the
// edge's region, as a split; an existing one merges it there).
export function splitEdgeTyped(l, e, fraction, name) {
  const region = String(name ?? "").trim();
  const drawing = geometrySection();
  const old = drawing.loops[l].edges[e].region;
  if (region === "") return toast("The new part needs a region name.");
  try {
    const next = D.splitEdgeAt(drawing, l, e, fraction, region);
    const isNew = !D.drawingRegions(drawing).includes(region);
    commit(next, isNew ? { op: "split", region: old, first: old, second: region } : null);
    select({ edges: [{ loop: l, edge: e + 1 }] });
  } catch (error) {
    toast(`${error.message.charAt(0).toUpperCase()}${error.message.slice(1)}.`);
  }
}

// Selects the first edge of `region` (the region table's typed split in
// a drawing); returns whether there is one.
export function selectEdges(region) {
  const drawing = geometrySection();
  for (const [l, loop] of D.loopsOf(drawing).entries())
    for (const [e, edge] of D.edgesOf(loop).entries())
      if (edge?.region === region) {
        select({ edges: [{ loop: l, edge: e }] });
        return true;
      }
  return false;
}

export function insertMidVertex(l, e) {
  const { drawing, vertex } = D.insertVertex(geometrySection(), l, e, 0.5);
  commit(drawing);
  select({ loop: l, vertex });
}

export function deleteVertexAt(l, v) {
  try {
    commit(D.deleteVertex(geometrySection(), l, v));
    select(null);
  } catch (error) {
    toast(`${error.message.charAt(0).toUpperCase()}${error.message.slice(1)}.`);
  }
}

export function deleteLoopAt(l) {
  try {
    commit(D.deleteLoop(geometrySection(), l));
    select(null);
    toast(`Loop ${l} deleted. Ctrl+Z brings it back.`);
  } catch (error) {
    toast(`${error.message.charAt(0).toUpperCase()}${error.message.slice(1)}.`);
  }
}

// --- tools ---------------------------------------------------------------------------------

export function setTool(tool) {
  if (editor.tool === tool) return;
  editor.tool = tool;
  editor.polygon = null;
  editor.gesture = null;
  editor.working = null;
  editor.marker = null;
  editor.hover = null;
  updateBar();
  redraw();
}

function toggleGrid() {
  editor.grid = !editor.grid;
  updateBar();
  redraw();
}

function updateBar() {
  const bar = editor.bar;
  if (!bar) return;
  const shown = editorActive() && geometryView() !== null;
  bar.hidden = !shown;
  if (!shown) return;
  const drawing = isDrawing();
  for (const button of bar.querySelectorAll("button[data-tool]")) {
    const tool = TOOLS.find((t) => t.id === button.dataset.tool);
    button.hidden = Boolean(tool.drawing) && !drawing;
    button.setAttribute("aria-pressed", String(editor.tool === tool.id));
  }
  for (const separator of bar.querySelectorAll(".sep")) separator.hidden = !drawing;
  const grid = bar.querySelector('[data-toggle="grid"]');
  grid.hidden = !drawing;
  grid.setAttribute("aria-pressed", String(editor.grid));
  grid.title = `Snap to a grid of ${gridStepNow()} (G)`;
  const canvas = editor.viewer.canvas;
  canvas.classList.toggle("tool-split", editor.tool === "split");
  canvas.classList.toggle("tool-add", ["rectangle", "circle", "polygon"].includes(editor.tool));
}

function icon(path, fill) {
  const ns = "http://www.w3.org/2000/svg";
  const svg = document.createElementNS(ns, "svg");
  svg.setAttribute("viewBox", "0 0 20 20");
  svg.setAttribute("aria-hidden", "true");
  const shape = document.createElementNS(ns, "path");
  shape.setAttribute("d", path);
  shape.setAttribute("fill", fill ? "currentColor" : "none");
  shape.setAttribute("stroke", "currentColor");
  shape.setAttribute("stroke-width", fill ? "0.8" : "1.6");
  shape.setAttribute("stroke-linejoin", "round");
  shape.setAttribute("stroke-linecap", "round");
  svg.append(shape);
  return svg;
}

function buildBar(wrap) {
  const button = (tool) =>
    h(
      "button",
      { type: "button", class: "icon tool", dataset: { tool: tool.id }, "aria-label": tool.label, title: `${tool.label} (${tool.key.toUpperCase()})`, "aria-pressed": "false", onclick: () => setTool(tool.id) },
      icon(tool.icon, tool.fill),
    );
  const grid = h(
    "button",
    { type: "button", class: "icon tool", dataset: { toggle: "grid" }, "aria-label": "Snap to the grid", "aria-pressed": "false", onclick: toggleGrid },
    icon("M5 5h.01M10 5h.01M15 5h.01M5 10h.01M10 10h.01M15 10h.01M5 15h.01M10 15h.01M15 15h.01", false),
  );
  const bar = h(
    "div",
    { class: "geometry-tools", role: "toolbar", "aria-label": "Geometry tools", hidden: true },
    button(TOOLS[0]),
    button(TOOLS[1]),
    h("span", { class: "sep", "aria-hidden": "true" }),
    TOOLS.slice(2).map(button),
    h("span", { class: "sep", "aria-hidden": "true" }),
    grid,
  );
  wrap.append(bar);
  return bar;
}

// --- the pointer -----------------------------------------------------------------------------

function world(v, p) {
  return v.toWorld(p.x, p.y);
}

// The split tool's marker under the pointer: on the drawing's exact
// curves, or on the server's view of a shape, an SVG or a mesh file.
function splitMarker(v, point, event) {
  const snap = reach(event) * 1.5;
  if (isDrawing()) return D.drawingSplitPoint(currentDrawing(), point, snap);
  return D.viewSplitPoint(geometryView(), point, snap);
}

function pointerdown(v, p, event) {
  if (!editorActive() || event.button !== 0) return false;
  const at = world(v, p);
  editor.pointer = at;
  if (editor.tool === "split") {
    const marker = splitMarker(v, at, event);
    // A press far from the boundary pans.
    if (!marker) return false;
    const [mx, my] = v.toScreen(...marker.point);
    if (Math.hypot(mx - p.x, my - p.y) > 3 * (event.pointerType === "touch" ? TOUCH_REACH_PX : REACH_PX)) return false;
    editor.marker = marker;
    startGesture({ kind: "split", screen: p, moved: false });
    redraw();
    return true;
  }
  if (!isDrawing()) return false;
  const drawing = geometrySection();
  if (editor.tool === "rectangle" || editor.tool === "circle") {
    startGesture({ kind: editor.tool, start: snapped(at), end: snapped(at), screen: p });
    return true;
  }
  if (editor.tool === "polygon") {
    startGesture({ kind: "polygon", screen: p });
    return true;
  }
  const hit = D.hitTest(drawing, at, reach(event), selectedEdges());
  if (!hit) return false;
  const base = drawing;
  if (hit.type === "vertex") {
    select({ loop: hit.loop, vertex: hit.vertex });
    startGesture({ kind: "vertex", loop: hit.loop, vertex: hit.vertex, start: at, origin: hit.point, base, screen: p, moved: false });
  } else if (hit.type === "control" || hit.type === "bend") {
    startGesture({ kind: hit.type, loop: hit.loop, edge: hit.edge, which: hit.which, start: at, origin: hit.point, base, screen: p, moved: false });
  } else if (hit.type === "move") {
    startGesture({ kind: "move", loop: hit.loop, start: at, base, screen: p, moved: false });
  } else if (hit.type === "edge") {
    const ref = { loop: hit.loop, edge: hit.edge };
    const has = selectedEdges().some((s) => s.loop === ref.loop && s.edge === ref.edge);
    if (event.shiftKey) select({ edges: has ? selectedEdges().filter((s) => !(s.loop === ref.loop && s.edge === ref.edge)) : [...selectedEdges(), ref] });
    else if (!has) select({ edges: [ref] });
    // Dragging an outline moves its loop.
    startGesture({ kind: "move", loop: hit.loop, start: at, base, screen: p, moved: false, fromEdge: true });
  }
  return true;
}

function pointermove(v, p, event) {
  if (!gestureCurrent()) return;
  const gesture = editor.gesture;
  const at = world(v, p);
  editor.pointer = at;
  if (!gesture.moved && Math.hypot(p.x - gesture.screen.x, p.y - gesture.screen.y) < CLICK_SLOP) return;
  gesture.moved = true;
  if (gesture.kind === "rectangle" || gesture.kind === "circle") gesture.end = snapped(at);
  else if (gesture.kind === "vertex") {
    const point = snapped([gesture.origin[0] + at[0] - gesture.start[0], gesture.origin[1] + at[1] - gesture.start[1]]);
    editor.working = D.moveVertex(gesture.base, gesture.loop, gesture.vertex, point);
    emit("drag", { loop: gesture.loop, vertex: gesture.vertex, point });
  } else if (gesture.kind === "control") {
    const point = snapped([gesture.origin[0] + at[0] - gesture.start[0], gesture.origin[1] + at[1] - gesture.start[1]]);
    editor.working = D.moveControl(gesture.base, gesture.loop, gesture.edge, gesture.which, point);
  } else if (gesture.kind === "bend") {
    editor.working = D.bendEdge(gesture.base, gesture.loop, gesture.edge, snapped(at));
  } else if (gesture.kind === "move") {
    let d = [at[0] - gesture.start[0], at[1] - gesture.start[1]];
    if (editor.grid) d = D.snapToGrid(d, gridStepNow());
    editor.working = D.moveLoop(gesture.base, gesture.loop, d);
  }
  redraw();
}

function pointerup(v, p, event, cancelled) {
  if (!gestureCurrent()) return;
  if (cancelled) return void cancelGesture();
  const gesture = editor.gesture;
  editor.gesture = null;
  const at = world(v, p);
  if (gesture.kind === "split") {
    if (!gesture.moved && editor.marker) splitAtMarker(editor.marker, p);
    return redraw();
  }
  if (gesture.kind === "rectangle" || gesture.kind === "circle") return finishShape(gesture, event);
  if (gesture.kind === "polygon") {
    if (!gesture.moved) addPolygonPoint(at, event);
    return redraw();
  }
  if (!gesture.moved) {
    editor.working = null;
    return redraw();
  }
  commit(editor.working);
  if (gesture.kind === "vertex") emit("selection");
}

// A rectangle or a circle drawn with the tool becomes a new loop (one
// region: a hole inside the domain, another piece outside), selected;
// the tool goes back to Select.
function finishShape(gesture, event) {
  const { start: a, end: b } = gesture;
  const tolerance = reach(event);
  const drawing = geometrySection();
  let loop = null;
  if (gesture.kind === "rectangle" && Math.abs(b[0] - a[0]) > tolerance && Math.abs(b[1] - a[1]) > tolerance) {
    const middle = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
    loop = D.rectangleLoop(a, b, D.newLoopRegion(drawing, middle));
  }
  if (gesture.kind === "circle") {
    const radius = Math.hypot(b[0] - a[0], b[1] - a[1]);
    if (radius > tolerance) loop = D.circleLoop(a, radius, D.newLoopRegion(drawing, a));
  }
  if (!loop) return redraw();
  addNewLoop(loop);
}

function addNewLoop(loop) {
  const next = D.addLoop(geometrySection(), loop);
  const l = next.loops.length - 1;
  commit(next);
  setTool("select");
  select({ edges: loop.edges.map((_, edge) => ({ loop: l, edge })) });
}

function addPolygonPoint(at, event) {
  const points = editor.polygon ?? [];
  const tolerance = reach(event);
  if (points.length >= 3 && Math.hypot(at[0] - points[0][0], at[1] - points[0][1]) <= tolerance) return closePolygon();
  // Snap to an existing vertex of the polygon being drawn is closing;
  // elsewhere the grid.
  editor.polygon = [...points, snapped(at)];
}

function closePolygon() {
  const points = editor.polygon;
  editor.polygon = null;
  if (!points || points.length < 3) {
    toast("A polygon needs three corners.");
    return redraw();
  }
  const middle = points.reduce((s, p) => [s[0] + p[0] / points.length, s[1] + p[1] / points.length], [0, 0]);
  addNewLoop(D.polygonLoop(points, D.newLoopRegion(geometrySection(), middle)));
}

function hover(v, p, event) {
  if (!editorActive()) {
    if (editor.hover || editor.marker) {
      editor.hover = null;
      editor.marker = null;
      redraw();
    }
    return;
  }
  const at = world(v, p);
  editor.pointer = at;
  if (editor.tool === "split") {
    editor.marker = splitMarker(v, at, event);
    // Only near the boundary.
    if (editor.marker) {
      const [sx, sy] = v.toScreen(...editor.marker.point);
      if (Math.hypot(sx - p.x, sy - p.y) > 3 * REACH_PX) editor.marker = null;
    }
    v.canvas.classList.toggle("over-boundary", Boolean(editor.marker));
    return redraw();
  }
  if (editor.tool === "polygon") return redraw();
  if (editor.tool !== "select" || !isDrawing()) return;
  const hit = D.hitTest(geometrySection(), at, reach(event), selectedEdges());
  const key = (x) => (x ? `${x.type}:${x.loop}:${x.vertex ?? x.edge}:${x.which ?? ""}` : "");
  v.canvas.classList.toggle("over-handle", Boolean(hit) && hit.type !== "edge");
  v.canvas.classList.toggle("over-edge", hit?.type === "edge");
  if (key(hit) === key(editor.hover)) return;
  editor.hover = hit;
  redraw();
}

function leave() {
  if (editor.hover || editor.marker) {
    editor.hover = null;
    editor.marker = null;
    redraw();
  }
}

// A click that no handle took: beside the drawing it clears the
// selection (no probe while the geometry is edited).
function click(v, p, event) {
  if (!ownsGeometry()) return false;
  void p;
  void event;
  if (editor.tool !== "select") return true;
  if (editor.selection) select(null);
  return true;
}

// A double click on an edge adds a vertex there.
function dblclick(v, p, event) {
  if (!ownsGeometry() || editor.tool !== "select") return editorActive() && editor.tool === "polygon";
  const hit = D.hitTest(geometrySection(), world(v, p), reach(event), []);
  if (hit?.type !== "edge") return false;
  if (hit.t <= 0 || hit.t >= 1) return true;
  const { drawing, vertex } = D.insertVertex(geometrySection(), hit.loop, hit.edge, hit.t);
  commit(drawing);
  select({ loop: hit.loop, vertex });
  return true;
}

// Keys on the picture: Escape cancels (a gesture, a polygon, a tool, the
// selection); Delete removes the selected vertex (or the loop of the
// selected edges); the arrows nudge the selected vertex (a grid step, or
// a pixel; Shift: ten); ] and [ select the next and previous vertex or
// edge of the loop, L and Shift+L the next and previous loop; Enter
// closes a polygon; V, S, R, C, P pick a tool, G the grid.
function key(v, event) {
  if (!editorActive() || event.ctrlKey || event.metaKey || event.altKey) return false;
  if (event.key === "Escape") {
    closeRegionPopover();
    if (cancelGesture()) return true;
    if (editor.polygon) editor.polygon = null;
    else if (editor.tool !== "select") setTool("select");
    else if (editor.selection) select(null);
    else return false;
    redraw();
    return true;
  }
  if (editor.tool === "polygon" && editor.polygon) {
    if (event.key === "Enter") {
      closePolygon();
      return true;
    }
    if (event.key === "Backspace" || event.key === "Delete") {
      editor.polygon = editor.polygon.slice(0, -1);
      if (editor.polygon.length === 0) editor.polygon = null;
      redraw();
      return true;
    }
  }
  const selection = editor.selection;
  if (isDrawing() && (event.key === "Delete" || event.key === "Backspace") && selection) {
    if (selection.vertex !== undefined) deleteVertexAt(selection.loop, selection.vertex);
    else {
      const loops = [...new Set(selection.edges.map((s) => s.loop))];
      if (loops.length === 1) deleteLoopAt(loops[0]);
    }
    return true;
  }
  const arrow = { ArrowLeft: [-1, 0], ArrowRight: [1, 0], ArrowUp: [0, 1], ArrowDown: [0, -1] }[event.key];
  if (arrow && isDrawing() && selection?.vertex !== undefined) {
    const step = (editor.grid ? gridStepNow() : 1 / v.scale) * (event.shiftKey ? 10 : 1);
    const vertex = D.verticesOf(D.loopsOf(geometrySection())[selection.loop])[selection.vertex];
    if (!D.isPoint(vertex)) return true;
    const point = [vertex[0] + arrow[0] * step, vertex[1] + arrow[1] * step];
    // Nudges of one vertex are one undo step until the picture loses
    // the focus.
    setInputGroup(`nudge:${selection.loop}:${selection.vertex}`);
    try {
      commit(D.moveVertex(geometrySection(), selection.loop, selection.vertex, editor.grid ? D.snapToGrid(point, gridStepNow()) : point));
    } finally {
      setInputGroup(null);
    }
    emit("selection");
    announce();
    return true;
  }
  // Selecting without a pointer: ] and [ step through the selected
  // loop's vertices and edges (vertex 0, edge 0, vertex 1, ...), L and
  // Shift+L through the loops.
  if (isDrawing() && editor.tool === "select" && (event.key === "]" || event.key === "[")) {
    const next = D.stepSelection(geometrySection(), editor.selection, event.key === "]" ? 1 : -1);
    if (next) select(next);
    return true;
  }
  if (isDrawing() && editor.tool === "select" && event.key.toLowerCase() === "l") {
    const next = D.stepLoop(geometrySection(), editor.selection, event.shiftKey ? -1 : 1);
    if (next) select(next);
    return true;
  }
  if (event.shiftKey) return false;
  const tool = TOOLS.find((t) => t.key === event.key.toLowerCase() && (!t.drawing || isDrawing()));
  if (tool) {
    setTool(tool.id);
    return true;
  }
  if (event.key.toLowerCase() === "g" && isDrawing()) {
    toggleGrid();
    return true;
  }
  return false;
}

// --- drawing ---------------------------------------------------------------------------------

function regionColor(name, names) {
  let index = names.indexOf(name);
  if (index < 0) index = names.length;
  return token(regionToken(index));
}

function strokeSamples(v, ctx, points) {
  ctx.beginPath();
  for (let i = 0; i < points.length; i += 2) {
    const [x, y] = v.toScreen(points[i], points[i + 1]);
    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  }
}

function drawGrid(v, ctx) {
  const step = gridStepNow();
  if (step * v.scale < GRID_MIN_PX) return;
  const { clientWidth: width, clientHeight: height } = v.canvas.parentElement;
  const [x0, y1] = v.toWorld(0, 0);
  const [x1, y0] = v.toWorld(width, height);
  ctx.fillStyle = token("--axis");
  for (let i = Math.ceil(x0 / step); i * step <= x1; i++)
    for (let j = Math.ceil(y0 / step); j * step <= y1; j++) {
      const [x, y] = v.toScreen(i * step, j * step);
      ctx.fillRect(x - 0.75, y - 0.75, 1.5, 1.5);
    }
}

function square(ctx, x, y, size) {
  ctx.beginPath();
  ctx.rect(x - size / 2, y - size / 2, size, size);
}

function diamond(ctx, x, y, size) {
  ctx.beginPath();
  ctx.moveTo(x, y - size);
  ctx.lineTo(x + size, y);
  ctx.lineTo(x, y + size);
  ctx.lineTo(x - size, y);
  ctx.closePath();
}

function drawDrawing(v, ctx) {
  const drawing = currentDrawing();
  if (!drawing) return;
  const names = (geometryView()?.regions ?? []).map((r) => r.name);
  for (const name of D.drawingRegions(drawing)) if (!names.includes(name)) names.push(name);
  const error = currentError("geometry");
  const bad = error ? D.drawingErrorTarget(error.path) : null;
  // An incomplete edge (a case read from a file) is not drawn: the
  // server's error names it.
  const samples = D.loopsOf(drawing).map((loop) =>
    D.edgesOf(loop).map((_, e) => {
      const curve = D.edgeCurve(loop, e);
      return curve ? D.sampleCurve(curve) : null;
    }),
  );
  // The fill (evenodd: holes stay empty), where the viewer would fill.
  if (v.fillWanted) {
    ctx.beginPath();
    for (const loop of samples) {
      let first = true;
      for (const points of loop)
        for (let i = 0; points && i < points.length; i += 2) {
          const [x, y] = v.toScreen(points[i], points[i + 1]);
          if (first) ctx.moveTo(x, y);
          else ctx.lineTo(x, y);
          first = false;
        }
      ctx.closePath();
    }
    ctx.fillStyle = token("--geometry-fill");
    ctx.fill("evenodd");
  }
  const selected = new Set(selectedEdges().map((s) => `${s.loop}:${s.edge}`));
  const hovered = editor.hover?.type === "edge" ? `${editor.hover.loop}:${editor.hover.edge}` : null;
  ctx.lineCap = "round";
  ctx.lineJoin = "round";
  samples.forEach((loop, l) =>
    loop.forEach((points, e) => {
      if (!points) return;
      const id = `${l}:${e}`;
      const region = drawing.loops[l].edges[e].region;
      const isBad = bad && bad.loop === l && (bad.edge === undefined || bad.edge === e);
      const highlight = selected.has(id) || (store.selectedRegion && store.selectedRegion === region) || (editor.marker && editor.tool === "split" && editor.marker.region === region);
      strokeSamples(v, ctx, points);
      if (highlight || isBad) {
        ctx.lineWidth = 8;
        ctx.strokeStyle = token("--isoline-halo");
        ctx.stroke();
      }
      ctx.lineWidth = highlight ? 4 : hovered === id ? 3.5 : 2;
      ctx.strokeStyle = isBad ? token("--critical") : regionColor(region, names);
      if (isBad) ctx.setLineDash([6, 4]);
      ctx.stroke();
      ctx.setLineDash([]);
    }),
  );
  if (editor.tool !== "select" && editor.tool !== "split") return drawVertices(v, ctx, drawing, bad, false);
  drawVertices(v, ctx, drawing, bad, editor.tool === "select");
  if (editor.tool !== "select") return;
  // The selected edges' handles, and the move handle of their loop.
  for (const s of selectedEdges()) {
    if (!drawing.loops[s.loop]?.edges[s.edge]) continue;
    const loop = drawing.loops[s.loop];
    for (const handle of D.edgeHandles(drawing, s.loop, s.edge)) {
      const [x, y] = v.toScreen(...handle.point);
      const isHover = editor.hover && editor.hover.type === handle.type && editor.hover.loop === s.loop && editor.hover.edge === s.edge && editor.hover.which === handle.which;
      if (handle.type === "control") {
        const n = loop.vertices.length;
        const anchor = handle.which === "control2" ? loop.vertices[(s.edge + 1) % n] : loop.vertices[s.edge];
        const anchors = handle.which === "control" ? [loop.vertices[s.edge], loop.vertices[(s.edge + 1) % n]] : [anchor];
        ctx.beginPath();
        for (const a of anchors) {
          ctx.moveTo(...v.toScreen(...a));
          ctx.lineTo(x, y);
        }
        ctx.lineWidth = 1;
        ctx.strokeStyle = token("--accent-text");
        ctx.stroke();
        diamond(ctx, x, y, isHover ? 6.5 : 5);
      } else {
        ctx.beginPath();
        ctx.arc(x, y, isHover ? 6 : 4.5, 0, 2 * Math.PI);
      }
      ctx.fillStyle = token("--surface");
      ctx.fill();
      ctx.lineWidth = 1.5;
      ctx.strokeStyle = token("--accent-text");
      ctx.stroke();
    }
  }
  const loops = [...new Set(selectedEdges().map((s) => s.loop))];
  if (loops.length === 1 && drawing.loops[loops[0]]) {
    const point = D.moveHandle(drawing, loops[0]);
    const [x, y] = v.toScreen(...point);
    const isHover = editor.hover?.type === "move";
    ctx.beginPath();
    ctx.arc(x, y, isHover ? 11 : 9.5, 0, 2 * Math.PI);
    ctx.fillStyle = token("--surface-2");
    ctx.fill();
    ctx.lineWidth = 1.5;
    ctx.strokeStyle = token("--accent-text");
    ctx.stroke();
    // A four-way arrow.
    ctx.beginPath();
    for (const [dx, dy] of [
      [1, 0],
      [-1, 0],
      [0, 1],
      [0, -1],
    ]) {
      ctx.moveTo(x + dx * 2, y + dy * 2);
      ctx.lineTo(x + dx * 6, y + dy * 6);
      ctx.moveTo(x + dx * 6 - dy * 2 - dx * 2, y + dy * 6 - dx * 2 - dy * 2);
      ctx.lineTo(x + dx * 6, y + dy * 6);
      ctx.lineTo(x + dx * 6 + dy * 2 - dx * 2, y + dy * 6 + dx * 2 - dy * 2);
    }
    ctx.lineWidth = 1.4;
    ctx.stroke();
  }
}

function drawVertices(v, ctx, drawing, bad, interactive) {
  const selection = editor.selection;
  D.loopsOf(drawing).forEach((loop, l) =>
    D.verticesOf(loop).forEach((point, k) => {
      if (!D.isPoint(point)) return;
      const [x, y] = v.toScreen(...point);
      const isSelected = selection?.vertex === k && selection.loop === l;
      const isHover = interactive && editor.hover?.type === "vertex" && editor.hover.loop === l && editor.hover.vertex === k;
      const isBad = bad && bad.loop === l && bad.vertex === k;
      square(ctx, x, y, isSelected || isHover ? 9 : interactive ? 7 : 5);
      ctx.fillStyle = isSelected ? token("--accent") : token("--surface");
      ctx.fill();
      ctx.lineWidth = 1.5;
      ctx.strokeStyle = isBad ? token("--critical") : isSelected || isHover ? token("--accent-text") : token("--ink-2");
      ctx.stroke();
    }),
  );
}

// The shape being added: a dashed outline in the accent colour.
function drawDraft(v, ctx) {
  const gesture = editor.gesture;
  ctx.setLineDash([5, 4]);
  ctx.lineWidth = 1.5;
  ctx.strokeStyle = token("--accent-text");
  if (gesture?.kind === "rectangle" && gesture.moved) {
    const [ax, ay] = v.toScreen(...gesture.start);
    const [bx, by] = v.toScreen(...gesture.end);
    ctx.strokeRect(Math.min(ax, bx), Math.min(ay, by), Math.abs(bx - ax), Math.abs(by - ay));
  }
  if (gesture?.kind === "circle" && gesture.moved) {
    const [cx, cy] = v.toScreen(...gesture.start);
    const radius = Math.hypot(gesture.end[0] - gesture.start[0], gesture.end[1] - gesture.start[1]) * v.scale;
    ctx.beginPath();
    ctx.arc(cx, cy, radius, 0, 2 * Math.PI);
    ctx.stroke();
  }
  if (editor.tool === "polygon" && editor.polygon) {
    const points = editor.polygon.map((p) => v.toScreen(...p));
    ctx.beginPath();
    points.forEach(([x, y], i) => (i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y)));
    if (editor.pointer) ctx.lineTo(...v.toScreen(...snapped(editor.pointer)));
    ctx.stroke();
    ctx.setLineDash([]);
    points.forEach(([x, y], i) => {
      square(ctx, x, y, i === 0 ? 9 : 6);
      ctx.fillStyle = i === 0 ? token("--accent") : token("--surface");
      ctx.fill();
      ctx.strokeStyle = token("--accent-text");
      ctx.stroke();
    });
  }
  ctx.setLineDash([]);
}

// The split tool's marker: a ring on the boundary, a diamond on a
// corner, over the region it would split.
function drawMarker(v, ctx) {
  const marker = editor.marker;
  if (!marker) return;
  if (!isDrawing()) {
    // The region on the server's view, highlighted.
    const view = geometryView();
    if (view?.kind === "curves") {
      ctx.beginPath();
      for (const loop of view.loops)
        for (const curve of loop.curves) {
          if (curve.region !== marker.region) continue;
          const q = curve.points;
          for (let i = 0; i < q.length; i += 2) {
            const [x, y] = v.toScreen(q[i], q[i + 1]);
            if (i === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
          }
        }
      ctx.lineCap = "round";
      ctx.lineJoin = "round";
      ctx.lineWidth = 5;
      ctx.strokeStyle = token("--accent-wash");
      ctx.stroke();
    }
  }
  const [x, y] = v.toScreen(...marker.point);
  ctx.lineWidth = 2;
  if (marker.corner || (marker.vertex ?? null) !== null) diamond(ctx, x, y, 7);
  else {
    ctx.beginPath();
    ctx.arc(x, y, 6, 0, 2 * Math.PI);
  }
  ctx.fillStyle = token("--surface");
  ctx.fill();
  ctx.strokeStyle = token("--ink");
  ctx.stroke();
  // A short cut across the boundary.
  ctx.beginPath();
  ctx.arc(x, y, 2, 0, 2 * Math.PI);
  ctx.fillStyle = token("--accent-text");
  ctx.fill();
}

function draw(v) {
  if (!editorActive()) return;
  const ctx = v.context;
  ctx.save();
  if (isDrawing() && editor.grid) drawGrid(v, ctx);
  if (ownsGeometry()) drawDrawing(v, ctx);
  if (editor.tool === "split") drawMarker(v, ctx);
  drawDraft(v, ctx);
  ctx.restore();
}

// --- wiring ------------------------------------------------------------------------------------

export function initEditor(viewer, wrap) {
  editor.viewer = viewer;
  editor.bar = buildBar(wrap);
  editor.status = h("div", { id: "editor-status", class: "visually-hidden", role: "status", "aria-live": "polite" });
  wrap.append(editor.status);
  viewer.setOverlay({
    draw,
    ownsGeometry,
    holdsView: () => ownsGeometry(),
    pointerdown,
    pointermove,
    pointerup,
    hover,
    click,
    dblclick,
    key,
    leave,
  });
}

// After any change of the store: the bar, the selection (an undo may
// have removed what it named) and the overlay.
export function updateEditor(reason) {
  if (!editor.viewer) return;
  if (reason === "case") {
    editor.selection = null;
    editor.polygon = null;
    editor.gesture = null;
    editor.working = null;
    closeRegionPopover();
  }
  // The selection stays while another step is open; it goes with the
  // drawing, or when an edit (an undo) removed what it named.
  if (!isDrawing()) {
    if (editor.selection) select(null);
    if (["rectangle", "circle", "polygon"].includes(editor.tool)) editor.tool = "select";
  } else if (!D.selectionValid(geometrySection(), editor.selection)) select(null);
  if (!editorActive()) {
    editor.polygon = null;
    cancelGesture();
  } else gestureCurrent();
  if (!editorActive()) closeRegionPopover();
  updateBar();
}
