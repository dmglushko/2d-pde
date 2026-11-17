// The picture: what is shown (set), the view transform (fit, zoom, pan)
// and the order of the layers (draw). The layers are drawn by layers.js,
// the legend and the saved picture by legend.js, and the pointer, touch
// and keyboard by interaction.js.

import { fieldScale, lockedScale, nodePositions } from "../pure/color.js";
import { gradientMagnitudes, isolineLevels, isolines, meshGrid, triangleAngles, viewTransform } from "../pure/mesh_math.js";
import { settings } from "../settings.js";
import { listen } from "./interaction.js";
import * as layers from "./layers.js";
import { legendData, legendNodes, picture } from "./legend.js";
import { colorTables, tableOf, token } from "./tokens.js";

// A pan or zoom shows the last full picture moved and scaled; the full
// picture is drawn again once the gesture pauses this long.
const SETTLE_MS = 120;
// Edges over a field's colours are drawn at this opacity (of the
// --mesh-edge token's): the colours are what is read there.
const EDGES_OVER_FIELD = 0.45;
// A field built for an earlier case is drawn at this opacity over the
// surface: greyed, never as current.
const OUTDATED_ALPHA = 0.4;

const sameLayers = (a = {}, b = {}) =>
  a.geometry === b.geometry && a.mesh === b.mesh && a.dirichlet === b.dirichlet && a.isolines === b.isolines && a.isolineCount === b.isolineCount;
const sameField = (a, b) => a === b || (a && b && a.values === b.values && a.scale === b.scale && a.label === b.label && a.indicators === b.indicators);

export class Viewer {
  // callbacks: onPin(x, y) for a click (or Enter) at a point;
  // onProfile({from, to}) for a line drawn with the profile tool;
  // onLock(quantity, {low, high} or null) from the legend's range lock;
  // onRegion(name) for a click on a boundary region (or its legend entry).
  constructor(canvas, tooltip, status, callbacks = {}) {
    this.canvas = canvas;
    this.callbacks = callbacks;
    this.tooltip = tooltip;
    this.status = status;
    this.context = canvas.getContext("2d");
    this.scale = 1;
    this.offset = { x: 0, y: 0 };
    this.data = {};
    this.fittedKey = null;
    this.offscreen = document.createElement("canvas");
    this.image = null;
    // The last full picture and the view it was drawn at.
    this.frame = document.createElement("canvas");
    this.frameView = null;
    this.blitRequest = null;
    this.settleTimer = null;
    // The profile line being drawn ({from, to}) or null.
    this.draft = null;
    // What is drawn over the picture and takes the pointer first (the
    // geometry editor, geometry_editor.js), or null: {draw(v),
    // ownsGeometry(), holdsView(), pointerdown/move/up, hover, click,
    // dblclick, key, leave}. It is drawn over the kept frame, so a
    // pointer move redraws only it (redrawOverlay), never the mesh or a
    // field.
    this.overlay = null;
    this.overlayRequest = null;

    new ResizeObserver(() => this.resize()).observe(canvas.parentElement);
    listen(this);
  }

  // data: {geometry (view), mesh (mesh view), meshStale, dirichlet (node
  // indices), field ({name, when, label, values, scale?}), colorBy,
  // locked ({field, gradient}: {low, high} or null), tool, layers
  // ({geometry, mesh, dirichlet, isolines, isolineCount}), probes,
  // profile, selectedRegion (a region name or null), legendNote (why a
  // comparison the user asked for is not drawn, or null)}. A field with
  // `outdated` is drawn greyed. Draws only when something shown changed; returns whether it
  // did (the legend changes only then).
  set(data) {
    const previous = this.data;
    const keys = ["geometry", "mesh", "meshStale", "dirichlet", "colorBy", "probes", "profile", "locked", "tool", "selectedRegion", "legendNote"];
    const changed = keys.some((k) => previous[k] !== data[k]) || !sameLayers(previous.layers, data.layers) || !sameField(previous.field, data.field);
    if (!changed) return false;
    const meshChanged = data.mesh !== previous.mesh;
    const colourChanged = meshChanged || !sameField(previous.field, data.field) || previous.colorBy !== data.colorBy || previous.locked !== data.locked;
    const linesChanged = previous.layers?.isolines !== data.layers?.isolines || previous.layers?.isolineCount !== data.layers?.isolineCount;
    this.data = data;
    this.canvas.classList.toggle("profile-tool", data.tool === "profile");
    if (meshChanged) this.prepareMesh();
    if (colourChanged) this.prepareField();
    if (colourChanged || linesChanged) this.prepareIsolines();
    const box = this.boundingBox();
    const key = box ? JSON.stringify(box) : null;
    if (key && key !== this.fittedKey) {
      // A drawing being edited keeps the view the user works in.
      const keep = this.fittedKey !== null && this.overlay?.holdsView?.();
      this.fittedKey = key;
      if (keep) this.draw();
      else this.fit();
    } else this.draw();
    return true;
  }

  // What does not depend on the view, once per mesh.
  prepareMesh() {
    const mesh = this.data.mesh;
    this.angles = mesh ? triangleAngles(mesh) : null;
    this.grid = mesh ? meshGrid(mesh) : null;
    this.qualityBuckets = mesh
      ? layers.bucketTriangles(this.angles.length, (k) => 1 - Math.min(this.angles[k], layers.QUALITY_MAX_DEGREES) / layers.QUALITY_MAX_DEGREES)
      : null;
  }

  // ... and once per field and colouring: whether the field has a value
  // at every node, the scale shown (the locked range, else the field's
  // own), the colour position of each node's value, and for |∇u| the
  // gradient per triangle.
  prepareField() {
    const { field, mesh, colorBy, locked = {} } = this.data;
    this.fieldComplete = Boolean(field && mesh && field.values.length * 2 === mesh.nodes.length && Array.prototype.every.call(field.values, (v) => Number.isFinite(v)));
    this.positions = null;
    this.gradients = null;
    this.gradientBuckets = null;
    this.indicatorLogs = null;
    this.shownScale = null;
    if (!field) return;
    if (colorBy === "indicator") {
      // log10 of each triangle's indicator: they span decades. A zero
      // indicator (nothing to estimate there) takes the lowest colour.
      const eta = field.indicators;
      if (!eta || !mesh || eta.length * 3 !== mesh.triangles.length) return;
      const logs = new Float64Array(eta.length);
      let low = Infinity;
      let high = -Infinity;
      for (let k = 0; k < eta.length; k++) {
        logs[k] = eta[k] > 0 ? Math.log10(eta[k]) : -Infinity;
        if (Number.isFinite(logs[k])) {
          low = Math.min(low, logs[k]);
          high = Math.max(high, logs[k]);
        }
      }
      if (!Number.isFinite(low)) low = high = 0;
      for (let k = 0; k < logs.length; k++) if (!Number.isFinite(logs[k])) logs[k] = low;
      const scale = (locked.indicator && lockedScale(locked.indicator.low, locked.indicator.high)) ?? fieldScale([low, high]);
      this.shownScale = scale;
      this.indicatorLogs = logs;
      this.gradientBuckets = layers.bucketTriangles(logs.length, (k) => scale.position(logs[k]));
      return;
    }
    if (colorBy === "gradient") {
      if (!this.fieldComplete) return;
      this.gradients = gradientMagnitudes(mesh.nodes, mesh.triangles, field.values);
      let largest = 0;
      for (const g of this.gradients) if (g > largest) largest = g;
      const scale = (locked.gradient && lockedScale(locked.gradient.low, locked.gradient.high)) ?? fieldScale([0, largest]);
      this.shownScale = scale;
      this.gradientBuckets = layers.bucketTriangles(this.gradients.length, (k) => scale.position(this.gradients[k]));
      return;
    }
    this.shownScale = (locked.field && lockedScale(locked.field.low, locked.field.high)) ?? field.scale ?? fieldScale(field.values);
    if (this.fieldComplete) this.positions = nodePositions(field.values, this.shownScale);
  }

  // ... and the isolines of a complete field, at levels evenly inside the
  // scale shown (the scale of every snapshot, for a solution).
  prepareIsolines() {
    this.isolines = null;
    const { field, mesh, layers: shown = {}, colorBy } = this.data;
    if (!shown.isolines || !this.fieldComplete || colorBy === "gradient" || colorBy === "indicator") return;
    const scale = this.shownScale ?? field.scale ?? fieldScale(field.values);
    const levels = isolineLevels(scale.low, scale.high, shown.isolineCount ?? 10);
    this.isolines = { levels, segments: isolines(mesh.nodes, mesh.triangles, field.values, levels), positions: levels.map((v) => scale.position(v)), scale };
  }

  // The pixel ratio the picture is drawn at: the screen's, at most the
  // "picture quality" setting.
  pixelRatio() {
    return Math.min(window.devicePixelRatio || 1, settings.pixelRatioCap);
  }

  boundingBox() {
    const g = this.data.geometry?.bounding_box;
    if (g) return g;
    const mesh = this.data.mesh;
    if (!mesh || mesh.nodes.length === 0) return null;
    let min = [Infinity, Infinity];
    let max = [-Infinity, -Infinity];
    for (let i = 0; i < mesh.nodes.length; i += 2) {
      min = [Math.min(min[0], mesh.nodes[i]), Math.min(min[1], mesh.nodes[i + 1])];
      max = [Math.max(max[0], mesh.nodes[i]), Math.max(max[1], mesh.nodes[i + 1])];
    }
    return { min, max };
  }

  resize() {
    const ratio = this.pixelRatio();
    const { clientWidth: width, clientHeight: height } = this.canvas.parentElement;
    this.canvas.width = Math.max(1, Math.round(width * ratio));
    this.canvas.height = Math.max(1, Math.round(height * ratio));
    if (this.fittedKey && !this.userMoved) this.fit();
    else this.draw();
  }

  fit() {
    const box = this.boundingBox();
    const { clientWidth: width, clientHeight: height } = this.canvas.parentElement;
    if (!box || width === 0) return this.draw();
    const w = Math.max(box.max[0] - box.min[0], 1e-12);
    const hgt = Math.max(box.max[1] - box.min[1], 1e-12);
    this.scale = 0.88 * Math.min(width / w, height / hgt);
    this.offset = {
      x: width / 2 - ((box.min[0] + box.max[0]) / 2) * this.scale,
      y: height / 2 + ((box.min[1] + box.max[1]) / 2) * this.scale,
    };
    this.userMoved = false;
    this.draw();
  }

  // World (x up) to CSS pixels (y down), and back.
  toScreen(x, y) {
    return [this.offset.x + x * this.scale, this.offset.y - y * this.scale];
  }
  toWorld(px, py) {
    return [(px - this.offset.x) / this.scale, (this.offset.y - py) / this.scale];
  }

  // Zooms by `factor` keeping the point at (px, py) (CSS pixels) in place.
  zoomAt(px, py, factor) {
    const [wx, wy] = this.toWorld(px, py);
    this.scale *= factor;
    this.offset = { x: px - wx * this.scale, y: py + wy * this.scale };
    this.viewMoved();
  }

  // Zooms about the centre of the picture (the zoom buttons, + and -).
  zoomBy(factor) {
    const { clientWidth: width, clientHeight: height } = this.canvas.parentElement;
    this.zoomAt(width / 2, height / 2, factor);
  }

  // --- drawing ---------------------------------------------------------------

  // True when the picture is coloured by a field (its colours are what is
  // read: edges go fainter).
  colouredByField() {
    const { colorBy, field } = this.data;
    return Boolean(field) && !field.outdated && (colorBy === "field" || (colorBy === "gradient" && this.gradients) || (colorBy === "indicator" && this.indicatorLogs));
  }

  draw() {
    this.cancelGesture();
    const ctx = this.context;
    const ratio = this.canvas.width / Math.max(1, this.canvas.parentElement.clientWidth);
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.fillStyle = token("--surface");
    ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    this.ratio = ratio;

    const { geometry, mesh, meshStale, layers: shown = {}, colorBy, field } = this.data;
    const hasMesh = mesh && mesh.triangles.length > 0;
    const current = hasMesh && !meshStale;
    // A field built for an earlier case is drawn greyed (an "outdated"
    // stamp is over the picture, main.js), on the mesh it was built on.
    const fieldShown = hasMesh && Boolean(field) && (current || field.outdated);
    let note = "";

    // The editor draws a drawing it edits itself (from the case, at
    // pointer speed); the server's view of it may be a step behind.
    const edited = Boolean(this.overlay?.ownsGeometry?.());
    this.fillWanted = !current && !fieldShown;
    if (geometry?.kind === "curves" && !current && !fieldShown && !edited) layers.fillGeometry(this, geometry);
    if (current && colorBy === "quality") layers.drawQuality(this, mesh);
    if (fieldShown) {
      ctx.globalAlpha = field.outdated ? OUTDATED_ALPHA : 1;
      if (colorBy === "field" && this.positions) layers.rasterField(this, mesh, this.positions, tableOf(this.shownScale));
      if ((colorBy === "gradient" || colorBy === "indicator") && this.gradientBuckets) layers.fillBuckets(this, mesh, this.gradientBuckets, tableOf(this.shownScale));
      ctx.globalAlpha = 1;
      if (colorBy === "gradient" && !this.gradients) note = "|∇| needs a value at every node";
      if (colorBy === "indicator" && !this.indicatorLogs) note = "error indicators come from an adaptive solve";
    }
    if (hasMesh && shown.mesh) {
      const alpha = meshStale ? 0.35 : this.colouredByField() ? EDGES_OVER_FIELD : 1;
      if (!layers.drawEdges(this, mesh, alpha)) note = "edges hidden at this zoom";
    }
    if (shown.geometry) {
      if (current) layers.drawMeshBoundary(this, mesh);
      else if (geometry?.kind === "curves" && !edited) layers.drawCurves(this, geometry);
      else if (geometry?.kind === "mesh_file") layers.drawEdgeList(this, geometry);
    }
    if (this.data.selectedRegion && !edited) layers.drawSelectedRegion(this, current ? mesh : null, geometry, this.data.selectedRegion);
    if (fieldShown && this.isolines) layers.drawIsolines(this, this.isolines, tableOf(this.isolines.scale), colorBy === "field");
    if (fieldShown && colorBy === "field" && !this.fieldComplete) layers.drawFieldDots(this, mesh, field.values, this.shownScale, tableOf(this.shownScale));
    if (current && shown.dirichlet && this.data.dirichlet) layers.drawNodes(this, mesh, this.data.dirichlet);
    if (this.data.probes?.length) layers.drawProbes(this, this.data.probes);
    const line = this.draft ?? this.data.profile;
    if (line) layers.drawProfileLine(this, line);
    if (meshStale) note = fieldShown ? "outdated: run again to update" : "mesh outdated: run the Mesh step";
    if (this.data.tool === "profile" && !note) note = "drag across the picture to draw the profile line";
    this.status.textContent = note;
    this.keepFrame();
    this.overlay?.draw(this);
  }

  // --- the overlay -------------------------------------------------------------

  setOverlay(overlay) {
    this.overlay = overlay;
    this.draw();
  }

  // The overlay again over the kept frame (the picture under it did not
  // change), at most once per animation frame. A timer stands in where
  // animation frames do not run (a hidden or headless page).
  requestOverlay() {
    if (this.overlayRequest !== null) return;
    const run = () => {
      if (this.overlayRequest === null) return;
      cancelAnimationFrame(this.overlayRequest.frame);
      clearTimeout(this.overlayRequest.timer);
      this.overlayRequest = null;
      this.redrawOverlay();
    };
    this.overlayRequest = { frame: requestAnimationFrame(run), timer: setTimeout(run, 50) };
  }

  redrawOverlay() {
    const view = this.frameView;
    if (!view || this.frame.width !== this.canvas.width || this.frame.height !== this.canvas.height) return this.draw();
    if (view.scale !== this.scale || view.offset.x !== this.offset.x || view.offset.y !== this.offset.y) return this.blit();
    const ctx = this.context;
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.drawImage(this.frame, 0, 0);
    ctx.setTransform(this.ratio, 0, 0, this.ratio, 0, 0);
    this.overlay?.draw(this);
  }

  // --- gestures --------------------------------------------------------------

  keepFrame() {
    this.frame.width = this.canvas.width;
    this.frame.height = this.canvas.height;
    this.frame.getContext("2d").drawImage(this.canvas, 0, 0);
    this.frameView = { scale: this.scale, offset: { ...this.offset } };
  }

  cancelGesture() {
    if (this.blitRequest !== null) cancelAnimationFrame(this.blitRequest);
    this.blitRequest = null;
    clearTimeout(this.settleTimer);
    this.settleTimer = null;
  }

  // The view moved (pan or zoom): at most one frame per animation frame,
  // the last full picture moved and scaled; the full picture once the
  // gesture pauses.
  viewMoved() {
    this.userMoved = true;
    if (!this.frameView || this.frame.width !== this.canvas.width || this.frame.height !== this.canvas.height) return this.draw();
    if (this.blitRequest === null)
      this.blitRequest = requestAnimationFrame(() => {
        this.blitRequest = null;
        this.blit();
      });
    clearTimeout(this.settleTimer);
    this.settleTimer = setTimeout(() => this.draw(), SETTLE_MS);
  }

  blit() {
    const ctx = this.context;
    const r = this.ratio;
    const t = viewTransform(this.frameView, { scale: this.scale, offset: this.offset });
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.fillStyle = token("--surface");
    ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);
    ctx.drawImage(this.frame, t.x * r, t.y * r, this.frame.width * t.scale, this.frame.height * t.scale);
    ctx.setTransform(r, 0, 0, r, 0, 0);
    this.overlay?.draw(this);
  }

  // --- the legend ------------------------------------------------------------

  legendData() {
    return legendData(this);
  }

  // The legend's elements (regions, the colour bar with its range lock,
  // the Dirichlet nodes).
  legend() {
    return legendNodes(this.legendData(), this.callbacks.onLock, this.callbacks.onRegion);
  }

  // The picture as it is shown, with its legend underneath, as a PNG blob.
  picture() {
    this.draw();
    return picture(this);
  }

  // The colour tables (the saved picture and the legend draw them).
  colorTables() {
    return colorTables();
  }
}
