// Region actions right on the picture (docs/app.md, "The geometry
// editor"): a small card at the point clicked. For a region selected in
// the Geometry step: its name (Enter renames it), Merge into another,
// and Split (the split tool). Right after a split: the two new names,
// pre-filled, the second one selected for typing. Every change goes
// through the editor's region actions, so the conditions follow.

import { h } from "./dom.js";
import { mergeRegion, renameRegion, setTool } from "./geometry_editor.js";
import { freshResult, lastResult } from "./state.js";

let card = null;

function wrapElement() {
  return document.getElementById("canvas-wrap");
}

// Closes the card; names typed and not yet applied are applied, unless
// `keep` is false (Escape).
export function closeRegionPopover({ keep = true } = {}) {
  if (!card) return;
  const pending = card.pending;
  const had = card.contains(document.activeElement);
  card.pending = [];
  if (keep) for (const apply of pending) apply();
  card.remove();
  card = null;
  if (had) document.getElementById("view")?.focus({ preventScroll: true });
}

// A name field: Enter (or leaving it) renames the region it names now.
function nameField(name, label, onRenamed, pending) {
  let current = name;
  const input = h("input", { type: "text", value: name, "aria-label": label, class: "name-input", spellcheck: false, autocomplete: "off" });
  const apply = () => {
    const to = input.value.trim();
    if (to === "" || to === current) {
      input.value = current;
      return;
    }
    if (renameRegion(current, to)) {
      current = to;
      onRenamed?.(to);
    } else input.value = current;
  };
  input.addEventListener("keydown", (event) => {
    if (event.key === "Enter") {
      event.preventDefault();
      apply();
    }
  });
  input.addEventListener("change", apply);
  pending.push(apply);
  return input;
}

function place(element, screen) {
  const wrap = wrapElement();
  const margin = 8;
  const width = element.offsetWidth;
  const height = element.offsetHeight;
  let left = screen.x + 14;
  let top = screen.y + 14;
  if (left + width > wrap.clientWidth - margin) left = Math.max(margin, screen.x - width - 14);
  if (top + height > wrap.clientHeight - margin) top = Math.max(margin, screen.y - height - 14);
  element.style.left = `${left}px`;
  element.style.top = `${top}px`;
}

// screen: {x, y} in the picture's CSS pixels. Either {region} (actions on
// one region) or {split: {first, second}} (the names a split just made).
export function openRegionPopover(screen, { region = null, split = null }) {
  closeRegionPopover();
  const wrap = wrapElement();
  if (!wrap) return;
  const close = h("button", { type: "button", class: "icon close", "aria-label": "Close", title: "Close (Esc)", onclick: () => closeRegionPopover() }, "✕");
  const pending = [];
  let body;
  let focus;
  if (split) {
    const first = nameField(split.first, "First part's name", null, pending);
    const second = nameField(split.second, "Second part's name", null, pending);
    body = [h("div", { class: "pop-title" }, "Split into", close), h("div", { class: "pop-row" }, first), h("div", { class: "pop-row" }, second)];
    focus = second;
  } else {
    const names = ((freshResult("geometry") ?? lastResult("geometry"))?.regions ?? []).map((r) => r.name).filter((n) => n !== region);
    let current = region;
    const name = nameField(region, "Region name", (to) => (current = to), pending);
    const target = h("select", { "aria-label": "Merge into" }, names.map((n) => h("option", { value: n }, n)));
    body = [
      h("div", { class: "pop-title" }, h("span", { class: "visually-hidden" }, "Region"), name, close),
      names.length
        ? h(
            "div",
            { class: "pop-row" },
            target,
            h("button", { type: "button", class: "small", onclick: () => (closeRegionPopover({ keep: false }), mergeRegion(current, target.value)) }, "Merge into"),
          )
        : null,
      h(
        "div",
        { class: "pop-row" },
        h("button", { type: "button", class: "small", title: "Click the boundary where it should be split", onclick: () => (closeRegionPopover(), setTool("split")) }, "Split…"),
      ),
    ];
    focus = name;
  }
  card = h("div", { class: "region-pop", role: "dialog", "aria-label": split ? "Name the split regions" : `Region ${region}` }, body);
  card.pending = pending;
  card.addEventListener("keydown", (event) => {
    if (event.key === "Escape") {
      event.stopPropagation();
      closeRegionPopover({ keep: false });
    }
  });
  wrap.append(card);
  place(card, screen);
  focus.focus({ preventScroll: true });
  focus.select?.();
}

// A press elsewhere on the picture closes the card; so does another step.
export function wirePopoverClosing() {
  document.getElementById("view")?.addEventListener("pointerdown", () => closeRegionPopover(), true);
}
