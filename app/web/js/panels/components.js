// Forms for component slots from the catalog (docs/app.md, "The
// catalog"): a select of the components that fit the slot, the chosen
// component's description, its declared traits and its options. The Mesh
// and Solve steps build their cards from these.

import { foldable, h, optionsForm } from "../dom.js";
import { editCase } from "../state.js";

export function defaultChoice(component) {
  return { type: component.type, options: Object.fromEntries(component.options.map((o) => [o.name, o.default])) };
}

export function componentSelect(slot, choice, path, onPick, allowNone) {
  return h(
    "select",
    {
      "aria-label": slot.label,
      dataset: { path: `${path}.type` },
      onchange: (event) => onPick(event.target.value === "" ? null : defaultChoice(slot.components.find((c) => c.type === event.target.value))),
    },
    allowNone ? h("option", { value: "", selected: !choice }, "none") : null,
    slot.components.map((c) => h("option", { value: c.type, selected: choice?.type === c.type }, c.label)),
  );
}

// What a component declares about itself, as chips: the time orders a
// time method supports, whether a linear solver needs a symmetric matrix.
function traitChips(traits) {
  if (!traits) return null;
  const chips = [];
  if (Array.isArray(traits.time_orders)) chips.push(`time order ${traits.time_orders.join(", ")}`);
  if (traits.symmetric_matrix === true) chips.push("needs a symmetric matrix");
  if (traits.symmetric_matrix === false) chips.push("any square matrix");
  return chips.length ? h("div", { class: "row", style: "margin:4px 0" }, chips.map((text) => h("span", { class: "chip" }, text))) : null;
}

export function componentBody(slot, choice, path) {
  if (!choice) return [];
  const component = slot.components.find((c) => c.type === choice.type);
  if (!component) return [h("div", { class: "error-text" }, `Unknown component ${choice.type}`)];
  choice.options ??= {};
  return [
    h("div", { class: "help" }, component.description),
    traitChips(component.traits),
    component.options.length ? h("div", { dataset: { path: `${path}.options` } }, optionsForm(component.options, choice.options, `${path}.options`, () => editCase(() => {}, "edit", { sections: [path.split(".")[0]] }))) : null,
  ];
}

// A card for a slot that holds one component (or none, when optional).
// pick(next) stores the new choice in the case. It folds (the select
// stays in its header); `folded`: how it starts for a new viewer.
export function choiceCard(slot, choice, path, pick, { folded = false } = {}) {
  return foldable(
    {
      id: path,
      title: slot.label,
      folded,
      extras: [h("span", { class: "spacer" }), componentSelect(slot, choice, path, pick, slot.cardinality === "optional")],
      attributes: { class: "field", dataset: { path } },
    },
    h("div", { class: "help" }, slot.description),
    componentBody(slot, choice, path),
  );
}
