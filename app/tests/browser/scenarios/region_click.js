// A click on a boundary selects its region (highlighted), and opens its
// condition in the Problem step, or its row in the Geometry step; the
// legend's region buttons do the same from the keyboard.
export default async ({ example, w, q, tab, click, until, text, check, pointer }) => {
  example("poisson_rectangle.json");
  await until(() => text(q("#case-name")).includes("poisson_rectangle"), "the example");
  await until(() => q("#geometry-summary .stats"), "its geometry");
  tab("mesh");
  // The rectangle (2 × 1) is fitted at 0.88 of the picture: its left edge.
  const wrap = q("#canvas-wrap").getBoundingClientRect();
  const scale = 0.88 * Math.min(wrap.width / 2, wrap.height / 1);
  const fx = (wrap.width / 2 - scale) / wrap.width;
  pointer("pointerdown", fx, 0.5);
  pointer("pointerup", fx, 0.5);
  await until(() => q("#panel-problem:not([hidden]) .selected-region"), "the condition of the clicked region");
  const card = q("#panel-problem .selected-region");
  check("the Problem step opens on the region's condition", card.dataset.path === "problem.boundary_conditions.left", card.dataset.path);
  check("its card is open", card.querySelector(".fold-toggle").getAttribute("aria-expanded") === "true");
  check("no probe was pinned", !q(".probes-table tbody tr"));
  const pressed = () => q('#legend button.region-item[aria-pressed="true"]');
  check("the legend marks the region", text(pressed()) === "left", text(pressed()));
  // From the keyboard, in the Geometry step: the row.
  tab("geometry");
  const right = [...q("#legend").querySelectorAll("button.region-item")].find((b) => text(b) === "right");
  // A button: Enter on it is its click.
  right.focus();
  click(right);
  await until(() => q('#geometry-regions tr.selected[data-region="right"]'), "the region's row");
  check("in the Geometry step, its row is selected", q("#panel-geometry").hidden === false);
  void w;
};
