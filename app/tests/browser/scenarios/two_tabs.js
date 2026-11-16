// Two tabs are two clients: each keeps its own run on the server.
export default async ({ example, q, select, click, until, text, check, openPage, pointer }) => {
  example("heat_annulus.json");
  await until(() => text(q("#toast")).includes("heat_annulus"), "the heat case");
  await until(() => q("#geometry-summary .stats"), "its geometry");
  click(q("#run-all"));
  await until(() => text(q("#toast")).includes("up to date"), "Run all", 90000);
  const other = await openPage();
  const item = await until(() => other.q('[data-example="poisson_rectangle.json"]'), "the second page");
  item.dispatchEvent(new other.w.MouseEvent("click", { bubbles: true }));
  await until(() => (other.q("#toast")?.textContent ?? "").includes("poisson_rectangle"), "the Poisson case in the second page");
  await until(() => other.q("#geometry-summary .stats"), "its geometry");
  other.q("#run-all").dispatchEvent(new other.w.MouseEvent("click", { bubbles: true }));
  await until(() => (other.q("#toast")?.textContent ?? "").includes("up to date"), "Run all in the second page", 90000);
  check("the second page has its static solution", other.q("#timebar").hidden);
  // The first page's run is still on the server: u(t) at a probe needs it.
  // A point inside the annulus (radii 1 and 0.4, fitted at 0.88 of the
  // picture), on the right at r = 0.7: away from its boundary, where a
  // click selects the region instead.
  const wrap = q("#canvas-wrap").getBoundingClientRect();
  const inside = 0.5 + (0.7 * 0.88 * Math.min(wrap.width, wrap.height)) / 2 / wrap.width;
  pointer("pointerdown", inside, 0.5);
  pointer("pointerup", inside, 0.5);
  await until(() => q(".probe-chart path.line") || /failed|no run/i.test(text(q("#toast"))), "u(t) at the probe", 30000);
  check("the first page's run is still its own", Boolean(q(".probe-chart path.line")), text(q("#toast")));
  check("the first page still shows its time run", !q("#timebar").hidden);
};
