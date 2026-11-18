// The picture after a solve: field colours in its pixels, the legend
// names the field, the colour range locks and stays locked for the next
// solve, |∇u| and the line profile, zoom buttons.
export default async ({ example, q, qa, tab, click, select, type, until, idle, text, check, pixels, pointer, panel, button }) => {
  example("poisson_rectangle.json");
  await until(() => text(q("#toast")).includes("Opened"), "the example");
  await until(() => q("#geometry-summary .stats"), "its geometry");
  click(q("#run-all"));
  await until(() => text(q("#toast")).includes("up to date"), "Run all", 60000);
  click(q("#layer-mesh"));
  click(q("#layer-geometry"));
  await idle();
  const coloured = await until(() => (pixels().colours > 40 ? pixels() : null), "field colours in the picture");
  check("the picture shows the field's colours", coloured.colours > 40, String(coloured.colours));
  check("the Colour menu names the field", /field: u/.test(q('#color-by option[value="field"]').textContent));
  check("the legend names the field", /u at t = 0/.test(text(q("#legend"))), text(q("#legend")));
  click(button("Lock range", q("#legend")));
  const high = await until(() => qa("#legend .range-input")[1], "the range inputs");
  high.value = "10";
  high.dispatchEvent(new high.ownerDocument.defaultView.Event("change", { bubbles: true }));
  await until(() => qa("#legend .range-input")[1]?.value === "10", "the new range");
  check("the range is locked at the value typed", /locked/.test(text(q("#legend"))));
  // Solve again with another source: the range stays.
  tab("problem");
  const f = q('[data-path="problem.equation.text"] input');
  type(f, f.value.replace(/=\s*(.*)$/, "= ($1)*2"));
  tab("solve");
  click(button("Solve", panel("solve")));
  await until(() => /Up to date/.test(text(q("#panel-solve .step-status"))), "the new solution", 60000);
  check("the locked range is kept for the next run", qa("#legend .range-input")[1]?.value === "10", text(q("#legend")));
  click(button("Unlock", q("#legend")));
  await until(() => !q("#legend .range-input"), "the range to unlock");
  check("Unlock frees the range", button("Lock range", q("#legend")) !== undefined);
  // |∇u| per triangle.
  select(q("#color-by"), "gradient");
  await until(() => /\|∇u\|/.test(text(q("#legend"))), "the gradient legend");
  check("|∇u| colours the picture", pixels().colours > 5);
  // The line profile.
  select(q("#color-by"), "field");
  click(q("#profile-tool"));
  check("the profile tool is pressed", q("#profile-tool").getAttribute("aria-pressed") === "true");
  pointer("pointerdown", 0.2, 0.5);
  pointer("pointermove", 0.5, 0.5);
  pointer("pointermove", 0.8, 0.5);
  pointer("pointerup", 0.8, 0.5);
  await until(() => q(".profile-chart path.line"), "the profile chart");
  check("the profile names its field", /u at t = 0 along A → B/.test(text(q(".profile-chart"))), text(q(".profile-chart")));
  check("the tool ends after one line", q("#profile-tool").getAttribute("aria-pressed") === "false");
  check("no probe was pinned by the line", qa(".probes-table tbody tr").length === 0);
  click([...q(".profile-chart").querySelectorAll("button")].find((b) => /Remove line/.test(b.textContent)));
  await until(() => q(".profile-chart").hidden, "the line removed");
  // Zoom buttons.
  const before = pixels().hash;
  click(q("#zoom-in"));
  await until(() => pixels().hash !== before, "the zoom");
  check("Zoom in changes the picture", true);
  click(q("#fit-view"));
};
