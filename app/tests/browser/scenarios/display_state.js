// One resolved display: after the exact solution's expression changes,
// the picture, its legend, the Colour menu and the comparison controls
// all stop showing u − exact as current (the picture draws u, says why),
// and comparing again brings the difference back.
export default async ({ q, tab, panel, button, click, select, type, until, text, check }) => {
  tab("problem");
  const presets = q('#panel-problem select[aria-label="Problem preset"]');
  select(presets, [...presets.options].find((o) => /Laplace/.test(o.textContent)).value);
  // The status says "No errors" before the preset arrives: wait for it.
  await until(() => text(q("#toast")).includes("filled in"), "the preset");
  await until(() => /No errors/.test(text(q("#panel-problem .step-status"))), "the check");
  tab("solve");
  click(button("Solve", panel("solve")));
  await until(() => q("#solve-result .stats"), "the solution", 60000);
  const exact = () => q('#panel-solve input[aria-label="Exact solution"]');
  type(exact(), "x");
  click(button("Compare", panel("solve")));
  const section = () => q('.comparison[data-kind="exact"]');
  await until(() => section().querySelector(".compare-result .stat"), "the comparison", 30000);
  click(q("#show-exact"));
  const legendLabel = () => text(q("#legend .colorbar .bar-label"));
  await until(() => /^u − exact at t = /.test(legendLabel()), "the difference in the picture", 30000);
  check("u − exact is drawn and named so", /^u − exact/.test(legendLabel()), legendLabel());
  check("the Colour menu names it", /u − exact/.test(q('#color-by option[value="field"]').textContent));
  check("the colour bar has round ticks", q("#legend .bar-scale .ticks text") !== null);
  // The reproduced case: another expression in the case.
  type(exact(), "x + 1");
  await until(() => section().classList.contains("outdated-comparison"), "the comparison to go outdated");
  await until(() => /^u at t = /.test(legendLabel()), "the picture to stop showing u − exact", 10000);
  check("an outdated comparison is not drawn as current", /^u at t = /.test(legendLabel()), legendLabel());
  check("the legend says why", /previous expression/.test(text(q("#legend .legend-note"))), text(q("#legend")));
  check("the Colour menu agrees", q('#color-by option[value="field"]').textContent === "field: u", q('#color-by option[value="field"]').textContent);
  check("the checkbox keeps the choice and says why it is not drawn", q("#show-exact").checked && /previous expression/.test(text(section().querySelector(".difference-note"))));
  click(button("Compare", panel("solve")));
  await until(() => /^u − exact at t = /.test(legendLabel()), "the difference again", 30000);
  check("comparing again draws the difference again", !q("#legend .legend-note"), text(q("#legend")));
  // An edit of the solve's inputs: the result goes outdated, drawn greyed
  // with a stamp, never as current.
  tab("problem");
  type(q('#panel-problem [data-path^="problem.boundary_conditions."][data-path$=".text"] input'), "u = 0.25");
  await until(() => !q("#view-stamp").hidden, "the outdated stamp");
  check("an outdated result is stamped", /Outdated/.test(text(q("#view-stamp"))));
  check("and named outdated in the legend", /\(outdated\)/.test(legendLabel()), legendLabel());
  check("and draws no comparison", /^u at t = /.test(legendLabel()), legendLabel());
};
