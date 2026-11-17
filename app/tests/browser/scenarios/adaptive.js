// An adaptive solve of the L-shape: the cycle table and its chart, the
// solution and its error indicators drawn on the last (adapted) mesh, and
// the comparison with the exact solution adding each cycle's error.
export default async ({ example, q, qa, tab, click, select, until, idle, text, check, pixels, panel, button }) => {
  example("l_shape_adaptive.json");
  await until(() => text(q("#toast")).includes("l_shape_adaptive"), "the example");
  await until(() => q("#geometry-summary .stats"), "its geometry");
  click(q("#run-all"));
  await until(() => text(q("#toast")).includes("up to date"), "Run all", 90000);
  tab("solve");
  check("the adaptive mode is chosen", qa('#panel-solve input[name="solve-mode"]').some((r) => r.checked && /Adaptive/.test(r.parentElement.textContent)));
  await until(() => q("#solve-cycles .cycles-table"), "the cycle table");
  const rows = qa("#solve-cycles .cycles-table tbody tr");
  check("several cycles", rows.length > 5, String(rows.length));
  const nodes = rows.map((tr) => Number(tr.children[1].textContent.replace(/[^0-9]/g, "")));
  check("every cycle has more nodes", nodes.every((n, i) => i === 0 || n > nodes[i - 1]), nodes.join(", "));
  check("the stop reason", /tolerance met/.test(text(q("#solve-stats"))), text(q("#solve-stats")));
  check("a log–log chart with the N^(−1/2) slope", qa("#solve-cycles path.line.ref").length === 1);
  check("the adapted mesh can be exported", Boolean(button("Export adapted mesh", panel("solve"))));
  check("no reference section for an adaptive solve", !q('.comparison[data-kind="reference"]'));
  check("no refinement study for an adaptive solve", !q("#solve-study .study"));
  // The solution on the adapted mesh: its values fill the picture.
  click(q("#layer-mesh"));
  click(q("#layer-geometry"));
  await idle();
  const coloured = await until(() => (pixels().colours > 40 ? pixels() : null), "u in the picture");
  check("u colours the picture on the adapted mesh", coloured.colours > 40, String(coloured.colours));
  // The error indicators of the last cycle, per triangle.
  check("the indicator colouring is offered", !q('#color-by option[value="indicator"]').disabled);
  select(q("#color-by"), "indicator");
  await until(() => /log₁₀ η/.test(text(q("#legend"))), "the indicator legend");
  check("the indicators colour the picture", pixels().colours > 5);
  select(q("#color-by"), "field");
  // Compared with the exact solution: each cycle's error and its rate.
  click(button("Compare", panel("solve")));
  await until(() => qa("#solve-cycles .cycle-rate[data-of='l2']").length > 0, "each cycle's error", 60000);
  const rates = qa("#solve-cycles .cycle-rate[data-of='l2']")
    .map((td) => Number(td.textContent))
    .filter((v) => Number.isFinite(v));
  const late = rates.slice(-4);
  const mean = late.reduce((a, b) => a + b, 0) / late.length;
  check("the L2 error falls about like N^(−1)", mean > 0.7 && mean < 1.5, rates.join(", "));
  check("the chart adds the errors and the N^(−1) slope", qa("#solve-cycles path.line.ref").length === 2);
  click(q("#show-exact"));
  await until(() => /^u − exact/.test(text(q("#legend .colorbar .bar-label"))), "u − exact on the adapted mesh", 30000);
  check("u − exact is drawn on the adapted mesh", pixels().colours > 5);
  // The end state (screenshot): the indicators over the adapted mesh's
  // edges.
  click(q("#show-exact"));
  select(q("#color-by"), "indicator");
  click(q("#layer-mesh"));
  await until(() => /log₁₀ η/.test(text(q("#legend"))), "the indicator legend again");
  await idle();
};
