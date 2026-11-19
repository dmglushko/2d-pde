// An adaptive solve of the L-shape: the cycle table and its chart, the
// solution and its error indicators drawn on the last (adapted) mesh, the
// comparison with the exact solution adding each cycle's error (its lines
// hidden and shown from the chart's key, then removed), an earlier
// cycle's solution on its own mesh, and a mode switch that leaves the
// settings and the picture where they were.
export default async ({ example, q, qa, tab, click, select, until, idle, text, check, pixels, panel, button }) => {
  example("l_shape_adaptive.json");
  await until(() => text(q("#case-name")).includes("l_shape_adaptive"), "the example");
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
  click(q("#show-exact"));
  await idle();

  // The chart's key hides and shows its lines.
  const keyButton = (name) => qa("#solve-cycles .chart-key .key-toggle").find((b) => text(b) === name);
  const l2Lines = () => qa("#solve-cycles path.line.s2").length;
  check("the L2 error is drawn", l2Lines() === 1);
  click(keyButton("L2 error"));
  await idle();
  check("its name hides it", l2Lines() === 0 && keyButton("L2 error").getAttribute("aria-pressed") === "false");
  click(keyButton("L2 error"));
  await idle();
  check("and shows it again", l2Lines() === 1 && keyButton("L2 error").getAttribute("aria-pressed") === "true");

  // An earlier cycle in the picture: its own (coarser) mesh and solution.
  const lastLegend = text(q("#legend .colorbar .bar-label"));
  const pick = (n) => qa("#solve-cycles .cycle-pick").find((b) => text(b) === String(n));
  click(pick(2));
  await until(() => /in cycle 2 of/.test(text(q("#legend"))), "cycle 2 in the legend", 30000);
  check("the row of cycle 2 is marked", pick(2).getAttribute("aria-pressed") === "true" && pick(2).closest("tr").classList.contains("shown-cycle"));
  check("the note says which cycle is shown", /shows cycle 2 of/.test(text(q("#solve-cycles .cycle-shown"))), text(q("#solve-cycles .cycle-shown")));
  check("u colours the picture on cycle 2's mesh", pixels().colours > 40);
  click(button("Show the last", panel("solve")));
  await until(() => text(q("#legend .colorbar .bar-label")) === lastLegend, "the last cycle again");
  check("the last cycle is shown again", /shows the last cycle/.test(text(q("#solve-cycles .cycle-shown"))));

  // The exact solution removed: no errors in the table or the chart.
  click(button("Remove", panel("solve")));
  await until(() => qa("#solve-cycles .cycle-rate[data-of='l2']").length === 0, "the errors to go");
  check("the expression is gone", q('#panel-solve input[aria-label="Exact solution"]').value === "");
  check("the chart has the estimate only", qa("#solve-cycles path.line.s2").length === 0 && qa("#solve-cycles path.line.ref").length === 1);

  // The mode's cells fill the row (none centred inside a taller one).
  const cells = qa('#panel-solve .segmented .choice').map((c) => c.getBoundingClientRect().height);
  check("the mode's cells are as tall as each other", Math.max(...cells) - Math.min(...cells) < 1, cells.join(", "));
  // Another mode outdates the result: the choice and the picture stay put.
  const choice = (label) => qa('#panel-solve input[name="solve-mode"]').find((r) => r.parentElement.textContent.trim() === label);
  const canvasTop = q("#view").getBoundingClientRect().top;
  const inTime = choice("Adaptive in time");
  inTime.focus();
  const choiceTop = inTime.parentElement.getBoundingClientRect().top;
  click(inTime);
  await idle();
  await until(() => !q("#view-stamp").hidden, "the outdated stamp");
  const after = choice("Adaptive in time").parentElement.getBoundingClientRect().top;
  check("the chosen mode stays under the pointer", Math.abs(after - choiceTop) < 2, `${choiceTop} → ${after}`);
  check("the picture does not move", Math.abs(q("#view").getBoundingClientRect().top - canvasTop) < 1, `${canvasTop} → ${q("#view").getBoundingClientRect().top}`);
  click(choice("Adaptive"));
  await until(() => q("#view-stamp").hidden, "the result current again");

  // The end state (screenshot): cycle 3's solution over its mesh's edges.
  click(q("#layer-mesh"));
  click(pick(3));
  await until(() => /in cycle 3 of/.test(text(q("#legend"))), "cycle 3 in the legend", 30000);
  q("#solve-cycles .shown-cycle").scrollIntoView({ block: "center" });
  await idle();
};
