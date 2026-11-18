// An adaptive run in time (heat on the L-shape): the time integration's
// settings with the adaptation's, the adaptation table, each snapshot
// drawn on its own mesh as the time bar moves, the exports a run on
// several meshes has, and the exact comparison of every snapshot.
export default async ({ w, example, q, qa, tab, click, until, idle, text, check, pixels, panel, button, sleep }) => {
  example("l_shape_heat_adaptive.json");
  await until(() => text(q("#toast")).includes("l_shape_heat_adaptive"), "the example");
  await until(() => q("#geometry-summary .stats"), "its geometry");
  click(q("#run-all"));
  await until(() => text(q("#toast")).includes("up to date"), "Run all", 90000);
  tab("solve");
  check("the mode adaptive in time is chosen", qa('#panel-solve input[name="solve-mode"]').some((r) => r.checked && /Adaptive in time/.test(r.parentElement.textContent)));
  check("the time integration's settings are shown", Boolean(q('#panel-solve [data-path="solve.time"]')));
  check("the adaptation's settings are shown", Boolean(q('#panel-solve [data-path="solve.adaptive_time"]')));
  const steps = q('#panel-solve input[aria-label="Adapt every, steps"]');
  check("the schedule's steps field", steps?.value === "5", steps?.value);
  check("the damping method slot", /Damping method/.test(text(q("#panel-solve .settings-part"))));
  await until(() => q("#solve-cycles .adaptations-table"), "the adaptation table");
  const rows = qa("#solve-cycles .adaptations-table tbody tr");
  check("an adaptation every 5 steps", rows.length === 19, String(rows.length));
  check("refined at the adaptation times", rows.filter((tr) => /refined →/.test(tr.textContent)).length >= 15);
  check("the refinements are counted", /Refinements/.test(text(q("#solve-stats"))), text(q("#solve-stats")));
  check("no one-mesh CSV export", !button("Export .csv", panel("solve")));
  check("the VTK series and the last mesh export", Boolean(button("Export VTK series", panel("solve"))) && Boolean(button("Export adapted mesh", panel("solve"))));
  check("no reference section", !q('.comparison[data-kind="reference"]'));
  check("the time bar is shown", !q("#timebar").hidden);

  // Each snapshot on its own mesh: the first on the starting mesh.
  const display = await w.eval(`import("${new URL("js/display.js", w.location.href).href}")`);
  const nodes = () => display.pictureMesh().mesh?.nodes.length / 2;
  const slider = q("#time-slider");
  if (q("#time-play").getAttribute("aria-label") === "Pause") click(q("#time-play"));
  slider.value = slider.max;
  slider.dispatchEvent(new w.Event("input", { bubbles: true }));
  await idle();
  const last = nodes();
  slider.value = "0";
  slider.dispatchEvent(new w.Event("input", { bubbles: true }));
  await idle();
  const first = await until(() => (nodes() < last ? nodes() : null), "the starting mesh at t0");
  check("t0 is drawn on the starting mesh, t1 on the last", first < last / 10, `${first} → ${last}`);
  slider.value = "10";
  slider.dispatchEvent(new w.Event("input", { bubbles: true }));
  await until(() => /u at t = 1/.test(text(q("#legend"))), "the snapshot at t = 1");
  const middle = nodes();
  check("a snapshot in between on a mesh in between", middle > first && middle < last, `${first} < ${middle} < ${last}`);
  click(q("#layer-mesh"));
  await idle();
  const coloured = await until(() => (pixels().colours > 20 ? pixels() : null), "u in the picture");
  check("u colours the picture on its mesh", coloured.colours > 20, String(coloured.colours));
  click(q("#layer-mesh"));

  // The exact comparison: every snapshot against its own mesh.
  click(button("Compare", panel("solve")));
  const section = () => q('.comparison[data-kind="exact"]');
  await until(() => section()?.querySelector(".compare-result .stat"), "the comparison", 60000);
  click(q("#show-exact"));
  await until(() => /^u − exact/.test(text(q("#legend .colorbar .bar-label"))), "u − exact on the snapshot's mesh", 30000);
  check("u − exact is drawn on the snapshot's mesh", pixels().colours > 5);
  click(q("#show-exact"));
  slider.value = slider.max;
  slider.dispatchEvent(new w.Event("input", { bubbles: true }));
  await sleep(200);
  await idle();
};
