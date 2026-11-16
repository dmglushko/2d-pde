// A refinement study on the L-shape: three levels of the relative max
// edge length, the observed L2 order near the corner singularity's 4/3,
// and the case's mesh setting put back.
export default async ({ example, q, qa, tab, type, click, button, panel, until, text, check }) => {
  example("l_shape_corner.json");
  await until(() => text(q("#toast")).includes("l_shape_corner"), "the L-shape");
  await until(() => q("#geometry-summary .stats"), "its geometry");
  tab("mesh");
  type(q('input[data-path="mesh.spec.max_edge_length.relative"]'), "0.08");
  click(q("#run-all"));
  await until(() => text(q("#toast")).includes("up to date"), "Run all", 90000);
  tab("solve");
  const section = await until(() => q("#solve-study .study"), "the study's section");
  const toggle = section.querySelector(".fold-toggle");
  if (toggle.getAttribute("aria-expanded") !== "true") click(toggle);
  click(button("Run study", panel("solve")));
  await until(() => /solved again/.test(text(q("#solve-study"))) || /could not|Stopped/.test(text(q("#solve-study"))), "the study", 170000);
  const rows = qa("#solve-study .study-table tbody tr");
  check("three levels", rows.length === 3, String(rows.length));
  const l2 = qa('#solve-study td.study-order[data-norm="l2"]').map((td) => Number(td.textContent));
  check("the observed L2 order is near 4/3", l2.slice(1).every((o) => o > 1.1 && o < 1.75), l2.join(", "));
  check("a log–log chart with reference slopes", qa("#solve-study path.line.ref").length === 2);
  check("the mesh setting is back", q('input[data-path="mesh.spec.max_edge_length.relative"]').value === "0.08", q('input[data-path="mesh.spec.max_edge_length.relative"]').value);
  check("and its solution is current again", /Up to date/.test(text(q("#panel-solve .step-status"))), text(q("#panel-solve .step-status")));
};
