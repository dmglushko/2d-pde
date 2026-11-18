// A reference run: kept, compared after a change of the problem (a
// difference in double), and refused with the server's reason after a
// change of the mesh. The exact-solution comparison goes outdated when its
// expression changes.
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
  // The exact solution's comparison belongs to its expression.
  const exact = q('#panel-solve input[aria-label="Exact solution"]');
  type(exact, "x");
  click(button("Compare", panel("solve")));
  const section = () => q('.comparison[data-kind="exact"]');
  await until(() => section().querySelector(".compare-result .stat"), "the comparison", 30000);
  check("RMS and lumped L2 are shown", /RMS/.test(text(section())) && /L2 \(lumped\)/.test(text(section())), text(section()));
  check("the nodal error is told apart from the residual", /not the solver's residual/i.test(text(section())));
  check("the float32 floor is said", /float32/.test(text(section())));
  type(q('#panel-solve input[aria-label="Exact solution"]'), "x + 1");
  await until(() => section().classList.contains("outdated-comparison"), "the comparison to go outdated");
  check("an edited expression marks the comparison outdated", /for the previous expression/.test(text(section())));
  click(button("Compare", panel("solve")));
  await until(() => !section().classList.contains("outdated-comparison") && /1/.test(text(section().querySelector(".stat .value"))), "the new comparison", 30000);
  check("comparing again makes it current", !/for the previous expression/.test(text(section())));
  // Keep as reference, change a boundary value, solve and compare.
  const refSection = () => q('.comparison[data-kind="reference"]');
  const toggle = refSection().querySelector(".fold-toggle");
  if (toggle.getAttribute("aria-expanded") !== "true") click(toggle);
  click(button("Keep as reference", refSection()));
  await until(() => /This run is the reference/.test(text(refSection())), "the reference");
  check("the run is kept as the reference", true);
  tab("problem");
  const g = q('#panel-problem [data-path^="problem.boundary_conditions."][data-path$=".text"] input');
  type(g, "u = 0.5");
  tab("solve");
  click(button("Solve", panel("solve")));
  await until(() => button("Compare with reference", refSection() ?? q("body")), "the new run", 60000);
  click(button("Compare with reference", refSection()));
  const figure = await until(() => { const v = refSection().querySelector(".compare-result .stat .value"); return v && Number.isFinite(Number(v.textContent)) ? v : null; }, "the reference comparison", 30000);
  check("the difference from the reference is shown", Number(figure.textContent) > 0, figure.textContent);
  // Another mesh: the server refuses, and says why.
  tab("mesh");
  const bound = q('input[data-path="mesh.spec.max_edge_length.relative"]');
  type(bound, "0.07");
  tab("solve");
  click(button("Solve", panel("solve")));
  await until(() => button("Compare with reference", refSection() ?? q("body")) && /Up to date/.test(text(q("#panel-solve .step-status"))), "the run on another mesh", 60000);
  click(button("Compare with reference", refSection()));
  await until(() => /another mesh/.test(text(refSection())), "the refusal", 30000);
  check("a reference on another mesh is refused with the server's reason", true);
  click(button("Forget reference", refSection()));
  await until(() => /No reference yet/.test(text(refSection())), "the reference forgotten");
  check("Forget reference", true);
};
