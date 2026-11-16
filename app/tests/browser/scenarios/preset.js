// A preset applies and solves; comparing with its exact solution gives a
// tiny error.
export default async ({ q, tab, panel, button, click, select, type, until, text, check }) => {
  tab("problem");
  const presets = q('#panel-problem select[aria-label="Problem preset"]');
  const laplace = [...presets.options].find((o) => /Laplace/.test(o.textContent));
  check("the Laplace preset is offered", Boolean(laplace));
  select(presets, laplace.value);
  await until(() => text(q("#toast")).includes("filled in"), "the preset");
  await until(() => /No errors/.test(text(q("#panel-problem .step-status"))), "its check");
  check("the problem is checked without errors", true);
  tab("solve");
  click(button("Solve", panel("solve")));
  await until(() => q("#solve-result .stats"), "the solution", 60000);
  check("the legend shows u", /u at t = /.test(text(q("#legend"))), text(q("#legend")));
  const exact = q('#panel-solve input[aria-label="Exact solution"]');
  check("the comparison is offered", Boolean(exact));
  type(exact, "x");
  click(button("Compare", panel("solve")));
  const figure = await until(() => { const v = q("#compare-result .stat .value"); return v && Number.isFinite(Number(v.textContent)) ? v : null; }, "the comparison", 30000);
  const error = Number(figure.textContent);
  check("u = x is solved exactly (up to float32)", Number.isFinite(error) && error < 1e-5, figure.textContent);
  click(q("#show-error"));
  await until(() => /u − exact/.test(text(q("#legend"))), "the error in the picture");
  check("the picture shows u − exact", true);
};
