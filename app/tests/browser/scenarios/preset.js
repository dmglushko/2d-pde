// A preset applies and solves; comparing with its exact solution gives a
// tiny error (in double, on the server); a broken expression is marked on
// its input.
export default async ({ q, tab, panel, button, click, select, type, until, text, check }) => {
  tab("problem");
  // The presets are in the case menu, next to the examples.
  const laplace = [...q("#problem-presets").querySelectorAll("button")].find((b) => /Laplace/.test(b.textContent));
  check("the Laplace preset is offered in the case menu", Boolean(laplace));
  check("the Problem step has no preset select of its own", !q('#panel-problem select[aria-label="Problem preset"]'));
  click(laplace);
  await until(() => /lap\(u\)\s*=\s*0/.test(q('[data-path="problem.equation.text"] input')?.value ?? ""), "the preset");
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
  const figure = await until(() => { const v = q('.comparison[data-kind="exact"] .compare-result .stat .value'); return v && Number.isFinite(Number(v.textContent)) ? v : null; }, "the comparison", 30000);
  const error = Number(figure.textContent);
  check("u = x is solved exactly (in double, not float32 rounding)", Number.isFinite(error) && error < 1e-12, figure.textContent);
  click(q("#show-exact"));
  await until(() => /u − exact/.test(text(q("#legend"))), "the error in the picture");
  check("the picture shows u − exact", true);
  type(exact, "sin(");
  click(button("Compare", panel("solve")));
  const marked = await until(() => q('#panel-solve .field.invalid[data-path="exact"]'), "the marked expression", 30000);
  check("a broken expression is marked on its input", text(marked.querySelector(".error-text")).length > 0);
};
