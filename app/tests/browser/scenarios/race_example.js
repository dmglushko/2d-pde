// An example still loading never replaces a case started meanwhile, and a
// preset's answer never overwrites an edit made after it was asked for.
export default async ({ example, q, tab, select, click, type, until, idle, text, check }) => {
  example("wave_square.json");
  click(q("#new-case"));
  await idle();
  await until(() => q("#geometry-summary .stats"), "the geometry");
  check("the new case stays", text(q("#toast")).includes("Started a new case"), text(q("#toast")));
  check("the example did not replace it", q("#save-case").title === "Save as case.json", q("#save-case").title);
  check("the previous case can be restored", !q("#restore-case").hidden);

  // A preset asked for, then an edit before its answer.
  tab("problem");
  const heat = q('#problem-presets [data-preset^="heat"]');
  click(heat);
  const f = q('[data-path="problem.equation.text"] input');
  type(f, "-lap(u) = 42");
  await idle();
  await until(() => /not applied/.test(text(q("#toast"))), "the preset to be refused");
  check("the edit stays", q('[data-path="problem.equation.text"] input').value === "-lap(u) = 42");
  check("the toast says why", /changed while it was prepared/.test(text(q("#toast"))), text(q("#toast")));
};
