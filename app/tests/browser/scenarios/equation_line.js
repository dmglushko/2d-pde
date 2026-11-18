// The equation and the conditions as one line: what a line means shows
// under it at once, its error after a pause in typing, the field never
// moves while typing, the symbol buttons insert at the caret, and a form
// switches between its line and its coefficients.
export default async ({ q, qa, tab, type, click, until, sleep, text, check, button }) => {
  tab("problem");
  const line = () => q('[data-path="problem.equation.text"] input');
  const meaning = () => text(q('[data-derived-of="problem.equation"]'));
  await until(() => /a_xx = -1/.test(meaning()), "what the default line means");
  check("a new case writes the equation as a line", line().value === "-lap(u) = 1", line().value);

  // Typing key by key: the field stays where it is.
  const top = line().getBoundingClientRect().top;
  const typed = "u_t - ∇·((1 + x^2)∇u) = 1";
  let moved = 0;
  for (let i = 1; i <= typed.length; i++) {
    type(line(), typed.slice(0, i));
    await sleep(15);
    moved = Math.max(moved, Math.abs(line().getBoundingClientRect().top - top));
  }
  check("the field does not move while typing", moved < 0.5, String(moved));
  await until(() => /b_x = -2\*x/.test(meaning()), "the divergence form expanded");
  check("the line means its coefficients", /a_xx = -\(1 \+ x\^2\)/.test(meaning()) && /m1 = 1/.test(meaning()), meaning());
  check("the time order follows", /first order/.test(text(q("#problem-summary"))), text(q("#problem-summary")));

  // A symbol at the caret.
  const input = line();
  input.focus();
  input.setSelectionRange(0, 0);
  click(button("∇", q('[data-path="problem.equation.text"]')));
  check("a symbol goes in at the caret", input.value.startsWith("∇u_t"), input.value);

  // An error waits for a pause, then says what is wrong at the line.
  type(line(), "u*u_x = 1");
  const cell = q('[data-path="problem.equation.text"]');
  await until(() => /linear in u/.test(text(cell)), "the error after a pause");
  check("the error names the part", /'u\*u_x' multiplies u by u/.test(text(cell)), text(cell));

  // Coefficients and back.
  type(line(), "-div(k*grad(u)) + u = 0");
  click(button("Add", q("#panel-problem")));
  await until(() => q('input[aria-label="Parameter name"]'), "a parameter");
  const name = q('input[aria-label="Parameter name"]');
  name.value = "k";
  name.dispatchEvent(new name.ownerDocument.defaultView.Event("change", { bubbles: true }));
  await until(() => /a_xx = -k/.test(meaning()), "k is a parameter");
  click(button("Coefficients", q("#panel-problem")));
  await until(() => q('[data-path="problem.equation.a_xx"] input'), "the coefficients");
  check("the coefficients are the line's", q('[data-path="problem.equation.a_xx"] input').value === "-k" && q('[data-path="problem.equation.c"] input').value === "1");
  await until(() => button("As a line", q("#panel-problem")), "the switch back");
  await until(() => /No errors/.test(text(q("#panel-problem .step-status"))), "the check of the coefficients");
  click(button("As a line", q("#panel-problem")));
  await until(() => line(), "the line again");
  check("the coefficients print as a line", line().value === "-k*u_xx - k*u_yy + u = 0", line().value);

  // A condition as a line.
  const bottom = q('[data-path="problem.boundary_conditions.bottom.text"] input');
  type(bottom, "u_n + 2*u = 0");
  await until(() => /natural/.test(text(q('[data-kind-of="bottom"]'))), "the condition's kind");
  check("a condition line is classified", /beta_n = 1/.test(text(q('[data-derived-of="problem.boundary_conditions.bottom"]'))));
  check("no field shows an error", qa("#panel-problem .error-text").length === 0, qa("#panel-problem .error-text").map((e) => e.textContent).join(" | "));
};
