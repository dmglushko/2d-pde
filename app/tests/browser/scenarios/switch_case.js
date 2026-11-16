// Switching the case while a request runs shows only the new case's
// results.
export default async ({ example, q, tab, panel, button, click, select, until, idle, text, check }) => {
  example("wave_square.json");
  await until(() => text(q("#toast")).includes("wave_square"), "the wave case");
  await until(() => q("#geometry-summary .stats"), "its geometry");
  tab("solve");
  click(button("Solve", panel("solve")));
  await until(() => /Solving|Running|Meshing/.test(text(q("#panel-solve .step-status"))) || q(".steps .dot.busy"), "the solve to start");
  example("poisson_rectangle.json");
  await until(() => text(q("#toast")).includes("poisson_rectangle"), "the Poisson case");
  // The server answers in order: once the new case's geometry is here
  // and nothing is in flight, the wave case's answers have come and gone.
  await until(() => q("#geometry-summary .stats") && !q(".steps .dot.busy"), "the Poisson geometry");
  await idle();
  check("no result of the wave case is shown", !q("#solve-result .stats"), text(q("#solve-result")));
  check("no time bar from the wave case", q("#timebar").hidden);
  check("the Solve step is not running", !/Running|Solving/.test(text(q("#panel-solve .step-status"))), text(q("#panel-solve .step-status")));
  click(q("#run-all"));
  await until(() => text(q("#toast")).includes("up to date"), "Run all", 60000);
  check("the new case's static solution is shown", /u at t = 0\b/.test(text(q("#legend"))) && q("#timebar").hidden, text(q("#legend")));
};
