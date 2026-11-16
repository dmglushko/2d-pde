// Switching the case while a request runs shows only the new case's
// results.
export default async ({ q, tab, panel, button, click, select, until, sleep, text, check }) => {
  select(q("#examples"), "wave_square.json");
  await until(() => text(q("#toast")).includes("wave_square"), "the wave case");
  await until(() => q("#geometry-summary .stats"), "its geometry");
  tab("solve");
  click(button("Solve", panel("solve")));
  await sleep(40);
  select(q("#examples"), "poisson_rectangle.json");
  await until(() => text(q("#toast")).includes("poisson_rectangle"), "the Poisson case");
  await sleep(3000);
  check("no result of the wave case is shown", !q("#solve-result .stats"), text(q("#solve-result")));
  check("no time bar from the wave case", q("#timebar").hidden);
  check("the Solve step is not running", !/Running|Solving/.test(text(q("#panel-solve .step-status"))), text(q("#panel-solve .step-status")));
  click(q("#run-all"));
  await until(() => text(q("#toast")).includes("up to date"), "Run all", 60000);
  check("the new case's static solution is shown", /u at t = 0\b/.test(text(q("#legend"))) && q("#timebar").hidden, text(q("#legend")));
};
