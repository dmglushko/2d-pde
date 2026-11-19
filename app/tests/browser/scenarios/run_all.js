// Run all on an example ends with the solution in the picture.
export default async ({ example, q, qa, click, select, until, text, check }) => {
  example("poisson_rectangle.json");
  await until(() => text(q("#case-name")).includes("poisson_rectangle"), "the example");
  await until(() => q("#geometry-summary .stats"), "its geometry");
  click(q("#run-all"));
  await until(() => text(q("#toast")).includes("up to date"), "Run all", 60000);
  check("the picture colours by the field", q("#color-by").value === "field", q("#color-by").value);
  check("the legend shows u", /u at t = /.test(text(q("#legend"))), text(q("#legend")));
  const dots = qa(".steps button").map((b) => `${b.dataset.step}:${b.querySelector(".dot").className}`);
  check("every run step is up to date", ["geometry", "mesh", "assemble", "solve"].every((s) => dots.includes(`${s}:dot done`)), dots.join(" "));
  check("a static solution has no time bar", q("#timebar").hidden);
  check("the Solve head says up to date", /Up to date/.test(text(q("#panel-solve .step-status"))), text(q("#panel-solve .step-status")));
};
