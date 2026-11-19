// A short window: the page never scrolls below its interface (each column
// scrolls on its own). The reports: a caret says a row opens; the time
// run's integrator report says where its time went; the remesher's plan
// is a note, never a warning.
export default async ({ example, d, w, q, qa, tab, click, until, text, check }) => {
  example("heat_annulus.json");
  await until(() => text(q("#toast")).includes("heat_annulus"), "the heat case");
  await until(() => q("#geometry-summary .stats"), "its geometry");
  click(q("#run-all"));
  await until(() => text(q("#toast")).includes("up to date"), "Run all", 90000);

  const page = d.scrollingElement;
  check("the page does not scroll", page.scrollHeight <= w.innerHeight + 1, `${page.scrollHeight} > ${w.innerHeight}`);

  // The mesh: the plan is a note; no warning counts it.
  tab("mesh");
  await until(() => q("#panel-mesh details.report"), "the mesh reports");
  const chip = q("#panel-mesh .warning-chip");
  check("the plan is no warning", !chip || !/plan:/.test(text(chip)), text(chip));
  const pipeline = qa("#panel-mesh details.report").find((r) => /pipeline/.test(text(r.querySelector("summary"))));
  check("the pipeline report", !!pipeline);
  pipeline.open = true;
  check("its plan is a note", /plan: /.test(text(pipeline.querySelector(".report-note"))), text(pipeline));
  check("a report row has a caret", !!pipeline.querySelector("summary .caret"));

  // The run: where the integrator's time went.
  tab("solve");
  const integrator = await until(() => qa("#panel-solve details.report").find((r) => /integrator/.test(text(r.querySelector("summary")))), "the integrator report");
  integrator.open = true;
  const breakdown = integrator.querySelector(".time-breakdown");
  check("the integrator says where its time went", !!breakdown, text(integrator));
  for (const part of ["assembly", "prepare", "solve"])
    check(`the time of ${part}`, new RegExp(`${part}\\s*[0-9.]+ ms`).test(text(breakdown)), text(breakdown));
  check("its metrics stay", /steps/.test(text(integrator.querySelector("table.data:not(.time-breakdown)"))));
  check("the page still does not scroll", page.scrollHeight <= w.innerHeight + 1, `${page.scrollHeight} > ${w.innerHeight}`);
  // Every step, with its folded parts open.
  for (const step of ["geometry", "mesh", "problem", "assemble", "solve"]) {
    tab(step);
    for (const toggle of qa(`#panel-${step} [data-fold][aria-expanded="false"]`)) click(toggle);
    for (const details of qa(`#panel-${step} details`)) details.open = true;
    check(`the page does not scroll on ${step}`, page.scrollHeight <= w.innerHeight + 1, `${page.scrollHeight} > ${w.innerHeight}`);
  }
  // For the screenshot: the integrator's report.
  qa("#panel-solve details.report").find((r) => /integrator/.test(text(r.querySelector("summary")))).scrollIntoView();
};
