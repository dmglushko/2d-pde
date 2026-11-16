// Every step: its head (title, status) on top; a Next link once its
// result is up to date; the Problem step's status; no inline styles.
export default async ({ q, qa, tab, click, select, until, text, check }) => {
  select(q("#examples"), "heat_annulus.json");
  await until(() => text(q("#toast")).includes("heat_annulus"), "the heat case");
  await until(() => q("#geometry-summary .stats"), "its geometry");
  click(q("#run-all"));
  await until(() => text(q("#toast")).includes("up to date"), "Run all", 90000);
  // The problem check that waited for the run follows it.
  await until(() => /No errors/.test(text(q("#panel-problem .step-status"))), "the problem check");
  for (const step of ["geometry", "mesh", "problem", "assemble"]) {
    tab(step);
    const panel = q(`#panel-${step}`);
    const head = panel.firstElementChild;
    check(`${step}: the head comes first`, head?.classList.contains("step-head"));
    check(`${step}: up to date with a Next link`, /Next: /.test(text(head)), text(head));
  }
  tab("problem");
  check("problem: no errors", /No errors/.test(text(q("#panel-problem .step-status"))), text(q("#panel-problem .step-status")));
  const styled = qa(".panels [style], .topbar [style], .legend [style]");
  check("no inline styles in the panels", styled.length === 0, styled.slice(0, 3).map((e) => e.outerHTML.slice(0, 80)).join(" | "));
  const lead = qa(".panel p.lead");
  check("no lead paragraphs", lead.length === 0);
};
