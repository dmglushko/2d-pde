// A reload of the tab is the same client: it finds its run on the server
// and shows it again, with the probes it had. A copy of the tab (here a
// second page with the same sessionStorage) is another client.
export default async ({ example, q, click, until, text, check, pointer, reloadPage, openPage }) => {
  example("heat_annulus.json");
  await until(() => text(q("#toast")).includes("heat_annulus"), "the heat case");
  await until(() => q("#geometry-summary .stats"), "its geometry");
  click(q("#run-all"));
  await until(() => text(q("#toast")).includes("up to date"), "Run all", 90000);
  // A point inside the annulus (radii 1 and 0.4, fitted at 0.88 of the
  // picture), on the right at r = 0.7: away from its boundary, where a
  // click selects the region instead.
  const wrap = q("#canvas-wrap").getBoundingClientRect();
  const inside = 0.5 + (0.7 * 0.88 * Math.min(wrap.width, wrap.height)) / 2 / wrap.width;
  pointer("pointerdown", inside, 0.5);
  pointer("pointerup", inside, 0.5);
  await until(() => q(".probes-table tbody tr"), "the probe");
  const page = await reloadPage();
  await until(() => /shown again/.test(page.q("#toast")?.textContent ?? ""), "the run found again", 30000);
  check("the reloaded tab shows its run again", !page.q("#timebar").hidden);
  check("the solve is up to date", /Up to date/.test(page.q("#panel-solve .step-status")?.textContent ?? ""), page.q("#panel-solve .step-status")?.textContent);
  await until(() => page.q(".probes-table tbody tr"), "the probe again");
  check("the probe is back (kept in this tab)", page.qa(".probes-table tbody tr").length === 1);
  await until(() => page.q(".probe-chart path.line"), "u(t) from the run found again", 30000);
  check("u(t) comes from the run found again", true);
  // A copy of the tab: it does not take this tab's run.
  const other = await openPage();
  await until(() => other.q("#geometry-summary .stats"), "the copy's geometry", 30000);
  await until(() => other.q("#timebar") && other.w.document.readyState === "complete", "the copy");
  await new Promise((resolve) => setTimeout(resolve, 1500));
  check("a copy of the tab is another client (no run found)", other.q("#timebar").hidden && !/shown again/.test(other.q("#toast")?.textContent ?? ""));
};
