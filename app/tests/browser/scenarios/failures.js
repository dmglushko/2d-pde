// Failed secondary requests are said beside what they concern, with
// Retry; nothing stays "loading" and u is never drawn in place of a
// difference that failed. The charts read with the keyboard.
export default async ({ example, w, q, qa, click, until, text, check, pointer, key }) => {
  example("heat_annulus.json");
  await until(() => text(q("#toast")).includes("heat_annulus"), "the heat case");
  await until(() => q("#geometry-summary .stats"), "its geometry");
  click(q("#run-all"));
  await until(() => text(q("#toast")).includes("up to date"), "Run all", 90000);
  // Every request to `path` fails (no answer) while it is in the set.
  const failing = new Set();
  const fetch = w.fetch;
  w.fetch = (path, ...rest) => (failing.has(path) ? Promise.reject(new TypeError("Failed to fetch")) : fetch(path, ...rest));
  failing.add("/api/solve/values");
  const slider = q("#time-slider");
  // Far from the start, which plays and is loaded as the run comes.
  slider.value = "45";
  slider.dispatchEvent(new w.Event("input", { bubbles: true }));
  await until(() => !q("#view-overlay").hidden && q("#view-overlay").classList.contains("failed"), "the snapshot's failure");
  check("a failed snapshot is said over the picture", /does not answer/.test(text(q("#view-overlay"))), text(q("#view-overlay")));
  const retry = [...q("#view-overlay").querySelectorAll("button")].find((b) => b.textContent === "Retry");
  check("with Retry", Boolean(retry));
  failing.delete("/api/solve/values");
  click(retry);
  await until(() => q("#view-overlay").hidden && /^u at t = 4\.5$/.test(text(q("#legend .bar-label"))), "the snapshot after Retry");
  check("Retry loads it", true);
  // u(t) at a probe.
  failing.add("/api/solve/probes");
  // A point inside the annulus (radii 1 and 0.4, fitted at 0.88 of the
  // picture), on the right at r = 0.7: away from its boundary, where a
  // click selects the region instead.
  const wrap = q("#canvas-wrap").getBoundingClientRect();
  const inside = 0.5 + (0.7 * 0.88 * Math.min(wrap.width, wrap.height)) / 2 / wrap.width;
  pointer("pointerdown", inside, 0.5);
  pointer("pointerup", inside, 0.5);
  await until(() => q(".probe-chart .inline-failure"), "the probes' failure");
  check("a failed u(t) is said at its chart", /u at the probes did not arrive/.test(text(q(".probe-chart"))), text(q(".probe-chart")));
  check("the table does not say loading", !qa(".probes-table td").some((td) => td.textContent === "…"));
  failing.delete("/api/solve/probes");
  click([...q(".probe-chart").querySelectorAll("button")].find((b) => b.textContent === "Retry"));
  await until(() => q(".probe-chart path.line"), "u(t) after Retry");
  check("Retry draws u(t)", true);
  check("the table's value is the server's double", /in double from the server/.test(text(q(".probe-source"))));
  // The chart without a mouse: focus, arrows, a spoken readout; its data.
  const plot = q(".probe-chart .chart-plot");
  plot.focus();
  key(plot, "Home");
  key(plot, "ArrowRight");
  const spoken = plot.querySelector("[aria-live]");
  check("the arrow keys read the chart", /^t = 0\.1: P1 /.test(spoken.textContent), spoken.textContent);
  check("the readout is shown", !plot.querySelector(".chart-tip").hidden);
  check("the chart's data can be downloaded", Boolean([...q(".probe-chart").querySelectorAll("button")].find((b) => b.textContent === "Download data (CSV)")));
  w.fetch = fetch;
};
