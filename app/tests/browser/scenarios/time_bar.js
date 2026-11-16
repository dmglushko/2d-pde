// The time bar plays and pauses; probes are pinned by click and keyboard
// and get a u(t) chart.
export default async ({ w, q, qa, click, select, until, sleep, text, check }) => {
  select(q("#examples"), "heat_annulus.json");
  await until(() => text(q("#toast")).includes("heat_annulus"), "the heat case");
  await until(() => q("#geometry-summary .stats"), "its geometry");
  click(q("#run-all"));
  await until(() => text(q("#toast")).includes("up to date"), "Run all", 90000);
  check("the time bar is shown", !q("#timebar").hidden);
  const slider = q("#time-slider");
  check("the slider covers every snapshot", Number(slider.max) === 50, slider.max);
  slider.value = "10";
  slider.dispatchEvent(new w.Event("input", { bubbles: true }));
  await sleep(50);
  check("the legend follows the slider", text(q("#legend")).includes(text(q(".time-now")).replace("t = ", "")), `${text(q("#legend"))} / ${text(q(".time-now"))}`);
  click(q("#time-play"));
  await sleep(700);
  const playing = Number(slider.value);
  check("Play moves on", playing > 10, String(playing));
  check("the button says Pause", q("#time-play").getAttribute("aria-label") === "Pause");
  click(q("#time-play"));
  const paused = Number(slider.value);
  await sleep(500);
  check("Pause stops", Number(slider.value) === paused, `${paused} -> ${slider.value}`);
  // A probe by click, one by keyboard.
  const canvas = q("#view");
  const rect = canvas.getBoundingClientRect();
  const at = { clientX: rect.left + rect.width * 0.7, clientY: rect.top + rect.height / 2, button: 0, pointerId: 1, bubbles: true };
  canvas.dispatchEvent(new w.PointerEvent("pointerdown", at));
  canvas.dispatchEvent(new w.PointerEvent("pointerup", at));
  canvas.focus();
  canvas.dispatchEvent(new w.KeyboardEvent("keydown", { key: "Enter", bubbles: true }));
  await sleep(100);
  const rows = qa(".probes-table tbody tr");
  check("two probes are listed", rows.length === 2, String(rows.length));
  check("the u(t) chart is drawn", qa(".probe-chart path.line").length === 2);
  click(q("#layer-isolines"));
  await sleep(100);
  check("isolines can be shown", q("#layer-isolines").checked);
};
