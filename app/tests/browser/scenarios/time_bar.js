// The time bar plays and pauses; probes are pinned by click and keyboard
// and get a u(t) chart.
export default async ({ example, w, q, qa, click, select, until, sleep, text, check }) => {
  // sleep only where the check is that nothing happens (Pause).
  example("heat_annulus.json");
  await until(() => text(q("#case-name")).includes("heat_annulus"), "the heat case");
  await until(() => q("#geometry-summary .stats"), "its geometry");
  click(q("#run-all"));
  await until(() => text(q("#toast")).includes("up to date"), "Run all", 90000);
  check("the time bar is shown", !q("#timebar").hidden);
  const slider = q("#time-slider");
  // The run plays from its start in real time (5 s of simulated time).
  check("a run plays from its start", q("#time-play").getAttribute("aria-label") === "Pause" && Number(slider.value) < 50, slider.value);
  check("the slider covers every snapshot", Number(slider.max) === 50, slider.max);
  slider.value = "10";
  slider.dispatchEvent(new w.Event("input", { bubbles: true }));
  await until(() => text(q("#legend")).includes(`u at t = ${text(q(".time-now")).replace("t = ", "")}`), "the legend to follow");
  check("the legend follows the slider", true);
  click(q("#time-play"));
  const playing = await until(() => (Number(slider.value) > 12 ? Number(slider.value) : null), "Play to move on");
  check("Play moves on", playing > 10, String(playing));
  check("the button says Pause", q("#time-play").getAttribute("aria-label") === "Pause");
  click(q("#time-play"));
  const paused = Number(slider.value);
  await sleep(4 * (1000 / 12)); // four playback frames: nothing may move
  check("Pause stops", Number(slider.value) === paused, `${paused} -> ${slider.value}`);
  // A probe by click, one by keyboard.
  const canvas = q("#view");
  const rect = canvas.getBoundingClientRect();
  const at = { clientX: rect.left + rect.width * 0.7, clientY: rect.top + rect.height / 2, button: 0, pointerId: 1, bubbles: true };
  canvas.dispatchEvent(new w.PointerEvent("pointerdown", at));
  canvas.dispatchEvent(new w.PointerEvent("pointerup", at));
  canvas.focus();
  canvas.dispatchEvent(new w.KeyboardEvent("keydown", { key: "Enter", bubbles: true }));
  const rows = await until(() => (qa(".probes-table tbody tr").length === 2 ? qa(".probes-table tbody tr") : null), "two probes");
  check("two probes are listed", rows.length === 2, String(rows.length));
  await until(() => qa(".probe-chart path.line").length === 2, "the u(t) chart");
  check("the u(t) chart is drawn", true);
  const plain = q("#view").toDataURL();
  click(q("#layer-isolines"));
  await until(() => q("#view").toDataURL() !== plain, "the isolines");
  check("isolines can be shown", q("#layer-isolines").checked);
};
