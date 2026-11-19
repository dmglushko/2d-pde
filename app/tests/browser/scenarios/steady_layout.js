// What must not move or hide: the sidebar's width follows its handle
// (and is remembered); the legend's colour bar and the time bar's slider
// stay put while snapshots change; the problem's summary keeps its four
// chips (and its height) when an edit changes a fact; the colour range
// can follow each snapshot; the Threads card is always there.
export default async ({ w, d, q, qa, tab, click, example, select, type, until, idle, text, check, pixels }) => {
  example("wave_square.json");
  await until(() => text(q("#case-name")).includes("wave_square"), "the case");
  click(q("#run-all"));
  await until(() => text(q("#toast")).includes("up to date"), "Run all", 90000);
  await idle();

  // The sidebar's handle.
  const handle = q("#sidebar-handle");
  const sidebar = q("#sidebar");
  const before = sidebar.getBoundingClientRect().width;
  const finite = (name) => Number.isFinite(Number(handle.getAttribute(name)));
  check("the separator has its values", finite("aria-valuenow") && finite("aria-valuemin") && finite("aria-valuemax"), handle.outerHTML.slice(0, 200));
  const at = (x) => ({ clientX: x, clientY: 300, button: 0, pointerId: 7, bubbles: true });
  const right = sidebar.getBoundingClientRect().right;
  handle.dispatchEvent(new w.PointerEvent("pointerdown", at(right)));
  handle.dispatchEvent(new w.PointerEvent("pointermove", at(right + 120)));
  handle.dispatchEvent(new w.PointerEvent("pointerup", at(right + 120)));
  await until(() => sidebar.getBoundingClientRect().width > before + 100, "the sidebar to widen");
  check("dragging the handle widens the sidebar", true);
  check("the width is remembered", Number(w.localStorage.getItem("pde2d-app-sidebar-width")) > before + 100, w.localStorage.getItem("pde2d-app-sidebar-width"));
  handle.focus();
  handle.dispatchEvent(new w.KeyboardEvent("keydown", { key: "ArrowLeft", bubbles: true }));
  await until(() => sidebar.getBoundingClientRect().width < before + 120 - 10, "ArrowLeft to narrow it");
  check("ArrowLeft narrows it", true);
  check("its values stay numbers", finite("aria-valuenow") && finite("aria-valuemax"), handle.getAttribute("aria-valuemax"));
  // A narrower window clamps the width; a wide one gives the chosen one back.
  const chosen = sidebar.getBoundingClientRect().width;
  const frame = w.frameElement;
  const wide = frame.style.width;
  frame.style.width = `${Math.round(chosen + 360 - 80)}px`;
  await until(() => sidebar.getBoundingClientRect().width < chosen - 40, "the narrow window to clamp it");
  frame.style.width = wide;
  await until(() => Math.abs(sidebar.getBoundingClientRect().width - chosen) < 1, "the chosen width again");
  check("a window made wide again gives the chosen width back", true);
  handle.dispatchEvent(new w.MouseEvent("dblclick", { bubbles: true }));
  await until(() => Math.abs(sidebar.getBoundingClientRect().width - before) < 1, "the default width");
  check("a double click gives the default width back", w.localStorage.getItem("pde2d-app-sidebar-width") === null && finite("aria-valuenow"));

  // Snapshots with times of different widths: nothing moves.
  const slider = q("#time-slider");
  const show = async (i) => {
    slider.value = String(i);
    slider.dispatchEvent(new w.Event("input", { bubbles: true }));
    await idle();
    // The legend names the snapshot shown (its values loaded: no stand-in).
    const time = text(q(".time-now .steady-text")).replace("t = ", "");
    await until(() => text(q("#legend .bar-when .steady-text")) === `at t = ${time}` && q("#view-overlay").hidden, `the legend at t = ${time}`);
    return { bar: q("#legend .bar-scale").getBoundingClientRect().left, slider: slider.getBoundingClientRect().width, legend: text(q("#legend .bar-when .steady-text")) };
  };
  const one = await show(1);
  const two = await show(Number(slider.max) - 3);
  check("the legend's time changed", one.legend !== two.legend, `${one.legend} / ${two.legend}`);
  check("the colour bar stays put", Math.abs(one.bar - two.bar) < 0.5, `${one.bar} ${two.bar}`);
  check("the slider keeps its width", Math.abs(one.slider - two.slider) < 0.5, `${one.slider} ${two.slider}`);

  // The colour range of each snapshot.
  const range = q("#legend select.range-choice");
  check("a run in time offers the range choice", Boolean(range) && range.value === "run");
  const ends = () => text(q("#legend .bar-scale"));
  const whole = ends();
  select(range, "snapshot");
  await until(() => q("#legend select.range-choice")?.value === "snapshot", "this snapshot's range");
  check("this snapshot's range is its own", ends() !== whole, `${whole} / ${ends()}`);
  select(q("#legend select.range-choice"), "run");
  await until(() => ends() === whole, "the whole run's range again");
  // The focused range choice stays focused while snapshots change.
  q("#legend select.range-choice").focus();
  await show(3);
  await show(Number(slider.max) - 1);
  check("the range choice keeps the focus while the picture changes", d.activeElement === q("#legend select.range-choice"), d.activeElement?.outerHTML.slice(0, 60));

  // The Threads card: the assembly's count, and a note on the solver.
  tab("solve");
  const threads = [...qa("#panel-solve .card")].find((c) => /^Threads/.test(text(c.querySelector(".card-head"))));
  check("the Threads card is there, open", Boolean(threads) && !threads.querySelector(".fold-body").hidden);
  check("it has the assembly's threads", Boolean(threads?.querySelector('[data-path="solve.assembler.threads"]')));
  check("it says which solvers take threads", /runs in one thread/.test(text(threads)) || Boolean(threads?.querySelector('[data-path="solve.linear_solver.options.threads"]')), text(threads));

  // The problem's summary: an edit that changes the time order keeps its
  // four chips and its height.
  tab("problem");
  await until(() => qa("#problem-summary .summary-chips .chip").length === 4, "the summary");
  const height = q("#problem-summary").getBoundingClientRect().height;
  const line = q('[data-path="problem.equation.text"] input');
  type(line, "-lap(u) = 1");
  line.dispatchEvent(new w.Event("change", { bubbles: true }));
  await until(() => /Time order 0/.test(text(q("#problem-summary"))), "the static problem's facts");
  check("still four chips", qa("#problem-summary .summary-chips .chip").length === 4);
  check("the summary keeps its height", Math.abs(q("#problem-summary").getBoundingClientRect().height - height) < 1, `${height} ${q("#problem-summary").getBoundingClientRect().height}`);
  type(line, "u_tt - lap(u) = 1 0");
  line.dispatchEvent(new w.Event("change", { bubbles: true }));
  await until(() => q("#problem-summary .summary-chips.stale"), "the facts of the last readable equation, muted");
  check("an unreadable equation keeps the last facts", /Time order 0/.test(text(q("#problem-summary"))), text(q("#problem-summary")));
  check("no inline styles", qa(".panels [style], .legend [style], .timebar [style]").length === 0);
  // Fit (⌂) fits the picture again after a zoom.
  tab("solve");
  q("#fit-view").click();
  await idle();
  const fitted = pixels().hash;
  click(q("#zoom-in"));
  click(q("#zoom-in"));
  await until(() => pixels().hash !== fitted, "the zoom");
  q("#fit-view").click();
  await until(() => pixels().hash === fitted, "⌂ to fit the picture again");
  check("⌂ fits the picture again", true);
};
