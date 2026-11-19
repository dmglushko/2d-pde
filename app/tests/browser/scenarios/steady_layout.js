// What must not move or hide: the sidebar's width follows its handle
// (and is remembered); the legend's colour bar and the time bar's slider
// stay put while snapshots change; the problem's summary keeps its four
// chips (and its height) when an edit changes a fact; the colour range
// can follow each snapshot; the Threads card is always there.
export default async ({ w, d, q, qa, tab, click, example, select, type, until, idle, text, check }) => {
  example("wave_square.json");
  await until(() => text(q("#toast")).includes("wave_square"), "the case");
  click(q("#run-all"));
  await until(() => text(q("#toast")).includes("up to date"), "Run all", 90000);
  await idle();

  // The sidebar's handle.
  const handle = q("#sidebar-handle");
  const sidebar = q("#sidebar");
  const before = sidebar.getBoundingClientRect().width;
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
  handle.dispatchEvent(new w.MouseEvent("dblclick", { bubbles: true }));
  await until(() => Math.abs(sidebar.getBoundingClientRect().width - before) < 1, "the default width");
  check("a double click gives the default width back", w.localStorage.getItem("pde2d-app-sidebar-width") === null);

  // Snapshots with times of different widths: nothing moves.
  const slider = q("#time-slider");
  const show = async (i) => {
    slider.value = String(i);
    slider.dispatchEvent(new w.Event("input", { bubbles: true }));
    await until(() => text(q(".time-now .steady-text")) === text(q(".time-now .steady-text")) && /at t = /.test(text(q("#legend"))), "the snapshot");
    await idle();
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
  d.querySelector("#reset-view").click();
  check("Reset view is there", true);
};
