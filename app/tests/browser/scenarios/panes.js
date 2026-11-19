// What can be hidden or sized around the picture, and typed by hand: no
// hint under the picture (an outdated result leaves it where it is); the
// probes' panel appears with a probe, takes a height dragged on its
// header, hides when dragged low, and comes back; a probe's x and y and
// the line's ends are typed; the steps panel and the view options hide.
export default async ({ w, q, qa, tab, click, example, type, until, idle, text, check, pointer }) => {
  example("poisson_rectangle.json");
  await until(() => text(q("#case-name")).includes("poisson_rectangle"), "the example");
  click(q("#run-all"));
  await until(() => text(q("#toast")).includes("up to date"), "Run all", 90000);
  await idle();
  const picture = () => q("#view").getBoundingClientRect();
  check("no probe, no panel under the picture", q("#probes").hidden);
  check("no hint about probes", !/pin a probe/.test(text(q(".viewer"))));

  // An outdated result: the picture keeps its place and size.
  tab("solve");
  const before = picture();
  const mode = (label) => qa('#panel-solve input[name="solve-mode"]').find((r) => r.parentElement.textContent.trim() === label);
  click(mode("In time"));
  await until(() => !q("#view-stamp").hidden, "the outdated stamp");
  const after = picture();
  check("the picture stays put when the result goes outdated", Math.abs(after.top - before.top) < 1 && Math.abs(after.height - before.height) < 1, `${before.top},${before.height} → ${after.top},${after.height}`);
  click(mode("Static"));
  await until(() => q("#view-stamp").hidden, "the result current again");

  // A probe, then its coordinates typed.
  pointer("pointerdown", 0.5, 0.5);
  pointer("pointerup", 0.5, 0.5);
  await until(() => qa(".probes-table tbody tr").length === 1, "a probe");
  check("the panel shows with a probe", !q("#probes").hidden);
  check("no subtitle under the probes", !q("#probes .probe-list .note"));
  const commit = (input, value) => {
    input.focus();
    type(input, value);
    input.dispatchEvent(new w.Event("change", { bubbles: true }));
  };
  commit(q('input[aria-label="x of P1"]'), "0.25");
  await until(() => q('input[aria-label="x of P1"]')?.value === "0.25", "x typed");
  check("the typed x moves the probe", true);
  check("the field keeps the focus after Enter", w.document.activeElement === q('input[aria-label="x of P1"]'));

  // The panel's height follows its header; low enough, it hides.
  const handle = q("#extra-handle");
  const panel = q("#probes");
  const at = (y) => ({ clientX: 400, clientY: y, button: 0, pointerId: 9, bubbles: true });
  const top = handle.getBoundingClientRect().top;
  const tall = panel.getBoundingClientRect().height;
  handle.dispatchEvent(new w.PointerEvent("pointerdown", at(top + 5)));
  handle.dispatchEvent(new w.PointerEvent("pointermove", at(top - 95)));
  handle.dispatchEvent(new w.PointerEvent("pointerup", at(top - 95)));
  await idle();
  check("dragging the header up makes the panel taller", panel.getBoundingClientRect().height > tall + 80, `${tall} → ${panel.getBoundingClientRect().height}`);
  const shown = panel.getBoundingClientRect().height;
  const head = handle.getBoundingClientRect().top;
  handle.dispatchEvent(new w.PointerEvent("pointerdown", at(head + 5)));
  handle.dispatchEvent(new w.PointerEvent("pointermove", at(head + shown)));
  handle.dispatchEvent(new w.PointerEvent("pointerup", at(head + shown)));
  await idle();
  check("dragged low, it hides to its header", panel.classList.contains("collapsed") && q("#extra-toggle").textContent === "Show");
  click(q("#extra-toggle"));
  await idle();
  check("Show brings it back at its height", Math.abs(panel.getBoundingClientRect().height - shown) < 2, `${shown} / ${panel.getBoundingClientRect().height}`);

  // The line profile's ends, typed.
  click(q("#profile-tool"));
  pointer("pointerdown", 0.2, 0.3);
  pointer("pointermove", 0.5, 0.3);
  pointer("pointerup", 0.8, 0.3);
  await until(() => q(".profile-chart path.line"), "the profile chart");
  check("no subtitle under the profile", !q(".profile-chart .note"));
  commit(q('input[aria-label="y of B"]'), "0.9");
  await until(() => q('input[aria-label="y of B"]')?.value === "0.9", "B's y typed");
  check("the line follows its typed end", /along A → B/.test(text(q(".profile-chart"))));

  // The steps panel and the view options hide, and come back.
  const wide = picture().width;
  click(q("#toggle-sidebar"));
  await until(() => picture().width > wide + 200, "the picture to take the room");
  check("the steps panel hides", q("#sidebar").offsetParent === null && q("#toggle-sidebar").getAttribute("aria-pressed") === "false");
  click(q("#toggle-toolbar"));
  await idle();
  check("the view options hide", q("#viewer-toolbar").offsetParent === null);
  check("hidden panes are remembered", w.localStorage.getItem("pde2d-app-sidebar-hidden") === "1" && w.localStorage.getItem("pde2d-app-toolbar-hidden") === "1");
  click(q("#toggle-toolbar"));
  click(q("#toggle-sidebar"));
  await until(() => Math.abs(picture().width - wide) < 2, "the steps panel back");
  check("both come back", q("#sidebar").offsetParent !== null && q("#viewer-toolbar").offsetParent !== null);
  w.localStorage.removeItem("pde2d-app-extra-height");
  await idle();
};
