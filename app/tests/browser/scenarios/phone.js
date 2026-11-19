// At phone width: no horizontal scroll, the tabs fit and come first, the
// picture follows, then the panel.
export default async ({ example, d, w, q, qa, click, select, until, nextFrame, text, check, pointer }) => {
  example("heat_annulus.json");
  await until(() => text(q("#case-name")).includes("heat_annulus"), "the heat case");
  await until(() => q("#geometry-summary .stats"), "its geometry");
  click(q("#run-all"));
  await until(() => text(q("#toast")).includes("up to date"), "Run all", 90000);
  check("the time bar is under the picture", !q("#timebar").hidden && q("#timebar").getBoundingClientRect().top >= q(".canvas-wrap").getBoundingClientRect().bottom - 1);
  const width = w.innerWidth;
  check("no horizontal page scroll", d.documentElement.scrollWidth <= width, `${d.documentElement.scrollWidth} > ${width}`);
  const wide = qa("body *").filter((e) => e.getBoundingClientRect().right > width + 1 && !e.closest(".viewer-toolbar"));
  check("nothing sticks out on the right", wide.length === 0, wide.slice(0, 4).map((e) => `${e.tagName}.${e.className}`).join(", "));
  const tabs = qa(".steps button");
  check("every tab fits", tabs.every((b) => b.getBoundingClientRect().right <= width + 0.5 && b.scrollWidth <= b.clientWidth + 1), tabs.map((b) => Math.round(b.getBoundingClientRect().right)).join(","));
  const top = (s) => q(s).getBoundingClientRect().top;
  check("tabs, then the picture, then the panel", top(".steps") < top(".canvas-wrap") && top(".canvas-wrap") < top(".panels"), `${top(".steps")} ${top(".canvas-wrap")} ${top(".panels")}`);
  // Scrolled to the panel's end, the tabs and the picture stay in view.
  w.scrollTo(0, d.documentElement.scrollHeight);
  await until(() => w.scrollY > 0, "the scroll");
  await nextFrame();
  const tabsBottom = q(".steps").getBoundingClientRect().bottom;
  const picture = q(".viewer-main").getBoundingClientRect();
  check("the tabs stay at the top", Math.abs(q(".steps").getBoundingClientRect().top) < 1, String(q(".steps").getBoundingClientRect().top));
  check("the picture stays under them", Math.abs(picture.top - tabsBottom) < 2 && picture.bottom < w.innerHeight * 0.6, `${picture.top} ${tabsBottom} ${picture.bottom} of ${w.innerHeight}`);

  // The picture's height follows its handle and is remembered.
  const handle = q("#picture-handle");
  check("the height handle is shown", handle.getBoundingClientRect().height > 0);
  const before = q("#canvas-wrap").getBoundingClientRect().height;
  const r = handle.getBoundingClientRect();
  const at = (y) => ({ clientX: r.left + r.width / 2, clientY: y, pointerId: 7, bubbles: true });
  handle.dispatchEvent(new w.PointerEvent("pointerdown", at(r.top + 5)));
  handle.dispatchEvent(new w.PointerEvent("pointermove", at(r.top + 65)));
  handle.dispatchEvent(new w.PointerEvent("pointerup", at(r.top + 65)));
  const after = await until(() => { const h = q("#canvas-wrap").getBoundingClientRect().height; return Math.abs(h - before - 60) < 2 ? h : null; }, "the taller picture");
  check("dragging the handle makes the picture taller", Math.abs(after - before - 60) < 2, `${before} -> ${after}`);
  check("the height is remembered", Number(w.localStorage.getItem("pde2d-app-picture-height")) === Math.round(after));
  // Touch targets: the zoom buttons and the info buttons.
  const small = [...d.querySelectorAll(".zoom-buttons button, button.info")].filter((b) => b.offsetParent && (b.getBoundingClientRect().width < 24 || b.getBoundingClientRect().height < 24));
  check("small controls are at least 24 px", small.length === 0, small.map((b) => b.getAttribute("aria-label")).join(", "));
  // Two fingers pinch: the picture zooms.
  w.scrollTo(0, 0);
  const view = () => q("#view").toDataURL();
  const start = view();
  pointer("pointerdown", 0.4, 0.5, 11);
  pointer("pointerdown", 0.6, 0.5, 12);
  pointer("pointermove", 0.3, 0.5, 11);
  pointer("pointermove", 0.7, 0.5, 12);
  pointer("pointerup", 0.3, 0.5, 11);
  pointer("pointerup", 0.7, 0.5, 12);
  await until(() => view() !== start, "the pinch to zoom");
  check("a pinch zooms the picture", true);
  check("a pinch pins no probe", d.querySelectorAll(".probes-table tbody tr").length === 0);
  // The top bar is one row: the case's actions are in its menu.
  const bar = q(".topbar").getBoundingClientRect();
  check("the top bar is one row", bar.height < 64, String(bar.height));
  const middles = qa(".topbar .brand, .case-actions > *").filter((e) => e.offsetParent).map((e) => { const r = e.getBoundingClientRect(); return r.top + r.height / 2; });
  check("its controls sit on one line", Math.max(...middles) - Math.min(...middles) < 6, middles.map(Math.round).join(","));
  // The picture can be folded away and back.
  const collapse = q("#collapse-picture");
  check("the collapse control is shown", collapse.getBoundingClientRect().width > 0);
  click(collapse);
  check("the picture folds away", q("#canvas-wrap").getBoundingClientRect().height === 0 && collapse.getAttribute("aria-expanded") === "false");
  click(collapse);
  check("and comes back", q("#canvas-wrap").getBoundingClientRect().height > 100 && collapse.getAttribute("aria-expanded") === "true");
};
