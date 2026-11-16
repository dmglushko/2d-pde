// At phone width: no horizontal scroll, the tabs fit and come first, the
// picture follows, then the panel.
export default async ({ d, w, q, qa, click, select, until, sleep, text, check }) => {
  select(q("#examples"), "heat_annulus.json");
  await until(() => text(q("#toast")).includes("heat_annulus"), "the heat case");
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
  await sleep(100);
  const tabsBottom = q(".steps").getBoundingClientRect().bottom;
  const picture = q(".viewer-main").getBoundingClientRect();
  check("the tabs stay at the top", Math.abs(q(".steps").getBoundingClientRect().top) < 1, String(q(".steps").getBoundingClientRect().top));
  check("the picture stays under them", Math.abs(picture.top - tabsBottom) < 2 && picture.bottom < w.innerHeight * 0.6, `${picture.top} ${tabsBottom} ${picture.bottom} of ${w.innerHeight}`);
};
