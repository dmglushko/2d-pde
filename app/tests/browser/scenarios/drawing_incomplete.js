// A drawing that is incomplete (a Bézier without its control, a loop
// without vertices) is read back after a reload as it is: the page draws
// what is complete, never throws, and shows the server's error at its
// path.
export default async ({ w, idle, until, check, text, reloadPage }) => {
  const geometry = {
    source: "drawing",
    loops: [
      {
        vertices: [
          [0, 0],
          [2, 0],
          [2, 1],
          [0, 1],
        ],
        edges: [
          { kind: "line", region: "a" },
          { kind: "quadratic", region: "b" },
          { kind: "line", region: "c" },
          { kind: "line", region: "d" },
        ],
      },
      { vertices: [], edges: [{ kind: "line", region: "e" }] },
    ],
  };
  await idle();
  w.localStorage.setItem("pde2d-app-case", JSON.stringify({ format: "pde2d-app-case", version: 1, geometry }));
  w.localStorage.setItem("pde2d-app-step", "geometry");
  const page = await reloadPage();
  const pw = page.w;
  await until(() => page.q("#panel-geometry [data-error]"), "the server's error");
  const error = text(page.q("#panel-geometry [data-error]"));
  check("the error names what is missing", error.includes("expected a point"), error);
  check("the error is at the drawing", Boolean(page.q('#panel-geometry .field.invalid[data-path="geometry.loops"]')));
  // The picture and the keys work on what is complete.
  const canvas = page.q("#view");
  const r = canvas.getBoundingClientRect();
  for (const [fx, fy] of [
    [0.5, 0.5],
    [0.3, 0.7],
  ])
    canvas.dispatchEvent(new pw.PointerEvent("pointermove", { clientX: r.left + r.width * fx, clientY: r.top + r.height * fy, pointerId: 3, pointerType: "mouse", bubbles: true }));
  const key = (k, extra = {}) => canvas.dispatchEvent(new pw.KeyboardEvent("keydown", { key: k, bubbles: true, cancelable: true, ...extra }));
  key("]");
  check("a complete vertex can be selected", text(page.q("#editor-status")).startsWith("Loop 0, vertex 0"), text(page.q("#editor-status")));
  // Loop 1 has no vertex: L stays on loop 0.
  key("l");
  check("a loop without vertices is passed over", text(page.q("#editor-status")).startsWith("Loop 0, vertex 0"), text(page.q("#editor-status")));
  key("]");
  key("]");
  key("]");
  check("an incomplete edge can be selected", text(page.q("#editor-status")) === "Loop 0, edge 1: quadratic, region b", text(page.q("#editor-status")));
  key("Escape");
  await new Promise((resolve) => pw.setTimeout(resolve, 200));
  pw.dispatchEvent(new pw.Event("pagehide"));
  const kept = JSON.parse(pw.localStorage.getItem("pde2d-app-case"));
  check("the case keeps the drawing as it was", JSON.stringify(kept.geometry) === JSON.stringify(geometry), JSON.stringify(kept.geometry));
};
