// Step 5: solving. The library has no solvers yet; the catalog says so,
// and this panel shows what is planned. When the server's catalog lists
// a "solve" section with available: true, its slots are built here like
// the mesh panel's (docs/app.md, "Adding the solvers").

import { h, fill } from "../dom.js";
import { store } from "../state.js";

export const solvePanel = {
  build(container) {
    const section = store.catalog.sections.find((s) => s.id === "solve");
    fill(container,
      h("h2", {}, "Solve"),
      section?.available
        ? h("p", {}, "Solvers are available.")
        : [
            h("div", { class: "notice" }, section?.message ?? "Not available yet."),
            h("h3", {}, "Planned"),
            h(
              "ul",
              {},
              h("li", {}, "Linear solvers: sparse LU (direct) and BiCGSTAB (iterative)."),
              h("li", {}, "Time methods: implicit Euler and the implicit midpoint rule for first order in time; Newmark for second order."),
              h("li", {}, "Step control: a fixed step dt first."),
              h("li", {}, "Results: the solution at chosen times, drawn on the mesh like the fields of the Assemble step."),
            ),
            h("p", { class: "note" }, "Until then, the Assemble step shows the system that a solver will receive."),
          ],
    );
  },
  update() {},
};
