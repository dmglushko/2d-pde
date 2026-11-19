// The geometry editor: a new drawing, a polygon hole drawn with the tool,
// a vertex dragged (one undo step), undo, a region split by a click on
// the boundary (a drawing in the page, a shape through the server's
// locator) with its names edited right there, and the conditions that
// follow.
export default async ({ d, w, q, qa, tab, click, until, text, check, idle, key, type, reloadPage }) => {
  tab("geometry");
  const stats = () => text(q("#geometry-summary .stats"));
  // A figure of the Geometry step's result by its label.
  const stat = (label) => text([...qa("#geometry-summary .stat")].find((e) => text(e.querySelector(".label")) === label)?.querySelector(".value"));
  const regions = () => qa("#geometry-regions tr[data-region]").map((r) => r.dataset.region);
  const canvas = q("#view");
  // The picture fits the 2 x 1 rectangle at 0.88 of its size; a world
  // point's place on the picture.
  const at = (x, y, box = { min: [-1, -0.5], max: [1, 0.5] }) => {
    const r = canvas.getBoundingClientRect();
    const scale = 0.88 * Math.min(r.width / (box.max[0] - box.min[0]), r.height / (box.max[1] - box.min[1]));
    const ox = r.width / 2 - ((box.min[0] + box.max[0]) / 2) * scale;
    const oy = r.height / 2 + ((box.min[1] + box.max[1]) / 2) * scale;
    return { clientX: r.left + ox + x * scale, clientY: r.top + oy - y * scale };
  };
  const pointer = (kind, x, y, extra = {}) =>
    canvas.dispatchEvent(new w.PointerEvent(kind, { ...at(x, y), button: 0, buttons: kind === "pointerup" ? 0 : 1, pointerId: 7, pointerType: "mouse", isPrimary: true, bubbles: true, ...extra }));
  const clickAt = (x, y) => {
    pointer("pointermove", x, y, { buttons: 0 });
    pointer("pointerdown", x, y);
    pointer("pointerup", x, y);
  };
  const tool = (name) => click(q(`.geometry-tools button[data-tool="${name}"]`));
  const savedCase = () => {
    w.dispatchEvent(new w.Event("pagehide"));
    return JSON.parse(w.localStorage.getItem("pde2d-app-case"));
  };

  // A new drawing: a blank rectangle with the shape's four sides.
  const drawing = [...qa('input[name="geometry-source"]')].find((input) => text(input.parentElement) === "Drawing");
  drawing.click();
  await until(() => savedCase().geometry.source === "drawing", "the drawing source");
  await idle();
  await until(() => stats().includes("Loops"), "the drawing read");
  check("a new drawing is a rectangle with four regions", regions().join(",") === "bottom,right,top,left", regions().join(","));
  check("the drawing tools are on the picture", !q(".geometry-tools").hidden && !q('.geometry-tools [data-tool="polygon"]').hidden);

  // A polygon hole: four clicks inside, Enter closes it.
  tool("polygon");
  for (const [x, y] of [
    [-0.4, -0.2],
    [0.2, -0.2],
    [0.2, 0.2],
    [-0.4, 0.2],
  ])
    clickAt(x, y);
  key(canvas, "Enter");
  await until(() => savedCase().geometry.loops.length === 2, "the polygon added");
  await idle();
  await until(() => stat("Holes") === "1", "the hole read");
  check("the polygon is a hole of the drawing", stat("Loops") === "2" && stat("Holes") === "1", stats());
  check("its region is named hole", regions().includes("hole"), regions().join(","));
  check("the tool is Select again", q('.geometry-tools [data-tool="select"]').getAttribute("aria-pressed") === "true");

  // Drag the outer rectangle's top right vertex: one undo step.
  key(canvas, "Escape");
  pointer("pointerdown", 1, 0.5);
  await until(() => q("#drawing-selection .selection-card"), "the vertex selected");
  check("the vertex's fields show it", text(q("#drawing-selection .selection-title")).startsWith("Vertex 2"), text(q("#drawing-selection")));
  for (const s of [0.25, 0.5, 0.75, 1]) pointer("pointermove", 1 + 0.25 * s, 0.5 + 0.2 * s);
  pointer("pointerup", 1.25, 0.7);
  await until(() => savedCase().geometry.loops[0].vertices[2][0] !== 1, "the vertex moved");
  const moved = savedCase().geometry.loops[0].vertices[2];
  check("the vertex is where it was dropped", Math.abs(moved[0] - 1.25) < 1e-9 && Math.abs(moved[1] - 0.7) < 1e-9, JSON.stringify(moved));
  check("its x field follows", Math.abs(Number(q('#drawing-selection input[aria-label="x"]').value) - 1.25) < 1e-9, q('#drawing-selection input[aria-label="x"]').value);
  await idle();
  // Typed coordinates move it too.
  const y = q('#drawing-selection input[aria-label="y"]');
  type(y, "0.6");
  y.dispatchEvent(new w.Event("change", { bubbles: true }));
  await until(() => savedCase().geometry.loops[0].vertices[2][1] === 0.6, "the typed y");
  check("a typed coordinate moves the vertex", true);
  // Undo: the typed y, then the whole drag in one step.
  key(d.body, "z", { ctrlKey: true });
  await until(() => savedCase().geometry.loops[0].vertices[2][1] !== 0.6, "the typed y undone");
  key(d.body, "z", { ctrlKey: true });
  await until(() => savedCase().geometry.loops[0].vertices[2][0] === 1, "the drag undone");
  check("one undo takes the whole drag back", savedCase().geometry.loops[0].vertices[2][1] === 0.5);
  check("the hole stays", savedCase().geometry.loops.length === 2);
  await idle();

  // Escape in the middle of a drag: the drawing and its fields go back.
  const xField = () => q('#drawing-selection input[aria-label="x"]');
  pointer("pointerdown", 1, 0.5);
  await until(() => xField(), "the vertex selected again");
  for (const s of [0.5, 1]) pointer("pointermove", 1 + 0.3 * s, 0.5);
  check("the field follows the drag", Math.abs(Number(xField().value) - 1.3) < 1e-9, xField().value);
  key(canvas, "Escape");
  check("Escape puts the field back", xField().value === "1", xField().value);
  pointer("pointermove", 1.4, 0.5);
  pointer("pointerup", 1.4, 0.5);
  check("and the drawing", savedCase().geometry.loops[0].vertices[2].join(",") === "1,0.5", JSON.stringify(savedCase().geometry.loops[0].vertices[2]));
  // An undo in the middle of a drag (here of the hole) ends the drag:
  // releasing it does not bring the undone drawing back.
  pointer("pointerdown", 1, 0.5);
  for (const s of [0.5, 1]) pointer("pointermove", 1 + 0.3 * s, 0.5);
  key(d.body, "z", { ctrlKey: true });
  await until(() => savedCase().geometry.loops.length === 1, "the hole undone");
  pointer("pointermove", 1.4, 0.5);
  pointer("pointerup", 1.4, 0.5);
  await idle();
  const after = savedCase().geometry;
  check("a release after an undo changes nothing", after.loops.length === 1 && after.loops[0].vertices[2].join(",") === "1,0.5", JSON.stringify(after.loops));
  check("the field shows the drawing", xField() === null || xField().value === "1", xField()?.value);
  key(d.body, "z", { ctrlKey: true, shiftKey: true });
  await until(() => savedCase().geometry.loops.length === 2, "the hole redone");
  await idle();

  // Split the top by a click on it: the drawing gets a vertex there and
  // the names are offered right away.
  tool("split");
  clickAt(0.3, 0.5);
  await until(() => q(".region-pop"), "the names of the split");
  const fields = qa(".region-pop input");
  check("the split's names are offered", fields.length === 2 && fields[0].value === "top" && fields[1].value === "top_2", fields.map((f) => f.value).join(","));
  check("the second name is ready for typing", d.activeElement === fields[1]);
  fields[1].value = "inlet";
  key(fields[1], "Enter");
  await until(() => savedCase().geometry.loops[0].edges.some((e) => e.region === "inlet"), "the renamed part");
  const top = savedCase().geometry.loops[0];
  check("a vertex was added on the top", top.vertices.length === 5 && Math.abs(top.vertices[3][0] - 0.3) < 1e-9 && top.vertices[3][1] === 0.5, JSON.stringify(top.vertices));
  key(fields[1], "Escape");
  await idle();
  await until(() => regions().includes("inlet"), "the regions read");
  check("the top is two regions", regions().includes("top") && regions().includes("inlet"), regions().join(","));
  const conditions = savedCase().problem.boundary_conditions;
  check("the new part has the top's condition", JSON.stringify(conditions.inlet) === JSON.stringify(conditions.top), JSON.stringify(conditions));

  // Without a pointer: ] and [ select vertices and edges, L the loops;
  // the live region says what is selected; the selected edge splits at a
  // typed fraction.
  const status = () => text(q("#editor-status"));
  key(canvas, "Escape");
  key(canvas, "Escape");
  key(canvas, "]");
  check("] selects the first vertex", status() === "Loop 0, vertex 0: x -1, y -0.5", status());
  check("its fields are shown", xField()?.value === "-1", xField()?.value);
  key(canvas, "]");
  check("] then its edge", status() === "Loop 0, edge 0: line, region bottom", status());
  key(canvas, "[");
  key(canvas, "]");
  const fraction = q('#drawing-selection .split-row input[aria-label^="Split at"]');
  type(fraction, "0.25");
  [...qa("#drawing-selection button")].find((b) => b.textContent === "Split").click();
  await until(() => savedCase().geometry.loops[0].edges.some((e) => e.region === "bottom_2"), "the typed split");
  const typed = savedCase();
  check("the typed split adds a vertex at a quarter", typed.geometry.loops[0].vertices[1].join(",") === "-0.5,-0.5", JSON.stringify(typed.geometry.loops[0].vertices));
  check("the part after it is the new region, with the condition", typed.geometry.loops[0].edges[1].region === "bottom_2" && JSON.stringify(typed.problem.boundary_conditions.bottom_2) === JSON.stringify(typed.problem.boundary_conditions.bottom));
  check("the new part is selected and said", status() === "Loop 0, edge 1: line, region bottom_2", status());
  key(canvas, "l");
  check("L selects the next loop", status().startsWith("Loop 1, vertex 0"), status());
  await idle();

  // A corner splits exactly there: the right side and the bottom merged
  // in a shape, split at their corner through the server.
  click(q("#new-case"));
  await until(() => text(q("#case-name")) === "" && q("#undo").disabled, "a new case");
  await idle();
  tab("geometry");
  await until(() => regions().length === 4, "the rectangle");
  q('#geometry-regions tr[data-region="right"] button').click();
  const merge = q("#geometry-regions select");
  merge.value = "bottom";
  [...qa("#geometry-regions button")].find((b) => b.textContent === "Apply" && b.closest(".edit-line")?.textContent.includes("Merge")).click();
  await idle();
  await until(() => regions().length === 3, "the merge read");
  tool("split");
  clickAt(0.995, -0.497);
  await until(() => q(".region-pop"), "the split at the corner");
  const edits = savedCase().geometry.region_edits;
  const split = edits[edits.length - 1];
  check("the split is a region edit", split.op === "split" && split.region === "bottom" && split.second === "bottom_2", JSON.stringify(split));
  check("at the corner exactly: two thirds of bottom and right", Math.abs(split.fraction - 2 / 3) < 1e-15, String(split.fraction));
  key(q(".region-pop input"), "Escape");
  await idle();
  await until(() => regions().includes("bottom_2"), "the split read");
  const view = await (await w.fetch("/api/geometry", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(savedCase()) })).json();
  check("a corner split keeps the four lines", view.loops[0].curves.length === 4 && view.loops[0].curves.every((c) => c.kind === "line"), JSON.stringify(view.loops[0].curves.map((c) => c.region)));

  // A condition set after the split survives a rename of the part.
  tab("problem");
  const g = () => q('[data-path="problem.boundary_conditions.bottom_2.text"] input');
  await until(() => g(), "the new part's condition");
  type(g(), "u = 5");
  g().dispatchEvent(new w.Event("change", { bubbles: true }));
  await until(() => savedCase().problem.boundary_conditions.bottom_2.text === "u = 5", "u = 5");
  tab("geometry");
  q('#geometry-regions tr[data-region="bottom_2"] button').click();
  const renameTo = q('#geometry-regions input[aria-label="New name"]');
  renameTo.value = "outlet";
  [...qa("#geometry-regions button")].find((b) => b.textContent === "Apply" && b.closest(".edit-line")?.textContent.includes("Rename")).click();
  await until(() => savedCase().problem.boundary_conditions.outlet, "the renamed part");
  const renamed = savedCase();
  check("the renamed part keeps its edited condition", renamed.problem.boundary_conditions.outlet.text === "u = 5", JSON.stringify(renamed.problem.boundary_conditions));
  check("the split took the name", renamed.geometry.region_edits.length === 2 && renamed.geometry.region_edits[1].second === "outlet", JSON.stringify(renamed.geometry.region_edits));
  await idle();
  await until(() => regions().includes("outlet"), "the rename read");

  // Edit as drawing: the shape's curves with their edits become a drawing.
  [...qa("#panel-geometry button")].find((b) => b.textContent === "Edit as drawing").click();
  await until(() => savedCase().geometry.source === "drawing", "the drawing");
  const converted = savedCase().geometry;
  check("the edited shape is a drawing with its regions", converted.loops[0].edges.map((e) => e.region).join(",") === "bottom,outlet,top,left", converted.loops[0].edges.map((e) => e.region).join(","));
  await idle();
  check("its regions read the same", regions().join(",") === "bottom,outlet,top,left", regions().join(","));

  // A reload finds the drawing and its tools again.
  savedCase();
  const page = await reloadPage();
  await until(() => page.q("#geometry-summary .stats"), "the page again");
  page.q('.steps button[data-step="geometry"]').click();
  await until(() => page.qa("#geometry-regions tr[data-region]").length === 4, "the drawing's regions after the reload");
  check("after a reload the drawing is there", page.q('input[name="geometry-source"]:checked')?.parentElement.textContent.trim() === "Drawing");
  check("and its tools", !page.q(".geometry-tools").hidden && !page.q('.geometry-tools [data-tool="rectangle"]').hidden);
};
