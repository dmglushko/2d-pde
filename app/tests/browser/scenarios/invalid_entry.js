// Text that is not a number blocks the runs, visibly: the step head says
// which field, Mesh and Run all are disabled with the reason, the field
// says the case keeps its previous value; a decimal comma is named.
export default async ({ q, tab, panel, button, click, type, until, text, check }) => {
  tab("mesh");
  const angle = q('input[data-path="mesh.refiner.options.min_angle_degrees"]');
  check("the field is a text field for decimals", angle.type === "text" && angle.inputMode === "decimal", `${angle.type} ${angle.inputMode}`);
  const before = angle.value;
  type(angle, "abc");
  const notice = await until(() => (text(q("#panel-mesh .invalid-notice")) ? q("#panel-mesh .invalid-notice") : null), "the notice in the head");
  check("the head names the field", /1 field is not a number: .*angle/i.test(text(notice)), text(notice));
  check("Mesh is blocked", button("Mesh", panel("mesh")).disabled && /Blocked: /.test(button("Mesh", panel("mesh")).title));
  check("Run all is blocked", q("#run-all").disabled && /not a number/.test(q("#run-all").title), q("#run-all").title);
  const message = text(angle.closest(".field").querySelector("[data-local-error]"));
  check("the field says the case keeps its value", message.includes(`The case still has ${before}`), message);
  tab("solve");
  check("the Solve step says so too", /not a number/.test(text(q("#panel-solve .invalid-notice"))));
  check("Solve is blocked", button("Solve", panel("solve")).disabled);
  click(q("#panel-solve .invalid-notice button"));
  await until(() => q("#panel-mesh") && !q("#panel-mesh").hidden && q(".panels").ownerDocument.activeElement === angle, "the link to the field");
  check("the link opens the step and focuses the field", true);
  type(angle, "0,5");
  await until(() => /decimal point/.test(text(angle.closest(".field").querySelector("[data-local-error]"))), "the comma message");
  check("a decimal comma is named", true);
  type(angle, "26.5");
  await until(() => !text(q("#panel-mesh .invalid-notice")) && !button("Mesh", panel("mesh")).disabled, "the block to clear");
  check("a number clears the block", !q("#run-all").disabled);
};
